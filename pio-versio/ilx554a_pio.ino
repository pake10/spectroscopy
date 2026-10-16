#include <Arduino.h>
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/regs/addressmap.h"

// ============================================================================
// Pre-assembled PIO Program (from ilx554a.pio)
// Side-set pin 0 = GP2 (phi_CLK), SET pin 0 = GP3 (phi_ROG)
// ============================================================================
#define ilx554a_wrap_target 0
#define ilx554a_wrap        12

static const uint16_t ilx554a_program_instructions[] = {
    //     .wrap_target
    0x90a0, //  0: pull   block           side 1
    0xb027, //  1: mov    x, osr          side 1
    // flush_loop:
    0xa342, //  2: nop                    side 0 [3]
    0x1342, //  3: jmp    x--, 2          side 1 [3]
    0xb342, //  4: nop                    side 1 [3]
    0xf700, //  5: set    pins, 0         side 1 [7]
    0xf301, //  6: set    pins, 1         side 1 [3]
    0x90a0, //  7: pull   block           side 1
    0xb047, //  8: mov    y, osr          side 1
    0xf024, //  9: set    x, 4            side 1
    // pixel_loop:
    0xa142, // 10: nop                    side 0 [1]
    0x4120, // 11: in     x, 32           side 0 [1]
    0x138a, // 12: jmp    y--, 10         side 1 [3]
    //     .wrap
};

static const struct pio_program ilx554a_program = {
    .instructions = ilx554a_program_instructions,
    .length = 13,
    .origin = -1,
};

static inline pio_sm_config ilx554a_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + ilx554a_wrap_target, offset + ilx554a_wrap);
    sm_config_set_sideset(&c, 1, false, false);
    return c;
}

static inline void ilx554a_program_init(PIO pio, uint sm, uint offset, uint clk_pin, uint rog_pin) {
    pio_sm_config c = ilx554a_program_get_default_config(offset);

    // Map side-set to phi_CLK and SET to phi_ROG
    sm_config_set_sideset_pins(&c, clk_pin);
    sm_config_set_set_pins(&c, rog_pin, 1);

    pio_gpio_init(pio, clk_pin);
    pio_gpio_init(pio, rog_pin);

    // Initialize both pins HIGH (idle state)
    uint32_t pin_mask = (1u << clk_pin) | (1u << rog_pin);
    pio_sm_set_pins_with_mask(pio, sm, pin_mask, pin_mask);
    pio_sm_set_pindirs_with_mask(pio, sm, pin_mask, pin_mask);

    // Autopush 32-bit trigger word (0x04) into RX FIFO on `in x, 32`
    sm_config_set_in_shift(&c, false, true, 32);

    // Run PIO at 2.0 MHz (0.5 us per tick -> 250 kHz pixel clock)
    float div = (float)clock_get_hz(clk_sys) / 1000000.0f; // tässä alun perine 2000000.0f, muutettu 1000000.0f
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

// ============================================================================
// Sensor & DMA Configuration
// ============================================================================
constexpr uint PIN_CLK      = 16;    // GP16 -> phi_CLK (via 5V 74HCT buffer)
constexpr uint PIN_ROG      = 17;    // GP17 -> phi_ROG (via 5V 74HCT buffer)
constexpr uint PIN_ADC      = 26;   // GP26 / ADC0 -> Buffered CCD VOUT

constexpr uint NUM_PIXELS   = 2090;
// Extra 4.0 us flush cycles between frames to set integration time
// Total T_int = (2090 + FLUSH_CYCLES) * 4.0 us + 8.0 us (~10 ms for 410)
constexpr uint32_t FLUSH_CYCLES = 410;

static uint16_t frame_buf[NUM_PIXELS];
static const uint8_t sync_header[4] = {0xAA, 0x55, 0xAA, 0x55};

static PIO pio_inst = pio0;
static uint pio_sm = 0;
static int dma_trig_chan = -1;
static int dma_cap_chan = -1;

void setup() {
    // Initialize native USB CDC Serial
    Serial.begin(115200);

    // 1. Configure ADC0 (GP26) for DMA-driven single conversions
    adc_init();
    adc_gpio_init(PIN_ADC);
    adc_select_input(0);
    adc_fifo_setup(
        true,   // Write each conversion to the ADC FIFO
        true,   // Enable DMA data request (DREQ_ADC)
        1,      // Assert DREQ when 1 sample is present
        false,  // Disable error bit
        false   // Full 12-bit resolution (16-bit words)
    );
    adc_set_clkdiv(0); // Fastest conversion (96 cycles = 2.0 us)

    // 2. Load and start PIO program on PIO0
    // Claim an unused state machine so we don't collide with Arduino libraries
    pio_sm = pio_claim_unused_sm(pio_inst, true);
    uint offset = pio_add_program(pio_inst, &ilx554a_program);
    ilx554a_program_init(pio_inst, pio_sm, offset, PIN_CLK, PIN_ROG);

    // 3. Configure DMA Channel A: PIO RX FIFO -> ADC_CS (atomic bit-set alias)
    dma_trig_chan = dma_claim_unused_channel(true);
    dma_channel_config trig_cfg = dma_channel_get_default_config(dma_trig_chan);
    channel_config_set_transfer_data_size(&trig_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&trig_cfg, false);
    channel_config_set_write_increment(&trig_cfg, false);
    channel_config_set_dreq(&trig_cfg, pio_get_dreq(pio_inst, pio_sm, false));

    dma_channel_configure(
        dma_trig_chan,
        &trig_cfg,
        hw_set_alias_untyped(&adc_hw->cs), // Write 0x04 to set START_ONCE bit
        &pio_inst->rxf[pio_sm],            // Source: PIO RX FIFO
        NUM_PIXELS,
        false
    );

    // 4. Configure DMA Channel B: ADC FIFO -> frame_buf
    dma_cap_chan = dma_claim_unused_channel(true);
    dma_channel_config cap_cfg = dma_channel_get_default_config(dma_cap_chan);
    channel_config_set_transfer_data_size(&cap_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cap_cfg, false);
    channel_config_set_write_increment(&cap_cfg, true);
    channel_config_set_dreq(&cap_cfg, DREQ_ADC);

    dma_channel_configure(
        dma_cap_chan,
        &cap_cfg,
        frame_buf,       // Destination buffer in RAM
        &adc_hw->fifo,   // Source: ADC FIFO
        NUM_PIXELS,
        false
    );
}

void loop() {
    // 1. Clear residual FIFO entries before arming a new frame
    adc_fifo_drain();
    pio_sm_clear_fifos(pio_inst, pio_sm);

    // 2. Re-arm both DMA channels for 2090 pixels
    dma_channel_set_trans_count(dma_trig_chan, NUM_PIXELS, true);
    dma_channel_set_write_addr(dma_cap_chan, frame_buf, true);

    // 3. Push integration flush count and pixel count to start the frame
    pio_sm_put_blocking(pio_inst, pio_sm, FLUSH_CYCLES);
    pio_sm_put_blocking(pio_inst, pio_sm, NUM_PIXELS - 1);

    // 4. Block until all 2090 ADC samples are captured in RAM
    dma_channel_wait_for_finish_blocking(dma_cap_chan);

    // 5. Stream raw binary frame over USB CDC if the Python script is connected
    if (Serial) {
        Serial.write(sync_header, sizeof(sync_header));
        Serial.write(reinterpret_cast<const uint8_t*>(frame_buf), sizeof(frame_buf));
        Serial.flush();
    }
}