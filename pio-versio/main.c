#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/regs/addressmap.h"
#include "ilx554a.pio.h"

#define PIN_CLK         16
#define PIN_ROG         17
#define PIN_ADC         26   // ADC0

#define NUM_PIXELS      2090
// Extra 4.0 us flush cycles between frames to adjust integration time
// Total T_int = (2090 + FLUSH_CYCLES) * 4.0 us + 8.0 us
// Example: 410 flush cycles -> ~10.0 ms total integration time
#define FLUSH_CYCLES    410

static uint16_t frame_buf[NUM_PIXELS];
static const uint8_t sync_header[4] = {0xAA, 0x55, 0xAA, 0x55};

int main() {
    stdio_init_all();
    // Crucial: Disable CRLF translation so 0x0A bytes in binary ADC data aren't corrupted
    stdio_set_translate_crlf(&stdio_usb, false);

    // 1. Initialize ADC on GP26 (ADC0)
    adc_init();
    adc_gpio_init(PIN_ADC);
    adc_select_input(0);
    adc_fifo_setup(
        true,   // Write each completed conversion to the ADC FIFO
        true,   // Enable DMA data request (DREQ_ADC)
        1,      // Assert DREQ when at least 1 sample is present
        false,  // Disable error bit
        false   // Keep full 12-bit resolution (16-bit words)
    );
    adc_set_clkdiv(0); // Fastest single-shot conversion (96 cycles = 2.0 us)

    // 2. Initialize PIO0 State Machine 0
    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &ilx554a_program);
    ilx554a_program_init(pio, sm, offset, PIN_CLK, PIN_ROG);

    // 3. Configure DMA Channel 0: PIO RX FIFO -> ADC_CS (atomic bit-set alias)
    int dma_trig_chan = dma_claim_unused_channel(true);
    dma_channel_config trig_cfg = dma_channel_get_default_config(dma_trig_chan);
    channel_config_set_transfer_data_size(&trig_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&trig_cfg, false);
    channel_config_set_write_increment(&trig_cfg, false);
    channel_config_set_dreq(&trig_cfg, pio_get_dreq(pio, sm, false)); // DREQ_PIO0_RX0

    dma_channel_configure(
        dma_trig_chan,
        &trig_cfg,
        hw_set_alias_untyped(&adc_hw->cs), // Write 0x04 to set START_ONCE bit atomically
        &pio->rxf[sm],                     // Read trigger word from PIO RX FIFO
        NUM_PIXELS,
        false
    );

    // 4. Configure DMA Channel 1: ADC FIFO -> frame_buf
    int dma_cap_chan = dma_claim_unused_channel(true);
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

    while (true) {
        // Clean FIFOs before arming the frame
        adc_fifo_drain();
        pio_sm_clear_fifos(pio, sm);

        // Arm both DMA channels for 2090 pixels
        dma_channel_set_trans_count(dma_trig_chan, NUM_PIXELS, true);
        dma_channel_set_write_addr(dma_cap_chan, frame_buf, true);

        // Trigger the frame by pushing FLUSH_CYCLES and (NUM_PIXELS - 1) to PIO TX FIFO
        pio_sm_put_blocking(pio, sm, FLUSH_CYCLES);
        pio_sm_put_blocking(pio, sm, NUM_PIXELS - 1);

        // Wait until all 2090 ADC samples have been transferred to frame_buf
        dma_channel_wait_for_finish_blocking(dma_cap_chan);

        // Stream frame to host PC if USB CDC is connected
        if (stdio_usb_connected()) {
            fwrite(sync_header, 1, sizeof(sync_header), stdout);
            fwrite(frame_buf, sizeof(uint16_t), NUM_PIXELS, stdout);
            fflush(stdout);
        }
    }
}