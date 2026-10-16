import sys
import threading
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button

# Serial & Sensor Configuration
SERIAL_PORT = "COM4"  # Change to "COMx" on Windows
BAUD_RATE = 115200            # Ignored by USB CDC, runs at native USB speed
NUM_PIXELS = 2090
ACTIVE_START = 32             # First effective pixel (S1)
ACTIVE_END = 2080             # Last effective pixel + 1 (S2048 is index 2079)
OB_START, OB_END = 12, 30     # Optical Black reference pixels (D13..D30)
SYNC_HEADER = b"\xaa\x55\xaa\x55"
FRAME_BYTES = NUM_PIXELS * 2

# Averaging Configuration
DEFAULT_AVG_FRAMES = 16       # Initial number of frames to average
MAX_AVG_FRAMES = 128          # Maximum capacity of the circular frame buffer

# Shared state between serial reader thread and GUI
frame_buffer = np.zeros((MAX_AVG_FRAMES, NUM_PIXELS), dtype=np.float32)
write_idx = 0
valid_frames = 0
target_avg_frames = DEFAULT_AVG_FRAMES
frame_lock = threading.Lock()
running = True


def push_frame(new_frame: np.ndarray):
    """Writes a new frame into the circular buffer under lock."""
    global write_idx, valid_frames
    with frame_lock:
        frame_buffer[write_idx] = new_frame
        write_idx = (write_idx + 1) % MAX_AVG_FRAMES
        if valid_frames < MAX_AVG_FRAMES:
            valid_frames += 1


def get_averaged_frame() -> tuple[np.ndarray, int]:
    """Returns the mean of the most recent N frames and the actual count used."""
    with frame_lock:
        count = min(valid_frames, target_avg_frames)
        if count == 0:
            return np.zeros(NUM_PIXELS, dtype=np.float32), 0

        # Extract the most recent `count` frames from the circular buffer
        indices = (write_idx - 1 - np.arange(count)) % MAX_AVG_FRAMES
        avg_frame = np.mean(frame_buffer[indices], axis=0)
        return avg_frame, count


def reset_buffer(_=None):
    """Clears the accumulated frame history."""
    global valid_frames, write_idx
    with frame_lock:
        valid_frames = 0
        write_idx = 0


def serial_reader_thread(port: str):
    """Continuously syncs on SYNC_HEADER and pushes 2090-pixel frames to the ring buffer."""
    global running
    buf = bytearray()

    try:
        with serial.Serial(port, BAUD_RATE, timeout=0.5) as ser:
            ser.reset_input_buffer()
            while running:
                chunk = ser.read(4096)
                if not chunk:
                    continue
                buf.extend(chunk)

                while True:
                    idx = buf.find(SYNC_HEADER)
                    if idx == -1:
                        if len(buf) > 3:
                            del buf[:-3]
                        break

                    if idx > 0:
                        del buf[:idx]

                    if len(buf) < 4 + FRAME_BYTES:
                        break

                    payload = buf[4 : 4 + FRAME_BYTES]
                    del buf[: 4 + FRAME_BYTES]

                    frame = np.frombuffer(payload, dtype="<u2") & 0x0FFF
                    push_frame(frame)
    except serial.SerialException as e:
        print(f"Serial error: {e}", file=sys.stderr)
        running = False


def main():
    global running, target_avg_frames
    port = sys.argv[1] if len(sys.argv) > 1 else SERIAL_PORT
    thread = threading.Thread(target=serial_reader_thread, args=(port,), daemon=True)
    thread.start()

    fig, (ax_cal, ax_raw) = plt.subplots(2, 1, figsize=(11, 7.5), sharex=True)
    fig.canvas.manager.set_window_title("Sony ILX554A Linear CCD Live View (Averaged)")
    plt.subplots_adjust(bottom=0.16, hspace=0.25)

    x_active = np.arange(2048)
    x_all = np.arange(NUM_PIXELS)

    # Top plot: Dark-subtracted & inverted optical intensity (2048 active pixels)
    (line_cal,) = ax_cal.plot(x_active, np.zeros(2048), color="#0072B2", lw=1.1)
    title_cal = ax_cal.set_title(
        f"Calibrated Optical Intensity (Averaging: {DEFAULT_AVG_FRAMES} frames)"
    )
    ax_cal.set_ylabel("Intensity (ADC Counts)")
    ax_cal.set_ylim(-100, 4096)
    ax_cal.set_xlim(0, 2048)
    ax_cal.grid(True, alpha=0.3)

    # Bottom plot: Averaged raw 12-bit ADC readout across all 2090 clock cycles
    (line_raw,) = ax_raw.plot(x_all, np.zeros(NUM_PIXELS), color="#D55E00", lw=0.9)
    ax_raw.axvspan(0, ACTIVE_START, color="gray", alpha=0.2, label="Leading Dummy / Optical Black")
    ax_raw.axvspan(ACTIVE_END, NUM_PIXELS, color="gray", alpha=0.2, label="Trailing Dummy")
    ax_raw.set_title("Averaged Raw ADC Readout")
    ax_raw.set_xlabel("Pixel Index")
    ax_raw.set_ylabel("Raw 12-bit ADC")
    ax_raw.set_ylim(0, 4096)
    ax_raw.legend(loc="lower right")
    ax_raw.grid(True, alpha=0.3)

    # Interactive Slider for Frame Averaging (1 to MAX_AVG_FRAMES)
    ax_slider = plt.axes([0.15, 0.05, 0.58, 0.03])
    slider_avg = Slider(
        ax=ax_slider,
        label="Avg Frames (N)",
        valmin=1,
        valmax=MAX_AVG_FRAMES,
        valinit=DEFAULT_AVG_FRAMES,
        valstep=1,
    )

    def on_slider_change(val):
        global target_avg_frames
        with frame_lock:
            target_avg_frames = int(val)

    slider_avg.on_changed(on_slider_change)

    # Reset Button to clear history on rapid signal changes
    ax_btn = plt.axes([0.78, 0.045, 0.12, 0.04])
    btn_reset = Button(ax_btn, "Reset Buffer")
    btn_reset.on_clicked(reset_buffer)

    def update(_):
        avg_raw, count = get_averaged_frame()
        if count == 0:
            return line_cal, line_raw

        # Compute dark baseline from averaged Optical Black pixels (D13..D30)
        ob_level = np.mean(avg_raw[OB_START:OB_END])
        active_intensity = ob_level - avg_raw[ACTIVE_START:ACTIVE_END]

        line_cal.set_ydata(active_intensity)
        line_raw.set_ydata(avg_raw)
        title_cal.set_text(
            f"Calibrated Optical Intensity (Averaging: {count}/{target_avg_frames} frames | "
            f"SNR Gain: {np.sqrt(count):.1f}x)"
        )
        return line_cal, line_raw

    ani = FuncAnimation(fig, update, interval=25, blit=False, cache_frame_data=False)
    try:
        plt.show()
    finally:
        running = False


if __name__ == "__main__":
    main()
