import serial
import struct
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore

# Configuration
SERIAL_PORT = 'COM4'  # Update to match your Pico's port
BAUD_RATE = 2000000
NUM_PIXELS = 2048

# Setup PyQtGraph Window
app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="ILX554A Linear CCD")
win.resize(1000, 600)
plot = win.addPlot(title="Sensor Output (12-bit)")
plot.setLabel('bottom', 'Pixel Index')
plot.setLabel('left', 'Intensity (ADC Value)')
plot.setYRange(0, 4096)
curve = plot.plot(pen='y')

# Open Serial
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

def update():
    """Reads the serial buffer, finds the latest complete frame, and plots it."""
    in_waiting = ser.in_waiting
    
    # We need at least one full frame (4 byte SYNC + 4096 byte payload)
    if in_waiting >= 4100:
        raw_data = ser.read(in_waiting)
        
        # Search backwards to find the most recent sync header (drops stale frames)
        idx = raw_data.rfind(b'SYNC')
        
        if idx != -1 and len(raw_data) >= idx + 4 + (NUM_PIXELS * 2):
            # Extract exactly the payload size
            frame_data = raw_data[idx+4 : idx+4+(NUM_PIXELS * 2)]
            
            # Unpack Little-Endian Unsigned Shorts (16-bit)
            pixels = struct.unpack('<' + 'H' * NUM_PIXELS, frame_data)
            curve.setData(pixels)

# Run update loop tightly at 10ms intervals
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)

if __name__ == '__main__':
    QtWidgets.QApplication.instance().exec_()
