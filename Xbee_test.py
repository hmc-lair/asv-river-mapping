import serial
from xbee import XBee 

serial_port = serial.Serial('/dev/tty.usbserial-DN02Z6QY', 9600)
bee = XBee(serial_port)

print(bee.wait_read_frame()['rf_data'])