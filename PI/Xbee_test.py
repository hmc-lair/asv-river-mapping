import serial
from xbee import XBee 

serial_port = serial.Serial('/dev/tty.usbserial-DN02Z6QY', 9600)
bee = XBee(serial_port)
bee.tx(dest_addr = '\x00\x00', data = 'hi')
# print(bee.wait_read_frame()['rf_data'])