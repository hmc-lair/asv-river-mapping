import serial
import xbee

def main():
	port_names = {'GPS': '/dev/tty.usbserial-DN02Z3LX',
					'xbee': '/dev/tty.usbserial-DN02Z3LX'}

	# initialize(port_names)

def initialize(port_names):
	'''function to initialize the serial ports'''
	GPS_ser = serial.Serial(port_names['GPS'], bytesize = 8)
	GPS_ser.baudrate = 19200
	GPS_ser.parity = serial.PARITY_NONE
	GPS_ser.stop_bits = serial.STOPBITS_ONE

	xbee_ser = serial.Serial(port_names['xbee'], 9600)
	XBEE = xbee(xbee_ser)

if __name__ == "__main__":
	main()