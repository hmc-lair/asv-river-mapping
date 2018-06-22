import serial
from xbee import XBee
import time

def main():
	### GPS_sample ###
	# GSV_sample = '$GPGSV,3,3,10,26,29,257,51,27,10,147,45,45,,,,,,,,*74'
	# VTG_sample = '$GPVTG,308.88,T,308.88,M,0,0.04,N,0.08,K*42'
	# GGA_sample = '$GPGGA,144049.0,5100.1325,N,11402.2729,W,1,07,1.0,1027.4,M,0,M,,010*61'

	port_names = {'GPS': '/dev/tty.usbserial-FT8VW9AR',
				  'xbee': '/dev/tty.usbserial-DN02Z3LX',
				  'ADCP': 'TO BE INCLUDED',
				  'VCU': 'TO BE INCLUDED',
				  'DVL': 'TO BE INCLUDED'}

	shore_xbee_addr = '\x00\x0A'

	## VCU baud rate is 11520, Data bits 8, parity is none

	###### Setting up GPS ######
	print('Finding GPS...')
	while True:
		try:
			GPS_ser = serial.Serial(port_names['GPS'], bytesize = 8)
			GPS_ser.baudrate = 19200
			GPS_ser.parity = serial.PARITY_NONE
			GPS_ser.stop_bits = serial.STOPBITS_ONE
		except OSError as e:
			pass
		else:
			break
	print('GPS Found! \r')

	###### Setting up the Xbee ######
	print('Finding Xbee...')
	while True:
		try:
			xbee_ser = serial.Serial(port_names['xbee'], 9600)
			XBEE = XBee(xbee_ser, callback = send_command2ASV)
		except OSError as e:
			pass
		else:
			break
	print('Xbee found!')


	# Main loop
	while(True):
		try:

			# send GPS if available
			if(GPS_ser.in_waiting > 0):
				data_str = GPS_ser.readline().decode()
				if (right_GPS_msg(data_str)):
					XBEE.tx(dest_addr = shore_xbee_addr, data = data_str)
		
		except KeyboardInterrupt:
			break

	GPS_ser.close()
	XBEE.halt()
	xbee_ser.close()

def right_GPS_msg(GPS_msg):
	''' Decide whether this GPS message is to be sent or not.
		Condition to be sent:
			GPGGA msg with signal present'''
	GPS_data = GPS_msg.split(',')
	if GPS_data[0] == '$GPGGA':
		if GPS_data[6] == '0':
			# if there's no signal then we don't care
			return False
		else:
			return True
	else:
		return False

def send_command2ASV(data):
	''' Send command to the ASV Vessel Control Unit, triggered 
	when there's an incoming command'''
	throttle = 0
	rudder = 0
	message = "!pwm, *, %4.3f, %4.3f, %4.3f, *, *\r\n" % (throttle, throttle, rudder)
	print(data)

def enter_autonomous_mode():
	''' Tell the VCU to enter autonomous mode'''
	message = '!SetAutonomousControl\r\n'

if __name__ == "__main__":
	main()