from digi.xbee.devices import XBeeDevice
import serial
import time

COM_PORT = '/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FT8VW9AR-if00-port0' #Pi ADCP
xbee_port = '/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DN02Z3LX-if00-port0' #Pi xbee
# xbee_port = '/dev/tty.usbserial-DN02Z6QY'
GPS_PORT = '/dev/tty.usbserial-FT8VWDWP'
BAUD_RATE = "115200"
ser = serial.Serial(COM_PORT, BAUD_RATE, stopbits=serial.STOPBITS_ONE)
GPS_ser = serial.Serial(GPS_PORT, bytesize = 8)
boat_xbee = XBeeDevice(xbee_port, 9600)

f = open('results.bin', 'wb')

###############################################################################
# GPS Setup Function
###############################################################################
def GPS_setup(gps_ser):
	gps_ser.baudrate = 19200
	gps_ser.parity = serial.PARITY_NONE
	gps_ser.stop_bits = serial.STOPBITS_ONE
	return

def data_received_callback(xbee_message):
	data = xbee_message.data
	if data == "STOP":
		raise Exception("Terminating")
	return data

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

###############################################################################
# XBEE Setup Function
###############################################################################
'''
Discover the central xbee

Inputs:
	xbee - XBeeDevice
Outputs:
	central_xbee - XBeeDevice (None if no discovery)
'''
def discover_xbee(xbee):
	xbee.open()
	xbee_network = xbee.get_network()
	xbee_network.start_discovery_process()
	while xbee_network.is_discovery_running():
		time.sleep(0.5)
	print(xbee_network.get_devices())
	central_xbee = xbee_network.discover_device('central')

	if central_xbee == None:
		print('No device found... ')
	else:
		print('device found! waiting..')
		xbee.send_data(central_xbee, b'Device found! Waiting for starting cue...')
		msg = xbee.read_data(100)
		print(msg.data)
	return central_xbee


###############################################################################
# ADCP Setup/Helper Functions
###############################################################################
'''
Setup ADCP connection
'''
def setup():
	ser.flush()
	print("Starting ADCP communication")
	ser.write(b'+++')
	time.sleep(3)
	s = read_response()
	print('Startup message: ', s)
	return ser

'''
Sends specified command and outputs response

Inputs:
	ser - Serial port
	command - in bytes
'''
def send(command):
	ser.write(command + b'\r\n')
	s = read_response(ser, verbose=True)
	return s

'''
Reads entire output message
'''
def read_response(verbose=False):
	response = b''
	cur_line = b''
	while True:
		s = ser.read()
		response += s
		cur_line += s
		if verbose and s == b'\n':
			print(cur_line)
			cur_line = b''

		if b'>' in response: #stop character
			return response
	return

###############################################################################
# ADCP Control Commands
###############################################################################
def start_ping():
	print("Requesting Pings")
	ser.write(b'CS\r\n')
	ser.read(10) #response includes command	

def stop_ping():
	print('Stopping Pings')
	ser.write(b'CSTOP\r\n')
	
def read_ensemble(verbose=False):
	header = ser.read(2)
	if header != b'\x7f\x7f':
		print('ERROR no header: ', header)
  
	num_bytes = ser.read(2)
	bytes_to_checksum = int.from_bytes(num_bytes, byteorder='little')-4
	if verbose:
		print('Num: ', bytes_to_checksum)
	
	data = ser.read(bytes_to_checksum)
	if verbose:
		print('Data: ', data)

	#use checksum to verify no errors
	checksum = ser.read(2)
	checksum_int = int.from_bytes(checksum, byteorder='little') 
	datasum_int = sum(b'\x7f\x7f' + num_bytes + data) % 2**16

	if checksum_int != datasum_int:
		print('ERROR: ', checksum_int, datasum_int)

	#read data to file
	all_data = b'\x7f\x7f' + num_bytes + data + checksum
	f.write(all_data)

	return all_data

###############################################################################

def main():
	# Setting up
	setup()
	GPS_setup(GPS_ser)
	central_xbee = discover_xbee(boat_xbee)
	boat_xbee.set_sync_ops_timeout(10)
	boat_xbee.add_data_received_callback(data_received_callback)

	if central_xbee == None:
		return
	
	start_ping()
	try:
		while True:
			ensemble = read_ensemble(verbose=False)
			print(ensemble)

			# Sending GPS messages			
			if GPS_ser.in_waiting > 0:
				data_str = GPS_ser.readline().decode()
				if (right_GPS_msg(data_str)):
					boat_xbee.send_data_async(central_xbee, data_str)

			# Sending essential ensemble message

	except:
		pass


	stop_ping()
	ser.write(b'===')
	ser.close()
	boat_xbee.close()
	GPS_ser.close()
	f.close()

if __name__=='__main__':
	main()
