# Test drive around in RC mode and write the ADCP raw data to a file
from digi.xbee.devices import XBeeDevice
import time
import datetime
import serial

class Simple_controller():

	def __init__(self):
		self.xbee_port = "/dev/tty.usbserial-DN02Z6QY"
		self.local_xbee = XBeeDevice(self.xbee_port, 9600)
		self.local_xbee.open()
		self.local_xbee.serial_port.reset_input_buffer()
		self.boat_xbee = []

		self.hem_lat = 'N/A'
		self.hem_long = 'N/A'
		self.adcp_lat = 'N/A'
		self.velocity = 'N/A'
		self.depth = 'N/A'
		self.roll = 'N/A'
		self.pitch = 'N/A'
		self.yaw = 'N/A'
		self.adcp_lat = 'N/A'
		self.adcp_long = 'N/A'

		self.last_gps_received = datetime.datetime.now()
		self.last_adcp_received = datetime.datetime.now()


	'''
	Discover the boat xbee
	Modify: self.boat_xbee
	'''
	def discover_boat(self):
		print('Discovering device')
		xbee_network = self.local_xbee.get_network()
		xbee_network.start_discovery_process()
		while xbee_network.is_discovery_running():
			time.sleep(0.5)
		self.boat_xbee = xbee_network.discover_device('boat')

	'''
	Call back function whenever a message arrived. Sort out GPS
	and ADCP information

	Input:
		xbee_message: messages from the other xbee
	'''
	def data_received_callback(self,xbee_message):
		data = xbee_message.data.decode()
		parsed_data = data.split(',')

		if parsed_data[0] == '$GPGGA':
			self.last_gps_received = datetime.datetime.now()
			print('Received GPS: ', data)
		elif parsed_data[0] == '$ADCP':
			self.adcp_gps_received = datetime.datetime.now()
			print('Received ADCP: ', data )

	''' 
	Initialize connection between boat and controller
	'''
	def start_connection(self):
		# setting up xbee communication
		self.local_xbee.add_data_received_callback(self.data_received_callback)
		self.discover_boat()

		if self.boat_xbee == None:
			print('device not found!')
			return False
		else:
			print('device found! Sending start messages')

		# sending start command to the boat
		start_msg = "START".encode()
		self.local_xbee.send_data_async(self.boat_xbee, start_msg)

		return True

	'''
	End connection between boat and controller
	'''
	def end_connection():
		end_msg = "STOP".encode()
		self.local_xbee.send_data_async(self.boat_xbee, end_msg)
		self.local_xbee.close()


