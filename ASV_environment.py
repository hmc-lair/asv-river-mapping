import serial
from xbee import XBee

class ASV_environment:

	def __init__(self):

		self.map = [] # Not sure what this will look like

		self.robot_mode = "HARDWARE MODE" # HARDWARE MODE
		# setup xbee communication
		if (self.robot_mode == "HARDWARE MODE"):
			self.serial_port = serial.Serial('/dev/tty.usbserial-DN02Z3LX', 9600)
			self.xbee = XBee(self.serial_port)

	