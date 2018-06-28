import math
from digi.xbee.devices import XBeeDevice

class ASV_state:

	def __init__(self, x = 0.0, y = 0.0, theta = 0.0):
		self.set_state(x, y, theta)

	def set_state(self, x, y, theta):
		self.x = x
		self.y = y
		self.v = 0.0
		self.theta = theta
		self.omega = 0.0

		# other information
		self.id = 0
		self.xbee = XBeeDevice()
		self.sensors = [0.0, 0.0, 0.0, 0.0]
		self.mode = "MANUAL MODE"
		self.current_dest = [0.0, 0.0, 0.0] # x, y, heading
		self.way_pts = []
		self.manual_command = []
		self.origin = []
		self.offset = 0.0