import math

class ASV_state:

	def __init__(self, x = 0.0, y = 0.0, theta = 0.0):
		self.set_state(x, y, theta)

	def set_state(self, x, y, theta):
		self.x = x
		self.y = y
		self.v = 0.0
		self.a = 0.0

		self.roll = 0.0
		self.pitch = 0.0
		
		self.theta = theta
		self.omega = 0.0
		self.angle_acc = 0.0

		self.lat = 0.0
		self.lon = 0.0

	def __str__(self):
		return str([self.x, self.y, self.theta])