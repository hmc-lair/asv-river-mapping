import math

class ASV_state:

	def __init__(self, x = 0.0, y = 0.0, theta = 0.0):
		self.set_state(x, y, theta)

	def set_state(self, x, y, theta):
		self.x = x
		self.y = y
		self.v = 0
		self.a = 0.0

		# Course velocity and course angle as presented by GPVTG message
		self.v_course = 0 
		self.ang_course = 0

		self.current_v = 3
		self.current_ang = math.pi/5

		self.roll = 0.0
		self.pitch = 0.0
		
		self.theta = theta
		self.omega = 0.0
		self.angle_acc = 0.0

		self.lat = 0.0
		self.lon = 0.0

	def __str__(self):
		return str([self.x, self.y, self.theta])