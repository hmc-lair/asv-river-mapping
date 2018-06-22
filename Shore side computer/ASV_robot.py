from ASV_environment import *

class ASV_robot:

	def __init__(self, environment):
		self.environment = environment
		self.measurements = []

	def update_sensor_measurements(self):
		'''Update GPS, Compass, DVL, Depth (more to come) measurements'''
		pass

	def localize(self):
		'''Use whatever state estimation technique to localize the ASV'''
		pass

	def update_control(self):
		'''Send command messages to the ASV'''
		pass

	def point_track(self):
		'''Generate motor values given a point and current state'''
		pass

	def log_data(self):
		'''Log relevant data'''
		pass

	def update_odometry(self):
		'''Given DVL velocities update odometry'''
		pass
