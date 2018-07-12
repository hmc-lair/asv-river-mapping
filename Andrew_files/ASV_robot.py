from ASV_environment import *
from ASV_state import *
import math

class ASV_robot:

	def __init__(self, environment):
		self.environment = environment
		self.state_est = ASV_state()
		# self.measurements = ASV_state()

		# Way points to navigate
		self.way_points = []
		self.dt = 0.01

		# robot commands
		self.rudder = 0.0
		self.R_motor = 0.0
		self.L_motor = 0.0

	def main_loop(self):
		'''main control loop for the ASV'''

		# 1. Predict asv position
		# 2. If GPS/DVL/Compass available, update accordingly
		# 3. Motion plan
		# 4. Update control signals
		pass

	def localize_with_GPS(self):
		'''state estimate with GPS'''
		# 1. Update position with GPS
		pass

	def localize_with_DVL(self):
		'''state estimate with DVL'''
		# 1. Update position with DVL
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

	def predict(self):
		'''Predict with just the internal states of the robot'''
		self.state_est.x = self.state_est.x + self.state_est.v * math.cos(self.state_est.theta) * self.dt
		self.state_est.y = self.state_est.x + self.state_est.v * math.sin(self.state_est.theta) * self.dt
		self.state_est.theta = self.state_est.theta + self.state_est.omega * self.dt

	def update_odometry(self):
		'''Given DVL velocities update odometry'''
		pass

class ASV_robot_sim(ASV_robot):

	def __init__(self, environment):
		super().__init__(self, environment)

	def main_loop(self):
		pass

	def localize_with_DVL(self):
		pass

	def localize_with_GPS(self):
		pass


