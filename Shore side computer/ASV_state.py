class ASV_state:

	def __init__(self, lat = 0.0, lon = 0.0, heading = 0.0):
		self.set_state(lat, lon, heading)

	def set_state(self, lat, lon, heading):
		self.lat = lat
		self.lon = lon
		self.heading = heading