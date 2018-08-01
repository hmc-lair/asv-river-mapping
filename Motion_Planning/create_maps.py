'''
createMaps.py
Creates occupancy grid and info map for RRT planner
'''
import numpy as np
import matplotlib.image as mpimg
from scipy import signal
from scipy import ndimage
import scipy.io as sio
import os
import utm
import gdal
import struct

sigma = 2 #Gaussian blur for info map
tau = 0.4 #Thresholding

BEAM_ANGLE = 20 #degrees
TRANSDUCER_OFFSET = 0.1 #m
SCALING_FACTOR = 10

#ASV log files
millikan1 = 'Log/millikan_7-19/ALL_18-07-11 23.59.36.bin' #1) GOOD MAP OF MILLIKAN
millikan2 = 'Log/millikan_7-19/ALL_18-07-12 00.16.04.bin' #2) Good
millikan3 = 'Log/millikan_7-19/ALL_18-07-11 23.36.33.bin' #3) Good
millikan4 = 'Log/millikan_7-19/ALL_18-07-12 00.24.24.bin' #4) Large range

lake1 = '../ASV_Controller/Log/lake_7-27/ALL_18-07-12 06.49.05.bin'
river1 = '../ASV_Controller/Log/river_7-27/ALL_18-07-12 06.47.41.bin'

data_file = river1

# To crop GEOTIFF use:
# gdal_translate -srcwin 3000 9000 4000 3000 input.tif output.tif
map_file = '../Maps/river_7-27.tif'

# Plot parameters
CELL_RES = 1
win = 5
sigma_slope = 0.1204
sigma_offset = 0.6142

###############################################################################
# Map Helper Functions
###############################################################################

def load_map(filename):
	dataset = gdal.Open(filename)
	data = dataset.ReadAsArray()
	_, height, width = data.shape
	geo_trans = dataset.GetGeoTransform()
	inv_trans = gdal.InvGeoTransform(geo_trans)

	img = mpimg.imread(filename)
	return img, inv_trans, geo_trans, width, height

###############################################################################
# GPS/ADCP File Processing
###############################################################################

# Returns pixel coordinates from GPS data
def read_data_file(filename, inv_trans):
	f = open(data_file, 'rb')
	all_data = b''
	
	for line in f.readlines():
		all_data += line

	split_data = all_data.split(b'###')
	#GPS_data = list(filter(lambda x: x.split(b',')[0] == b'$GPS', split_data[:-1]))
	
	ADCP_data = list(filter(lambda x: x.split(b',')[0] == b'$ADCP', split_data[:-1]))
	state_data = list(filter(lambda x: x.split(b',')[0] == b'$STATE', split_data[:-1]))
	
	#GPS data
	state_data_split = [state.split(b',') for state in state_data]
	all_states = [(float(state[1]), float(state[2])) for state in state_data_split]
	ASV_X = []
	ASV_Y = []
	for x, y in all_states:
		ASV_X.append(x)
		ASV_Y.append(y)

	#Water depths
	depths = read_ADCP_file(ADCP_data)

	return np.asarray(ASV_X), np.asarray(ASV_Y), depths

def read_ADCP_file(data):
	depths = []
	for ensemble in data:
		ensemble_data = ensemble[6:] #Remove '$ADCP,'
		result = read_ensemble(ensemble_data)
		vals = [float(d)*np.cos(BEAM_ANGLE*np.pi/180.) for d in result[6]]
		depths.append(np.average(vals) + TRANSDUCER_OFFSET)
	return -np.asarray(depths)


##########################################################

def generateGradientMap():
	img, inv_trans, geo_trans, MAP_WIDTH, MAP_HEIGHT = load_map(map_file)

	#GPS DATA
	ASV_X, ASV_Y, Z = read_data_file(data_file, inv_trans)

	if len(ASV_X) != len(Z):
		print('Size mismatch!', len(ASV_X), len(Z))
		ASV_X = ASV_X[:-1]
		ASV_Y = ASV_Y[:-1]

	min_depth = Z.min()

	#Normalize positions
	min_x = min(ASV_X)
	min_y = min(ASV_Y)
	X = [v - min_x for v in ASV_X]
	Y = [v - min_y for v in ASV_Y]
	max_x = max(X)
	max_y = max(Y)

	# Method 0: 1D Kalman Filter
	m = int(np.ceil(max_x/CELL_RES))
	n = int(np.ceil(max_y/CELL_RES))
	baseFloor = min_depth
	B = baseFloor*np.ones((m,n))
	Bvar = np.zeros((m,n))

	for t in range(len(ASV_X)):
		cur_x = X[t]
		cur_y = Y[t]
		cur_alt = Z[t]

		i = int(np.floor(cur_x/CELL_RES))
		j = int(np.floor(cur_y/CELL_RES))

		for k in range(max(0,i-win), min(m, i+win)):
			for l in range(max(0,j-win), min(n,j+win)):
				dist2 = 0.1+CELL_RES*((k-i)**2+(l-j)**2)**0.5
					
				if B[k][l] == baseFloor:
					B[k][l] = cur_alt
					Bvar[k][l] = (dist2*sigma_slope+sigma_offset)**2
				else:
					var = (dist2*sigma_slope+sigma_offset)**2
					cur_K = float(Bvar[k][l])/(Bvar[k][l] + var)
					B[k][l] = B[k][l]+cur_K*(cur_alt - B[k][l])
					Bvar[k][l] = Bvar[k][l]-cur_K*Bvar[k][l];

	B_new = np.zeros((n,m))
	for i in range(n):
		for j in range(m):
			B_new[i][j] = B[j][i]

	Gx, Gy = np.gradient(B_new) # gradients with respect to x and y
	G = (Gx**2+Gy**2)**.5  # gradient magnitude
	
	with open('river_map.csv', 'w') as f:
		for i in range(len(G)):
			for j in range(len(G[0])):
				if G[i][j] > .4:
					G[i][j] = 0
				G[i][j] *= SCALING_FACTOR

				f.write(str(round(G[i][j], 3)))
				if j < len(G[0])-1:
					f.write(',')
			f.write('\n')

	return G, len(G), len(G[0])

def setupInfoMap(filename):
	# Load file
	E = np.loadtxt(open(filename, 'r'), delimiter=',')
	m,n = E.shape
	return E, m, n

def createInfoMap(E, m, n):
	#Calculate gradient of E using Sobel operator
	Sobel_x = [[-1,0,1],[-2,0,2],[-1,0,1]]
	Sobel_y = [[-1,-2,-1],[0,0,0],[1,2,1]]
	G_x = signal.convolve2d(Sobel_x, E)
	G_y = signal.convolve2d(Sobel_y, E)
	G = np.zeros((m,n)) #Gradient of E
	for i in range(m):
		for j in range(n):
			G[i][j] = np.sqrt(G_x[i][j]**2 + G_y[i][j]**2)

	#Info map is Gaussian blur kernel * G
	return ndimage.filters.gaussian_filter(G,sigma)

''' function that takes in 2d info map and scores it for calculating coverage metric at end of planner'''
def scoreInfoMap(infoMap):

	map_score = 0
	xlen = len(infoMap[0])
	ylen = len(infoMap)
	max_val = 0

	for x in range(xlen):
		for y in range(ylen):
			map_score += infoMap[y][x]
			if infoMap[y][x] > max_val:
					max_val = infoMap[y][x]	
				
	return map_score, max_val

###############################################################################
# ADCP Reading Data
###############################################################################

def read_ensemble(data):
	all_data = data
	num_bytes = int.from_bytes(all_data[2:4], byteorder='little')
	# print(all_data[:2])
	# print('Num bytes: ', num_bytes)
	num_types = all_data[5]

	return_data = [] # current offset, roll, pitch, yaw
					# depth cell length, velocities, bt_range, bt_velocity, GPS
	
	# OFFSETS
	# 1. Fix Leader
	# 2. Variable Leader
	# 3. Velocity Profile
	# 6. Bottom Track
	# 9. Vertical Beam Range
	# 10-13. GPS
	offsets = []
	for i in range(num_types):
		offset = all_data[6+2*i:8+2*i]
		offset_int = int.from_bytes(offset, byteorder='little')
		offsets.append(offset_int)

	# FIXED LEADER
	fixed_offset = offsets[0]

	num_beams = all_data[fixed_offset+8]
	num_cells = all_data[fixed_offset+9]
	pings_per_ensemble = int.from_bytes(all_data[fixed_offset+10: fixed_offset+12], byteorder='little')
	depth_cell_length = int.from_bytes(all_data[fixed_offset+12: fixed_offset+14], byteorder='little') # cm
	coord_transform = all_data[fixed_offset+25]

	# VARIABLE LEADER
	variable_offset = offsets[1]

	transducer_depth = int.from_bytes(all_data[variable_offset+16: variable_offset+18], byteorder='little')*0.1 #1 dm
	heading = int.from_bytes(all_data[variable_offset+18: variable_offset+20], byteorder='little')*0.01 #0 to 359.99
	pitch = int.from_bytes(all_data[variable_offset+20: variable_offset+22], byteorder='little', signed=True)*0.01 #-20 to 20
	roll = int.from_bytes(all_data[variable_offset+22: variable_offset+24], byteorder='little', signed=True)*0.01 #-20 to 20
	salinity = int.from_bytes(all_data[variable_offset+24: variable_offset+26], byteorder='little') #0 to 40 part per thousand
	temperature = int.from_bytes(all_data[variable_offset+26: variable_offset+28], byteorder='little')*0.01 #-5 to 40 degrees

	# VELOCITY PROFILE
	velocity_profile_offset = offsets[2]
	relative_velocities = []

	for i in range(num_cells):
		start_offset = velocity_profile_offset + 2 + 2*i
		# Average over beams
		vel = []
		for j in range(num_beams):
			curVel = int.from_bytes(all_data[start_offset + 2*j: start_offset + 2 + 2*j], byteorder='little', signed=True)
			vel.append(curVel)
		relative_velocities.append(vel)

	bt_offset = offsets[5]
	bt_pings_per_ensemble = int.from_bytes(all_data[bt_offset+2:bt_offset+4], byteorder='little')
	bt_ranges = [] # ranges measurement for each beam (bt = 0 is bad data) # cm
	bt_velocities = [] # there are one more velocity data.. though not sure what it's for?
	beam_percent_good = []
	max_tracking_depth = int.from_bytes(all_data[bt_offset+70:bt_offset+72], byteorder = 'little')
	bt_error_vel_max = int.from_bytes(all_data[bt_offset + 10:bt_offset+12], byteorder = 'little')
	for i in range(4):
		bt_ranges.append(int.from_bytes(all_data[bt_offset+16+i*2:bt_offset+18+i*2], byteorder = 'little')*.01)
		bt_velocities.append(int.from_bytes(all_data[bt_offset+24+i*2:bt_offset+26+i*2], byteorder = 'little',signed=True))
		beam_percent_good.append(all_data[bt_offset+40+i])

	# VERTICAL BEAM RANGE
	vb_offset = offsets[8]
	vb_range = int.from_bytes(all_data[vb_offset+4:vb_offset+8], byteorder = 'little') # in millimeter

	time_stamp = ''
	if num_types == 16:
		# GPS Data
		GPS_offsets = offsets[13:15]
		msg_types = []
		msg_sizes = []
		delta_times_bytes = []
		delta_times_double = [] # difference between GPS message and ensemble
		GPS_msg = []

		for g_offset in GPS_offsets:
			msg_size = int.from_bytes(all_data[g_offset+4:g_offset+6], byteorder = 'little')
			msg_types.append(int.from_bytes(all_data[g_offset+2:g_offset+4], byteorder = 'little'))
			msg_sizes.append(msg_size)
			delta_times_bytes.append(all_data[g_offset+6:g_offset+14])
			GPS_msg.append(all_data[g_offset+15: g_offset+15+msg_size])

		time_stamp = GPS_msg[0].decode().split(',')[1]

		delta_times_double = [struct.unpack('d', b)[0] for b in delta_times_bytes] # convert to double

	return_offset = 0
	return_data = [return_offset, roll, pitch, heading, depth_cell_length, 
		relative_velocities, bt_ranges, bt_velocities, time_stamp]

	return return_data
