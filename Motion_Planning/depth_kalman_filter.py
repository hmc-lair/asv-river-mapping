'''
depth_kalman_filter.py
Kalman filter + log file reading helpers
'''
import numpy as np

# Plot parameters
win = 5
sigma_slope = 0.1204
sigma_offset = 0.6142

def kalman_filter(ASV_nor, ASV_eas, Z, CELL_RES):
	min_depth = Z.min()

	#Normalize positions
	min_x = min(ASV_nor)
	min_y = min(ASV_eas)
	X = [v - min_x for v in ASV_nor]
	Y = [v - min_y for v in ASV_eas]
	max_x = max(X)
	max_y = max(Y)

	# Method 0: 1D Kalman Filter
	m = int(np.ceil(max_x/CELL_RES))
	n = int(np.ceil(max_y/CELL_RES))
	baseFloor = min_depth
	B = baseFloor*np.ones((m,n))
	Bvar = np.zeros((m,n))

	for t in range(len(ASV_nor)):
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
	return B, m, n, min_x, min_y

###############################################################################
# GPS/ADCP File Processing
###############################################################################

# Returns pixel coordinates from GPS data
def read_data_file(filename):
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
	ASV_nor = []
	ASV_eas = []
	for x, y in all_states:
		ASV_nor.append(x)
		ASV_eas.append(y)

	#Water depths
	depths = read_ADCP_file(ADCP_data)

	return np.asarray(ASV_nor), np.asarray(ASV_eas), depths

def read_ADCP_file(data):
	depths = []
	for ensemble in data:
		ensemble_data = ensemble[6:] #Remove '$ADCP,'
		result = read_ensemble(ensemble_data)
		vals = [float(d)*np.cos(BEAM_ANGLE*np.pi/180.) for d in result[6]]
		depths.append(np.average(vals) + TRANSDUCER_OFFSET)
	return -np.asarray(depths)

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