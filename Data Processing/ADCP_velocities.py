from ADCP_readbytes import *
from Bathymetry_visualization import *
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import math
import numpy as np
import numpy.matlib

ANGLE_OFFSET = 45
BEAM_ANGLE = 20 / 180 * math.pi
GPS_file = '../PI/Log/GPS_18-07-02 17.27.07.txt'


def convert_velocities(roll, pitch, heading, velocities, depth_cell_lengths):
	'''Convert velocity profiles from beam to earth coordinate'''
	earth_velocities = []
	# loop over each ensemble
	for i in range(len(velocities)):
		roll_r = roll[i] / 180 * math.pi
		pitch_r = pitch[i]/180 * math.pi
		heading_r = heading[i]/180 * math.pi

		# loop over each depth cell at ensemble
		v_ensemble = []
		depth = depth_cell_lengths[i] # to meters
		for depth_cell_beams in velocities[i]:
			# convert each depth cell into global coord
			depth = depth + depth_cell_lengths[i]
			v_x_R, v_y_R, v_z_R, v_error = beam2Instrument(BEAM_ANGLE, depth_cell_beams)
			v_x, v_y, v_z = instrument2Earth(roll_r, pitch_r, heading_r, v_x_R, v_y_R, v_z_R)
			v_ensemble.append((v_x, v_y, v_z, depth * 0.01))
		earth_velocities.append(v_ensemble)

	return earth_velocities

def convert_bt_velocities(roll, pitch, heading, bt_vels):
	bt_earth_vel = []
	for i in range(len(bt_vels)):

		roll_r = roll[i] / 180 * math.pi
		pitch_r = pitch[i]/180 * math.pi
		heading_r = heading[i]/180 * math.pi

		v_x_R, v_y_R, v_z_R, v_error = beam2Instrument(BEAM_ANGLE, bt_vels[i])
		v_x, v_y, v_z = instrument2Earth(roll_r, pitch_r, heading_r, v_x_R, v_y_R, v_z_R)
		bt_earth_vel.append((v_x, v_y, v_z, v_error))

	return bt_earth_vel

def instrument2Earth(roll, pitch, heading, v_x, v_y, v_z):
	''' Convert velocities in the instrument frame to the earth frame '''
	rot_matrix1 = np.matrix([[math.cos(heading), math.sin(heading), 0],
							[-math.sin(heading), math.cos(heading), 0],
							[0, 0, 1]])
	rot_matrix2 = np.matrix([[1, 0, 0],
							[0, math.cos(pitch), -math.sin(pitch)], 
							[0, math.sin(pitch), math.cos(pitch)]])
	rot_matrix3 = np.matrix([[math.cos(roll), 0, math.sin(roll)], 
							[0, 1, 0],
							[-math.sin(roll), 0, math.cos(roll)]])

	inst2earth_rot = rot_matrix1 * rot_matrix2 * rot_matrix3

	v_inst = np.array([v_x, v_y, v_z])

	v_earth = v_inst * inst2earth_rot

	# print(v_earth[0][1])
	# v_earth([0])
	# v_earth = v_earth[0]
	# print('v_earth: ', v_earth)
	# print("v_earth 0:", v_earth[0])
	return v_earth.item((0,0)), v_earth.item((0,1)), v_earth.item((0,2))


def beam2Instrument(beam_angle, beams):
	''' Convert beam coordinate to instrument coordinate
	see the teledyne beam transformation guide for definition'''
	c = 1 # convex transducer head
	a = 1/(2*math.sin(beam_angle))
	b = 1/(4*math.cos(beam_angle))
	d = a/math.sqrt(2)
	THREE_BEAM = False
	BAD_BEAM = False
	bad_count = 0

	for i in range(len(beams)):
		if abs(beams[i]) >= 32700:
			beams[i] = 0
			bad_count += 1

	if bad_count == 1:
		THREE_BEAM = True
	elif bad_count >= 2:
		BAD_BEAM = True
		THREE_BEAM = False

	# print("Bad beams: ", BAD_BEAM)
	# print(beams)

	beams = [float(x) for x in beams]

	
	if THREE_BEAM:
		if abs(beams[0]) >= 32700:
			beams[0] = beams[1] - beams[2] - beams[3]
		elif abs(beams[1]) >= 32700:
			beams[1] = beams[0] - beams[2] - beams[3]
		elif abs(beams[2]) >= 32700:
			beams[2] = beams[0] + beams[1] - beams[3]
		else:
			beams[3] = beams[0] + beams[1] - beams[2]
		vel_x = c * a * (beams[0] - beams[1])
		vel_y = c * a * (beams[3] - beams[2])
		vel_z = b * (beams[0] + beams[1] + beams[2] + beams[3])
		vel_error = 0
	elif BAD_BEAM:
		vel_x = 0
		vel_y = 0
		vel_z = 0
		vel_error = 0
	else:
		vel_x = c * a * (beams[0] - beams[1])
		vel_y = c * a * (beams[3] - beams[2])
		vel_z = b * (beams[0] + beams[1] + beams[2] + beams[3])
		vel_error = d * (beams[0] + beams[1] - beams[2] - beams[3])

	return vel_x, vel_y, vel_z, vel_error

def angleDiff(angle):
	while angle > math.pi:
		angle = angle - 2 * math.pi
	while angle < -math.pi:
		angle = angle + 2 * math.pi

	return angle

def read_ADCP_file(filename):
    # READ FILE
    with open(filename,'rb') as f:
        all_data = b''
        for line in f.readlines():
            data = line
            all_data += data 
    # print(all_data[:1000])
    return all_data

def main():
	# setup file reading
	adcp_file = 'Log/ADCP_18-07-02 17.27.07.bin'
	gps_file = 'Log/GPS_18-07-02 17.27.07.txt'
	map_file = '../Maps/kern_river.tif'
	cur_offset = 0
	ensemble_num = 0
	raw_adcp = read_ADCP_file(adcp_file)

	# Data to be processed
	relative_velocities = []
	bt_velocities = []
	rolls = []
	pitches = []
	headings = []
	depth_cell_lengths = []
	adcp_timestamp_str = []

	ASV_x = []
	ASV_y = []

	# Read adcp data
	print('Loading ADCP')
	while cur_offset < len(raw_adcp):
		cur_ensemble = read_ensemble(raw_adcp, cur_offset)
		cur_offset = cur_ensemble[0]

		# Extracting data
		rolls.append(cur_ensemble[1])
		pitches.append(cur_ensemble[2])
		headings.append(cur_ensemble[3])
		depth_cell_lengths.append(cur_ensemble[4])
		relative_velocities.append(cur_ensemble[5])
		bt_velocities.append(cur_ensemble[6])
		adcp_timestamp_str.append(cur_ensemble[7])

	# Load GPS data
	print('Loading GPS')
	
	img, inv_trans, _, _, _ = load_map(map_file)
	ASV_x, ASV_y= read_GPS_file(gps_file, inv_trans)

	# Assumptions: boat is always flat... (for now)
	v_water_earth = convert_velocities(rolls, pitches, headings, relative_velocities, depth_cell_lengths)
	v_bt_earth = convert_bt_velocities(rolls, pitches, headings, bt_velocities)
	
	ratio = float(len(v_bt_earth))/len(ASV_x)
	
	v_water_earth_down = []
	v_bt_earth_down = []

	for i in range(len(ASV_x)):
		v_water_earth_down.append(v_water_earth[int(i*ratio)])
		v_bt_earth_down.append(v_bt_earth[int(i*ratio)])

	# Method 1: Use 1D Kalman filter to create velocity profiles
	fig = plt.figure()
	ax = fig.gca(projection='3d')
	ax.hold(True)
	for i in range(len(v_water_earth_down)):
		# for each depth cell
		# print(v_water_earth_down[i])
		depth = [x[3] for x in v_water_earth_down[i]]
		x = np.matlib.repmat(ASV_x[i],1, len(v_water_earth_down[i]))[0].tolist()
		y = np.matlib.repmat(ASV_y[i],1, len(v_water_earth_down[i]))[0].tolist()
		v_x = [v[0] for v in v_water_earth_down[i]]
		v_y = [v[1] for v in v_water_earth_down[i]]
		v_z = [v[3] for v in v_water_earth_down[i]]
		# print("x: ", len(x))
		# print("y: ", len(y))
		# print("v_x: ", len(v_x))
		# print("v_y: ", len(v_y))
		# print("depth: ", len(depth))
		# print("v_z: ", len(v_z))
		q = ax.quiver(x, y, depth, v_x, v_y, v_z, length=0.1, cmap='Reds', lw=2)

	# bt_vel
	# v_x_bt = np.array([vel[0] for vel in v_bt_earth_down])
	# v_y_bt = np.array([vel[1] for vel in v_bt_earth_down])

	# # print(max(v_y_bt))
	# # surface relative
	# v_water_surface = [vels[0] for vels in v_water_earth_down]
	# v_x_water = np.array([vel[0] for vel in v_water_surface])
	# v_y_water = np.array([vel[1] for vel in v_water_surface])
	# plt.quiver(ASV_x, ASV_y,  v_x_water - v_x_bt, v_y_water - v_y_bt)
	# print(len(v_x_water))
	# print(len(v_y_water))
	# print(bt_vel_earth)
	# print(arrow_x)
	# print(surface)
	# imgplot = plt.imshow(img)
	plt.show()


if __name__ == "__main__":
	main()