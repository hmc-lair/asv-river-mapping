import matplotlib.pyplot as plt
def main():
	f = open('afternoon_test-7-19.bin', 'rb')
	all_data = b''
	
	for line in f.readlines():
		all_data += line

	split_data = all_data.split(b'###')
	GPS_data = list(filter(lambda x: x.split(b',')[0] == b'$GPS', split_data[:-1]))
	ADCP_data = list(filter(lambda x: x.split(b',')[0] == b'$ADCP', split_data[:-1]))
	state_data = list(filter(lambda x: x.split(b',')[0] == b'$STATE', split_data[:-1]))
	
	state_data_split = [state.split(b',') for state in state_data]
	ASV_x = [float(state[1]) for state in state_data_split]
	ASV_y = [float(state[2]) for state in state_data_split] 
	# print(state_data[0][0])
	plt.plot(ASV_x, ASV_y, color='black', zorder=2)
	plt.xlabel('Meters')
	plt.ylabel('Meters')

	plt.show()
	# print(state_data)
	# print(ADCP_data[0][6:8])
	# print(read_ensemble(ADCP_data[0][6:]))

	# print(ADCP_data[0][:6])

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
    # print('Offsets: ', offsets)
    # print('Data IDs: ', [all_data[x:x+2] for x in offsets])
    # print('Data types:', num_types)

    # FIXED LEADER
    fixed_offset = offsets[0]

    num_beams = all_data[fixed_offset+8]
    num_cells = all_data[fixed_offset+9]
    pings_per_ensemble = int.from_bytes(all_data[fixed_offset+10: fixed_offset+12], byteorder='little')
    depth_cell_length = int.from_bytes(all_data[fixed_offset+12: fixed_offset+14], byteorder='little') # cm
    coord_transform = all_data[fixed_offset+25]
    print('Coord Transform: ', coord_transform)
    # print('Depth cell length', depth_cell_length)

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
            #print('Beam vel', j, curVel)
            vel.append(curVel)
        #vel = vel/float(num_beams)
        relative_velocities.append(vel)
    # print(relative_velocities)
    # print('Num cells: ', num_cells)
    # print('Velocity profile: ', relative_velocities)

    # BOTTOM TRACK (abbr. bt) (see page 154)

        # Coordinate system for velocity:
            # 1. Earth Axis (default): East, North, Up (right hand orthogonal) 
                # need to set heading alignment (EA), heading bias (EB) correctly
                # make sure heading sensors are active (EZ)
            # 2. Radial Beam Coordinates: "raw beam measurements" (not orthogonal)
            # 3. Instrument coordinates: X, Y, UP, X is directon of beam 2, Y is beam 3. Compass
                # measures the offset of Y from magnetic north
            # 4. Ship coordinates: starboard, forward, mast (pitch, roll, yaw)

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

    # print('BT values: ', bt_ranges, bt_velocities, beam_percent_good)
    # print('bt beam 1 corr magnitude', bt_correlation_mag)
    # print('bt pings/ensemble ', bt_pings_per_ensemble)
    # print('error max value', bt_error_vel_max)

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
        # print('GPS_msg: ', GPS_msg)
        # print('Delta ts: ', delta_times_double )
    # return_data = [current_offset + num_bytes + 2, roll, pitch, yaw, depth_cell_length, 
    #     relative_velocities]
    # print("delta time," ,msg_types)
    # print('GPS:', GPS_msg)
    # print('Vertical beam range:', vb_range)
    return_offset = 0
    return_data = [return_offset, roll, pitch, heading, depth_cell_length, 
        relative_velocities, bt_velocities, time_stamp]

    return return_data


if __name__ == '__main__':
	main()