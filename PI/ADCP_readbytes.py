import struct

def main():
    # READ FILE
    with open('data/test_results.bin','rb') as f:
        all_data = b''
        for line in f.readlines():
            data = line
            all_data += data 
    # print(all_data)

    num_types = all_data[5]
    
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

    # FIXED LEADER
    fixed_leader_offset = offsets[0]

    num_beams = all_data[fixed_leader_offset+8]
    num_cells = all_data[fixed_leader_offset+9]
    pings_per_ensemble = int.from_bytes(all_data[fixed_leader_offset+10: fixed_leader_offset+12], byteorder='little')
    depth_cell_length = int.from_bytes(all_data[fixed_leader_offset+12: fixed_leader_offset+14], byteorder='little')
    print('Num beams: ', num_beams)
    print('Num cells: ', num_cells)

    # VARIABLE LEADER

    # VELOCITY PROFILE

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

    for i in range(4):
        bt_ranges.append(int.from_bytes(all_data[bt_offset+16+i*2:bt_offset+18+i*2], byteorder = 'little'))
        bt_velocities.append(int.from_bytes(all_data[bt_offset+24+i*2:bt_offset+26+i*2], byteorder = 'little'))
        beam_percent_good.append(all_data[bt_offset+40+i])

    # VERTICAL BEAM RANGE
    vb_offset = offsets[8]
    vb_range = int.from_bytes(all_data[vb_offset+4:vb_offset+8], byteorder = 'little') # in millimeter

    # GPS Data
    GPS_offsets = offsets[9:13]
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

    delta_times_double = [struct.unpack('d', b)[0] for b in delta_times_bytes] # convert to double

if __name__ == '__main__':
    main()