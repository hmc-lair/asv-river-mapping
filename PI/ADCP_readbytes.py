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
    fixed_offset = offsets[0]

    num_beams = all_data[fixed_offset+8]
    num_cells = all_data[fixed_offset+9]
    pings_per_ensemble = int.from_bytes(all_data[fixed_offset+10: fixed_offset+12], byteorder='little')
    depth_cell_length = int.from_bytes(all_data[fixed_offset+12: fixed_offset+14], byteorder='little')
    print('Num beams: ', num_beams)
    print('Num cells: ', num_cells)

    # VARIABLE LEADER
    variable_offset = offsets[1]

    transducer_depth = int.from_bytes(all_data[variable_offset+16: variable_offset+18], byteorder='little')*0.1 #1 dm
    heading = int.from_bytes(all_data[variable_offset+18: variable_offset+20], byteorder='little')*0.01 #0 to 359.99
    pitch = int.from_bytes(all_data[variable_offset+20: variable_offset+22], byteorder='little', signed=True)*0.01 #-20 to 20
    roll = int.from_bytes(all_data[variable_offset+22: variable_offset+24], byteorder='little', signed=True)*0.01 #-20 to 20
    salinity = int.from_bytes(all_data[variable_offset+24: variable_offset+26], byteorder='little') #0 to 40 part per thousand
    temperature = int.from_bytes(all_data[variable_offset+26: variable_offset+28], byteorder='little')*0.01 #-5 to 40 degrees
    print('Transducer depth: ', transducer_depth)
    print('Heading, pitch, roll: ', heading, pitch, roll)
    print('Salinity, temp: ', salinity, temperature)

    # VELOCITY PROFILE
    velocity_profile_offset = offsets[2]
    relative_velocities = []

    for i in range(num_cells):
        start_offset = velocity_profile_offset + 2 + 2*i
        # Average over beams
        vel = 0
        for j in range(num_beams):
            vel += int.from_bytes(all_data[start_offset + 2*j: start_offset + 2 + 2*j], byteorder='little', signed=True)
        vel = vel/float(num_beams)
        relative_velocities.append(vel)
    print('Depth cell relative velocities: ', relative_velocities)

    # BOTTOM TRACK

    # VERTICAL BEAM RANGE



if __name__ == '__main__':
    main()