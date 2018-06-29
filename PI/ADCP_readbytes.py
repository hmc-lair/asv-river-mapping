def main():
    # READ FILE
    with open('data/test_results.bin','rb') as f:
        all_data = b''
        for line in f.readlines():
            data = line
            all_data += data 
    print(all_data)

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
    print('Offsets: ', offsets)
    print('Data IDs: ', [all_data[x:x+2] for x in offsets])

    # VARIABLE LEADER

    # VELOCITY PROFILE

    # BOTTOM TRACK

    # VERTICAL BEAM RANGE



if __name__ == '__main__':
    main()