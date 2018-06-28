import binascii

def main():
    #read file
    with open('results.bin','rb') as f:
        result = b''
        for line in f.readlines():
            data = line
            result += data 
    print(result)
    
    #num data types
    print(result[5])
    num_types = result[5]
    
    #OFFSETS
    offsets = []
    for i in range(num_types):
        offset = result[6+2*i:8+2*i]
        offset_int = int.from_bytes(offset, byteorder='little')
        offsets.append(offset_int)
    print('Offsets: ', offsets)
    
    #FIXED LEADER DATA
    print([result[x:x+2] for x in offsets])
    #fixed_leader_offset = offsets[0]
    #fixed_leader_header = result[fixed_leader_offset:fixed_leader_offset+2]
    #print('Header: ', fixed_leader_header)
    #print(int.from_bytes(header, byteorder='little'))
main()
