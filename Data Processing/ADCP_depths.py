from ADCP_readbytes import *

ADCP_file = '../PI/Log/ADCP_18-07-02 17.27.07.bin'

def main():
    f = open('depths.txt', 'w')
    f_times = open('timestamps.txt', 'w')
    depths = []

    # Read file
    data = read_ADCP_file(ADCP_file)
    cur_offset = 0
    ensemble_num = 1
    while cur_offset < len(data):
        cur_offset, timestamp, bt_ranges = read_ensemble(data, cur_offset)
        ensemble_num += 1
        d = sum(bt_ranges)/4.
        f.write(str(d) + ',')
        f_times.write(str(int(float(timestamp))) + ',')
        depths.append(d)

    f.close()
    f_times.close()

if __name__ == '__main__':
    main()