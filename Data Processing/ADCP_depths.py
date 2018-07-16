from ADCP_readbytes_jane import *

ADCP_file = '../PI/Log/ADCP_18-07-02 17.27.07.bin'

def main():
    f = open('depths_all.txt', 'w')
    f_headings = open('headings.txt', 'w')
    f_times = open('timestamps.txt', 'w')

    # Read file
    data = read_ADCP_file(ADCP_file)
    cur_offset = 0
    ensemble_num = 1

    while cur_offset < len(data):
        cur_offset, timestamp, heading, bt_ranges = read_ensemble(data, cur_offset)
        ensemble_num += 1
        f_headings.write(str(heading) + ',')
        for d in bt_ranges:
            f.write(str(d) + ',')
        f.write('\n')
        #f_times.write(str(int(float(timestamp))) + ',')

    f.close()
    f_headings.close()
    f_times.close()

if __name__ == '__main__':
    main()