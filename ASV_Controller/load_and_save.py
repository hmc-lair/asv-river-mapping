import sys
import numpy as np
from read_asv_data import read_ensemble

BEAM_ANGLE = 20 #degrees
TRANSDUCER_OFFSET = 0.1 #m

def read_ADCP_file(data):
    depths = []
    depth_cell_length = []
    relative_velocities = []
    bt_velocties = []

    for ensemble in data:
        ensemble_data = ensemble[6:] #Remove '$ADCP,'
        result = read_ensemble(ensemble_data)

        # Depth data
        vals = [float(d)*np.cos(BEAM_ANGLE*np.pi/180.) for d in result[6]]

        # Velocities Data
        cur_depth_cell_length = result[4]
        cur_relative_velocities = result[5]
        cur_bt_velocties = result[7]

        depths.append(np.average(vals) + TRANSDUCER_OFFSET)
        depth_cell_length.append(cur_depth_cell_length)
        relative_velocities.append(cur_relative_velocities)
        bt_velocties.append(cur_bt_velocties)
    return -np.asarray(depths), np.asarray(depth_cell_length), np.asarray(relative_velocities), np.asarray(bt_velocties)


# Returns pixel coordinates from GPS data
def read_data_file(filename):
    f = open(filename, 'rb')
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
    depths, depth_cell_length, relative_velocities, bt_velocties = read_ADCP_file(ADCP_data)

    return np.asarray(ASV_X), np.asarray(ASV_Y), depths, depth_cell_length, relative_velocities,bt_velocties

def main(argv):
    data_file = argv
    name = argv.split('/')[2][4:-4]
    output_filename = "Log/DATA_" + name + ".csv"

    print("Reading data from ... " + argv)
    print("Outputing data to ... " + output_filename)

    ASV_X, ASV_Y, Z, depth_cell_length, relative_velocities, bt_velocities = read_data_file(data_file)
    bt_velocities = np.array(bt_velocities, dtype =object)

    bt_vel = [list(x) for x in bt_velocities]
    bt_vel[0].append(0)
    bt_vel = np.asarray(bt_vel)
    bt_vel[0] = list(bt_velocities[0])

    output_array = np.array([ASV_X, ASV_Y, Z, depth_cell_length,relative_velocities, bt_vel])
    output_array.dump(output_filename)

    X = np.load(output_filename)



if __name__ == "__main__":
	main(sys.argv[1])