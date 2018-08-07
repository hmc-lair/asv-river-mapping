import numpy as np
import matplotlib.pyplot as plt
import math

# data_file = "Log/DATA_18-07-12 06.47.41.csv" #River
data_file = "Log/DATA_18-07-12 06.49.05.csv" # Lake

def main():
    # Load data (see load_and_save.py)

    all_data = np.load(data_file)
    ASV_X = all_data[0].astype(float)
    ASV_Y = all_data[1].astype(float)
    Z = all_data[2].astype(float)
    depth_cell_length = all_data[3]
    relative_velocities = all_data[4]
    bt_velocties = all_data[5]

    v_errors_006m = [] # error for cell located vertically 0.06 apart
    
    max_z_distance = 8
    z_error_interval = 1
    z_error_range = np.arange(1, max_z_distance, z_error_interval) # 7 meters, spread 1 m apart
    z_errors = [[] for i in range(len(z_error_range))] 

    max_v_distance = 5
    v_error_interval = 0.06
    v_error_range = np.arange(0, max_v_distance, v_error_interval)
    v_errors = [[] for i in range(len(v_error_range))]

    error_pairs_1m = [] # error pairings for 1m
    # Find all readings that are within 1m of each other
    for i in range(len(ASV_X)):
        for j in range(i, len(ASV_X)):
            p1_x = ASV_X[i]
            p1_y = ASV_Y[i]

            p2_x = ASV_X[j]
            p2_y = ASV_Y[j]

            dist = np.sqrt((p1_x - p2_x)**2 + (p1_y - p2_y)**2)
            distance_range = math.ceil(dist/z_error_interval)
            if distance_range < max_z_distance:
                z_errors[distance_range-1].append(Z[i] - Z[j])

            for k in range(len(relative_velocities[i])):
                for l in range(k + 1, len(relative_velocities[j])):
                    current_depth = (k+1) * depth_cell_length[i] * 0.01
                    next_depth = (l+1) * depth_cell_length[i] * 0.01

                    p1_z = current_depth
                    p2_z = next_depth

                    dist3 = np.sqrt((p1_x - p2_x)**2 + (p1_y - p2_y)**2 + (p1_z - p2_z)**2)
                    distance3_range = math.ceil(dist3/v_error_interval)
                    # print(distance3_range)
                    if distance3_range < len(v_error_range):
                        v1 = np.asarray(relative_velocities[i][k][:3])*0.001
                        v2 = np.asarray(relative_velocities[j][l][:3])*0.001
                        mask1 = np.logical_and(v1 < 10, v1 > -10)
                        mask2 = np.logical_and(v2 < 10, v2 > -10)
                        v_errors[distance3_range-1].append(np.linalg.norm(v1[mask1]) - np.linalg.norm(v2[mask2]))


    z_vars = [np.var(x) for x in z_errors]
    v_vars = [np.var(x) for x in v_errors]
    # print(v_errors)

    plt.hist(z_errors[0], bins='auto')
    plt.title("Depth error for cell " + str(1) + "m apart; " + "var = " + str(z_vars[0]))
    plt.xlabel('errors (m)')
    plt.ylabel('frequency')

    plt.figure()
    plt.hist(z_errors[1], bins='auto')
    plt.title("Depth error for cell " + str(2) + "m apart"  + "var = " + str(z_vars[1]))
    plt.xlabel('errors (m)')
    plt.ylabel('frequency')

    plt.figure()
    plt.hist(z_errors[2], bins='auto')
    plt.title("Depth error for cell " + str(3) + "m apart" + "var = " + str(z_vars[2]))
    plt.xlabel('errors (m)')
    plt.ylabel('frequency')

    plt.figure()
    plt.hist(z_errors[3], bins='auto')
    plt.title("Depth error for cell " + str(4) + "m apart" + "var = " + str(z_vars[3]))
    plt.xlabel('errors (m)')
    plt.ylabel('frequency')

    plt.figure()
    plt.hist(v_errors[0], bins='auto')
    plt.title("Velocity error for cell " + str(1) + "m apart; " + "var = " + str(v_vars[0]))
    plt.xlabel('errors (m)')
    plt.ylabel('frequency')

    depth_co = np.polyfit(z_error_range, z_vars, 1)
    print(v_vars)
    v_co = np.polyfit(v_error_range[:-1], v_vars[:-1], 1)

    plt.figure()
    plt.plot(z_error_range, z_vars, '-o')
    plt.plot(z_error_range, z_error_range*depth_co[0] + depth_co[1])
    plt.title('Distances and Depth Variance; Linear fit: ' + str(depth_co))
    plt.xlabel('cell distances (m)')
    plt.ylabel('variances')

    plt.figure()
    plt.plot(v_error_range, v_vars, '-o')
    plt.plot(v_error_range[:-1], v_error_range[:-1]*v_co[0] + v_co[1])
    plt.title('Distances and Velocities Variance; Linear fit: ' + str(v_co))
    plt.xlabel('cell distances (m)')
    plt.ylabel('variances')

    plt.show()
    

    



if __name__ == "__main__":
    main()