import matplotlib.pyplot as plt
import matplotlib
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import numpy as np
import utm
import gdal

from read_asv_data import read_ensemble


# Plot parameters
CELL_RES = 0.5
win = 1
sigma_slope = 0.1204
sigma_offset = 0.6142

BEAM_ANGLE = 20 #degrees
TRANSDUCER_OFFSET = 0.1 #m

data_file = "Log/lake_7-27/ALL_18-07-12 06.49.05.bin" # Lake
# data_file = "Log/river_7-27/ALL_18-07-12 06.47.41.bin" # River


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
    return -np.asarray(depths), depth_cell_length, np.asarray(relative_velocities), np.asarray(bt_velocties)

# Returns pixel coordinates from GPS data
def read_data_file(filename):
    f = open(data_file, 'rb')
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

#Read data from 1 trial
def main():
    # TODO: Learn how to load maps
    # img, inv_trans, geo_trans, MAP_WIDTH, MAP_HEIGHT = load_map(map_file)
    #GPS DATA

    print("Reading data...")
    # Note: Velocities are in millimeters per second
    ASV_X, ASV_Y, Z, depth_cell_length, relative_velocities, bt_velocties = read_data_file(data_file)


    if len(ASV_X) != len(Z):
        print('Size mismatch!', len(ASV_X), len(Z))
        ASV_X = ASV_X[:-1]
        ASV_Y = ASV_Y[:-1]

    min_depth = Z.min()

    # #Normalize positions
    # min_x = min(ASV_X)
    # min_y = min(ASV_Y)
    # X = [v - min_x for v in ASV_X]
    # Y = [v - min_y for v in ASV_Y]
    # max_x = max(X)
    # max_y = max(Y)

    # Method 0: 1D Kalman Filter
    # m = int(np.ceil(max_x/CELL_RES))
    # n = int(np.ceil(max_y/CELL_RES))
    # baseFloor = min_depth
    # B = baseFloor*np.ones((m,n))
    # Bvar = np.zeros((m,n))

    # for t in range(len(ASV_X)):
    #     cur_x = X[t]
    #     cur_y = Y[t]
    #     cur_alt = Z[t]

    #     i = int(np.floor(cur_x/CELL_RES))
    #     j = int(np.floor(cur_y/CELL_RES))

    #     for k in range(max(0,i-win), min(m, i+win)):
    #         for l in range(max(0,j-win), min(n,j+win)):
    #             dist2 = 0.1+CELL_RES*((k-i)**2+(l-j)**2)**0.5
                    
    #             if B[k][l] == baseFloor:
    #                 B[k][l] = cur_alt
    #                 Bvar[k][l] = (dist2*sigma_slope+sigma_offset)**2
    #             else:
    #                 var = (dist2*sigma_slope+sigma_offset)**2
    #                 cur_K = float(Bvar[k][l])/(Bvar[k][l] + var)
    #                 B[k][l] = B[k][l]+cur_K*(cur_alt - B[k][l])
    #                 Bvar[k][l] = Bvar[k][l]-cur_K*Bvar[k][l];

    ###########################################################################
    # PLOTS
    # 1) Map w/ ASV path
    # imgplot = plt.imshow(img)
    # X_pix = []
    # Y_pix = []
    # for i in range(len(ASV_X)): #translate from UTM to pixel coordinates
    #     row,col = gdal.ApplyGeoTransform(inv_trans, ASV_X[i], ASV_Y[i])
    #     X_pix.append(row)
    #     Y_pix.append(col)
    # plt.plot(X_pix, Y_pix, color='black', zorder=2, label='GPS readings')

    # #mission wps
    # mission_X, mission_Y = read_mission_file(mission_file, inv_trans)
    # plt.plot(mission_X, mission_Y, color='red', marker='.', label='Mission plan')
    # plt.legend()

    # 2) Depth surface map
    # B_new = np.zeros((n,m))
    # for i in range(n):
    #     for j in range(m):
    #         B_new[i][j] = B[j][i]

    # X_plot, Y_plot = np.meshgrid(np.arange(min_x, min_x + m), np.arange(min_y, min_y + n))
    # ax1 = plt.figure(figsize=(8,6)).gca(projection='3d')
    # surf = ax1.plot_surface(X_plot, Y_plot, B_new, cmap=cm.viridis) #depths

    # p = ax1.plot(ASV_X, ASV_Y, np.zeros(len(X)), color='red') #ASV path
    # ax1.view_init(200, -50)
    # ax1.set_zlabel('Depth (m)')
    # ax1.invert_zaxis()
    # ax1.set_xlabel('Easting (m)')
    # ax1.set_ylabel('Northing (m)')


    # 3) Velocity Surface Map
    # bad_bt_data = [] # flags to signify bag data
    # bad_rel_data = []
    # for i in range(len(relative_velocities)):
    #     # if robot is moving at greater than 10 meters/s 
    #     if np.linalg.norm(bt_velocties[i]) > 10000:
    #         bad_bt_data.append(True)

    #     else:
    #         bad_bt_data.append(False)
    #     bad_rel_data.append([])
    #     for j in range(len(relative_velocities[i])):
    #         if np.linalg.norm(relative_velocities[i][j]) > 30000:
    #             bad_rel_data[i].append(True)
    #         else:
    #             bad_rel_data[i].append(False)
    # print(bad_rel_data)

    ### RAW Surface Velocities
    # Relative Surface Velocities
    v_sur_rel = [vel[0] for vel in relative_velocities]
    # bad_sur_rel = [bad[0] for bad in bad_rel_data]
    vx_sur_rel = np.asarray([v[0] for v in v_sur_rel])
    vy_sur_rel = np.asarray([v[1] for v in v_sur_rel])

    # Boat Velocities 
    v_boat = bt_velocties
    vx_boat = np.asarray([v[0] for v in v_boat])
    vy_boat = np.asarray([v[1] for v in v_boat])

    # Absolute Velocities
    vx_sur_abs = vx_sur_rel - vx_boat
    vy_sur_abs = vy_sur_rel - vy_boat


    ### Kalman Filtered Surface Velocities
    
    #Normalize positions
    min_x = min(ASV_X)
    min_y = min(ASV_Y)
    X = [v - min_x for v in ASV_X]
    Y = [v - min_y for v in ASV_Y]
    max_x = max(X)
    max_y = max(Y)

    min_current = 0
    # 1D Kalman Filter
    m = int(np.ceil(max_x/CELL_RES)) # Define number of cells
    n = int(np.ceil(max_y/CELL_RES))
    vx_current_min = 0
    vy_current_min = 0
    B_vx = vx_current_min*np.ones((m,n)) # initiliaze current velocities for each cell
    B_vy = vy_current_min*np.ones((m,n))
    B_vx_var = np.zeros((m,n))
    B_vy_var = np.zeros((m,n))
    win = 2
    # CELL_RES = 0.1

    for t in range(len(ASV_X)):
        # print("Progress: %d out of %d " %(t, len(ASV_X)))
        cur_x = X[t]
        cur_y = Y[t]
        cur_vx = vx_sur_abs[t]
        cur_vy = vy_sur_abs[t]

        i = int(np.floor(cur_x/CELL_RES)) # find which cell we're in
        j = int(np.floor(cur_y/CELL_RES))

        # loop over a defined window
        for k in range(max(0,i-win), min(m, i+win)):
            for l in range(max(0,j-win), min(n,j+win)):
                # find from current cell to that cell
                dist2 = 0.1+CELL_RES*((k-i)**2+(l-j)**2)**0.5
                
                # if this is first time updating this cell...
                # set the cell value to current readings
                if B_vx[k][l] == vx_current_min:
                    if abs(cur_vx) > 1000:
                        continue
                    elif abs(cur_vy) > 1000:
                        continue
                        B_vx[k][l] = cur_vx
                        B_vy[k][l] = cur_vy

                        # Calcualte error (related to distance)
                        B_vx_var[k][l] = (dist2*sigma_slope+sigma_offset)**2
                        B_vy_var[k][l] = (dist2*sigma_slope+sigma_offset)**2
                else:
                    if abs(cur_vx) > 1000:
                        continue
                    elif abs(cur_vy) > 1000:
                        continue
                    else:
                        # Current Error for the cell
                        var = (dist2*sigma_slope+sigma_offset)**2
                        cur_Kx = float(B_vx_var[k][l])/(B_vx_var[k][l] + var)
                        cur_Ky = float(B_vy_var[k][l])/(B_vy_var[k][l] + var)

                        B_vx[k][l] = B_vx[k][l]+cur_Kx*(cur_vx - B_vx[k][l])
                        B_vy[k][l] = B_vy[k][l]+cur_Ky*(cur_vy - B_vy[k][l])
                        B_vx_var[k][l] = B_vx_var[k][l]-cur_Kx*B_vx_var[k][l]
                        B_vy_var[k][l] = B_vy_var[k][l]-cur_Ky*B_vy_var[k][l]

    X_plot, Y_plot = np.meshgrid(np.arange(min_x, min_x + m*CELL_RES, CELL_RES), np.arange(min_y, min_y + n*CELL_RES, CELL_RES))
    
    # Flip
    B_vx_new = np.zeros((n,m))
    B_vy_new = np.zeros((n,m))
    for i in range(n):
        for j in range(m):
            B_vx_new[i][j] = B_vx[j][i]
            B_vy_new[i][j] = B_vy[j][i]
    # print(X_plot)
    # Plotting
    # plt.figure()
    # print(max(B_vx), min(B_vy))
    # print(B_vx[1,:])

    norm = matplotlib.colors.Normalize()
    B_speed = np.sqrt(B_vx_new**2 + B_vy_new**2)
    norm.autoscale(B_speed)
    cm = matplotlib.cm.copper
    sm = matplotlib.cm.ScalarMappable(cmap=cm, norm=norm)
    sm.set_array([])

    plt.quiver(X_plot, Y_plot, B_vx_new, B_vy_new, color=cm(norm(B_speed)))
    plt.colorbar(sm)
    # plt.title('Relative Surface Velocities')

    # plt.figure()
    # plt.quiver(ASV_X, ASV_Y, vx_boat, vy_boat)
    # plt.title('Boat Velocities')

    # plt.figure()
    # Q = plt.quiver(X_plot, Y_plot, B_vx, vy_sur_rel - vy_boat, pivot='tip', scale=1000)
    # qk = plt.quiverkey(Q, 0.9, 0.9, 1, r'$1 \frac{m}{s}$', labelpos='E',
    #                coordinates='figure')
    # plt.scatter(ASV_X, ASV_Y, color='k', s=5)
    plt.title('Absolute Velocities')
    plt.show()

if __name__ == '__main__':
    main()