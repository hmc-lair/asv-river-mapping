import matplotlib.pyplot as plt
import matplotlib
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import numpy as np
import utm
import gdal

from read_asv_data import read_ensemble

## PLOTS
# Overhead velocity with color intensity
# Cross section velocity with color intensity
# 3D map with depth profile.... maybe? (need to fix the scale issue)

# Plot parameters
CELL_RES = 0.5
Z_CELL_RES = 0.06
sigma_slope = 2
sigma_offset = 0.6142
win = 5
z_win = 1
BEAM_ANGLE = 20 #degrees
TRANSDUCER_OFFSET = 0.1 #m

# data_file = "Log/lake_7-27/ALL_18-07-12 06.49.05.bin" # Lake
data_file = "Log/river_7-27/ALL_18-07-12 06.47.41.bin" # River
map_file = '../Maps/river_7-27.tif'

def load_map(filename):
    dataset = gdal.Open(filename)
    data = dataset.ReadAsArray()
    _, height, width = data.shape
    geo_trans = dataset.GetGeoTransform()
    inv_trans = gdal.InvGeoTransform(geo_trans)

    img = mpimg.imread(filename)
    return img, inv_trans, geo_trans, width, height

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
    img, inv_trans, geo_trans, MAP_WIDTH, MAP_HEIGHT = load_map(map_file)


    ########################## Reading Data ##########################
    print("Reading data...")
    # Note: Velocities are in millimeters per second
    ASV_X, ASV_Y, Z, depth_cell_length, relative_velocities, bt_velocties = read_data_file(data_file)


    if len(ASV_X) != len(Z):
        print('Size mismatch!', len(ASV_X), len(Z))
        ASV_X = ASV_X[:-1]
        ASV_Y = ASV_Y[:-1]

    ##################################### Depth Kalman Filter ########################################
    depth_win = 10
    min_depth = 0
    #Normalize positions
    min_x = min(ASV_X)
    min_y = min(ASV_Y)
    X = [v - min_x for v in ASV_X]
    Y = [v - min_y for v in ASV_Y]
    max_x = max(X)
    max_y = max(Y)

    # Method 0: 1D Kalman Filter
    m = int(np.ceil(max_x/CELL_RES))
    n = int(np.ceil(max_y/CELL_RES))
    baseFloor = min_depth
    B = baseFloor*np.ones((m,n))
    Bvar = np.zeros((m,n))

    for t in range(len(ASV_X)):
        cur_x = X[t]
        cur_y = Y[t]
        cur_alt = Z[t]

        i = int(np.floor(cur_x/CELL_RES))
        j = int(np.floor(cur_y/CELL_RES))

        for k in range(max(0,i-depth_win), min(m, i+depth_win)):
            for l in range(max(0,j-depth_win), min(n,j+depth_win)):
                dist2 = 0.1+CELL_RES*((k-i)**2+(l-j)**2)**0.5
                    
                if B[k][l] == baseFloor:
                    B[k][l] = cur_alt
                    Bvar[k][l] = (dist2*sigma_slope+sigma_offset)**2
                else:
                    var = (dist2*sigma_slope+sigma_offset)**2
                    cur_K = float(Bvar[k][l])/(Bvar[k][l] + var)
                    B[k][l] = B[k][l]+cur_K*(cur_alt - B[k][l])
                    Bvar[k][l] = Bvar[k][l]-cur_K*Bvar[k][l];
    B_new = np.zeros((n,m))
    for i in range(n):
        for j in range(m):
            B_new[i][j] = B[j][i]

    ##################################### Velocity Kalman Filter ########################################
    v_sur_rel = [vel[0] for vel in relative_velocities]
    vx_sur_rel = np.asarray([v[0] for v in v_sur_rel])
    vy_sur_rel = np.asarray([v[1] for v in v_sur_rel])

    # Boat Velocities 
    v_boat = bt_velocties
    vx_boat = np.asarray([v[0] for v in v_boat])
    vy_boat = np.asarray([v[1] for v in v_boat])

    # Absolute Velocities
    vx_sur_abs = (vx_sur_rel - vx_boat) * 0.001
    vy_sur_abs = (vy_sur_rel - vy_boat) * 0.001
    v_speed = np.sqrt(vx_sur_abs**2 + vy_sur_abs**2)
    print(max(v_speed))

    #################################  3D Kalman Filter #########################
    print("Starting Kalman Filtering")
    depth_cell_nums = [len(v) for v in relative_velocities]
    depths = []

    ## Find discrete depth values
    for i in range(len(relative_velocities)):
        depths.append([])
        for j in range(len(relative_velocities[i])):
            depths[i].append(j * depth_cell_length[i] * 0.01 + TRANSDUCER_OFFSET ) 
    depths = np.asarray(depths)
    
    # Set boundaries
    min_x = min(ASV_X)
    min_y = min(ASV_Y)
    min_z = TRANSDUCER_OFFSET

    X = [v - min_x for v in ASV_X]
    Y = [v - min_y for v in ASV_Y]

    max_x = max(X)
    max_y = max(Y)
    max_z = np.max(np.max(depths))

    min_current = 0
    l = int(np.ceil(max_z/Z_CELL_RES))
    m = int(np.ceil(max_x/CELL_RES)) # Define number of cells
    n = int(np.ceil(max_y/CELL_RES))

    vx_current_min = 0
    vy_current_min = 0
    vz_current_min = 0

    B_vx = vx_current_min*np.ones((m,n,l)) # initiliaze current velocities for each cell
    B_vy = vy_current_min*np.ones((m,n,l))
    B_vz = vz_current_min*np.ones((m,n,l))
    B_vx_var = np.zeros((m,n,l))
    B_vy_var = np.zeros((m,n,l))
    B_vz_var = np.zeros((m,n,l))
    
    ## 1D Kalman Filter for 3D current data
    for t in range(len(ASV_X)):
        for d in range(len(depths[t])):
            # print("Progress: %d out of %d " %(t, len(ASV_X)))
            max_depth = -np.max(depths[t])
            cur_x = X[t]
            cur_y = Y[t] 
            cur_z = depths[t][d]

            cur_vx = (relative_velocities[t][d][0] - vx_boat[t]) * 0.001
            cur_vy = (relative_velocities[t][d][1] - vy_boat[t]) * 0.001
            cur_vz = relative_velocities[t][d][2] * 0.001

            i = int(np.floor(cur_x/CELL_RES)) # find which cell we're in
            j = int(np.floor(cur_y/CELL_RES))
            k = int(np.floor(cur_z/Z_CELL_RES))
            # Do not compute if the readings are bad
            if abs(cur_vx) > 10 or abs(cur_vy) > 10 or abs(cur_vz) > 10:
                # Filter out bad data
                continue
            else:
                # loop over the 3d space in a defined window
                for a in range(max(0,i-win), min(m, i+win)):
                    for b in range(max(0,j-win), min(n,j+win)):
                        for c in range(max(0,k-z_win), min(l, k+z_win)):
                            if (c * Z_CELL_RES > max_depth):
                                # find from current cell to that cell
                                dist2 = 0.1+CELL_RES*((a-i)**2+(b-j)**2 + (c-k)**2 )**0.5
                                # if this is first time updating this cell...
                                # set the cell value to current readings
                                if B_vx[a,b,c] == vx_current_min:
                                    B_vx[a,b,c] = cur_vx
                                    B_vy[a,b,c] = cur_vy
                                    B_vz[a,b,c] = cur_vz
                                    
                                    # Calcualte error (related to distance)
                                    B_vx_var[a,b,c] = (dist2*sigma_slope+sigma_offset)**2
                                    B_vy_var[a,b,c] = (dist2*sigma_slope+sigma_offset)**2
                                    B_vz_var[a,b,c] = (dist2*sigma_slope+sigma_offset)**2
                                else:
                                    # Current Error for the cell
                                    var = (dist2*sigma_slope+sigma_offset)**2
                                    cur_Kx = float(B_vx_var[a,b,c])/(B_vx_var[a,b,c] + var)
                                    cur_Ky = float(B_vy_var[a,b,c])/(B_vy_var[a,b,c] + var)
                                    cur_Kz = float(B_vz_var[a,b,c])/(B_vz_var[a,b,c] + var)
                                    B_vx[a,b,c] = B_vx[a,b,c]+cur_Kx*(cur_vx - B_vx[a,b,c])
                                    B_vy[a,b,c] = B_vy[a,b,c]+cur_Ky*(cur_vy - B_vy[a,b,c])
                                    B_vz[a,b,c] = B_vz[a,b,c]+cur_Kz*(cur_vz - B_vz[a,b,c])

                                    B_vx_var[a,b,c] = B_vx_var[a,b,c]-cur_Kx*B_vx_var[a,b,c]
                                    B_vy_var[a,b,c] = B_vy_var[a,b,c]-cur_Ky*B_vy_var[a,b,c]
                                    B_vz_var[a,b,c] = B_vz_var[a,b,c]-cur_Kz*B_vz_var[a,b,c]
                                    
    # Flip
    B_vx_new = np.zeros((n,m,l))
    B_vy_new = np.zeros((n,m,l))
    B_vz_new = np.zeros((n,m,l))
    for i in range(n):
        for j in range(m):
            B_vx_new[i,j,:] = B_vx[j,i,:]
            B_vy_new[i,j,:] = B_vy[j,i,:]
            B_vz_new[i,j,:] = B_vz[j,i,:]

    y_axis = np.arange(min_y, min_y + n*CELL_RES, CELL_RES)
    x_axis = np.arange(min_x, min_x + m*CELL_RES, CELL_RES)
    z_axis = -np.arange(min_z, min_z + l*Z_CELL_RES, Z_CELL_RES)
    X_plot, Y_plot, Z_plot = np.meshgrid(x_axis, y_axis, z_axis)

    x_unfiltered = X_plot
    y_unfiltered = Y_plot
    # Filter out zero vectors
    mask = np.logical_or(B_vx_new != 0, B_vy_new != 0, B_vz_new != 0)
    X_plot = np.where(mask, X_plot, np.nan)
    Y_plot = np.where(mask, Y_plot, np.nan)
    Z_plot = np.where(mask, Z_plot, np.nan)
    B_vx_new = np.where(mask, B_vx_new, 0)
    B_vy_new = np.where(mask, B_vy_new, 0)
    B_vz_new = np.where(mask, B_vz_new, 0)
    B_speed = np.sqrt(B_vx_new**2 + B_vy_new**2)

    print(np.shape(mask), np.shape(X_plot), np.shape(Z_plot), np.shape(B_speed))
    skip = 3
    print("DONE!")

    ################################ Plotting Data ######################

    ####### Fig 1: Ugh Awful 3D Views
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    Q=plt.quiver(X_plot[::skip], Y_plot[::skip], Z_plot[::skip], B_vx_new[::skip], B_vy_new[::skip], B_vz_new[::skip], cmap="coolwarm", length = 0.08)
    Q.set_array(np.ravel(B_speed[0,:][::skip]))
    plt.colorbar(Q,extend='both')
    plt.axis('equal')
    plt.title("Filtered 3D Velocities")

    ####### Fig 2: Top Velocity View
    fig2 = plt.figure()
    C = plt.pcolormesh(X_plot[:,:,0], Y_plot[:,:,0], B_speed[:,:,0])
    plt.colorbar(extend ="both")
    Q=plt.quiver(X_plot[:,:,0], Y_plot[:,:,0], B_vx_new[:,:,0], B_vy_new[:,:,0])
    plt.axis('equal')
    plt.title("Filtered Surface Velocities")
    plt.xlabel("UTM Easting (meters)")
    plt.ylabel("UTM Northing (meters)")

    ####### Fig 3: Top Velocity View with Map
    fig3 = plt.figure()
    imgplot = plt.imshow(img)

    # Translate ASV path xy
    X_auv_pix = []
    Y_auv_pix = []
    for i in range(len(ASV_X)): #translate from UTM to pixel coordinates
        row,col = gdal.ApplyGeoTransform(inv_trans, ASV_X[i], ASV_Y[i])
        X_auv_pix.append(row)
        Y_auv_pix.append(col)
    plt.plot(X_auv_pix, Y_auv_pix, color='black', zorder=2, label='GPS readings')

    # Translate Velocity x, y
    x_surface = X_plot[:,:,0]
    y_surface = Y_plot[:,:,0]
    x_pix = np.ones(np.shape(x_surface))
    y_pix = np.ones(np.shape(x_surface))
    for i in range(len(x_surface)): # translate velocity xy into pixel o=coordinates
        for j in range(len(x_surface[0])):
            x_pix[i,j], y_pix[i,j] = gdal.ApplyGeoTransform(inv_trans, x_surface[i,j], y_surface[i,j])
    
    Q=plt.quiver(x_pix, y_pix, B_vx_new[:,:,0], B_vy_new[:,:,0], cmap="coolwarm")
    Q.set_array(np.ravel(B_speed[:,:,0]))
    plt.colorbar(Q,extend='both')
    plt.title("Filtered Surface Velocities with Map Underlay")
    plt.xlabel("UTM Easting (meters)")
    plt.ylabel("UTM Northing (meters)")

    ##### Fig 4: Top Raw Data
    fig4 = plt.figure()
    mask = v_speed <= 10 # filtered out speed higher than 10 meters per second
    X = ASV_X[mask]
    Y = ASV_Y[mask]
    vx_sur_abs = vx_sur_abs[mask]
    vy_sur_abs = vy_sur_abs[mask]
    v_speed = v_speed[mask]
    Q = plt.quiver(ASV_X, ASV_Y, vx_sur_abs, vy_sur_abs, cmap="coolwarm")
    Q.set_array(np.ravel(v_speed))
    plt.colorbar(Q,extend='both')
    plt.title('Raw Surface Velocities')
    plt.xlabel("UTM Easting (meters)")
    plt.ylabel("UTM Northing (meters)")

    ##### Fig 5: Depth Data
    fig5 = plt.figure()
    C = plt.pcolormesh(x_unfiltered[:,:,0],y_unfiltered[:,:,0], B_new, cmap=cm.viridis)
    plt.colorbar(C,extend='both')
    Q = plt.quiver(x_surface, y_surface, B_vx_new[:,:,0], B_vy_new[:,:,0], cmap="coolwarm")
    Q.set_array(np.ravel(B_speed[:,:,0]))
    plt.colorbar(Q,extend='both')
    plt.title('Filtered Velocities with Depth Underlay')
    plt.xlabel("UTM Easting (meters)")
    plt.ylabel("UTM Northing (meters)")
    plt.show()


if __name__ == '__main__':
    main()