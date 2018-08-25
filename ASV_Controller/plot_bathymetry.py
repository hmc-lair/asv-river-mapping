import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import numpy as np
import utm
import gdal

from read_asv_data import read_ensemble

BEAM_ANGLE = 20 #degrees
TRANSDUCER_OFFSET = 0.1 #m

# Last Bakersfield deployment
river_lawnmower = 'Log/river_8-21/ALL_18-07-12 14.12.07.bin' # 5MB
river_transect_long = 'Log/river_8-21/ALL_18-07-12 15.03.46.bin' #9.1MB
river_longlawnmower = 'Log/river_8-21/ALL_18-07-12 15.41.49.bin' #10MB
river_transect = 'Log/river_8-21/ALL_18-07-12 14.51.23.bin'
river_idk = 'Log/river_8-21/ALL_18-07-12 14.30.01.bin'

lake1 = 'Log/lake_7-27/ALL_18-07-12 06.49.05.bin'

########################################


data_file = river_transect_long

#Mission files
mission_file = 'Missions/river_transect_long.csv'

# To crop GEOTIFF use:
# gdal_translate -srcwin 3000 9000 4000 3000 input.tif output.tif
map_file = '../Maps/river_8-13.tif'
# map_file = '../Maps/cast.tif'
# map_file = '../Maps/lake_7-27.tif'

# Plot parameters
CELL_RES = 1 #cell resolution

# Kalman filter parameters
win = 5 #window size
sigma_slope = 0.1204
sigma_offset = 0.6142

###############################################################################
# Map Helper Functions
###############################################################################

def load_map(filename):
    dataset = gdal.Open(filename)
    data = dataset.ReadAsArray()
    _, height, width = data.shape
    geo_trans = dataset.GetGeoTransform()
    inv_trans = gdal.InvGeoTransform(geo_trans)

    img = mpimg.imread(filename)
    return img, inv_trans, geo_trans, width, height

###############################################################################
# GPS/ADCP File Processing
###############################################################################

# Returns pixel coordinates from GPS data
def read_data_file(filename, inv_trans):
    f = open(data_file, 'rb')
    all_data = b''
    
    for line in f.readlines():
        all_data += line

    split_data = all_data.split(b'###')
    #GPS_data = list(filter(lambda x: x.split(b',')[0] == b'$GPS', split_data[:-1]))
    
    ADCP_data = list(filter(lambda x: x.split(b',')[0] == b'$ADCP', split_data[:-1]))
    state_data = list(filter(lambda x: x.split(b',')[0] == b'$STATE', split_data[:-1]))

    #Control data
    ctrl_data = list(filter(lambda x: x.split(b',')[0] == b'$CTRL', split_data[:-1]))
    ctrl_data_split = [state.split(b',') for state in ctrl_data]
    all_wps = [(float(state[-1]), float(state[-2])) for state in ctrl_data_split]
    wp_nor = []
    wp_eas = []
    for x, y in all_wps:
        wp_nor.append(x)
        wp_eas.append(y)
    wp_nor = wp_nor[1:] #removing initial 1.0 values
    wp_eas = wp_eas[1:] 
    
    #GPS data
    state_data_split = [state.split(b',') for state in state_data]
    all_states = [(float(state[1]), float(state[2])) for state in state_data_split]
    ASV_nor = []
    ASV_eas = []
    for x, y in all_states:
        ASV_nor.append(x)
        ASV_eas.append(y)

    #Water depths
    depths = read_ADCP_file(ADCP_data)

    return np.asarray(ASV_nor), np.asarray(ASV_eas), depths, np.asarray(wp_nor), np.asarray(wp_eas)

def read_ADCP_file(data):
    depths = []
    for ensemble in data:
        ensemble_data = ensemble[6:] #Remove '$ADCP,'
        result = read_ensemble(ensemble_data)
        vals = [float(d)*np.cos(BEAM_ANGLE*np.pi/180.) for d in result[6]]
        depths.append(np.average(vals) + TRANSDUCER_OFFSET)
    return -np.asarray(depths)

###############################################################################
# Mission File Processing
###############################################################################

def read_mission_file(filename, inv_trans):
    coords = []
    with open(filename, 'r') as f:
        for line in f.readlines():
            if line[-1] == '\n':
                line = line[:-1]
            coords.append(list(map(float,line.split(','))))
    X_all = []
    Y_all = []
    R = []
    C = []
    for lat,lon in coords:
        x, y, _,_ = utm.from_latlon(lat, lon)
        row,col = gdal.ApplyGeoTransform(inv_trans, x, y)
        X_all.append(x)
        Y_all.append(y)
        R.append(row)
        C.append(col)
        
    return X_all, Y_all, R, C

###############################################################################

'''
This function isn't being used
'''
def get_track_errs(ASV_nor, ASV_eas, wp_nor, wp_eas, mission_nor, mission_eas):
    errs = []
    p1 = [mission_eas[0], mission_nor[0]] #p1 and p2 define current line we want to follow
    p2 = [mission_nor[1], mission_eas[1]]

    wps_nor = []
    wps_eas = []
    for i in range(len(ASV_nor)): #at each timestamp, get tracking error
        if wp_nor[i] != 1.0 and wp_nor[i] not in wps_nor:
            wps_nor.append(wp_nor[i])
        if wp_eas[i] != 1.0 and wp_eas[i] not in wps_eas:
            wps_eas.append(wp_eas[i])

    # #Cut off mess at beginning...
    # print('# pts: ', len(ASV_nor))
    # cut = 770
    # ASV_nor = ASV_nor[cut:cut+40]
    # ASV_eas = ASV_eas[cut:cut+40]
    # Z = Z[cut:cut+40]
    # print('# pts (cut): ', len(ASV_nor))

    # # get dist errors for first few points...
    # avg = 0
    # num = 40
    # for i in range(num):
    #     x = ASV_eas[i]
    #     y = ASV_nor[i]
    #     d = np.sqrt((mission_nor[1]-mission_nor[0])**2 + (mission_eas[1]-mission_eas[0])**2)
    #     dist = np.abs((mission_nor[1]-mission_nor[0])*x - (mission_eas[1]-mission_eas[0])*y + mission_eas[1]*mission_nor[0] - mission_nor[1]*mission_eas[0])/d
    #     print('Dist error: ', dist)
    #     avg += dist
    # print('Avg error: ', avg/num)
    return errs

#Read data from 1 trial
def main():
    img, inv_trans, geo_trans, MAP_WIDTH, MAP_HEIGHT = load_map(map_file)

    # mission wps
    mission_nor, mission_eas, mission_X, mission_Y = read_mission_file(mission_file, inv_trans)
    #GPS DATA
    ASV_nor, ASV_eas, Z, wp_nor, wp_eas = read_data_file(data_file, inv_trans)

    #This next line isn't being used, see MATLAB code for tracking error...
    tracking_errs = get_track_errs(ASV_nor, ASV_eas, wp_nor, wp_eas, mission_nor, mission_eas)

    if len(ASV_nor) != len(Z):
        print('Size mismatch!', len(ASV_nor), len(Z))
        ASV_nor = ASV_nor[:-1]
        ASV_eas = ASV_eas[:-1]

    min_depth = Z.min()

    #Normalize positions
    min_x = min(ASV_nor)
    min_y = min(ASV_eas)
    X = [v - min_x for v in ASV_nor]
    Y = [v - min_y for v in ASV_eas]
    max_x = max(X)
    max_y = max(Y)

    # Method 0: 1D Kalman Filter
    m = int(np.ceil(max_x/CELL_RES))
    n = int(np.ceil(max_y/CELL_RES))
    baseFloor = min_depth
    B = baseFloor*np.ones((m,n))
    Bvar = np.zeros((m,n))

    for t in range(len(ASV_nor)):
        cur_x = X[t]
        cur_y = Y[t]
        cur_alt = Z[t]

        i = int(np.floor(cur_x/CELL_RES))
        j = int(np.floor(cur_y/CELL_RES))

        for k in range(max(0,i-win), min(m, i+win)):
            for l in range(max(0,j-win), min(n,j+win)):
                dist2 = 0.1+CELL_RES*((k-i)**2+(l-j)**2)**0.5
                    
                if B[k][l] == baseFloor:
                    B[k][l] = cur_alt
                    Bvar[k][l] = (dist2*sigma_slope+sigma_offset)**2
                else:
                    var = (dist2*sigma_slope+sigma_offset)**2
                    cur_K = float(Bvar[k][l])/(Bvar[k][l] + var)
                    B[k][l] = B[k][l]+cur_K*(cur_alt - B[k][l])
                    Bvar[k][l] = Bvar[k][l]-cur_K*Bvar[k][l];

    ###########################################################################
    # PLOTS
    # 1) Map w/ ASV path
    imgplot = plt.imshow(img)
    X_pix = []
    Y_pix = []
    for i in range(len(ASV_nor)): #translate from UTM to pixel coordinates
        row,col = gdal.ApplyGeoTransform(inv_trans, ASV_nor[i], ASV_eas[i])
        X_pix.append(row)
        Y_pix.append(col)
    plt.plot(X_pix, Y_pix, color='black', zorder=2, label='GPS readings')
    plt.plot(mission_X, mission_Y, color='red', marker='.', label='Mission plan')
    plt.legend()

    X_plot, Y_plot = np.meshgrid(np.arange(min_y, min_y + n*CELL_RES, CELL_RES), np.arange(min_x, min_x + m*CELL_RES, CELL_RES))
    
    # 2) Depth surface map
    ax1 = plt.figure(figsize=(8,6)).gca(projection='3d')
    ax1.plot_surface(X_plot, Y_plot, B, cmap=cm.viridis) #depths
    ax1.plot(ASV_eas, ASV_nor, np.zeros(len(X)), color='red') #ASV path
    ax1.view_init(200, -50)
    ax1.set_zlabel('Depth (m)')
    ax1.invert_zaxis()
    ax1.set_xlabel('Easting (m)')
    ax1.set_ylabel('Northing (m)')

    # 3) Scatter plot of raw data
    ax2 = plt.figure(figsize=(8,6)).gca(projection='3d')

    ax2.scatter(ASV_eas, ASV_nor, Z, c=Z, cmap=cm.viridis)
    ax2.plot(ASV_eas, ASV_nor, np.zeros(len(X)), color='red') #ASV path
    ax2.view_init(200, -50)
    ax2.set_zlabel('Depth (m)')
    ax2.invert_zaxis()
    ax2.set_xlabel('Easting (m)')
    ax2.set_ylabel('Northing (m)')

    # 4) Gradient plot
    # Gx, Gy = np.gradient(B) # gradients with respect to x and y
    # G = (Gx**2+Gy**2)**.5  # gradient magnitude
    
    # for i in range(len(G)):
    #     for j in range(len(G[0])):
    #         if G[i][j] > .4:
    #             G[i][j] = 0
    #         G[i][j] *= 4

    # N = G/G.max()  # normalize 0..1

    # ax1 = plt.figure(figsize=(8,6)).gca(projection='3d')
    # ax1.plot_surface(X_plot, Y_plot, B, facecolors=cm.viridis(N))
    # ax1.plot(ASV_eas, ASV_nor, np.zeros(len(X)), color='red') #ASV path
    # ax1.view_init(200, -50)
    # ax1.set_zlabel('Depth (m)')
    # ax1.invert_zaxis()
    # ax1.set_xlabel('Easting (m)')
    # ax1.set_ylabel('Northing (m)')

    plt.show()

if __name__ == '__main__':
    main()