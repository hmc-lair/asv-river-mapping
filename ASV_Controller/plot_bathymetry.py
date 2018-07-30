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

#ASV log files
millikan1 = 'Log/millikan_7-19/ALL_18-07-11 23.59.36.bin' #1) GOOD MAP OF MILLIKAN
millikan2 = 'Log/millikan_7-19/ALL_18-07-12 00.16.04.bin' #2) Good
millikan3 = 'Log/millikan_7-19/ALL_18-07-11 23.36.33.bin' #3) Good
millikan4 = 'Log/millikan_7-19/ALL_18-07-12 00.24.24.bin' #4) Large range

lake1 = 'Log/lake_7-27/ALL_18-07-12 06.49.05.bin'

river1 = 'Log/river_7-27/ALL_18-07-12 06.47.41.bin'
river2 = 'Log/river_7-27/ALL_18-07-12 06.26.13.bin'

data_file = river1

#Mission files
#TODO: add mission file
mission_file = 'Missions/river_last.csv'

# To crop GEOTIFF use:
# gdal_translate -srcwin 3000 9000 4000 3000 input.tif output.tif
map_file = '../Maps/output.tif'

# Plot parameters
CELL_RES = 0.5
win = 5
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
    
    #GPS data
    state_data_split = [state.split(b',') for state in state_data]
    all_states = [(float(state[1]), float(state[2])) for state in state_data_split]
    ASV_X = []
    ASV_Y = []
    for x, y in all_states:
        ASV_X.append(x)
        ASV_Y.append(y)

    #Water depths
    depths = read_ADCP_file(ADCP_data)

    return np.asarray(ASV_X), np.asarray(ASV_Y), depths

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
    X = []
    Y = []
    for lat,lon in coords:
        x, y, _,_ = utm.from_latlon(lat, lon)
        row,col = gdal.ApplyGeoTransform(inv_trans, x, y)
        X.append(row)
        Y.append(col)
        
    return X, Y


###############################################################################

#Read data from 1 trial
def main():
    img, inv_trans, geo_trans, MAP_WIDTH, MAP_HEIGHT = load_map(map_file)

    #GPS DATA
    ASV_X, ASV_Y, Z = read_data_file(data_file, inv_trans)

    if len(ASV_X) != len(Z):
        print('Size mismatch!', len(ASV_X), len(Z))
        ASV_X = ASV_X[:-1]
        ASV_Y = ASV_Y[:-1]

    min_depth = Z.min()

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
    for i in range(len(ASV_X)): #translate from UTM to pixel coordinates
        row,col = gdal.ApplyGeoTransform(inv_trans, ASV_X[i], ASV_Y[i])
        X_pix.append(row)
        Y_pix.append(col)
    plt.plot(X_pix, Y_pix, color='black', zorder=2, label='GPS readings')

    #mission wps
    mission_X, mission_Y = read_mission_file(mission_file, inv_trans)
    plt.plot(mission_X, mission_Y, color='red', marker='.', label='Mission plan')
    plt.legend()

    # 2) Depth surface map
    B_new = np.zeros((n,m))
    for i in range(n):
        for j in range(m):
            B_new[i][j] = B[j][i]

    X_plot, Y_plot = np.meshgrid(np.arange(min_x, min_x + m*CELL_RES, CELL_RES), np.arange(min_y, min_y + n*CELL_RES, CELL_RES))
    ax1 = plt.figure(figsize=(8,6)).gca(projection='3d')
    surf = ax1.plot_surface(X_plot, Y_plot, B_new, cmap=cm.viridis) #depths

    p = ax1.plot(ASV_X, ASV_Y, np.zeros(len(X)), color='red') #ASV path
    ax1.view_init(200, -50)
    ax1.set_zlabel('Depth (m)')
    ax1.invert_zaxis()
    ax1.set_xlabel('Easting (m)')
    ax1.set_ylabel('Northing (m)')
    plt.show()

if __name__ == '__main__':
    main()