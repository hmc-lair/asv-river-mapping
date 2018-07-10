import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from scipy.interpolate import griddata

import numpy as np
import utm
import gdal

depths_file = 'depths.txt'
timestamps_file = 'timestamps.txt'
GPS_file = '../PI/Log/GPS_18-07-02 17.27.07.txt'

# To crop GEOTIFF use:
# gdal_translate -srcwin 3000 9000 4000 3000 input.tif output.tif
map_file = '../Maps/kern_river.tif'

# Plot parameters
CELL_RES = 1
win = 5
sigma_slope = 0.1204
sigma_offset = 0.6142

###############################################################################
# Map Helper Functions
###############################################################################

def load_map(filename):
    dataset = gdal.Open(filename)
    data = dataset.ReadAsArray()
    geo_trans = dataset.GetGeoTransform()
    inv_trans = gdal.InvGeoTransform(geo_trans)

    img = mpimg.imread(filename)
    return img, inv_trans

###############################################################################
# GPS File Processing
###############################################################################

# Returns UTM coordinates from GPS data
def read_GPS_file(filename, inv_trans):
    all_data = []
    timestamps = []
    with open(filename, 'r') as f:
        for line in f.readlines():
            vals = line.split(',')
            if line[:6] == '$GPGGA' and len(vals) > 10:
                timestamps.append(int(float(vals[1])))
                lat_str,_,lon_str,_ = vals[2:6]
                lat = str_to_coord(lat_str)
                lon = -str_to_coord(lon_str) #W = negative, generalize later
                x, y, _, _ = utm.from_latlon(lat, lon)
                all_data.append((x, y))
    
    ASV_X = []
    ASV_Y = []
    for x, y in all_data:
        row,col = gdal.ApplyGeoTransform(inv_trans, x, y)
        ASV_X.append(row)
        ASV_Y.append(col)

    return ASV_X, ASV_Y, timestamps

def str_to_coord(coord_str):
    if len(coord_str) == 12:
        coord_str = '0' + coord_str #Add 0 to front
    deg = int(coord_str[:3])
    minutes = float(coord_str[3:])/60
    return deg + minutes

###############################################################################
# ADCP File Processing
###############################################################################

def read_ADCP_file(filename, num_measurements):
    with open(filename, 'r') as f:
        depths = f.readline().split(',')[:-1]
    with open(timestamps_file,'r') as f:
        ADCP_timestamps = f.readline().split(',')[:-1]
    ADCP_timestamps = [int(t) for t in ADCP_timestamps]

    ratio = float(len(depths))/num_measurements

    depths_down = np.zeros(num_measurements) #downsample
    for i in range(num_measurements):
        depths_down[i] = depths[int(i*ratio)]

    return -depths_down #depths are negative

###############################################################################

#Read data from 1 trial
def main():
    img, inv_trans = load_map(map_file)

    #GPS DATA
    ASV_X, ASV_Y, GPS_timestamps = read_GPS_file(GPS_file, inv_trans)
    num_measurements = len(ASV_X)

    #ADCP DATA
    Z = read_ADCP_file(depths_file, num_measurements) #Z = water depths
    min_depth = Z.min()

    #Normalize positions
    min_x = min(ASV_X)
    min_y = min(ASV_Y)
    X = [v - min_x for v in ASV_X]
    Y = [v - min_y for v in ASV_Y]
    max_x = max(X)
    max_y = max(Y)
    print('Max vals:', max_x, max_y)

    # Method 0: 1D Kalman Filter
    m = int(np.ceil(max_x/CELL_RES))
    n = int(np.ceil(max_y/CELL_RES))
    baseFloor = min_depth
    B = baseFloor*np.ones((m,n))
    Bvar = np.zeros((m,n))

    for t in range(num_measurements):
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

    # Method 1: Linear Interpolation
    # xi, yi = np.mgrid[X.min():X.max():1000j, Y.min():Y.max():1000j]

    # # Z is a matrix of x-y values
    # points = []
    # for i in range(len(X)):
    #     points.append([X[i],Y[i]])

    # zi = griddata(points, Z, (xi, yi), method='linear')
    # for i in range(len(zi)):
    #     for j in range(len(zi[0])):
    #         if np.isnan(zi[i][j]):
    #             zi[i][j] = min_depth

    ###########################################################################
    # PLOTS
    # 1) Map w/ ASV path
    # imgplot = plt.imshow(img)
    # plt.plot(ASV_X, ASV_Y, color='black')
    # plt.contourf(xi, yi, zi, 10, cmap=plt.cm.rainbow)

    # 2) Depth surface map
    X, Y = np.meshgrid(np.arange(n), np.arange(m))
    ax1 = plt.figure(figsize=(8,6)).gca(projection='3d')
    surf = ax1.plot_surface(X, Y, B, cmap=cm.viridis)

    plt.show()

if __name__ == '__main__':
    main()