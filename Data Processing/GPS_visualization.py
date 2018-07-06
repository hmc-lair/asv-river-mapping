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

###############################################################################
# GPS File Processing
###############################################################################

# Returns UTM coordinates from GPS data
def read_GPS_file(filename):
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
    # print(all_data)
    # print(timestamps)
    return all_data, timestamps

def str_to_coord(coord_str):
    if len(coord_str) == 12:
        coord_str = '0' + coord_str #Add 0 to front
    deg = int(coord_str[:3])
    minutes = float(coord_str[3:])/60
    return deg + minutes

###############################################################################
# Map Helper Functions
###############################################################################

def load_map(filename):
    dataset = gdal.Open(filename)
    data = dataset.ReadAsArray()

    geo_trans = dataset.GetGeoTransform()
    inv_trans = gdal.InvGeoTransform(geo_trans)

    #Plot
    img = mpimg.imread(filename)
    return img, inv_trans

###############################################################################

#Read data from 1 trial
def main():
    #GPS DATA
    img, inv_trans = load_map(map_file)
    ASV_pos, GPS_timestamps = read_GPS_file(GPS_file)

    ASV_X = []
    ASV_Y = []
    for x, y in ASV_pos:
        row,col = gdal.ApplyGeoTransform(inv_trans, x, y)
        ASV_X.append(row)
        ASV_Y.append(col)
    plt.plot(ASV_X, ASV_Y, color='black')

    #ADCP DATA
    with open(depths_file, 'r') as f:
        depths = f.readline().split(',')[:-1]
    with open(timestamps_file,'r') as f:
        ADCP_timestamps = f.readline().split(',')[:-1]
    ADCP_timestamps = [int(t) for t in ADCP_timestamps]

    #Method 1: Assume start readings at same time, equal spacing
    ratio = float(len(depths))/len(ASV_X)
    depths_down = np.zeros(len(ASV_X))
    for i in range(len(ASV_X)):
        depths_down[i] = depths[int(i*ratio)]

    X = np.asarray(ASV_X)
    Y = np.asarray(ASV_Y)
    Z = -depths_down

    #Method 2: Assume sensors have same clock
    # X = []
    # Y = []
    # Z = []
    # for i in range(len(GPS_timestamps)):
    #     if GPS_timestamps[i] in ADCP_timestamps: # if we have both GPS and ADCP data
    #         X.append(ASV_X[i])
    #         Y.append(ASV_Y[i])
    #         Z.append(-float(depths[i]))
    # X = np.asarray(X)
    # Y = np.asarray(Y)
    # Z = np.asarray(Z)

    #Plot
    imgplot = plt.imshow(img)

    # create x-y points to be used in heatmap
    xi = np.linspace(X.min(),X.max(),1000)
    yi = np.linspace(Y.min(),Y.max(),1000)

    # Z is a matrix of x-y values
    zi = griddata((X, Y), Z, (xi[None,:], yi[:,None]), method='linear')

    # Create the contour plot
    plt.contourf(xi, yi, zi, 10, cmap=plt.cm.rainbow)
    plt.colorbar()
    #ax1 = plt.figure(figsize=(8,6)).gca(projection='3d')
    #surf = ax1.scatter(X, Y, Z, cmap=cm.viridis)
    
    plt.show()

if __name__ == '__main__':
    main()