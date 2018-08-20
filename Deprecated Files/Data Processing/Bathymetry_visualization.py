import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from scipy.interpolate import griddata
import numpy as np
import utm
import gdal


BEAM_ANGLE = 20 #degrees

heading_file = 'headings.txt'
depths_file = 'depths_all.txt'
timestamps_file = 'timestamps.txt'
GPS_file = '../PI/Log/GPS_18-07-02 17.17.09.txt' #lake
#GPS_file = '../PI/Log/GPS_18-07-02 17.27.07.txt' #river

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
    _, height, width = data.shape
    geo_trans = dataset.GetGeoTransform()
    inv_trans = gdal.InvGeoTransform(geo_trans)

    img = mpimg.imread(filename)
    return img, inv_trans, geo_trans, width, height

###############################################################################
# GPS File Processing
###############################################################################

# Returns pixel coordinates from GPS data
def read_GPS_file(filename, inv_trans):
    all_data = []
    with open(filename, 'r') as f:
        for line in f.readlines():
            vals = line.split(',')
            if line[:6] == '$GPGGA' and len(vals) > 10:
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

    return np.asarray(ASV_X), np.asarray(ASV_Y)

def str_to_coord(coord_str):
    per_index = coord_str.find('.')
    if per_index == 4:
        coord_str = '0' + coord_str #Add 0 to front
    deg = int(coord_str[:3])
    minutes = float(coord_str[3:])/60
    return deg + minutes

###############################################################################
# ADCP File Processing
###############################################################################

def read_ADCP_file(filename, num_measurements):
    all_depths = []
    with open(filename, 'r') as f:
        for line in f.readlines():
            depths = line.split(',')[:-1]
            depths = [float(d)*np.cos(BEAM_ANGLE*np.pi/180.) for d in depths]
            all_depths.append(np.average(depths))
    ratio = float(len(all_depths))/num_measurements

    depths_down = []
    for i in range(num_measurements):
        depths_down.append(-all_depths[int(i*ratio)])

    return np.asarray(depths_down) #depths are negative

###############################################################################
# ADCP File Processing
###############################################################################

def read_heading_file(filename, num_measurements):
    with open(filename, 'r') as f:
        headings = f.readline().split(',')[:-1]
    # Heading w.r.t. beam 3 (45 clockwise from N)
    for i in range(len(headings)):
        h_val = float(headings[i])
        if h_val < 45:
            headings[i] = (h_val-45) + 360
        else:
            headings[i] = h_val - 45

    ratio = float(len(headings))/num_measurements

    headings_down = []
    for i in range(num_measurements):
        headings_down.append(headings[int(i*ratio)])

    return headings_down

###############################################################################

#Read data from 1 trial
def main():
    img, inv_trans, geo_trans, MAP_WIDTH, MAP_HEIGHT = load_map(map_file)

    #GPS DATA
    ASV_X, ASV_Y = read_GPS_file(GPS_file, inv_trans)
    num_measurements = len(ASV_X)

    #ADCP DATA
    Z = read_ADCP_file(depths_file, num_measurements) #Z = water depths
    min_depth = Z.min()

    #HEADINGS
    headings = read_heading_file(heading_file, num_measurements)

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
    xi, yi = np.mgrid[ASV_X.min():ASV_X.max():1000j, ASV_Y.min():ASV_Y.max():1000j]

    # Z is a matrix of x-y values
    points = []
    for i in range(len(ASV_X)):
        points.append([ASV_X[i],ASV_Y[i]])

    zi = griddata(points, Z, (xi, yi), method='linear')
    for i in range(len(zi)):
        for j in range(len(zi[0])):
            if np.isnan(zi[i][j]):
                zi[i][j] = min_depth

    ###########################################################################
    # PLOTS
    # 1) Map w/ ASV path
    imgplot = plt.imshow(img)
    plt.plot(ASV_X, ASV_Y, color='black', zorder=2)
    plt.xlabel('Meters')
    plt.ylabel('Meters')

    x0, y0 = gdal.ApplyGeoTransform(geo_trans, 0, 0) #UTM of top left corner
    x1, y1 = gdal.ApplyGeoTransform(geo_trans, 4000, 3000)
    print('Offsets', x1-x0, y1-y0)

    ax = plt.gca() # grab the current axis
    ax.set_xticks(np.arange(0,MAP_WIDTH+500,500)) # choose which x locations to have ticks
    xlabels = np.linspace(0, x1-x0, int(MAP_WIDTH/500.)+1)
    xlabels = [int(p) for p in xlabels]
    ax.set_xticklabels(xlabels) # set the labels to display at those ticks

    ax.set_yticks(np.arange(0,MAP_HEIGHT,500)) # choose which x locations to have ticks
    ylabels = np.linspace(0, y0-y1, int(MAP_HEIGHT/500.)+1)
    ylabels = [int(p) for p in ylabels]
    ax.set_yticklabels(ylabels) # set the labels to display at those ticks

    #plt.contourf(xi, yi, zi, 10, cmap=plt.cm.rainbow, zorder=1)

    # 2) Depth surface map
    # B_new = np.zeros((n,m))
    # for i in range(n):
    #     for j in range(m):
    #         B_new[i][j] = B[j][i]

    # X_plot, Y_plot = np.meshgrid(np.arange(m), np.arange(n))
    # ax1 = plt.figure(figsize=(8,6)).gca(projection='3d')
    # surf = ax1.plot_surface(X_plot, Y_plot, B_new, cmap=cm.viridis)

    # p = ax1.plot(X, Y, np.zeros(len(X)), color='red')
    # ax1.view_init(200, -50)
    # ax1.set_zlabel('Depth (m)')
    # ax1.invert_zaxis()
    # ax1.set_xlabel('Easting (m)')
    # ax1.set_ylabel('Northing (m)')
    plt.show()

if __name__ == '__main__':
    main()