import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import utm
import gdal
import os

GPS_file = '../PI/Log/GPS_18-07-02 17.17.09.txt'

# To crop GEOTIFF use:
# gdal_translate -srcwin 3000 9000 4000 3000 input.tif output.tif
map_file = '../Maps/kern_river.tif'

###############################################################################
# GPS File Processing
###############################################################################

# Returns UTM coordinates from GPS data
def read_GPS_file(filename):
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
    # print(all_data)
    return all_data

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
    imgplot = plt.imshow(img)
    return inv_trans


###############################################################################

# Read data from 1 trial
def main():
    inv_trans = load_map(map_file)

    ASV_pos = read_GPS_file(GPS_file)

    ASV_X = []
    ASV_Y = []
    for lat, lon in ASV_pos:
        row,col = gdal.ApplyGeoTransform(inv_trans, lat, lon)
        ASV_X.append(row)
        ASV_Y.append(col)

    plt.plot(ASV_X, ASV_Y)
    plt.show()


if __name__ == '__main__':
    main()