GPS_file = '../PI/Log/GPS_18-07-02 17.27.07.txt'
map_file = ''

###############################################################################
# GPS File Processing
###############################################################################

def read_GPS_file(filename):
    all_data = []
    with open(filename, 'r') as f:
        for line in f.readlines():
            vals = line.split(',')
            if line[:6] == '$GPGGA' and len(vals) >= 6:
                lat_str,_,lon_str,_ = vals[2:6]
                lat = str_to_coord(lat_str)
                lon = -str_to_coord(lon_str) #W = negative, generalize later
                all_data.append((lat,lon))
    # print(all_data)
    return all_data

def str_to_coord(coord_str):
    if len(coord_str) == 12:
        coord_str = '0' + coord_str #Add 0 to front
    deg = int(coord_str[:3])
    minutes = float(coord_str[3:])/60
    return deg + minutes

###############################################################################
# Map Loading
###############################################################################



###############################################################################

# Read data from 1 trial
def main():
    read_GPS_file(GPS_file)


if __name__ == '__main__':
    main()