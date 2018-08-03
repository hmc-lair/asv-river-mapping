import numpy as np
import utm

def main():
	# Files
	traj_file = 'mission.txt'
	map_file = 'map.csv'

	mission_file = 'millikan_RRT_test.csv'

	# Load map data
	f = open(map_file, 'r')
	origin_x, origin_y, CELL_RES = list(map(float,f.readline()[:-1].split(',')))
	print origin_x, origin_y, CELL_RES
	f.close()

	# Load mission waypoints
	wps = np.loadtxt(open(traj_file, 'r'), delimiter=',')
	print wps

	latlons = []
	with open(mission_file, 'w') as f:
		for r,c in wps:
			x = r*CELL_RES + origin_x
			y = c*CELL_RES + origin_y
			lat,lon = utm.to_latlon(x, y, 11, 'S')
			latlons.append([lat, lon])
			f.write(str(lat) + ',' + str(lon) + '\n')



	E = np.loadtxt(open(map_file, 'r'), delimiter=',', skiprows=1)
	m,n = E.shape
	print m,n



	


if __name__ == '__main__':
	main()