'''
simulate_mission_bathymetry.py

Simulate RRT mission collecting depth data -> re-generate surface plot
'''
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import numpy as np
import utm

from depth_kalman_filter import *

traj_file = 'mission.txt'
map_file = 'maps/river_map_depth.csv'

ASV_SPEED = 1 #m/s
TIMESTEP = .5 #s

def main():
	# Load map data
	f = open(map_file, 'r')
	origin_nor, origin_eas, CELL_RES = list(map(float,f.readline()[:-1].split(',')))
	print(origin_nor, origin_eas, CELL_RES)
	f.close()

	E = np.loadtxt(open(map_file, 'r'), delimiter=',', skiprows=1)

	# Load traj waypoints
	wps = np.loadtxt(open(traj_file, 'r'), delimiter=',')

	depths = []
	all_wps = []
	for i in range(len(wps)-1):
		start = wps[i]
		end = wps[i+1]
		pts = get_intermediate_points(start, end)
		print('start, end', start, end)
		for x,y in pts:
			row = int(y)
			col = int(x)
			print(row,col)
			all_wps.append([y, x])
			depths.append(E[row][col])
	
	ASV_nor = []
	ASV_eas = []
	for r,c in all_wps:
		nor = c*CELL_RES + origin_nor
		eas = r*CELL_RES + origin_eas
		ASV_nor.append(nor)
		ASV_eas.append(eas)


	# all_wps = []
	# for i in range(len(wps)-1):
	# 	start = wps[i]
	# 	end = wps[i+1]
	# 	pts = get_intermediate_points(start, end)
	# 	for x,y in pts:
	# 		row = int(y)
	# 		col = int(x)
	# 		all_wps.append([y, x])
	# 		depths.append(E[row][col])

	# ASV_nor = []
	# ASV_eas = []
	# for r,c in all_wps:
	# 	nor = r*CELL_RES + origin_nor
	# 	eas = c*CELL_RES + origin_eas
	# 	ASV_nor.append(nor)
	# 	ASV_eas.append(eas)
	# 	depths.append(E[int(r)][int(c)])
		

	ASV_nor = np.asarray(ASV_nor)
	ASV_eas = np.asarray(ASV_eas)
	Z = np.asarray(depths)

	###########################################################################
	

	###########################################################################
	# PLOTS
	B, m, n, min_x, min_y = kalman_filter(ASV_nor, ASV_eas, Z, CELL_RES)
	print(B.shape)
	X_plot, Y_plot = np.meshgrid(np.arange(min_y, min_y + n*CELL_RES, CELL_RES), np.arange(min_x, min_x + m*CELL_RES, CELL_RES))
	print(X_plot.shape, Y_plot.shape)

	ax1 = plt.figure(figsize=(8,6)).gca(projection='3d')
	ax1.plot_surface(X_plot, Y_plot, B, cmap=cm.viridis) #depths
	ax1.plot(ASV_eas, ASV_nor, np.zeros(len(ASV_eas)), color='red') #ASV path
	ax1.view_init(200, -50)
	ax1.set_zlabel('Depth (m)')
	ax1.invert_zaxis()
	ax1.set_xlabel('Easting (m)')
	ax1.set_ylabel('Northing (m)')

	plt.show()

def get_intermediate_points(start, end):
	total_d = dist(start, end)
	theta = np.arctan2(end[1] - start[1], end[0] - start[0])
	print(theta)

	num_pts = int((float(total_d)/(TIMESTEP*ASV_SPEED)))-1
	inc_d = float(total_d)/num_pts
	# print('num points', num_pts)

	points = []
	for i in range(num_pts):
		d = i*inc_d
		newState = [0,0]
		newState[0] = d*np.cos(theta) + start[0] #X
		newState[1] = d*np.sin(theta) + start[1] #Y
		points.append(newState)
	#points.append(end)

	# print(points)
	return points
		

def dist(start, end):
	return np.sqrt((start[0] - end[0])**2 + (start[1] - end[1])**2)


if __name__ == '__main__':
	# start = [0,0]
	# end = [3, 4]
	# get_intermediate_points(start, end)
	main()