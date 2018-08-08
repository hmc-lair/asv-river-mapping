'''
create_maps.py
Creates occupancy grid and info map for RRT planner
'''
import numpy as np
import matplotlib.image as mpimg
from scipy import signal
from scipy import ndimage
import scipy.io as sio
import os
import utm
import gdal
import struct

from depth_kalman_filter import *

sigma = 1 #Gaussian blur for info map
tau = 0.4 #Thresholding
SCALING_FACTOR = 10

#ASV log files
millikan1 = '../ASV_Controller/Log/millikan_7-19/ALL_18-07-11 23.59.36.bin' #1) GOOD MAP OF MILLIKAN
millikan2 = '../ASV_Controller/Log/millikan_7-19/ALL_18-07-12 00.16.04.bin' #2) Good
millikan3 = '../ASV_Controller/Log/millikan_7-19/ALL_18-07-11 23.36.33.bin' #3) Good
millikan4 = '../ASV_Controller/Log/millikan_7-19/ALL_18-07-12 00.24.24.bin' #4) Large range
lake1 = '../ASV_Controller/Log/lake_7-27/ALL_18-07-12 06.49.05.bin'
river1 = '../ASV_Controller/Log/river_7-27/ALL_18-07-12 06.47.41.bin'

data_file = river1

##########################################################

def generateDepthGradientMaps():
	#GPS Data
	ASV_nor, ASV_eas, Z = read_data_file(data_file)

	if len(ASV_nor) != len(Z):
		print('Size mismatch!', len(ASV_nor), len(Z))
		ASV_nor = ASV_nor[:-1]
		ASV_eas = ASV_eas[:-1]

	B, m, n, min_x, min_y = kalman_filter(ASV_nor, ASV_eas, Z, CELL_RES)

	Gx, Gy = np.gradient(B) # gradients with respect to x and y
	G = (Gx**2+Gy**2)**.5  # gradient magnitude

	for i in range(len(G)):
		for j in range(len(G[0])):
			if G[i][j] > .4:
				G[i][j] = 0
			G[i][j] *= 4

	N = G/G.max()  # normalize 0..1

	with open('map_depth.csv', 'w') as f:
		f.write(str(min_x) + ',' + str(min_y) + ',' + str(CELL_RES) + '\n')
		for i in range(len(B)):
			for j in range(len(B[0])):
				f.write(str(round(B[i][j], 3)))
				if j < len(B[0])-1:
					f.write(',')
			f.write('\n')
	
	with open('map_gradient_norm.csv', 'w') as f:
		f.write(str(min_x) + ',' + str(min_y) + ',' + str(CELL_RES) + '\n')
		for i in range(len(N)):
			for j in range(len(N[0])):
				N[i][j] *= SCALING_FACTOR

				f.write(str(round(N[i][j], 3)))
				if j < len(N[0])-1:
					f.write(',')
			f.write('\n')

	return N, len(G), len(G[0])

def setupInfoMap(filename):
	# Load origin UTM
	f = open(filename, 'r')
	origin_x, origin_y, CELL_RES = list(map(float,f.readline()[:-1].split(',')))
	f.close()

	# Load file
	E = np.loadtxt(open(filename, 'r'), delimiter=',', skiprows=1)
	m,n = E.shape
	return E, m, n, origin_x, origin_y, CELL_RES

def createInfoMap(E, m, n):
	#Calculate gradient of E using Sobel operator
	Sobel_x = [[-1,0,1],[-2,0,2],[-1,0,1]]
	Sobel_y = [[-1,-2,-1],[0,0,0],[1,2,1]]
	G_x = signal.convolve2d(Sobel_x, E)
	G_y = signal.convolve2d(Sobel_y, E)
	G = np.zeros((m,n)) #Gradient of E
	for i in range(m):
		for j in range(n):
			G[i][j] = np.sqrt(G_x[i][j]**2 + G_y[i][j]**2)

	#Info map is Gaussian blur kernel * G
	return ndimage.filters.gaussian_filter(G,sigma)

''' function that takes in 2d info map and scores it for calculating coverage metric at end of planner'''
def scoreInfoMap(infoMap):

	map_score = 0
	xlen = len(infoMap[0])
	ylen = len(infoMap)
	max_val = 0

	for x in range(xlen):
		for y in range(ylen):
			map_score += infoMap[y][x]
			if infoMap[y][x] > max_val:
					max_val = infoMap[y][x]	
				
	return map_score, max_val

if __name__ == '__main__':
	generateDepthGradientMaps()
