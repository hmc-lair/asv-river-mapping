'''
rayCasting.py

Functions related to computing visited cells/intersection points
'''
import numpy as np
from planner_params import *

'''
Inverse of Distance between two points
inversed used due to faster speed relative to regular distance
'''
def invdist(state1, state2):
	try:
		return ((state1[0]-state2[0])**2 + (state1[1] - state2[1])**2)**(-0.5)
	except:
		return 0

'''
Returns set of cells (x,y) visited by travelling from start to end
m x n: Dimensions of information map
'''
def getVisitedCells(starts, ends, m, n):
	# Deprecated vars: leftover params from multi-view planning
	NUM_ANGLE_RANGES = 1
	ENABLE_ANGLES = False

	newVisitedCells = set()

	for k in range(len(ends)):
		start = starts[k]
		end = ends[k]

		if start == end: #If same point, nothing to compute
			continue
					
		#this block is basically just to find distance between start and end without trig and to do the 
		#vectorization without a separate class
		deltaX = end[0] - start[0]
		deltaY = end[1] - start[1]
		theta = np.arctan2(deltaY,deltaX)

		magnitude_squared = deltaX**2 + deltaY**2
		inv_sq = magnitude_squared**(-0.5)

		norm_delta_x = deltaX*inv_sq
		norm_delta_y = deltaY*inv_sq

		rcVec = [norm_delta_x*RC_DELTA, norm_delta_y*RC_DELTA]
		currentVec = [start[0], start[1]]

		numCasts = int(1/(RC_DELTA*invdist(start, end))) 

		#calculating which of the views we're coming at the cell from
		norm_theta = int(theta/((2*np.pi)/NUM_ANGLE_RANGES))

		#parameterize along the ray
		for i in range(numCasts):
			if ENABLE_ANGLES:
				currentCell = (int(currentVec[0]), int(currentVec[1]), norm_theta)
			else:
				currentCell = (int(currentVec[0]), int(currentVec[1]))
			#if currentCell not in newVisitedCells:
			newVisitedCells.add(currentCell)
			#enable/disable including a wider band of visited cells around the bot for the sake of later bots
			if ENABLE_SPREAD:
				for k in range(-SCANNING_SPREAD, SCANNING_SPREAD+1):
					for l in range(-SCANNING_SPREAD, SCANNING_SPREAD+1):
						if ENABLE_ANGLES:
							cell = (currentCell[0]+k, currentCell[1]+l, norm_theta)
						else:
							cell = (currentCell[0]+k, currentCell[1]+l)
						if cell not in newVisitedCells:
							newVisitedCells.add(cell)
			currentVec = [currentVec[0] + rcVec[0], currentVec[1] + rcVec[1]]

	return newVisitedCells

def lineRayIntersectionPoint(robotState, newNode, point1, point2):
	''' Given a robot position, a new node, and preplanned expansion, find the intersection point
	Args:
		robotState: [x,y] of robot's current pos
		newNode: [x,y] of expansion position
		point1: [x,y] of first check segment endpoint
		point2: [x,y] of second check segment endpoint
	Return:
		intersection point with expansion
		newNode if intersection point doesn't exist'''

	pointdeltaX = point2[0] - point1[0]
	pointdeltaY = point2[1] - point1[1]

	deltaX = newNode[0] - robotState[0]
	deltaY = newNode[1] - robotState[1]
	inv_sq = getInvDist(deltaX, deltaY)

	#get heading vector
	norm_delta_x = deltaX*inv_sq
	norm_delta_y = deltaY*inv_sq

	#get vectors for computing cross product
	pointtobotX = point1[0] - robotState[0]
	pointtobotY = point1[1] - robotState[1]
	bottopointX = robotState[0] - point1[0]
	bottopointY = robotState[1] - point1[1]


	# check if lines are parallel
	if np.cross([norm_delta_x, norm_delta_y],[pointdeltaX, pointdeltaY]) == 0:
		return newNode
	else:
		v = np.cross([pointtobotX, pointtobotY],[pointdeltaX, pointdeltaY]) / np.cross([norm_delta_x, norm_delta_y],[pointdeltaX, pointdeltaY])   # bot vector scaling factor
		u = np.cross([bottopointX, bottopointY], [norm_delta_x, norm_delta_y])  / np.cross([pointdeltaX, pointdeltaY], [norm_delta_x, norm_delta_y])  # wall vector scaling factor

		# check if intersection exists
		# v > 0 - Robot facing wall
		# 0 < u < 1 - Intersection in wall
		if v < 0 or u < 0 or u > 1:
			return newNode
		else:
			intPointX = point1[0] + (u*point2[0])
			intPointY = point1[1] + (u*point2[1])
			return [int(intPointX), int(intPointY)]	

'''inverse distance function used to speed up runtime in comparison to regular getDist'''
def getInvDist(state1, state2):
	try:
		return ((state1[0]-state2[0])**2 + (state1[1] - state2[1])**2)**(-0.5)
	except:
		return 0