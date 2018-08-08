'''
path_planner.py

Main function for ASV RRT planner
'''
import time
import utm
import numpy as np
import random as rand
import matplotlib.pyplot as plt

import ray_casting as rc
from planner_params import *

'''
Tree implementation for multi-robot RRT
'''
class TreeNode(object):
	def __init__(self, state, parent, score, time):
		self.state = state #[x,y]
		self.score = score #info gain of edges (independent of parent!), [s1,...sn]
		self.visitedCells = set() #includes all visited cells of parent traced up to root!
		self.children = []
		self.parent = parent #A node
		self.time = time

'''
Inputs:
gamma - percent randomness for expanding to new nodes
numCycles - number of node expansions
maxTime - time limit for AUV paths

Output:
Information gain of best path
'''
def rrt(graph_bool, numCycles, maxTime, mission_file):
	startTime = time.time()

	#Initialize information maps
	infoMap = maps.createInfoMap(E, m, n)

	if graph_bool:
		plt.matshow(infoMap,cmap='viridis') #show map in path graphing figure
		plt.colorbar() #infoMap scale for figure
		print("Done creating maps", m, n)

	map_score,_ = maps.scoreInfoMap(infoMap)

	######################################################
	bestPath = []
	bestScore = 0
	bestDist = -1
	bestInfoMap = None

	startState = [0,0]

	for i in range(numBatches):
		HICs = []
		paths = [] #End nodes of paths
		tree = TreeNode(startState, None, 0, 0)

		#Initialize hashtable for nodes
		nodeTable = dict()
		for j in range(NUM_BINS+1):
			nodeTable[j] = []
		nodeTable[0] =[tree]

		for j in range(numCycles):
			c = selectNodeToExpand(nodeTable, HICs)
			cNew = expandNewNode(c, nodeTable, infoMap, HICs, maxTime)
			#c.children.append(cNew)

			if ENABLE_PRUNING:
				prune(cNew,infoMap)
			if endGame(cNew, maxTime):
				paths.append(cNew)

		##################################################

		# Return best path
		bestBatchPath = [] #3D array of path positions for each robot
		bestBatchScore = 0
		bestBatchDist = -1

		print('Num paths:', len(paths))

		for path in paths:
			pathList,_ = getPath(path, maxTime)
			points = []
			for p in pathList:
				points.append(p.state)

			#Length of path (only works for 1 robot)
			dist = 0
			for i in range(1,len(points)):
				dist += getDist(points[i-1], points[i])

			#Score of path
			score = computeScore(pathList[-1].visitedCells, infoMap)

			if score/dist >= bestBatchScore/bestBatchDist:
				bestBatchPath = points
				bestBatchScore = score
				bestBatchDist = dist
		
		if bestBatchScore/bestBatchDist > bestScore/bestDist:
			bestScore = bestBatchScore
			bestDist = bestBatchDist
			bestPath = bestBatchPath

	endTime = time.time()

	##################################################
	# Post processing
	coverage = float(bestScore)/map_score
	print("Best % coverage:", coverage)
	print("Path time", bestDist/ASV_SPEED)
	print('Best path: ', bestPath)
	
	# Save path
	with open('mission.txt', 'w') as f:
		for p in bestPath:
			f.write(str(round(p[0],3)) + ',' + str(round(p[1],3)) + '\n')

	# Save lat/lons
	latlons = []
	with open(mission_file, 'w') as f:
		for r,c in bestPath:
			x = r*CELL_RES + origin_x
			y = c*CELL_RES + origin_y
			lat,lon = utm.to_latlon(x, y, 11, 'S')
			latlons.append([lat, lon])
			f.write(str(lat) + ',' + str(lon) + '\n')

	if graph_bool:
		X = [p[0] for p in bestPath]
		Y = [p[1] for p in bestPath]
		plt.plot(X,Y, '-o', color='r')
		plt.show()

	return endTime - startTime, coverage

############################### HELPER FUNCTIONS ###############################

'''
Expands parent node and returns the new, expanded node
'''
def expandNewNode(parent, bins, infoMap, HICs, maxTime):
	diveDist = dynamicDive()
	newTime = parent.time + diveDist/ASV_SPEED

	state = -1
	while state == -1: #Make sure within boundaries
		state = performDive(parent.state, diveDist, infoMap, HICs)
	
	newVisitedCells = rc.getVisitedCells([parent.state], [state], len(infoMap), len(infoMap[0]))
	score = computeScore(newVisitedCells, infoMap)

	#Add to HICs if above threshold
	if score > HIC_THRES:
		if ENABLE_HICS:
			HICs.append([parent.state, state])


	newNode = TreeNode(state, parent, 0, newTime)
	addNodeToBins(bins, newNode, newTime, maxTime)

	#Update visitedCells & score
	newNode.score = computeScore(newVisitedCells, infoMap)
	newNode.visitedCells = newVisitedCells.union(parent.visitedCells)

	return newNode

'''
Performs dive from endpoint of *ith* parent and returns new position
'''
def performDive(p_end, d, infoMap, HICs):
	randVal = rand.random()
	if randVal >= gamma and len(HICs) > 0: #Towards random HIC
		h = HICs[rand.randint(0,len(HICs)-1)][1]
		theta = np.arctan2(h[1] - p_end[1], h[0] - p_end[0])

	else: #Towards random bearing
		theta = rand.uniform(0,2*np.pi)

	newState = [0,0]
	newState[0] = d*np.cos(theta) + p_end[0] #X
	newState[1] = d*np.sin(theta) + p_end[1] #Y

	# If new point outside of map, don't add
	if newState[0] >= n or newState[0] < 0:
		#print("outside range", newState[0])
		return -1
	if newState[1] >= m or newState[1] < 0:
		#print("outside range", newState[1])
		return -1
	return newState

### Other helper functions ###
'''
Selecting and adding nodes to hashtable
'''
def selectNodeToExpand(bins, HICs):
	randBinIndex = rand.choice(list(bins.keys())[:-1]) #Choose random bin, then choose node

	while len(bins[randBinIndex]) == 0:
		randBinIndex = rand.choice(list(bins.keys())[:-1])

	return rand.choice(bins[randBinIndex])

def addNodeToBins(bins, node, nodeTime, maxTime):
	x = node.state[0]
	y = node.state[1]
	binIndex = int((float(y)/m)*NUM_BINS)
	#print(y, height, binIndex)
	if binIndex > NUM_BINS:
		binIndex = NUM_BINS

	bins[binIndex].append(node)


def dynamicDive():
	return rand.uniform(MIN_DIVE_LENGTH, MAX_DIVE_LENGTH) #Add dive angle later

'''compute score of a path by adding up information gain of all visited cells'''
def computeScore(visitedCells, infoMap):
	score = 0
	for x,y in visitedCells: #loop through all visitedCells
		if (0 <= x < n) and (0 <= y < m):
			score += infoMap[y][x] #return aggregate score
	return score

def endGame(endNode, maxTime):
	#return endNode.time > maxTime
	return endNode.state[1] >= m - ENDGAME_THRES

def getDist(state1, state2): #2D distance since all nodes are surface points
	return ((state1[0]-state2[0])**2 + (state1[1] - state2[1])**2)**.5

def getPath(endNode, maxTime):
	curNode = endNode
	dist = 0
	count = 0
	path = []

	# while curNode.time > maxTime: #Cutting all paths to be desired time
	# 	curNode = curNode.parent

	while curNode.parent != None:
		path.insert(0, curNode)
		dist += getDist(curNode.parent.state, curNode.state)
		curNode = curNode.parent
		count += 1
	path.insert(0, curNode)
	return path, dist

def prune(cNew,infoMap):
	c = cNew.parent

	if c.parent == None:
		return

	cells_ik = rc.getVisitedCells([c.parent.state], [cNew.state], len(infoMap), len(infoMap[0]))

	O_ik = computeScore(cells_ik.difference(c.parent.visitedCells), infoMap)
	t_ik = getDist(c.parent.state, cNew.state) / ASV_SPEED
	O_ijjk = c.score + cNew.score

	if t_ik <= 0 or (cNew.time - c.time) <= 0:
		return

	if (O_ik / t_ik) > (O_ijjk / (cNew.time - c.time)):
		print('prune')
		cNew.visitedCells = c.parent.visitedCells.union(cells_ik)
		c.parent.children.append(cNew)
		if cNew in c.children:
			c.children.remove(cNew)
		if c in c.parent.children:
			c.parent.children.remove(c)
		cNew.parent = c.parent
		cNew.score = O_ik

# Test rrt
def main():
	numCycles = 2000
	maxTime = -1
	rrt(True, numCycles, maxTime, 'rrt_test.csv')

if __name__ == "__main__":
	main()