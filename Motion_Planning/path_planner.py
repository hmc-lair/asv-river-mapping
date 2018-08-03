'''
path_planner.py

Main function for ASV RRT planner
'''
import time
import numpy as np
import random as rand
import matplotlib.pyplot as plt

import ray_casting as rc
import graph_results as graph
from planner_params import *

'''
Tree implementation for multi-robot RRT
'''
class TreeNode(object):
	def __init__(self, end, parent, scores, time):
		self.end = end #[[x1,y1]...[xn,yn]]
		self.scores = scores #info gain of edges (independent of parent!), [s1,...sn]
		self.visitedCells = set()
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
def rrtSerial(graph_bool, numCycles, maxTime, numRobots):
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
	bestInfoMap = None

	startState = [[10,0]] #list of locations to start the bots
	# xcoord = int(INFO_MAP_SIZE*0.85)
	# ycoord = INFO_MAP_SIZE
	# for i in range(numRobots): #generate a set of unique start states
	# 	startState.append([xcoord, int(((ycoord*.8)*(float(i)/numRobots))+(ycoord*.1))])

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
			c = selectNodeToExpand(nodeTable, HICs, maxTime)
			cNew = expandNewNode(c, nodeTable, infoMap, HICs, maxTime, numRobots)
			#c.children.append(cNew)

			if ENABLE_PRUNING:
				prune(cNew,infoMap)
			if endGame(cNew, maxTime):
				paths.append(cNew)

		##################################################

		# Return best path
		bestBatchPath = [] #3D array of path positions for each robot
		bestBatchScore = 0

		for path in paths:
			pathList,_ = getPath(path, numRobots, maxTime)
			points = []
			for p in pathList:
				points.append(p.end)

			score = computeScore(pathList[-1].visitedCells, infoMap)

			if score >= bestBatchScore:
				bestBatchPath = points
				bestBatchScore = score
	
		if bestBatchScore > bestScore:
			bestScore = bestBatchScore
			bestPath = bestBatchPath

	endTime = time.time()

	##################################################
	
	#Length of bestPath
	dists = []
	for j in range(numRobots):
		robotDist = 0
		for i in range(1,len(bestPath)):
			curDist = getDist(bestPath[i-1][j], bestPath[i][j])
			robotDist += curDist
		dists.append(robotDist)

	coverage = float(bestScore)/map_score

	print("Best % coverage:", coverage)
	#print "Runtime", endTime - startTime
	print("Path time", dists[0]/AUV_SPEED)

	if graph_bool:
		graph.graphResults(numRobots, bestPath)

	# store path
	print('Best path: ', bestPath)
	with open('mission.txt', 'w') as f:
		for [p] in bestPath:
			f.write(str(round(p[0],3)) + ',' + str(round(p[1],3)) + '\n')

	return endTime - startTime, coverage

############################### HELPER FUNCTIONS ###############################

'''
Expands parent node and returns the new, expanded node
'''
def expandNewNode(parent, bins, infoMap, HICs, maxTime, numRobots):
	diveDist = dynamicDive()

	newStates = [[0,0] for i in range(numRobots)]
	newTime = parent.time + diveDist/AUV_SPEED
	allVisitedCells = set()

	# Expand each robot serially
	order = np.arange(0,numRobots)
	rand.shuffle(order)
	prevExpansions = []
	for i in order:
		state = -1
		while state == -1: #Make sure within boundaries
			state = performDive(parent.end[i], i, diveDist, infoMap, HICs, prevExpansions)
		newStates[i] = state

		newVisitedCells = rc.getVisitedCells([parent.end[i]], [state], len(infoMap), len(infoMap[0]))
		allVisitedCells = allVisitedCells.union(newVisitedCells)

		score = computeScore(newVisitedCells, infoMap)

		#Add to HICs if above threshold
		if score > HIC_THRES:
			if ENABLE_HICS:
				# print("HIC", score)
				HICs.append([parent.end[i], state])


	newNode = TreeNode(newStates, parent, 0, newTime)
	#Add to bins
	addNodeToBins(bins, newNode, newTime, maxTime)

	#Update visitedCells & score
	newNode.score = computeScore(allVisitedCells, infoMap)
	newNode.visitedCells = allVisitedCells.union(parent.visitedCells)

	return newNode

def checkCollisions(start, prevExpansions):
	badThetas = []
	for x,y,theta_0 in prevExpansions:
		thetaPrime = np.arctan2(start[1]-y,start[0]-x)
		if thetaPrime < 0:
			thetaPrime += 2*np.pi
		deltaTheta = thetaPrime - theta_0
		thetaPrime -= np.pi #Reverse direction
		if thetaPrime < 0:
			thetaPrime += 2*np.pi
		theta = thetaPrime + deltaTheta
		badThetas.append(theta)
	return badThetas


def testCollisionChecking():
	start = (1,1)
	prevExpansions = [(5,-2,np.pi)]
	print(checkCollisions(start, prevExpansions)[0]*180./np.pi)

'''
Performs dive from endpoint of *ith* parent and returns new position
'''
def performDive(p_end, i, d, infoMap, HICs, prevExpansions):
	randVal = rand.random()
	if randVal >= gamma and len(HICs) > 0: #Towards random HIC
		h = HICs[rand.randint(0,len(HICs)-1)][1]
		theta = np.arctan2(h[1] - p_end[1], h[0] - p_end[0])

	else: #Towards random bearing
		theta = rand.uniform(0,2*np.pi)

	# Collision checking can be re-enabled for multi-robot planning
	# # Check if collision 
	# if COLLISION_PREV:
	# 	willCollide = True
	# 	badThetas = checkCollisions(p_end, prevExpansions)

	# 	while willCollide:
	# 		count = 0
	# 		for t in badThetas:
	# 			if np.fabs(t-theta) < COLLISION_THRES*np.pi/180:
	# 				count += 1
	# 		if count == 0:
	# 			willCollide = False
	# 		else:
	# 			theta = rand.uniform(0,2*np.pi)

	# 	prevExpansions.append((p_end[0], p_end[1], theta))

	newState = [0,0]
	newState[0] = d*np.cos(theta) + p_end[0] #X
	newState[1] = d*np.sin(theta) + p_end[1] #Y

	# TODO: If new point outside of map, don't add
	if newState[0] >= n or newState[0] < 0:
		print("outside range", newState[0])
		return -1
	if newState[1] >= m or newState[1] < 0:
		print("outside range", newState[1])
		return -1

	return newState

# Extend dive to be distance d
def extendDive(start, end, d):
	theta = np.arctan2(float(end[1] - start[1]), float(end[0] - start[0]))
	newEnd = [0,0]
	newEnd[0] = int(d*np.cos(theta)) + start[0] #X
	newEnd[1] = int(d*np.sin(theta)) + start[1] #Y
	return newEnd

### Other helper functions ###
'''
Selecting and adding nodes to hashtable
'''
def selectNodeToExpand(bins, HICs, maxTime):
	randBinIndex = rand.choice(list(bins.keys())[:-1]) #Choose random bin, then choose node

	while len(bins[randBinIndex]) == 0:
		randBinIndex = rand.choice(list(bins.keys())[:-1])

	return rand.choice(bins[randBinIndex])

def addNodeToBins(bins, node, nodeTime, maxTime):
	binIndex = int(nodeTime/(maxTime/NUM_BINS))
	if binIndex > 5:
		binIndex = 5
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
	return endNode.time > maxTime

def getDist(state1, state2): #2D distance since all nodes are surface points
	return ((state1[0]-state2[0])**2 + (state1[1] - state2[1])**2)**.5

def getPath(endNode, numRobots, maxTime):
	curNode = endNode
	dist = 0
	count = 0
	path = []

	while curNode.time > maxTime:
		curNode = curNode.parent

	while curNode.parent != None:
		path.insert(0, curNode)
		for i in range(numRobots):
			dist += getDist(curNode.parent.end[i], curNode.end[i])
		curNode = curNode.parent
		count += 1
	path.insert(0, curNode)
	return path, dist

def prune(cNew,infoMap):
	c = cNew.parent

	if c.parent == None:
		return

	cells_ik = rc.getVisitedCells(c.parent.end, cNew.end, len(infoMap), len(infoMap[0]))

	if c.score <= 1:
		# print("Removing useless")
		cNew.visitedCells = c.parent.visitedCells.union(cells_ik)
		c.parent.children.append(cNew)
		if cNew in c.children:
			c.children.remove(cNew)
		cNew.parent = c.parent

	O_ik = computeScore(cells_ik.difference(c.parent.visitedCells), infoMap)
	t_ik = getDist(c.parent.end[0], cNew.end[0]) / AUV_SPEED
	O_ijjk = c.score + cNew.score

	if t_ik <= 0 or (cNew.time - c.time) <= 0:
		return

	if (O_ik / t_ik) > (O_ijjk / (cNew.time - c.time)):
		# print("Pruning")
		cells_ik = rc.getVisitedCells(c.parent.end, cNew.end, len(infoMap), len(infoMap[0]))
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
	maxTime = 250
	numRobots = 1
	rrtSerial(True, numCycles, maxTime, numRobots)

if __name__ == "__main__":
	main()