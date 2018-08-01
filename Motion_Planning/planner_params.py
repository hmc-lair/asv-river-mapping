'''
planner_params.py

Standardized planner parameters
'''
import math
import create_maps as maps

#Planner parameters
gamma = 0.5 #proportion of the time planner picks HIC to expand from instead of random node
numBatches = 3 #Number of times tree is restarted (keeping HICs list)

#Node selection parameter
NUM_BINS = 5

#Tuning parameters
ENABLE_HICS = True #turn on/off high info center node selection
ENABLE_PRUNING = True #turn on/off cutting short inefficient pairs of edges
ENABLE_ANGLES = False #turn on/off consideration of different cell view angles

HIC_THRES = 50 #score threshold determining whether or not a node is an HIC
COLLISION_PREV = False
COLLISION_THRES = 10 #Degrees of cushion on either side

#AUV/Environment Parameters
AUV_SPEED = 1.028 #m/s #TODO: Fix scaling, cell != 1m
DIVE_DEPTH = -3 #HFB
TURN_RAD = 20 #m
WATER_DEPTH = 40 #m
DIVE_ANGLE = 45 #degrees
MAX_DIVE_LENGTH = 20
MIN_DIVE_LENGTH = 5
SURF_LENGTH = float(WATER_DEPTH) / math.tan(DIVE_ANGLE*math.pi/180)

#Collision Parameters
COLLISION_DIST = 10 #number of cells

#Raycasting Parameters
WIDTH = 5 * 10 #.1 scaling, so # of cells 
SCANNING_SPREAD = 2 #TODO: make this a multiple of WIDTH
RC_DELTA = .5
ENABLE_SPREAD = True
NUM_ANGLE_RANGES = 4

#Used to compute robot start position
INFO_MAP_SIZE = 45 #TODO: Fix this

#Maps
E, m , n = maps.setupInfoMap('maps/river_map.csv')