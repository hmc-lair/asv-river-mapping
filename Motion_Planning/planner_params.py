'''
planner_params.py

Standardized planner parameters
'''
import math
import create_maps as maps

#Maps
E, m , n, origin_x, origin_y, CELL_RES = maps.setupInfoMap('maps/river_map_gradient_norm.csv')

#Planner parameters
gamma = 0.5 #proportion of the time planner picks HIC to expand from instead of random node
numBatches = 3 #Number of times tree is restarted (keeping HICs list)

#Node selection parameter
NUM_BINS = 5

#Tuning parameters
ENABLE_HICS = True #turn on/off high info center node selection
ENABLE_PRUNING = True #turn on/off cutting short inefficient pairs of edges
ENABLE_ANGLES = False #turn on/off consideration of different cell view angles

HIC_THRES = 1#50 #score threshold determining whether or not a node is an HIC
ENDGAME_THRES = 5 #radius of endgame region (along edge)

#AUV/Environment Parameters
ASV_SPEED = 1.0*CELL_RES #m/s 
TURN_RAD = 20 #m
MAX_DIVE_LENGTH = 15#20
MIN_DIVE_LENGTH = 5

#Raycasting Parameters
WIDTH = 5 * 10 #.1 scaling, so # of cells 
SCANNING_SPREAD = 5 #TODO: make this a multiple of WIDTH
RC_DELTA = .5
ENABLE_SPREAD = True
NUM_ANGLE_RANGES = 4