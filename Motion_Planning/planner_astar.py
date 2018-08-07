from __future__ import print_function
import matplotlib.pyplot as plt
import matplotlib.colors as clr
import numpy as np

# Map creation
x_plot, y_plot, vx, vy = np.load('maps/RIVER_SURFACE.csv')
m,n = vx.shape

# Tuning parameters
V_max = 1.5 #m/s
V_fm = 2 #m/s, maximum flow velocity

COST = 'time' 
#COST = 'energy'

#Straight across
# start = (25,20)
# goal = (0,20)

#With current diagonal
# start = (15,15)
# goal = (9,0)

#Against current diagonal
# start = (9,20)
# goal = (25,0)

def main():
	#MAP SETUP
	plt.quiver(x_plot, y_plot, vx, vy)
	# Removing nans
	for i in range(m):
		for j in range(n):
			if np.isnan(x_plot[i][j]):
				x_plot[i][j] = 99999999999
			if np.isnan(y_plot[i][j]):
				y_plot[i][j] = 99999999999
	x_min = x_plot.min()
	y_min = y_plot.min()

	#A-STAR GRAPH SETUP
	graph = AStarGraph()

	result = []
	path_nodes = [(0,0), (25,0), (25,5), (0,5), (0,10), (25,10), (25,15), (0,15), (0,20), (25,20)]
	for i in range(len(path_nodes)-1):
		start = path_nodes[i]#len(path_nodes)-1-i]
		goal = path_nodes[i+1]#len(path_nodes)-2-i]
		path, cost = AStarSearch(start, goal, graph)
		result += path

	# print ("route", result)
	# print ("cost", cost)

	#c = ['b' for v in result]
	# c[0] = 'g'
	# c[-1] = 'r'
	c = ['g','r']

	plt.plot([v[0]+x_min for v in result], [v[1]+y_min for v in result])
	plt.scatter([v[0]+x_min for v in [result[0],result[-1]]], [v[1]+y_min for v in [result[0],result[-1]]], c=c, marker='o')
	plt.show()
 
class AStarGraph(object):
	#Define a class board like grid with two barriers
 
	def __init__(self):
		self.barriers = []
		#self.barriers.append([(2,4),(2,5),(2,6),(3,6),(4,6),(5,6),(5,5),(5,4),(5,3),(5,2),(4,2),(3,2)])
 
	def heuristic(self, start, goal):
		#Use Chebyshev distance heuristic if we can move one square either
		#adjacent or diagonal
		D = 1
		D2 = 1
		dx = abs(start[0] - goal[0])
		dy = abs(start[1] - goal[1])
		# return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

		if COST == 'time':
			return 0#np.sqrt((goal[0]-start[0])**2 + (goal[1]-start[1])**2)/(V_max + V_fm)
		elif COST == 'energy':
			return 0
			
 
	def get_vertex_neighbours(self, pos):
		n = []
		#Moves allow link a chess king
		for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]:
			x2 = pos[0] + dx
			y2 = pos[1] + dy
			# if x2 < 0 or x2 > 7 or y2 < 0 or y2 > 7:
			# 	continue
			n.append((x2, y2))
		return n
 
	def move_cost(self, a, b):
		if a[0] >= n or a[0] < 0:
			return 100
		if a[1] >= m or a[1] < 0:
			return 100 #Extremely high cost to enter barrier squares

		d = np.asarray([b[0]-a[0], b[1]-a[1]])
		V_fx = vx[a[1]][a[0]] #Check this?
		V_fy = vy[a[1]][a[0]]
		V_f = np.asarray([V_fx, V_fy])

		if np.linalg.norm(V_f) == 0:
			#print('cant divide by 0')
			theta = 0
		else:
			theta = float(np.dot(d, V_f))/(np.linalg.norm(d)*np.linalg.norm(V_f))
		dx = np.linalg.norm(d)*np.cos(theta)
		dy = np.linalg.norm(d)*np.sin(theta)
		V_f = np.linalg.norm(V_f)

		#Time minimizing cost
		temp = (V_max**2)*(dx**2 + dy**2) - (V_f**2)*(dy**2)
		if temp < 0:
			# print('Cant fight current')
			return 100
		dt = (V_f/(V_f**2 - V_max**2))*dx - np.sqrt(temp) / (V_f**2 - V_max**2)

		#Energy minimizing cost
		de = ((dx**2 + dy**2)/(dt**2) - 2*V_f*(dx/dt) + V_f**2)*dt
		
		if COST == 'time':
			return dt
		else:
			return de
 
def AStarSearch(start, end, graph):
 
	G = {} #Actual movement cost to each position from the start position
	F = {} #Estimated movement cost of start to end going via this position
 
	#Initialize starting values
	G[start] = 0 
	F[start] = graph.heuristic(start, end)
 
	closedVertices = set()
	openVertices = set([start])
	cameFrom = {}
 
	while len(openVertices) > 0:
		#Get the vertex in the open list with the lowest F score
		current = None
		currentFscore = None
		for pos in openVertices:
			if current is None or F[pos] < currentFscore:
				currentFscore = F[pos]
				current = pos
 
		#Check if we have reached the goal
		if current == end:
			#Retrace our route backward
			path = [current]
			while current in cameFrom:
				current = cameFrom[current]
				path.append(current)
			path.reverse()
			return path, F[end] #Done!
 
		#Mark the current vertex as closed
		openVertices.remove(current)
		closedVertices.add(current)
 
		#Update scores for vertices near the current position
		for neighbour in graph.get_vertex_neighbours(current):
			if neighbour in closedVertices: 
				continue #We have already processed this node exhaustively
			candidateG = G[current] + graph.move_cost(current, neighbour)
 
			if neighbour not in openVertices:
				openVertices.add(neighbour) #Discovered a new vertex
			elif candidateG >= G[neighbour]:
				continue #This G score is worse than previously found
 
			#Adopt this G score
			cameFrom[neighbour] = current
			G[neighbour] = candidateG
			H = graph.heuristic(neighbour, end)
			F[neighbour] = G[neighbour] + H
 
	raise RuntimeError("A* failed to find a solution")
 
if __name__=="__main__":
	main()