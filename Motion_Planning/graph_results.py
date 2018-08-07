#Graphing functions using matplotlib
import matplotlib.pyplot as plt

def graphResults(N, bestPath):
	colorList = ['red', 'green', 'cyan', 'magenta', 'white', 'yellow', 'blue', 'black']

	X = [[] for i in range(N)]
	Y = [[] for i in range(N)]

	for i in range(len(bestPath)):
		for j in range(N):
			X[j].append(bestPath[i][j][0])
			Y[j].append(bestPath[i][j][1])

	for i in range(N):
		plt.plot(X[i],Y[i], '-o', color=colorList[i%8],zorder=3)

	plt.show()