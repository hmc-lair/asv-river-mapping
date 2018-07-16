from ASV.ASV_environment import *
from ASV.ASV_robot import *

#######################################
# This file is to be run on the ASV 
# with the rest of the ASV submodule
#######################################


def main():
	env = ASV_environment()
	robot = ASV_robot(env)

	robot.robot_setup()

	
	while True:
		try:
			if robot.terminate == True:
				robot.main_loop()
				break
			else:
				robot.main_loop()
		except KeyboardInterrupt:
			robot.terminate = True
			robot.main_loop()
			break

	# print('terminated')
	# quit()


if __name__ == '__main__':
	main()