from Central_controller import *
from Central_graphics import *

def main():
	
	# instantiate robot navigation classes
    central = Central_Controller()
    graphics = Central_graphics(central)
    
    # set time step size in seconds
    deltaT = 0.1
    # loop over time
    while True:
        try:

            # update graphics, but stop the thread if user stopped the gui
            if not graphics.update():
                break
            
            # update robots
            central.update_robots(deltaT)
        
            # maintain timing
            time.sleep(deltaT)

        except (KeyboardInterrupt, SystemExit):
            print("Shutting sytem down...")
            for i in range(0, central.num_robots):
                print("Stopping robot ", i)
                central.robots[i]["Mode"] = "MANUAL CONTROL"
                central.robots[i]["Manual Ctrl"] = [0, 0]

            central.local_xbee.close()
            raise

if __name__ == '__main__':
	main()