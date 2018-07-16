from ASV_controller import *
from ASV_graphics import *

def main():
    # instantiate robot navigation classes
    controller = ASV_Controller()
    graphics = ASV_graphics(controller)

    # set time step size in seconds
    #deltaT = 0.1
    # loop over time
    while True:
        try:
            # update graphics, but stop the thread if user stopped the gui
            if not graphics.update():
                break

            # maintain timing
            #time.sleep(deltaT)

        except (KeyboardInterrupt, SystemExit):
            print("Shutting sytem down...")
            controller.local_xbee.del_data_received_callback(controller.data_received_callback)
            end_msg = "STOP".encode()
            controller.local_xbee.send_data_async(controller.boat_xbee,end_msg)
            controller.local_xbee.close()
            break

    controller.quit()

if __name__ == '__main__':
    main()