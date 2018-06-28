from ASV_state import *
from ASV_sim_robot import *
import serial
import time
from xbee import XBee
from digi.xbee.devices import XBeeDevice

### This module controls the communication between Central and ASV,
### path planning (?), and data logging

class Central_Controller:
    
    def __init__(self):
            
        # create vars for hardware vs simulation
        self.controller_mode = "SIMULATION MODE"#"SIMULATION MODE" or "HARDWARE MODE"

        self.num_robots = 1
        self.robots = []
        self.log_names = []

        self.tkimage = []
        self.image = []

        if self.controller_mode == "HARDWARE MODE":
            # setup xbee communication
            self.local_xbee = XBeeDevice("/dev/tty.usbserial-DN02Z6QY", 9600)
            self.local_xbee.open()  
            self.last_queue_clear = time.time()
            print(self.local_xbee)

            # Obtain the Xbee network which contains all the ASVs
            self.xbee_network = self.local_xbee.get_network()
            print(self.xbee_network)

            # Setup the robots xbees
            for i in range (0,self.num_robots):
                robot = ASV_state(i,0,0,0)
                node_name = "ASV_" + str(i)
                r = self.xbee_network.discover_device(node_name)

                if r is None:
                    print("Could not find the remote device")
                    try:
                        r = self.xbee_network.discover_device("ASV")
                        print("Trying to find remote device")
                    except:
                        self.local_xbee.close() 
                        exit(1)

                robot.xbee = r

                self.robots.append(robot)
                self.make_headers(i)
        else:

            # Setup the sim robots
            for i in range (0,self.num_robots):
                r = ASV_sim_robot(i)
                robot = ASV_state(i,0,0,0)
                robot.sim_bot = r
                self.robots.append(robot)
                self.make_headers(i)


    def update_calibration(self, robot):
        if self.controller_mode == "HARDWARE MODE":
            transmit_data = ("C " + str(self.robots[robot]["Offset"]) + " " + str(self.robots[robot]["Origin"][0]) +  " " + str(self.robots[robot]["Origin"][1])).encode()
            print("Transmitted Data: ", transmit_data)
            self.local_xbee.send_data_async(self.robots[robot]["Xbee"], transmit_data)
        else:
            sim_robot = self.robots[robot].sim_robot
            sim_robot.heading_offset = self.robots[robot].offset
            sim_robot.origin = self.robots[robot].origin
        

    
    def update_robots(self, deltaT):

        #print("Manual Ctrl: ", self.robots[0]["Manual Ctrl"][0], self.robots[0]["Manual Ctrl"][1])

        
        # loop over all robots and send the control signal through the xbee
        for i in range (0,self.num_robots):
            
            if self.controller_mode == "HARDWARE MODE":
                # Clear recieved xbee messages every .2 seconds to prevent compounding latency
                if time.time() - self.last_queue_clear > .2:
                    self.local_xbee.flush_queues()
                    self.last_queue_clear = time.time()

                if self.robots[i]["Mode"] == "MANUAL MODE":
                    transmit_data = ("M " + str(self.robots[i].manual_command[0]) +  " " + str(self.robots[i].manual_command[1])).encode()
                else:
                    transmit_data = ("A " + str(self.robots[i].current_dest[0]) +  " " + str(self.robots[i].current_dest[1]) + " " + str(self.robots[i].current_dest[2])).encode()

                print("Transmitted Data: ", transmit_data)
                self.local_xbee.send_data_async(self.robots[i].xbee, transmit_data)

                receive_msg = self.local_xbee.read_data()

                if receive_msg != None:
                    receive_data = receive_msg.data.decode("utf-8").split(" ")
                    print("Received Data: ", receive_data)
                    self.robots[i].x = 0
                    self.robots[i].y = 0
                    self.robots[i].theta = 0
                    # self.robots[i]["Est."] = [float(data) for data in receive_data[0:3]]
                    # self.robots[i]["Sensor"] = [float(data) for data in receive_data[3:]]

            else:
                sim_robot = self.robots[i].sim_bot

                if self.robots[i]["Mode"] == "MANUAL MODE":
                    sim_robot.control_mode = self.robots[i].mode
                    sim_robot.manual_control_left_motor = self.robots[i].manual_command[0]
                    sim_robot.manual_control_right_motor = self.robots[i].manual_command[1]
                else:
                    sim_robot.control_mode = self.robots[i].mode
                    sim_robot.state_des.set_state(self.robots[i].current_dest[0], self.robots[i].current_dest[1], self.robots[i].current_dest[2])

                sim_robot.update(deltaT)

                receive_data= sim_robot.get_data().split(" ")
                self.robots[i].x = 0
                self.robots[i].y = 0
                self.robots[i].theta = 0
                self.robots[i]["Sensor"] = [0,0,0,0]
                # self.robots[i]["Est."] = [float(data) for data in receive_data[0:3]]
                # self.robots[i]["Sensor"] = [float(data) for data in receive_data[3:]]

            self.log_data(i)

    def make_headers(self, robot):

        file_name = 'Log/Bot' + str(robot) + '_' + datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S') + '.txt'
        self.log_names.append(file_name)
        f = open(file_name, 'a+')
        f.write('{0} {1:^1} {2:^1} {3:^1} {4:^1} {5:^1} {6:^1} {7:^1} {8:^1} {9:^1} {10:^1} \n'.format('Mode', 'X_est', 'Y_est', 'Theta_est', 'X_des', 'Y_des', 'Theta_des','IMU_Heading', 'Num_Sats', 'Lat', 'Lon'))
        f.close()

        
    def log_data(self, robot):

        f = open(self.log_names[robot], 'a+')

        state_est_str = ' '.join(['%.2f' % x for x in self.robots[robot]["Est."]])
        state_des_str = ' '.join(['%.2f' % x for x in self.robots[robot]["Des."]])
        sensor_str = ' '.join(['%.2f'  % x for x in self.robots[robot]["Sensor"][0:2]]) + ' ' + ' '.join([str(x) for x in self.robots[robot]["Sensor"]][2:4])
        
        # edit this line to have data logging of the data you care about
        data = [str(x) for x in [self.robots[robot].mode , state_est_str, state_des_str, sensor_str]]
        
        f.write(' '.join(data) + '\n')
        f.close()
        

            
    def quit(self):
        print("Shutting sytem down...")
        for i in range(0, self.num_robots):
            print("Stopping robot ", i)
            self.robots[i].mode = "MANUAL MODE"
            self.robots[i].manual_command = [0, 0]

        self.update_robots(0.1)
        time.sleep(1)
        self.local_xbee.close() 
        exit(1)
            
            
            