from ASV_state import *
import serial
import time
from xbee import XBee
from digi.xbee.devices import XBeeDevice
import ASV.ASV_robot as ASV_robot
import ASV.ASV_environment as ASV_environment
import utm

### This module controls the communication between Central and ASV,
### path planning (?), and data logging
class ASV_Controller:
    
    def __init__(self):
        # robot state
        self.robot_state = ASV_state()

        self.mode = "SIM MODE"
        self.ASV_sim = None

        # ADCP measuremens
        self.depth = 0.0
        self.v_boat = 0.0

        if self.mode == "HARDWARE MODE":
            # initialize xbee
            ser = serial.Serial("/dev/tty.usbserial-DN02Z6QY", 9600)
            ser.flush()
            ser.close()
            self.local_xbee = XBeeDevice("/dev/tty.usbserial-DN02Z6QY", 9600)
            self.local_xbee.open()
            self.boat_xbee = None # no ASV xbee yet

            # Setting up Xbee communication
            self.local_xbee.add_data_received_callback(self.data_received_callback)
            self.discover_boat(self.local_xbee)
            
            if self.boat_xbee == None:
                print('device not found!')
                return
            else:
                print('device found! Sending start messages')

            # Sending start command to the boat
            start_msg = "START".encode()
            self.local_xbee.send_data_async(self.boat_xbee, start_msg)
        
        elif self.mode ==  "SIM MODE":
            self.sim_env = ASV_environment.ASV_sim_env()
            self.robot = ASV_robot.ASV_sim(self.sim_env)
            
            # Specify robot location (50,50) in pixel coordinates
            self.robot.actual_state.x = 396168.0
            self.robot.actual_state.y = 3777912.0
            self.robot.actual_state.theta = math.pi/4
            self.robot.estimate_state()

    ###############################################################################
    # XBEE Setup Functions
    ###############################################################################

    '''
    Discover the boat xbee

    Input: 
        xbee(XBeeDevice): local XbeeDevice
    Output:
        boat_xbee(XBeeDevice): None if there is nothing
    '''
    def discover_boat(self, xbee):
        print('Discovering device')
        xbee_network = xbee.get_network()
        xbee_network.start_discovery_process()
        while xbee_network.is_discovery_running():
            time.sleep(0.5)
        self.boat_xbee = xbee_network.discover_device('boat')

    '''
    Call back function whenever a message arrived. Sort out GPS
    and ADCP information

    Input:
        xbee_message: messages from the other xbee
    '''
    def data_received_callback(self, xbee_message):
        try:
            address = xbee_message.remote_device.get_64bit_addr()
            data = xbee_message.data.decode()
            parsed_data = data.split(',')

            if parsed_data[0] == '!DATA':
                #print('Received GPS: ', data)
                #TODO: update GPS data
                self.robot.state_est.x = float(parsed_data[1])
                self.robot.state_est.y = float(parsed_data[2])
                self.robot.state_est.theta = float(parsed_data[3])
                self.depth = float(parsed_data[4])



        except KeyboardInterrupt:
            print("I'm here!")
            end_msg = "STOP".encode()
            self.local_xbee.send_data_async(self.boat_xbee,end_msg)

    def quit(self):
        if self.mode == "HARDWARE MODE":
            print("Shutting sytem down...")
            end_msg = "STOP".encode()
            self.local_xbee.send_data_async(self.boat_xbee,end_msg)
            time.sleep(1)
            self.local_xbee.close() 
            exit(1)
        else:
            exit(1)

    ################################################################################
    # Data Logging
    ################################################################################
    
    # TODO: log GPS/ADCP data to 1 file
    def log_data(data):
        pass

        
            
            
            