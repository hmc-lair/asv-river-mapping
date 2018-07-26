from ASV.ASV_environment import *
from ASV_state import *
import datetime
import math
import utm
import threading
import time
import struct
import numpy.random
import numpy as np
from digi.xbee.exception import XBeeException

class ASV_robot:

    def __init__(self, environment):
        self.environment = environment
        self.state_est = ASV_state()

        # angle offset
        self.adcp_angle_offset = 45 # degrees
        self.speed_des = 0.5

        # Way points to navigate
        self.cur_des_point = ASV_state()
        self.cur_des_point.set_state(1,1,1)
        self.way_points = []
        self.des_reached = True
        self.dist_threshold = 3
        self.dt = 0.01

        self.motor_stop = False

        # Controller Params
        self.Kp_ang = 20
        self.Kp_nom = 500
        self.Kd = 0
        self.Ki = 0
        self.last_uL = 0.0
        self.last_uR = 0.0
        self.last_ang_error = 0.0
        self.last_ang_error2 = 0.0

        # robot commands
        self.rudder = 0.0
        self.R_motor = 0.0
        self.L_motor = 0.0
        self.back_spin_threshold = 700
        self.forward_threshold = 500

        # GPS Data and Coordinate
        self.GPS_fix_quality = 0
        self.GPS_raw_msg = ''
        self.GPS_Time = ''
        self.utm_x = 0
        self.utm_y = 0
        self.GPS_received = False
        self.GPS_speed = 0
        self.GPS_course = 0 # course over ground in degrees '0 - 359'

        self.beam_angle = 20

        # magnetometer data
        self.roll = 0.0
        self.pitch = 0.0
        self.heading = 0.0
        self.mag_received = True
        self.heading_offset = -20.0 #magnetic vs. true north

        # ADCP Data
        self.ADCP_roll = 0.0
        self.ADCP_pitch = 0.0
        self.ADCP_heading = 0.0
        self.v_boat = [0, 0]
        self.depths = [0, 0, 0, 0]
        self.bt_boat_beams = [0, 0, 0,0]
        self.relative_surface_velocities = [0,0,0,0]
        self.surface_velocities = [0,0,0,0]
        self.ADCP_received = False

        # threads
        self.GPS_thread = None
        self.ADCP_thread = None
        self.mag_thread = None
        self.check_xbee_thread = None

        self.bad_connection_limit = 52

        # Program termination
        self.terminate = False

        # Data logging
        time_stamp = datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S')
        
        # Not putting things in separate file for now
        # self.gps_filename = 'Log/GPS_' + time_stamp + ".txt"
        # self.gps_f = open(self.gps_filename, 'w')
        # self.adcp_filename = 'Log/ADCP_' + time_stamp + ".bin"
        # self.adcp_f = open(self.adcp_filename, 'wb')
        self.log_data = True
        if self.log_data == True:
            self.all_data_filename = 'Log/ALL_' + time_stamp + ".bin"
            self.all_data_f = open(self.all_data_filename, 'wb')
        # data to log: GPS, ADCP, heading

    def main_loop(self):
        '''main control loop for the ASV'''
        # 1. Predict asv position (maybe not now)
        # 2. If GPS/DVL/Compass available, update accordingly
        if self.GPS_received:
            self.localize_with_GPS()

        if self.ADCP_received:
            self.localize_with_ADCP()

        if self.mag_received:
            self.localize_with_mag()
            # print("Heading: ", self.angleDiff_deg(self.heading))
            # print("Theta:", self.state_est.theta)

        # 3. Motion plan
        # time.sleep(0.5)
        # Set destination 
        self.update_waypoint()
        
        # print("Current Destination: ", self.cur_des_point.x, self.cur_des_point.y)
        # print("Dest X Y: ", self.cur_des_point.x, self.cur_des_point.y)
        # print("robot x y: ", self.state_est.x, self.state_est.y)
        # print("Heading: ", self.heading)
        
        # Compute control signal
        port, strboard = self.point_track2(self.cur_des_point)

        print("Left %f, Right %f" % (-port, -strboard))
        # 4. Update control signals
        self.update_control(port, strboard)
        #self.update_control(-200, 300)
        #print("Left %f Right %f " % (-port, -strboard)) 

        # if (strboard > port):
        #     print("turning left")
        # elif (strboard < port):
        #     print("turning right")
        # self.update_control(strboard, port)

        time.sleep(0.2)

        # print("working?")
        self.report_to_shore()

        if self.terminate == True:
            self.robot_shutdown()

    def robot_setup(self):
        print('Setting up...')
        self.environment.disable_xbee = True

        # Setup Hardware
        if (self.environment.robot_mode == "HARDWARE MODE"):
            self.environment.setup_GPS()
            self.environment.setup_motors()
            self.environment.setup_ADCP()
            self.environment.setup_magnetometer()
            
            if self.environment.disable_xbee == True:
                print('xbee disabled')
            else:
                self.environment.dest_xbee = self.environment.discover_xbee()

        # Create Xbee call back for communication
        if self.environment.disable_xbee == True:
            pass
        else:
            if self.environment.dest_xbee == None:
                self.terminate = True
                self.robot_shutdown()
            else:
                self.environment.my_xbee.add_data_received_callback(self.xbee_callback)
                self.check_xbee_thread = threading.Thread(name = 'Xbee Thread', target = self.check_xbee)
                self.check_xbee_thread.setDaemon(True)
                self.check_xbee_thread.start()

        # Initialize GPS thread
        self.GPS_thread = threading.Thread(name = 'GPS Thread', target = self.update_GPS)
        self.GPS_thread.setDaemon(True)

        # Initilialize ADCP thread
        self.ADCP_thread = threading.Thread(name = 'ADCP Thread', target = self.update_ADCP)
        self.ADCP_thread.setDaemon(True)

        # Initialize Magnetometer thread
        self.mag_thread = threading.Thread(name = 'ADCP Thread', target = self.update_mag)
        self.mag_thread.setDaemon(True)

        print('Starting main loop...')
        
        # Begin threads
        self.GPS_thread.start()
        self.environment.start_ping()
        self.ADCP_thread.start()
        self.mag_thread.start()

    def robot_shutdown(self):
        self.update_control(0, 0)
        if self.environment.disable_xbee != True:
            self.environment.my_xbee.close()

    def update_control(self, L, R):
        '''Send command messages to the ASV'''
    
        # print("Starboard: %f Port: %f" % (L, R))
        if self.motor_stop == True:
            motor_L = 0
            motor_R = 0
        else:
            motor_L = L
            motor_R = R

        left_command = '!G 1 %d' % motor_L
        right_command = '!G 1 %d' % motor_R

        self.environment.starboard_ser.write(right_command.encode() + b'\r\n')
        self.environment.port_ser.write(left_command.encode() + b'\r\n')

    def update_waypoint(self):
        ''' iterate through the set of way points'''
        if self.des_reached:
            if len(self.way_points) == 1:
                self.cur_des_point = self.cur_des_point
            else:
                if len(self.way_points) == 0:
                    return
                self.way_points = self.way_points[1:]
                self.cur_des_point = self.way_points[0]
                self.des_reached = False

    def add_way_point(self, way_point):
        '''add way point'''
        self.way_points.append(way_point)

    def clear_way_points(self):
        self.way_points = []

    def point_track(self, des_point):
        '''Generate motor values given a point and current state'''

        angle_offset = math.atan2(des_point.y - self.state_est.y,
                                des_point.x - self.state_est.x)
        distance = math.sqrt((des_point.y - self.state_est.y)**2 + 
                (des_point.x - self.state_est.x)**2)

        if distance <= self.dist_threshold or self.des_reached:
            # if self.des_reached == False: # Just print the first time destination reached
            # print('Destination reached! Turning off motors.')
            self.stop_motor = True
            self.des_reached = True
            u_starboard = 0.0
            u_port = 0.0
        else:
            # Simple heading PD control
            ang_error = self.angleDiff(angle_offset- self.state_est.theta)
            # print(ang_error)
            # print('ang_error', ang_error)
            # print(des_point.x, des_point.y)
            
            # PID control
            # uL = -(self.last_uL + ang_error *(self.Ki * self.dt + self.Kp + self.Kd/self.dt) + self.last_ang_error*(-self.Kp - 2 * self.Kd/self.dt) + self.last_ang_error2 * self.Kd/self.dt)
            # uR = self.last_uR + ang_error *(self.Ki * self.dt + self.Kp + self.Kd/self.dt) + self.last_ang_error*(-self.Kp - 2 * self.Kd/self.dt) + self.last_ang_error2 * self.Kd/self.dt
           
            # P control (working for pool!)
            # uL = -self.Kp * ang_error
            # uR = self.Kp * ang_error

            ### PI control (working for pool!)
            uL = self.last_uL - (ang_error * (self.Kp_ang + self.Ki * self.dt) - self.last_ang_error * self.Kp_ang)
            uR = self.last_uR + (ang_error * (self.Kp_ang + self.Ki * self.dt) - self.last_ang_error * self.Kp_ang)
            self.last_ang_error = ang_error
            self.last_ang_error2 = self.last_ang_error
            self.last_uL = uL
            self.last_uR = uR
            u_nom = 150
            uR = uR * 100
            uL = uL * 100
            
            ### Kinetic Model (not working)
            # e_x = des_point.x - self.state_est.x 
            # e_y = des_point.y - self.state_est.y
            # v_x_des = 0
            # v_y_des = 0
            # current_x = 0.2
            # current_y = 0
            # u_nom = e_x * math.cos(self.state_est.theta) + e_y * math.sin(self.state_est.theta) + v_x_des * math.cos(self.state_est.theta) + v_y_des * math.sin(self.state_est.theta) + current_x * math.cos(self.state_est.theta) - current_y * math.sin(self.state_est.theta)
            # r = e_x * math.sin(self.state_est.theta) - e_y * math.cos(self.state_est.theta) + v_x_des * math.sin(self.state_est.theta) - v_y_des * math.cos(self.state_est.theta) + current_x * math.sin(self.state_est.theta) - current_y * math.cos(self.state_est.theta)
            # u_nom = u_nom * 10
            # uR = -r/2 * 100
            # uL = r/2 * 100

            # percent_reached = (math.pi - abs(ang_error))/math.pi
            # if percent_reached <= 0.7:
            #     u_nom = 0
            # else:
             #percent_reached * 150
            # if (abs(ang_error) >= 0.2):
            #     u_nom = 0
            
            # uR = uR * 100
            # uL = uL * 100
            # print("angle error %f" % ang_error)
            # print("u_nom %f" % u_nom)
            # print("ur %f, ul %f" % (uR, uL))
            # cap the distance
            u_starboard = -min(max(u_nom + uR, -self.back_spin_threshold), self.forward_threshold)
            u_port = -min(max(u_nom + uL, -self.back_spin_threshold), self.forward_threshold)

        return u_port, u_starboard

    def point_track2(self, des_point):
        ''' Point tracker that uses ASV's course velocity instead of heading'''
        speed_des = self.speed_des

        # Find robot velocity (make this into asv state later)
        robot_vx = self.state_est.v_course * math.cos(self.state_est.ang_course)
        robot_vy = self.state_est.v_course * math.sin(self.state_est.ang_course)
        
        # calculate desired angle displacement
        des_angle = math.atan2(des_point.y - self.state_est.y,
                                des_point.x - self.state_est.x)
        
        # Find angle difference between desired trajectory and current trajectory
        v_x_des = speed_des * math.cos(des_angle)
        v_y_des = speed_des * math.sin(des_angle)
        robot_v = np.array([robot_vx, robot_vy])
       
        v_des_ang = math.atan2(v_y_des, v_x_des)
        robot_v_ang = math.atan2(robot_vy, robot_vx)
        angle_off = self.angleDiff(v_des_ang - robot_v_ang)
        # if abs(angle_off) > 1.0:
        #     # make sure it doesn't go over or below 1
        #     angle_off = abs(angle_off)/angle_off * 1.0

        # # acos is only positive so check which trajectory is on top
        # angle_off = math.acos(float(angle_off))

        # if self.angleDiff(v_des_ang - robot_v_ang) < 0:
        #     angle_off = -angle_off
        course_speed = math.sqrt(np.dot(robot_v,robot_v))
        # print("vx: %f, vy: %f" % (robot_vx, robot_vy))
        # print("v_desx: %f, v_desy: %f" % (v_x_des, v_y_des))
        # print("Angle off: ", angle_off)
        # print("Speed :" , course_speed)

        ### Use current to predict offset
        # Current displacement vector
        # current_x = self.state_est.current_v * math.cos(self.state_est.current_ang)
        # current_y = self.state_est.current_v * math.sin(self.state_est.current_ang)
        # cur_displacement_x = current_x 
        # cur_displacement_y = current_y 
        # cur_displacement = np.array([cur_displacement_x, cur_displacement_y])

        # # Distance displacement vector
        # des_direction = np.array([self.state_est.x - des_point.x, self.state_est.y - des_point.y])
        # new_direction = des_direction - cur_displacement
        # angle_off = np.dot(new_direction, des_direction)/(np.sqrt(np.dot(new_direction, new_direction)) * np.sqrt(np.dot(des_direction, des_direction)))
        # if abs(angle_off) > 1.0:
        #     angle_off = abs(angle_off)/angle_off * 1.0
        # angle_off = math.acos(float(angle_off))
        # new_angle = self.angleDiff(des_angle - angle_off)
        # des_angle = des_angle
        # print("Des direction ", des_direction)
        # print("New direction ", new_direction)
        # print("Desired Angle %f, New Angle %f" % (des_angle, new_angle))

        distance = math.sqrt((des_point.y - self.state_est.y)**2 + 
                (des_point.x - self.state_est.x)**2)

        if distance <= self.dist_threshold or self.des_reached:
            # if self.des_reached == False: # Just print the first time destination reached
            # print('Destination reached! Turning off motors.')
            self.stop_motor = True
            self.des_reached = True
            u_starboard = 0.0
            u_port = 0.0
        else:
            # Simple heading PD control
            ang_error = self.angleDiff(des_angle- self.state_est.theta)
            ang_error = angle_off
            # print(des_point.x, des_point.y)
            
            uL = self.last_uL - (ang_error * (self.Kp_ang + self.Ki * self.dt) - self.last_ang_error * self.Kp_ang)
            uR = self.last_uR + (ang_error * (self.Kp_ang + self.Ki * self.dt) - self.last_ang_error * self.Kp_ang)
            

            self.last_ang_error = ang_error
            self.last_ang_error2 = self.last_ang_error
            self.last_uL = uL
            self.last_uR = uR
            u_nom = (speed_des - course_speed) * self.Kp_nom + 100
            print("u_nom: ", u_nom)
            print("current speed: ", self.state_est.v_course)
            uL = uL * 50
            uR = uR * 50
            # if uR > 0:
            #     uR = uR * 80
            # else:
            #     uR = uR * 30

            # if uL > 0:
            #     uL = uL * 80
            # else:
            #     uL = uL * 30
            

            u_starboard = -min(max(u_nom + uR, -self.back_spin_threshold), self.forward_threshold)
            u_port = -min(max(u_nom + uL, -self.back_spin_threshold), self.forward_threshold)


        return u_port, u_starboard

    def log_data(self):
        '''Log relevant data'''
        pass

    def localize_with_GPS(self):
        '''state estimate with GPS. For now it's just setting the x and y to 
        the utm x and y value '''
        # 1. Update position with GPS
        self.state_est.x = self.utm_x
        self.state_est.y = self.utm_y
        self.GPS_received = False

    def localize_with_mag(self):
        '''state estimate with magnetometer. For now it's setting the heading and 
        angle data with the magnetometer '''
        self.state_est.theta = self.angleDiff((self.heading)/180 * math.pi)
        self.state_est.roll = self.angleDiff(self.roll/180 * math.pi)
        self.state_est.pitch = self.angleDiff(self.pitch/180 * math.pi)
        self.mag_received = False

    def predict(self):
        '''Predict with just the internal states of the robot'''
        self.state_est.x = self.state_est.x + self.state_est.v * math.cos(self.state_est.theta) * self.dt
        self.state_est.y = self.state_est.x + self.state_est.v * math.sin(self.state_est.theta) * self.dt
        self.state_est.theta = self.state_est.theta + self.state_est.omega * self.dt

    def localize_with_ADCP(self):
        '''state estimate with DVL'''
        # self.state_est.theta = self.heading / 180 * math.pi
        # self.v_boat[0], self.v_boat[1] = self.convert_bt_velocities(self.roll, self.pitch, self.heading, self.bt_boat_beams)
        self.v_boat[0] = self.bt_boat_beams[0]
        self.v_boat[1] = self.bt_boat_beams[1]

        self.surface_velocities[0] = self.v_boat[0] - self.relative_surface_velocities[0]
        self.surface_velocities[1] = self.v_boat[1] - self.relative_surface_velocities[1]
        # print("Surface Velocities: ", self.surface_velocities)
        # print("Bt velocities: ", self.v_boat)
        self.ADCP_received = False

    def update_odometry(self):
        '''Given DVL velocities update odometry'''
        pass    


###############################################################################
# XBEE Functions
###############################################################################
    def xbee_callback(self, xbee_message):
        data = xbee_message.data.decode()
        parsed_data = data.split(',')
        print(data)
        if parsed_data[0] == "!QUIT":
            self.terminate = True
            print("Stop message received. Terminating...")
        elif parsed_data[0] == "!CLEARWPS":
            self.clear_way_points()
            self.des_reached = True
        elif parsed_data[0] == "!WP":
            wp = ASV_state()
            wp.x = float(parsed_data[1]) # way point in local x, y
            wp.y = float(parsed_data[2]) 
            self.add_way_point(wp)
            self.des_reached = False
        elif parsed_data[0] == "!STARTMISSION":
            self.cur_des_point = self.way_points[0]
            self.des_reached = False
        elif parsed_data[0] == "!ABORTMISSION":
            self.way_points = []
            self.motor_stop  = True
        elif parsed_data[0] == "!STOP":
            self.motor_stop = True
            print('Stop message received. Motors stopped.')
        elif parsed_data[0] == "!START":
            self.motor_stop = False
            print('Start message received. Motors re-started.')
        elif parsed_data[0] == "!HEADINGOFFSET":
            self.heading_offset = float(parsed_data[1])
            print('Heading message received. Angle offsetted')
        elif parsed_data[0] == "!SETSPEED":
            self.speed_des = float(parsed_data[1])
        elif parsed_data[0] == "!GAIN":
            self.Kp_ang = float(parsed_data[1])
            self.Kp_nom = float(parsed_data[2])
            self.forward_threshold = float(parsed_data[3])
            self.back_spin_threshold = float(parsed_data[4])
        else:
            print('Unknown command!')

        return data

    def report_to_shore(self):
        '''Send information back to shore'''
        if self.environment.disable_xbee == True:
            pass
        else:
            if self.terminate == False:
                average_depth = sum(self.depths)/len(self.depths)
                current_speed = math.sqrt(self.v_boat[0]**2 + self.v_boat[1]**2)
                msg = "!DATA, %f, %f, %f, %f, %f, %f" % (self.state_est.x, self.state_est.y, self.heading, average_depth, current_speed)
                msg_binary = msg.encode()
                self.environment.my_xbee.send_data_async(self.environment.dest_xbee, msg_binary)

    def check_xbee(self):
        '''Keep checking if the xbee is connected'''
        self.environment.my_xbee.set_sync_ops_timeout(10)
        bad_count = 0
        while True:
            if self.terminate == True:
                break
            else:
                print("Checking")
                try:
                    self.environment.my_xbee.send_data(self.environment.dest_xbee, b'Hi')
                    bad_count = 0
                except XBeeException:
                    print("bad response")
                    time.sleep(1)
                    if bad_count >= self.bad_connection_limit:
                        self.terminate = True
                        break
                    bad_count += 1

###############################################################################
# Magnetometer Functions
###############################################################################
    def update_mag(self):
        while True:
            if self.terminate == True:
                break
            else:
                # print("In mag loop: ")
                data_str = self.environment.mag_ser.readline()
                # print(data_str)
                mag_data =data_str.decode().split(',')
                # self.all_data_f.write(data_str)
                self.heading = float(mag_data[1]) + 90 + self.heading_offset
                self.pitch = float(mag_data[2])
                self.roll = float(mag_data[3])
                self.mag_received = True

###############################################################################
# GPS Functions
###############################################################################
    def update_GPS(self):
        # TODO: if no lock add a hard stop
        while True:
            if self.terminate == True:
                break
            else:
                if self.environment.GPS_ser.in_waiting != 0:
                    data_str = self.environment.GPS_ser.readline()

                    # Data Logging
                    if self.log_data == True:
                        self.all_data_f.write("$GPS,".encode() + data_str + b'###')
                        # self.gps_f.write(data_str.decode())
                    self.process_GPS(data_str)

        # self.gps_f.close()
        self.environment.GPS_ser.close()

    def process_GPS(self, data):
        ''' Convert GPS (lat, lon) -> UTM -> Local XY '''
        data_decoded = data.decode()
        # data_decoded = '$GPGGA,194502.00,3526.9108198,N,11854.8502196,W,2,15,0.8,142.610,M,-29.620,M,7.0,0131*78'
        raw_msg = data_decoded.split(',')
        if raw_msg[0] == '$GPGGA':
            self.GPS_fix_quality = raw_msg[6]
            #print(raw_msg)
            if self.GPS_fix_quality == '0' or self.GPS_fix_quality == '\x00' or self.GPS_fix_quality == '\x000':
                self.GPS_received = False
                # print('Bad GPS, no fix :(')
            else:
                if "\x00" in raw_msg[2]:
                    return
                if "\x00" in raw_msg[4]:
                    return
                self.GPS_raw_msg = raw_msg
                self.GPS_Time = raw_msg[1]
                self.state_est.lat = self.str_to_coord(raw_msg[2])
                self.state_est.lon = -self.str_to_coord(raw_msg[4])
                # print("lat lon:", self.state_est.lat, self.state_est.lon)
                self.utm_x, self.utm_y, _, _ = utm.from_latlon(self.state_est.lat, self.state_est.lon)       
                self.state_est.x = self.utm_x
                self.state_est.y = self.utm_y
                self.GPS_received = True
                print('GPS: ', self.state_est.x, self.state_est.y)
        elif raw_msg[0] == '$GPVTG':
            # print(raw_msg)
            if len(raw_msg) < 10:
                return
            if raw_msg[9] == 'N' or raw_msg[1] == "":
                # not valid data
                return
            else:
                # print(raw_msg)
                speed_msg = ''
                course_msg = ''
                if "\x00" in raw_msg[7]:
                    for i in range(len(raw_msg[7])):
                        if raw_msg[7][i] == "\x00":
                            speed_msg += "0"
                        else:
                            speed_msg += raw_msg[7][i]
                else:
                    speed_msg = raw_msg[7]

                if "\x00" in raw_msg[1]:
                    for i in range(len(raw_msg[1])):
                        if raw_msg[1][i] == "\x00":
                            course_msg += "0"
                        else:
                            course_msg += raw_msg[1][i]
                else:
                    course_msg = raw_msg[1]

                self.GPS_speed = float(speed_msg)*1000/3600 # from km/hr to m/s
                self.GPS_course = float(course_msg) # course over ground in degrees
                self.state_est.v_course = self.GPS_speed
                self.state_est.ang_course = self.angleDiff((-self.GPS_course + 360 + 90)/180.0 * math.pi)
                # self.GPS_course = (-self.state_est.ang_course + 90)/180 * math.pi

                # print("Speed %f, Course %f" % (self.state_est.v_course, self.state_est.ang_course))



    def str_to_coord(self, coord_str):
        per_index = coord_str.find('.')
        if per_index == 4:
            coord_str = '0' + coord_str #Add 0 to front
        # print("Coord string: ", coord_str)
        # print('Coord len: ', len(coord_str))
        deg = int(coord_str[:3])
        minutes = float(coord_str[3:])/60
        return deg + minutes

###############################################################################
# ADCP Functions
###############################################################################
    
    def update_ADCP(self):
        while True:
            if self.terminate == True:
                break
            else:
                ensemble = self.read_ensemble(verbose=False)
                data = self.extract_data(ensemble)
                self.ADCP_heading = self.angleDiff_deg(data[0] - self.adcp_angle_offset)
                self.ADCP_roll = data[1]
                self.ADCP_pitch = data[2]
                self.depths = data[3:7]
                self.bt_boat_beams = data[7:11]
                self.relative_surface_velocities = data[11:15]
                self.ADCP_received = True
                # print(self.bt_boat_beams)

        self.environment.stop_ping()
        # self.adcp_f.close()
        if self.log_data == True:
            self.all_data_f.close()
        
        self.environment.ADCP_ser.close()

    def read_ensemble(self,verbose=False):
        '''Read an ensemble of ADCP data. Then log it in a file'''
        header = self.environment.ADCP_ser.read(2)
        if header != b'\x7f\x7f':
            print('ERROR no header: ', header)
      
        num_bytes = self.environment.ADCP_ser.read(2)
        bytes_to_checksum = int.from_bytes(num_bytes, byteorder='little')-4
        if verbose:
            print('Num: ', bytes_to_checksum)
        
        data = self.environment.ADCP_ser.read(bytes_to_checksum)
        if verbose:
            print('Data: ', data)

        #use checksum to verify no errors
        checksum = self.environment.ADCP_ser.read(2)
        checksum_int = int.from_bytes(checksum, byteorder='little') 
        datasum_int = sum(b'\x7f\x7f' + num_bytes + data) % 2**16

        if checksum_int != datasum_int:
            print('ERROR: ', checksum_int, datasum_int)

        #read data to file
        all_data = b'\x7f\x7f' + num_bytes + data + checksum
        # self.adcp_f.write(all_data)

        current_state = [self.state_est.x, self.state_est.y, self.state_est.theta, self.state_est.roll, self.state_est.pitch]
        current_state_str = "$STATE," + ",".join(map(str,current_state)) + "###"
        
        # Data Logging
        if self.log_data  == True:
            self.all_data_f.write(current_state_str.encode())
            self.all_data_f.write("$ADCP,".encode() + all_data + b'###')

        return all_data

        '''
    Input:
        Ensemble: byte str ings of ensemble data
    Output:
        Array of values from the ADCP package
    '''
    def extract_data(self, all_data):
            num_types = all_data[5]
            
            # OFFSETS
            # 1. Fix Leader
            # 2. Variable Leader
            # 3. Velocity Profile
            # 6. Bottom Track
            # 9. Vertical Beam Range
            # 10-13. GPS
            offsets = []
            for i in range(num_types):
                offset = all_data[6+2*i:8+2*i]
                offset_int = int.from_bytes(offset, byteorder='little')
                offsets.append(offset_int)
            # print('Offsets: ', offsets)
            # print('Data IDs: ', [all_data[x:x+2] for x in offsets])

            # FIXED LEADER
            fixed_offset = offsets[0]

            num_beams = all_data[fixed_offset+8]
            num_cells = all_data[fixed_offset+9]
            pings_per_ensemble = int.from_bytes(all_data[fixed_offset+10: fixed_offset+12], byteorder='little')
            depth_cell_length = int.from_bytes(all_data[fixed_offset+12: fixed_offset+14], byteorder='little')
            coord_transform = all_data[fixed_offset+25]
            heading_alignment = int.from_bytes(all_data[fixed_offset+26:fixed_offset+28], byteorder='little')
            # print('Heading alignment: ', heading_alignment)

            # VARIABLE LEADER
            variable_offset = offsets[1]

            transducer_depth = int.from_bytes(all_data[variable_offset+16: variable_offset+18], byteorder='little')*0.1 #1 dm
            heading = int.from_bytes(all_data[variable_offset+18: variable_offset+20], byteorder='little')*0.01 #0 to 359.99
            pitch = int.from_bytes(all_data[variable_offset+20: variable_offset+22], byteorder='little', signed=True)*0.01 #-20 to 20
            roll = int.from_bytes(all_data[variable_offset+22: variable_offset+24], byteorder='little', signed=True)*0.01 #-20 to 20
            salinity = int.from_bytes(all_data[variable_offset+24: variable_offset+26], byteorder='little') #0 to 40 part per thousand
            temperature = int.from_bytes(all_data[variable_offset+26: variable_offset+28], byteorder='little')*0.01 #-5 to 40 degrees

            # VELOCITY PROFILE
            velocity_profile_offset = offsets[2]
            relative_velocities = []

            for i in range(num_cells):
                start_offset = velocity_profile_offset + 2 + 2*i
                # Average over beams
                vel = []
                for j in range(num_beams):
                    curVel = int.from_bytes(all_data[start_offset + 2*j: start_offset + 2 + 2*j], byteorder='little', signed=True)
                    vel.append(curVel)
                relative_velocities.append(vel)

            # BOTTOM TRACK (abbr. bt) (see page 154)

                # Coordinate system for velocity:
                    # 1. Earth Axis (default): East, North, Up (right hand orthogonal) 
                        # need to set heading alignment (EA), heading bias (EB) correctly
                        # make sure heading sensors are active (EZ)
                    # 2. Radial Beam Coordinates: "raw beam measurements" (not orthogonal)
                    # 3. Instrument coordinates: X, Y, UP, X is directon of beam 2, Y is beam 3. Compass
                        # measures the offset of Y from magnetic north
                    # 4. Ship coordinates: starboard, forward, mast (pitch, roll, yaw)

            bt_offset = offsets[5]
            bt_pings_per_ensemble = int.from_bytes(all_data[bt_offset+2:bt_offset+4], byteorder='little')
            bt_ranges = [] # ranges measurement for each beam (bt = 0 is bad data) # cm
            bt_velocities = [] # there are one more velocity data.. though not sure what it's for?
            beam_percent_good = []
            max_tracking_depth = int.from_bytes(all_data[bt_offset+70:bt_offset+72], byteorder = 'little')

            for i in range(4):
                bt_ranges.append(int.from_bytes(all_data[bt_offset+16+i*2:bt_offset+18+i*2], byteorder = 'little'))
                bt_velocities.append(int.from_bytes(all_data[bt_offset+24+i*2:bt_offset+26+i*2], byteorder = 'little'))
                beam_percent_good.append(all_data[bt_offset+40+i])
            
            # VERTICAL BEAM RANGE
            vb_offset = offsets[8]
            vb_range = int.from_bytes(all_data[vb_offset+4:vb_offset+8], byteorder = 'little') # in millimeter

            # GPS Data
            GPS_offsets = offsets[9:13]
            msg_types = []
            msg_sizes = []
            delta_times_bytes = []
            delta_times_double = [] # difference between GPS message and ensemble
            GPS_msg = []

            for g_offset in GPS_offsets:
                msg_size = int.from_bytes(all_data[g_offset+4:g_offset+6], byteorder = 'little')
                msg_types.append(int.from_bytes(all_data[g_offset+2:g_offset+4], byteorder = 'little'))
                msg_sizes.append(msg_size)
                delta_times_bytes.append(all_data[g_offset+6:g_offset+14])
                GPS_msg.append(all_data[g_offset+15: g_offset+15+msg_size])

            delta_times_double = [struct.unpack('d', b)[0] for b in delta_times_bytes] # convert to double
            surface_vel = relative_velocities[0]
            essential = [heading, roll, pitch]
            essential = essential + bt_ranges
            essential = essential + bt_velocities
            essential = essential + surface_vel
            return essential

    def convert_bt_velocities(self, roll, pitch, heading, bt_vels):
        bt_earth_vel = []

        roll_r = roll / 180 * math.pi
        pitch_r = pitch / 180 * math.pi
        heading_r = heading / 180 * math.pi

        v_x_R, v_y_R, v_z_R, v_error = beam2Instrument(self.beam_angle, bt_vels[i])
        v_x, v_y, v_z = instrument2Earth(roll_r, pitch_r, heading_r, v_x_R, v_y_R, v_z_R)

        return v_x, v_y

    def beam2Instrument(self, beam_angle, beams):
        ''' Convert beam coordinate to instrument coordinate
        see the teledyne beam transformation guide for definition'''
        c = 1 # convex transducer head
        a = 1/(2*math.sin(beam_angle))
        b = 1/(4*math.cos(beam_angle))
        d = a/math.sqrt(2)
        THREE_BEAM = False
        BAD_BEAM = False
        bad_count = 0
        for i in range(len(beams)):
            if abs(beams[i]) >= 32700:
                bad_count += 1

        if bad_count == 1:
            THREE_BEAM = True
        elif bad_count >= 2:
            BAD_BEAM = True

        
        if THREE_BEAM:
            vel_x = c * a * (beams[0] - beams[1])
            vel_y = c * a * (beams[0] + beams[1] - 2 * beams[2])
            vel_z = 2 * b * (beams[0] + beams[1])
            vel_error = 0
        elif BAD_BEAM:
            vel_x = 0
            vel_y = 0
            vel_z = 0
            vel_error = 0
        else:
            vel_x = c * a * (beams[0] - beams[1])
            vel_y = c * a * (beams[3] - beams[2])
            vel_z = b * (beams[0] + beams[1] + beams[2] + beams[3])
            vel_error = d * (beams[0] + beams[1] - beams[2] - beams[3])

        return vel_x, vel_y, vel_z, vel_error

    def instrument2Earth(self, roll, pitch, heading, v_x, v_y, v_z):
        ''' Convert velocities in the instrument frame to the earth frame '''
        rot_matrix1 = np.matrix([[math.cos(heading), math.sin(heading), 0],
                                [-math.sin(heading), math.cos(heading), 0],
                                [0, 0, 1]])
        rot_matrix2 = np.matrix([[1, 0, 0],
                                [0, math.cos(pitch), -math.sin(pitch)], 
                                [0, math.sin(pitch), math.cos(pitch)]])
        rot_matrix3 = np.matrix([[math.cos(roll), 0, math.sin(roll)], 
                                [0, 1, 0],
                                [-math.sin(roll), 0, math.cos(roll)]])

        inst2earth_rot = rot_matrix1 * rot_matrix2 * rot_matrix3

        v_inst = np.array([v_x, v_y, v_z])

        v_earth = v_inst * inst2earth_rot

        # print(v_earth[0][1])
        # v_earth([0])
        # v_earth = v_earth[0]
        # print('v_earth: ', v_earth)
        # print("v_earth 0:", v_earth[0])
        return v_earth.item((0,0)), v_earth.item((0,1)), v_earth.item((0,2))


    def angleDiff(self,angle):
        while angle > math.pi:
            angle = angle - 2 * math.pi
        while angle < -math.pi:
            angle = angle + 2 * math.pi

        return angle

    def angleDiff_deg(self, angle):
        while angle > 180:
            angle = angle - 2 * 180
        while angle < -180:
            angle = angle + 2 * 180

        return angle

class ASV_sim(ASV_robot):

    def __init__(self, environment):
        super().__init__(environment)
        self.dt = 0.1
        self.actual_state = ASV_state(0,0,0)

    def sim_loop(self):
        uR, uL = self.point_track2(self.cur_des_point)
        # print("Left %f Right %f " % (uL, uR))  
        # print(uR, uL)
        self.estimate_state()
        self.update_state(self.actual_state, uR, uL)

        self.update_waypoint()
        time.sleep(0.1)
        # self.gen_fake_data()

    def estimate_state(self):
        self.state_est = self.actual_state

    def gen_fake_data(self):
        fake_gps = '$GPGGA,194502.00,3526.9108198,N,11854.8502196,W,2,15,0.8,142.610,M,-29.620,M,7.0,0131*78'
        gps_msg = b'$GPS,' + fake_gps.encode() + b'###'

        f = open('sample_adcp.bin', 'rb')
        fake_ADCP = f.readline()
        adcp_msg = "$ADCP,".encode() + fake_ADCP + b'###'

        fake_state = [self.state_est.x, self.state_est.y, self.state_est.theta, self.state_est.roll, self.state_est.pitch]
        fake_state_str = "$STATE," + ",".join(map(str,fake_state)) + "\n" + '###'
        fake_state_msg = fake_state_str.encode()
        
        if self.log_daa == True:
            self.all_data_f.write(gps_msg)
            self.all_data_f.write(adcp_msg)
            self.all_data_f.write(fake_state_msg)
        

    def update_state(self, state, uR, uL):
        ''' for simulation '''
        # uL = 0
        # uR = 0
        uR = -uR/ 200 /self.dt
        uL = -uL/ 200 / self.dt

        current_v = 3
        current_ang = 0

        b_l = 35 # sim linear drag
        b_r = 30 # sim rotational drag 
        I_zz = 30 # sim moment of inertia 
        m = 30 # sim mass
        robot_radius = 0.5
  
       # update state
        state.a = 10*(uR + uL)/m - b_l/m * state.v
        state.ang_acc = -b_r / I_zz * state.omega + 1/I_zz * 2 * robot_radius * (uL - uR)

        state.v = state.v + state.a * self.dt
        state.omega = state.omega + state.ang_acc * self.dt

        robot_vx = state.v * math.cos(state.theta) + state.current_v * math.cos(state.current_ang)
        robot_vy = state.v * math.sin(state.theta) + state.current_v * math.sin(state.current_ang)
        v_robot = np.array([robot_vx, robot_vy])
        
        state.ang_course = math.atan2(v_robot[1], v_robot[0])
        state.v_course = math.sqrt(np.dot(v_robot,v_robot))

        print("course: ", state.v_course)
        print("course ang: ", state.ang_course)

        state.omega = min(max(state.omega, -1), 1)
        
        # update position
        state.x = state.x + state.v*math.cos(self.angleDiff(state.theta)) * self.dt + self.state_est.current_v * math.cos(self.angleDiff(self.state_est.current_ang)) * self.dt
        state.y = state.y + state.v*math.sin(self.angleDiff(state.theta)) * self.dt + self.state_est.current_v * math.sin(self.angleDiff(self.state_est.current_ang))* self.dt
        state.theta = self.angleDiff(state.theta + state.omega * self.dt) 
        
        # print(state.v)
        # print(state.theta)
        # print("x, y:", state.x, state.y)
        # print(state.y)
        # self.state_est.lat, self.state_est.lon = utm.to_latlon(self.utm_x, self.utm_y, 11, 'S')

        # keep this to return the updated state
        return state



