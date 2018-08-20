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
        self.speed_des = 3

        # Way points to navigate
        self.cur_des_point = ASV_state()
        self.last_des_point = ASV_state()

        self.cur_des_point.set_state(1,1,1)
        self.way_points = []
        self.des_reached = True
        self.dist_threshold = 2
        self.dt = 0.01
        self.motor_stop = True
        self.first_GPS = True
        self.first_point = True # first fence point

        # Controller Params
        # Heading Control Param
        self.Kp_ang = 500 # also used for normal point tracker
        self.Kp_nom = 1000 # speed control for normal point tracker
        self.Kd = 0       # not used for now
        
        # LOS Integral Control Param
        self.Ki = 20     # integral gain for offset angle
        self.last_offtrack_error =0 # Memories for LOS integral control
        self.last_ang_error = 0.0 # Memories for derivative and integral control
        self.last_ang_error2 = 0.0
        self.last_u_rudder = 0.0

        # Course keep controller param
        # Controller 2
        self.K_v = 0.5 # relate distance to vertical velocity
        self.K_vert = 600 # relate velocity to thrust value
        self.K_latAng = 0.1 # relate lateral velocity to distance
        self.angle_update_rate = 5 # decide how often to control the angles
        self.velocity_update_rate = 5 # decide how often to control the velocity
        self.v_x_des = 0.5

        self.ang_update_count = 0 # Memories to control angle and velocity update rate
        self.vert_update_count = 0
        self.last_v_des=  0
        self.last_ang_des = 0
        self.last_drift_error = 0
        self.last_drift_error2 = 0
        self.error_integral = 0
        self.v_x = 0 # current velocity toward the point

        # Controller 1
        self.nom_thrust = 500
        self.K_va = 0.05 # relate desired velocity to angle
        self.K_latV = 300 # for changing the thrust on lateral control

        self.current_angle = 0.0

        # robot commands
        self.rudder = 0.0
        self.R_motor = 0.0
        self.L_motor = 0.0
        self.back_spin_threshold = 1000
        self.forward_threshold = 1000

        self.port = 0
        self.strboard = 0

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
        self.heading_offset = -12.0 #magnetic vs. true north

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

        self.bad_connection_limit = 100

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

        self.mission_debug = True
        self.control_debug = False

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

        # 3. Motion plan
        # Set destination 
        self.update_waypoint()
        
        # Compute control signal
        # port, strboard = self.diff_point_track(self.cur_des_point) # differential drive controller
        # port, strboard, rudder = self.point_track(self.cur_des_point) # servo drive controller
        # self.port, self.strboard, self.rudder = self.integral_LOS_pt(self.cur_des_point)
        self.port, self.strboard, self.rudder = self.main_controller(self.cur_des_point, self.v_x_des)

        # System Identificaiton Test
        # self.port = -800
        # self.strboard = -800
        # self.rudder = 1800

        # 4. Update control signals
        if self.motor_stop == False:
            self.update_control(self.port, self.strboard, self.rudder)
        else:
            self.rudder = 1600
            self.environment.send_servo_command(self.rudder)



        # if self.log_data:
            # Log control message
            # current_state = [self.state_est.x, self.state_est.y, self.state_est.theta, self.state_est.v_course, self.state_est.ang_course, rudder, port, strboard]
            # current_state_str = "$CTRL," + ",".join(map(str,current_state)) + "###"
            # print(current_state_str)
            # self.all_data_f.write(current_state_str.encode())

        if self.control_debug:
            # print("Left %f, Right %f" % (-self.port, -self.strboard))
            # print("Rudder: %d" % (self.rudder))
            # print("Boat speed: ", self.state_est.v_course)
            # print("Course angle: ", self.state_est.ang_course)
            print("Des Speed: %f, Kp_a: %f, Kp_nom: %f, U_lim: %f, L_lim: %f" % (self.speed_des, self.Kp_ang, self.Kp_nom, self.forward_threshold, self.back_spin_threshold))


        time.sleep(0.2)

        # print("working?")
        self.report_to_shore()

        if self.terminate == True:
            self.robot_shutdown()

    def robot_setup(self):
        print('Setting up...')
        self.environment.disable_xbee = False

        # Setup Hardware
        if (self.environment.robot_mode == "HARDWARE MODE"):
            self.environment.setup_GPS()
            self.environment.setup_motors()
            # self.environment.setup_ADCP()
            self.environment.setup_arduino()
            
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
        # self.environment.start_ping()
        # self.ADCP_thread.start()
        self.mag_thread.start()

    def robot_shutdown(self):
        self.update_control(0, 0, 1600)
        if self.environment.disable_xbee != True:
            self.environment.my_xbee.close()

    def update_control(self, L, R, rudder = 1515):
        '''Send command messages to the ASV'''
    
        # print("Starboard: %f Port: %f" % (L, R))
        if self.motor_stop == True:
            motor_L = 0
            motor_R = 0
            rudder = 1600
        else:
            motor_L = L
            motor_R = R
            rudder = rudder

        left_command = '!G 1 %d' % motor_L
        right_command = '!G 1 %d' % motor_R

        self.environment.send_servo_command(rudder)
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

    ######################## Controllers ###############################
    def main_controller(self, des_point, v_x_des):
        '''Main controller that switch between transect control2 and 
        the normal point tracker''' 
        uR = 0
        uL = 0
        rudder = 0
        if des_point.control_type == 1:
            # Use the regular point tracker
            uR, uL, rudder = self.point_track(self.cur_des_point)
            return uL, uR, rudder
        elif des_point.control_type == 2:
            # Use the transect control2
            uR, uL, rudder = self.transect_control2(self.cur_des_point, self.v_x_des)
            return uL, uR, rudder

    ########## Controller 1 - Fixed Thrust Transect Control ############
    def transect_control1(self, des_point, v_x_des):
        '''Controller 1: Given a thrust
        1. Calculate displacement relative to line
        2. Calculate correction y velocity
        3. Calculate correction ASV angle relative to line
        Parallel: Use v_x difference to calculate thrust
        Move to first point with regular point tracker'''

        # Parameters: u_nom (nomianl speed)
        # Kp_ang (turning gain)
        # K_lat (thruster gain for lateral speed)
        # K_va (ASV angle calculation gain for vertical compensation)
        # K_v (Velocity adjustment from position)
        u_nom = self.nom_thrust
        if self.first_point == True:
            # Do regular point track to the first point
            u_starboard, u_port, u_rudder = self.point_track(des_point)
        else:
            distance = math.sqrt((des_point.y - self.state_est.y)**2 + (des_point.x - self.state_est.x)**2)
            if distance <= self.dist_threshold or self.des_reached:
                self.stop_motor = True
                self.des_reached = True
                self.last_des_point.x = self.cur_des_point.x
                self.last_des_point.y = self.cur_des_point.y
                self.first_point = False

                u_starboard = 0.0
                u_port = 0.0
                u_rudder = 1600
            else:
                # Calculate correction Velocity base on displacement
                v_vert = self.vertical_speed_control(des_point)
                if self.ang_update_count >= self.angle_update_rate:
                    ang_des = self.calc_vertical_ang(des_point, v_vert)
                    self.last_ang_des = ang_des
                    self.ang_update_count = 0
                else:
                    ang_des = self.last_ang_des
                    self.ang_update_count += 1

                u_rudder = self.heading_control(ang_des)
                u_correct = self.lateral_speed_control(des_point, v_x_des)
                u_starboard = u_nom + u_correct
                u_port = u_nom + u_correct

                u_starboard = -min(max(u_starboard, -self.back_spin_threshold), self.forward_threshold)
                u_port = -min(max(u_port, -self.back_spin_threshold), self.forward_threshold) 

        return u_port, u_starboard, u_rudder

    def heading_control(self, heading):
        '''Orient the robot heading by turning the rudder'''
        u_rudder = -self.angleDiff(heading - self.state_est.theta) * self.Kp_ang
        u_rudder = int(u_rudder + 1515)
        if (u_rudder > 1834):
            u_rudder = 1834
        elif (u_rudder < 1195):
            u_rudder = 1195

        return u_rudder

    def lateral_speed_control(self, des_point, v_x_des):
        '''Adjust the ASV thrust according to the transect (x) speed'''
        # Calculate drift away from the line
        K_lat = self.K_latV

        cur_point = self.last_des_point
        next_point = des_point

        line_angle = math.atan2((next_point.y - cur_point.y),(next_point.x - cur_point.x))                
        v_ang_from_line = self.angleDiff(self.state_est.ang_course - line_angle)
        v_x = self.state_est.v_course * math.cos(v_ang_from_line)

        heading_from_line = self.angleDiff(self.state_est.theta - line_angle)

        u_nom = (v_x_des - v_x) * K_lat #* math.cos(heading_from_line)
        # if math.cos(heading_from_line) < 0.5:
        #     u_nom = 0

        print("V_x", v_x)
        print("U_nom", u_nom)
        return u_nom

    def calc_vertical_ang(self, des_point, v_y_des):
        '''Adjust the ASV angle relative to the line according to vertical (y) speed'''
        # Calculate drift away from the line
        K_va = self.K_va
        cur_point = self.last_des_point
        next_point = des_point

        # Calculate line angle, velocity angle, and velocity away from line
        line_angle = math.atan2((next_point.y - cur_point.y),(next_point.x - cur_point.x))                
        v_ang_from_line = self.angleDiff(self.state_est.ang_course - line_angle)
        v_y = self.state_est.v_course * math.sin(v_ang_from_line)

        # Robot heading relative to line
        heading_from_line = self.angleDiff(self.state_est.theta - line_angle)

        # Adjust robot heading relative to line
            # Bigger v_y -> robot orientated more normal to the line
            # Smaller v_y -> robot orientated more toward the endpoints
        des_line_heading = self.angleDiff(-(v_y - v_y_des) * K_va + heading_from_line)            
        if abs(des_line_heading) > math.pi/2:
            des_line_heading = des_line_heading/abs(des_line_heading) * math.pi/2   

        new_ang = self.angleDiff(des_line_heading + line_angle)
        return new_ang

    def vertical_speed_control(self, des_point):
        '''calculate current velocity toward the waypoint, increase or decrease u_nom'''
        # Calculate drift away from the line
        K_v = self.K_v
        cur_point = self.last_des_point
        next_point = des_point

        line_angle = math.atan2((next_point.y - cur_point.y),(next_point.x - cur_point.x))                
        position_angle = math.atan2((self.state_est.y - cur_point.y), (self.state_est.x - cur_point.x))

        pos_ang_from_line = self.angleDiff(position_angle - line_angle)
        dist = math.sqrt((self.state_est.y - cur_point.y)**2 + (self.state_est.x - cur_point.x)**2 )
        drift_distance = dist * math.sin(pos_ang_from_line)

        self.error_integral = self.error_integral + drift_distance * self.dt
        # self.last_drift_error = drift_distance
        # self.last_drift_error2 = self.last_drift_error

        v_correct = -drift_distance * K_v
        return v_correct


    #######################################################################
    ############## Controller 2: Fixed Angle Variable Thrust ##############
    #######################################################################

    def transect_control2(self, des_point, v_x_des):
        if self.first_point == True:
            # Do regular point track to the first point
            u_starboard, u_port, u_rudder = self.point_track(des_point)
        else:
            distance = math.sqrt((des_point.y - self.state_est.y)**2 + (des_point.x - self.state_est.x)**2)
            if distance <= self.dist_threshold or self.des_reached:
                self.stop_motor = True
                self.des_reached = True
                self.last_des_point.x = self.cur_des_point.x
                self.last_des_point.y = self.cur_des_point.y
                self.first_point = False

                u_starboard = 0.0
                u_port = 0.0
                u_rudder = 1600
            else:
                # Calculate correction Velocity base on displacement
                u_rudder = 1515
                if self.vert_update_count >= self.velocity_update_rate:
                    u_vert = self.vertical_speed_control(des_point)
                    self.last_v_des = u_vert
                    self.vert_update_count = 0
                else:
                    self.vert_update_count += 1
                    u_vert = self.last_v_des

                if self.ang_update_count >= self.angle_update_rate:
                    ang_des = self.calc_lateral_ang(des_point, v_x_des)
                    self.last_ang_des = ang_des
                    self.ang_update_count = 0
                else:
                    self.ang_update_count += 1
                    ang_des = self.last_ang_des

                u_rudder = self.heading_control(ang_des)
                u_port, u_starboard = self.vertical_position_hold(self.cur_des_point, -u_vert)
        
        return u_port, u_starboard, u_rudder

    def calc_lateral_ang(self, des_point, v_x_des):
        '''Adjust the ASV angle according to the desired transect (x) speed'''

        # Calculate drift away from the line
        cur_point = self.last_des_point
        next_point = des_point

        line_angle = math.atan2((next_point.y - cur_point.y),(next_point.x - cur_point.x))                
        v_ang_from_line = self.angleDiff(self.state_est.ang_course - line_angle)
        v_x = self.state_est.v_course * math.cos(v_ang_from_line)

        self.v_x = v_x

        # Calculate heading difference between robot and the line
        heading_from_line = self.angleDiff(self.state_est.theta - line_angle)

        if heading_from_line < 0:
            des_line_heading = self.angleDiff(-(v_x - v_x_des) * self.K_latAng + heading_from_line)
        else:
            des_line_heading = self.angleDiff((v_x - v_x_des) * self.K_latAng + heading_from_line)
        
        new_ang = self.angleDiff(des_line_heading + line_angle)
        return new_ang

    def vertical_position_hold(self, des_point, vertical_speed):
        ''' hold position '''
        # Got to the first point. Now we hold position on the fence line

        # Calculate drift away from the line
        cur_point = self.last_des_point
        next_point = des_point
        u_nom = self.K_vert

        # 1. Angle of the line
        # 2. Angle between the robot pos and line
        # 3. Angle between the robot_v and line
        line_angle = math.atan2((next_point.y - cur_point.y),(next_point.x - cur_point.x))                
        v_ang_from_line = self.angleDiff(self.state_est.ang_course - line_angle)
        drift_v = self.state_est.v_course * math.sin(v_ang_from_line)
        heading_from_line = self.angleDiff(self.state_est.theta - line_angle)
        
        position_angle = math.atan2((self.state_est.y - cur_point.y), (self.state_est.x - cur_point.x))
        pos_ang_from_line = self.angleDiff(position_angle - line_angle)
        # dist = math.sqrt((self.state_est.y - cur_point.y)**2 + (self.state_est.x - cur_point.x)**2 )
        # drift_distance = dist * math.sin(pos_ang_from_line)
        # print("drift d", drift_distance)
        thrust_dir = heading_from_line/abs(heading_from_line)
        
        u_nom = (drift_v+ vertical_speed) * -thrust_dir * u_nom 

        u_starboard = -min(max(u_nom, -self.back_spin_threshold), self.forward_threshold)
        u_port = -min(max(u_nom, -self.back_spin_threshold), self.forward_threshold)
        
        return u_port, u_starboard


    def integral_LOS_pt(self, des_point):
        '''Using integral line of sight tracking'''
        track_angle = math.atan2(des_point.y - self.last_des_point.y,
                                des_point.x - self.last_des_point.x)

        angle_off = math.atan2(des_point.y - self.state_est.y,
                                des_point.x - self.state_est.x)
        distance = math.sqrt((des_point.y - self.state_est.y)**2 + 
                (des_point.x - self.state_est.x)**2)
        
        # Track error
            # on, x offset along the trajectory line
            # offset perpendicular to the line
        on_track_error = distance * math.cos(self.angleDiff(angle_off - track_angle))
        off_track_error = distance * math.sin(self.angleDiff(angle_off - track_angle))

        int_gain = self.Ki
        off_int = int_gain * off_track_error * self.dt / 2 + int_gain * self.last_offtrack_error * self.dt/2

        self.last_offtrack_error = off_track_error
        
        if abs(off_int) == 0.0:
            off_int = 0
        if on_track_error == 0.0:
            on_track_error = 0.0001

        ang_int = math.atan(off_int/on_track_error)
        des_angle = self.angleDiff(angle_off + ang_int)

        # direction_off = self.angleDiff(angle_off - self.state_est.ang_course)
        robot_vx = self.state_est.v_course * math.cos(self.state_est.ang_course)
        robot_vy = self.state_est.v_course * math.sin(self.state_est.ang_course)
        robot_v = np.array([robot_vx, robot_vy])
        desired_dir = np.array([math.cos(angle_off), math.sin(angle_off)])

        # direction_off = math.acos(np.dot(robot_v, desired_dir) / (self.state_est.v_course * 1))
        # speed_to_dest = self.state_est.v_course * math.cos(direction_off)
        speed_to_dest = np.dot(desired_dir, robot_v) #* desired_dir
        # speed_to_dest = math.sqrt(np.dot(speed_to_dest, speed_to_dest))
        
        # print("Last", self.last_des_point)
        # print("Current", self.state_est)
        print("#################")
        print("Speed in Dir ", speed_to_dest)
        # print("Offtrack ", off_track_error)
        # print("ANG OFF ", angle_off)
        # # print("Track angle", track_angle)
        # # print("Distance ", distance)
        # print("integral off ", off_int)
        # print("on error ", on_track_error)
        # print("ANG INT", ang_int)
        # print("ANG DES", des_angle)

        if distance <= self.dist_threshold or self.des_reached:
            self.stop_motor = True
            self.des_reached = True
            self.last_des_point.y = self.cur_des_point.y
            self.last_des_point.x = self.cur_des_point.x
            # self.last_offtrack_error = 0
            u_starboard = 0.0
            u_port = 0.0
            u_rudder = 1600
        else:
            # Simple heading PD control
            angle_error = -self.angleDiff(des_angle - self.state_est.theta)
            print("ANG ERROR ", angle_error)
            # u_rudder = self.last_u_rudder + (angle_error * (self.Kp_ang + self.Ki * self.dt) - self.last_ang_error * self.Kp_ang)
            # u_rudder = self.last_u_rudder + (angle_error * (self.Kp_ang + self.Kd / self.dt) + self.last_ang_error * (-self.Kp_ang - 2*self.Kd/self.dt) + self.last_ang_error2 * self.Kd/self.dt)
            u_rudder = angle_error * self.Kp_ang

            self.last_ang_error = angle_error
            self.last_ang_error2 = self.last_ang_error
            self.last_u_rudder = u_rudder
            u_nom = (self.speed_des - self.state_est.v_course) * self.Kp_nom + 100


            u_port = u_nom
            u_starboard = u_nom

            u_rudder = int(u_rudder + 1600)
            if (u_rudder > 1834):
                u_rudder = 1834
            elif (u_rudder < 1195):
                u_rudder = 1195

            print("Rudder ", u_rudder)

            u_starboard = -min(max(u_nom, -self.back_spin_threshold), self.forward_threshold)
            u_port = -min(max(u_nom, -self.back_spin_threshold), self.forward_threshold)

        return u_port, u_starboard, u_rudder

    def point_track(self, des_point):
        '''Use the rudder motor to help point tracking'''
        # calculate desired angle displacement
        des_angle = math.atan2(des_point.y - self.state_est.y,
                                des_point.x - self.state_est.x)
        
        speed_des = self.speed_des
        v_x_des = speed_des * math.cos(des_angle)
        v_y_des = speed_des * math.sin(des_angle)

        distance = math.sqrt((des_point.y - self.state_est.y)**2 + 
                (des_point.x - self.state_est.x)**2)

        if distance <= self.dist_threshold or self.des_reached:
            self.stop_motor = True
            self.des_reached = True
            self.first_point = False
            self.last_des_point.x = self.cur_des_point.x
            self.last_des_point.y = self.cur_des_point.y
            u_starboard = 0.0
            u_port = 0.0
            u_rudder = 1515
        else:
            # Simple heading PD control
            angle_error = -self.angleDiff(des_angle - self.state_est.theta)
            v_angle_to_point = self.angleDiff(self.state_est.ang_course - des_angle)
            v_x = self.state_est.v_course * math.cos(v_angle_to_point)
           
            self.v_x = v_x

            # u_rudder = self.last_u_rudder + (angle_error * (self.Kp_ang + self.Kd / self.dt) + self.last_ang_error * (-self.Kp_ang - 2*self.Kd/self.dt) + self.last_ang_error2 * self.Kd/self.dt)
            u_rudder = angle_error * self.Kp_ang
            self.last_ang_error = angle_error
            self.last_ang_error2 = self.last_ang_error
            self.last_u_rudder = u_rudder

            u_nom = (speed_des - v_x) * self.Kp_nom + 100

            u_port = u_nom
            u_starboard = u_nom

            u_rudder = int(u_rudder + 1600)
            if (u_rudder > 1834):
                u_rudder = 1834
            elif (u_rudder < 1195):
                u_rudder = 1195

            u_starboard = -min(max(u_nom, -self.back_spin_threshold), self.forward_threshold)
            u_port = -min(max(u_nom, -self.back_spin_threshold), self.forward_threshold)

        return u_port, u_starboard, u_rudder

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
        if self.mission_debug:
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
            wp.control_type = int(parsed_data[3])
            self.add_way_point(wp)
            self.des_reached = False
        elif parsed_data[0] == "!STARTMISSION":
            self.cur_des_point = self.way_points[0]
            self.last_des_point.x = self.state_est.x
            self.last_des_point.y = self.state_est.y
            self.des_reached = False
            self.first_point = True
        elif parsed_data[0] == "!ABORTMISSION":
            self.way_points = []
            self.motor_stop  = True
        elif parsed_data[0] == "!STOP":
            self.motor_stop = True
            print('Stop message received. Motors stopped.')
        elif parsed_data[0] == "!START":
            self.motor_stop = False
            self.first_point = True
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
        elif parsed_data[0] == "!TRANSECT":
            self.K_v = float(parsed_data[1])
            self.K_latAng = float(parsed_data[2])
            self.K_vert = float(parsed_data[3])
            self.velocity_update_rate = int(parsed_data[4])
            self.angle_update_rate = int(parsed_data[5])
            self.v_x_des = float(parsed_data[6])
            print('Transect Gains Received.')
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
                msg = "!DATA, %f, %f, %f, %f, %f, %f, %f" % (self.state_est.x, self.state_est.y, self.heading, average_depth, current_speed, self.state_est.v_course, self.v_x)
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
                # print("Checking")
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
                data_str = self.environment.arduino_ser.readline()
                # print(data_str)
                mag_data =data_str.split(b',')
                try:
                    if mag_data[0] == b'$MAG':
                        self.heading = float(mag_data[1]) + 90 + self.heading_offset
                        self.pitch = float(mag_data[2])
                        self.roll = float(mag_data[3])
                        self.mag_received = True
                    elif mag_data[0] == b'$SERVO':
                        # print(mag_data[1][:])
                        servo_data = mag_data[1][0]
                except:
                    continue

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
                        current_state = [self.state_est.x, self.state_est.y, self.state_est.theta, self.state_est.v_course, self.state_est.ang_course, self.rudder, self.port, self.strboard]
                        current_state_str = "$CTRL," + ",".join(map(str,current_state)) + "###"
                        self.all_data_f.write(current_state_str.encode())

                        self.all_data_f.write("$GPS,".encode() + data_str + b'###')
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
                self.utm_x, self.utm_y, _, _ = utm.from_latlon(self.state_est.lat, self.state_est.lon)       
                self.state_est.x = self.utm_x
                self.state_est.y = self.utm_y
                self.GPS_received = True
                
                if self.first_GPS:
                    self.cur_des_point.x = self.utm_x
                    self.cur_des_point.y = self.utm_y
                    self.first_GPS = False
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

        current_state = [self.state_est.x, self.state_est.y, self.state_est.theta, self.state_est.roll, self.state_est.pitch, self.state_est.v_course, self.state_est.ang_course]
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
        self.motor_stop = True
        self.first_point = True
        
        time_stamp = datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S')

        if self.log_data == True:
            self.all_data_filename = 'Log/ALL_' + time_stamp + ".csv"
            self.all_data_f = open(self.all_data_filename, 'w')

    def sim_loop(self):
        if self.first_GPS:
            self.cur_des_point.x = self.state_est.x
            self.cur_des_point.y = self.state_est.y
            self.first_GPS = False

        # uR, uL, rudder = self.integral_LOS_pt(self.cur_des_point)
        # uR, uL, rudder = self.point_track(self.cur_des_point)
        # uR, uL, rudder = self.transect_control2(self.cur_des_point, 1.5)
        uL, uR, rudder = self.main_controller(self.cur_des_point, 1.5)
        self.estimate_state()
        
        if self.motor_stop == True:
            uR = 0
            uL = 0
            rudder = 1515
        # print(uR, uL, rudder)
        self.update_state(self.actual_state, uR, uL, rudder)

        # Calc v_x

        cur_point = self.last_des_point
        next_point = self.cur_des_point

        line_angle = math.atan2((next_point.y - cur_point.y),(next_point.x - cur_point.x))                
        v_ang_from_line = self.angleDiff(self.state_est.ang_course - line_angle)
        v_x = self.state_est.v_course * math.cos(v_ang_from_line)
        v_y = self.state_est.v_course * math.sin(v_ang_from_line)

        current_state = [self.state_est.x, self.state_est.y, self.state_est.theta, self.state_est.v_course, self.state_est.ang_course, rudder, uL, uR, self.cur_des_point.x, self.cur_des_point.y, v_x, v_y]
        current_state_str = "$CTRL," + ",".join(map(str,current_state))
        if self.log_data:
            self.all_data_f.write(current_state_str)
            self.all_data_f.write('\n')


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
        
        # if self.log_data == True:

            # self.all_data_f.write(gps_msg)
            # self.all_data_f.write(adcp_msg)
            # self.all_data_f.write(fake_state_msg)
        

    def update_state(self, state, uR, uL, rudder):
        ''' for simulation '''

        uR = -uR/ 100 
        uL = -uL/ 100 

        # Rudder Angle
        rudder_rate = (1894 - 1195)/90
        rudder_ang = (rudder - 1515) / rudder_rate
        rudder_ang = -rudder_ang / 180 * math.pi

        b_l = 4 # sim linear drag
        b_r = 2.5 # sim rotational drag 
        I_zz = 50 # sim moment of inertia 
        m = 5 # sim mass
        robot_radius = 0.5
        x_cg = 0.9
        w = 20
  
         # Rudder Angle
       #  b_l = 5 # sim linear drag
       #  b_r = 2.5 # sim rotational drag 
       #  I_zz = 50 # sim moment of inertia 
       #  m = 5 # sim mass
       #  robot_radius = 0.5
  
       # # update state
       #  state.a = (uR + uL)/m - b_l/m * state.v
       #  state.ang_acc = -b_r / I_zz * state.omega  + x_cg * rudder_ang * (uR + uL) * math.sin(rudder_ang) # + 1/I_zz * 2 * robot_radius * (uL - uR)

       # update river current
        # current_v = 2 * state.x / w * (2*state.x / w - 2) * state.current_v
        current_v = state.current_v

       # update state
        state.a = (uR + uL)/m - b_l/m * state.v
        state.ang_acc = -b_r / I_zz * state.omega  +  (uR + uL) * math.sin(rudder_ang) * x_cg

        state.v = state.v + state.a * self.dt
        state.omega = state.omega + state.ang_acc * self.dt 

        robot_vx = state.v * math.cos(state.theta) + current_v * math.cos(state.current_ang)
        robot_vy = state.v * math.sin(state.theta) + current_v * math.sin(state.current_ang)
        v_robot = np.array([robot_vx, robot_vy])
        
        state.ang_course = math.atan2(v_robot[1], v_robot[0])
        state.v_course = math.sqrt(np.dot(v_robot,v_robot))

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



