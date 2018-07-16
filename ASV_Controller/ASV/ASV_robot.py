from ASV.ASV_environment import *
from ASV_state import *
import datetime
import math
import utm
import threading
import time
import struct

class ASV_robot:

    def __init__(self, environment):
        self.environment = environment
        self.state_est = ASV_state()

        # angle offset
        self.adcp_angle_offset = 45 # degrees

        # Way points to navigate
        self.cur_des_point = ASV_state()
        self.cur_des_point.set_state(1,1,1)
        self.way_points = [self.cur_des_point]
        self.des_reached = False
        self.dist_threshold = 0.5 
        self.dt = 0.01

        self.motor_stop = False

        # Controller Params
        self.Kp = 1.0

        # robot commands
        self.rudder = 0.0
        self.R_motor = 0.0
        self.L_motor = 0.0

        # GPS Data and Coordinate
        self.GPS_fix_quality = 0
        self.GPS_raw_msg = ''
        self.GPS_Time = ''
        self.utm_x = 0
        self.utm_y = 0
        self.GPS_received = False

        # Origin
        self.origin_lat = 0.0
        self.origin_long = 0.0
        self.origin_x_utm = 0.0
        self.origin_y_utm = 0.0

        self.beam_angle = 20

        # ADCP Data
        self.roll = 0.0
        self.pitch = 0.0
        self.heading = 0.0
        self.v_boat = [0, 0]
        self.depths = [0, 0, 0, 0]
        self.bt_boat_beams = [0, 0, 0,0]
        self.ADCP_received = False

        # threads
        self.GPS_thread = None
        self.ADCP_thread = None

        # Program termination
        self.terminate = False

        # Data logging
        time_stamp = datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S')
        self.gps_filename = 'Log/GPS_' + time_stamp + ".txt"
        self.gps_f = open(self.gps_filename, 'w')
        self.adcp_filename = 'Log/ADCP_' + time_stamp + ".bin"
        self.adcp_f = open(self.adcp_filename, 'wb')

        self.all_data_filename = 'Log/ALL_' + time_stamp + ".bin"
        self.all_data_f = open(self.all_data_filename, 'wb')

    def main_loop(self):
        '''main control loop for the ASV'''
        # 1. Predict asv position (maybe not now)
        # 2. If GPS/DVL/Compass available, update accordingly
        if self.GPS_received:
            self.localize_with_GPS()

        if self.ADCP_received:
            self.localize_with_ADCP()

        # 3. Motion plan
        # time.sleep(0.5)
        # Set destination 
        self.update_waypoint()
        
        # print("Current Destination: ", self.cur_des_point.x, self.cur_des_point.y)
        # print("Current Origin: ", self.origin_x, self.origin_y)
        print("Dest X Y: ", self.cur_des_point.x, self.cur_des_point.y)
        print("robot x y: ", self.state_est.x, self.state_est.y)
        print("Heading: ", self.heading)
        # Compute control signal
        strboard, port = self.point_track(self.cur_des_point)

        # 4. Update control signals
        self.update_control(strboard, port)

        # time.sleep(0.1)

        self.report_to_shore()

        if self.terminate == True:
            self.robot_shutdown()

    def robot_setup(self):
        print('Setting up...')
        self.environment.start_ping()
        # Initialize GPS thread
        self.GPS_thread = threading.Thread(name = 'GPS Thread', target = self.update_GPS)
        self.GPS_thread.setDaemon(True)

        # Initilialize ADCP thread
        self.ADCP_thread = threading.Thread(name = 'ADCP Thread', target = self.update_ADCP)
        self.ADCP_thread.setDaemon(True)

        # Create Xbee call back for communication
        if self.environment.dest_xbee == None:
            self.terminate = True
            self.robot_shutdown()
        else:
            self.environment.my_xbee.add_data_received_callback(self.xbee_callback)

        print('Starting main loop...')
        # Begin threads
        self.GPS_thread.start()
        self.ADCP_thread.start()

    def robot_shutdown(self):
        self.update_control(0, 0)
        self.environment.my_xbee.close()

    def update_control(self, L, R):
        '''Send command messages to the ASV'''
        # self.starboard_ser.write('!G 1 %d', L)
        # self.port_ser.write('!G 1 %d', R)
        # print("Starboard: %f Port: %f" % (L, R))
        if self.motor_stop == True:
            print("Port %f Starboard: %f" % (0, 0))
        else:
            print("Port: %f Starboard: %f" % (L, R))
        pass

    def update_waypoint(self):
        ''' iterate through the set of way points'''
        if self.des_reached:
            if len(self.way_points) == 1:
                self.cur_des_point = self.cur_des_point
            else:
                self.way_points = self.way_points[1:]
                self.cur_des_point = self.way_points[0]

    def add_way_points(self, way_points):
        '''add way points'''
        #TODO Modify this later for multiple points
        self.way_points = [way_points]

    def point_track(self, des_point):
        '''Generate motor values given a point and current state'''
        angle_offset = math.atan2(des_point.y - self.state_est.y,
                                des_point.x - self.state_est.x)

        distance = math.sqrt((des_point.y - self.state_est.y)**2 + 
                (des_point.x - self.state_est.x)**2)

        if distance <= self.dist_threshold:
            self.des_reached = True
            starboard = 0.0
            port = 0.0
        else:
            # Simple heading P control
            heading_diff = self.angleDiff(self.state_est.theta - angle_offset)
            uR = -self.Kp * (heading_diff)
            uL = self.Kp * heading_diff

            # Simple velocity P control
            u_nom = (starboard/abs(starboard)) * distance * self.Kp
            u_starboard = u_nom + uR
            u_port = u_nom + uL

        return u_starboard, u_port

    def log_data(self):
        '''Log relevant data'''
        pass

    def localize_with_GPS(self):
        '''state estimate with GPS'''
        # 1. Update position with GPS
        # self.state_est.x = self.utm_x - self.origin_x
        # self.state_est.y = self.utm_y - self.origin_y
        self.GPS_received = False


    def predict(self):
        '''Predict with just the internal states of the robot'''
        self.state_est.x = self.state_est.x + self.state_est.v * math.cos(self.state_est.theta) * self.dt
        self.state_est.y = self.state_est.x + self.state_est.v * math.sin(self.state_est.theta) * self.dt
        self.state_est.theta = self.state_est.theta + self.state_est.omega * self.dt

    def localize_with_ADCP(self):
        '''state estimate with DVL'''
        self.state_est.theta = self.heading / 180 * math.pi
        # self.v_boat[0], self.v_boat[1] = self.convert_bt_velocities(self.roll, self.pitch, self.heading, self.bt_boat_beams)
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
        elif parsed_data[0] == "!WP":
            self.cur_des_point.x = float(parsed_data[1]) # way point in local x, y
            self.cur_des_point.y = float(parsed_data[2]) 
            self.add_way_points(self.cur_des_point)
        elif parsed_data[0] == "!ORIGIN":
            self.origin_x_utm = float(parsed_data[1])
            self.origin_y_utm = float(parsed_data[2])
            # self.origin_x, self.origin_y, _, _ = utm.from_latlon(self.str_to_coord(self.origin_lat), self.str_to_coord(self.origin_long))
        elif parsed_data[0] == "!MISSION":
            self.way_points = []
            for p in parsed_data[1].split(";"):
                x, y = list(map(float, p.split(" ")))
                des_point = ASV_state(x, y)
                self.way_points.append(des_point)
        elif parsed_data[0] == "!ABORTMISSION":
            self.way_points = []
            self.motor_stop  = True
        elif parsed_data[0] == "!STOP":
            self.motor_stop = True
        elif parsed_data[0] == "!START":
            self.motor_stop = False
        return data

    def report_to_shore(self):
        '''Send information back to shore'''
        if self.terminate == False:
            average_depth = sum(self.depths)/len(self.depths)
            msg = "!DATA, %f, %f, %f, %f, %f, %f" % (self.state_est.x, self.state_est.y, self.heading, average_depth, self.v_boat[0], self.v_boat[1])
            # print(msg)
            msg_binary = msg.encode()
            self.environment.my_xbee.send_data_async(self.environment.dest_xbee, msg_binary)

###############################################################################
# GPS Functions
###############################################################################
    def update_GPS(self):
        while True:
            if self.terminate == True:
                break
            else:
                if self.environment.GPS_ser.in_waiting != 0:
                    data_str = self.environment.GPS_ser.readline()
                    self.all_data_f.write(data_str)
                    self.gps_f.write(data_str.decode())
                    self.process_GPS(data_str)

        self.gps_f.close()
        self.environment.GPS_ser.close()

    def process_GPS(self, data):
        ''' Convert GPS (lat, lon) -> UTM -> Local XY '''
        data_decoded = data.decode()
        # data_decoded = '$GPGGA,194502.00,3526.9108198,N,11854.8502196,W,2,15,0.8,142.610,M,-29.620,M,7.0,0131*78'
        raw_msg = data_decoded.split(',')
        if raw_msg[0] == '$GPGGA':
            self.GPS_fix_quality = raw_msg[6]
            if self.GPS_fix_quality != '0':
                self.GPS_raw_msg = raw_msg
                self.GPS_Time = raw_msg[1]
                self.state_est.lat = self.str_to_coord(raw_msg[2])
                self.state_est.lon = -self.str_to_coord(raw_msg[4])
                self.utm_x, self.utm_y, _, _ = utm.from_latlon(self.state_est.lat, self.state_est.lon)       
                self.state_est.x = self.utm_x - self.origin_x_utm
                self.state_est.y = self.utm_y - self.origin_y_utm
                self.GPS_received = True
            else:
                self.GPS_received = False
                print('Bad GPS, no fix :(')

    def str_to_coord(self, coord_str):
        if len(coord_str) == 12:
            coord_str = '0' + coord_str #Add 0 to front
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
                self.heading = self.angleDiff_deg(data[0] - self.adcp_angle_offset)
                self.roll = data[1]
                self.pitch = data[2]
                self.depths = data[3:7]
                self.bt_boat_beams = data[7:11]
                self.ADCP_received = True

        self.environment.stop_ping()
        self.adcp_f.close()
        self.all_data_f.close()
        self.environment.ADCP_ser.close()

    def read_ensemble(self,verbose=False):
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
        self.adcp_f.write(all_data)
        self.all_data_f.write(all_data)

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
                vel = 0
                for j in range(num_beams):
                    vel += int.from_bytes(all_data[start_offset + 2*j: start_offset + 2 + 2*j], byteorder='little', signed=True)
                vel = vel/float(num_beams)
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
            essential = [heading, roll, pitch]
            essential = essential + bt_ranges
            essential = essential + bt_velocities
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

    def sim_loop(self):
        pass

    def update_motor(self):
        pass

    def simulate_movement(self, R, L):
        '''Very simple function to simulate robot movement. Reutnr change in distance
        and orientation'''
        self.state_est.a = (R + L) / 10
        self.state_est.v = self.state_est.v + self.state_est.a * self.dt
        self.omega = self.state_est.v * math.cos(self.state_est.theta)

        delta_s, delta_theta = self.simulate_odometry(left_dist, right_dist)

        return delta_s, delta_theta

    def update_state(self, state, uL, uR):
        ''' for simulation '''
        b_l = 1 # sim linear drag
        b_r = 5 # sim rotational drag 
        I_zz = 60 # sim moment of inertia 
        m = 50 # sim mass
        robot_radius = 0.5

       # update state
        state.a = (uL + uR)/m - b_l/m * state.v
        state.ang_acc = -b_r / I_zz * state.omega + 1/I_zz * 2 * robot_radius * (uR - uL)

        state.v = state.v + state.a * self.dt
        state.omega = state.omega + state.ang_acc * self.dt

        # update position
        state.x = state.x + state.v*math.cos(self.angleDiff(-state.theta + math.pi/2)) * self.dt
        state.y = state.y + state.v*math.sin(self.angleDiff(-state.theta + math.pi/2)) * self.dt
        state.theta = self.angleDiff(state.theta + state.omega * self.dt)
        # print(state.y)
        # self.state_est.lat, self.state_est.lon = utm.to_latlon(self.utm_x, self.utm_y, 11, 'S')

        # keep this to return the updated state
        return state



