import math
import datetime
import time
import random
from ASV_state import *

class ASV_sim_robot:

    def __init__(self, robot_id):
        self.state_est = ASV_state()
        self.state_est.set_state(0,0,0)
        self.state_des = ASV_state()
        self.state_des.set_state(0,0,0)
        self.last_state_est = ASV_state()
        self.last_state_est.set_state(0, 0, 0)

        self.control_mode = "MANUAL MODE"
        self.R = 0
        self.L = 0
        self.manual_control_left_motor = 0
        self.manual_control_right_motor = 0
        self.imu_measurement = 0
        self.gps_measurements = [0, 0, 0]
        self.last_imu_measurement = self.imu_measurement
        self.last_gps_measurements = self.gps_measurements

        self.origin = [0, 0]
        self.earth_radius = 6371000
        self.heading_offset = 0

        self.Kp = 1
        self.distance_threshold = 1
        self.angle_threshold = 0.05
        self.point_turn_threshold = 1
        
        self.Kpho = 2#8#1.0
        self.Kalpha = 10#40.0#25#2.0
        self.Kbeta = -0.5#-10#-0.5
        self.max_thruster_power = 200
        self.point_tracked = False

        self.drift_theta = 0
        self.monitoring_drift = False
        self.sim_drift_x = 0.0
        self.sim_drift_y = 0.0
        self.Kp_drift = 2.5
        self.Ki_drift = .3
        self.tot_drift_error = 0
        self.last_drift_check = 0

    def update(self, deltaT):


        # get sensor measurements
        self.imu_measurement, self.gps_measurements = self.update_sensor_measurements(deltaT)

        # update simulated real position, find ground truth for simulation
        self.state_est = self.localize(self.state_est, self.imu_measurement, self.gps_measurements[1], self.gps_measurements[2])

        # determine new control signals
        self.R, self.L = self.update_control()
        
        # send the control measurements to the robot
        self.send_control(self.R, self.L, deltaT)
        
        
    
    def update_sensor_measurements(self, deltaT):

        v = 0.00238*((self.R + self.L)/2)
        w = -7*pow(10, -6)*pow((self.L - self.R)/2, 2) + 0.0059*(self.L - self.R)/2

        dx = v*deltaT*math.cos(self.state_est.theta) + self.sim_drift_x
        dy = v*deltaT*math.sin(self.state_est.theta) + self.sim_drift_y
        dtheta = w*deltaT 
        x = self.state_est.x + dx
        y = self.state_est.y + dy
        lon = (x/(math.pi/180.0*self.earth_radius*math.cos(self.origin[0]))) + self.origin[1]
        lat = y/(math.pi/180.0*self.earth_radius) + self.origin[0]
        
        imu_measurement = self.angle_wrap(self.last_imu_measurement*(math.pi/180) + dtheta)*(180/math.pi)
        gps_measurements = [0, lat, lon]

        self.last_imu_measurement = imu_measurement
        
        return imu_measurement, gps_measurements

        
        
    def localize(self, state, heading, lat, lon):
        state_est = self.update_state(state, heading, lat, lon)
    
        return state_est
    
        
        
    def update_control(self):
        if self.control_mode == "MANUAL MODE":
            desiredThrusterSpeedR = int(self.manual_control_right_motor*255/100)
            desiredThrusterSpeedL =int(self.manual_control_left_motor*255/100)
            
        elif self.control_mode == "AUTONOMOUS MODE":   
            if not self.point_tracked:
                desiredThrusterSpeedR, desiredThrusterSpeedL = self.point_tracker_control()
            else:
                desiredThrusterSpeedR, desiredThrusterSpeedL = self.drift_tracker_control()
            
        return desiredThrusterSpeedR, desiredThrusterSpeedL
  


    def point_tracker_control(self):
        #print("point tracking")
        delta_x = self.state_des.x - self.state_est.x
        delta_y = self.state_des.y - self.state_est.y

        alpha = self.angle_wrap(-self.state_est.theta + self.angle_wrap(math.atan2(delta_y, delta_x)))
        rho = pow(pow(delta_x, 2) + pow(delta_y, 2), .5)

        beta = self.angle_wrap(-self.state_est.theta - alpha)

        beta = self.angle_wrap(beta + self.angle_wrap(self.state_des.theta))

        print(alpha)
        print(beta)
        desiredV = self.Kpho*rho
        desiredW = self.Kalpha*alpha + self.Kbeta*beta

        if rho < self.distance_threshold:
            self.point_tracked = True
            self.monitoring_drift = True
            self.last_state_est.copy(self.state_est)
            self.drift_theta = self.state_est.theta

        desiredThrusterSpeedR, desiredThrusterSpeedL = self.get_thruster_speed(desiredV, desiredW)

        # print("R: ", desiredThrusterSpeedR, "L: ", desiredThrusterSpeedL, "DesV: ", desiredV, "DesW: ", desiredW, end="\r")
                
        return desiredThrusterSpeedR,desiredThrusterSpeedL

    def drift_tracker_control(self):

        delta_x = self.state_est.x - self.state_des.x
        delta_y = self.state_est.y - self.state_des.y 
        rho = pow(pow(delta_x, 2) + pow(delta_y, 2), .5)

        #if time.time() - self.last_drift_check > 1:
        self.last_drift_check = time.time()
    
        print("monitoring_drift")
        dx = self.last_state_est.x - self.state_est.x 
        dy = self.last_state_est.y - self.state_est.y 

        # If not point tracking in drift mode
        if dy == 0:
            if dx > 0:
                epsilon = math.pi
            else:
                epsilon = 0
        else:
            epsilon = math.atan2(dy, dx)

        if abs(self.angle_wrap(self.drift_theta - epsilon)) > .1 and abs(self.angle_wrap(abs(self.angle_wrap(self.drift_theta - epsilon)) - math.pi)) > .2:
            self.drift_theta = self.angle_wrap(self.drift_theta - (self.drift_theta - epsilon))

            self.last_state_est.copy(self.state_est)




        '''
        if self.monitoring_drift:
            print("monitoring_drift")
            dx = self.last_state_est.x - self.state_est.x 
            dy = self.last_state_est.y - self.state_est.y 

            r = pow(pow(dx, 2) + pow(dy, 2), 0.5)
            # If not point tracking in drift mode
            if r > 0.5:
                if dy == 0:
                    if dx > 0:
                        self.drift_theta = math.pi
                    else:
                        self.drift_theta = 0
                else:
                    self.drift_theta = math.atan2(dy, dx)

                self.monitoring_drift = False

            desiredW = 0
            desiredV = 0
        else:
        '''
        gamma = math.atan2(delta_y, delta_x)

        current_theta = self.angle_wrap(self.drift_theta + math.pi)

        phi = abs(self.angle_wrap(gamma - current_theta))
        
        if phi > math.pi/2:
            desiredV = self.Ki_drift*self.tot_drift_error
        else:
            desiredV = self.Kp_drift*rho*math.cos(phi) + self.Ki_drift*self.tot_drift_error
            #print("phi", phi)
            #print('error', rho)
            #print("P: ", self.Kp_drift*rho*math.cos(phi))
            #print("I: ", self.Ki_drift*self.tot_drift_error)

        self.tot_drift_error = self.tot_drift_error + rho*math.cos(phi)

        print("dtheta: ", self.drift_theta )
        desiredW = self.Kp*(self.angle_wrap(self.drift_theta - self.state_est.theta))

        if rho > self.distance_threshold+1:
            self.point_tracked = False
            self.monitoring_drift = False
            self.drift_time = 0
            self.tot_drift_error = 0

        desiredThrusterSpeedR, desiredThrusterSpeedL = self.get_thruster_speed(desiredV, desiredW)

        print("R: ", desiredThrusterSpeedR, "L: ", desiredThrusterSpeedL, "DesV: ", desiredV, "DesW: ", desiredW)
                
        return desiredThrusterSpeedR,desiredThrusterSpeedL
    
    def send_control(self, R, L, deltaT):
        pass

    def get_thruster_speed(self, desiredV, desiredW):
        # A positive omega des should result in a positive spin
        if desiredW < 0:
            desiredThrusterSpeedR = 428.5*desiredV - (86.167*pow(desiredW, 2) - 144.17*desiredW)
            desiredThrusterSpeedL = 428.5*desiredV + (86.167*pow(desiredW, 2) - 144.17*desiredW)
        else:
            desiredThrusterSpeedR = 428.5*desiredV + (86.167*pow(desiredW, 2) + 144.17*desiredW)
            desiredThrusterSpeedL = 428.5*desiredV - (86.167*pow(desiredW, 2) + 144.17*desiredW)
        

        if abs(desiredThrusterSpeedR) > self.max_thruster_power or abs(desiredThrusterSpeedL) > self.max_thruster_power:
            scaling_factor = self.max_thruster_power/max(abs(desiredThrusterSpeedR), abs(desiredThrusterSpeedL))
            desiredThrusterSpeedR *= scaling_factor
            desiredThrusterSpeedL *= scaling_factor

        return desiredThrusterSpeedR, desiredThrusterSpeedL

        
    def set_manual_control_motors(self, R, L):
        
        self.manual_control_right_motor = int(R*255/100)
        self.manual_control_left_motor = int(L*255/100)                                                         
   
      
    def update_state(self, state, heading, lat, lon):

        x = (lon-self.origin[1])*math.pi/180.0*self.earth_radius*math.cos(self.origin[0])
        y = (lat-self.origin[0])*math.pi/180.0*self.earth_radius

        # Basic way to filter out gps noise
        if abs(x) < 100000:
            state.x = x

        if abs(y) < 100000:
            state.y = y
 
        state.theta = self.angle_wrap(-(heading + 90 + self.heading_offset)*(math.pi/180.0)) 

        return state

    def angle_wrap(self, a):
        while a > math.pi:
            a = a - 2*math.pi
        while a < -math.pi:
            a = a + 2*math.pi
        return a

    def ang_diff(self, a, b):
        return self.angle_wrap(a - b)

    def get_data(self):
        msg = self.state_est.__repr__() + " " + str(self.imu_measurement) + " " + " ".join([str(val) for val in self.gps_measurements])
        #print("msg: " + msg)
        return msg

            
        
        
        
        
        
        