from tkinter import *
from PIL import Image, ImageTk
import gdal
import utm
import time
import digi.xbee.models as XBeeModel
import numpy as np
import math

# IMAGE_WIDTH = 325.
# IMAGE_HEIGHT = 250.
# MAP_FILE = '../Maps/cast.tif'

IMAGE_WIDTH = 300
IMAGE_HEIGHT = 200
MAP_FILE = '../Maps/river_8-13.tif'


# IMAGE_WIDTH = 1000
# IMAGE_HEIGHT = 700
# MAP_FILE = '../Maps/lake.tif'

MAP_WIDTH = 800
MAP_HEIGHT = 600

POINT_RADIUS = 3

class ASV_graphics:
    def __init__(self, controller):
        # Load map for display/GPS conversions
        dataset = gdal.Open(MAP_FILE)
        data = dataset.ReadAsArray()
        self.geo_trans = dataset.GetGeoTransform()
        self.inv_trans = gdal.InvGeoTransform(self.geo_trans)

        # Control parameters
        self.robot_stopped = True
        self.quit_gui = False
        self.mission_wps = [] #format [(lat, lon)...]

        #######################################################################
        # GUI SECTION
        #######################################################################

        # Set up GUI
        self.controller = controller
        self.tk = Tk()
        self.tk.title("ASV Control Interface")
        self.tk.protocol("WM_DELETE_WINDOW", self.on_quit)

        # GUI marker variables (labels on map)
        # Go to location
        self.goto_coords = (-1,-1) #UTM! #TODO: CURRENTLY NOT CHANGED
        self.location_select_mode = False
        self.cur_pos_marker = None
        self.target_pos_marker = None

        # Mission planning
        self.add_wps_mode = False
        self.remove_wps_mode = False
        self.running_mission_mode = False
        self.set_border_mode = False
        self.wp_markers = []
        self.wp_labels = []

        self.border_markers = []

        # Origin setting
        self.origin_coords = (0,0) #pixel coords
        x, y = gdal.ApplyGeoTransform(self.geo_trans, 0,0)
        print('Origin UTM:', x, y)
        self.set_origin_mode = False

        # Frames: Sidebar + Map Area
        self.sidebar_frame = Frame(self.tk, width=400, relief='sunken', borderwidth=2)
        self.map_frame = Frame(self.tk)
        self.sidebar_frame.pack(expand=True, fill='both', side='left', anchor='nw')
        self.map_frame.pack(expand=True, fill='both', side='right')

        # GPS Coordinates
        self.gps_title = Label(self.sidebar_frame, anchor='w', text='ASV GPS Information', font='Helvetica 14 bold').pack()
        self.gps = Label(self.sidebar_frame, anchor='w', width=30, text='Latitude: ???\nLongitude: ???\nHeading: ???')
        self.gps_local = Label(self.sidebar_frame, anchor='w', width=30, text='x: ???, y: ???')
        self.gps.pack()
        #self.gps_local.pack()

        # ADCP Data
        self.adcp_title = Label(self.sidebar_frame, anchor='w', text='ADCP Information', font='Helvetica 14 bold').pack()
        self.adcp_data = Label(self.sidebar_frame, anchor='w', width=30, text='Water depth: ???\nCurrent speed: ???')
        self.adcp_data.pack()

        # ASV Control Panel
        self.control_title = Label(self.sidebar_frame, anchor='w', text='ASV Control Panel', font='Helvetica 14 bold').pack()
        # 1) Go to map location
        # self.control_wp_dxdy = Label(self.sidebar_frame, anchor='w', width=30, text='dx: ???, dy: ???')
        # self.control_wp_dxdy.pack()
        self.start_frame = Frame(self.sidebar_frame)
        self.start_frame.pack()
        self.start_stop = Button(self.start_frame, anchor='w', text='Start ASV', command=self.on_startstop)
        self.start_stop.pack(side='left')
        self.mission = Button(self.start_frame, anchor='w', text='Start Mission', command=self.on_toggle_mission)
        self.mission.pack(side='right')

        self.clear_wps = Button(self.sidebar_frame, anchor='w', text='Clear All Waypoints', command=self.on_clear_wps).pack()

        # 2) Mission planning
        self.mission_title = Label(self.sidebar_frame, anchor='w', text='Mission Planning', font='Helvetica 14 bold').pack()
        scrollbar = Scrollbar(self.sidebar_frame)
        scrollbar.pack(side='right')
        self.w_name = Label(self.sidebar_frame, text='')
        self.wp_list = Listbox(self.sidebar_frame, width=30, yscrollcommand=scrollbar.set)
        self.wp_list.pack()
        self.wp_list.bind('<<ListboxSelect>>', self.on_waypoint_selection)
        scrollbar.config(command = self.wp_list.yview)

        self.mission_file_frame = Frame(self.sidebar_frame)
        self.mission_file_frame.pack()
        self.load_mission = Button(self.mission_file_frame, anchor='w', text='Load Mission File', command=self.on_load_mission).pack(side='left')
        self.save_mission = Button(self.mission_file_frame, anchor='w', text='Save Mission', command=self.on_save_mission).pack(side='right')

        self.mission_add_wps = Button(self.sidebar_frame, anchor='w', text='Add Waypoints', command=self.on_toggle_add_wps)
        self.mission_add_wps.pack()
        self.mission_remove_wps = Button(self.sidebar_frame, anchor='w', text='Remove Waypoints', command=self.on_toggle_remove_wps)
        self.mission_remove_wps.pack()

        # Map Configuration
        self.map_config = Label(self.sidebar_frame, anchor='w', text='Configuration', font='Helvetica 14 bold').pack()
        # self.origin = Button(self.sidebar_frame, anchor='w', text='Set Map Origin', command=self.on_toggle_set_origin)
        # self.origin.pack()
        # self.set_heading_offset = Button(self.sidebar_frame, anchor='w', text='Set Heading Offset', command=self.on_set_heading_offset)
        # self.set_heading_offset.pack()
        # self.heading_offset_label = Label(self.sidebar_frame, anchor='w', text='Heading Offset (deg)').pack(side='left')
        # self.heading_offset = Entry(self.sidebar_frame, width=10)
        # self.heading_offset.insert(END, '-20')
        # self.heading_offset.pack(side='right')

        self.speed_frame = Frame(self.sidebar_frame)
        self.speed_frame.pack()
        # self.set_desired_speed = Button(self.speed_frame, anchor='w', text='Set Desired Speed', command=self.on_set_desired_speed)
        # self.set_desired_speed.pack()
        self.desired_speed_label = Label(self.speed_frame, anchor='w', text='Desired Speed (m/s)').pack(side='left')
        self.desired_speed = Entry(self.speed_frame, width=10)
        self.desired_speed.insert(END, '1')
        self.desired_speed.bind('<Return>', self.on_set_desired_speed)
        self.desired_speed.pack(side='right')

        # Control Params
        self.set_control_params = Button(self.sidebar_frame, anchor='w', text='Set Control Params', command=self.on_set_control)
        self.set_control_params.pack()

        self.Kp_frame = Frame(self.sidebar_frame)
        self.Kp_frame.pack()
        self.Kp_ang_frame = Frame(self.Kp_frame)
        self.Kp_ang_frame.pack(side='left')
        self.Kp_ang_label = Label(self.Kp_ang_frame, anchor='w', text='K_ang').pack(side='left')
        self.Kp_ang = Entry(self.Kp_ang_frame, width=5)
        self.Kp_ang.insert(END, '300')
        self.Kp_ang.pack(side='right')
        self.Kp_nom_frame = Frame(self.Kp_frame)
        self.Kp_nom_frame.pack(side='right')
        self.Kp_nom_label = Label(self.Kp_nom_frame, anchor='w', text='K_nom').pack(side='left')
        self.Kp_nom = Entry(self.Kp_nom_frame, width=5)
        self.Kp_nom.insert(END, '1000')
        self.Kp_nom.pack(side='right')

        self.throttle_frame = Frame(self.sidebar_frame)
        self.throttle_frame.pack()
        self.fwd_limit_frame = Frame(self.throttle_frame)
        self.fwd_limit_frame.pack(side='left')
        self.fwd_limit_label = Label(self.fwd_limit_frame, anchor='w', text='Fwd Limit').pack(side='left')
        self.fwd_limit = Entry(self.fwd_limit_frame, width=5)
        self.fwd_limit.insert(END, '1000')
        self.fwd_limit.pack(side='right')
        self.bwd_limit_frame = Frame(self.throttle_frame)
        self.bwd_limit_frame.pack(side='right')
        self.bwd_limit_label = Label(self.bwd_limit_frame, anchor='w', text='Bwd Limit').pack(side='left')
        self.bwd_limit = Entry(self.bwd_limit_frame, width=5)
        self.bwd_limit.insert(END, '1000')
        self.bwd_limit.pack(side='right')

        # Tracing border
        self.border = Button(self.sidebar_frame, anchor='w', text='Trace Border', command=self.on_toggle_border)
        self.border.pack()
        self.clear_border = Button(self.sidebar_frame, anchor='w', text='Clear Border', command=self.on_clear_border).pack()
        self.load_border = Button(self.sidebar_frame, anchor='w', text='Load Border', command=self.on_load_border).pack()
        self.save_border = Button(self.sidebar_frame, anchor='w', text='Save Border', command=self.on_save_border).pack()

        # Load map image
        pilImg = Image.open(MAP_FILE)
        pilImg = pilImg.resize((MAP_WIDTH,MAP_HEIGHT), Image.ANTIALIAS)
        self.img = ImageTk.PhotoImage(pilImg)
        
        # map
        self.canvas = Canvas(self.map_frame, width=MAP_WIDTH, height=MAP_HEIGHT)
        self.canvas.create_image(0,0, image=self.img, anchor=NW)
        self.canvas.pack()
        self.canvas.bind("<Button 1>", self.on_location_click)

        self.origin_marker1 = self.canvas.create_line(0, -10, 0, 10, fill='black', width=2)
        self.origin_marker2 = self.canvas.create_line(-10, 0, 10, 0, fill='black', width=2)

    ###########################################################################
    # Location Conversions
    ###########################################################################
    
    def pixel_to_laton(self, col, row):
        img_col = int(col*IMAGE_WIDTH/MAP_WIDTH)
        img_row = int(row*IMAGE_HEIGHT/MAP_HEIGHT)
        x, y = gdal.ApplyGeoTransform(self.geo_trans, img_col, img_row)
        lat, lon = utm.to_latlon(x, y, 11, 'S')
        return lat, lon

    def latlon_to_pixel(self, lat, lon):
        x, y, _,_  = utm.from_latlon(lat, lon)
        img_col, img_row = gdal.ApplyGeoTransform(self.inv_trans, x, y)
        row = int(img_row*MAP_HEIGHT/IMAGE_HEIGHT)
        col = int(img_col*MAP_WIDTH/IMAGE_WIDTH)
        return row, col

    ###########################################################################
    # Mission Callbacks
    ###########################################################################

    def on_toggle_mission(self):
        if self.running_mission_mode:
            # sending a series of way point to the robot
            self.running_mission_mode = False
            self.mission.configure(text='Start Mission')
            
            if self.controller.mode == 'HARDWARE MODE':
                msg = '!ABORTMISSION'
                self.controller.local_xbee.send_data_async(self.controller.boat_xbee, msg.encode())
            elif self.controller.mode == "SIM MODE":
                msg = '!ABORTMISSION'
                xbee_msg= XBeeModel.message.XBeeMessage(msg.encode(), None, None)
                self.controller.robot.xbee_callback(xbee_msg)
           
            print('Mission aborted!')
        else:
            print('Starting mission...')
            self.running_mission_mode = True
            self.mission.configure(text='Abort Mission')

            self.mission_wps = []
            print(self.wp_list.get(0, END))
            for p in self.wp_list.get(0, END):
                lat, lon = list(map(float, p.split(',')))
                x, y,_, _ = utm.from_latlon(lat, lon)
                self.mission_wps.append((x,y))

            if self.controller.mode == 'HARDWARE MODE':
                for x, y in self.mission_wps:
                    #Send waypoints one at a time
                    way_point_msg = "!WP, %f, %f" % (x, y)
                    print(way_point_msg)
                    self.controller.local_xbee.send_data_async(self.controller.boat_xbee, way_point_msg.encode())
                # print('Waiting for wps to send...')
                # time.sleep(2)
                start_mission_msg = "!STARTMISSION"
                self.controller.local_xbee.send_data_async(self.controller.boat_xbee, start_mission_msg.encode())
            else:
                for x, y in self.mission_wps:
                    #Send waypoints one at a time
                    way_point_msg = "!WP, %f, %f" % (x, y)
                    xbee_msg = XBeeModel.message.XBeeMessage(way_point_msg.encode(), None, None)
                    self.controller.robot.xbee_callback(xbee_msg)
                print('Waiting for wps to send...')
                time.sleep(2)
                start_mission_msg = "!STARTMISSION"
                xbee_msg = XBeeModel.message.XBeeMessage(start_mission_msg.encode(), None, None)
                self.controller.robot.xbee_callback(xbee_msg)
            print('Mission started!')

    def on_toggle_add_wps(self):
        if self.add_wps_mode:
            self.add_wps_mode = False
            self.mission_add_wps.configure(text='Add Waypoints')
        else:
            self.add_wps_mode = True
            self.mission_add_wps.configure(text='Done Selecting Waypoints')
        print('Mission planning mode: ', self.add_wps_mode)

    def on_toggle_remove_wps(self):
        if self.remove_wps_mode:
            self.remove_wps_mode = False
            self.mission_remove_wps.configure(text='Remove Waypoints')
        else:
            self.remove_wps_mode = True
            self.mission_remove_wps.configure(text='Done Removing Waypoints')
        print('Remove wps mode: ', self.remove_wps_mode)

    def on_waypoint_selection(self, event):
        selection = self.wp_list.curselection()
        index = selection[0]
        selected_wp = self.wp_markers[index]
        selected_wp_label = self.wp_labels[index]
        if self.remove_wps_mode:
            print('Removing wp...')
            self.wp_markers.pop(index)
            self.wp_labels.pop(index)
            self.wp_list.delete(index)
            self.canvas.delete(selected_wp)
            self.canvas.delete(selected_wp_label)
            for i in range(len(self.wp_labels)): #update wp ordering
                self.canvas.itemconfig(self.wp_labels[i], text=str(i+1))
        else:
            x0, y0, _, _ = self.canvas.coords(selected_wp)
            lat, lon = self.pixel_to_laton(x0 + POINT_RADIUS, y0 + POINT_RADIUS)
            print('wp: ', lat, lon)

    def on_clear_wps(self):
        print('Clearing all waypoints...')
        command_msg = "!CLEARWPS"
        if self.controller.mode == 'HARDWARE MODE':
            self.controller.local_xbee.send_data_async(self.controller.boat_xbee, command_msg.encode())
        else:
            xbee_msg = XBeeModel.message.XBeeMessage(command_msg.encode(), None, None)
            self.controller.robot.xbee_callback(xbee_msg)

        #Clear list and add waypoints
        if self.wp_list.index('end') != 0:
            self.wp_list.delete(0, 'end')
            for marker in self.wp_markers:
                self.canvas.delete(marker)
            for label in self.wp_labels:
                self.canvas.delete(label)
            self.wp_markers = []
            self.wp_labels = []

    def on_load_mission(self):
        mission_file = input('Mission file name? ') #example: sample_mission (file @ Missions/sample_mission.csv)
        coords = []
        with open('Missions/' + mission_file + '.csv', 'r') as f:
            for line in f.readlines():
                if line[-1] == '\n':
                    line = line[:-1]
                coords.append(list(map(float,line.split(','))))

        #Clear list and add waypoints
        if self.wp_list.index('end') != 0:
            self.wp_list.delete(0, 'end')
            for marker in self.wp_markers:
                self.canvas.delete(marker)
            for label in self.wp_labels:
                self.canvas.delete(label)
            self.wp_markers = []
            self.wp_labels = []
        for lat, lon in coords:
            row, col = self.latlon_to_pixel(lat, lon)
            self.wp_markers.append(self.draw_circle(col, row))
            self.wp_labels.append(self.draw_label(col, row))
            self.wp_list.insert('end', self.w_name.cget('text') + str(lat) + ',' + str(lon))

    def on_save_mission(self):
        mission_file = input('Mission file name? ')
        print(mission_file)
        #TODO: check if mission already in directory. if so ask if want to overwrite!
        with open('Missions/' + mission_file + '.csv', 'w') as f:
            for wp in self.wp_list.get(0, 'end'):
                f.write(wp + '\n')


    ###########################################################################
    # Border Callbacks
    ###########################################################################

    def on_toggle_border(self):
        if self.set_border_mode:
            self.set_border_mode = False
            self.border.configure(text='Trace Border')
        else:
            self.set_border_mode = True
            self.border.configure(text='Done Tracing Border')

    def on_clear_border(self):
        for point in self.border_markers:
            self.canvas.delete(point)
        self.border_markers = []

    def on_load_border(self):
        border_file = input('Border file name? ') #example: sample_mission (file @ Missions/sample_mission.csv)
        coords = []
        with open('Borders/' + border_file + '.csv', 'r') as f:
            for line in f.readlines():
                if line[-1] == '\n':
                    line = line[:-1]
                coords.append(list(map(float,line.split(','))))
        self.on_clear_border()
        for lat, lon in coords:
            row, col = self.latlon_to_pixel(lat, lon)
            self.border_markers.append(self.draw_circle(col, row, border=True))

    def on_save_border(self):
        border_file = input('Border file name? ')
        #TODO: check if mission already in directory. if so ask if want to overwrite!
        with open('Borders/' + border_file + '.csv', 'w') as f:
            for point in self.border_markers:
                x0, y0, _, _ = self.canvas.coords(point)
                lat, lon = self.pixel_to_laton(x0 + POINT_RADIUS, y0 + POINT_RADIUS)
                f.write(str(lat) + ',' + str(lon) + '\n')

    ###########################################################################
    # Miscellaneous Callbacks
    ###########################################################################

    def on_toggle_set_origin(self):
        if self.set_origin_mode:
            self.set_origin_mode = False
            self.origin.configure(text='Set Map Origin')
        else:
            self.set_origin_mode = True
            self.origin.configure(text='Select Map Origin...')


    # Function to be called when map location clicked
    def on_location_click(self, event):
        #CHANGE MAP ORIGIN
        if self.set_origin_mode:
            self.canvas.delete(self.origin_marker1)
            self.canvas.delete(self.origin_marker2)
            self.origin_marker1 = self.canvas.create_line(event.x, event.y-10, event.x, event.y+10, fill='black', width=2)
            self.origin_marker2 = self.canvas.create_line(event.x-10, event.y, event.x+10, event.y, fill='black', width=2)
            self.set_origin(event.x, event.y)
            #Reset button
            self.set_origin_mode = False
            self.origin.configure(text='Set Map Origin')

        #ADDING WAYPOINTS TO MISSION
        elif self.add_wps_mode:
            self.wp_markers.append(self.draw_circle(event.x, event.y))
            self.wp_labels.append(self.draw_label(event.x, event.y))
            lat, lon = self.pixel_to_laton(event.x, event.y)
            print ('Adding waypoint: ', lat, lon)
            self.wp_list.insert('end', self.w_name.cget('text') + str(lat) + ',' + str(lon))

    # Function to be called for start/stop
    def on_startstop(self):
        if self.robot_stopped:
            print('Starting motors...')
            command_msg = "!START"
            if self.controller.mode == 'HARDWARE MODE':
                self.controller.local_xbee.send_data_async(self.controller.boat_xbee, command_msg.encode())
            else:
                xbee_msg = XBeeModel.message.XBeeMessage(command_msg.encode(), None, None)
                self.controller.robot.xbee_callback(xbee_msg)
            self.start_stop['text'] = 'Stop ASV'
            self.robot_stopped = False
        else:
            print('Stopping motors!')
            command_msg = "!STOP"
            if self.controller.mode == 'HARDWARE MODE':
                self.controller.local_xbee.send_data_async(self.controller.boat_xbee, command_msg.encode())
            else:
                xbee_msg = XBeeModel.message.XBeeMessage(command_msg.encode(), None, None)
                self.controller.robot.xbee_callback(xbee_msg)
            self.start_stop['text'] = 'Start ASV'
            self.robot_stopped = True

    def on_quit(self):
        print('QUIT!')
        command_msg = "!QUIT"
        if self.controller == None:
            self.quit_gui = True
        elif self.controller.mode == 'HARDWARE MODE':
            self.controller.local_xbee.send_data_async(self.controller.boat_xbee, command_msg.encode())
            self.on_startstop()
        self.tk.destroy()
        self.quit_gui = True

    def on_set_heading_offset(self):
        heading_msg = "!HEADINGOFFSET, %f" % float(self.heading_offset.get())
        print(heading_msg)
        if self.controller.mode == 'HARDWARE MODE':
            self.controller.local_xbee.send_data_async(self.controller.boat_xbee, heading_msg.encode())

    def on_set_desired_speed(self, event):
        print('PRESSED ENTER!')
        speed_msg = '!SETSPEED, %f' % float(self.desired_speed.get())
        print(speed_msg)
        if self.controller.mode == 'HARDWARE MODE':
            self.controller.local_xbee.send_data_async(self.controller.boat_xbee, speed_msg.encode())

    def on_set_control(self):
        control_msg = '!GAIN, %f, %f, %f, %f' % (float(self.Kp_ang.get()), float(self.Kp_nom.get()), float(self.fwd_limit.get()), float(self.bwd_limit.get()))
        print(control_msg)
        if self.controller.mode == 'HARDWARE MODE':
            self.controller.local_xbee.send_data_async(self.controller.boat_xbee, control_msg.encode())

    ###########################################################################
    # Updating GUI
    ###########################################################################

    # Called at every iteration of main loop
    def update(self):
        self.update_GPS()
        self.update_ADCP()

        # Simulation Mode
        if self.controller.mode == "SIM MODE":
            self.controller.robot.sim_loop()

        # update the graphics
        self.tk.update()

        return self.quit_gui == False


    def update_GPS(self):
        '''Get local x, y coordinate from robot. Then convert to utm for graphing'''
        x = self.controller.robot.state_est.x
        y = self.controller.robot.state_est.y
        # print('x,y: ', x, y)

        if self.controller.mode == "SIM MODE":
            heading = self.controller.robot.state_est.theta/ math.pi * 180
        else:
            heading = self.controller.robot.state_est.theta

        # Convert local x y to lat lon
        if x <= 0:
            lat = 0.0
            lon = 0.0
        else:
            lat, lon = utm.to_latlon(x, y, 11, 'S')
        self.gps['text'] = 'Latitude: ' + str(lat) + '\nLongitude: ' + str(lon) + '\nHeading: ' + str(round(heading,2))
 
        # Convert UTM to graphing row and column
        img_col, img_row = gdal.ApplyGeoTransform(self.inv_trans, x, y)
        row = int(img_row*MAP_HEIGHT/IMAGE_HEIGHT)
        col = int(img_col*MAP_WIDTH/IMAGE_WIDTH)

        # Update ASV location on map
        self.draw_arrow(col, row, heading*np.pi/180.)
        self.gps_local['text'] = 'x: ' + str(col - self.origin_coords[0]) + ', y: ' + str(-row + self.origin_coords[1])

        if self.goto_coords[0] != -1:
            self.control_wp_dxdy['text'] = 'dx: ' + str(int(self.goto_coords[0]-x)) + ', dy: ' + str(int(self.goto_coords[1]-y))

        # Add border
        if self.set_border_mode:
            self.border_markers.append(self.draw_circle(col, row, border=True)) #TODO: make this more elegant?

    def update_ADCP(self):
        depth = self.controller.depth
        current = self.controller.cur_speed
        self.adcp_data['text'] = 'Water depth: ' + str(depth) + "\nCurrent speed: " + str(current)

    ###########################################################################
    # Helper Functions
    ###########################################################################
    
    def set_origin(self, col, row):
        self.origin_coords = (col, row)

    def draw_circle(self, x, y, border=False):
        x1, y1 = (x - POINT_RADIUS), (y - POINT_RADIUS)
        x2, y2 = (x + POINT_RADIUS), (y + POINT_RADIUS)
        color = 'red'
        outline = 'black'
        if border:
            color = 'orange'
            outline = 'orange'
        return self.canvas.create_oval(x1, y1, x2, y2, fill=color, outline=outline)

    def draw_label(self, x, y):
        return self.canvas.create_text(x + 2*POINT_RADIUS, y - 2*POINT_RADIUS, text=str(self.wp_list.index('end')+1))

    '''
    Return points to draw arrow at x, y. Points at theta (radians)
    '''
    def draw_arrow(self, x, y, theta):
        dx = POINT_RADIUS*1.5
        dy = POINT_RADIUS*1.5
        p1 = self.rotate([x + dx, y], [x,y], theta)
        p2 = self.rotate([x - 2*dx, y - dy], [x,y], theta)
        p3 = self.rotate([x - dx, y], [x,y], theta)
        p4 = self.rotate([x - 2*dx, y + dy], [x,y], theta)
        points = p1 + p2 + p3 + p4

        if self.cur_pos_marker != None:
            self.canvas.delete(self.cur_pos_marker)

        self.cur_pos_marker = self.canvas.create_polygon(points, fill='yellow', outline='black')

    def rotate(self, p, origin, theta):
        x = origin[0] + (p[0]-origin[0])*np.cos(theta) + (p[1]-origin[1])*np.sin(theta)
        y = origin[1] - (p[0]-origin[0])*np.sin(theta) + (p[1]-origin[1])*np.cos(theta)
        return [x,y]

if __name__ == '__main__':
    my_gui = ASV_graphics(None)
    my_gui.tk.mainloop()
