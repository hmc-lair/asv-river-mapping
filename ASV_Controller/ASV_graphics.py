from tkinter import *
from PIL import Image, ImageTk
import gdal
import utm
import time
import digi.xbee.models as XBeeModel
import numpy as np

#Milikan pond is 170x110
#Cast is 
IMAGE_WIDTH = 325.
IMAGE_HEIGHT = 250.
MAP_FILE = '../Maps/cast.tif'


# IMAGE_WIDTH = 1000
# IMAGE_HEIGHT = 800
# MAP_FILE = '../Maps/river_section.tif'

MAP_WIDTH = 800
MAP_HEIGHT = 600

POINT_RADIUS = 5

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
        self.wp_markers = []
        # Origin setting
        self.origin_coords = (0,0) #pixel coords
        self.set_origin_mode = False

        # Frames: Sidebar + Map Area
        self.sidebar_frame = Frame(self.tk, width=300, bg='white', height=500, relief='sunken', borderwidth=2)
        self.map_frame = Frame(self.tk)
        self.sidebar_frame.pack(expand=True, fill='both', side='left', anchor='nw')
        self.map_frame.pack(expand=True, fill='both', side='right')

        # GPS Coordinates
        self.gps_title = Label(self.sidebar_frame, anchor='w', text='ASV GPS Information', font='Helvetica 14 bold').pack()
        self.gps = Label(self.sidebar_frame, anchor='w', width=30, text='Latitude: ???\nLongitude: ???\nHeading: ???')
        self.gps_local = Label(self.sidebar_frame, anchor='w', width=30, text='x: ???, y: ???')
        self.gps.pack()
        self.gps_local.pack()

        # ADCP Data
        self.adcp_title = Label(self.sidebar_frame, anchor='w', text='ADCP Information', font='Helvetica 14 bold').pack()
        self.adcp_data = Label(self.sidebar_frame, anchor='w', width=30, text='Water depth: ???\nCurrent speed: ???')
        self.adcp_data.pack()

        # ASV Control Panel
        self.control_title = Label(self.sidebar_frame, anchor='w', text='ASV Control Panel', font='Helvetica 14 bold').pack()
        # 1) Go to map location
        self.control_wp_dxdy = Label(self.sidebar_frame, anchor='w', width=30, text='dx: ???, dy: ???')
        self.control_wp_dxdy.pack()
        self.start_stop = Button(self.sidebar_frame, anchor='w', text='Start ASV', command=self.on_startstop)
        self.start_stop.pack()
        self.clear_wps = Button(self.sidebar_frame, anchor='w', text='Clear All Waypoints', command=self.on_clear_wps).pack()

        # 2) Mission planning
        self.mission_title = Label(self.sidebar_frame, anchor='w', text='Mission Planning', font='Helvetica 14 bold').pack()
        scrollbar = Scrollbar(self.sidebar_frame)
        scrollbar.pack(side='right')
        self.w_name = Label(self.sidebar_frame, text='WP: ')#some label
        self.wp_list = Listbox(self.sidebar_frame, width=30, yscrollcommand=scrollbar.set)
        self.wp_list.pack()
        self.wp_list.bind('<<ListboxSelect>>', self.on_waypoint_selection)
        scrollbar.config(command = self.wp_list.yview)

        self.mission_add_wps = Button(self.sidebar_frame, anchor='w', text='Add Waypoints', command=self.on_toggle_add_wps)
        self.mission_add_wps.pack()
        self.mission_remove_wps = Button(self.sidebar_frame, anchor='w', text='Remove Waypoints', command=self.on_toggle_remove_wps)
        self.mission_remove_wps.pack()
        self.mission = Button(self.sidebar_frame, anchor='w', text='Start Mission', command=self.on_toggle_mission)
        self.mission.pack()

        # Map Configuration
        self.map_config = Label(self.sidebar_frame, anchor='w', text='Configuration', font='Helvetica 14 bold').pack()
        self.origin = Button(self.sidebar_frame, anchor='w', text='Set Map Origin', command=self.on_toggle_set_origin)
        self.origin.pack()
        self.set_heading_offset = Button(self.sidebar_frame, anchor='w', text='Set Heading Offset', command=self.on_set_heading_offset)
        self.set_heading_offset.pack()
        self.heading_offset_label = Label(self.sidebar_frame, anchor='w', text='Heading Offset (deg)').pack(side='left')
        self.heading_offset = Entry(self.sidebar_frame, width=10)
        self.heading_offset.insert(END, '-20')
        self.heading_offset.pack(side='right')

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

    ###########################################################################
    # Callbacks
    ###########################################################################

    def on_toggle_mission(self):
        if self.running_mission_mode:
            self.running_mission_mode = False
            self.mission.configure(text='Start Mission')
            if self.controller.mode == 'HARDWARE MODE':
                msg = '!ABORTMISSION'
                self.controller.local_xbee.send_data_async(self.controller.boat_xbee, msg.encode())
            print('Mission aborted!')
        else:
            print('Starting mission...')
            self.running_mission_mode = True
            self.mission.configure(text='Abort Mission')

            self.mission_wps = []
            for p in self.wp_list.get(0, END):
                lat, lon = list(map(float, p.split(' ')[1].split(',')))
                x, y,_, _ = utm.from_latlon(lat, lon)
                self.mission_wps.append((x,y))

            if self.controller.mode == 'HARDWARE MODE':
                for x, y in self.mission_wps:
                    #Send waypoints one at a time
                    way_point_msg = "!WP, %f, %f" % (x, y)
                    self.controller.local_xbee.send_data_async(self.controller.boat_xbee, way_point_msg.encode())
                start_mission_msg = "!STARTMISSION"
                self.controller.local_xbee.send_data_async(self.controller.boat_xbee, start_mission_msg.encode())
            else:
                for x, y in self.mission_wps:
                    #Send waypoints one at a time
                    way_point_msg = "!WP, %f, %f" % (x, y)
                    xbee_msg = XBeeModel.message.XBeeMessage(way_point_msg.encode(), None, None)
                    self.controller.robot.xbee_callback(xbee_msg)
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
        if self.remove_wps_mode:
            print('Removing wp...')
            self.wp_markers.pop(index)
            self.wp_list.delete(index)
            self.canvas.delete(selected_wp)
        else:
            x0, y0, _, _ = self.canvas.coords(selected_wp)
            lat, lon = self.pixel_to_laton(x0 + POINT_RADIUS, y0 + POINT_RADIUS)
            print('Added wp: ', lat, lon)

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
            x1, y1 = (event.x - POINT_RADIUS), (event.y - POINT_RADIUS)
            x2, y2 = (event.x + POINT_RADIUS), (event.y + POINT_RADIUS)
            self.wp_markers.append(self.canvas.create_oval(x1, y1, x2, y2, fill='blue'))

            lat, lon = self.pixel_to_laton(event.x, event.y)
            print ('Adding waypoint: ', lat, lon)
            self.wp_list.insert('end', self.w_name.cget("text") + str(lat) + ',' + str(lon))

    def on_clear_wps(self):
        print('Clearing all waypoints...')
        command_msg = "!CLEARWPS"
        if self.controller.mode == 'HARDWARE MODE':
            self.controller.local_xbee.send_data_async(self.controller.boat_xbee, command_msg.encode())
        else:
            xbee_msg = XBeeModel.message.XBeeMessage(command_msg.encode(), None, None)
            self.controller.robot.xbee_callback(xbee_msg)

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
        heading = self.controller.robot.state_est.theta

        # Convert local x y to lat lon
        if x <= 0:
            lat = 0.0
            lon = 0.0
        else:
            lat, lon = utm.to_latlon(x, y, 11, 'S')
        self.gps['text'] = 'Latitude: ' + str(round(lat, 5)) + '\nLongitude: ' + str(round(lon, 5)) + '\nHeading: ' + str(round(heading,2))
 
        # Convert UTM to graphing row and column
        img_col, img_row = gdal.ApplyGeoTransform(self.inv_trans, x, y)
        row = int(img_row*MAP_HEIGHT/IMAGE_HEIGHT)
        col = int(img_col*MAP_WIDTH/IMAGE_WIDTH)

        # Update ASV location on map
        self.draw_arrow(col, row, heading*np.pi/180.)
        self.gps_local['text'] = 'x: ' + str(col - self.origin_coords[0]) + ', y: ' + str(-row + self.origin_coords[1])

        if self.goto_coords[0] != -1:
            self.control_wp_dxdy['text'] = 'dx: ' + str(int(self.goto_coords[0]-x)) + ', dy: ' + str(int(self.goto_coords[1]-y))

    def update_ADCP(self):
        depth = self.controller.depth
        current = self.controller.v_boat
        self.adcp_data['text'] = 'Water depth: ' + str(depth) + "\nCurrent speed: " + str(current)

    ###########################################################################
    # ASV Commands
    ###########################################################################
    def set_origin(self, col, row):
        self.origin_coords = (col, row)

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
