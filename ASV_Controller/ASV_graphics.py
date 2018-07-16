from tkinter import *
from PIL import Image, ImageTk
import gdal
import utm
import time
import digi.xbee.models as XBeeModel

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
        self.location_select_mode = False
        self.cur_pos_marker = None
        self.target_pos_marker = None
        # Mission planning
        self.add_wps_mode = False
        self.remove_wps_mode = False
        self.running_mission_mode = False
        self.wp_markers = []

        # Frames: Sidebar + Map Area
        self.sidebar_frame = Frame(self.tk, width=300, bg='white', height=500, relief='sunken', borderwidth=2)
        self.map_frame = Frame(self.tk)
        self.sidebar_frame.pack(expand=True, fill='both', side='left', anchor='nw')
        self.map_frame.pack(expand=True, fill='both', side='right')

        # GPS Coordinates
        self.gps_title = Label(self.sidebar_frame, anchor='w', text='ASV GPS Information', font='Helvetica 14 bold').pack()
        self.gps = Label(self.sidebar_frame, anchor='w', width=30, text='Latitude: ???\nLongitude: ???\nHeading: ???\n')
        self.gps.pack()

        # ADCP Data
        self.adcp_title = Label(self.sidebar_frame, anchor='w', text='ADCP Information', font='Helvetica 14 bold').pack()
        self.adcp_data = Label(self.sidebar_frame, anchor='w', width=30, text='Water depth: ???\nCurrent speed: ???\n')
        self.adcp_data.pack()

        # ASV Control Panel
        self.control_title = Label(self.sidebar_frame, anchor='w', text='ASV Control Panel', font='Helvetica 14 bold').pack()

        # 1) Go to map location
        self.control_wp = Label(self.sidebar_frame, anchor='w', width=30, text='Target Waypoint:\nLatitude: ???\nLongitude: ???\n')
        self.control_wp.pack()
        self.goto = Button(self.sidebar_frame, anchor='w', text='Go to Map Location', command=self.on_toggle_goto)
        self.goto.pack()
        self.start_stop = Button(self.sidebar_frame, anchor='w', text='Start ASV', command=self.on_startstop)
        self.start_stop.pack()

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

        # Load map image
        pilImg = Image.open(MAP_FILE)
        pilImg = pilImg.resize((MAP_WIDTH,MAP_HEIGHT), Image.ANTIALIAS)
        self.img = ImageTk.PhotoImage(pilImg)
        
        # map
        self.canvas = Canvas(self.map_frame, width=MAP_WIDTH, height=MAP_HEIGHT)
        self.canvas.create_image(0,0, image=self.img, anchor=NW)
        self.canvas.pack()
        self.canvas.bind("<Button 1>", self.on_location_click)

        # origin
        self.origin_x_utm = 0
        self.origin_y_utm = 0
        self.set_origin(0,0)

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
            #Send mission waypoints to ASV and start mission
            mission_msg = "!MISSION," 
            for x, y in self.mission_wps:
                mission_msg += "%f %f;" % (x, y)

            #TODO: ADD COUNTDOWN BEFORE SENDING COMMANDS
            print('Starting countdown...')
            for i in range(10):
                print(str(10-i))
                time.sleep(1)
            if self.controller.mode == 'HARDWARE MODE':
                self.controller.local_xbee.send_data_async(self.controller.boat_xbee, mission_msg.encode())
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
            print('You are clicking on me!')
            x0, y0, _, _ = self.canvas.coords(selected_wp)
            lat, lon = self.pixel_to_laton(x0 + POINT_RADIUS, y0 + POINT_RADIUS)
            print(lat, lon)

    def on_toggle_goto(self):
        if self.location_select_mode:
            self.location_select_mode = False
            self.goto.configure(text='Go to Map Location')
        else:
            self.location_select_mode = True
            self.goto.configure(text='Select Map Location...')
        print('Go to map location mode: ', self.location_select_mode)

    # Function to be called when map location clicked
    def on_location_click(self, event):
        #ADDING WAYPOINTS TO MISSION
        if self.add_wps_mode:
            x1, y1 = (event.x - POINT_RADIUS), (event.y - POINT_RADIUS)
            x2, y2 = (event.x + POINT_RADIUS), (event.y + POINT_RADIUS)
            self.wp_markers.append(self.canvas.create_oval(x1, y1, x2, y2, fill='blue'))

            lat, lon = self.pixel_to_laton(event.x, event.y)
            print ('Adding waypoint: ', lat, lon)
            self.wp_list.insert('end', self.w_name.cget("text") + str(lat) + ',' + str(lon))

        #GO TO DESTINATION ON MAP
        else:
            if self.location_select_mode == False:
                return
            if self.target_pos_marker == None:
                x1, y1 = (event.x - POINT_RADIUS), (event.y - POINT_RADIUS)
                x2, y2 = (event.x + POINT_RADIUS), (event.y + POINT_RADIUS)
                self.target_pos_marker = self.canvas.create_oval(x1, y1, x2, y2, fill='red')
            else:
                old_pos = self.canvas.coords(self.target_pos_marker)
                dx = event.x - (old_pos[0] + POINT_RADIUS)
                dy = event.y - (old_pos[1] + POINT_RADIUS)
                self.canvas.move(self.target_pos_marker, dx, dy)

            self.go_to_location(event.x, event.y)

            # Reset button
            self.location_select_mode = False
            self.goto.configure(text='Go to Map Location')

    # Function to be called for start/stop
    def on_startstop(self):
        if self.robot_stopped:
            print('Starting motors...')
            command_msg = "!START"
            if self.controller.mode == 'HARDWARE MODE':
                self.controller.local_xbee.send_data_async(self.controller.boat_xbee, command_msg.encode())
            self.start_stop['text'] = 'Stop ASV'
            self.robot_stopped = False
        else:
            print('Stopping motors!')
            command_msg = "!STOP"
            if self.controller.mode == 'HARDWARE MODE':
                self.controller.local_xbee.send_data_async(self.controller.boat_xbee, command_msg.encode())
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

    ###########################################################################
    # Updating GUI
    ###########################################################################

    # Called at every iteration of main loop
    def update(self):
        self.update_GPS()
        self.update_ADCP()

        # Simulation Mode
        if self.controller.mode == "SIM MODE":
            self.controller.robot.update_state(self.controller.robot.state_est, 0, 0)
            print(self.controller.robot.cur_des_point.x)

        # update the graphics
        self.tk.update()

        return self.quit_gui == False


    def update_GPS(self):
        '''Get local x, y coordinate from robot. Then convert to utm for graphing'''
        x_local = self.controller.robot.state_est.x
        y_local = self.controller.robot.state_est.y
        heading = self.controller.robot.state_est.theta

        # Convert local x y to lat lon
        lat, lon = utm.to_latlon(x_local + self.origin_x_utm, y_local + self.origin_y_utm, 11, 'S')
        self.gps['text'] = 'Latitude: ' + str(lat) + '\nLongitude: ' + str(lon) + '\nHeading: ' + str(heading) + '\n'
        
        # Convert local x y to utm
        x_utm = self.origin_x_utm + x_local
        y_utm = self.origin_y_utm + y_local

        # Convert UTM to graphing row and column
        img_col, img_row = gdal.ApplyGeoTransform(self.inv_trans, x_utm, y_utm)

        col = int(img_col*MAP_WIDTH/IMAGE_WIDTH)
        row = int(img_row*MAP_HEIGHT/IMAGE_HEIGHT)

        # Update ASV location on map
        if self.cur_pos_marker == None:
            x1, y1 = (col - POINT_RADIUS), (row - POINT_RADIUS)
            x2, y2 = (col + POINT_RADIUS), (row + POINT_RADIUS)
            self.cur_pos_marker = self.canvas.create_oval(x1, y1, x2, y2, fill='green')
        else:
            old_pos = self.canvas.coords(self.cur_pos_marker)
            dx = col - (old_pos[0] + POINT_RADIUS)
            dy = row - (old_pos[1] + POINT_RADIUS)
            self.canvas.move(self.cur_pos_marker, dx, dy)

    def update_ADCP(self):
        depth = self.controller.depth
        current = self.controller.v_boat
        self.adcp_data['text'] = 'Water depth: ' + str(depth) + "\nCurrent speed: " + str(current) + '\n'

    ###########################################################################
    # ASV Commands
    ###########################################################################
    def set_origin(self, row, col):
        # Send map origin to PI
        x, y = gdal.ApplyGeoTransform(self.geo_trans, row, col)
        self.origin_x_utm = x
        self.origin_y_utm = y

    #Go to location specified by mouse click (pixel coords -> GPS)
    def go_to_location(self, col, row):
        img_col = int(col*IMAGE_WIDTH/MAP_WIDTH)
        img_row = int(row*IMAGE_HEIGHT/MAP_HEIGHT)

        # Send map origin to PI
        self.set_origin(0,0)
        command_msg = "!ORIGIN, %f, %f" % (self.origin_x_utm, self.origin_y_utm)

        x_des_utm, y_des_utm = gdal.ApplyGeoTransform(self.geo_trans, img_col, img_row)
        x_des_local = x_des_utm - self.origin_x_utm # convert to local coordinate
        y_des_local = x_des_utm - self.origin_y_utm

        print('UTM: ', x_des_utm, y_des_utm)
        lat, lon = utm.to_latlon(x_des_utm, y_des_utm, 11, 'S') #11, S is UTM zone for Kern River
        print('Lat/lon: ', lat, lon)
        print('Local: ', x_des_local, y_des_local)
        
        self.control_wp['text'] = 'Target Waypoint:\nLatitude: '+ str(lat) + '\nLongitude: ' + str(lon) + '\n'
        self.target_GPS = (lat, lon)

        #Send command to ASV to move to x, y
        way_point_msg = "!WP, %f, %f" % (x_des_local, y_des_local)

        # Sending data over
        if self.controller.mode == 'HARDWARE MODE':
            self.controller.local_xbee.send_data_async(self.controller.boat_xbee, command_msg.encode())
            self.controller.local_xbee.send_data_async(self.controller.boat_xbee, way_point_msg.encode())
        else:
            way_pt_msg = XBeeModel.message.XBeeMessage(way_point_msg.encode(), None, None)
            origin_msg = XBeeModel.message.XBeeMessage(command_msg.encode(), None, None)

            self.controller.robot.xbee_callback(way_pt_msg)
            self.controller.robot.xbee_callback(origin_msg)

    '''
    Return points to draw arrow at row, col
    '''
    def draw_arrow(self, row, col):
        dx = 5
        dy = 5
        p1 = (row, col + dx)
        p2 = (row - dy, col - 3*dx)
        p3 = (row, col - dx)
        p4 = (row + dy, col - 3*dx)
        return p1, p2, p3, p4
        


if __name__ == '__main__':
    my_gui = ASV_graphics(None)
    my_gui.tk.mainloop()
