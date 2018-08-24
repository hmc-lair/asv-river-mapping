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

# IMAGE_WIDTH = 200
# IMAGE_HEIGHT = 150
# MAP_FILE = '../Maps/millikan.tif'

# IMAGE_WIDTH = 1000
# IMAGE_HEIGHT = 700
# MAP_FILE = '../Maps/lake_7-27.tif'

IMAGE_WIDTH = 300
IMAGE_HEIGHT = 200
MAP_FILE = '../Maps/river_8-13.tif'

MAP_WIDTH = 600
MAP_HEIGHT = 400

POINT_RADIUS = 5

point_track_color = 'red'
transect_color = 'orange'
border_color = 'green'

'''
Pop-up window for clearing waypoints (are you sure you want to do this)
'''
class popupWindow(object):
    def __init__(self,master):
        top=self.top=Toplevel(master)
        self.l=Label(top,text="Remove all waypoints?").pack()
        response = ''
        self.yes=Button(top,text='Yes',command=self.cleanup_yes).pack(side='left')
        self.no=Button(top,text='No',command=self.cleanup_no).pack(side='right')
        self.response = None

    def cleanup_yes(self):
        self.response = True
        self.top.destroy()

    def cleanup_no(self):
        self.response = False
        self.top.destroy()

'''
Handle exceptions in Tk loop
'''
def handle_exception(exception, value, traceback):
    print("Caught exception:", exception)
    print("Try again...")

'''
Main class for ASV graphics
'''
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
        self.tk.report_callback_exception=handle_exception
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
        self.transect_mode = False
        # self.repeat_mission_mode = False
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
        self.sidebar_frame = Frame(self.tk, width=500, relief='sunken', borderwidth=2)
        self.map_frame = Frame(self.tk)
        self.sidebar_frame.pack(expand=True, fill='both', side='left', anchor='nw')
        self.map_frame.pack(expand=True, fill='both', side='right')


        #######################################################################
        # SIDEBAR FRAME
        #######################################################################

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

        self.auv_status = Label(self.sidebar_frame, anchor='w', text='ASV Status: STOPPED', fg="red")
        self.auv_status.pack()

        self.transect_mission_label = Label(self.sidebar_frame, anchor='w', text='Transect Mission: NO', fg="red")
        self.transect_mission_label.pack()
        self.transect_mission = Button(self.sidebar_frame, anchor='w', text='Enable Transect Mission', command=self.on_toggle_transect)
        self.transect_mission.pack()

        # Repeat mission (has been commented out because not useful)
        # self.repeat_mission_label = Label(self.sidebar_frame, anchor='w', text='Repeat Mission: NO', fg="red")
        # self.repeat_mission_label.pack()

        # self.repeat_frame = Frame(self.sidebar_frame)
        # self.repeat_frame.pack()
        # self.repeat_label = Label(self.repeat_frame, anchor='w', text='# Repeats').pack(side='left')
        # self.repeat_times = Entry(self.repeat_frame, width=5)
        # self.repeat_times.insert(END, '10')
        # self.repeat_times.pack(side='right')
        # self.repeat_mission = Button(self.sidebar_frame, anchor='w', text='Enable Repeat Mission', command=self.on_toggle_repeat_mission)
        # self.repeat_mission.pack()

        self.start_frame = Frame(self.sidebar_frame)
        self.start_frame.pack()
        self.start_stop = Button(self.start_frame, anchor='w', text='Start ASV', command=self.on_startstop)
        self.start_stop.pack(side='left')
        self.mission = Button(self.start_frame, anchor='w', text='Start Mission', command=self.on_toggle_mission)
        self.mission.pack(side='right')


        # 2) Mission planning
        self.mission_title = Label(self.sidebar_frame, anchor='w', text='Mission Planning', font='Helvetica 14 bold').pack()
        self.mission_disclaimer = Label(self.sidebar_frame, 
            anchor='w', text='Red=Point Tracking(1), Orange=Transect(2).\nClick on WP in map to change color.', font='Helvetica 10 italic').pack()
        self.mission_frame = Frame(self.sidebar_frame)
        self.scrollbar = Scrollbar(self.mission_frame)
        self.scrollbar.pack(side='right')
        self.w_name = Label(self.mission_frame, text='')
        self.wp_list = Listbox(self.mission_frame, width=30, height=12, yscrollcommand=self.scrollbar.set)
        self.wp_list.pack(side='left')
        self.wp_list.bind('<<ListboxSelect>>', self.on_waypoint_selection)
        self.scrollbar.config(command = self.wp_list.yview)
        self.mission_frame.pack()

        self.mission_file_frame = Frame(self.sidebar_frame)
        self.mission_file_frame.pack()
        self.load_mission = Button(self.mission_file_frame, anchor='w', text='Load Mission File', command=self.on_load_mission).pack(side='left')
        self.save_mission = Button(self.mission_file_frame, anchor='w', text='Save Mission to File', command=self.on_save_mission).pack(side='right')

        self.mission_add_wps = Button(self.sidebar_frame, anchor='w', text='Add Waypoints', command=self.on_toggle_add_wps)
        self.mission_add_wps.pack()
        self.mission_remove_wps = Button(self.sidebar_frame, anchor='w', text='Remove Waypoints', command=self.on_toggle_remove_wps)
        self.mission_remove_wps.pack()

        self.clear_wps = Button(self.sidebar_frame, anchor='w', text='Clear All Waypoints', command=self.on_clear_wps)
        self.clear_wps.pack()

        #######################################################################
        # MAP CANVAS
        #######################################################################

        # Load map image
        pilImg = Image.open(MAP_FILE)
        pilImg = pilImg.resize((MAP_WIDTH,MAP_HEIGHT), Image.ANTIALIAS)
        self.img = ImageTk.PhotoImage(pilImg)
        
        # map
        self.canvas = Canvas(self.map_frame, width=MAP_WIDTH, height=MAP_HEIGHT)
        self.canvas.create_image(0,0, image=self.img, anchor=NW)
        self.canvas.pack(side='top')
        self.canvas.bind("<Button 1>", self.on_location_click)
        self.origin_marker1 = self.canvas.create_line(0, -10, 0, 10, fill='black', width=2) #to create a cross
        self.origin_marker2 = self.canvas.create_line(-10, 0, 10, 0, fill='black', width=2)

        #######################################################################
        # CONFIGURATION FRAME
        #######################################################################

        self.control_config_frame = Frame(self.map_frame, height=15)
        self.control_config_frame.pack(side='left')

        self.misc_frame = Frame(self.map_frame)
        self.misc_frame.pack(side='right')

        self.border_frame = Frame(self.misc_frame, height=15)
        self.border_frame.pack(side='top')
        self.compass_frame = Frame(self.misc_frame, height=15)
        self.compass_frame.pack(side='bottom')

        # Tracing border
        self.border_label = Label(self.border_frame, anchor='w', text='Border Configuration', font='Helvetica 14 bold').pack()
        self.border = Button(self.border_frame, anchor='w', text='Trace Border', command=self.on_toggle_border)
        self.border.pack()
        self.clear_border = Button(self.border_frame, anchor='w', text='Clear Border', command=self.on_clear_border).pack()
        self.load_border = Button(self.border_frame, anchor='w', text='Load Border', command=self.on_load_border).pack()
        self.save_border = Button(self.border_frame, anchor='w', text='Save Border', command=self.on_save_border).pack()

        # Compass calibration
        self.compass_label = Label(self.compass_frame, anchor='w', text='Compass Calibration', font='Helvetica 14 bold').pack()
        self.heading_frame = Frame(self.compass_frame)
        self.heading_frame.pack()
        self.heading_offset_label = Label(self.heading_frame, anchor='w', text='Heading Offset (deg)').pack(side='left')
        self.heading_offset = Entry(self.heading_frame, width=6)
        self.heading_offset.insert(END, '-12')
        self.heading_offset.pack(side='right')
        self.set_heading_offset = Button(self.compass_frame, anchor='w', text='Set Heading Offset', command=self.on_set_heading_offset).pack()

        # Control parameters/speed
        self.control_config = Label(self.control_config_frame, anchor='w', text='Control', font='Helvetica 14 bold').pack()
        self.control_info = Label(self.control_config_frame, anchor='w', text='ASV Speed: ?\n Speed to Dest: ?\n', fg="red")
        self.control_info.pack()
        # self.origin = Button(self.sidebar_frame, anchor='w', text='Set Map Origin', command=self.on_toggle_set_origin)
        # self.origin.pack()

        self.speed_frame = Frame(self.control_config_frame)
        self.speed_frame.pack()
        # self.set_desired_speed = Button(self.speed_frame, anchor='w', text='Set Desired Speed', command=self.on_set_desired_speed)
        # self.set_desired_speed.pack()
        # self.desired_speed_label = Label(self.speed_frame, anchor='w', text='Desired Speed (m/s)').pack(side='left')
        # self.desired_speed = Entry(self.speed_frame, width=10)

        # self.desired_speed.insert(END, '3')
        # self.desired_speed.bind('<Return>', self.on_set_desired_speed)
        # self.desired_speed.pack(side='right')

        #######################################################################
        # Point Tracking Parameters
        #######################################################################

        self.Kp_frame = Frame(self.control_config_frame)
        self.Kp_frame.pack()

        self.Kp_ang_label = Label(self.Kp_frame, anchor='w', text='K_ang').pack(side='left')
        self.Kp_ang = Entry(self.Kp_frame, width=5)
        self.Kp_ang.insert(END, '800')
        self.Kp_ang.pack(side='left')
        self.Kp_nom_label = Label(self.Kp_frame, anchor='w', text='K_nom').pack(side='left')
        self.Kp_nom = Entry(self.Kp_frame, width=5)
        self.Kp_nom.insert(END, '1000')
        self.Kp_nom.pack(side='left')
        self.K_vi_label = Label(self.Kp_frame, anchor='w', text='K_vi').pack(side='left')
        self.K_vi = Entry(self.Kp_frame, width=5)
        self.K_vi.insert(END, '0')
        self.K_vi.pack(side='left')

        #######################################################################
        # Transect Parameters
        #######################################################################

        self.transect_frame1 = Frame(self.control_config_frame)
        self.transect_frame1.pack()
        self.K_v_label = Label(self.transect_frame1, anchor='w', text='K_v').pack(side='left')
        self.K_v = Entry(self.transect_frame1, width=5)
        self.K_v.insert(END, '0.5')
        self.K_v.pack(side='left')
        self.K_latAng_label = Label(self.transect_frame1, anchor='w', text='K_latAng').pack(side='left')
        self.K_latAng = Entry(self.transect_frame1, width=5)
        self.K_latAng.insert(END, '0.5')
        self.K_latAng.pack(side='left')
        self.K_vert_label = Label(self.transect_frame1, anchor='w', text='K_vert').pack(side='left')
        self.K_vert = Entry(self.transect_frame1, width=5)
        self.K_vert.insert(END, '1000')
        self.K_vert.pack(side='left')

        self.transect_frame2 = Frame(self.control_config_frame)
        self.transect_frame2.pack()
        self.v_rate_label = Label(self.transect_frame2, anchor='w', text='v_rate').pack(side='left')
        self.v_rate = Entry(self.transect_frame2, width=5)
        self.v_rate.insert(END, '5')
        self.v_rate.pack(side='left')
        self.a_rate_label = Label(self.transect_frame2, anchor='w', text='a_rate').pack(side='left')
        self.a_rate = Entry(self.transect_frame2, width=5)
        self.a_rate.insert(END, '5')
        self.a_rate.pack(side='left')
        self.vx_des_label = Label(self.transect_frame2, anchor='w', text='vx_des').pack(side='left')
        self.vx_des = Entry(self.transect_frame2, width=5)
        self.vx_des.insert(END, '1')
        self.vx_des.pack(side='left')

        #######################################################################
        # Throttle Thresholds
        #######################################################################

        self.throttle_frame = Frame(self.control_config_frame)
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

        self.set_control_params = Button(self.control_config_frame, anchor='w', text='Set Control Params', command=self.on_set_control).pack()

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

    def get_mission_wps(self, num_repeats):
        wps = []
        for i in range(num_repeats):
            count = 0
            for p in self.wp_list.get(0, END):
                lat, lon = list(map(float, p.split(',')))
                x, y,_, _ = utm.from_latlon(lat, lon)

                #get color
                color = self.canvas.itemcget(self.wp_markers[count], 'fill')
                mode = 1 #point track
                if color == transect_color:
                    mode = 2 #transect
                # if count == 0:
                #     mode = 1
                # else:
                #     mode = 2

                wps.append((x,y,mode))
                count += 1
        return wps


    def on_toggle_mission(self):
        if self.running_mission_mode:
            # sending a series of way point to the robot
            self.running_mission_mode = False
            self.mission.configure(text='Start Mission')
            
            if self.controller.mode == 'HARDWARE MODE':
                msg = '!ABORTMISSION'
                self.controller.local_xbee.send_data_async(self.controller.boat_xbee, msg.encode())
                msg = '!CLEARWPS' #also clear waypoints
                self.controller.local_xbee.send_data_async(self.controller.boat_xbee, msg.encode())
            elif self.controller.mode == "SIM MODE":
                msg = '!ABORTMISSION'
                xbee_msg= XBeeModel.message.XBeeMessage(msg.encode(), None, None)
                self.controller.robot.xbee_callback(xbee_msg)
                msg = '!CLEARWPS' #also clear waypoints
                xbee_msg= XBeeModel.message.XBeeMessage(msg.encode(), None, None)
                self.controller.robot.xbee_callback(xbee_msg)
           
            print('Mission aborted!')
        else:
            print('Starting mission...')
            self.running_mission_mode = True
            self.mission.configure(text='Abort Mission')

            # if self.repeat_mission_mode:
            #     num_repeats = int(self.repeat_times.get())
            #     self.mission_wps = self.get_mission_wps(num_repeats)
            # else:
            self.mission_wps = self.get_mission_wps(1)

            print('Waypoints:', self.mission_wps)

            if self.controller.mode == 'HARDWARE MODE':
                wp_checksum_msg = '!WPNUM, %d' % len(self.mission_wps)
                self.controller.local_xbee.send_data_async(self.controller.boat_xbee, wp_checksum_msg.encode())
                time.sleep(0.1)
                for x, y, mode in self.mission_wps:
                    #Send waypoints one at a time
                    way_point_msg = "!WP, %f, %f, %d" % (x, y, mode)
                    print(way_point_msg)
                    self.controller.local_xbee.send_data_async(self.controller.boat_xbee, way_point_msg.encode())
                    time.sleep(0.1)
                start_mission_msg = "!STARTMISSION"
                self.controller.local_xbee.send_data_async(self.controller.boat_xbee, start_mission_msg.encode())
            else:
                for x, y, mode in self.mission_wps:
                    #Send waypoints one at a time
                    way_point_msg = "!WP, %f, %f, %d" % (x, y, mode)
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
            self.mission_add_wps.configure(text='Done Adding Waypoints')
        print('Add wps mode: ', self.add_wps_mode)

    def on_toggle_remove_wps(self):
        if self.remove_wps_mode:
            self.remove_wps_mode = False
            self.mission_remove_wps.configure(text='Remove Waypoints')
        else:
            self.remove_wps_mode = True
            self.mission_remove_wps.configure(text='Done Removing Waypoints')
        print('Remove wps mode: ', self.remove_wps_mode)

    def on_toggle_transect(self):
        if self.transect_mode:
            self.transect_mode = False
            self.transect_mission.configure(text='Enable Transect Mission')
        else:
            self.transect_mode = True
            self.transect_mission.configure(text='Disable Transect Mission')
        print('Transect mission mode: ', self.transect_mode)
        transect_text = 'NO'
        if self.transect_mode:
            transect_text = 'YES'
        self.transect_mission_label['text'] = 'Transect Mission: ' + transect_text

        #send update to ASV
        msg = '!TRANSECT, %d' % int(self.transect_mode)
        print(msg)
        if self.controller.mode == 'HARDWARE MODE':
            self.controller.local_xbee.send_data_async(self.controller.boat_xbee, msg.encode())
        else:
            xbee_msg = XBeeModel.message.XBeeMessage(msg.encode(), None, None)
            self.controller.robot.xbee_callback(xbee_msg)

    # def on_toggle_repeat_mission(self):
    #     if self.repeat_mission_mode:
    #         self.repeat_mission_mode = False
    #         self.repeat_mission.configure(text='Enable Repeat Mission')
    #     else:
    #         self.repeat_mission_mode = True
    #         self.repeat_mission.configure(text='Disable Repeat Mission')
    #     print('Repeat mission mode: ', self.repeat_mission_mode)
    #     repeat_text = 'NO'
    #     if self.repeat_mission_mode:
    #         repeat_text = 'YES'
    #     self.repeat_mission_label['text'] = 'Repeat Mission: ' + repeat_text

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

    def on_wp_click(self, event):
        items = event.widget.find_closest(event.x, event.y)
        if items:
            wp_id = items[0]
            if self.canvas.itemcget(wp_id, 'fill') == point_track_color:
                self.canvas.itemconfigure(wp_id, fill=transect_color)
            else:
                self.canvas.itemconfigure(wp_id, fill=point_track_color)

    def on_clear_wps(self):
        self.w=popupWindow(self.sidebar_frame)
        self.clear_wps['state'] = 'disabled'
        self.sidebar_frame.wait_window(self.w.top)
        self.clear_wps['state'] = 'normal'

        if self.w.response:
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
        else:
            print('Cancelling clear waypoint request')

    def on_load_mission(self):
        mission_file = input('Mission file name (exclude ".csv")? ') #example: sample_mission (file @ Missions/sample_mission.csv)
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
            self.canvas.tag_bind(self.wp_markers[-1], '<ButtonPress-1>', self.on_wp_click) 

    def on_save_mission(self):
        mission_file = input('Mission file name (exclude ".csv")? ')
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
        border_file = input('Border file name (exclude ".csv")? ') #example: sample_mission (file @ Missions/sample_mission.csv)
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
        border_file = input('Border file name (exclude ".csv")? ')
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
            self.canvas.tag_bind(self.wp_markers[-1], '<ButtonPress-1>', self.on_wp_click)       

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

        status_text = 'STARTED'
        if self.robot_stopped:
            status_text = 'STOPPED'
        self.auv_status['text'] = 'AUV Status: ' + status_text

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

    def on_set_control(self):
        control_msg = '!GAIN, %f, %f, %f, %f' % (float(self.Kp_ang.get()), float(self.Kp_nom.get()), float(self.fwd_limit.get()), float(self.bwd_limit.get()))
        print(control_msg)
        if self.controller.mode == 'HARDWARE MODE':
            self.controller.local_xbee.send_data_async(self.controller.boat_xbee, control_msg.encode())

        #Also send transect params...
        control_msg = '!CONTROL, %f, %f, %f, %d, %d, %f, %f' % (float(self.K_v.get()), float(self.K_latAng.get()), float(self.K_vert.get()), int(self.v_rate.get()), int(self.a_rate.get()), float(self.vx_des.get()), float(self.K_vi.get()))
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
        self.update_speeds()

        # Simulation Mode
        if self.controller.mode == "SIM MODE":
            self.controller.robot.sim_loop()

        # update the graphics
        self.tk.update()

        return self.quit_gui == False

    def update_speeds(self):
        asv_speed = self.controller.robot.state_est.v_course
        to_dest = self.controller.v_x
        self.control_info['text'] = 'ASV Speed: ' + str(round(asv_speed, 3)) + '\nSpeed to Dest: ' + str(round(to_dest,3)) + '\n'

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
        color = point_track_color
        outline = 'black'
        if border:
            color = border_color
            outline = border_color
        if self.transect_mode:
            color = transect_color
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
