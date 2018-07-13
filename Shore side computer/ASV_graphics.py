from tkinter import *
from PIL import Image, ImageTk
import gdal
import utm
import time

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

        # Set up GUI
        self.controller = controller
        self.tk = Tk()
        self.tk.title("ASV Control Interface")
        self.robot_stopped = True
        self.quit_gui = False
        self.tk.protocol("WM_DELETE_WINDOW", self.on_quit)
        # lat, lon = utm.to_latlon(x, y, 11, 'S') #11, S is UTM zone for Kern River
        # print('Top left Lat/lon: ', lat, lon)

        # Frames: Sidebar + Map Area
        self.sidebar_frame = Frame(self.tk, width=300, bg='white', height=500, relief='sunken', borderwidth=2)
        self.map_frame = Frame(self.tk)
        self.sidebar_frame.pack(expand=True, fill='both', side='left', anchor='nw')
        self.map_frame.pack(expand=True, fill='both', side='right')

        # Position variables (labels on map)
        self.cur_pos = None
        self.target_pos = None
        self.location_select_mode = False

        # GPS Coordinates
        self.gps_title = Label(self.sidebar_frame, anchor='w', text='ASV GPS Information').pack()
        self.gps = Label(self.sidebar_frame, anchor='w', width=30, text='Latitude: ???\nLongitude: ???\nHeading: ???\n')
        self.gps.pack()

        # ADCP Data
        self.adcp_title = Label(self.sidebar_frame, anchor='w', text='ADCP Information').pack()
        self.adcp_data = Label(self.sidebar_frame, anchor='w', width=30, text='Water depth: ???\nCurrent speed: ???\n')
        self.adcp_data.pack()

        # Control Panel
        self.control_title = Label(self.sidebar_frame, anchor='w', text='ASV Control Panel').pack()
        self.control_waypoint = Label(self.sidebar_frame, anchor='w', width=30, text='Target Waypoint:\nLatitude: ???\nLongitude: ???\n')
        self.control_waypoint.pack()

        # Buttons
        self.goto = Button(self.sidebar_frame, anchor='w', text='Go to Map Location', command=self.on_toggle_goto)
        self.goto.pack()
        self.start_stop = Button(self.sidebar_frame, anchor='w', text='Start ASV', command=self.on_startstop)
        self.start_stop.pack()
        #self.quit = Button(self.sidebar_frame, anchor='w', text='QUIT GUI', command=self.on_quit).pack()

        # Load map image
        pilImg = Image.open(MAP_FILE)
        pilImg = pilImg.resize((MAP_WIDTH,MAP_HEIGHT), Image.ANTIALIAS)
        self.img = ImageTk.PhotoImage(pilImg)
        
        # map
        self.canvas = Canvas(self.map_frame, width=MAP_WIDTH, height=MAP_HEIGHT)
        self.canvas.create_image(0,0, image=self.img, anchor=NW)
        self.canvas.pack()
        self.canvas.bind("<Button 1>", self.on_location_click)

    ###########################################################################
    # Callbacks
    ###########################################################################

    def on_toggle_goto(self):
        if self.location_select_mode:
            self.location_select_mode = False
            self.goto.configure(text='Go to Map Location')
        else:
            self.location_select_mode = True
            self.goto.configure(text='Select Map Location...')
        print('Mode: ', self.location_select_mode)

    # Function to be called when map location clicked
    def on_location_click(self, event):
        if self.location_select_mode == False:
            return
        print(event.x, event.y)

        if self.target_pos == None:
            x1, y1 = (event.x - POINT_RADIUS), (event.y - POINT_RADIUS)
            x2, y2 = (event.x + POINT_RADIUS), (event.y + POINT_RADIUS)
            self.target_pos = self.canvas.create_oval(x1, y1, x2, y2, fill='red')
        else:
            old_pos = self.canvas.coords(self.target_pos)
            dx = event.x - (old_pos[0] + POINT_RADIUS)
            dy = event.y - (old_pos[1] + POINT_RADIUS)
            self.canvas.move(self.target_pos, dx, dy)

        self.go_to_location(event.x, event.y)

        # Reset button
        self.location_select_mode = False
        self.goto.configure(text='Go to Map Location')

    # Function to be called for start/stop
    def on_startstop(self):
        if self.robot_stopped:
            print('Starting motors...')
            command_msg = "!START"
            if self.controller.boat_xbee != None:
                self.controller.local_xbee.send_data_async(self.controller.boat_xbee, command_msg.encode())
            self.start_stop['text'] = 'Stop ASV'
            self.robot_stopped = False
        else:
            print('Stopping motors!')
            command_msg = "!STOP"
            if self.controller.boat_xbee != None:
                self.controller.local_xbee.send_data_async(self.controller.boat_xbee, command_msg.encode())
            self.start_stop['text'] = 'Start ASV'
            self.robot_stopped = True

    def on_quit(self):
        print('QUIT!')
        command_msg = "!QUIT"
        if self.controller.boat_xbee != None:
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

        # update the graphics
        self.tk.update()

        return self.quit_gui == False

    def update_GPS(self):
        lat = self.controller.robot.lat
        lon = self.controller.robot.lon
        heading = self.controller.robot.heading

        self.gps['text'] = 'Latitude: ' + str(lat) + '\nLongitude: ' + str(lon) + '\nHeading: ' + str(heading) + '\n'
        x, y,_, _ = utm.from_latlon(lat, lon)
        img_col, img_row = gdal.ApplyGeoTransform(self.inv_trans, x, y)

        col = int(img_col*MAP_WIDTH/IMAGE_WIDTH)
        row = int(img_row*MAP_HEIGHT/IMAGE_HEIGHT)

        # Update ASV location on map
        if self.cur_pos == None:
            x1, y1 = (col - POINT_RADIUS), (row - POINT_RADIUS)
            x2, y2 = (col + POINT_RADIUS), (row + POINT_RADIUS)
            self.cur_pos = self.canvas.create_oval(x1, y1, x2, y2, fill='green')
        else:
            old_pos = self.canvas.coords(self.cur_pos)
            dx = col - (old_pos[0] + POINT_RADIUS)
            dy = row - (old_pos[1] + POINT_RADIUS)
            self.canvas.move(self.cur_pos, dx, dy)

    def update_ADCP(self):
        depth = self.controller.depth
        current = self.controller.v_boat

        # self.adcp_depth['text'] = 'Water depth: ' + str(depth)
        # self.adcp_current['text'] = "Current speed: " + str(current) + '\n'
        self.adcp_data['text'] = 'Water depth: ' + str(depth) + "\nCurrent speed: " + str(current) + '\n'

    ###########################################################################
    # ASV Commands
    ###########################################################################

    #Go to location specified by mouse click (pixel coords -> GPS)
    def go_to_location(self, col, row):
        img_col = int(col*IMAGE_WIDTH/MAP_WIDTH)
        img_row = int(row*IMAGE_HEIGHT/MAP_HEIGHT)

        # Send map origin to PI
        x, y = gdal.ApplyGeoTransform(self.geo_trans, 0, 0)
        command_msg = "!ORIGIN, %f, %f" % (x, y)
        self.controller.local_xbee.send_data_async(self.controller.boat_xbee, command_msg.encode())

        x, y = gdal.ApplyGeoTransform(self.geo_trans, img_col, img_row)
        print('UTM: ', x, y)
        lat, lon = utm.to_latlon(x, y, 11, 'S') #11, S is UTM zone for Kern River
        print('Lat/lon: ', lat, lon)
        self.control_waypoint['text'] = 'Target Waypoint:\nLatitude: '+ str(lat) + '\nLongitude: ' + str(lon) + '\n'

        #Send command to ASV to move to x, y
        way_point_msg = "!WP, %f, %f" % (x, y)
        self.controller.local_xbee.send_data_async(self.controller.boat_xbee, way_point_msg.encode())

    #Display ASV path plan 
    # positions: (row,col)
    def show_path(self, positions):
        pass


if __name__ == '__main__':
    my_gui = ASV_graphics(None)
    my_gui.tk.mainloop()
