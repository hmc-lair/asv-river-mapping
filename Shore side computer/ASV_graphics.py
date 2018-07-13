from tkinter import *
from PIL import Image, ImageTk
import gdal
import utm

#Milikan pond is 170x110

MAP_WIDTH = 1000
MAP_HEIGHT = 800
IMAGE_WIDTH = 1000
IMAGE_HEIGHT = 800
MAP_FILE = '../Maps/river_section.tif'

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
        self.adcp_depth = Label(self.sidebar_frame, anchor='w', text='Water depth: ???')
        self.adcp_depth.pack()
        self.adcp_current = Label(self.sidebar_frame, anchor='w', text='Current speed: ???\n')
        self.adcp_current.pack()

        # Control Panel
        self.control_title = Label(self.sidebar_frame, anchor='w', text='ASV Control Panel').pack()
        self.control_waypoint = Label(self.sidebar_frame, anchor='w', width=30, text='Target Waypoint:\nLatitude: ???\nLongitude: ???\n')
        self.control_waypoint.pack()

        # Buttons
        self.goto = Button(self.sidebar_frame, anchor='w', text='Go to Map Location', command=self.on_toggle_goto)
        self.goto.pack()
        self.stop = Button(self.sidebar_frame, anchor='w', text='Stop ASV', command=self.on_stop).pack()
        self.quit = Button(self.sidebar_frame, anchor='w', text='Quit Gui', command=self.on_quit).pack()

        # Load map image
        pilImg = Image.open(MAP_FILE)
        pilImg = pilImg.resize((MAP_WIDTH,MAP_HEIGHT), Image.ANTIALIAS)
        self.img = ImageTk.PhotoImage(pilImg)
        
        # map
        self.canvas = Canvas(self.map_frame, width=MAP_WIDTH, height=MAP_HEIGHT)
        self.canvas.create_image(0,0, image=self.img, anchor=NW)
        self.canvas.pack()
        self.canvas.bind("<Button 1>", self.on_location_click)

        # origin point
        self.origin_lat = 35.44073027
        self.origin_lon = -118.9001349
        self.origin_x, self.origin_y,_, _ = utm.from_latlon(self.origin_lat, self.origin_lon)
        col, row = gdal.ApplyGeoTransform(self.inv_trans, self.origin_x, self.origin_y)
        x1, y1 = (col - 5), (row - 5)
        x2, y2 = (col + 5), (row + 5)
        self.origin_dot = self.canvas.create_oval(x1, y1, x2, y2, fill='black')


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
            x1, y1 = (event.x - 5), (event.y - 5)
            x2, y2 = (event.x + 5), (event.y + 5)
            self.target_pos = self.canvas.create_oval(x1, y1, x2, y2, fill='red')
        else:
            old_pos = self.canvas.coords(self.target_pos)
            dx = event.x - (old_pos[0] + 5)
            dy = event.y - (old_pos[1] + 5)
            self.canvas.move(self.target_pos, dx, dy)
        self.go_to_location(event.x, event.y)

        # Reset button
        self.location_select_mode = False
        self.goto.configure(text='Go to Map Location')

    # Function to be called for emergency stop
    def on_stop(self):
        #TODO: Stop motors
        print('Motors stopped!')
        command_msg = "!STOP"
        # self.controller.local_xbee.send_data_async(self.controller.boat_xbee, command_msg.encode())

    def on_quit(self):
        print('QUIT!')
        command_msg = "!QUIT"
        # self.controller.local_xbee.send_data_async(self.controller.boat_xbee, command_msg.encode())
        self.on_stop()

    ###########################################################################
    # Updating GUI
    ###########################################################################

    # Called at every iteration of main loop
    def update(self):
        self.update_GPS()
        self.update_ADCP()

        # update the graphics
        self.tk.update()

        return True


    def update_GPS(self):
        lat = self.controller.robot.lat
        lon = self.controller.robot.lon
        heading = self.controller.robot.heading

        self.gps['text'] = 'Latitude: ' + str(lat) + '\nLongitude: ' + str(lon) + '\nHeading: ' + str(heading) + '\n'
        x, y,_, _ = utm.from_latlon(lat, lon)
        col, row = gdal.ApplyGeoTransform(self.inv_trans, x, y)

        # Update ASV location on map
        if self.cur_pos == None:
            x1, y1 = (col - 5), (row - 5)
            x2, y2 = (col + 5), (row + 5)
            self.cur_pos = self.canvas.create_oval(x1, y1, x2, y2, fill='green')
        else:
            old_pos = self.canvas.coords(self.cur_pos)
            dx = col - (old_pos[0] + 5)
            dy = row - (old_pos[1] + 5)
            self.canvas.move(self.cur_pos, dx, dy)

    def update_ADCP(self):
        depth = self.controller.depth
        current = self.controller.v_boat

        self.adcp_depth['text'] = 'Water depth: %f' % (depth)
        self.adcp_current['text'] = "Current speed: %f" % (current)

    ###########################################################################
    # ASV Commands
    ###########################################################################

    #Go to location specified by mouse click (pixel coords -> GPS)
    def go_to_location(self, row, col):
        x, y = gdal.ApplyGeoTransform(self.geo_trans, row, col)
        print('UTM: ', x, y)
        lat, lon = utm.to_latlon(x, y, 11, 'S') #11, S is UTM zone for Kern River
        print('Lat/lon: ', lat, lon)

        self.control_waypoint['text'] = 'Target Waypoint:\nLatitude: '+ str(lat) + '\nLongitude: ' + str(lon) + '\n'

        #TODO: Send command to ASV to move to x, y
        way_point_msg = "!WP, %f, %f" % (x, y)
        # self.controller.local_xbee.send_data_async(self.controller.boat_xbee, way_point_msg.encode())

        return

    #Display ASV path plan 
    # positions: (row,col)
    def show_path(self, positions):
        pass


if __name__ == '__main__':
    my_gui = ASV_graphics(None)
    my_gui.tk.mainloop()
