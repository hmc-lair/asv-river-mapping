from tkinter import *
from PIL import Image, ImageTk
import gdal
import utm

MAP_WIDTH = 1000
MAP_HEIGHT = 800
IMAGE_WIDTH = 1000
IMAGE_HEIGHT = 800
MAP_FILE = '../Maps/river_section.tif'

class ASV_graphics:
    def __init__(self, environment):
        # Load map for display/GPS conversions
        dataset = gdal.Open(MAP_FILE)
        data = dataset.ReadAsArray()
        self.geo_trans = dataset.GetGeoTransform()
        self.inv_trans = gdal.InvGeoTransform(self.geo_trans)

        # Set up GUI
        self.environment = environment
        self.tk = Tk()
        self.tk.title("ASV Control Interface")

        # Frames: Sidebar + Map Area
        self.sidebar_frame = Frame(self.tk, width=300, bg='white', height=500, relief='sunken', borderwidth=2)
        self.map_frame = Frame(self.tk)
        self.sidebar_frame.pack(expand=True, fill='both', side='left', anchor='nw')
        self.map_frame.pack(expand=True, fill='both', side='right')

        # GPS Coordinates
        self.gps = Label(self.sidebar_frame, anchor='w', width=30, text='Latitude: ???\nLongitude: ???\nHeading: ???\n')
        #self.gps.grid(row=0, column=0)
        self.gps.pack()

        # Position variables
        self.cur_pos = None
        self.target_pos = None
        self.location_select_mode = False

        # Callbacks
        # Function to be called when "Go to location" clicked
        def on_toggle_goto():
            if self.location_select_mode:
                self.location_select_mode = False
                self.goto.configure(text='Go to Map Location')
            else:
                self.location_select_mode = True
                self.goto.configure(text='Select Map Location...')
            print('Mode: ', self.location_select_mode)

        # Function to be called when map location clicked
        def on_location_click(event):
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
        def on_stop():
            #TODO: Stop motors
            print('Motors stopped!')
            pass

        # Buttons
        # Go to map location
        self.goto = Button(self.sidebar_frame, anchor='w', text='Go to Map Location', command=on_toggle_goto)
        #self.goto.grid(row=1, column=0)
        self.goto.pack()

        # Emergency stop
        self.stop = Button(self.sidebar_frame, anchor='w', text='STOP ASV', command=on_stop)
        #self.stop.grid(row=2, column=0)
        self.stop.pack()

        # Load map image
        pilImg = Image.open(MAP_FILE)
        pilImg = pilImg.resize((MAP_WIDTH,MAP_HEIGHT), Image.ANTIALIAS)
        self.img = ImageTk.PhotoImage(pilImg)
        
        # map
        self.canvas = Canvas(self.map_frame, width=MAP_WIDTH, height=MAP_HEIGHT)
        self.canvas.create_image(0,0, image=self.img, anchor=NW)
        #walls = self.canvas.create_rectangle(150, 100, 400, 300,outline='red')
        #self.canvas.grid(row=0,column=0, columnspan = 2)
        self.canvas.pack()
        self.canvas.bind("<Button 1>", on_location_click)

    # called at every iteration of main loop
    def update(self):
        # update the graphics
        self.tk.update()
        return True

    #Go to location specified by mouse click (pixel coords -> GPS)
    def go_to_location(self, row, col):
        x, y = gdal.ApplyGeoTransform(self.geo_trans, row, col)
        print('UTM: ', x, y)
        lat, lon = utm.to_latlon(x, y, 11, 'S') #11, S is UTM zone for Kern River
        print('Lat/lon: ', lat, lon)

        #TODO: Send command to ASV to move to x, y
        return

    #Update readings
    def update_GPS(self, lat, lon, heading):
        self.gps.configure(text='Latitude: ' + str(lat) + 
            '\nLongitude: ' + str(lon) + '\nHeading: ' + str(heading) + '\n')
        x, y,_, _ = utm.from_latlon(lat, lon)
        col, row = gdal.ApplyGeoTransform(self.inv_trans, x, y)
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
        pass

    #Display ASV path plan 
    # positions: (row,col)
    def show_path(self, positions):
        pass


if __name__ == '__main__':
    my_gui = ASV_graphics(None)
    my_gui.tk.mainloop()
