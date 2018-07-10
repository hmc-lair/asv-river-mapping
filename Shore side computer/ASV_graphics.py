from Tkinter import *
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
		# Load map for GPS conversions
		dataset = gdal.Open(MAP_FILE)
		data = dataset.ReadAsArray()

		self.geo_trans = dataset.GetGeoTransform()
		self.inv_trans = gdal.InvGeoTransform(self.geo_trans)

		# Set up GUI
		self.environment = environment
		self.tk = Tk()

		self.tk.title("ASV Control Interface")

		# sidebar
		self.sidebar = Frame(self.tk, width=300, bg='white', height=500, relief='sunken', borderwidth=2)
		self.sidebar.pack(expand=True, fill='both', side='left', anchor='nw')

		self.gps = Label(self.sidebar, anchor='w', text='Latitude: ???\nLongitude: ???\nHeading: ???\n')
		self.gps.grid(row=0, column=0)
		self.gps.pack()

		# main content area
		self.map_frame = Frame(self.tk)
		self.map_frame.pack(expand=True, fill='both', side='right')

		pilImg = Image.open(MAP_FILE)
		pilImg = pilImg.resize((MAP_WIDTH,MAP_HEIGHT), Image.ANTIALIAS)
		self.img = ImageTk.PhotoImage(pilImg)

		#function to be called when mouse is clicked
		def on_location_click(event):
			print(event.x, event.y)
			self.go_to_location(event.x, event.y)

		self.canvas = Canvas(self.map_frame, width=MAP_WIDTH, height=MAP_HEIGHT)
		self.canvas.create_image(0,0, image=self.img, anchor=NW)
		walls = self.canvas.create_rectangle(150, 100, 400, 300,outline='red')
		self.canvas.grid(row=0,column=0, columnspan = 2)
		self.canvas.pack()
		self.canvas.bind("<Button 1>",on_location_click)

		self.control = Label(self.map_frame, anchor='w', text='Latitude: ???\nLongitude: ???\nHeading: ???')
		self.control.pack(side='left')

		self.deploy = Button(self.map_frame, anchor='w', text='Deploy')
		self.deploy.pack(side='right')

		#######################################################################

		# MENU
		def donothing():
			return
		menubar = Menu(self.tk)
		filemenu = Menu(menubar, tearoff=0)
		filemenu.add_command(label="New", command=donothing)
		filemenu.add_command(label="Open", command=donothing)
		filemenu.add_command(label="Save", command=donothing)
		filemenu.add_command(label="Save as...", command=donothing)
		filemenu.add_command(label="hihihihihiihi", command=donothing)

		filemenu.add_separator()

		filemenu.add_command(label="Exit", command=self.tk.quit)
		menubar.add_cascade(label="File", menu=filemenu)
		editmenu = Menu(menubar, tearoff=0)
		editmenu.add_command(label="Undo", command=donothing)

		editmenu.add_separator()

		editmenu.add_command(label="Cut", command=donothing)
		editmenu.add_command(label="Copy", command=donothing)
		editmenu.add_command(label="Paste", command=donothing)
		editmenu.add_command(label="Delete", command=donothing)
		editmenu.add_command(label="Select All", command=donothing)

		menubar.add_cascade(label="Edit", menu=editmenu)
		helpmenu = Menu(menubar, tearoff=0)
		helpmenu.add_command(label="Help Index", command=donothing)
		helpmenu.add_command(label="About...", command=donothing)
		menubar.add_cascade(label="Help", menu=helpmenu)

		self.tk.config(menu=menubar)

	def greet(self):
		print("Greetings!")

	#Go to location specified by mouse click (pixel coords -> GPS)
	def go_to_location(self, row, col):
		x, y = gdal.ApplyGeoTransform(self.geo_trans, row, col)
		print('UTM: ', x, y)
		lat, lon = utm.to_latlon(x, y, 11, 'S') #11, S is UTM zone for Kern River
		print('Lat/lon: ', lat, lon)
		return


if __name__ == '__main__':
	my_gui = ASV_graphics(None)
	my_gui.tk.mainloop()
