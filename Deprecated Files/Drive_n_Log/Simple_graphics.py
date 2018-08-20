
from tkinter import *
import datetime
from PIL import Image, ImageTk


class Simple_graphics():

	def __init__(self, controller):

		###### Setting up tkinter #####

		self.tk = Tk()
		self.tk.title('Simple ADCP Monitor')
		self.gui_stopped = False

		self.GPS_Frame = Frame(self.tk, highlightbackground = 'black', highlightcolor="black", highlightthickness=1)
		self.ensemble_Frame = Frame(self.tk, highlightbackground = 'black', highlightcolor='black', highlightthickness = 1)
		self.timing_frame = Frame(self.tk, highlightbackground = 'black', highlightcolor='black', highlightthickness = 1)

		# Time Labels
		self.gps_timer = StringVar()
		self.adcp_timer = StringVar()
		self.gps_time_label = Label(self.timing_frame, textvariable = self.gps_timer).pack()
		self.adcp_time_label = Label(self.timing_frame, textvariable = self.adcp_timer).pack()


		# GPS labels
		self.hem_lat = StringVar()
		self.hem_long = StringVar()
		self.lat_label = Label(self.GPS_Frame, textvariable = self.hem_lat, justify=RIGHT).pack()
		self.long_label = Label(self.GPS_Frame, textvariable = self.hem_long, justify=RIGHT).pack()

		# Ensmble Labels
		self.depth = StringVar()
		self.velocity = StringVar()
		self.roll = StringVar()
		self.pitch = StringVar()
		self.yaw = StringVar()
		self.adcp_lat = StringVar()
		self.adcp_long = StringVar()
		self.depth_label = Label(self.ensemble_Frame, textvariable = self.depth, justify=RIGHT).pack()
		self.velocity_label = Label(self.ensemble_Frame, textvariable = self.velocity).pack()
		self.roll_label = Label(self.ensemble_Frame, textvariable = self.roll).pack()
		self.pitch_label = Label(self.ensemble_Frame, textvariable = self.pitch).pack()
		self.yaw_label = Label(self.ensemble_Frame, textvariable = self.yaw).pack()
		self.adcp_lat_label = Label(self.ensemble_Frame, textvariable = self.adcp_lat).pack()
		self.adcp_long_label = Label(self.ensemble_Frame, textvariable = self.adcp_long).pack()

		self.GPS_Frame.grid(column=0, row=0)
		self.ensemble_Frame.grid(column=1, row=0)
		self.timing_frame.grid(column=2, row = 0)

		##### controller #####
		self.controller = controller

	def update_labels(self):
		self.hem_lat.set( 'Hemisphere Latitude: ' + self.controller.hem_lat)
		self.hem_long.set('Hemisphere Longtidue: ' + self.controller.hem_long)
		self.velocity.set('Water velocity: ' + self.controller.velocity)
		self.depth.set('Water depth: ' + self.controller.depth)
		self.roll.set('Roll: ' + self.controller.roll)
		self.pitch.set('Pitch: ' + self.controller.pitch)
		self.yaw.set('Yaw: ' + self.controller.yaw)
		self.adcp_lat.set('ADCP_lat: ' + self.controller.adcp_lat)
		self.adcp_long.set('ADCP_long: ' + self.controller.adcp_long)

		self.gps_timer.set("Time since last gps measurement: " + str(datetime.datetime.now() - self.controller.last_gps_received))
		self.adcp_timer.set("Time since last adcp measurement: " + str(datetime.datetime.now() - self.controller.last_adcp_received))


	def update(self):		
		# update gui labels
		self.update_labels()
		
		# update the graphics
		self.tk.update()
		
		# check for quit
		if self.gui_stopped:
			return False
		else:
			return True