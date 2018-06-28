import math
from tkinter import *
from PIL import Image, ImageTk
import keyboard
from tkinter import ttk

class Central_graphics:
    
    def __init__(self, controller):
        self.central = controller
        self.robot_images = []
        self.environment_width = 122 * 2
        self.environment_height = 122 * 2

        self.tk = Tk()
        self.tk.title("ASV Remote Controller")
        self.gui_stopped = False

        ######### Top Left Frame ############
        self.top_left_frame = Frame(self.tk, highlightbackground="black", highlightcolor="black", highlightthickness=1)

        # My computer canvas size is w=600, h=500
        self.scale = 2.05
        self.canvas = Canvas(self.top_left_frame, width=self.environment_width*self.scale*1.2, height=self.scale* self.environment_height)
        #self.canvas = Canvas(self.top_left_frame, width=600, height=500)
        self.canvas.pack()


        # open image
        self.bckgnd_image = ImageTk.PhotoImage(file="test_map.tiff")
        print(self.bckgnd_image.width())
        
        # draw image
        self.canvas.create_image(int(self.canvas.cget('width'))/2, int(self.canvas.cget('height'))/2, image=self.bckgnd_image, anchor=CENTER)


        ######### Middle Top Right Frame ############
        self.middle_top_right_frame = Frame(self.tk)
        self.state_label_frame = Frame(self.middle_top_right_frame)
        self.state_est_frame = Frame(self.middle_top_right_frame)
        self.state_des_frame = Frame(self.middle_top_right_frame)

         # add range sensor measurements
        self.x_label = Label(self.state_label_frame, text="X (m): ", justify=RIGHT).pack()
        self.y_label = Label(self.state_label_frame, text="Y (m): ", justify=RIGHT).pack()
        self.theta_label = Label(self.state_label_frame, text="Heading (radians): ", justify=RIGHT).pack()

        self.estimate_label = Label(self.state_est_frame, text="Estimate").pack(padx=10)
        self.x_est = StringVar()
        self.y_est = StringVar()
        self.theta_est = StringVar()
        self.x_label = Label(self.state_est_frame, textvariable = self.x_est).pack()
        self.y_label = Label(self.state_est_frame, textvariable = self.y_est).pack()
        self.theta_label = Label(self.state_est_frame, textvariable = self.theta_est).pack()
       
        self.desire_label = Label(self.state_des_frame, text="Desired").pack()
        # add text entry for desired X
        self.x_des_text = StringVar()
        self.x_des_entry = Entry(self.state_des_frame, justify = RIGHT, textvariable=self.x_des_text)
        self.x_des_entry.insert(10,"0.0")
        self.x_des_entry.pack()
        
        # add text entry for desired Y
        self.y_des_text = StringVar()
        self.y_des_entry = Entry(self.state_des_frame, justify = RIGHT, textvariable=self.y_des_text)
        self.y_des_entry.insert(10,"0.0")
        self.y_des_entry.pack()
        
        # add text entry for desired Theta
        self.theta_des_text = StringVar()
        self.theta_des_entry = Entry(self.state_des_frame, justify = RIGHT, textvariable=self.theta_des_text)
        self.theta_des_entry.insert(10,"0.0")
        self.theta_des_entry.pack()
        
        # add autonomous mode button
        self.set_desired_button = Button(self.state_des_frame, text="Enter", anchor="s", wraplength=100, command=self.autonomous_mode).pack()

        self.state_label_frame.pack(side=LEFT, pady=[21,1])
        self.state_est_frame.pack(side=LEFT)
        self.state_des_frame.pack(side=LEFT, pady=[18,1])

         ######### Top Right Frame ############
        self.top_right_frame = Frame(self.tk)
        self.robot_name = Label(self.top_right_frame, text="Robot: ", font=('Times', '12', 'bold')).pack(side=LEFT, pady =[50, 0])
        self.robot_select_var = StringVar()
        self.robot_select = ttk.Combobox(self.top_right_frame, textvariable=self.robot_select_var, values=[str(i) for i in range(0, self.central.num_robots)])
        self.robot_select.bind('<<ComboboxSelected>>', self.update_selected_robot)
        self.robot_select.pack(side=LEFT,  pady =[50, 0])
        self.robot_select.set(0)
        self.mode_var = StringVar()
        self.mode_label_var = Label(self.top_right_frame, textvariable=  self.mode_var).pack(side=LEFT,  pady =[50, 0])

        ######### Middle Bottom Right Frame ############
        self.middle_bot_right_frame = Frame(self.tk)
        self.sensor_display_frame = Frame(self.middle_bot_right_frame)
        self.sensor_entry_frame = Frame(self.middle_bot_right_frame)
        self.sensor_label = Label(self.sensor_display_frame, text = "Sensor Values", font=('Times', '10', 'bold underline'), justify=LEFT).pack(pady =[40, 0])

        # add imu sensor measurements
        self.imu_sensor_var = StringVar()
        self.imu_sensor_label = Label(self.sensor_display_frame, textvariable = self.imu_sensor_var, justify=LEFT).pack()

        # add gps sensor measurements
        self.gps_sensor_var_0 = StringVar()
        self.gps_sensor_var_1 = StringVar()
        self.gps_sensor_var_2 = StringVar()
        
        self.gps_sensor_label_0 = Label(self.sensor_display_frame, textvariable = self.gps_sensor_var_0, justify=LEFT).pack()
        self.gps_sensor_label_1 = Label(self.sensor_display_frame, textvariable = self.gps_sensor_var_1, justify=LEFT).pack()
        self.gps_sensor_label_2 = Label(self.sensor_display_frame, textvariable = self.gps_sensor_var_2, justify=LEFT).pack()

        self.offset_entry = Entry(self.sensor_entry_frame, justify = RIGHT)
        self.offset_entry.insert(10,"0.0")
        self.offset_entry.pack(pady=[80,1])
        self.set_offset_button = Button(self.sensor_entry_frame, text="Set Heading Offset (Deg)", anchor="s", wraplength=200, command=self.set_offset).pack()


        # add set origin point button
        self.lat_origin_entry = Entry(self.sensor_entry_frame, justify = RIGHT)
        self.lat_origin_entry.insert(10,"0.0")
        self.lat_origin_entry.pack()

        self.lon_origin_entry = Entry(self.sensor_entry_frame, justify = RIGHT)
        self.lon_origin_entry.insert(10,"0.0")
        self.lon_origin_entry.pack()
        self.set_origin_button = Button(self.sensor_entry_frame, text="Set Origin", anchor="s", wraplength=100, command=self.set_origin).pack()

        self.sensor_display_frame.pack(side=LEFT)
        self.sensor_entry_frame.pack(side=LEFT)

    
        
        ######### Bottom Right Frame ############
        self.bottom_right_frame = Frame(self.tk)
        self.stop_button = Button(self.bottom_right_frame,text="Stop Robot",font=('Times', '10', 'bold'), anchor="s", wraplength=100, command=self.stop, width=30).pack(side=BOTTOM, padx=5)

        # add motor control slider
        self.forward_control = Scale(self.bottom_right_frame, from_=-100, to=100, length  = 400,label="Forward Control",tickinterval=50, orient=HORIZONTAL)
        self.forward_control.pack(pady=[40,0])
        
        # add rotation control slider
        self.rotate_control = Scale(self.bottom_right_frame, from_=-100, to=100, length  = 400,label="Rotate Control",tickinterval=50, orient=HORIZONTAL)
        self.rotate_control.pack()

        self.last_rotate_control = 0
        self.last_forward_control = 0
        self.R = 0
        self.L = 0

        ######### Bottom Left Frame ############
        self.bottom_left_frame = Frame(self.tk)
        # add run experiment button
        self.experiment_button = Button(self.bottom_left_frame,text="Run Experiment",font=('Times', '10', 'bold'), anchor="s", wraplength=200, command=self.stop, width=30, height=2).pack(side=LEFT, padx=5, pady= 10)

        # add stop all button
        self.stop_all_button = Button(self.bottom_left_frame, text="Stop All", font=('Times', '10', 'bold'), anchor="s", wraplength=100, command=self.stop_all,  width=30, height=2).pack(side=LEFT, padx=2, pady= 10)
  
        # add quit button
        self.quit_button = Button(self.bottom_left_frame, text="Quit", anchor="s",font=('Times', '10', 'bold'), wraplength=100, command=self.quit,  width=30, height=2).pack(side = RIGHT, padx=2, pady= 10)

        # Arrange all the frames
        self.top_left_frame.grid(column=0, row=0, rowspan=4)
        self.top_right_frame.grid(column=1, row=0)
        self.bottom_left_frame.grid(column=0, row=4)
        self.middle_top_right_frame.grid(column=1, row=1)
        self.middle_bot_right_frame.grid(column=1, row=2)
        self.bottom_right_frame.grid(column=1, row=3)

        
        

        '''
        # add autonomous mode button
        self.auton_mode_button = Button(self.bottom_frame, text="Autonomous", anchor="s", wraplength=100, command=self.autonomous_mode).pack()
        '''
        

        for i in range(0, self.central.num_robots):
            self.initial_draw_robot(i)

    def scale_points(self, points, scale):
        scaled_points = []
        for i in range(len(points)-1):
            
            if i % 2 == 0:
                # for x values, just multiply times scale factor to go from meters to pixels
                scaled_points.append(self.environment_width/2*scale + points[i]*scale)   
                
                # only flip y for x,y points, not for circle radii
                scaled_points.append(self.environment_height/2*scale - points[i+1]*scale)   
                    
        return scaled_points
    
    
    def reverse_scale_points(self, points, scale):
        reverse_scaled_points = []
        for i in range(len(points)-1):
            
            if i % 2 == 0:
                # for x values, just multiply times scale factor to go from meters to pixels
                reverse_scaled_points.append(-self.environment_width/2 + points[i]/scale)   
                
                # only flip y for x,y points, not for circle radii
                reverse_scaled_points.append(self.environment_height/2 - points[i+1]/scale)   
                    
        return reverse_scaled_points
    
    def initial_draw_robot(self, robot):
            
        # open image
        self.robot_image = Image.open("ASV_Picv2.gif").convert('RGBA') 
        
        # gif draw
        #robot.tkimage = PhotoImage(file = "E160_robot_image.gif")
        rotated_image = self.robot_image.rotate(45)
        rotated_image = rotated_image.resize((50, 50), Image.ANTIALIAS)
        photoimage = ImageTk.PhotoImage(self.robot_image.rotate(40))
        self.central.tkimage.append(photoimage)
        self.central.image.append(self.canvas.create_image(self.central.robots[robot]["Est."][0], self.central.robots[robot]["Est."][1], image=photoimage))

        
    def draw_robot(self, robot):
        # gif update
        self.central.tkimage[robot["ID"]] = ImageTk.PhotoImage(self.robot_image.rotate(180/3.14*robot["Est."][2]).resize((50, 50), Image.ANTIALIAS))
        self.central.image[robot["ID"]] = self.canvas.create_image(robot["Est."][0], robot["Est."][1], image=self.central.tkimage[robot["ID"]])
        #self.central.image = self.canvas.create_image(robot["Est."][0], robot["Est."][1], image=self.central.tkimage)
        robot_points = self.scale_points([robot["Est."][0]+20, robot["Est."][1]+10], self.scale)
        self.canvas.coords(self.central.image[robot["ID"]], *robot_points)

    def get_inputs(self):

        if(keyboard.is_pressed('space')):
            self.rotate_control.set(0) 
            self.forward_control.set(0)
        elif(keyboard.is_pressed('w')):
            temp = self.forward_control.get() + 5
            self.rotate_control.set(0) 
            self.forward_control.set(temp)
        elif (keyboard.is_pressed('s')):
            temp = self.forward_control.get() - 5
            self.rotate_control.set(0) 
            self.forward_control.set(temp)
        elif (keyboard.is_pressed('a')):
            temp = self.rotate_control.get() - 5
            self.forward_control.set(0) 
            self.rotate_control.set(temp)
        elif (keyboard.is_pressed('d')):
            temp = self.rotate_control.get() + 5
            self.forward_control.set(0) 
            self.rotate_control.set(temp)

        selected_robot = self.robot_select.current()

        if abs(self.forward_control.get()) > 0 or abs(self.rotate_control.get()) > 0:


            self.central.robots[selected_robot]["Mode"] = "MANUAL MODE"

        # check to see if forward slider has changed
        if abs(self.forward_control.get()-self.last_forward_control) > 0:
            self.rotate_control.set(0)       
            self.last_forward_control = self.forward_control.get()
            self.last_rotate_control = 0         
            
            # extract what the R and L motor signals should be
            self.central.robots[selected_robot]["Manual Ctrl"][0] = self.forward_control.get()
            self.central.robots[selected_robot]["Manual Ctrl"][1] = self.forward_control.get()
  
        # check to see if rotate slider has changed
        elif abs(self.rotate_control.get()-self.last_rotate_control) > 0:
            self.forward_control.set(0)       
            self.last_rotate_control = self.rotate_control.get()
            self.last_forward_control = 0         
        
            # extract what the R and L motor signals should be
            self.central.robots[selected_robot]["Manual Ctrl"][0] = -self.rotate_control.get()
            self.central.robots[selected_robot]["Manual Ctrl"][1] = self.rotate_control.get()

            return

    def set_origin(self):
        for i in range(0, self.central.num_robots):
            lat_origin = float(self.lat_origin_entry.get())
            lon_origin = float(self.lon_origin_entry.get())

            self.central.robots[i]["Origin"] = [lat_origin, lon_origin]

            self.central.update_calibration(i)

    def set_offset(self):
        selected_robot = self.robot_select.current()

        offset = float(self.offset_entry.get())
        self.central.robots[selected_robot]["Offset"] = offset

        self.central.update_calibration(selected_robot)

    def autonomous_mode(self):

        # update sliders on gui
        self.forward_control.set(0)
        self.rotate_control.set(0)
        self.last_forward_control = 0
        self.last_rotate_control = 0

        selected_robot = self.robot_select.current()
        
        x_des = float(self.x_des_entry.get())
        y_des = float(self.y_des_entry.get())
        theta_des = float(self.theta_des_entry.get())

        self.central.robots[selected_robot]["Des."] = [x_des, y_des, theta_des]
        self.central.robots[selected_robot]["Mode"] = "AUTONOMOUS MODE"
        self.central.robots[selected_robot]["Manual Ctrl"] = [0, 0]
        
    def stop(self):
        # update sliders on gui
        self.forward_control.set(0)
        self.rotate_control.set(0)       
        self.last_forward_control = 0
        self.last_rotate_control = 0  

        selected_robot = self.robot_select.current()

        self.central.robots[selected_robot]["Mode"] = "MANUAL MODE"
        self.central.robots[selected_robot]["Manual Ctrl"] = [0, 0]
        

    # Have it stop all the robots
    def stop_all(self):
        
        # update sliders on gui
        self.forward_control.set(0)
        self.rotate_control.set(0)       
        self.last_forward_control = 0
        self.last_rotate_control = 0  

        for i in range(0, self.central.num_robots):
            self.central.robots[i]["Mode"] = "MANUAL MODE"
            self.central.robots[i]["Manual Ctrl"] = [0, 0]

        
    def quit(self):
        self.stop_all()
        self.forward_control.set(0)
        self.rotate_control.set(0)  
        self.gui_stopped = True
        
    def update_labels(self):
        selected_robot = self.robot_select.current()
        
        self.imu_sensor_var.set("Heading (Degrees):  " + str(self.central.robots[selected_robot]["Sensor"][0]))
                
        self.gps_sensor_var_0.set("Number of Satellites:  " + str(self.central.robots[selected_robot]["Sensor"][1]))
        self.gps_sensor_var_1.set("Latitude:  " + str(self.central.robots[selected_robot]["Sensor"][2]))
        self.gps_sensor_var_2.set("Longitude:  " + str(self.central.robots[selected_robot]["Sensor"][3]))
        
        
        self.x_est.set(str(self.central.robots[selected_robot]["Est."][0]))
        self.y_est.set(str(self.central.robots[selected_robot]["Est."][1]))
        self.theta_est.set(str(self.central.robots[selected_robot]["Est."][2]))

        self.mode_var.set(str(self.central.robots[selected_robot]["Mode"]))

        
    # called at every iteration of main loop
    def update(self):
        
        # update gui labels
        self.update_labels()
        
        
        for i in range(0, self.central.num_robots):
            self.draw_robot(self.central.robots[i])
        
        # update the graphics
        self.tk.update()

        # check for gui buttons
        self.get_inputs()
        
        
        # check for quit
        if self.gui_stopped:
            self.central.quit()
            return False
        else:
            return True


    def update_selected_robot(self, input):

        # Stop all the robots in manual to prevent collisions
        for i in range(0, self.central.num_robots):
            if self.central.robots[i]["Mode"] == "MANUAL MODE":
                self.central.robots[i]["Manual Ctrl"] = [0, 0]


        selected_robot  = self.robot_select.current()
        print(str(self.central.robots[selected_robot]["Des."][0]))
        self.x_des_entry.delete(0, 'end')
        self.x_des_entry.insert(10, str(self.central.robots[selected_robot]["Des."][0]))
        self.y_des_entry.delete(0, 'end')
        self.y_des_entry.insert(10, str(self.central.robots[selected_robot]["Des."][1]))
        self.theta_des_entry.delete(0, 'end')
        self.theta_des_entry.insert(10, str(self.central.robots[selected_robot]["Des."][2]))
        #self.x_des_entry.set(str(self.central.robots[selected_robot]["Des."][0]))
        #self.x_des_text.set(str(self.central.robots[selected_robot]["Des."][0]))
        #self.y_des_text.set(str(self.central.robots[selected_robot]["Des."][1]))
        #self.theta_des_text.set(str(self.central.robots[selected_robot]["Des."][2]))



