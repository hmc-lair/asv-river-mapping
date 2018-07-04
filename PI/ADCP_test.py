from digi.xbee.devices import XBeeDevice
import serial
import time
import threading
import queue
import struct
import datetime

GPS_PORT = '/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FT8VW9AR-if00-port0' #Pi ADCP
xbee_port = '/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DN02Z3LX-if00-port0' #Pi xbee
COM_PORT = '/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FT8VWDWP-if00-port0'
BAUD_RATE = "115200"
ADCP_ser = serial.Serial(COM_PORT, BAUD_RATE, stopbits=serial.STOPBITS_ONE)
GPS_ser = serial.Serial(GPS_PORT, bytesize = 8)
boat_xbee = XBeeDevice(xbee_port, 9600)

adcp_filename = 'Log/ADCP_' + datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S') + '.bin'
gps_filename = 'Log/GPS_' + datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S') + '.txt'

f = open(adcp_filename, 'wb')
gps_f = open(gps_filename,'w')

###############################################################################
# GPS Setup Function
###############################################################################
def GPS_setup(gps_ser):
	gps_ser.baudrate = 19200
	gps_ser.parity = serial.PARITY_NONE
	gps_ser.stop_bits = serial.STOPBITS_ONE
	time.sleep(0.1)
	gps_ser.flushInput()
	return

def data_received_callback(xbee_message):
	data = xbee_message.data
	if data == "STOP":
            print("?")
            #raise Exception("Terminating")
	return data

def right_GPS_msg(GPS_msg):
	''' Decide whether this GPS message is to be sent or not.
		Condition to be sent:
			GPGGA msg with signal present'''
	GPS_data = GPS_msg.split(',')
	if GPS_data[0] == '$GPGGA':
		if GPS_data[6] == '0':
			# if there's no signal then we don't care
			return False
		else:
			return True
	else:
		return False

'''
Function that wraps GPS reading sequence for a thread
Execute in GPS thread to prevent blocking the program
'''

def GPS_read(dest_xbee):
    while True:
        try:
            if GPS_ser.in_waiting != 0:
                data_str = GPS_ser.readline()
                boat_xbee.send_data_async(dest_xbee, data_str)
                gps_f.write(data_str.decode())
                message = boat_xbee.read_data()
                if message != None:
                    break
                
        except KeyboardInterrupt:
            GPS_ser.close()
            #print(data_str.decode())
    return

###############################################################################
# XBEE Setup Function
###############################################################################
'''
Discover the central xbee

Inputs:
	xbee - XBeeDevice
Outputs:
	central_xbee - XBeeDevice (None if no discovery)
'''
def discover_xbee(xbee):
	xbee.open()
	xbee_network = xbee.get_network()
	xbee_network.start_discovery_process()
	while xbee_network.is_discovery_running():
		time.sleep(0.5)
	print(xbee_network.get_devices())
	central_xbee = xbee_network.discover_device('central')

	if central_xbee == None:
		print('No device found... ')
	else:
		print('device found! waiting..')
		xbee.send_data(central_xbee, b'Device found! Waiting for starting cue...')
		msg = xbee.read_data(100)
		print(msg.data)
	return central_xbee


###############################################################################
# ADCP Setup/Helper Functions
###############################################################################
'''
Setup ADCP connection
'''
def setup():
	ADCP_ser.flush()
	print("Starting ADCP communication")
	ADCP_ser.write(b'+++')
	time.sleep(3)
	s = read_response(verbose=True)
	print('Startup message: ', s)
	return ADCP_ser

'''
Sends specified command and outputs response

Inputs:
	ser - Serial port
	command - in bytes
'''
def send(command):
	ADCP_ser.write(command + b'\r\n')
	s = read_response(ser, verbose=True)
	return s

'''
Reads entire output message
'''
def read_response(verbose=False):
	response = b''
	cur_line = b''
	while True:
		s = ADCP_ser.read()
		response += s
		cur_line += s
		if verbose and s == b'\n':
			print(cur_line)
			cur_line = b''

		if b'>' in response: #stop character
			return response
	return

###############################################################################
# ADCP Control Commands
###############################################################################
def start_ping():
	print("Requesting Pings")
	ADCP_ser.write(b'CS\r\n')
	ADCP_ser.read(10) #response includes command	

def stop_ping():
	print('Stopping Pings')
	ADCP_ser.write(b'CSTOP\r\n')
	
def read_ensemble(verbose=False):
	header = ADCP_ser.read(2)
	if header != b'\x7f\x7f':
		print('ERROR no header: ', header)
  
	num_bytes = ADCP_ser.read(2)
	bytes_to_checksum = int.from_bytes(num_bytes, byteorder='little')-4
	if verbose:
		print('Num: ', bytes_to_checksum)
	
	data = ADCP_ser.read(bytes_to_checksum)
	if verbose:
		print('Data: ', data)

	#use checksum to verify no errors
	checksum = ADCP_ser.read(2)
	checksum_int = int.from_bytes(checksum, byteorder='little') 
	datasum_int = sum(b'\x7f\x7f' + num_bytes + data) % 2**16

	if checksum_int != datasum_int:
		print('ERROR: ', checksum_int, datasum_int)

	#read data to file
	all_data = b'\x7f\x7f' + num_bytes + data + checksum
	f.write(all_data)

	return all_data

'''
Input:
    Ensemble: byte str ings of ensemble data
Output:
    Array of values from the ADCP package
'''
def extract_data(all_data):
        num_types = all_data[5]
        
        # OFFSETS
        # 1. Fix Leader
        # 2. Variable Leader
        # 3. Velocity Profile
        # 6. Bottom Track
        # 9. Vertical Beam Range
        # 10-13. GPS
        offsets = []
        for i in range(num_types):
            offset = all_data[6+2*i:8+2*i]
            offset_int = int.from_bytes(offset, byteorder='little')
            offsets.append(offset_int)
        # print('Offsets: ', offsets)
        # print('Data IDs: ', [all_data[x:x+2] for x in offsets])

        # FIXED LEADER
        fixed_offset = offsets[0]

        num_beams = all_data[fixed_offset+8]
        num_cells = all_data[fixed_offset+9]
        pings_per_ensemble = int.from_bytes(all_data[fixed_offset+10: fixed_offset+12], byteorder='little')
        depth_cell_length = int.from_bytes(all_data[fixed_offset+12: fixed_offset+14], byteorder='little')

        # VARIABLE LEADER
        variable_offset = offsets[1]

        transducer_depth = int.from_bytes(all_data[variable_offset+16: variable_offset+18], byteorder='little')*0.1 #1 dm
        heading = int.from_bytes(all_data[variable_offset+18: variable_offset+20], byteorder='little')*0.01 #0 to 359.99
        pitch = int.from_bytes(all_data[variable_offset+20: variable_offset+22], byteorder='little', signed=True)*0.01 #-20 to 20
        roll = int.from_bytes(all_data[variable_offset+22: variable_offset+24], byteorder='little', signed=True)*0.01 #-20 to 20
        salinity = int.from_bytes(all_data[variable_offset+24: variable_offset+26], byteorder='little') #0 to 40 part per thousand
        temperature = int.from_bytes(all_data[variable_offset+26: variable_offset+28], byteorder='little')*0.01 #-5 to 40 degrees

        # VELOCITY PROFILE
        velocity_profile_offset = offsets[2]
        relative_velocities = []

        for i in range(num_cells):
            start_offset = velocity_profile_offset + 2 + 2*i
            # Average over beams
            vel = 0
            for j in range(num_beams):
                vel += int.from_bytes(all_data[start_offset + 2*j: start_offset + 2 + 2*j], byteorder='little', signed=True)
            vel = vel/float(num_beams)
            relative_velocities.append(vel)

        # BOTTOM TRACK (abbr. bt) (see page 154)

            # Coordinate system for velocity:
                # 1. Earth Axis (default): East, North, Up (right hand orthogonal) 
                    # need to set heading alignment (EA), heading bias (EB) correctly
                    # make sure heading sensors are active (EZ)
                # 2. Radial Beam Coordinates: "raw beam measurements" (not orthogonal)
                # 3. Instrument coordinates: X, Y, UP, X is directon of beam 2, Y is beam 3. Compass
                    # measures the offset of Y from magnetic north
                # 4. Ship coordinates: starboard, forward, mast (pitch, roll, yaw)

        bt_offset = offsets[5]
        bt_pings_per_ensemble = int.from_bytes(all_data[bt_offset+2:bt_offset+4], byteorder='little')
        bt_ranges = [] # ranges measurement for each beam (bt = 0 is bad data) # cm
        bt_velocities = [] # there are one more velocity data.. though not sure what it's for?
        beam_percent_good = []
        max_tracking_depth = int.from_bytes(all_data[bt_offset+70:bt_offset+72], byteorder = 'little')

        for i in range(4):
            bt_ranges.append(int.from_bytes(all_data[bt_offset+16+i*2:bt_offset+18+i*2], byteorder = 'little'))
            bt_velocities.append(int.from_bytes(all_data[bt_offset+24+i*2:bt_offset+26+i*2], byteorder = 'little'))
            beam_percent_good.append(all_data[bt_offset+40+i])
        
        # VERTICAL BEAM RANGE
        vb_offset = offsets[8]
        vb_range = int.from_bytes(all_data[vb_offset+4:vb_offset+8], byteorder = 'little') # in millimeter

        # GPS Data
        GPS_offsets = offsets[9:13]
        msg_types = []
        msg_sizes = []
        delta_times_bytes = []
        delta_times_double = [] # difference between GPS message and ensemble
        GPS_msg = []

        for g_offset in GPS_offsets:
            msg_size = int.from_bytes(all_data[g_offset+4:g_offset+6], byteorder = 'little')
            msg_types.append(int.from_bytes(all_data[g_offset+2:g_offset+4], byteorder = 'little'))
            msg_sizes.append(msg_size)
            delta_times_bytes.append(all_data[g_offset+6:g_offset+14])
            GPS_msg.append(all_data[g_offset+15: g_offset+15+msg_size])

        delta_times_double = [struct.unpack('d', b)[0] for b in delta_times_bytes] # convert to double
        essential = [heading, roll, pitch]
        essential = essential + bt_ranges
        essential = essential + bt_velocities
        print(essential)
        return essential

def ADCP_read():
    count = 0
    while True:
        #print('reading ensemble')
        #time.sleep(10)
        ensemble = read_response(verbose = False)
        count = count + 1
        print('ensemble #:', count)
        print('emsemble wait: ', ADCP_ser.in_waiting)
        #print(ensemble)
        
###############################################################################

def main():
	# Setting up
	setup()
	GPS_setup(GPS_ser)
	central_xbee = discover_xbee(boat_xbee)
	boat_xbee.set_sync_ops_timeout(10)
	boat_xbee.add_data_received_callback(data_received_callback)

	if central_xbee == None:
		return
	
	start_ping()
	#thread1 = threading.Thread(target = ADCP_read).start()
	GPS_thread = threading.Thread(target = GPS_read, args=(central_xbee,),).start()
	#GPS_ser.flushInput()
	ensemble_count = 0
	while True:
            try:
                ensemble = read_ensemble(verbose=False)
                ensemble_count = ensemble_count + 1
                data = extract_data(ensemble)
                data = '$ADCP,' + str(data)[1:-1]
                print("ensemble_count", ensemble_count)
                # Sending essential ensemble message
                boat_xbee.send_data_async(central_xbee, data.encode())
                message = boat_xbee.read_data()
                if message != None:
                    break
            except KeyboardInterrupt:
                f.close()
                gps_f.close()
                break

	stop_ping()
	ADCP_ser.write(b'===')
	ADCP_ser.close()
	#boat_xbee.close()
	GPS_ser.close()
	f.close()
	gps_f.close()

if __name__=='__main__':
	main()
