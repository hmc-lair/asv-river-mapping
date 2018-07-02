# Test drive around in RC mode and write the ADCP raw data to a file
from digi.xbee.devices import XBeeDevice
import time
import datetime
import serial

# initialize xbee
node_name = 'boat'
ser = serial.Serial("/dev/tty.usbserial-DN02Z6QY", 9600)
ser.flush()
ser.close()
local_xbee = XBeeDevice("/dev/tty.usbserial-DN02Z6QY", 9600)
local_xbee.open()
CLOSED = False


###############################################################################
# XBEE Setup Functions
###############################################################################

'''
Discover the boat xbee

Input: 
	xbee(XBeeDevice): local XbeeDevice
Output:
	boat_xbee(XBeeDevice): None if there is nothing
'''
def discover_boat(xbee):
	print('Discovering device')
	xbee_network = xbee.get_network()
	xbee_network.start_discovery_process()
	while xbee_network.is_discovery_running():
	    time.sleep(0.5)
	boat_xbee = xbee_network.discover_device('boat')
	return boat_xbee

'''
Call back function whenever a message arrived. Sort out GPS
and ADCP information

Input:
	xbee_message: messages from the other xbee
'''
def data_received_callback(xbee_message):
	try:
		if CLOSED == False:
		    address = xbee_message.remote_device.get_64bit_addr()
		    data = xbee_message.data.decode()
		    parsed_data = data.split(',')

		    if parsed_data[0] == '$GPGGA':
		    	print('Received GPS: ', data)
		    elif parsed_data[0] == '$ADCP':
		    	print('Received ADCP: ', data)
		else:
			end_msg = "STOP".encode()
			local_xbee.send_data_async(boat_xbee,end_msg)
			local_xbee.close()
	except KeyboardInterrupt:
		CLOSED = True
		return


################################################################################


# initialize file
file_name = 'Log/ADCP' + '_' + datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S') + '.txt'
f = open(file_name, 'a+')


def main():
	CLOSED = False
	# Setting up Xbee communication
	local_xbee.add_data_received_callback(data_received_callback)
	boat_xbee = discover_boat(local_xbee)
	if boat_xbee == None:
		print('device not found!')
		return
	else:
		print('device found! Sending start messages')

	# Sending start command to the boat
	start_msg = "START".encode()
	local_xbee.send_data_async(boat_xbee, start_msg)

	while True:
		try:
			continue
		except KeyboardInterrupt:
			CLOSED = True
			end_msg = "STOP".encode()
			local_xbee.send_data_async(boat_xbee,end_msg)
			local_xbee.close()
			break


	# Terminating the data aquisition
	end_msg = "STOP".encode()
	local_xbee.send_data_async(boat_xbee,end_msg)
	local_xbee.close()

if __name__ == "__main__":
	main()






