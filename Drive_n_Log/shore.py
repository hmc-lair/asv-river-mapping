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
xbee_network = local_xbee.get_network()
xbee_network.start_discovery_process()

# discovering devices
print('Discovering device')
while xbee_network.is_discovery_running():
    time.sleep(0.5)
boat_xbee = xbee_network.discover_device('boat')

# initialize file
file_name = 'Log/ADCP' + '_' + datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S') + '.txt'
f = open(file_name, 'a+')

def data_received_callback(xbee_message):
    address = xbee_message.remote_device.get_64bit_addr()
    data = xbee_message.data
    print("Received data from %s: %s" % (address, data))


def main():
	local_xbee.add_data_received_callback(data_received_callback)

	if boat_xbee == None:
		print('device not found!')
	else:
		print('device found! Sending start messages')
		start_msg = "START".encode()
		local_xbee.send_data_async(boat_xbee, start_msg)
		count = 0
		while True:
			try:
				continue
			except KeyboardInterrupt:
				break

		end_msg = "STOP".encode()
		local_xbee.send_data_async(boat_xbee,end_msg)
		local_xbee.close()

if __name__ == "__main__":
	main()






