# Test drive around in RC mode and write the ADCP raw data to a file
from digi.xbee.devices import XBeeDevice
import time
import datetime
import serial

# initialize xbee
node_name = 'boat'
# ser = serial.Serial("/dev/tty.usbserial-DN02Z6QY", 9600)
# ser.flush()
# ser.close()

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
        data = xbee_message.data.decode()
        parsed_data = data.split(',')
        print(data)
    except KeyboardInterrupt:
        end_msg = "STOP".encode()
        local_xbee.send_data_async(boat_xbee,end_msg)

def main():
    CLOSED = False
    # Setting up Xbee communication
    local_xbee = XBeeDevice("/dev/tty.usbserial-DN02Z6QY", 9600)
    local_xbee.open()
    local_xbee.add_data_received_callback(data_received_callback)

    boat_xbee = discover_boat(local_xbee)
    if boat_xbee == None:
        print('device not found!')
        local_xbee.close()
        return
    else:
        print('device found! Sending start messages')
        msg = "START!"
        local_xbee.send_data_async(boat_xbee, msg.encode())

    x = 0
    y = 0
    while True:
        try:
            # print("?")
            x += 10
            y += 10
            new_msg = "!WP, %d, %d" % (x, y)
            origin_msg = "!ORIGIN, 33.119211, 118.229211"
            local_xbee.send_data_async(boat_xbee, origin_msg.encode())
            time.sleep(0.5)
            continue
        except KeyboardInterrupt:
            local_xbee.del_data_received_callback(data_received_callback)
            end_msg = "!STOP".encode()
            local_xbee.send_data_async(boat_xbee,end_msg)
            break


    # Terminating the data aquisition
    local_xbee.close()

if __name__ == "__main__":
    main()

