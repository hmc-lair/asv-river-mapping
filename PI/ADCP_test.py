from digi.xbee.devices import XBeeDevice
import serial
import time

COM_PORT = "/dev/ttyUSB0" #Pi ADCP
xbee_port = '/dev/ttyUSB1' #Pi xbee
# COM_PORT = "/dev/tty.usbserial-FT8VW9AR" # for John's macbook
BAUD_RATE = "115200"
ser = serial.Serial(COM_PORT, BAUD_RATE, stopbits=serial.STOPBITS_ONE)
boat_xbee = XBeeDevice(xbee_port, 9600)

###############################################################################
# Set up Xbee
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
    return central_xbee


###############################################################################
# Setup/Helper Functions
###############################################################################
'''
Setup ADCP connection
'''
def setup():
    ser.flush()
    print("Starting ADCP communication")
    ser.write(b'+++')
    time.sleep(3)
    s = read_response()
    print('Startup message: ', s)
    return ser

'''
Sends specified command and outputs response

Inputs:
    ser - Serial port
    command - in bytes
'''
def send(command):
    ser.write(command + b'\r\n')
    s = read_response(ser, verbose=True)
    return s

'''
Reads entire output message
'''
def read_response(verbose=False):
    response = b''
    cur_line = b''
    while True:
        s = ser.read()
        response += s
        cur_line += s
        if verbose and s == b'\n':
            print(cur_line)
            cur_line = b''

        if b'>' in response: #stop character
            return response
    return

###############################################################################
# Control Commands
###############################################################################
def start_ping():
    print("Requesting Pings")
    ser.write(b'CS\r\n')
    ser.read(10) #response includes command    

def stop_ping():
    print('Stopping Pings')
    ser.write(b'CSTOP\r\n')
    
def read_ensemble(verbose=False):
    header = ser.read(2)
    if header != b'\x7f\x7f':
        print('ERROR no header: ', header)
  
    num_bytes = ser.read(2)
    bytes_to_checksum = int.from_bytes(num_bytes, byteorder='little')-4
    if verbose:
        print('Num: ', bytes_to_checksum)
    
    data = ser.read(bytes_to_checksum)
    if verbose:
        print('Data: ', data)

    #use checksum to verify no errors
    checksum = ser.read(2)
    checksum_int = int.from_bytes(checksum, byteorder='little') 
    datasum_int = sum(b'\x7f\x7f' + num_bytes + data) % 2**16

    if checksum_int != datasum_int:
        print('ERROR: ', checksum_int, datasum_int)

    #read data to file
    hex_data = binascii.hexlify(b'\x7f\x7f' + num_bytes + data + checksum)
    f = open('results.bin', 'wb')
    f.write(data)
    f.close()

    return data

###############################################################################

def main():
    setup()
    central_xbee = discover_xbee(boat_xbee)
    
    if central_xbee == None:
        print("No device")
        ser.close()
        boat_xbee.close()
        return
    else:
        print('Device found! Waiting for starting cue...')
        msg = boat_xbee.read_data(100)
        print(msg.data)
    
    boat_xbee.set_sync_ops_timeout(10)
    
    start_ping()
    for i in range(2):
        ensemble = read_ensemble(verbose=False)
        print(ensemble)
        boat_xbee.send_data(central_xbee, ensemble[0:100])
    
    stop_ping()
    ser.write(b'===')
    ser.close()
    boat_xbee.close()

if __name__=='__main__':
    main()
