import serial
import time

COM_PORT = "/dev/ttyUSB0" #Pi
# COM_PORT = "/dev/tty.usbserial-FT8VW9AR" # for John's macbook
BAUD_RATE = "115200"
ser = serial.Serial(COM_PORT, BAUD_RATE, stopbits=serial.STOPBITS_ONE)


###############################################################################
# Setup/Helper Functions
###############################################################################
'''
Setup connection
'''
def setup():
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

###############################################################################

def main():
    setup()
    start_ping()
    ser.read(10) #response includes command
    
    for i in range(2):
        read_ensemble(verbose=True)
    
    stop_ping()
    ser.write(b'===')
    ser.close()

if __name__=='__main__':
    main()
