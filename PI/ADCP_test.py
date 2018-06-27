import serial
import time

# COM_PORT = "/dev/ttyUSB0" #Pi
COM_PORT = "/dev/tty.usbserial-FT8VW9AR" # for John's macbook
BAUD_RATE = "115200"
ser = serial.Serial(COM_PORT, BAUD_RATE, stopbits=serial.STOPBITS_ONE)

'''
Setup connection
'''
def setup():
    print("Starting ADCP communication")
    ser.write(b'+++')
    s = read_response(ser)
    #print('Startup message: ', s)
    return ser

'''
Sends specified command and outputs response

Inputs:
    ser - Serial port
    command - in bytes
'''
def send(command):
    ser.write(command + b'\r')
    s = read_response(ser, verbose=True)
    return s

'''
Reads entire output message
'''
def read_response(port, verbose=False):
    response = b''
    cur_line = b''
    while True:
        s = port.read()
        response += s
        cur_line += s
        if verbose and s == b'\n':
            print(cur_line[:-2])
            cur_line = b''

        if b'>' in response: #stop character
            return response
    return

def start_ping():
	print("Requesting Pings")
	ser.write(b'CS\r\n')

def stop_ping():
	print('Stopping Pings')
	ser.write(b'CSTOP\r\n')
	# s = ser.read(10)
	# time.sleep(1)
	# print(s)

def main():
    setup()
    start_ping()
    while True:
    	try:
    		if ser.in_waiting > 0:
    			data = ser.readline()
    			print(data)
    	except KeyboardInterrupt:
    		break
    stop_ping()
    ser.close()

if __name__=='__main__':
    main()
