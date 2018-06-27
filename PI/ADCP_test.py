import serial
import time

# COM_PORT = "/dev/ttyUSB0" #tbd
COM_PORT = "/dev/tty.usbserial-FT8VW9AR" # for John's macbook
BAUD_RATE = "115200"
ser = serial.Serial(COM_PORT, BAUD_RATE, stopbits=serial.STOPBITS_ONE)


def setup():
    print("Starting ADCP communication")
    ser.write(b'+++')
    time.sleep(1)
    s = ser.read(100)
    print('Message: ', s.decode('utf-8'))

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
