import serial
import time

arduino_port = '/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_FFFFFFFFFFFF515B2503-if00'
arduino_ser = serial.Serial(arduino_port, 9600)

def main():
        time.sleep(5) # Waiting for arduino to reset...
	print('Arduino reset?')

	servo_val = 1515
	send_servo_command(1515)
	time.sleep(3)
	send_servo_command(1195)
	time.sleep(3)
	send_servo_command(1834)
    time.sleep(3)
    send_servo_command(1600)

	

def send_servo_command(val):
	command = '$%d@' % val
	print(command)
	arduino_ser.write(command.encode())

if __name__ == '__main__':
	main()
