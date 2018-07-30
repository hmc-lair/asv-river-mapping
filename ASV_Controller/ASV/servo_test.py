import serial

def main():
	arduino_port = '/dev/tty.usbmodem1411'
	arduino_ser = serial.Serial(arduino_port, 9600)
	time.sleep(5) # Waiting for arduino to reset...
	print('Arduino reset?')

	servo_val = 1515
	send_servo_command(1515)
	time.sleep(3)
	send_servo_command(1195)
	time.sleep(3)
	send_servo_command(1834)

	

def send_servo_command(val):
	command = '$%d@' % val
	print(command)
	arduino_ser.write(command.encode())

if __name__ == '__main__':
	main()