import serial
import time

motor_ser = serial.Serial('/dev/tty.usbserial-FT8VW9AR', 115200)


def send_command2ASV(throttle, rudder):
	''' Send command to the ASV Vessel Control Unit, triggered 
	when there's an incoming command'''
	throttle = throttle
	rudder = rudder
	message = "!pwm, *, %4.3f, %4.3f, %4.3f, *, *\r\n" % (throttle, throttle, rudder)
	motor_ser.write(message.encode())
	print(message)

def enter_autonomous_mode():
	''' Tell the VCU to enter autonomous mode'''
	message = '!SetAutonomousControl\r\n'
	motor_ser.write(message.encode())
	print(message)


enter_autonomous_mode()
send_command2ASV(10, 10)
time.sleep(1)
send_command2ASV(0, 0)
