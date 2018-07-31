import serial
import time
import threading

arduino_port = '/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_FFFFFFFFFFFF515B2503-if00'
# arduino_port = '/dev/tty.usbmodem1411'
arduino_ser = serial.Serial(arduino_port, 9600)

terminate = False

def main():
    time.sleep(3) # Waiting for arduino to reset...
    print('Arduino reset?')

    # Initialize Magnetometer thread
    a_thread = threading.Thread(name = 'Arduino Thread', target = update_mag)
    a_thread.setDaemon(True)
    a_thread.start()

    servo_val = 1515
    send_servo_command(1600)


def update_mag():
    while True:
        if terminate == True:
            break
        else:
            print("In mag loop: ")
            data_str = arduino_ser.readline()
            print(data_str)
            mag_data =data_str.decode().split(',')
            
            if mag_data[0] == '$SERVO':
                print(mag_data[1:])
            # self.all_data_f.write(data_str)

def send_servo_command(val):
    command = '$%d@' % val
    arduino_ser.write(command.encode())
    print(command)

if __name__ == '__main__':
    main()
