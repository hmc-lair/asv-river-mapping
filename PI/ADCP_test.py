import serial

COM_PORT = "/dev/ttyUSB0" #tbd
BAUD_RATE = "115200"

def setup():
    print("Starting ADCP communication")
    ser = serial.Serial(COM_PORT, BAUD_RATE, stopbits=serial.STOPBITS_ONE)
    ser.write(b'+++')
    s = ser.read(80)
    print('Message: ', s)

def main():
    setup()

if __name__=='__main__':
    main()
