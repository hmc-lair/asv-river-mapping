import serial
import time
from xbee import XBee
from digi.xbee.devices import XBeeDevice

class ASV_environment:

    def __init__(self):

        self.map = [] # Not sure what this will look like
        self.data_stream = '' # current data stream from the xbee

        # Initialize port names
        self.GPS_PORT = '/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FT8VW9AR-if00-port0' #Pi ADCP
        # self.GPS_PORT = '/dev/tty.usbserial-FT8VW9AR'
        # self.XBEE_PORT = '/dev/tty.usbserial-DN02Z6QY'
        self.XBEE_PORT = '/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DN02Z3LX-if00-port0'
        self.ADCP_PORT = '/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FT8VWDWP-if00-port0'
        # self.ADCP_PORT = '/dev/tty.usbserial-FT8VWDWP'
        self.starboard_PORT = ''
        self.port_PORT = ''
        self.servo_PORT = ''

        # serial ports
        self.GPS_ser = None
        self.my_xbee = None
        self.dest_xbee = None
        self.starboard_ser = None
        self.port_ser = None
        self.ADCP_ser = None

        # Create serial port
        self.robot_mode = "HARDWARE MODE" # HARDWARE MODE
        
        # Setup Hardware
        if (self.robot_mode == "HARDWARE MODE"):
            self.setup_GPS()
            # self.setup_motors()
            self.setup_ADCP()
            self.dest_xbee = self.discover_xbee()


    def setup_GPS(self):
        self.GPS_ser = serial.Serial(self.GPS_PORT, bytesize = 8)
        self.GPS_ser.baudrate = 19200
        self.GPS_ser.parity = serial.PARITY_NONE
        self.GPS_ser.stop_bits = serial.STOPBITS_ONE
        self.GPS_ser.write("$JASC,GPGGA,20" + b'\r\n')
        time.sleep(0.1)
        self.GPS_ser.flushInput()

    def setup_motors(self):
        self.port_ser = serial.Serial(self.port_PORT, 115200)
        self.starboard_ser = serial.Serial(self.starboard_PORT, 115200)

###############################################################################
# ADCP Functions
###############################################################################

    def setup_ADCP(self):
        ''' Get the ADCP ready for pinging '''
        BAUD_RATE = "115200"
        self.ADCP_ser = serial.Serial(self.ADCP_PORT, BAUD_RATE, stopbits=serial.STOPBITS_ONE)
        self.ADCP_ser.flush()
        print("Starting ADCP communication")
        self.ADCP_ser.write(b'+++')
        time.sleep(3)
        s = self.read_ADCP_response(verbose=True)
        print('Startup message: ', s)


    def send_ADCP(self, command):
        self.ADCP_ser.write(command + b'\r\n')
        s = self.read_ADCP_response( verbose=True)
        return s

    def read_ADCP_response(self, verbose=False):
        response = b''
        cur_line = b''
        while True:
            s = self.ADCP_ser.read()
            response += s
            cur_line += s
            if verbose and s == b'\n':
                print(cur_line)
                cur_line = b''

            if b'>' in response: #stop character
                return response
        return

    def start_ping(self):
        print("Requesting Pings")
        self.ADCP_ser.write(b'CS\r\n')
        self.ADCP_ser.read(10) #response includes command  

    def stop_ping(self):
        print('Stopping Pings')
        self.ADCP_ser.write(b'CSTOP\r\n')  

    def discover_xbee(self):
        self.my_xbee = XBeeDevice(self.XBEE_PORT, 9600)
        self.my_xbee.open()
        xbee_network = self.my_xbee.get_network()
        xbee_network.start_discovery_process()
        dest_xbee = None

        print('Looking for devices...')
        tries = 0
        while dest_xbee == None:
            if tries >= 5:
                break
            while xbee_network.is_discovery_running():
                time.sleep(0.5)
            dest_xbee = xbee_network.discover_device('central')
            tries += 1

        if dest_xbee == None:
            print('No device found... ')
            return None
        else:
            print('device found! waiting..')
            self.my_xbee.send_data(dest_xbee, b'Device found! Waiting for starting cue...')
            msg = self.my_xbee.read_data(100)
            return dest_xbee

class ASV_sim_env(ASV_environment):

    def __init__(self):
        self.robot_mode = "SIM MODE"

    def setup_GPS(self):
        pass

    def setup_motors(self):
        pass

    def setup_ADCP(self):
        ''' Get the ADCP ready for pinging '''
        pass


    def send_ADCP(self, command):
        pass

    def read_ADCP_response(self, verbose=False):
        pass

    def start_ping(self):
        pass

    def stop_ping(self):
        pass 

    def discover_xbee(self):
        pass

