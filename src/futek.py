#!/usr/bin/python3

""" This code contains a class for working with Futek force sensor via serial port """

from serial.serialutil import to_bytes
from sensor_class import SensorBase
import struct
from serial import Serial, SerialException
import datetime
import time

class Futek(SensorBase):
    """ Main class """
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, debug = 1, write_to_fl=1):
        """ Open serial port if needed.
        Args:
            debug - if true, other methods print information
         """
        super().__init__(port, baudrate, debug, write_to_fl,folder_name="futek_data")
        self.port = port
        self._debug = debug
        # Needed to read a trash first
        time.sleep(3)
    
    def readData(self, write_to_file=0, msg=""):
        """ Read data from serial port and print to file if needed"""
        # self.ser.flush()
        self.ser.write(b"kek\n")
        try:
            s = ''
            while len(s) == 0:
                s = self.ser.readline().decode('utf-8').strip()
                break
            reading = int(s)
            # reading = int(self.ser.read(self.ser.inWaiting()).decode('utf-8'))
            self.cur_timestamp = datetime.datetime.now().timestamp()
            if self._debug:
                print(reading)
            if write_to_file:
                self.write_data_to_file(reading,msg=msg)
            return reading
        except Exception as e:
            print(e)
            print("Data was corrupted")
            return -1
