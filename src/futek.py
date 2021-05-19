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

    def __init__(self, port='/dev/ttyACM0', baudrate=115200, debug=1, write_to_fl=1):
        """ Open serial port if needed.
        Args:
            debug - if true, other methods print information
         """
        super().__init__(port, baudrate, debug, write_to_fl, folder_name="futek_data")
        self.port = port
        self._debug = debug
        # Inisialisation (empirical)
        time.sleep(3)

    def readData(self, write_to_file=0, msg=""):
        """ Read data from serial port and print to file if needed"""
        self.ser.write(b"\n")
        try:
            s = ''
            while len(s) == 0:
                s = self.ser.readline().decode('utf-8').rstrip()
                break
            reading = int(s)
            self.cur_timestamp = datetime.datetime.now().timestamp()
            if self._debug:
                print(self.raw2F(reading))
            if write_to_file:
                self.write_data_to_file(self.raw2F(reading), msg=msg)
            return reading
        except Exception as e:
            print(e)
            print("Data was corrupted")
            return -1

    def raw2F(self, raw_data):
        """ Linear model Poly1:
        fitpoints(x) = p1*x + p2
        Coefficients (with 95% confidence bounds):
        p1 =     0.03099  (0.03079, 0.03119)
        p2 =    0.002797  (-0.01346, 0.01906)
        """
        p1 = 0.031
        p2 = 0.002
        g = 9.8 
        F = round(g* (p1*raw_data + p2),2)
        return F

    def F2raw(self, F):
        p1 = 0.031
        p2 = 0.002
        g = 9.8
        return round((F/g - p2)/p1,2)


if __name__ == '__main__':
    a = Futek()
    while True:
        k = a.readData(write_to_file=1)
    # while True:
    #     if a.readData()>=100:
    #         a.readData(write_to_file=1,msg="kek")
