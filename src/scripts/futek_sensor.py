#!/usr/bin/python3

""" This code contains a class for working with Futek force sensor via serial port """

from serial.serialutil import to_bytes
from sensor_class import SensorBase
import struct
from serial import Serial, SerialException
import datetime
import time


class FutekSensor(SensorBase):
    """ Main class """

    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, debug=0, write_to_fl=1,file_name=None, folder_name="futek_data"):
        """ Open serial port if needed.
        Args:
            debug - if true, other methods print information
         """
        super().__init__(port, baudrate, debug, write_to_fl, folder_name=folder_name,file_name=file_name)
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
            if s == '':
                return -1
            else:
                reading = int(s)
            self.cur_timestamp = datetime.datetime.now().timestamp()
            if self._debug:
                print(self.raw2F(reading))
            if write_to_file:
                self.write_data_to_file(self.raw2F(reading), msg=msg)
            return reading
        except ValueError:
            return -1
        except Exception as e:
            print(e)
            print("Data was corrupted")
            return -1

    def raw2F(self, raw_data):
        """ Linear model Poly1:
        fitpoints(x) = p1*x + p2
     Coefficients (with 95% confidence bounds):
       p1 =    0.006321  (0.006239, 0.006403)
       p2 =     -0.1972  (-0.2043, -0.19)
        """
        p1 = 0.006321
        p2 = -0.1978
        g = 9.8 
        F = round(g* (p1*raw_data + p2),2)
        return F

    def F2raw(self, F):
        p1 = 0.006321
        p2 = -0.1972
        g = 9.8
        return round((F/g - p2)/p1,2)


if __name__ == '__main__':
    a = FutekSensor(debug=1)
    # futek = FutekSensor(port='/dev/ttyUSB2',file_name="futek_extra")
    while True:
        k = a.readData(write_to_file=1)
        # d = futek.readData(write_to_file=1)
