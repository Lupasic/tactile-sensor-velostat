#!/usr/bin/python3

# https://docs-python.ru/standart-library/modul-datetime-python/primery-ispolzovanija-datetime-datetime/
import struct
from sensor_class import SensorBase
import datetime
import time
from math import exp

class VelostatSensor(SensorBase):
    """ The class of our handmade sensor, which provide the interface for reading data from IMU and force sensors """
    time_stamp = False
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200, imu_bytes=0, force_bytes=4, debug = 1, write_to_fl=1,file_name=None,folder_name="velostat_data",calib_coeff=None):
        '''
        If some data is not exist (for instance we are not sending imu data), then imu_bytes should be equal to 0
        calib_coefficient can be either [[],[]] (amount of sensors), or None
        Firstly - IMU data, next - Force
        '''
        super().__init__(port, baudrate, debug, write_to_fl,folder_name=folder_name,file_name=file_name)
        self.port = port
        self.calib_coeff =calib_coeff
        self._debug = debug
        self.imu_bytes = imu_bytes
        self.force_bytes = force_bytes

    
    def read_all_data(self, write_to_file=0,msg=""):
        """ Docstring """
        self.ser.write(b"\n")
        full_array = []
        i = 0
        data = self.ser.read(self.imu_bytes + self.force_bytes)
        self.cur_timestamp = datetime.datetime.now().timestamp()
        for b in struct.iter_unpack('<f', data[:self.imu_bytes]):
            full_array.append(b[0])
        for d in struct.iter_unpack('<f',data[self.imu_bytes:]):
            if self.calib_coeff is not None: 
               full_array.append(self.raw2F(d[0],self.calib_coeff[i]))
               i = i + 1
            else:
                full_array.append(d[0])
        if self._debug:
            print(full_array)
        if write_to_file:
            self.write_data_to_file(full_array,msg=msg)
        return full_array

    def read_force_data(self, write_to_file=0):
        """ Docstring """
        temp = self.read_all_data()[self.imu_bytes/4:]
        if self._debug:
            print(temp)
        # self.imu_bytes/4 because of float data (4 bytes)
        return temp

    def read_all_imu_data(self, write_to_file=0):
        """ Docstring """
        temp = self.read_all_data()[:self.imu_bytes/4]
        if self._debug:
            print(temp)
        # self.imu_bytes/4 because of float data (4 bytes)
        return temp

    def read_quart(self, write_to_file=0):
        """ TODO """
        a = self.read_all_imu_data()
        return a
                
    def raw2F(self, raw_data, cur_calib_data):
        """ General model Exp2:
        General model Exp2:
        fitpoints1(x) = a*exp(b*x) + c*exp(d*x)
        Coefficients (with 95% confidence bounds):
        a =      0.3014  (0.1987, 0.4041)
        b =   -0.008011  (-0.01477, -0.001247)
        c =       1.648  (1.281, 2.014)
        d =     -0.2337  (-0.3073, -0.1601)
        """
        a = cur_calib_data[0]
        b = cur_calib_data[1]
        c = cur_calib_data[2]
        d = cur_calib_data[3]
        g = 9.8
        F = round(g* (a*exp(b*raw_data) + c*exp(d*raw_data)),2)
        return F

    # def F2raw(self, F):
    #     p1 = 0.031
    #     p2 = 0.002
    #     g = 9.8
    #     return round((F/g - p2)/p1,2)


if __name__ == '__main__':
    a = VelostatSensor()
#  new 1 [[0.3014, -0.008011, 1.648, -0.2337]]
    # a = VelostatSensor(calib_coeff=[[0.432, -0.03483, 1.742, -0.2784]])
    while True:
        k = a.read_all_data(write_to_file=1)