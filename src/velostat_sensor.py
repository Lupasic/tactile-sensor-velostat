#!/usr/bin/python3

# https://docs-python.ru/standart-library/modul-datetime-python/primery-ispolzovanija-datetime-datetime/
from serial import Serial
import struct
from sensor_class import SensorBase
import time

class VelostatSensor(SensorBase):
    """ The class of our handmade sensor, which provide the interface for reading data from IMU and force sensors """
    time_stamp = False
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, imu_bytes=0, force_bytes=8, debug = 1, write_to_fl=1):
        '''
        If some data is not exist (for instance we are not sending imu data), then imu_bytes should be equal to 0

        Firstly - IMU data, next - Force
        '''
        super().__init__(port, baudrate, debug, write_to_fl,folder_name="velostat_data")
        self.port = port
        self._debug = debug
        self.imu_bytes = imu_bytes
        self.force_bytes = force_bytes

    
    def read_all_data(self, write_to_file=0,msg=""):
        """ Docstring """
        full_array = []
        data = self.ser.read(self.imu_bytes + self.force_bytes)
        for b in struct.iter_unpack('<f', data[:self.imu_bytes]):
            full_array.append(b[0])
        for d in struct.iter_unpack('<h',data[self.imu_bytes:]):
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