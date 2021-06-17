#!/usr/bin/python3

# https://docs-python.ru/standart-library/modul-datetime-python/primery-ispolzovanija-datetime-datetime/
import struct
from sensor_class import SensorBase
import datetime
import time

class VelostatSensor(SensorBase):
    """ The class of our handmade sensor, which provide the interface for reading data from IMU and force sensors """
    time_stamp = False
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200, imu_bytes=0, force_bytes=4, debug = 1, write_to_fl=1,file_name=None,folder_name="velostat_data"):
        '''
        If some data is not exist (for instance we are not sending imu data), then imu_bytes should be equal to 0

        Firstly - IMU data, next - Force
        '''
        super().__init__(port, baudrate, debug, write_to_fl,folder_name=folder_name,file_name=file_name)
        self.port = port
        self._debug = debug
        self.imu_bytes = imu_bytes
        self.force_bytes = force_bytes

    
    def read_all_data(self, write_to_file=0,msg=""):
        """ Docstring """
        full_array = []
        data = self.ser.read(self.imu_bytes + self.force_bytes)
        self.cur_timestamp = datetime.datetime.now().timestamp()
        for b in struct.iter_unpack('<f', data[:self.imu_bytes]):
            full_array.append(b[0])
        for d in struct.iter_unpack('<f',data[self.imu_bytes:]):
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

def velostat_calibration(exp_time=15):
    masses = [0, 0.06, 0.12,0.11,0.16,0.210,0.232,0.264, 0.290,0.325,0.370, 0.432, 0.477, 0.530, 0.590, 0, 0.652, 0.867,
    0.926,0.974, 1.032,1.157]
    message = ["0",
    "glue",
    "glue + rope",
    "metal crap",
    "crap + glue",
    "battery grey",
    "battery gray + printed",
    "battery black",
    "battery blue",
    "brick + glue",
    "brick + crap",
    "brick + crap + glue",
    "brick + gray battery",
    "brick + brick",
    "brick + brick + glue",
    "0",
    "brick + brick + glue + rope",
    "battery pack",
    "battery pack + glue",
    "battery pack + crap",
    "battery pack + crap + glue",
    "battery pack + blue battery"] 

    for i in range(len(message)):
        a = VelostatSensor(file_name=str(i),folder_name="velostat_data_for_calibration")
        print("Put on sensor "+ message[i])
        input()
        t0= time.time()
        while time.time() - t0 <= exp_time:
            k = a.read_all_data(write_to_file=1,msg=str(masses[i]))
        a.close()

if __name__ == '__main__':
    velostat_calibration()
    # a = VelostatSensor()
    # while True:
    #     k = a.read_all_data(write_to_file=1)