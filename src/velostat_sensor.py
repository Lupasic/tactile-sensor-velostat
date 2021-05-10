#!/usr/bin/python3

# https://docs-python.ru/standart-library/modul-datetime-python/primery-ispolzovanija-datetime-datetime/
from serial import Serial
import struct
import time

class VelostatSensor:
    """ The class of our handmade sensor, which provide the interface for reading data from IMU and force sensors """
    time_stamp = False
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, imu_bytes=0, force_bytes=8, debug = 1, write_to_fl=1):
        '''
        If some data is not exist (for instance we are not sending imu data), then imu_bytes should be equal to 0

        Firstly - IMU data, next - Force
        '''
        self.port = port
        self.ser = Serial(port,baudrate,timeout=2)
        self.imu_bytes = imu_bytes
        self.force_bytes = force_bytes
        self._debug = debug
        try:
            if not self.ser.isOpen():
                self.ser.open()            
                print("Port has been opened")
            else:
                print("Port was opened before")
        except SerialException as e:
            print ("error open serial port")
            exit()
        # Needed to read a trash first
        if write_to_fl:
            self.open_file()
    
    def read_all_data(self, write_to_file=0):
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
            write_to_file(full_array)
        return full_array

    def read_force_data(self, write_to_file=0):
        """ Docstring """
        temp = read_all_data()[self.imu_bytes/4:]
        if self._debug:
            print(temp)
        # self.imu_bytes/4 because of float data (4 bytes)
        return temp

    def read_all_imu_data(self, write_to_file=0):
        """ Docstring """
        temp = read_all_data()[:self.imu_bytes/4]
        if self._debug:
            print(temp)
        # self.imu_bytes/4 because of float data (4 bytes)
        return temp

    def read_quart(self, write_to_file=0):
        """ TODO """
        a = read_all_imu_data()
        return a


    def open_file(self):
        temp_name = int(input("input pure file name: "))
        if self.time_stamp:
            pure_file_name = temp_name + "_" + datetime.datetime.now().strftime('%d-%m-%Y_%H:%M:%S')
        else:
            pure_file_name = temp_name
        self.file_name = "velostat_data/experiment_" + pure_file_name + ".txt"
        self.text_file = open(file_name,"w")

    def write_data_to_file(self, data):
        """Open file if it is needed and write data and timestamp to file. Filename """
        if self.__open_file_fl__ == 0:
            self.open_file()
            self.__open_file_fl__ = 1
        string_data = ''
        for cur_var in data:
            string_data = string_data + " " + str(cur_var)
        self.text_file.write(string_data + " " + datetime.datetime.now().timestamp())

    def close(self):
        """ Close port and file (if exists and open)"""
        if self.ser.isOpen():
            self.ser.close()
            print("\n Close " + self.port + " port")
        if self.text_file is not None:
            self.text_file.close()
            print("\n Close " + self.file_name + " file")