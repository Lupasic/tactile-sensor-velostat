#!/usr/bin/python3

from serial import Serial
import struct
import time

class VelostatSensor:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, imu_bytes=0, force_bytes=8, debug = 0):
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
            self.ser.open()
            print ("Port has been opened")
        except SerialException as e:
            print ("error open serial port: ") + str(e)
            exit()
    
    def read_all_data(self, write_to_file=0):
        full_array = []
        data = self.ser.read(self.imu_bytes + self.force_bytes)
        for b in struct.iter_unpack('<f', data[:self.imu_bytes]):
            full_array.append(b)
        for d in struct.iter_unpack('<h',data[self.imu_bytes:]):
            full_array.append(d)
        if self._debug:
            print(full_array)
        if write_to_file:
            write_to_file(full_array)
        return full_array

    def read_force_data(self, write_to_file=0):
        temp = read_all_data()[self.imu_bytes/4:]
        if self._debug:
            print(temp)
        # self.imu_bytes/4 because of float data (4 bytes)
        return temp

    def read_all_imu_data(self, write_to_file=0):
        temp = read_all_data()[:self.imu_bytes/4]
        if self._debug:
            print(temp)
        # self.imu_bytes/4 because of float data (4 bytes)
        return temp

    def read_quart(self, write_to_file=0):
        a = read_all_imu_data()
        return a


    def close(self):
        if self.ser.isOpen():
            self.ser.close()
            print("\n Close " + self.port + " port")
        if self.text_file.isOpen():
            self.text_file.close()
            print("\n Close " + self.file_name + " file")