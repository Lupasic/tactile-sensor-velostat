#!/usr/bin/python3

""" This code contains a class for working with Futek force sensor via serial port """

import struct
from serial import Serial, SerialException
import datetime

class Futek:
    """ Main class """
    time_stamp = False
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, debug = 1, write_to_fl=1):
        """ Open serial port if needed.
        Args:
            debug - if true, other methods print information
         """
        self.port = port
        self._debug = debug
        self.text_file = None
        self.ser = Serial(port,baudrate,timeout=1)
        try:
            if not self.ser.isOpen():
                self.ser.open()            
                print("Port has been opened")
            else:
                print("Port was opened before")
        except SerialException as e:
            print ("error open serial port")
            exit()
        if write_to_fl:
            self.open_file()
        # Needed to read a trash first
        self.ser.readline()
        self.ser.readline()
        self.ser.readline()
    
    def readData(self, write_to_file=0):
        """ Read data from serial port and print to file if needed"""
        try:
            reading = int(self.ser.readline().decode('utf-8'))
            self.cur_timestamp = datetime.datetime.now().timestamp()
            if self._debug:
                print(reading)
            if write_to_file:
                self.write_data_to_file(reading)
            return reading
        except Exception:
            pass
            return None


    def open_file(self):
        temp_name = input("input pure file name: ")
        print(temp_name)
        if self.time_stamp:
            pure_file_name = temp_name + "_" + datetime.datetime.now().strftime('%d-%m-%Y_%H:%M:%S')
        else:
            pure_file_name = temp_name
        self.file_name = "futek_data/experiment_" + pure_file_name + ".txt"
        print("name is " + self.file_name)
        self.text_file = open(self.file_name,"w")

    def write_data_to_file(self, data):
        """Open file if it is needed and write data and timestamp to file. Filename """
        if self.text_file is not None:
            if self._debug:
                print(str(data) + " " + str(self.cur_timestamp))
            self.text_file.write(str(data) + " " + str(self.cur_timestamp)+"\n")
        else:
            print("File is not exist, cannot write to file")

            
    def close(self):
        """ Close port and file (if exists and open)"""
        if self.ser.isOpen():
            self.ser.close()
            print("\n Close " + self.port + " port")
        if self.text_file is not None:
            self.text_file.close()
            print("\n Close " + self.file_name + " file")
