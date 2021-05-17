#!/usr/bin/python3

""" This code contains a basic sensor class """

import struct
from serial import Serial, SerialException
import datetime


class SensorBase:
    """ Basic class """
    filename_time_stamp = False
    def __init__(self, port, baudrate, debug = 1, write_to_fl=1, folder_name=""):
        """ Open serial port if needed.
        Args:
            debug - if true, other methods print information
         """
        self.port = port
        self._debug = debug
        self.text_file = None
        try:
            self.ser = Serial(port,baudrate,timeout=1)
        except SerialException:
            print ("error open serial port")
            exit()
        if write_to_fl:
            self.open_file(folder_name)


    def open_file(self, folder_name):
        temp_name = input("input pure file name: ")
        if self.filename_time_stamp:
            pure_file_name = temp_name + "_" + datetime.datetime.now().strftime('%d-%m-%Y_%H:%M:%S')
        else:
            pure_file_name = temp_name
        self.file_name = folder_name + "/experiment_" + pure_file_name + ".txt"
        print("name is " + self.file_name)
        self.text_file = open(self.file_name,"w")

    def write_data_to_file(self, data, msg=""):
        """Open file if it is needed and write data and timestamp to file. Filename """
        if isinstance(data,list):
            string_data = ''
            for cur_var in data:
                string_data = string_data + str(cur_var) + " "
        else:
            string_data = str(data) + " "
        if msg == "":
            pass
        else:
            msg = " " + msg
        if self.text_file is not None:
            if self._debug:
                print(string_data + str(self.cur_timestamp) + msg)
            self.text_file.write(string_data + str(self.cur_timestamp)+ msg + "\n")
        else:
            print("File is not exist, cannot write to file")

            
    def close(self):
        """ Close port and file (if exists and open)"""
        if self.ser.isOpen():
            self.ser.close()
            print("Close " + self.port + " port")
        if self.text_file is not None:
            self.text_file.close()
            print("Close " + self.file_name + " file")
