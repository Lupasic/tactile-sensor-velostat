#!/usr/bin/python3

# import serial
from serial import Serial, SerialException
import time

class Futek:
    # ser = None
    # text_file = None
    # file_name = ''
    # port = ''
    __open_file_fl__ = 0 
    def __init__(self, port='/dev/ttyACM0', baudrate=9600, debug = 0):
        self.port = port
        self._debug = debug
        self.ser = Serial(port,baudrate,timeout=0.02)
        try:
            self.ser.open()
            print ("Port has been opened")
        except SerialException as e:
            print ("error open serial port: ") + str(e)
            exit()
    
    def readData(self, write_to_file=0):
        reading = self.ser.readline().decode('utf-8')
        if self._debug:
            print(reading)
        if write_to_file:
            write_to_file(reading)
        return reading


    def open_file(self):
        te = time.gmtime()
        cur_time_string = str(te.tm_mday)+"."+str(te.tm_mon)+"."+str(te.tm_year)+"_"+ str(te.tm_hour+3)+":"+ str(te.tm_min)+":" + str(te.tm_sec)
        self.file_name = "exp_data/Experiment_" + cur_time_string + ".txt"
        self.text_file = open(file_name,"w")

    def write_data_to_file(self, data):
        if self.__open_file_fl__ == 0:
            open_file()
            self.__open_file_fl__ = 1
        else:
            te = time.gmtime()
            text_file.write(str(data) + " " + str(te.tm_hour+3)+":"+ str(te.tm_min)+":" + str(te.tm_sec))

            
    def close(self):
        if self.ser.isOpen():
            self.ser.close()
            print("\n Close " + self.port + " port")
        if self.text_file.isOpen():
            self.text_file.close()
            print("\n Close " + self.file_name + " file")
