#!/usr/bin/python3

import serial
import time

# # class velostat_sensor

# import serial
# import struct
# import sys, csv
# from matplotlib.lines import Line2D
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# from time import sleep
# import numpy as np


# # class Scope:
# #     def __init__(self, ax, maxt=40, dt=1):
# #         self.ax = ax
# #         self.dt = dt
# #         self.maxt = maxt
# #         self.tdata = [0]
# #         self.ydata = [0]
# #         self.line = Line2D(self.tdata, self.ydata)
# #         self.ax.add_line(self.line)
# #        #  self.ax.set_ylim(-.1, 1.1)
# #         self.ax.set_xlim(0, self.maxt)

# #     def update(self, y):
# #         lastt = self.tdata[-1]
# #         if lastt > self.tdata[0] + self.maxt:  # reset the arrays
# #             self.tdata = [self.tdata[-1]]
# #             self.ydata = [self.ydata[-1]]
# #             self.ax.set_xlim(self.tdata[0], self.tdata[0] + self.maxt)
# #             self.ax.figure.canvas.draw()

# #         t = self.tdata[-1] + self.dt
# #         self.tdata.append(t)
# #         self.ydata.append(y)
# #         self.line.set_data(self.tdata, self.ydata)
# #         return self.line,

# class DataStructure:
#     force_data = []
#     __imu_velocity__ = [0]*3

#     def addSensorData(self, x):
#         self.force_data.append(x)


# # def readData():
    

# ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=2)
# pData = []

# plt.ion()

# i = 0

# structure = DataStructure()
# while True:
#     data = ser.read(8)
#     cur_force_sensor_data = []
#     # print(data)
#     for b in struct.iter_unpack('<f', data[:0]):
#         pass
#         # print(b, end='')
#         # print('\n')
#     for d in struct.iter_unpack('<h',data[0:]):
#         cur_force_sensor_data.append(d)
#         print(d)
#     structure.addSensorData(cur_force_sensor_data)
#     cur_force_sensor_data=[]

#     # fig, ax = plt.subplots()
#     # scope = Scope(ax)
#     # print(structure.force_data[-1][2][0])



# # real-time plotting loop
#     # try:
#     #     i = i+1
#     #     print(structure.force_data[-1][0][0])
#     #     # pData.append(structure.getForceData()[-1][0][0])
#     #     # # print(pData)
#     #     # # pyplot.ylim([0, 1])
#     #     # # del pData[0]
#     #     # # l1.set_xdata([k for k in range(i)])
#     #     # # l1.set_ydata(pData)  # update the data
#     #     # plt.plot([k for k in range(i)],pData)  # update the plot
#     #     # plt.pause(0.0001)
#     #     # sleep(1)
#     # except KeyboardInterrupt:
#     #     break
