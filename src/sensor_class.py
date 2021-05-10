#!/usr/bin/python3

""" This code contains a basic sensor class """

import struct
from serial import Serial, SerialException
import datetime

class SensorBase:
    