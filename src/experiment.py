#!/usr/bin/python3

import time
from futek import Futek
# import ur5e
# import velostat_sensor

a = Futek()
k = a.readData()
k = a.readData()
k = a.readData()
# while True:
    # k = a.readData()
    # time.sleep(0.02)
a.close()