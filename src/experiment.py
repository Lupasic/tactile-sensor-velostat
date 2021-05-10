#!/usr/bin/python3

import time
from futek import Futek
# import ur5e
# import velostat_sensor

a = Futek()
k = a.readData(write_to_file=1)
k = a.readData()
k = a.readData(write_to_file=1)
# while True:
    # k = a.readData()
    # time.sleep(0.02)
a.close()