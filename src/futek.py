#!/usr/bin/python3

import serial
import time

# class Futek
# #!/usr/bin/python3

# import serial
# import keyboard
# import time

# experiments_amount = 10
# fl = 0

# def change_fl():
# 	global fl
# 	fl = 1

# ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.1)
# ser.close()
# try:
#     ser.open()
#     print ("Port has been opened")
# except Exception as e:
#     print ("error open serial port: ") + str(e)
#     exit()

# for i in range(experiments_amount):
# 	te = time.gmtime()
# 	cur_time_string = str(te.tm_mday)+"."+str(te.tm_mon)+"."+str(te.tm_year)+"_"+ str(te.tm_hour+3)+":"+ str(te.tm_min)+":" + str(te.tm_sec)
# 	text_file = open("exp_data/Experiment_" + cur_time_string + ".txt","w")
# 	while True:
# 		keyboard.add_hotkey('Ctrl + 1', change_fl)
# 		if fl == 1:
# 			print("Close doc " + str(i))
# 			fl =0
# 			text_file.close()
# 			break
# 		try:
# 			reading = ser.readline().decode('utf-8')
# 			print(reading)
# 			text_file.write(reading)
# 		except UnicodeDecodeError:
# 			pass
# 		except KeyboardInterrupt:
# 			if ser.isOpen():
# 				ser.close()
# 				text_file.close()
# 				print("\nClose the program")
