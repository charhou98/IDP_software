import serial #Serial imported for Serial communication
import time #Required to use delay functions
#import numpy as np
import json

ArduinoSerial = serial.Serial('/dev/cu.usbmodem14401',9600)
time.sleep(2) #wait for 2 secounds for the communication to get established
#print ArduinoSerial.readline() #read the serial data and print it as line
print ("Starting program")


position = {"error_time": 0, "p_sid_dis": 35.0, "yellow": 0, "front_dis": 32.0, "p_error": 0.0, "red": 0, "error": 0.0, "sid_dis": 35.0,"checklist":[], "check":0}

print('has started motor')
while 1: #Do this forever
    for i in range (5):
        try:
            with open("positions.json") as f:
                position = json.load(f)
        except:
            pass

    ArduinoSerial.write('5')
    time.sleep(7)
    ArduinoSerial.write('2')
    time.sleep(7)
    ArduinoSerial.write('7')
