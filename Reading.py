######read the data from arduino serial and output to json files
import serial
import json
import time
serial_data = []  # create an empty list for the data output
ArduinoSerial = serial.Serial('/dev/cu.usbmodem14501', 9600)  # Create Serial port object called arduinoSerialData
time.sleep(2)  # wait for 2 seconds for the communication to get established
 # read the serial data

while 1:
    with open("positions.json") as f:
        d = json.load(f)
    serial_line = ArduinoSerial.readline()
    serial_data.append(serial_line)  #
    temp_list=serial_line.split(" ")


    front_dis = float(temp_list[0])
    print(front_dis)
    d["front_dis"] = front_dis
    #except:
     #   print "sensor error"
    sid_dis = float(temp_list[1])
    print(sid_dis)
    d["sid_dis"] = sid_dis
    #except:
     #   print "sensor error"s

    #d["y_pos"] = serial_data[y]
    #d["direc"] = serial_data[pos]


    with open('positions.json','w') as f:
        json.dump(d,f)
