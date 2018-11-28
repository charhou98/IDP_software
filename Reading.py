######read the data from arduino serial and output to json files
import serial
import json
import time
serial_data = []  # create an empty list for the data output
ArduinoSerial = serial.Serial('/dev/cu.usbmodem14401', 9600)  # Create Serial port object called arduinoSerialData
time.sleep(2)  # wait for 2 seconds for the communication to get established
 # read the serial data

while 1:
    with open("positions.json") as f:
        d = json.load(f)
    serial_line = ArduinoSerial.readline()
    serial_data.append(serial_line)  #
    templist=serial_line.split(" ")
    temp_list = [x for x in templist if x != '']

#error handling
    d["p_sid_dis"] = d["sid_dis"]
    d["p_error"] = d["error"]
    d["error"] = d["p_sid_dis"] - float(temp_list[0])



    if abs(d["error"] ) >= 5 and d["error"]*d["p_error"] >= 0:
        d["error_time"] += 1
    else:
        d["error_time"] = 0


    front_dis = float(temp_list[1])
    print(front_dis)
    d["front_dis"] = front_dis
    #except:
     #   print "sensor error"
    sid_dis = float(temp_list[0])
    print(sid_dis)
    d["sid_dis"] = sid_dis

    checklist_raw = temp_list[2:10]
    colorlist_raw = temp_list[10:18]
    colorlist = []
    checklist = []
    for item in checklist_raw:
        checklist.append(int(item))
    for item in colorlist_raw:
        colorlist.append(int(item))
    d["checklist"] = checklist
    d["colorlist"] = colorlist
    print (d)
    #except:
     #   print "sensor error"s

    #d["y_pos"] = serial_data[y]
    #d["direc"] = serial_data[pos]


    with open('positions.json','w') as f:
        json.dump(d,f)
