######read the data from arduino serial and output to json files
import serial
import json
import time
serial_data = []  # create an empty list for the data output
ArduinoSerial = serial.Serial('/dev/cu.usbmodem14301', 9600)  # Create Serial port object called arduinoSerialData
time.sleep(2)  # wait for 2 seconds for the communication to get established
 # read the serial data

while 1:
    with open("positions.json") as f:
        d = json.load(f)
    serial_line = ArduinoSerial.readline()
    serial_data.append(serial_line)  #
    temp_list=serial_line.split(" ")


    if temp_list[0] == "Heading":
        direc = float(temp_list[2])
        print(direc)
        d["direc"] = direc
    elif temp_list[0] != "Out":
        distance = float(temp_list[0])
        print(distance)
        d["front_dis"] = distance

    with open("positions.json","w") as f:
        json.dump(d,f)








    #d["y_pos"] = serial_data[y]
    #d["direc"] = serial_data[pos]


    #with open(".json") as f:
     #   json.dump(d)
