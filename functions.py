import serial #Serial imported for Serial communication
import time #Required to use delay functions
#import numpy as np
import json

def turn(x,i):

    x0 = 30
    dx = 30

    turning_point = [30, 30, 30, 60,60,60,90,90,90,120]
    #create a list of turning points


    # get the live position and direction

    if abs(x - turning_point[i]) <= 5 :#when turning point is reached

        print(turning_point[i])
        print (x)#i is counter
        print(i)

        #tell left motor to turn back
        ArduinoSerial.write('6')
        #tell right motor to turn forward
        time.sleep(3)  # wait for 3 seconds for the car to complete turning

        ArduinoSerial.write('5')
        return 1# tell motor to stop turning and go back to straight movement
         # counter plus 1, prepare for the next turning point
    return 0

def check(check):
    if check == 1:
        ArduinoSerial.write('13')



def safe_mine(yellow,x):

    if yellow == '1':
        ArduinoSerial.write('7')#stop
        ArduinoSerial.write('8') # swiper down
        back(x)
        ArduinoSerial.write('9')



def dangerous_mine(red):
    if red == '1':
        ArduinoSerial.write('7')  # stop
        time.sleep(3)
        ArduinoSerial.write('5')
    #create the list for the positions of dangerous mines

    # get the live position and direction   
    #sense.x = x

    #dangerous_mine_is_detected = yellow_value_given_by_color_sensor

    # when dangerous mine is detected
    #if dangerous_mine_is_detected == 1:
     #   print ("turn on red LED")
        #record the position of the dangerous mine
      #  dangerious_mine_position.append({sense.x, sense.y})
        # stay for 3 seconds
       # time.sleep(3)
        #return (x,y)


#def centre_position(x): 

 #   x_centre_point = 120
    # give a centre position

    # get the live position and direction
  #  sense.x = x


    #when arrive at centre point, go back to original point
#    if round(sense.x - x_centre_point) == 0
        # turn the car and let it point to the original point
 #       ArduinoSerial.write('turn')
        # when home direction has reached
  #      if round(sense.direction - home_direction) == 0:
            # tell motor to stop turning and go back to straight movement
   #         ArduinoSerial.write('straight')
            # when home is arrived, stop moving permanently
    #        if round(sense.x) == 0 and round(sense.y) == 0:
     #           break



def error_control(error):
    if error > 0:
        ArduinoSerial.write('10') #move to the inside
        time.sleep(1)
        ArduinoSerial.write('5')
    if error < 0:
        ArduinoSerial.write('11') #move to outside
        time.sleep(1)
        ArduinoSerial.write('5')



def back(x):

    second = x/10
    ArduinoSerial.write('12')#code for car to go back
    time.sleesssp(second)
    ArduinoSerial.write('5')

ArduinoSerial = serial.Serial('/dev/cu.usbmodem14301',9600)
time.sleep(2) #wait for 2 secounds for the communication to get established
#print ArduinoSerial.readline() #read the serial data and print it as line
print ("Starting program")
yellow = []
red = []
i=0
position = {"error_time": 0, "p_sid_dis": 35.0, "yellow": 0, "front_dis": 32.0, "p_error": 0.0, "red": 0, "error": 0.0, "sid_dis": 35.0,"checklist":[], "check":0}
#ArduinoSerial.write('6')
#ime.sleep(3)
ArduinoSerial.write('5')
print('has started motor')
while 1: #Do this forever


    serial_line = ArduinoSerial.readline()
    #serial_data.append(serial_line)  #
    temp_list=serial_line.split(" ")

#error handling
    position["p_sid_dis"] = position["sid_dis"]
    position["p_error"] = position["error"]
    position["error"] = position["p_sid_dis"] - float(temp_list[0])


    if abs(position["error"] ) >= 2 and position["error"]*position["p_error"] >= 0:
        position["error_time"] += 1
    else:
        position["error_time"] = 0

#how to read sensor



    front_dis = float(temp_list[1])
    print(front_dis)
    position["front_dis"] = front_dis
    #except:
     #   print "sensor error"
    sid_dis = float(temp_list[0])
    print(sid_dis)
    position["sid_dis"] = sid_dis
    print (position)


    x = position["front_dis"]
    y = position["sid_dis"]

    checklist_raw = temp_list[2:]
    checklist = []
    for item in checklist_raw:
        checklist.append(int(item))

    yellow = 0
    red = 0
    if 1 in checklist:
        if (checklist[0] == 1 or checklist[7] == 1) and sid_dis <= 30:
            check = 0
        else:
            check = 1

    check(check)

    if 3 in checklist:
        if 2 in checklist:
            yellow = 1
        else:
            red = 1

    safe_mine(yellow,x)
    dangerous_mine(red)



    i+= turn(x,i)


    #error control
    if int(position["error_time"]) >= 2:
        error_control(position["error"])

 

    





