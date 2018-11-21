import serial #Serial imported for Serial communication
import time #Required to use delay functions
import numpy as np
import json

def turn(x,i):

    x0 = 30
    dx = 30

    turning_point = [30, 30, 30, 60,60,60,90,90,90]
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

def safe_mine(yellow):
    # create the list for the positions of safe mines

    # get the live position and direction
    #sense.x = x


    #detect safe mine

    #safe_mine_is_detected = yellow_value_given_by_color_sensor 

    #when safe mine is detected
    if yellow == '1':
        ArduinoSerial.write('7')#stop
        ArduinoSerial.write('8')#yello led
        #record the position of the safe mine
        #safe_mine_position.append({sense.x, sense.y})
        #stay for 3 seconds
        time.sleep(3)
        #tell the car to push out the mine
        ArduinoSerial.write('5')
        #ArduinoSerial.write('prepare_push')
        #don't turn the direction this time
#        return (x,y)


def dangerous_mine(red):
    if red == '1':
        ArduinoSerial.write('7')  # stop
        ArduinoSerial.write('9')  # red led
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

#def error_adjust(x,y,direc,request_direc):
 #   if request_direc-direc>= 10:
  #      print ("turn to one side")
   # elif request_direc-direc <= -10:
    #    print ("turn to the other side")_

def error_control(error):
    if error > 0:
        ArduinoSerial.write('10') #move to the inside
        time.sleep(1)
        ArduinoSerial.write('5')
    if error < 0:
        ArduinoSerial.write('11') #move to outside
        time.sleep(1)
        ArduinoSerial.write('5')


#def back(x,y,direc):
 #   x_wall =25
    # identify the wall position

  #  sense.x = x
   # sense.y = y
    #sense.direction = direc

    #if sense.x == x_wall: #when the wall is hit, go back
    #tell car to go back
#    ArduinoSerial.write('back')
    #go back to the last turning point
 #   if round(sense.x - turning_point[i][0]) == 0 and round(sense.y - turning_point[i][1]) == 0: #when the last turning point is reached
  #  ArduinoSerial.write('straight')  # tell motor to stop turning and go back to straight movement


#above is the function defining
#the following is the main operating functionimport json
ArduinoSerial = serial.Serial('/dev/cu.usbmodem14501',9600)
time.sleep(2) #wait for 2 secounds for the communication to get established
#print ArduinoSerial.readline() #read the serial data and print it as line
print ("Starting program")
yellow = []
red = []
i=0
ArduinoSerial.write('5')
print('has started motor')
while 1: #Do this forever
    with open('positions.json') as json_data:
        position = json.load(json_data)

    x = position["front_dis"]
    y = position["sid_dis"]
    yellow = position["yellow"]
    red = position["red"]
    safe_mine(yellow)
    dangerous_mine(red)

    #direc = position["current_dir"]

    #sdirec = position["current_dir"]


    i+= turn(x,i)


    #error control
    #if position["error_time"] >= 4:
     #   error_control(d["error"])



    #back(x,y,direc)

    #centre_position(x,y,direc)
    #if y_pos:
     #   yellow.append(y_pos)
    #if r_pos:
     #   red.append(r_pos)
    #centre_position(x,y,direc)

    #error_adjust(x,y,direc)





    time.sleep(1)





