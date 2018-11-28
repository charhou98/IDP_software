import serial #Serial imported for Serial communication
import time #Required to use delay functions
#import numpy as np
import json

def turn(x,i):
    print i

    turning_point = [30, 30, 30, 60,60,60,90,90,90,120,120,30,30]
    #create a list of turning points


    # get the live position and direction

    if abs(x - turning_point[i]) <= 5 and i <12:#when turning point is reached

        print(turning_point[i])
        print (x)#i is counter
        print(i)

        ArduinoSerial.write('6')
        #tell right motor to turn forward
        time.sleep(3)  # wait for 3 seconds for the car to complete turning

        ArduinoSerial.write('5')
        return 1# tell motor to stop turning and go back to straight movement
         # counter plus 1, prepare for the next turning point
    elif i == 12:
        ArduinoSerial.write('7')
    return 0

def checkit(check):
    if check == 1:
        print 'need to check'
        ArduinoSerial.write('7')
        time.sleep(0.5)
        ArduinoSerial.write('3')
        time.sleep(2)


        return 1
    return 0



def safe_mine(yellow,x,y):

    if yellow == 1:
        print 'yellow detected'
        ArduinoSerial.write('5')
        time.sleep(2)
        return (x,y)
    return 0



def dangerous_mine(red,x,y):
    if red == 1:
        print 'red detected' # stop
        ArduinoSerial.write('5')
        time.sleep(2)
        return (x,y)
    return 0
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
    time.sleep(second)

    ArduinoSerial.write('5')




ArduinoSerial = serial.Serial('/dev/cu.usbmodem14401',9600)
time.sleep(2) #wait for 2 secounds for the communication to get established
#print ArduinoSerial.readline() #read the serial data and print it as line
print ("Starting program")
yellowco = []
redco = []
i=0
print i
position = {"error_time": 0, "p_sid_dis": 35.0, "yellow": 0, "front_dis": 32.0, "p_error": 0.0, "red": 0, "error": 0.0, "sid_dis": 35.0,"checklist":[], "check":0}
#ArduinoSerial.write('6')
#ime.sleep(3)
ArduinoSerial.write('5')
print('has started motor')
while 1: #Do this forever
    for k in range (5):
        try:
            with open("positions.json") as f:
                position = json.load(f)
        except:
            pass



    colorlist = []
    checklist = []
    x = position["front_dis"]
    y = position["sid_dis"]
    print (x,y)
    checklist = position["checklist"]

    print checklist

    yellow = 0
    red = 0
    check = 0
    if 1 in checklist:
        if ((checklist[0] == 1 or checklist[7] == 1) and y >= 160) or i == 0:
            check = 0
        else:
            check = 1
    print i

    print check

    valicheck = checkit(check)

    if valicheck:
        for j in range (5):
            try:
                with open("positions.json") as f:
                    positioncolor = json.load(f)
            except:
                pass
        colorlist = positioncolor["colorlist"]
        print colorlist
        if 2 in colorlist:
            yellow = 1
        elif 3 in colorlist:
            red = 1


    print red
    print yellow

    if yellow ==0 and red == 0:
        ArduinoSerial.write('5')




    

    yellowdata = safe_mine(yellow,x,y)
    reddata = dangerous_mine(red,x,y)

    if yellowdata != 0:
        print yellowdata

        if (i %4) == 0:
            yellowdata = (yellowdata[0],243-yellowdata[1])

        elif (i % 4) == 1:
            yellowx = yellowdata[0]
            yellowy = yellowdata[1]
            yellowdata == (yellowy,243-yellowx)
        elif (i %4) == 2:
            pass
        elif (i % 4) == 3:
            yellowdata = (yellowdata[1],yellowdata[0])


        yellowco.append(yellowdata)
        print yellowco

    if reddata!= 0:
        if (i %4) == 0:
            reddata[1] == 243-reddata[1]

        elif (i % 4) == 1:

            reddata = (reddata[1],243-reddata[0])
        elif (i %4) == 2:
            pass
        elif (i % 4) == 3:
            reddata = (reddata[1],reddata[0])
        redco.append(reddata)
        print redco





    i+= turn(x,i)

    time.sleep(0.1)



    #error control
    if int(position["error_time"]) >= 2:
        error_control(position["error"])

 

    





