import serial #Serial imported for Serial communication
import time #Required to use delay functions
import numpy as np



x0 = 30
y0 = 35
dx = 20
dy = 20



def turn(x,y,direc):

    x0 = 30
    y0 = 35
    dx = 20
    dy = 20

    turning_point = list({(x0, y0), (x0, y0), (x0, y0), (x0 + dx, y0),
                 (x0 + dx, y0 + dy), (x0 + dx, y0 + dy), (x0 + dx, y0 + dy), (x0 + 2*dx, y0 + dy),
                 (x0 + 2*dx, y0 + 2*dy), (x0 + 2*dx, y0 + 2*dy), (x0 + 2*dx, y0 + 2*dy), (x0 + 3*dx, y0 + 2*dy),
                 (x0 + 3*dx, y0 + 3*dy), (x0 + 3*dx, y0 + 3*dy), (x0 + 3*dx, y0 + 3*dy), (x0 + 4*dx, y0 + 3*dy),
                 (x0 + 4*dx, y0 + 4*dy), (x0 + 4*dx, y0 + 4*dy), (x0 + 4*dx, y0 + 4*dy), (x0 + 5*dx, y0 + 4*dy),
                 (x0 + 5*dx, y0 + 4*dy), (x0 + 5*dx, y0 + 4*dy), (x0 + 5*dx, y0 + 4*dy), (x0 + 6*dx, y0 + 5*dy),
                 (x0 + 6*dx, y0 + 5*dy), (x0 + 6*dx, y0 + 5*dy), (x0 + 6*dx, y0 + 5*dy), (x0 + 7*dx, y0 + 6*dy),
                 (x0 + 7*dx, y0 + 6*dy), (x0 + 7*dx, y0 + 6*dy), (x0 + 7*dx, y0 + 6*dy), (x0 + 8*dx, y0 + 7*dy),
                 (x0 + 8*dx, y0 + 7*dy), (x0 + 8*dx, y0 + 7*dy), (x0 + 8*dx, y0 + 7*dy), (x0 + 9*dx, y0 + 8*dy),
                 (x0 + 9*dx, y0 + 8*dy), (x0 + 9*dx, y0 + 8*dy), (x0 + 9*dx, y0 + 8*dy), (x0 + 10*dx, y0 + 9*dy)})
    #create a list of turning points

    set_direction = list({1.52, 5.96, 4.43, 3.20, 1.52, 5.96, 4.43, 3.20, 1.52, 5.96, 4.43, 3.20, 1.52, 5.96, 4.43, 3.20,
                 1.52, 5.96, 4.43, 3.20, 1.52, 5.96, 4.43, 3.20, 1.52, 5.96, 4.43, 3.20, 1.52, 5.96, 4.43, 3.20,
                 1.52, 5.96, 4.43, 3.20, 1.52, 5.96, 4.43, 3.20, 1.52, 5.96, 4.43, 3.20, 1.52, 5.96, 4.43, 3.20,
                 1.52, 5.96, 4.43, 3.20, 1.52, 5.96, 4.43, 3.20, 1.52, 5.96, 4.43, 3.20, 1.52, 5.96, 4.43, 3.20,
                 1.52, 5.96, 4.43, 3.20, 1.52, 5.96, 4.43, 3.20, 1.52, 5.96, 4.43, 3.20, 1.52, 5.96, 4.43, 3.20,
                 1.52, 5.96, 4.43, 3.20, 1.52, 5.96, 4.43, 3.20, 1.52, 5.96, 4.43, 3.20, 1.52, 5.96, 4.43, 3.20})
    # create a list of designated turning directions

    # get the live position and direction
    sense.x = x
    sense.y = y
    sense.direction = direc
    if round(sense.x - turning_point[i][0]) == 0 and round(sense.y - turning_point[i][1]) == 0: #when turning point is reached
        i = 0 #i is counter

        #tell left motor to turn back
        ArduinoSerial.write('turn')
        #tell right motor to turn forward


        if round(sense.direction - set_direction[i]) == 0:  #when direction has changed
            ArduinoSerial.write('straight')               # tell motor to stop turning and go back to straight movement


        def back(x,y,direc):
            x_wall =25
            # identify the wall position

            sense.x = x
            sense.y = y
            sense.direction = direc

            if sense.x == x_wall: #when the wall is hit, go back
            #tell car to go back
            ArduinoSerial.write('back')
            #go back to the last turning point
            if round(sense.x - turning_point[i][0]) == 0 and round(sense.y - turning_point[i][1]) == 0: #when the last turning point is reached
                ArduinoSerial.write('straight')  # tell motor to stop turning and go back to straight movement

        i += 1  # counter plus 1, prepare for the next turning point

def safe_mine(x,y,direc):
    # create the list for the positions of safe mines

    # get the live position and direction
    sense.x = x
    sense.y = y
    sense.direction = direc

    #when safe mine is detected
    if safe_mine_is_detected == 1:
        print("turn on yellow LED")
        #record the position of the safe mine
        #safe_mine_position.append({sense.x, sense.y})
        #stay for 3 seconds
        time.sleep(3)
        #tell the car to push out the mine
        print("push out the mine")
        ArduinoSerial.write('prepare_push') 
        #don't turn the direction this time
        pass turn()
        return (x,y)


def dangerous_mine(x,y,direc):
    #create the list for the positions of dangerous mines

    # get the live position and direction   
    sense.x = x
    sense.y = y
    sense.direction = direc
    # when dangerous mine is detected
    if dangerous_mine_is_detected == 1:
        print ("turn on red LED")
        #record the position of the dangerous mine
        dangerious_mine_position.append({sense.x, sense.y})
        # stay for 3 seconds
        time.sleep(3)
        return (x,y)


def centre_position(x,y,direc):
    x_centre_point =
    y_centre_point =
    # give a centre position

    # get the live position and direction
    sense.x = x
    sense.y = y
    sense.direction = direc

    #when arrive at centre point, go back to original point
    if round(sense.x - x_centre_point) == 0 and round(sense.y - y_centre_point) == 0:
        # turn the car and let it point to the original point
        ArduinoSerial.write('turn')
        # when home direction has reached
        if round(sense.direction - home_direction) == 0:
            # tell motor to stop turning and go back to straight movement
            ArduinoSerial.write('straight')
            # when home is arrived, stop moving permanently
            if round(sense.x) == 0 and round(sense.y) == 0:
                break

def error_adjust(x,y,direc,request_direc):
    if request_direc-direc>= 10:
        print ("turn to one side")
    elif request_direc-direc <= -10:
        print ("turn to the other side")_



#above is the function defining
#the following is the main operating function
import json

 
ArduinoSerial = serial.Serial('/dev/cu.usbmodem14301',9600) #Create Serial port object called arduinoSerialData
time.sleep(2) #wait for 2 secounds for the communication to get established
print ArduinoSerial.readline() #read the serial data and print it as line
print ("Starting program")
yellow = []
red = []
 
while 1: #Do this forever
    with open('positions.json') as json_data:
        position = json_data.load()

    x = position["current_x"]
    y = position["current_y"]
    direc = position["current_dir"]
    turn(x,y,direc)
    back(x,y,direc)
    y_pos = safe_mine(x,y,direc)
    r_pos = dangerous_mine(x,y,direc)
    #centre_position(x,y,direc)
    if y_pos:
        yellow.append(y_pos)
    if r_pos:
        red.append(r_pos)
    centre_position(x,y,direc)

    error_adjust(x,y,direc)





    time.sleep(1)





