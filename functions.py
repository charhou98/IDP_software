import serial #Serial imported for Serial communication
import time #Required to use delay functions

turning_point = {} #create a list of turning points
set_direction = {} #create a list of designated turning directions


def turn(x,y,direc):
    # get the live position and direction
    sense.x = x
    sense.y = y
    sense.direction = direc
    if round(sense.x - x.turning_point[i]) == 0 and round(sense.y - y.turning_point[i]) == 0: #when turning point is reached
        i = 0 #i is counter
        #tell left motor to turn back
        print("left motor back")
        #tell right motor to turn forward
        print("right motor forward")

        if round(sense.direction - set_direction[i]) == 0:  #when direction has changed
            print("stop turning")                # tell motor to stop turning and go back to straight movement

        i += 1    #counter plus 1, prepare for the next turning point

def back(x,y,direc):
    # get the live position and direction
    sense.x = x
    sense.y = y
    sense.direction = direct
    if sense.x == x.wall or sense.y == y.wall: #when the wall is hit, go back
        # tell left motor to turn back
        print("left motor back")
        # tell right motor to turn back
        print("right motor back")
        #go back to the last turning point
        if round(sense.x - x.turning_point[i]) == 0 and round(sense.y - y.turning_point[i]) == 0: #when the last turning point is reached
            print("stop turning")  # tell motor to stop turning and go back to straight movement


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
    #give a centre position
    x.centre_point =
    y.centre_point =
    # get the live position and direction
    sense.x = x
    sense.y = y
    sense.direction = direc

    #when arrive at centre point, go back to original point
    if round(sense.x - x.centre_point) == 0 and round(sense.y - y.centre_point) == 0:
        # turn the car and let it point to the original point
        # tell left motor to turn back
        print("left motor back")
        # tell right motor to turn forward
        print("right motor forward")
        # when home direction has reached
        if round(sense.direction - home_direction) == 0:
            # tell motor to stop turning and go back to straight movement
            print("stop turning")
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

 
ArduinoSerial = serial.Serial('COM4',9600) #Create Serial port object called arduinoSerialData
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
    #back(x,y,direc)
    y_pos = safe_mine(x,y,direc)
    r_pos = dangerous_mine(x,y,direc)
    centre_position(x,y,direc)
    if y_pos:
        yellow.append(y_pos)
    if r_pos:
        red.append(r_pos)
    centre_position(x,y,direc)

    error_adjust(x,y,direc)





    time.sleep(1)


