import serial #Serial imported for Serial communication
import time #Required to use delay functions

turning_point = {} #create a list of turning points
set_direction = {} #create a list of designated turning directions

def turn():
    # get the live position and direction
    sense.x =
    sense.y =
    sense.direction =
    if round(sense.x - x.turning_point[i]) == 0 and round(sense.y - y.turning_point[i]) == 0: #when turning point is reached
        i = 0 #i is counter
        #tell left motor to turn back
        print("left motor back")
        #tell right motor to turn forward
        print("right motor forward")

        if round(sense.direction - set_direction[i]) == 0:  #when direction has changed
            print("stop turning")                # tell motor to stop turning and go back to straight movement

        i += 1    #counter plus 1, prepare for the next turning point

def back():
    # get the live position and direction
    sense.x =
    sense.y =
    sense.direction =
    if sense.x == x.wall or sense.y == y.wall: #when the wall is hit, go back
        # tell left motor to turn back
        print("left motor back")
        # tell right motor to turn back
        print("right motor back")
        #go back to the last turning point
        if round(sense.x - x.turning_point[i]) == 0 and round(sense.y - y.turning_point[i]) == 0: #when the last turning point is reached
            print("stop turning")  # tell motor to stop turning and go back to straight movement


def safe_mine():
    # create the list for the positions of safe mines
    safe_mine_position = {}
    # get the live position and direction
    sense.x =
    sense.y =
    sense.direction =
    #when safe mine is detected
    if safe_mine_is_detected == 1:
        #record the position of the safe mine
        safe_mine_position.append({sense.x, sense.y})
        #stay for 3 seconds
        time.sleep(3)
        #tell the car to push out the mine
        print("push out the mine")
        #don't turn the direction this time
        pass turn()


def dangerous_mine():
    #create the list for the positions of dangerous mines
    dangerious_mine_position = {}
    # get the live position and direction    sense.x =
    sense.y =
    sense.direction =
    # when dangerous mine is detected
    if dangerous_mine_is_detected == 1:
        #record the position of the dangerous mine
        dangerious_mine_position.append({sense.x, sense.y})
        # stay for 3 seconds
        time.sleep(3)


def centre_position():
    #give a centre position
    x.centre_point =
    y.centre_point =
    # get the live position and direction
    sense.x =
    sense.y =
    sense.direction =

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


