#!/usr/bin/python

import RPi.GPIO as GPIO
import time
import rospy
from sensor_msgs.msg import Range
import signal
import sys

FWD_THRESHOLD_DIST = 40 # in cm
BUG_FWD_DIST = 10
WALL_THRESHOLD_DIST = 30
WALL_DELTA = 20
DIR_SEP_DIST = 10 # Direction separation distance

FORWARD_TIME = 0.4
BUG_FORWARD_TIME = 0.3
BACKWARD_TIME = 0.2
TURN_TIME = 0.3
DIFF_DIST = 20

# BUG_MODE = 0 ==> Free, BUG_MODE = 1 ==> Obstacle circumnavigation

BUG_MODE = 0

MF = 1
MB = 2
TL = 3
TR = 4


H1 = 11
H2 = 9
H3 = 7
H4 = 8
PINS = [H1,H2,H3,H4]

US=[-1,-1,-1,-1,-1]

# WALL_SIDE = "left" ==> Circumnavigate such that wall remains on the left.
# WALL_SIDE = "right" ==> Circumnavigate such that wall remains on the right.
WALL_SIDE = "left"

# Ultrasonic ids / indices
ultrasonic_indices= {}
ultrasonic_indices["front"] = 0
ultrasonic_indices["front right"] = 1
ultrasonic_indices["front left"] = 2
ultrasonic_indices["left"] = 3
ultrasonic_indices["right"] = 4

def num(s):
    try:
        return int(s)
    except ValueError:
        return float(s)

def callback(data):
    us_num = num(data.header.frame_id) - 1
    if us_num >= 0 and us_num < 5:
        US[us_num] = data.range

def signal_handler(signal, frame):
    GPIO.cleanup()
    sys.exit(0)

def off(pin):
    GPIO.output(pin, False)

def on(pin):
    GPIO.output(pin, True)

def off_all():
    for x in PINS:
        off(x)

def move_generic(pin1, pin2, move_time):
    off_all()
    time.sleep(0.001)
    on(pin1)
    on(pin2)
    time.sleep(move_time)
    off_all()
   
def move_forward(move_time):
    move_generic(H1,H3,move_time)

def move_backward(move_time):
    move_generic(H2,H4,move_time)

def turn_left(turn_time):
    move_generic(H1,H4,turn_time)

def turn_right(turn_time):
    move_generic(H2,H3,turn_time)

def init_hbridge():
    for pin in PINS:
	GPIO.setup(pin,GPIO.OUT)

def get_wall_distance():
    return US[ultrasonic_indices[WALL_SIDE]]

def front_distance():
    return US[ultrasonic_indices["front"]]

def front_right_distance():
    return US[ultrasonic_indices["front right"]]

def front_left_distance():
    return US[ultrasonic_indices["front left"]]

def turn_towards_wall():
    rospy.loginfo("Turning towards the wall.")
    if WALL_SIDE == "left":
        turn_left(TURN_TIME)
    elif WALL_SIDE == "right":
        turn_right(TURN_TIME)

def turn_away_from_wall():
    rospy.loginfo("Turning away from the wall.")
    if WALL_SIDE == "left":
        turn_right(TURN_TIME)
    elif WALL_SIDE == "right":
        turn_left(TURN_TIME)


def decide_move(moves,last_moves):
    global MF, MB, TL, TR, DIFF_DIST
    if moves["mf"]:
        return MF
    fl = US[ultrasonic_indices["front left"]]
    f = US[ultrasonic_indices["front"]]
    fr = US[ultrasonic_indices["front right"]]
    l =  US[ultrasonic_indices["left"]]
    r =  US[ultrasonic_indices["right"]]
    if moves["tl"] and (fl - f > DIFF_DIST or (abs(fl - f) < DIFF_DIST and fr - f > DIFF_DIST)):
        return TL
    if moves["tr"] and (fr - f > DIFF_DIST or (abs(fr - f)  < DIFF_DIST and f - fl > DIFF_DIST)):
        return TR
    if moves["tl"]:
        return TL
    return TR

def log_us_data():
    data = ""
    fl = US[ultrasonic_indices["front left"]]
    f = US[ultrasonic_indices["front"]]
    fr = US[ultrasonic_indices["front right"]]
    l =  US[ultrasonic_indices["left"]]
    r =  US[ultrasonic_indices["right"]]
    data += ("fl:" + str(fl) + " ")
    data += ("f:"+str(f) + " ")
    data += ("fr:"+str(fr) + " ")
    data += ("l:"+str(l) + " ")
    data += ("r:"+str(r))
    rospy.loginfo(data)

def empty_space_front_left():
    fl = US[ultrasonic_indices["front left"]]
    return fl > BUG_FWD_DIST

def empty_space_front_right():
    fr = US[ultrasonic_indices["front right"]]
    return fr > BUG_FWD_DIST

def empty_space_front():
    f = US[ultrasonic_indices["front"]]
    return f > BUG_FWD_DIST

def empty_space_forward_bug():
    return empty_space_front_left() and empty_space_front_right() and empty_space_front()


def empty_space_forward():
    fl = US[ultrasonic_indices["front left"]]
    f = US[ultrasonic_indices["front"]]
    fr = US[ultrasonic_indices["front right"]]
    
    return f > FWD_THRESHOLD_DIST and fr > FWD_THRESHOLD_DIST and fl > FWD_THRESHOLD_DIST 



def main():
    signal.signal(signal.SIGINT, signal_handler)
    global US,MF,MB,TL,TR, ultrasonic_indices, BUG_MODE
    #Use BCM GPIO references
    #instead of physical pin numbers
    GPIO.setmode(GPIO.BCM)
    
    rospy.init_node('mover_1', anonymous=True,disable_signals=False)
    rospy.Subscriber("ultrasonic_data",Range,callback)

    init_hbridge()
    last_moves =[]
    observed_wall_once = False
    while(True):
        # In rospy spinning is not required as the callbacks
        # are executed in a separate thread
        # Wait for 0.5 seconds, to get updated readings 
        # for all ultrasonics
        
        time.sleep(1.2)
        
        log_us_data()
        
        if BUG_MODE == 0:
            if empty_space_forward():
                rospy.loginfo("Empty space forward!")
                move_forward(FORWARD_TIME)
            else:
                BUG_MODE = 1
                rospy.loginfo("Going into circumnavigating mode")

        else:
            if observed_wall_once == False:
                # Determine angle of approach to wall
                # -------------------------------
                #      ^    |  ^      |    ^
                #     /     |   \     |    |
                #    /      |    \    |    |
                #   /       |     \   |    |
                rospy.loginfo("Finding wall first time.")
                #if (front_left_distance() + DIR_SEP_DIST < empty_space_front()) and \
                #        (empty_space_front() + DIR_SEP_DIST < empty_space_front_right()):
                #    rospy.loginfo("Turning away from wall.")
                #    turn_away_from_wall()
                #elif (front_left_distance() > empty_space_front() + DIR_SEP_DIST) and \
                #        (empty_space_front() > empty_space_front_right() + DIR_SEP_DIST):
                #    rospy.loginfo("Turning towards from wall.")
                #    turn_towards

                # No need to determine
                # Until we get the wall on our left keep turning right
                wall_distance = get_wall_distance()
                if wall_distance < (WALL_THRESHOLD_DIST + WALL_DELTA):
                    rospy.loginfo("Found wall.")
                    observed_wall_once = True
                    continue

                turn_away_from_wall()
                if empty_space_forward_bug():
                    move_forward(BUG_FORWARD_TIME)

            else:
                wall_distance = get_wall_distance()
                if wall_distance < (WALL_THRESHOLD_DIST - WALL_DELTA):
                    rospy.loginfo("Wall distance less than threshold.")
                    turn_away_from_wall()
                
                elif wall_distance > (WALL_THRESHOLD_DIST + WALL_DELTA):
                    rospy.loginfo("Wall distance more than threshold")
                    turn_towards_wall() 
    
                elif empty_space_forward_bug():
                    rospy.loginfo("Wall distance in threshold range, empty space forward.")
                    move_forward(BUG_FORWARD_TIME)
                 
                else:
                    rospy.loginfo("Else!")
                    turn_away_from_wall()

                if empty_space_forward_bug():
                    move_forward(BUG_FORWARD_TIME)

if __name__ == "__main__":
    main()
