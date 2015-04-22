#!/usr/bin/python

import RPi.GPIO as GPIO
import time
import rospy
from sensor_msgs.msg import Range
import signal
import sys

THRESHOLD_DIST = 41 # in cm
FORWARD_TIME = 0.2
BACKWARD_TIME = 0.2
TURN_TIME = 0.3
SETTLE_TIME = 0.002
TRIGGER_TIME = 0.001
DIFF_DIST = 20

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
    #rospy.loginfo(str(us_num) + " " +str(data.range))
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
    #[off(x) for x in PINS]

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

def avoid_obstacles(moves):
    global ultrasonic_indices
    global US
    global THRESHOLD_DIST
    # First, avoid obstacles
    fl = US[ultrasonic_indices["front left"]]
    f = US[ultrasonic_indices["front"]]
    fr = US[ultrasonic_indices["front right"]]
    l =  US[ultrasonic_indices["left"]]
    r =  US[ultrasonic_indices["right"]]
    if  fl < THRESHOLD_DIST or f < THRESHOLD_DIST or fr < THRESHOLD_DIST:
        moves["mf"] = False
    if  l < THRESHOLD_DIST :
        moves["tl"] = False
    if r < THRESHOLD_DIST :
        moves["tr"] = False
    return moves

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



def main():
    signal.signal(signal.SIGINT, signal_handler)
    global US,MF,MB,TL,TR, ultrasonic_indices
    #Use BCM GPIO references
    #instead of physical pin numbers
    GPIO.setmode(GPIO.BCM)
    
    rospy.init_node('mover_1', anonymous=True,disable_signals=False)
    rospy.Subscriber("ultrasonic_data",Range,callback)

    init_hbridge()
#    while(True):
#        move_forward(1)
#        time.sleep(1)
#        move_backward(1)
#        time.sleep(1)
#        turn_left(1)
#        time.sleep(1)
#        turn_right(1)
#        time.sleep(1)
    last_moves =[]
    while(True):
        # In rospy spinning is not required as the callbacks
        # are executed in a separate thread
        # Wait for 0.5 seconds, to get updated readings 
        # for all ultrasonics
        time.sleep(1.2)
        # Initialize decicions
        # m = move, f = forward, b = backwards
        # t = turn, l = left, r = right
        moves = {}
        moves["mf"] = True
        moves["mb"] = False 
        moves["tl"] = True
        moves["tr"] = True
        
        moves = avoid_obstacles(moves)
        move = decide_move(moves,last_moves)
        last_moves.append(move)

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
        if move == MF:
            move_forward(FORWARD_TIME)
            rospy.loginfo("Moving forward")
        elif move == MB:
            move_backward(BACKWARD_TIME)
            rospy.loginfo("Moving Backward")
        elif move == TL:
            turn_left(TURN_TIME)
            rospy.loginfo("Turning Left")
        elif move == TR:
            turn_right(TURN_TIME)
            rospy.loginfo("Turning Right")
        
        
#        if min_dist < THRESHOLD_DIST:
            # Turn - Direction?
#            turn_left(TURN_TIME)
#
#        else:
#            move_forward(FORWARD_TIME) # Should be function of min_dist


if __name__ == "__main__":
    main()
