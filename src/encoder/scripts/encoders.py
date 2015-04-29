#!/usr/bin/python

# Node for the wheel encoders.
# Listens for clicks from the wheel encoders and publishes the count of clicks (1 each for L and R) since the start.
# Clicks are added/subtracted based on the direction of motion.

import rospy
import signal
import sys

from geometry_msgs.msg import Vector3
import RPi.GPIO as GPIO

LEFT_CLICKS = 0
RIGHT_CLICKS = 0

LEFT_ENCODER_PIN = 23
RIGHT_ENCODER_PIN = 24


# 0-UP, 1-RIGHT, 2-DOWN, 3-LEFT
MOTION_DIRECTION = 0

def left_click(pin):
    global LEFT_CLICKS
    if (MOTION_DIRECTION == 0 or MOTION_DIRECTION == 1):
        LEFT_CLICKS += 1
    else:
        LEFT_CLICKS -= 1

def right_click(pin):
    global RIGHT_CLICKS
    if (MOTION_DIRECTION == 0 or MOTION_DIRECTION == 3):
       RIGHT_CLICKS += 1
    else:
        RIGHT_CLICKS -= 1

def set_motion_direction(data):
    global MOTION_DIRECTION
    if (data.data == 'UP'):
        MOTION_DIRECTION = 0
    elif (data.data == 'RIGHT'):
        MOTION_DIRECTION = 1
    elif (data.data == 'DOWN'):
        MOTION_DIRECTION = 2
    elif (data.data == 'LEFT'):
        MOTION_DIRECTION = 3


def create_msg():
    global LEFT_CLICKS
    global RIGHT_CLICKS
    msg = Vector3()
    msg.x = int(LEFT_CLICKS)
    msg.y = int(RIGHT_CLICKS)
    msg.z = 0
    return msg

def signal_handler(signal, fram):
    GPIO.cleanup()
    sys.exit(0)

def talker():
    signal.signal(signal.SIGINT, signal_handler)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LEFT_ENCODER_PIN, GPIO.IN)
    GPIO.setup(RIGHT_ENCODER_PIN, GPIO.IN)
    pub = rospy.Publisher('encoder_data', Vector3, queue_size=10)
    rospy.init_node('encoders_node', anonymous=True, disable_signals=False )
    rate = rospy.Rate(3) # 1hz

    GPIO.add_event_detect(LEFT_ENCODER_PIN, GPIO.RISING, bouncetime=50)
    GPIO.add_event_callback(LEFT_ENCODER_PIN, left_click)

    GPIO.add_event_detect(RIGHT_ENCODER_PIN, GPIO.RISING, bouncetime=50)
    GPIO.add_event_callback(RIGHT_ENCODER_PIN, right_click)

    rospy.Subscriber("motion_direction", String, set_motion_direction)

    while not rospy.is_shutdown():
        encoder_msg = create_msg()
        pub.publish(encoder_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
