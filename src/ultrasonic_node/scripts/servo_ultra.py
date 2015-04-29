#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import smbus
import time
import signal
import sys
import subprocess
import tf
import math
import wiringpi2 as wiringpi
from time import sleep

from std_msgs.msg import String
from sensor_msgs.msg import Range

address = 0x04
flag = False
bus = smbus.SMBus(1)

def get_us_data_from(value):
    global address
    global flag
    global bus
    bus.write_byte(address, value)
    flag = True
    time.sleep(0.1)
    number = bus.read_byte(address)
    flag = False
    return number

def signal_handler(signal, frame):
    global flag
    global bus
    print('You pressed Ctrl+C!')
    if flag :
        bus.read_byte(address)
    sys.exit(0)

def broadcast_transforms():
    rospy.loginfo("Broadcasting transforms")
    br = tf.TransformBroadcaster()
    rospy.loginfo(str(rospy.Time.now()))

    # Front sensor
    br.sendTransform((0.12, 0.0, 0.0),tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0), rospy.Time.now(), "1", "base_link")
    
    # Front right sensor - 20 degrees clockwise rotation
    br.sendTransform((0.13, -0.06, 0.0),tf.transformations.quaternion_about_axis(-0.34906585, (0, 0, 1)), rospy.Time.now(), "2", "base_link")

    # Front left sensor - 5 degrees counter-clockwise rotation
    br.sendTransform((0.13, 0.06, 0.0),tf.transformations.quaternion_about_axis(0.0872664626,(0, 0, 1)), rospy.Time.now(), "3", "base_link")

    # Left sensor - 90 degrees counter-clockwise rotation
    br.sendTransform((0.0, 0.086, 0.0),tf.transformations.quaternion_about_axis(1.57079633, (0, 0, 1)), rospy.Time.now(), "4", "base_link")

    # Right sensor - 90 degrees clockwise rotation
    br.sendTransform((0.0, -0.086, 0.0),tf.transformations.quaternion_about_axis(-1.57079633, (0, 0, 1)), rospy.Time.now(), "5", "base_link")


def setup_servo():
    wiringpi.wiringPiSetupGpio()
    wiringpi.pinMode(18,2)      # hardware pwm only works on GPIO port 18  
    wiringpi.pwmSetMode(0)
    wiringpi.pwmSetClock(375)
    wiringpi.pwmSetRange(1024)
    wiringpi.pwmWrite(18,0)

def talker():
    signal.signal(signal.SIGINT, signal_handler)
    time.sleep(1)
    setup_servo()
    pub = rospy.Publisher('servo_ulta', Range, queue_size=10)
    rospy.init_node('servo_ultra', anonymous=True, disable_signals=False )
    rate = rospy.Rate(10) # 10hz
    ultrasonic_number = 1
    
    pause_time = 0.1
    delay_ultrasonic = 0.11
    STEP_ANGLE = 10

    CURR_ANGLE = (90.0 * math.pi) / 180.0

    mindt = 0.65
    maxdt = 2.48
    dt = [mindt, maxdt]
    extremes = [int(x * 1024.0 / 20.0) for x in dt]

    dt_range = extremes[1] - extremes[0]
    dt_step = int((dt_range / 180.0) * STEP_ANGLE)

    br = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        #broadcast_transforms()

        # Move servo to counter-clockwise extreme.
        wiringpi.pwmWrite(18, extremes[1])
        sleep(0.2)
        CURR_ANGLE = (90.0 * math.pi) / 180.0

        for i in range(extremes[1],extremes[0],-dt_step):         # 1025 because it stops at 1024
            wiringpi.pwmWrite(18,i)
            sleep(pause_time)
            br.sendTransform((0.15, 0.0, 0.0),tf.transformations.quaternion_about_axis(CURR_ANGLE, (0, 0, 1)), rospy.Time.now(), "1", "base_link")
            data = Range()
            data.header.frame_id = "1"
            data.header.stamp = rospy.Time.now()
            data.radiation_type = 0
            data.min_range = 0.05 
            data.max_range = 2.0
            data.field_of_view = 0.164
            try :
                data.range = float(get_us_data_from(ultrasonic_number)) /100.0
            except IOError:
                subprocess.call(['i2cdetect', '-y', '1'])
                data.range = 0.0
            pub.publish(data)
            CURR_ANGLE -= (STEP_ANGLE * math.pi) / 180.0

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
