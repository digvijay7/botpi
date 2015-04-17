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

def talker():
    signal.signal(signal.SIGINT, signal_handler)
    time.sleep(1)
    pub = rospy.Publisher('ultrasonic_data', Range, queue_size=10)
    rospy.init_node('ultrasonic_node', anonymous=True, disable_signals=False )
    rate = rospy.Rate(10) # 10hz
    ultrasonic_number = 1
    while not rospy.is_shutdown():
        data = Range()
        data.header.frame_id = ""+str(ultrasonic_number)
        data.header.stamp = rospy.Time.now()
        data.radiation_type = 0 # ULTRASONIC
        data.min_range = 5.0
        data.max_range = 200.0
        try :
            data.range = get_us_data_from(ultrasonic_number)
        except IOError:
            subprocess.call(['i2cdetect', '-y', '1'])
            data.range = 0
        #rospy.loginfo()
        pub.publish(data)
        ultrasonic_number += 1
        if ultrasonic_number == 6:
            ultrasonic_number = 1
        #rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
