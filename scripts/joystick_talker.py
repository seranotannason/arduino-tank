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
import math
import rospy
from std_msgs.msg import String, Int64
from sensor_msgs.msg import Joy

# This test function publishes random integers based on the current time

def callback(data):
    # We manipulate forward_value and rotate_value
    # to produce leftMotor and rightMotor values
    global left_motor
    global right_motor

    forward_value = int(data.axes[1] * 255)
    rotate_ratio = 1 - abs(data.axes[2])
    rospy.loginfo('rotate ratio: %s', rotate_ratio)
    if data.axes[2] > 0:
        # steer left, i.e. left_motor < right_motor
        right_motor = forward_value
        left_motor = right_motor * rotate_ratio
    else:
        # steer right, i.e. right_motor < left_motor
        left_motor = forward_value
        right_motor = left_motor * rotate_ratio
    
    rospy.loginfo('Left motor: %s', left_motor)
    rospy.loginfo('Right motor: %s', right_motor)
    
def listen_talker():
    rospy.init_node('joystick_talker', anonymous=True)
    pub = rospy.Publisher('values', Int64, queue_size=10)
    rate = rospy.Rate(5)
    rospy.Subscriber('joy', Joy, callback)
    
    while not rospy.is_shutdown():
        pub.publish(left_motor)
        pub.publish(right_motor)
        rate.sleep()

if __name__ == '__main__':
    try:
        left_motor = 0
        right_motor = 0
        listen_talker() 
        # this node will listen to joystick
        # and publish to listener (computer on rover)
    except rospy.ROSInterruptException:
        pass
