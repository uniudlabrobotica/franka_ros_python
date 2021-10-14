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
import std_msgs.msg
from franka_gripper.msg import MoveActionGoal
from franka_msgs.msg import FrankaState

from time import sleep

def franka_state_callback(msg):
    if franka_state_callback.counter == 0:
        print(msg.q)

    franka_state_callback.counter += 1
    
franka_state_callback.counter = 0

def print_state():
    state_sub = rospy.Subscriber("franka_state_controller/franka_states",
                                 FrankaState, franka_state_callback)
    sleep(1)


def move_the_gripper_real():
    pub = rospy.Publisher('franka_gripper/move/goal', MoveActionGoal, queue_size=10)
    pub_2 = rospy.Publisher('panda/franka_gripper/move/goal', MoveActionGoal, queue_size=10)
    even=True
    rate = rospy.Rate(.3) # 0.3hz
    while not rospy.is_shutdown():
        command=MoveActionGoal()
        command.goal.width=.08 if even else 0
        command.goal.speed=.1
        even=not even
        rospy.loginfo(command)
        pub.publish(command)
        pub_2.publish(command)
        rate.sleep()


def talker():

    rospy.init_node('talker', anonymous=True)
        
    print("Press enter to get the state")
    raw_input()
    print_state()

    print("Press enter to move the gripper")
    raw_input()
    move_the_gripper_real()

if __name__ == '__main__':
    try:
        talker()        
    except rospy.ROSInterruptException:
        pass
  
