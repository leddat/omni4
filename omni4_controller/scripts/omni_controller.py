#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import math
from math import sin, cos
from geometry_msgs.msg import Twist

SQRT_2 = math.sqrt(2)

def callback(msg):
    global vel_info
    vel_info = msg

def vel_control():
    rospy.init_node('omni_control')

    rospy.Subscriber('/cmd_vel', Twist, callback)

    pub_back = rospy.Publisher('/open_base/back_joint_velocity_controller/command', Float64, queue_size=1)
    pub_left = rospy.Publisher('/open_base/left_joint_velocity_controller/command', Float64, queue_size=1)
    pub_right = rospy.Publisher('/open_base/right_joint_velocity_controller/command', Float64, queue_size=1)
    pub_front = rospy.Publisher('/open_base/front_joint_velocity_controller/command', Float64, queue_size=1)

    rate=rospy.Rate(2)

    vel_back = Float64()
    vel_left = Float64()
    vel_right = Float64()
    vel_front = Float64()

    #distance form COM to wheel
    d = 0.4

    while not rospy.is_shutdown():
        vel_front.data = SQRT_2/2*(-vel_info.linear.x + vel_info.linear.y) + vel_info.angular.z*d

        vel_left.data = SQRT_2/2*(-vel_info.linear.x - vel_info.linear.y) + vel_info.angular.z*d

        vel_back.data = SQRT_2/2*( vel_info.linear.x - vel_info.linear.y) + vel_info.angular.z*d

        vel_right.data = SQRT_2/2*( vel_info.linear.x + vel_info.linear.y) + vel_info.angular.z*d

        #publish vel for each wheel
        pub_back.publish(vel_back)
        pub_left.publish(vel_left)
        pub_right.publish(vel_right)
        pub_front.publish(vel_front)
        
        rate.sleep()



if __name__ == '__main__':
    vel_info = Twist()
    vel_control()