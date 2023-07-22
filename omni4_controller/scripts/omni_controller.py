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

    pub_back_right = rospy.Publisher('/open_base/back_right_joint_velocity_controller/command', Float64, queue_size=1)
    pub_back_left = rospy.Publisher('/open_base/back_left_joint_velocity_controller/command', Float64, queue_size=1)
    pub_front_right = rospy.Publisher('/open_base/front_right_joint_velocity_controller/command', Float64, queue_size=1)
    pub_front_left = rospy.Publisher('/open_base/front_left_joint_velocity_controller/command', Float64, queue_size=1)

    rate=rospy.Rate(2)

    w_back_right = Float64()
    w_back_left = Float64()
    w_front_right = Float64()
    w_front_left = Float64()

    #distance form COM to wheel
    d = SQRT_2*0.03
    #wheel radious
    r = 0.01905

    while not rospy.is_shutdown():

        w_back_right.data = (SQRT_2/2*(-vel_info.linear.x + vel_info.linear.y) + vel_info.angular.z*d)/r 
        #rad/s
        w_front_right.data = (SQRT_2/2*(-vel_info.linear.x - vel_info.linear.y) + vel_info.angular.z*d)/r

        w_front_left.data = (SQRT_2/2*( vel_info.linear.x - vel_info.linear.y) + vel_info.angular.z*d)/r

        w_back_left.data = (SQRT_2/2*( vel_info.linear.x + vel_info.linear.y) + vel_info.angular.z*d)/r

        #publish vel for each wheel
        pub_back_right.publish(w_back_right)
        pub_back_left.publish(w_back_left)
        pub_front_right.publish(w_front_right)
        pub_front_left.publish(w_front_left)
        
        rate.sleep()



if __name__ == '__main__':
    vel_info = Twist()
    vel_control()