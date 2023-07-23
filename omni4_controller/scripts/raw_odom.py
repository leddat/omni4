#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import math
from math import sin, cos
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import  quaternion_from_euler
import tf

SQRT_2 = math.sqrt(2)

def callback_back_right(msg):
    global w_back_right
    w_back_right = msg

def callback_back_left(msg):
    global w_back_left
    w_back_left = msg

def callback_front_right(msg):
    global w_front_right
    w_front_right = msg

def callback_front_left(msg):
    global w_front_left
    w_front_left = msg



def raw_odom_calculate():
    rospy.init_node('omni_base')

    rospy.Subscriber('/open_base/back_right_joint_velocity_controller/command',Float64, callback_back_right)
    rospy.Subscriber('/open_base/back_left_joint_velocity_controller/command',Float64, callback_back_left)
    rospy.Subscriber('/open_base/front_right_joint_velocity_controller/command',Float64, callback_front_right)
    rospy.Subscriber('/open_base/front_left_joint_velocity_controller/command',Float64, callback_front_left)

    odom_pub = rospy.Publisher("/raw_odom",Odometry,queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()
    odom_msg = Odometry()

    #distance form COM to wheel
    d = SQRT_2*0.03
    #wheel radious
    r = 0.01905
    
    x = 0.0
    y = 0.0
    th = 0.0
    vx = 0.0
    vy = 0.0
    angular = 0.0


    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        
        vx = (SQRT_2*(-w_back_right.data - w_front_right.data + w_front_left.data + w_back_left.data))*r/4
        vy = (SQRT_2*(w_back_right.data - w_front_right.data - w_front_left.data + w_back_left.data))*r/4
        angular = (w_back_right.data + w_front_right.data + w_front_left.data + w_back_left.data)*r/(4*d)
        
        current_time = rospy.Time.now()

        dt = (current_time - last_time).to_sec()
        dx = (vx * cos(th) - vy * sin(th)) * dt
        dy = (vx * sin(th) + vy * cos(th)) * dt
        dth = angular * dt

        x += dx
        y += dy
        th += dth

        odom_quat = quaternion_from_euler (0,0,th)

        odom_broadcaster.sendTransform(
            (x, y, 0.0),
            odom_quat,
            current_time,
            "base_footprint",
            "odom"
        )

        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        odom_msg.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
        odom_msg.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, angular))

        odom_pub.publish(odom_msg)

        rate.sleep()



if __name__ == '__main__':
    w_back_right = Float64()
    w_back_left = Float64()
    w_front_right = Float64()
    w_front_left = Float64()
    print("Publish raw_odom")
    raw_odom_calculate()