#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
# from nav_msgs import Odometry

global_msg = None
pi_2 = 1.5708

def callback(msg):
    global_msg = msg 

def quat2yaw(q):
    return atan(2 * (q.x * q.y + q.w * q.z)/(q.w ** 2 + q.x ** 2 - q.y ** 2 - q.z**2))
    
def mround():
    rospy.init_node("circular_motion")
    pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=0)
    # sub = rospy.Subscribe("/odom/", Odometry, callback)
    rate = rospy.Rate(10)
    vel = Twist()
    vel.angular.z = 1
    vel.linear.x = 0.25

    while not rospy.is_shutdown():
        pub.publish(vel)
        rate.sleep()
'''
    for i in range(0,4):
        initial_position = global_msg.position
        distance = 0
       
        while not rospy.is_shutdown() and distance < 1:
            pub.publish(vel)
            dx = initial_position.x - global_msg.position.x
            dy = initial_position.y - global_msg.position.y
            distance = sqrt(dx**2 + dy**2)
        vel.linear.x = 0
        vel.angular.z = 1
        yaw = quat2yaw(global_msg.pose.pose.orientation)
        while not rospy.is_shutdown() and yaw < pi_2:
            pub.publish(vel)
        vel.linear.x = 0.5
        '''
