#!/usr/bin/env python2

#from cmotion.mround import *
#from cmotion.Q2 import *
import rospy
import rosbag
# from sensor_msgs.msg import PointCloud2
# from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf
import rospkg
# import subprocess
import math
import matplotlib.pyplot as plt
from matplotlib import use
use('TKAgg')
import numpy as np


class Map2Odom:
    def __init__(self):
        rospy.Subscriber("/odom", Odometry, self.map2odom, queue_size=10)
        self.path_publisher = rospy.Publisher("/path_taken", Path, queue_size=10)
        self.path = Path()

    def map2odom(self, msg):
        btf = tf.TransformBroadcaster()
        pose = PoseStamped()

        pose.pose = msg.pose.pose
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"

        btf.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "map", "odom")
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "map"
        self.path.poses.append(pose)
        self.path_publisher.publish(self.path)


def Q2():
    rospy.init_node('Path_taken_node')
    #rospack = rospkg.RosPack()
   # CMotionDir = rospack.get_path('cmotion') + '/Bags/'
   # player_proc = subprocess.Popen(['rosbag', 'play', 'CircularMotionBag.bag'], cwd=CMotionDir)
    pathPub = Map2Odom()
    rospy.spin()


def Q3():
    rospack = rospkg.RosPack()
    IMUPath = rospack.get_path('cmotion') + '/Bags/imu_odom.bag'
    IMUBag = rosbag.Bag(IMUPath)

    imu_acceleration = []
    imu_ang_vel = []
    for topic, msg, t in IMUBag.read_messages(topics=['/imu']):
        imu_acceleration.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        imu_ang_vel.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
    imu_acceleration = np.array(imu_acceleration)
    imu_ang_vel = np.array(imu_ang_vel)
    plt.subplot(121)
    pltax, = plt.plot(imu_acceleration[:, 0])
    pltay, = plt.plot(imu_acceleration[:, 1])
    # plt.plot(imu_acceleration[:][2])
    plt.title('IMU Linear Acceleration')
    plt.legend([pltax, pltay], ['ax', 'ay'])
    plt.subplot(122)
    # plt.plot(imu_ang_vel[:, 0])
    # plt.plot(imu_ang_vel[:, 1])
    pltyaw, = plt.plot(imu_ang_vel[:, 2])
    plt.title('IMU Angular Velocity')
    plt.legend([pltyaw], ['yaw'])
    plt.show()




if __name__ == "__main__":
    try:
        Q2()
    except rospy.ROSInterruptException:
        pass
