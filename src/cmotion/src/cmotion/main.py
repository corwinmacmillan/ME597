#!/usr/bin/env python2

#from cmotion.mround import *
#from cmotion.Q2 import *
import rospy
# import rosbag
# from sensor_msgs.msg import PointCloud2
# from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf
import rospkg
import subprocess
import math


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
        pose.header.frame_id = "odom"

        btf.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "map", "odom")
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "odom"
        self.path.poses.append(pose)
        od_msg = msg
        pose = od_msg.pose.pose.position
        orientation = od_msg.pose.pose.orientation
        roll, pitch, yaw = self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        self.path_publisher.publish(self.path)

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

def Q2():
    rospack = rospkg.RosPack()
    CMotionDir = rospack.get_path('cmotion') + '/Bags/'
    player_proc = subprocess.Popen(['rosbag', 'play', 'CircularMotionBag.bag'], cwd=CMotionDir)
    pathPub = Map2Odom()

    # pointCloudBag = rosbag.Bag(CMotionPath)
    # #pathBag = rosbag.Bag('../../../../2022-03-09-17-40-02.bag')
    # # IMUBag = rosbag.Bag('../../../../imu_odom.bag')
    # rate = rospy.Rate(10)
    # pointPub = rospy.Publisher('/camera/depth_registered/points', PointCloud2, queue_size=0)
    # imagePub = rospy.Publisher('/camera/rgb/image_color', Image, queue_size=0)
    # pathPub = rospy.Publisher('/path', Path, queue_size=10)
    #
    # path = Path()
    # pose = PoseStamped()
    # for topics, msgs, ts in zip(pointCloudBag.read_messages(topics=['/camera/depth_registered/points']),
    #                             pointCloudBag.read_messages(topics=['/camera/rgb/image_color']),
    #                             pointCloudBag.read_messages(topics=['/odom'])):
    #     pointPub.publish(msgs[0])
    #     imagePub.publish(msgs[1])
    #
    #     btf = tf.TransformBroadcaster()
    #
    #     pose.pose = msgs[2].pose.pose
    #     pose.header.stamp = ts[2]
    #     pose.header.frame_id = "odom"
    #     btf.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "map", "odom")
    #     path.header.stamp = rospy.Time.now()
    #     path.header.frame_id = "odom"
    #     path.poses.append(pose)
    #     pathPub.publish(path)
    #
    #     rate.sleep()


if __name__ == "__main__":
    try:
        Q2()
    except rospy.ROSInterruptException:
        pass
