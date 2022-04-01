#!/usr/bin/env python2

#from cmotion.mround import *
#from cmotion.Q2 import *
import rospy
# import rosbag
# from sensor_msgs.msg import PointCloud2
# from sensor_msgs.msg import Image
# # from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import Path
# import tf
import rospkg
import subprocess


def Q2():
    rospack = rospkg.RosPack()
    CMotionDir = rospack.get_path('cmotion') + '/Bags/'
    player_proc = subprocess.Popen(['rosbag', 'play', 'CircularMotionBag.bag'], cwd=CMotionDir)
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
