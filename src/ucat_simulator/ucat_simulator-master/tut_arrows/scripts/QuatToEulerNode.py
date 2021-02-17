#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import tf

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class QuatToEuler:
    def __init__(self):
        self.odom_sub = rospy.Subscriber("hw/imu", Imu, self.imuCallback)
        self.rpy_pub = rospy.Publisher("rpy", Vector3, queue_size=10)

    def odomCallback(self, odom_msg):
        self.quatCallback(odom_msg.pose.pose.orientation)

    def imuCallback(self, imu_msg):
        self.quatCallback(imu_msg.orientation)

    def quatCallback(self, quat_msg):
        (r, p, y) = tf.transformations.euler_from_quaternion([quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w])

        v = Vector3()
        v.x = r
        v.y = p
        v.z = y

        self.rpy_pub.publish(v)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('QuatToEuler')
    q2e = QuatToEuler()
    q2e.run()
