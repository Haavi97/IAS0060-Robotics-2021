#!/usr/bin/env python

"""
@author: keijo
"""

import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import numpy as np
import math

from Quaternion import Quaternion, getAngularVelocity


class ImuSim:
    def __init__(self):
        self.lastTimeStamp = None
        
        self.pos = None
        self.vel = None
        self.angularVel = None
        self.accel = None
        self.orientation = None
        
        self.odomSub = rospy.Subscriber("sim_odom", Odometry, self.odomCallback)
        self.imuPub = rospy.Publisher("imu", Imu, queue_size=10)
        
    def odomCallback(self, odomMsg):
        self.update(odomMsg)
        self.publishImu()
        
    def update(self, odomMsg):
        newTimeStamp = odomMsg.header.stamp.to_sec()
        newPos = np.array([odomMsg.pose.pose.position.x, odomMsg.pose.pose.position.y, odomMsg.pose.pose.position.z])
        newOrientation = odomMsg.pose.pose.orientation
        
        if self.lastTimeStamp is not None:
            dt = newTimeStamp - self.lastTimeStamp
            
            if dt > 0.0:
                if self.pos is not None:
                    newVel = (newPos - self.pos) / dt
                    
                    if self.vel is not None:
                        self.accel = (newVel - self.vel) / dt
                        
                    self.vel = newVel
            
                if self.orientation is not None:
                    q0 = Quaternion([self.orientation.w, self.orientation.x, self.orientation.y, self.orientation.z])
                    q1 = Quaternion([newOrientation.w, newOrientation.x, newOrientation.y, newOrientation.z])
                    self.angularVel = getAngularVelocity(q1, q0, dt)
            

        self.lastTimeStamp = newTimeStamp
        self.pos = newPos
        self.orientation = newOrientation

    def publishImu(self):
        if self.orientation is not None and self.accel is not None and self.angularVel is not None:
            imu = Imu()
            imu.header.stamp = rospy.Time.now()
            imu.header.frame_id = 'world'
            
            imu.orientation = self.orientation
            
            imu.linear_acceleration.x = self.accel[0]
            imu.linear_acceleration.y = self.accel[1]
            imu.linear_acceleration.z = self.accel[2]
            
            imu.angular_velocity.x = self.angularVel[0]
            imu.angular_velocity.y = self.angularVel[1]
            imu.angular_velocity.z = self.angularVel[2]
            
            self.imuPub.publish(imu)

if __name__=="__main__":
    rospy.init_node("Imu")

    imuSim = ImuSim()
    rospy.spin()

    
