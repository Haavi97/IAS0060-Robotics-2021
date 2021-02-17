#!/usr/bin/env python

"""
@author: keijo
"""

import rospy

from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class Odom3d:
    def __init__(self):
        self.PascalsPerAtmosphere = 101325.0
        self.MetersPerAtmosphere = 10.0
        self.MetersPerPascal = self.MetersPerAtmosphere / self.PascalsPerAtmosphere
        self.PressureAtSurface = self.PascalsPerAtmosphere
        
        self.z = 0.0
        self.z_var = 0.0
        
        self.fluidPressureSub = rospy.Subscriber("hw/pressure", FluidPressure, self.fluidPressureCallback)
        self.ekfSub = rospy.Subscriber("odometry/filtered", Odometry, self.ekfOfomCallback)
        
        self.odomPub = rospy.Publisher("odom", Odometry)
        
    def fluidPressureCallback(self, fluidPressureMsg):
        self.z = -self.fluidPressureToDepthFromSurface(fluidPressureMsg.fluid_pressure)
        self.z_var = self.fluidPressureToDepth(fluidPressureMsg.variance)
        
    def fluidPressureToDepth(self, pascals):
        return pascals * self.MetersPerPascal
        
    def fluidPressureToDepthFromSurface(self, pascals):
        return (pascals - self.PressureAtSurface) * self.MetersPerPascal
    
    def ekfOfomCallback(self, ekfOdomMsg):
        odom = Odometry()
        odom.header.frame_id = "world"
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose = ekfOdomMsg.pose.pose
        odom.pose.covariance[:] = ekfOdomMsg.pose.covariance[:]
        odom.pose.pose.position.z = self.z
        odom.pose.covariance[14] = self.z_var
        
        self.odomPub.publish(odom)

if __name__=="__main__":
    rospy.init_node("Odom3d")

    o3d = Odom3d()
    rospy.spin()
