#!/usr/bin/env python

"""
@author: keijo
"""

import rospy
import tf

import numpy as np
import math
from random import gauss

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

from tut_arrows_msgs.msg import BeaconPing

from SnelliusPothenot import solveSnelliusPothenot

def smallestAngle(target, source):
    a = target - source
    return (a + math.pi) % (2.0 * math.pi) - math.pi 

class Beacon:
    def __init__(self, id, location):
        self.id = id
        self.location = np.array(location)
        self.lastDirection = None
        self.lastTimeStamp = None
        
    def updateDirection(self, direction, timestamp):
        self.lastDirection = direction
        self.lastTimeStamp = timestamp

    def get2dLocation(self):
        return self.location[:2]


class OdomPose:
    def __init__(self):
        self.odomSub = rospy.Subscriber("odom", Odometry, self.odomCallback)
        self.posePub = rospy.Publisher("beacon_pose", PoseWithCovarianceStamped)
        
    def odomCallback(self, odomMsg):
        poseMsg = PoseWithCovarianceStamped()
        poseMsg.header.stamp = odomMsg.header.stamp
        poseMsg.header.frame_id = 'world'
        poseMsg.pose = odomMsg.pose
        self.posePub.publish(poseMsg)


class BeaconPose:
    def __init__(self):
        self.beaconSub = rospy.Subscriber('hw/beaconReceiver', BeaconPing, self.beaconPingCallback)
        self.posePub = rospy.Publisher("beacon_pose", PoseWithCovarianceStamped)
        
        beacon1 = Beacon('beacon1', [50.0, 50.0, -1.05])
        beacon2 = Beacon('beacon2', [-10.0, 20.0, -2.0])
        beacon3 = Beacon('beacon3', [40.0, -20.0, -1.0])
        
        self.beacons = {beacon1.id : beacon1, beacon2.id : beacon2, beacon3.id : beacon3}
        
    def beaconPingCallback(self, beaconPingMsg):
        if beaconPingMsg.id in self.beacons:
            x = beaconPingMsg.direction.x
            y = beaconPingMsg.direction.y
            
            new_direction = math.atan2(y, x)
                
            self.beacons[beaconPingMsg.id].updateDirection(new_direction, beaconPingMsg.header.stamp)
            
    def calcPose(self):
        for beacon in self.beacons.itervalues():
            if beacon.lastDirection == None:
                return
        
        B12 = abs(smallestAngle(self.beacons['beacon1'].lastDirection, self.beacons['beacon2'].lastDirection))
        B13 = abs(smallestAngle(self.beacons['beacon1'].lastDirection, self.beacons['beacon3'].lastDirection))
        B23 = abs(smallestAngle(self.beacons['beacon2'].lastDirection, self.beacons['beacon3'].lastDirection))
        
        A = None
        B = None
        C = None
        alpha = None
        beta = None
        
        largest = max(B12, B13, B23)
        if largest == B12:
            A = self.beacons['beacon1']
            B = self.beacons['beacon2']
            C = self.beacons['beacon3']
            alpha = B13
            beta = B23
        elif largest == B13:
            A = self.beacons['beacon1']
            B = self.beacons['beacon3']
            C = self.beacons['beacon2']
            alpha = B12
            beta = B23
        else:
            A = self.beacons['beacon2']
            B = self.beacons['beacon3']
            C = self.beacons['beacon1']
            alpha = B12
            beta = B13
            
        A_loc = A.get2dLocation()
        B_loc = B.get2dLocation()
        C_loc = C.get2dLocation()

        P = solveSnelliusPothenot(A_loc, B_loc, C_loc, alpha, beta)
        
        #print "P = " + str(P)
        
        AP_slope = math.atan2(P[1] - A_loc[1], P[0] - A_loc[0])
        
        yaw = None
        if AP_slope < 0.0:
            yaw = math.pi - A.lastDirection + AP_slope
        else:
            yaw = AP_slope + A.lastDirection
        
        self.publish2dPose(P, yaw)
        
    def publish2dPose(self, position, yaw):
        poseMsg = PoseWithCovarianceStamped()
        poseMsg.header.stamp = rospy.Time.now()
        poseMsg.header.frame_id = 'world'
        poseMsg.pose.pose.position.x = position[0]
        poseMsg.pose.pose.position.y = position[1]
        
        q = tf.transformations.quaternion_from_euler(0,0,yaw)
        
        poseMsg.pose.pose.orientation.x = q[0]
        poseMsg.pose.pose.orientation.y = q[1]
        poseMsg.pose.pose.orientation.z = q[2]
        poseMsg.pose.pose.orientation.w = q[3]
        
        poseMsg.pose.covariance[0] = 4.0
        poseMsg.pose.covariance[7] = 4.0
        #poseMsg.pose.covariance[14] = 1.5
        #poseMsg.pose.covariance[21] = 0.5
        #poseMsg.pose.covariance[28] = 0.5
        poseMsg.pose.covariance[35] = 0.57
        
        self.posePub.publish(poseMsg)
        
    def run(self):
        rate = rospy.Rate(5)
        
        while not rospy.is_shutdown():
            self.calcPose()
            rate.sleep()
        
        
if __name__=="__main__":
    rospy.init_node("BeaconPose")

    #op = OdomPose()
    #rospy.spin()
    
    bp = BeaconPose()
    bp.run()
    
    
