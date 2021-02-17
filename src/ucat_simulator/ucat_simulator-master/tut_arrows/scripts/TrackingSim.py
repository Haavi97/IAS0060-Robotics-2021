#!/usr/bin/env python

"""
@author: Walid REMMAS
@contact: remmas.walid@gmail.com
          +33 6 62 55 45 04
"""

import rospy

import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped, Pose, Vector3Stamped, Twist, Point, Vector3
from sensor_msgs.msg import Image, FluidPressure
from tut_arrows_msgs.msg import FlippersModeCmd, BeaconPing
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_msgs.msg import Float32


desiredDepth = 9
Kp = 6
D = 10
I = 1


class CameraBasedControl:
    def __init__(self, dt):

        self.dt = dt        # Sampling time

        self.error = np.array([0, 0, 0])  # Tracking error at time t
        self.lastError = np.array([0, 0, 0])  # Tracking error at time t-1

        self.rmsQueue = np.zeros(200)
        self.depthErrorQueue = np.zeros(5)

        self.U = np.zeros(6)  # Force and Torque to be applied

        self.wrench_msg = WrenchStamped()  # Force and torque message
        self.wrench_pub = rospy.Publisher(
            "force_req", WrenchStamped, queue_size=10)  # Force and torque publisher

        self.mode = FlippersModeCmd()  # Mode selection
        self.mode.mode = "FAST"  # Fins configuration ("FAST", "SLOW")
        self.wrench_mode_pub = rospy.Publisher(
            "force_mode", FlippersModeCmd, queue_size=25)  # Mode selection publisher

        self.error_msg = Float32()
        self.error_pub = rospy.Publisher("depth_error", Float32, queue_size=10)

        self.rpy_sub = rospy.Subscriber("rpy", Vector3, self.imuCallback)

        self.pressure_sub = rospy.Subscriber("/ucat0/hw/pressure", FluidPressure,
                                             self.depthCallback)

        self.depth = 0  # depth of robot
        self.roll = 0  # roll angle of robot
        self.pitch = 0  # pitch angle of robot
        self.yaw = 0  # yaw angle of robot

        self.count = 0

    # callback function of imu topic subscriber

    def imuCallback(self, imu_msg):
        self.roll = imu_msg.x
        self.pitch = imu_msg.y
        self.yaw = imu_msg.z

    def run(self):
        while not rospy.is_shutdown():
            self.step()
            rospy.sleep(self.dt)

    # Main loop

    def step(self):
        self.error[2] = self.depth - desiredDepth

        e1 = self.error
        e2 = (self.error - self.lastError) / self.dt
        self.lastError = self.error

        self.rmsQueue[:-1] = self.rmsQueue[1:]
        self.rmsQueue[-1] = self.error[2]

        self.depthErrorQueue[:-1] = self.depthErrorQueue[1:]
        self.depthErrorQueue[-1] = self.error[2]

        self.count += 1
        if(self.count % 200 == 0):
            rms = np.sqrt(np.mean(self.rmsQueue**2))
            rospy.loginfo("RMS: {:.5f}".format(rms))
            self.error_msg.data = rms
            self.error_pub.publish(self.error_msg)

        # Forces to apply : U = [surge, sway, heave, roll, pitch, yaw]
        # ===================================================================
        yawU = 0    
        depthU = Kp * e1[2] + D * e2[2] + I * sum(self.depthErrorQueue)
        surgeU = 0
        self.U = [surgeU, 0, depthU, 0, 0, yawU]
        # ===================================================================

        # Publishing forces to wrench_driver
        self.wrench_msg.header.stamp = rospy.Time.now()
        self.wrench_msg.wrench.force.x = self.U[0]
        self.wrench_msg.wrench.force.y = self.U[1]
        self.wrench_msg.wrench.force.z = self.U[2]
        self.wrench_msg.wrench.torque.x = self.U[3]
        self.wrench_msg.wrench.torque.y = self.U[4]
        self.wrench_msg.wrench.torque.z = self.U[5]

        # Publishing force vector to fins wrench driver
        self.wrench_pub.publish(self.wrench_msg)

    def depthCallback(self, msg):
        pressure = msg.fluid_pressure
        self.depth = (pressure-101325)/(1000*9.81)


if __name__ == '__main__':
    rospy.sleep(2)
    rospy.init_node("CameraBasedControl")
    mbc = CameraBasedControl(0.2)  # (dt)
    mbc.run()
