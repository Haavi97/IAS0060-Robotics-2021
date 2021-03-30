#!/usr/bin/env python
from __future__ import print_function
"""
@author: Walid REMMAS
@contact: remmas.walid@gmail.com
          +33 6 62 55 45 04
"""

import rospy
import cv2    # This imports openCV library
import numpy as np
import copy as cp


from cv_bridge import CvBridge, CvBridgeError  # This imports CvBridge
from math import pi

from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped, Pose, Vector3Stamped, Twist, Point, Vector3
from sensor_msgs.msg import Image, FluidPressure
from tut_arrows_msgs.msg import FlippersModeCmd, BeaconPing
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_msgs.msg import Float32


desiredDepth = 9
# 0 -> yax (x)
# 1 -> surge (radius, forward)
# 2 -> heave (depth)
Kp = [0.007, 0.1, 0.007]
D = [0, 0, 0]
I = [0.0001, 0, 0.002]
THRESHOLD = 10  # 10%
THRESHOLD_RADIUS = 20  # 20%
THRESHOLD_DEPTH = 20  # 20%
RADIUS_FACTOR = 1.35
radius_correction = 1

CLOSENESS_THRESHOLD = 20

FAST_THRESHOLD = 50

sigma = 50
sigma_2 = sigma**2
surge_sigma = 5
surge_sigma_2 = surge_sigma**2
max_error_queue = 5

# Blue color in BGR
color = (255, 0, 0)
# Line thickness of 2 px
thickness = 2


class CameraBasedControl:
    def __init__(self, dt):

        self.dt = dt        # Sampling time

        self.error = np.array([0, 0, 0])  # Tracking error at time t
        self.lastError = np.array([0, 0, 0])  # Tracking error at time t-1

        self.rmsQueue = np.zeros(200)
        self.depthErrorQueue = np.zeros(max_error_queue)
        self.yawErrorQueue = np.zeros(max_error_queue)
        self.surgeErrorQueue = np.zeros(max_error_queue)

        self.U = np.zeros(6)  # Force and Torque to be applied

        self.wrench_msg = WrenchStamped()  # Force and torque message
        self.wrench_pub = rospy.Publisher(
            "force_req", WrenchStamped, queue_size=10)  # Force and torque publisher

        self.mode = FlippersModeCmd()  # Mode selection
        self.mode.mode = "SLOW"  # Fins configuration ("FAST", "SLOW")
        self.wrench_mode_pub = rospy.Publisher(
            "force_mode", FlippersModeCmd, queue_size=25)  # Mode selection publisher
        self.wrench_mode_pub.publish(self.mode)

        self.yaw_error_msg = Float32()
        self.surge_error_msg = Float32()
        self.heave_error_msg = Float32()
        self.pyaw_msg = Float32()
        self.psurge_msg = Float32()
        self.pdepth_msg = Float32()
        self.yaw_error_pub = rospy.Publisher(
            "/yaw_error", Float32, queue_size=10)
        self.surge_error_pub = rospy.Publisher(
            "/surge_error", Float32, queue_size=10)
        self.heave_error_pub = rospy.Publisher(
            "/heave_error", Float32, queue_size=10)
        self.pyaw_error_pub = rospy.Publisher(
            "/pyaw_error", Float32, queue_size=10)
        self.psurge_error_pub = rospy.Publisher(
            "/psurge_error", Float32, queue_size=10)
        self.pdepth_error_pub = rospy.Publisher(
            "/pdepth_error", Float32, queue_size=10)

        self.rpy_sub = rospy.Subscriber("rpy", Vector3, self.imuCallback)

        self.pressure_sub = rospy.Subscriber("/ucat0/hw/pressure", FluidPressure,
                                             self.depthCallback)

        self.depth = 0  # depth of robot
        self.roll = 0  # roll angle of robot
        self.pitch = 0  # pitch angle of robot
        self.yaw = 0  # yaw angle of robot

        self.count = 0

        # Task 3 added from Moodle
        self.camera_sub = rospy.Subscriber(
            "/ucat0/camera/image", Image, self.onCamera)  # Subscriber to images topic
        # CvBridge will be used to convert the image into a usable format for openCV
        self.bridge = CvBridge()
        self.image = None  # This is where the image will be stored
        # Variable used to chose if user wants to display the images or not (1 for yes)
        self.show = 1

        # Object detection attributes:
        self.mask = None
        self.red_low = np.array([170, 50, 10])
        self.red_up = np.array([180, 255, 255])
        self.yellow_low = np.array([0, 0, 0])
        self.yellow_up = np.array([60, 255, 255])

        self.contours = None

        self.destCoordinates = None
        # Radius of dest circle
        self.radius = 35
        self.x, self.y, self.r = None, None, None
        self.previous_x, self.previous_y = None, None

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
        e1 = cp.deepcopy(self.error)
        e2 = (self.error - self.lastError) / self.dt
        self.lastError = cp.deepcopy(self.error)

        self.rmsQueue[:-1] = self.rmsQueue[1:]
        self.rmsQueue[-1] = self.error[2]

        self.depthErrorQueue[:-1] = self.depthErrorQueue[1:]
        self.depthErrorQueue[-1] = self.error[2]

        self.yawErrorQueue[:-1] = self.yawErrorQueue[1:]
        self.yawErrorQueue[-1] = self.error[0]

        self.surgeErrorQueue[:-1] = self.surgeErrorQueue[1:]
        self.surgeErrorQueue[-1] = self.error[1]

        # self.count += 1
        # if(self.count % 200 == 0):
        #     rms = np.sqrt(np.mean(self.rmsQueue**2))
        #     rospy.loginfo("RMS: {:.5f}".format(rms))
        #     self.error_msg.data = rms
        #     self.error_pub.publish(self.error_msg)

        # pSurge = np.exp(-(e1[0]**2 + e1[1]**2)/(2*surge_sigma_2))
        # pDepth = np.exp(-(e1[0]**2)/(2*sigma_2))
        # pYaw = 1 - pDepth

        pYaw = 1
        pSurge = 1
        pDepth = 1

        e1[0] = 0 if abs(self.error[0]) < THRESHOLD else self.error[0]
        e1[1] = 0 if abs(self.error[1]) < THRESHOLD_RADIUS else self.error[1]
        e1[2] = 0 if abs(self.error[2]) < THRESHOLD_DEPTH else self.error[2]

        factor = 1
        if self.error[2] < 0:
            factor = radius_correction
        if (abs(self.error[0]) >= abs(self.error[1])) and (abs(self.error[0]) >= abs(self.error[2]*factor)):
            pSurge = 0
            pDepth = 0
        elif (abs(self.error[1]*factor) >= abs(self.error[0])) and (abs(self.error[1]*factor) >= abs(self.error[2])):
            pYaw = 0
            pDepth = 0
        else:
            pYaw = 0
            pSurge = 0

        if max(self.error, key=abs) >= FAST_THRESHOLD and self.mode.mode != 'FAST':
            self.mode.mode = "FAST"  # Fins configuration ("FAST", "SLOW")
            self.wrench_mode_pub.publish(self.mode)
            print('\nFAST')
        elif self.mode.mode == 'FAST':
            self.mode.mode = "SLOW"  # Fins configuration ("FAST", "SLOW")
            self.wrench_mode_pub.publish(self.mode)
            print('\nSLOW')

        if self.x != None:
            print('(x,y) = ({},{})->(x, y) = ({}, {}), r: {}->r: {} error: {}, pYaw,pSurge,pDepth:{},{},{}'.format(
                self.x, self.y,
                self.destCoordinates[0], self.destCoordinates[1],
                self.radius, self.r,
                self.error, pYaw, pSurge, pDepth), end='\r')

        # Forces to apply : U = [surge, sway, heave, roll, pitch, yaw]
        # ===================================================================
        yawU = pYaw * (Kp[0] * e1[0] + D[0] * e2[0] +
                       I[0] * sum(self.yawErrorQueue))
        surgeU = pSurge * (Kp[1] * e1[1] + D[1] * e2[1] +
                           I[1] * sum(self.surgeErrorQueue))
        depthU = pDepth * (Kp[2] * e1[2] + D[2] * e2[2] +
                           I[2] * sum(self.depthErrorQueue))
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

        self.yaw_error_msg.data = self.error[0]
        self.surge_error_msg.data = self.error[1]
        self.heave_error_msg.data = self.error[2]
        self.pyaw_msg.data = pYaw*50
        self.psurge_msg.data = pSurge*60
        self.pdepth_msg.data = pDepth*70

        # Publishing force vector to fins wrench driver
        self.wrench_pub.publish(self.wrench_msg)
        self.yaw_error_pub.publish(self.yaw_error_msg)
        self.surge_error_pub.publish(self.surge_error_msg)
        self.heave_error_pub.publish(self.heave_error_msg)
        self.pyaw_error_pub.publish(self.pyaw_msg)
        self.psurge_error_pub.publish(self.psurge_msg)
        self.pdepth_error_pub.publish(self.pdepth_msg)

    def depthCallback(self, msg):
        pressure = msg.fluid_pressure
        self.depth = (pressure-101325)/(1000*9.81)

    def onCamera(self, image):
        try:
            # converting the image into bgr format
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            (h, w) = cv_image.shape[:2]

            self.destCoordinates = (w/2, h/2)

            # self.showImage(self.image)  # Calling function to display the image

            # Object detection:
            frame_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            self.mask = cv2.inRange(frame_HSV, self.yellow_low, self.yellow_up)

            # Destination circle after mask not to mess with the colors
            self.image = cv2.circle(
                cv_image, self.destCoordinates, self.radius, color, thickness)  # Storing the image

            # self.showMask(self.mask)

            # Contours
            _, self.contours, _ = cv2.findContours(
                self.mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            c = max(self.contours, key=cv2.contourArea)

            (x, y), radius = cv2.minEnclosingCircle(c)

            if self.previous_x != None:
                p1 = (x-CLOSENESS_THRESHOLD) <= self.previous_x <= (x +
                                                                    CLOSENESS_THRESHOLD)
                p2 = (y-CLOSENESS_THRESHOLD) <= self.previous_y <= (y +
                                                                    CLOSENESS_THRESHOLD)
                if p1 and p2:
                    self.previous_x, self.previous_y = x, y
                else:
                    max_p = (0, 0, 0)
                    max_area = 0
                    for cont in self.contours:
                        (x1, y1), r1 = cv2.minEnclosingCircle(cont)
                        area = r1*pi**2
                        p1 = (
                            x1-CLOSENESS_THRESHOLD) <= self.previous_x <= (x1+CLOSENESS_THRESHOLD)
                        p2 = (
                            y1-CLOSENESS_THRESHOLD) <= self.previous_y <= (y1+CLOSENESS_THRESHOLD)
                        p3 = x1 != x
                        p4 = y1 != y
                        if area > max_area and p1 and p2 and p3:
                            max_p = x1, y1, r1
                            max_area = area
                    if max_p != (0, 0, 0):
                        print(max_p)
                        x, y, radius = max_p
                        self.previous_x, self.previous_y = x, y
            else:
                c = max(self.contours, key=cv2.contourArea)
                (x, y), radius = cv2.minEnclosingCircle(c)
                self.previous_x, self.previous_y = x, y
            center = (int(x), int(y))
            radius = int(radius)

            self.error[0] = (100*(self.destCoordinates[0] -
                                  center[0]))/self.destCoordinates[0]
            self.error[1] = (100*(self.radius - radius)) / \
                int(float(self.radius)*RADIUS_FACTOR)
            self.error[2] = (100*(self.destCoordinates[1] -
                                  center[1]))/self.destCoordinates[1]

            cv2.circle(cv_image, center, radius, (255, 0, 0), thickness)
            cv2.drawContours(cv_image, self.contours[0], -1, (0, 255, 0), 3)
            self.showContours(cv_image)
            self.x, self.y, self.r = center[0], center[1], radius
        except CvBridgeError as e:
            print(e)
        except:
            print('Out of camera scope' + ' '*100, end='\r')

    def showImage(self, image):
        # Had to change this line. Probably is better to put a try/except statement
        # Make sure image is not empty and user want to display the image
        if(image.any() != None and self.show):
            # This displays the image in a new window
            cv2.imshow('U-CAT CAMERA', image)
        # If user presses "q" on keyboard, stop displaying the images
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.show = 0
            cv2.destroyAllWindows()  # This closes all openCV windows

    def showMask(self, mask):
        # Had to change this line. Probably is better to put a try/except statement
        if self.show:
            try:
                # This displays the image in a new window
                cv2.imshow('U-CAT RED MASK', mask)
            except:
                if mask == None:
                    rospy.loginfo("Mask is None")
        # If user presses "q" on keyboard, stop displaying the images
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.show = 0
            cv2.destroyAllWindows()  # This closes all openCV windows

    def showContours(self, image):
        # Had to change this line. Probably is better to put a try/except statement
        if self.show:
            try:
                cv2.imshow('U-CAT contours', image)
            except:
                if image == None:
                    rospy.loginfo("Mask is None")
        # If user presses "q" on keyboard, stop displaying the images
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.show = 0
            cv2.destroyAllWindows()  # This closes all openCV windows


if __name__ == '__main__':
    # rospy.sleep(2)
    rospy.init_node("CameraBasedControl")
    mbc = CameraBasedControl(0.2)  # (dt)
    mbc.run()
