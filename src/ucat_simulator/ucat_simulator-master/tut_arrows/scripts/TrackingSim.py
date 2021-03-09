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


from cv_bridge import CvBridge, CvBridgeError  # This imports CvBridge

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
        self.yawErrorQueue = np.zeros(5)
        self.surgeErrorQueue = np.zeros(5)

        self.U = np.zeros(6)  # Force and Torque to be applied

        self.wrench_msg = WrenchStamped()  # Force and torque message
        self.wrench_pub = rospy.Publisher(
            "force_req", WrenchStamped, queue_size=10)  # Force and torque publisher

        self.mode = FlippersModeCmd()  # Mode selection
        self.mode.mode = "SLOW"  # Fins configuration ("FAST", "SLOW")
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

        self.contours = None

        self.destCoordinates = None
        self.radius = None

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
        e1 = self.error
        e2 = (self.error - self.lastError) / self.dt
        self.lastError = self.error

        self.rmsQueue[:-1] = self.rmsQueue[1:]
        self.rmsQueue[-1] = self.error[2]

        self.depthErrorQueue[:-1] = self.depthErrorQueue[1:]
        self.depthErrorQueue[-1] = self.error[2]

        self.yawErrorQueue[:-1] = self.yawErrorQueue[1:]
        self.yawErrorQueue[-1] = self.error[0]

        self.surgeErrorQueue[:-1] = self.surgeErrorQueue[1:]
        self.surgeErrorQueue[-1] = self.error[1]

        self.count += 1
        if(self.count % 200 == 0):
            rms = np.sqrt(np.mean(self.rmsQueue**2))
            rospy.loginfo("RMS: {:.5f}".format(rms))
            self.error_msg.data = rms
            self.error_pub.publish(self.error_msg)

        # Forces to apply : U = [surge, sway, heave, roll, pitch, yaw]
        # ===================================================================
        yawU = 20 * e1[0] + 8 * e2[0] + 0.75 * sum(self.yawErrorQueue)
        depthU = 8 * e1[2] + 6 * e2[2] + 0.05 * sum(self.depthErrorQueue)
        surgeU = 10 * e1[1] + 20 * e2[1] + 10 * sum(self.surgeErrorQueue)
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

        #rospy.sleep(0.2)

    def depthCallback(self, msg):
        pressure = msg.fluid_pressure
        self.depth = (pressure-101325)/(1000*9.81)

    def onCamera(self, image):
        try:
            # converting the image into bgr format
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            (h, w) = cv_image.shape[:2]

            self.destCoordinates = (w/2, h/2)

            # Radius of circle
            self.radius = 15
            # Blue color in BGR
            color = (255, 0, 0)
            # Line thickness of 2 px
            thickness = 2
            self.image = cv2.circle(
                cv_image, self.destCoordinates, self.radius, color, thickness)  # Storing the image
            self.showImage(self.image)  # Calling function to display the image

            # Object detection:
            frame_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            self.mask = cv2.inRange(frame_HSV, self.red_low, self.red_up)

            #self.showMask(self.mask)

            # Contours
            _, self.contours, _ = cv2.findContours(
                self.mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            c = max(self.contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)

            self.error[0] = self.destCoordinates[0] - x
            self.error[2] = self.destCoordinates[1] - y
            self.error[1] = self.radius - w/2
            print('Ball found at: (x,y) = ({},{}), dest coordinates: (x, y) = ({}, {}), dest radius: {}, actual radius: {} error: {}'.format(
                x, y, self.destCoordinates[0], self.destCoordinates[1],self.radius, w/2, self.error), end='\r')
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
            cv2.drawContours(cv_image, self.contours[0], -1, (0, 255, 0), 3)
            self.showContours(cv_image)
        except CvBridgeError as e:
            print(e)

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
    rospy.sleep(2)
    rospy.init_node("CameraBasedControl")
    mbc = CameraBasedControl(0.2)  # (dt)
    mbc.run()
