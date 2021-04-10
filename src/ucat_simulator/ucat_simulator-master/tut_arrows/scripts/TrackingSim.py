#!/usr/bin/env python

"""
@author: Walid REMMAS
@contact: remmas.walid@gmail.com
          +33 6 62 55 45 04
"""

import matplotlib
import matplotlib.pyplot as plt

import rospy

import random

import numpy as np
from timeit import default_timer as timer


import cv2    # This imports openCV library 
import datetime
from cv_bridge import CvBridge, CvBridgeError # This imports CvBridge

from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped, Pose, Vector3Stamped, Twist, Point, Vector3
from sensor_msgs.msg import Image, FluidPressure
from tut_arrows_msgs.msg import FlippersModeCmd, BeaconPing
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from fuzzyController import FuzzyLogicController



class KalmanFilter:

    def __init__(self, x0, a, b, r, p0, q):
        self.x_list = [x0]
        #self.a = a
        #self.b = b
        self.r = r
        self.p_list = [p0]
        self.q = q

    def estimate(self, z, u):
        """
        z - [xt, yt, rt]
        """
        kg = float(self.p_list[-1]) / float((self.p_list[-1] + self.r))

        x = self.x_list[-1][0] + kg * (z[0] - self.x_list[-1][0])
        y = self.x_list[-1][1] + kg * (z[1] - self.x_list[-1][1])
        r = self.x_list[-1][2] + kg * (z[2] - self.x_list[-1][2])

        xt = [x, y, r]

        self.x_list.append(xt)

        self.p_list.append((1-kg)*self.p_list[-1] + self.q)

        return xt



class CameraBasedControl:

    universeOfDiscourseX = [-0.4, -0.1, 0.1, 0.4]
    universeOfDiscourseXDeriv = [-0.2, -0.04, 0.04, 0.2]
    universeOfDiscourseXOutput = [8, 0, -8]

    universeOfDiscourseY = [-0.4, -0.05, 0.05, 0.4]
    universeOfDiscourseYDeriv = [-0.2, -0.04, 0.04, 0.2]
    universeOfDiscourseYOutput = [8, 0, -8]

    universeOfDiscourseR = [-0.4, -0.05, 0.05, 0.4]
    universeOfDiscourseRDeriv = [-0.3, -0.04, 0.04, 0.3]
    universeOfDiscourseROutput = [-8, 0, 8]

    # Rules Table                neg |      zero  |    pos   |       Distance / Traffic light
    rulesTable = np.array([ [  'pos'   , 'stop'  ,  'neg' ], # negative velocity
			[   'pos'   , 'stop'  ,  'neg' ], # zero velocity
			[   'pos'   , 'stop'  ,  'neg' ]] # positive velocity
			) 

    x_controller = FuzzyLogicController(universeOfDiscourseX, universeOfDiscourseXDeriv,
				    universeOfDiscourseXOutput, rulesTable)
    y_controller = FuzzyLogicController(universeOfDiscourseY, universeOfDiscourseYDeriv,
				    universeOfDiscourseYOutput, rulesTable)
    r_controller = FuzzyLogicController(universeOfDiscourseR, universeOfDiscourseRDeriv,
				    universeOfDiscourseROutput, rulesTable)

    def __init__(self, dt):       

        self.dt = dt        # Sampling time

        self.camera_sub = rospy.Subscriber("/ucat0/camera/image", Image, self.onCamera) # Subscriber to images topic
        self.bridge = CvBridge()    # CvBridge will be used to convert the image into a usable format for openCV
        self.image = None # This is where the image will be stored
        self.show = 1 # Variable used to chose if user wants to display the images or not (1 for yes)

        self.error = np.array([0, 0, 0]) # Tracking error at time t
        self.lastError = np.array([0, 0, 0]) # Tracking error at time t-1
        self.depthTrackingError = 0
        self.lastDepthError = 0
        self.iterations = 0
        self.depthErrors = []

        self.kalman_filter = KalmanFilter([320, 240, 50], 0, 0, 50, 0, 5)
        
        self.lastRadiusError = 0
        self.lastMidpointXError = 0
        self.lastMidpointYError = 0

        self.last_plot_point_time = datetime.datetime.now()
        self.last_plot_point_diff = 500 # milliseconds

        self.cat_points_x = []
        self.cat_points_y = []
        self.cat_points_z = []

        self.cat_errors_x = []
        self.cat_errors_y = []
        self.cat_errors_z = []

        self.curr_abs_x = 0
        self.curr_abs_y = 0
        self.curr_abs_z = 0

        self.obj_points_x = []
        self.obj_points_y = []
        self.obj_points_z = []
        
        self.radius = 0
        self.targetRadius = 60
        
        self.radiusErrors = []
        self.midpointXErrors = []
        self.midpointYErrors = []

        self.U = np.zeros(6) # Force and Torque to be applied
        
        self.wrench_msg = WrenchStamped() # Force and torque message
        self.wrench_pub = rospy.Publisher("force_req", WrenchStamped, queue_size=10) # Force and torque publisher
	
        self.mode = FlippersModeCmd() # Mode selection
        self.mode.mode = "SLOW" # Fins configuration ("FAST", "SLOW")
        self.wrench_mode_pub = rospy.Publisher("force_mode", FlippersModeCmd, queue_size=25) # Mode selection publisher

        self.rpy_sub = rospy.Subscriber("rpy", Vector3, self.imuCallback)
        self.pressure_sub = rospy.Subscriber('/ucat0/hw/pressure', FluidPressure, self.depthCallback)
        self.obj_sub = rospy.Subscriber('/marker0/point', Point, self.trackerCallBack)
        self.cat_sub = rospy.Subscriber('/ucat0/sim_odom', Odometry, self.catCallback)

        self.depth = 5 # depth of robot
        self.roll = 0 # roll angle of robot
        self.pitch = 0 # pitch angle of robot
        self.yaw = 0 # yaw angle of robot

        self.depth = 0

        self.plot_shown = False
        self.full_zero_steps = 0
    
    # callback function of imu topic subscriber
    def imuCallback(self, imu_msg):
        self.roll = imu_msg.x
        self.pitch = imu_msg.y
        self.yaw = imu_msg.z

    def showImage(self, image):
        if(image.any() != None and self.show): # Make sure image is not empty and user want to display the image
	        cv2.imshow('U-CAT CAMERA',image) # This displays the image in a new window
        if cv2.waitKey(1) & 0xFF == ord('q'): # If user presses "q" on keyboard, stop displaying the images
            self.show = 0
            cv2.destroyAllWindows() # This closes all openCV windows

    def simpleRedApproach(self, cv_image):
        # ...

    	# blurred = cv2.GaussianBlur(image, (11, 11), 0)
    	img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    	# lower mask
    	lower_red = np.array([0,20,20])
    	upper_red = np.array([60,255,255])
    	mask = cv2.inRange(img_hsv, lower_red, upper_red)

    	# upper mask
    	#lower_red = np.array([160,20,20])
    	#upper_red = np.array([180,255,255])
    	#mask2 = cv2.inRange(img_hsv, lower_red, upper_red)

    	#mask = mask1 + mask2

        # Erode the mask and then dilate it to remove some noise.
    	#mask = cv2.erode(mask, None, iterations=2)
    	#mask = cv2.dilate(mask, None, iterations=2)

    	output_img = cv2.bitwise_and(img_hsv, img_hsv, mask=mask)

        grayscale = cv2.cvtColor(output_img, cv2.COLOR_BGR2GRAY)

    	image_2, contours, hierarchy = cv2.findContours(grayscale,cv2.RETR_LIST, \
                                   cv2.CHAIN_APPROX_SIMPLE)

    	# Find object with the biggest bounding box
    	mx = (0,0,0,0)      # biggest bounding box so far
    	mx_area = 0
    	for cont in contours:
    	    x,y,w,h = cv2.boundingRect(cont)
    	    area = w*h
    	    if area > mx_area:
    	        mx = x,y,w,h
    	        mx_area = area
    	x,y,w,h = mx
    	
        return x, y, w, h


    def onCamera(self, image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8") # converting the image into bgr format
            self.image = cv_image # Storing the image
    	     # Calling function to display the image
	        #start = timer()

            img_height = cv_image.shape[0]
            img_width = cv_image.shape[1]

            target_midpoint_x = img_width / 2
            target_midpoint_y = img_height / 2

            target_radius = self.targetRadius

            x, y, w, h = self.simpleRedApproach(cv_image)

            radius = (w + h) / 4 # Average the width and height
            midpoint_x = x + w / 2
            midpoint_y = y + h / 2

            midpoint_x += random.randint(-15, 15)
            midpoint_y += random.randint(-15, 15)
            radius += random.randint(-10, 10)

            if radius <= 0:
                radius = 1

            est_x, est_y, est_r = self.kalman_filter.estimate([midpoint_x, midpoint_y, radius], 0)
            
            if radius < 1:
                error_radius = 0
                error_midpoint_x = 0
                error_midpoint_y = 0
            else:
                if target_radius < 0:
                    error_radius = max(-1, (target_radius - radius) / float(self.targetRadius))
                else:
                    error_radius = min(1, (target_radius - radius) / float(self.targetRadius))
                    error_midpoint_x = (target_midpoint_x - midpoint_x) / 240.0
                    error_midpoint_y = (target_midpoint_y - midpoint_y) / 180.0

            use_estimates = True

            if use_estimates and est_r < 1:
                est_r = 0
                est_x = 0
                est_y = 0
            elif use_estimates:
                if est_r < 0:
                    error_radius = max(-1, (target_radius - est_r) / float(self.targetRadius))
                else:
                    error_radius = min(1, (target_radius - est_r) / float(self.targetRadius))
                    error_midpoint_x = (target_midpoint_x - est_x) / 240.0
                    error_midpoint_y = (target_midpoint_y - est_y) / 180.0

            self.radius = radius

            self.lastRadiusError = error_radius
            self.lastMidpointXError = error_midpoint_x
            self.lastMidpointYError = error_midpoint_y

            self.radiusErrors.append(error_radius)
            self.midpointXErrors.append(error_midpoint_x)
            self.midpointYErrors.append(error_midpoint_y)

            if abs(error_radius) < 0.45 and self.mode.mode == "FAST":
                new_mode = FlippersModeCmd()
                new_mode.mode = "SLOW"   # or "FAST"
                self.wrench_mode_pub.publish(new_mode)
                self.mode.mode = "SLOW"
            elif abs(error_radius) > 0.5 and self.mode.mode == "SLOW":
                new_mode = FlippersModeCmd()
                new_mode.mode = "FAST"   # or "SLOW"
                self.wrench_mode_pub.publish(new_mode)
                self.mode.mode = "FAST"


            cv2.circle(cv_image, (midpoint_x, midpoint_y), radius, (0, 0, 155), 2)

            # target circle
            cv2.circle(cv_image, (int(est_x), int(est_y)), int(est_r), (50, 155, 0), 5)
            cv2.circle(cv_image, (target_midpoint_x, target_midpoint_y), target_radius, (155, 0, 0), 2)


    	    #cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,0,155),1)

            self.showImage(cv_image)

        except CvBridgeError as e:
	        print(e)

    def depthCallback(self, depth_msg):
        self.depth = (depth_msg.fluid_pressure - 101325) / (1000 * 9.81)


    def run(self):
        while not rospy.is_shutdown():
            self.step()
            rospy.sleep(self.dt)

    def trackerCallBack(self, data):
        if self.curr_abs_x == 0 and self.curr_abs_y == 0 and self.curr_abs_z == 0:
            return

        if (datetime.datetime.now() - self.last_plot_point_time).total_seconds() * 1000 > (self.last_plot_point_diff):
            self.obj_points_x.append(data.x)
            self.obj_points_y.append(data.y)
            self.obj_points_z.append(data.z)

            self.cat_points_x.append(self.curr_abs_x)
            self.cat_points_y.append(self.curr_abs_y)
            self.cat_points_z.append(self.curr_abs_z)

            self.cat_errors_x.append(self.lastMidpointXError)
            self.cat_errors_y.append(self.lastMidpointYError)
            self.cat_errors_z.append(self.lastRadiusError)
            
            self.last_plot_point_time = datetime.datetime.now()        

    def catCallback(self, data):
        self.curr_abs_x = data.pose.pose.position.x
        self.curr_abs_y = data.pose.pose.position.y
        self.curr_abs_z = data.pose.pose.position.z

    # Main loop
    def step(self):
        e1 = self.error
        e2 = (self.error - self.lastError) / self.dt
        self.lastError = self.error
	
        desiredDepth = 10

        self.depthTrackingError = self.depth - desiredDepth
        
        kp = 0
        kd = 0

        kp_yaw = 10
        kd_yaw = 0

        kp_depth = 100
        kd_depth = 0
        kd_depth = 0

        kp_surge = 100 #0.03 * self.targetRadius * 2 #0.002
        kd_surge = 0 # 0 * 20#-0.0005

        # Forces to apply : U = [surge, sway, heave, roll, pitch, yaw]
        #===================================================================

        scale_factor = 1
        if self.lastRadiusError != 0 or self.lastMidpointYError != 0 or self.lastMidpointXError != 0:
                scale_factor = self.lastRadiusError  # maximum 1, gets lower with a lower radius_error
                scale_factor = scale_factor ** 3
                scale_factor = max(0.02, scale_factor)

        x_deriv = 0 if len(self.midpointXErrors) < 2 else self.midpointXErrors[-1] - self.midpointXErrors[-2]
        yawU = self.x_controller.control(self.lastMidpointXError, x_deriv)


        y_deriv = 0 if len(self.midpointYErrors) < 2 else self.midpointYErrors[-1] - self.midpointYErrors[-2]
        depthU = self.y_controller.control(self.lastMidpointYError, y_deriv)


        r_deriv = 0 if len(self.radiusErrors) < 2 else self.radiusErrors[-1] - self.radiusErrors[-2]
        surgeU = self.y_controller.control(self.lastRadiusError, r_deriv)

        m = max(abs(yawU), abs(depthU), abs(surgeU))
        if m == abs(yawU):
            yawU *= scale_factor
            depthU = 0
            surgeU = 0
            print("YAW", self.mode.mode)
        elif m == abs(depthU):
            yawU = 0
            depthU *= scale_factor
            surgeU = 0
            print("DEPTH", self.mode.mode)
        elif m == abs(surgeU):
            yawU = 0
            depthU = 0
            surgeU *= scale_factor
            print("SURGE", self.mode.mode)

        if self.lastMidpointXError == 0 and self.lastRadiusError == 0 and self.lastMidpointYError == 0:
            # Move back if the robot's lost track of the ball
            surgeU = -4

        self.U = [surgeU,0,depthU,0,0,yawU]
        #===================================================================

        self.lastDepthError = self.depthTrackingError
	
        # Publishing forces to wrench_driver    
        self.wrench_msg.header.stamp = rospy.Time.now()
        self.wrench_msg.wrench.force.x = self.U[0] 
        self.wrench_msg.wrench.force.y = self.U[1]
        self.wrench_msg.wrench.force.z = self.U[2]
        self.wrench_msg.wrench.torque.x = self.U[3]
        self.wrench_msg.wrench.torque.y = self.U[4]
        self.wrench_msg.wrench.torque.z = self.U[5]
            
        self.wrench_pub.publish(self.wrench_msg) # Publishing force vector to fins wrench driver

        plt.figure("U-CAT movement")
        plt.clf()
        plt.axis((2,9,7,14))
        
        plt.scatter(self.cat_points_x[-50::], self.cat_points_y[-50::], color=['blue'])
        plt.scatter(self.obj_points_x[-50::], self.obj_points_y[-50::], color=['red'])

        plt.figure("X Axis movement")
        plt.clf()
        plt.axis((0,201,-1,1))
        short_x = self.midpointXErrors[-200::]
        short_y = self.midpointYErrors[-200::]
        short_z = self.radiusErrors[-200::]

        x_axis = list(range(1, min(200, len(short_x)) + 1))
        
        plt.scatter(x_axis, short_x, color=['blue'])

        plt.figure("Y Axis movement")
        plt.clf()

        plt.axis((0,201,-1,1))

        plt.scatter(x_axis, short_y, color=['green'])
        plt.figure("Z Axis movement")
        plt.clf()

        plt.axis((0,201,-1,1))

        plt.scatter(x_axis, short_z, color=['red'])

        plt.pause(0.1)


if __name__ == '__main__':
    rospy.init_node("CameraBasedControl")
    mbc = CameraBasedControl(0.2) #(dt)
    mbc.run()

    
    
    
    
    
    
    
    
    
    
    
