#!/usr/bin/env python

"""
@author: keijo
"""

import rospy

from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import TwistWithCovarianceStamped

class MotionTwist:
    def __init__(self):        
        self.wrenchSub = rospy.Subscriber("force_req", WrenchStamped, self.wrenchCallback)
        self.twistPub = rospy.Publisher("twist", TwistWithCovarianceStamped)
        
    def wrenchCallback(self, wrenchMsg):
        twistMsg = TwistWithCovarianceStamped()
        twistMsg.header.stamp = wrenchMsg.header.stamp
        twistMsg.header.frame_id = 'ucat0/ekf_base_link'
        twistMsg.twist.twist.linear.x = 0.050 * wrenchMsg.wrench.force.x
        twistMsg.twist.twist.linear.y = 0.015 * wrenchMsg.wrench.force.y
        twistMsg.twist.twist.angular.z = 1.1 * wrenchMsg.wrench.torque.z
        self.twistPub.publish(twistMsg)

if __name__=="__main__":
    rospy.init_node("MotionTwist")

    mt = MotionTwist()
    rospy.spin()
