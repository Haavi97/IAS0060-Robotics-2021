#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


pub = None


def subscribe_callback(msg):
    linear_msg = msg.linear
    angular_msg = msg.angular

    rospy.loginfo(rospy.get_caller_id()+"\n"
                  "linear:\n%s\n"
                  "---------------\n"
                  "angular:\n%s\n", linear_msg, angular_msg)

    pubmsg = Twist(linear=linear_msg, angular=angular_msg)
    pub.publish(pubmsg)


def talker():
    global pub

    pub = rospy.Publisher('cmd_vel_received', Twist, queue_size=10)
    rospy.init_node('publisher_subscriber', anonymous=True)


def listener():
    rospy.Subscriber("turtle1/cmd_vel", Twist, subscribe_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

    listener()
