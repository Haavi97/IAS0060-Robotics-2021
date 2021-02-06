import rospy
from geometry_msgs.msg import Twist


pub = None


def subscribe_callback(msg):
    rospy.loginfo(rospy.get_caller_id()+"\n"
                  "linear: %s\n"
                  "---------------\n"
                  "angular: %s\n", msg.linear, msg.angular)

    linear_msg = msg.linear
    angular_msg = msg.angular

    pubmsg = Twist(linear=linear_msg, angular=angular_msg)
    pub.publish(pubmsg)


def talker():
    global pub

    pub = rospy.Publisher('cmd_vel_received', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("turtle1/cmd_vel", Twist, subscribe_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

    listener()
