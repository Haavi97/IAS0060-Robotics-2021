/*
 * imu2tf.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: arrows
 */

#include <ros/ros.h>
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

std::string robot_frame;

void IMUCallback(const sensor_msgs::ImuConstPtr& msg)
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0, 0, 0));
	tf::Quaternion q;
	tf::quaternionMsgToTF(msg->orientation, q);
	transform.setRotation(q);
	br.sendTransform(
			tf::StampedTransform(transform, ros::Time::now(), "world",
					robot_frame));
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "imu2tf");
	ros::NodeHandle n, n_("~");
	n_.getParam("robot_frame_id", robot_frame);

	ros::Subscriber sub = n.subscribe("imu", 10, &IMUCallback);

	ros::spin();
	return 0;
}
;

