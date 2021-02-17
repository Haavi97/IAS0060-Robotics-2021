/*
 * coordinateConverter.cpp
 *
 *  Created on: May 27, 2015
 *      Author: rasmus
 */

#include <ros/ros.h>
#include <tut_arrows_msgs/SetTarget.h>
#include <tf/LinearMath/Vector3.h>
#include <eigen3/Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

namespace
{
ros::ServiceClient setTargetServiceCaller;
ros::Publisher waypointVisualizationPublisher;

void publishWaypointVisualization(const geometry_msgs::Pose& pose)
{
	visualization_msgs::Marker visualizationMsg;
	visualizationMsg.header.stamp = ros::Time::now();
	visualizationMsg.header.frame_id = "/world";
	visualizationMsg.pose = pose;
	visualizationMsg.scale.x = 1;
	visualizationMsg.scale.y = 1;
	visualizationMsg.scale.z = 1;
	visualizationMsg.id = 0;
	visualizationMsg.type = visualization_msgs::Marker::SPHERE;
	visualizationMsg.action = visualization_msgs::Marker::MODIFY;
	visualizationMsg.scale.x = 1;
	visualizationMsg.scale.y = 1;
	visualizationMsg.scale.z = 1;
	visualizationMsg.color.a = 0.7;
	visualizationMsg.color.r = 0.0;
	visualizationMsg.color.g = 1.0;
	visualizationMsg.color.b = 0.0;
	waypointVisualizationPublisher.publish(visualizationMsg);
}

bool setTarget(Eigen::Vector3d xyz)
{
	tut_arrows_msgs::SetTarget service;
	service.request.pose.position.x = xyz.x();
	service.request.pose.position.y = xyz.y();
	service.request.pose.position.z = xyz.z();
	service.request.pose.orientation.x = 0;
	service.request.pose.orientation.y = 0;
	service.request.pose.orientation.z = 0;
	service.request.pose.orientation.w = 1;

	ROS_INFO("coordinate_converter: trying to set target waypoint to x=%f, y=%f, z=%f",
				service.request.pose.position.x, service.request.pose.position.y, service.request.pose.position.z);
	if(setTargetServiceCaller.call(service))
	{
		ROS_INFO("coordinate_converter: setting new target waypoint was successful");
		publishWaypointVisualization(service.request.pose);
		return true;
	}
	else
	{
		ROS_ERROR("coordinate_converter: FAILED to set target waypoint");
		return false;
	}
}

void rvizReceiver(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	ROS_INFO("Got waypoint from GUI");
	Eigen::Vector3d xyz(msg->pose.position.x, msg->pose.position.y, -5.0);
	setTarget(xyz);
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "SetTargetFromRviz");
	ros::NodeHandle nh;
	setTargetServiceCaller = nh.serviceClient<tut_arrows_msgs::SetTarget>("SetTarget");
	waypointVisualizationPublisher = nh.advertise<visualization_msgs::Marker>("waypoint_marker", 100);
	ros::Subscriber rvizWaypointSubscriber = nh.subscribe("/move_base_simple/goal", 1, rvizReceiver);
	ros::spin();
	return 0;
}
