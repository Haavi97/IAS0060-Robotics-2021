/*
 * coordinateConverter.cpp
 *
 *  Created on: May 27, 2015
 *      Author: rasmus
 */

#include <ros/ros.h>
#include <tut_arrows_msgs/SetTarget.h>
#include <tut_arrows_msgs/XYZToLatLonDepth.h>
#include <tf/LinearMath/Vector3.h>
#include <eigen3/Eigen/Core>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <tut_arrows_msgs/GetXYZ.h>

namespace
{
ros::ServiceClient setTargetServiceCaller, setPoseServiceCaller;
ros::ServiceServer xyzToLatLonDepthService;
ros::ServiceServer getXYZ;
std::unique_ptr<tf::TransformBroadcaster> br;
ros::Publisher waypointVisualizationPublisher, waypointGPSPublisher;
tf::Transform transform;
Eigen::Vector3d localZeroInGlobalCoords;

// See GeographicLib changelog from 1.36 to 1.37
const GeographicLib::Geodesic geod(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_a());

//const double EarthMeanRadius = 6367445; // meters
//const double EarthEquatorialRadius = 6378137; // meters
//const double EarthPolarRadius = 6356752.3; // meters
//double EarthRadiusAtLatitude(double latitudeDegrees)
//{
////	return EarthMeanRadius * std::cos(latitudeDegrees / 180.0 * 3.14159);
////	double phi = latitudeDegrees / 180.0 * 3.14159;
////	const double a = EarthEquatorialRadius, b = EarthPolarRadius;
////	return std::sqrt((std::pow(a*a*std::cos(phi),2) + std::pow(b*b*std::sin(phi),2))
////					/(std::pow(a*std::cos(phi),2) + std::pow(b*std::sin(phi),2)));
////	return (a*b*a*b)/std::pow(std::pow(a*std::cos(phi),2)+std::pow(b*std::sin(phi),2),1.5);
////	return a*a/std::sqrt(std::pow(a*cos(phi),2) + std::pow(b*std::sin(phi),2));
//	double x, y, z;
//	GeographicLib::Geocentric::WGS84.Forward(latitudeDegrees, 0, 0, x, y, z);
//	return x;
//}

Eigen::Vector3d LatLonDepthToXYZ(Eigen::Vector3d global)
{
	Eigen::Vector3d local = global - localZeroInGlobalCoords;
	double north, east;
	geod.Inverse(localZeroInGlobalCoords.x(), localZeroInGlobalCoords.y(), global.x(), localZeroInGlobalCoords.y(), north);
	geod.Inverse(localZeroInGlobalCoords.x(), localZeroInGlobalCoords.y(), localZeroInGlobalCoords.x(), global.y(), east);
	if(global.x() - localZeroInGlobalCoords.x() < 0)
		north *= -1;
	if(global.y() - localZeroInGlobalCoords.y() < 0)
		east *= -1;
//	double east = 3.14159 * EarthRadiusAtLatitude(lat) * lon / 180.0;
	return Eigen::Vector3d(north, -east, -global.z());
}

Eigen::Vector3d XYZToLatLonDepth(Eigen::Vector3d xyz)
{
	double north = xyz.x(), east = -xyz.y(), latitude, longitude, tmp;
	geod.Direct(localZeroInGlobalCoords.x(), localZeroInGlobalCoords.y(), 0, north, latitude, tmp);
	geod.Direct(localZeroInGlobalCoords.x(), localZeroInGlobalCoords.y(), 90, east, tmp, longitude);
//	longitude = east / (3.14159 * EarthRadiusAtLatitude(latitude)) * 180.0;
	return Eigen::Vector3d(latitude, longitude, xyz.z());
}

bool ConvertGPSToXYZ(tut_arrows_msgs::GetXYZ::Request& request, tut_arrows_msgs::GetXYZ::Response& response)
{
	Eigen::Vector3d lld(request.global.x, request.global.y, request.global.z);
	Eigen::Vector3d xyz = LatLonDepthToXYZ(lld);
	response.XYZ.x = xyz.x();
	response.XYZ.y = xyz.y();
	response.XYZ.z = xyz.z();
	return true;
}

bool getXYZToLatLonDepth(tut_arrows_msgs::XYZToLatLonDepth::Request& request, tut_arrows_msgs::XYZToLatLonDepth::Response& response)
{
	Eigen::Vector3d xyz(request.xyz.x, request.xyz.y, request.xyz.z);
	Eigen::Vector3d lld = XYZToLatLonDepth(xyz);
	response.lld.x = lld.x();
	response.lld.y = lld.y();
	response.lld.z = lld.z();
	return true;
}

void setBeacons()
{
	ros::NodeHandle nh, nhPrivate("~");
	XmlRpc::XmlRpcValue beacons;
	if (nhPrivate.getParam("beacons", beacons))
	{
		for (auto b : beacons)
		{
			const std::string& beaconId = b.first;
			std::map<std::string, double> beacon;
			if (nhPrivate.getParam("beacons/" + beaconId, beacon))
			{
				const double lat = beacon.at("latitude");
				const double lon = beacon.at("longitude");
				const double depth = beacon.at("depth");
				Eigen::Vector3d xyz = LatLonDepthToXYZ(Eigen::Vector3d(lat, lon, depth));
				std::map<std::string, double> beaconCoords;
				beaconCoords.insert(std::pair<std::string, double>("x", xyz.x()));
				beaconCoords.insert(std::pair<std::string, double>("y", xyz.y()));
				beaconCoords.insert(std::pair<std::string, double>("z", xyz.z()));
				nh.setParam("MotionEstimator/beacons/" + beaconId, beaconCoords);
			}
		}
	}
}

bool callPoseService(ros::ServiceClient& serviceCaller, Eigen::Vector3d xyz)
{
	tut_arrows_msgs::SetTarget service;
	service.request.pose.position.x = xyz.x();
	service.request.pose.position.y = xyz.y();
	service.request.pose.position.z = xyz.z();
	service.request.pose.orientation.x = 0;
	service.request.pose.orientation.y = 0;
	service.request.pose.orientation.z = 0;
	service.request.pose.orientation.w = 1;

	return serviceCaller.call(service);
}

void publishRobotTransform(Eigen::Vector3d xyz)
{
	transform.setOrigin(tf::Vector3(xyz.x(), xyz.y(), xyz.z()));
}

void tfTimerCallback(const ros::TimerEvent& event)
{
	br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "local"));
}

void setZero()
{
	ros::NodeHandle nh, nhPrivate("~");
        double lat, lon, depth;
        nhPrivate.getParam("input/latitude", lat);
        nhPrivate.getParam("input/longitude", lon);
        nhPrivate.getParam("input/depth", depth);
        localZeroInGlobalCoords = Eigen::Vector3d(lat, lon, 0);
}

void setRobotPose()
{
	ros::NodeHandle nh, nhPrivate("~");
	double lat, lon, depth;
	nhPrivate.getParam("input/latitude", lat);
	nhPrivate.getParam("input/longitude", lon);
	nhPrivate.getParam("input/depth", depth);
	//localZeroInGlobalCoords = Eigen::Vector3d(lat, lon, 0);
	Eigen::Vector3d xyz = LatLonDepthToXYZ(Eigen::Vector3d(lat, lon, depth));
	ROS_INFO("coordinate_converter: trying to set robot position to x=%f, y=%f, z=%f (lat=%f, lon=%f, depth=%f)",
			xyz.x(), xyz.y(), xyz.z(), lat, lon, depth);
	while (ros::ok() && !callPoseService(setPoseServiceCaller, xyz))
	{
		ROS_ERROR("coordinate_converter: Failed to set robot pose. Retry in 5 s");
		ros::Duration(5.0).sleep();
	}
	ROS_INFO("coordinate_converter: robot pose set successfully");
	publishRobotTransform(xyz);
}

void receiveGPS(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	if(msg->status.status < 0)
		return;
	Eigen::Vector3d xyz = LatLonDepthToXYZ(Eigen::Vector3d(msg->latitude, msg->longitude, -msg->altitude));
	if(!callPoseService(setPoseServiceCaller, xyz))
		ROS_ERROR("coordinate_converter: Failed to set robot pose.");
	else
	{
		publishRobotTransform(xyz);
		ROS_INFO("coordinate_converter: robot pose set successfully");
	}
}

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

void receiveCoordinates(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	Eigen::Vector3d xyz = LatLonDepthToXYZ(Eigen::Vector3d(msg->latitude, msg->longitude, -msg->altitude));
	setTarget(xyz);
}

void rvizReceiver(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	ROS_INFO("Got waypoint from GUI");
	Eigen::Vector3d xyz(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
	xyz += Eigen::Vector3d(transform.getOrigin());
	xyz.z() = -1.0;
	setTarget(xyz);

	Eigen::Vector3d coords = XYZToLatLonDepth(xyz);
	sensor_msgs::NavSatFix outMsg;
	outMsg.header.stamp = ros::Time::now();
	outMsg.latitude = coords.x();
	outMsg.longitude = coords.y();
	outMsg.altitude = coords.z();
	waypointGPSPublisher.publish(outMsg);
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "coordinate_converter");
	ros::NodeHandle nh;
//	Eigen::Vector3d a = LatLonDepthToXYZ(Eigen::Vector3d(59.3982805555556, 24.6619388888889, 3.0));
//	Eigen::Vector3d a2 = LatLonDepthToXYZ(Eigen::Vector3d(59.3982388888889, 24.6619972222222, 3.0));
//	Eigen::Vector3d b = XYZToLatLonDepth(a);
//	Eigen::Vector3d b2 = XYZToLatLonDepth(a2);
//	ROS_INFO("%f,%f,%f jaaa %f,%f,%f", a.x(), a.y(), a.z(), b.x(), b.y(), b.z());
//	ROS_INFO("%f,%f,%f jaaa %f,%f,%f", a2.x(), a2.y(), a2.z(), b2.x(), b2.y(), b2.z());
//	ROS_INFO("Dist: %f", (a-a2).norm());
//	for(int lat = 0; lat <= 90; lat++)
//		std::cout << lat << ": " << EarthRadiusAtLatitude(lat) << std::endl;
//	std::cout << "aaaa " << 3.14159 * EarthRadiusAtLatitude(59.3982805555556) * 24.6619388888889 / 180.0 - 3.14159 * EarthRadiusAtLatitude(59.3982388888889) *  24.6619972222222/ 180.0 << std::endl;
//	std::cout << "radii " << EarthRadiusAtLatitude(59.3982805555556) - EarthRadiusAtLatitude(59.3982388888889) << std::endl;
	br = std::unique_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	setPoseServiceCaller = nh.serviceClient<tut_arrows_msgs::SetTarget>("set_pose");
	setTargetServiceCaller = nh.serviceClient<tut_arrows_msgs::SetTarget>("SetTarget");
	xyzToLatLonDepthService = nh.advertiseService("xyzToLatLonDepth", getXYZToLatLonDepth);
	getXYZ = nh.advertiseService("GetXYZ", ConvertGPSToXYZ);
	waypointVisualizationPublisher = nh.advertise<visualization_msgs::Marker>("waypoint_marker", 100);
	waypointGPSPublisher = nh.advertise<sensor_msgs::NavSatFix>("coordinates_output", 100);
	ros::Subscriber coordinatesSubscriber = nh.subscribe("coordinates_input", 1, receiveCoordinates);
	ros::Subscriber GPSSubscriber = nh.subscribe("gps_input", 1, receiveGPS);
	ros::Subscriber rvizWaypointSubscriber = nh.subscribe("/move_base_simple/goal", 1, rvizReceiver);
	setZero();
	setBeacons();
	setRobotPose();
	//setBeacons();
	ros::Timer timer = nh.createTimer(ros::Duration(0.1), tfTimerCallback);
	ros::spin();
	return 0;
}
