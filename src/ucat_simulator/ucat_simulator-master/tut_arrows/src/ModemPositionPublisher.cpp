#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
#include "auv_msgs/AcousticModemData.h"
#include "tut_arrows_msgs/XYZToLatLonDepth.h"

class ModemPositionPublisher
{
public:
	ModemPositionPublisher();
	void odometryCallback(nav_msgs::Odometry::ConstPtr data);
	void run();

private:
	void sendPoint(const geometry_msgs::Point32& point);
	void sendCurrentLocation();

private:
	ros::NodeHandle nh_;
	ros::Subscriber odomSub_;
	ros::Publisher acousticPub_;
	ros::ServiceClient xyzToLatLonDepthConverter_;
	nav_msgs::Odometry::ConstPtr odom_;
};

ModemPositionPublisher::ModemPositionPublisher() : nh_()
{
	odomSub_ = nh_.subscribe("odom", 1, &ModemPositionPublisher::odometryCallback, this);
	acousticPub_ = nh_.advertise<auv_msgs::AcousticModemData>("send_buffer", 10);
	xyzToLatLonDepthConverter_ = nh_.serviceClient<tut_arrows_msgs::XYZToLatLonDepth>("xyzToLatLonDepth");
}

void ModemPositionPublisher::sendPoint(const geometry_msgs::Point32& point)
{
	uint32_t serial_size = ros::serialization::serializationLength(point);
	std::vector<uint8_t> buffer(serial_size);

	ros::serialization::OStream stream(&buffer[0], serial_size);
	ros::serialization::serialize(stream, point);

	auv_msgs::AcousticModemData outmsg;
	outmsg.header.stamp = ros::Time::now();
	outmsg.host = 2; // broadcast host would be 0
	outmsg.payload = buffer;
	acousticPub_.publish(outmsg);
}

void ModemPositionPublisher::sendCurrentLocation()
{
	if (!odom_)
	{
		//ROS_WARN("Odometry is missing");
		return;
	}

	tut_arrows_msgs::XYZToLatLonDepth xyzToLld;
	xyzToLld.request.xyz.x = odom_->pose.pose.position.x;
	xyzToLld.request.xyz.y = odom_->pose.pose.position.y;
	xyzToLld.request.xyz.z = odom_->pose.pose.position.z;

	if (xyzToLatLonDepthConverter_.call(xyzToLld))
	{
		geometry_msgs::Point32 point;
		point.x = xyzToLld.response.lld.x;
		point.y = xyzToLld.response.lld.y;
		point.z = xyzToLld.response.lld.z;

		sendPoint(point);
	}
	else
	{
		ROS_ERROR("Unable to convert xyz to GPS");
	}
}

void ModemPositionPublisher::odometryCallback(nav_msgs::Odometry::ConstPtr data)
{
	odom_ = data;
}

void ModemPositionPublisher::run()
{
	ros::Duration duration(15.0);

	while (ros::ok())
	{
		ros::spinOnce();

		//ROS_INFO("Sending current location");

		sendCurrentLocation();

		duration.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ModemPositionPublisher");
	ModemPositionPublisher modemPositionPublisher;
	modemPositionPublisher.run();
}
