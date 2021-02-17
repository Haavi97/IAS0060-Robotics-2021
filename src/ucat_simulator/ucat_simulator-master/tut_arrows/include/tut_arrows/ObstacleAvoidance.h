#ifndef TUT_ARROWS_OBSTACLE_AVOIDANCE_
#define TUT_ARROWS_OBSTACLE_AVOIDANCE_

#include <ros/ros.h>

#include <eigen3/Eigen/Dense> // for Eigen::Vector3d

#include "geometry_msgs/Wrench.h"
#include "sensor_msgs/Range.h"

struct SonarInfo
{
	// ROS topic for this sonar
	std::string topic;

	// pointing away from the direction of the sonar i.e. this is the obstacle avoidance force
	Eigen::Vector3d force;

	// latest range message
	sensor_msgs::RangeConstPtr rangeMsg;

	// user visible name
	std::string name;
};

class ObstacleAvoidance
{
public:
	ObstacleAvoidance();

	void onRangeMessage(sensor_msgs::Range::Ptr data);
	void onRangeMessage(SonarInfo& sonarInfo, sensor_msgs::RangeConstPtr data);

	Eigen::Vector3d getOAForce() const;

	bool applyOA(geometry_msgs::Wrench& wrench);

	bool enabled() const;

private:
	bool applyOAToAxis(double oaForce, double& force, char axis) const;

private:
	ros::NodeHandle nh_;

	std::vector<SonarInfo> sonars_;
	std::vector<ros::Subscriber> sonarSubs_;

	/// if obstacle is encountered then OA is active until timeout occurs
	double oaTimeout_;

	/// closer than minimum distance is interpreted as false positive (e.g. obstruction by fin)
	double oaMinDistance_;

	/// closer than OA distance threshold activates OA behavior
	double oaDistance_;

	bool oaEnabled_;

	Eigen::Vector3d scale_;
};

#endif
