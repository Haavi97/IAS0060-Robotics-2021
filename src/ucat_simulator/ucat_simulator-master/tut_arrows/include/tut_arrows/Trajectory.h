#ifndef TUT_ARROWS_TRAJECTORY_H_
#define TUT_ARROWS_TRAJECTORY_H_

#include <ros/ros.h>
#include <auv_msgs/AcousticModemData.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include "tut_arrows_msgs/LoadTargets.h"
#include "tut_arrows_msgs/SetTarget.h"
#include "tut_arrows_msgs/SetTrajectory.h"
#include "tut_arrows_msgs/TrajectoryPoint.h"

#include <fstream>
#include <memory>
#include <queue>

#include "tut_arrows/Target.h"

class Trajectory
{
public:
	Trajectory();

	bool loadMissionTargets(const std::string& filename);

	void resetCurrentTarget(Target::Ptr target);

	void addTarget(Target::Ptr target);

	void clearTargets();

	void update(const geometry_msgs::Pose& currentPose, double currentTime);

	bool hasTarget() const;

	bool loadTargets(tut_arrows_msgs::LoadTargets::Request& request, tut_arrows_msgs::LoadTargets::Response& response);
	bool setTarget(tut_arrows_msgs::SetTarget::Request& request, tut_arrows_msgs::SetTarget::Response& response);
	bool setTrajectory(tut_arrows_msgs::SetTrajectory::Request& request, tut_arrows_msgs::SetTrajectory::Response& response);

	void odometryCallback(nav_msgs::Odometry::ConstPtr data);

	void acousticModemCallback(const auv_msgs::AcousticModemData::ConstPtr& data);

	void run();

private:
	bool loadTargets();
	bool loadTargets(const std::string& filename);

	void nextTarget(const geometry_msgs::Pose& currentPose, double currentTime);

	bool isCurrentTargetReached() const;

private:
	ros::NodeHandle nh_;

	Target::Ptr currentTarget_;
	std::queue<Target::Ptr> targets_;

	ros::Publisher trajectoryPointPub_;
	ros::Publisher controlModePub_;
	ros::Subscriber odomSub_;
	ros::Subscriber acousticModemSub_;

	ros::ServiceServer setTargetSrv_;
	ros::ServiceServer loadTargetsSrv_;
	ros::ServiceServer setTrajectorySrv_;
	ros::ServiceClient GPSToLocal_;
	ros::ServiceClient startExplorer_;
	ros::ServiceClient stopExplorer_;

	double positionPrecision_;
	double orienatationPrecision_;

	geometry_msgs::Pose currentPose_;
	double currentTime_;

	nav_msgs::Odometry::ConstPtr odom_;

	bool isStarted_;

	double startupTime_;

	bool useTimedTargets_;

	double yawOffset_;
};

#endif // TUT_ARROWS_TRAJECTORY_H_
