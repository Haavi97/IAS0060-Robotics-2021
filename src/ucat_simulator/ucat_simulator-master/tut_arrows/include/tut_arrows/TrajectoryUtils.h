#ifndef TUT_ARROWS_TRAJECTORY_UTILS_H_
#define TUT_ARROWS_TRAJECTORY_UTILS_H_

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace TrajectoryUtils
{
	Eigen::Matrix<double, 6, 1> getPoseError(const geometry_msgs::Pose& currentPose, const geometry_msgs::Pose& targetPose);

	tf::Vector3 getError(const geometry_msgs::Vector3& current, const geometry_msgs::Vector3& target);
	tf::Vector3 getError(const geometry_msgs::Vector3& current, const tf::Vector3& target);
	tf::Vector3 getLocalError(const geometry_msgs::Vector3& current, const geometry_msgs::Vector3& target, const tf::Quaternion& orientation);
}

#endif
