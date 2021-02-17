#include "tut_arrows/TrajectoryUtils.h"
#include "tut_arrows/QuatUtils.h"

Eigen::Matrix<double, 6, 1> TrajectoryUtils::getPoseError(const geometry_msgs::Pose& currentPose, const geometry_msgs::Pose& targetPose)
{
	Eigen::Matrix<double, 6, 1> poseError;

	poseError(0)= targetPose.position.x - currentPose.position.x;
	poseError(1) = targetPose.position.y - currentPose.position.y;
	poseError(2) = targetPose.position.z - currentPose.position.z;

	double currentRoll, currentPitch, currentYaw;
	tf::Matrix3x3(QuatUtils::fromMsg(currentPose.orientation)).getRPY(currentRoll, currentPitch, currentYaw);

	double targetRoll, targetPitch, targetYaw;
	tf::Matrix3x3(QuatUtils::fromMsg(targetPose.orientation)).getRPY(targetRoll, targetPitch, targetYaw);

	//tf::Quaternion q0 = QuatUtils::fromMsg(currentPose.orientation);
	//tf::Quaternion q1 = QuatUtils::fromMsg(targetPose.orientation);
	//tf::Quaternion diff = q1 * q0.inverse();

	poseError(3) = QuatUtils::getSmallestRotation(targetRoll, currentRoll);
	poseError(4) = QuatUtils::getSmallestRotation(targetPitch, currentPitch);
	poseError(5) = QuatUtils::getSmallestRotation(targetYaw, currentYaw);

	return poseError;
}

tf::Vector3 TrajectoryUtils::getError(const geometry_msgs::Vector3& current, const geometry_msgs::Vector3& target)
{
	tf::Vector3 error;
	error.setX(target.x - current.x);
	error.setY(target.y - current.y);
	error.setZ(target.z - current.z);
	return error;
}

tf::Vector3 TrajectoryUtils::getError(const geometry_msgs::Vector3& current, const tf::Vector3& target)
{
	tf::Vector3 error;
	error.setX(target.x() - current.x);
	error.setY(target.y() - current.y);
	error.setZ(target.z() - current.z);
	return error;
}

tf::Vector3 TrajectoryUtils::getLocalError(const geometry_msgs::Vector3& current, const geometry_msgs::Vector3& target, const tf::Quaternion& orientation)
{
	return tf::quatRotate(orientation, getError(current, target));
}
