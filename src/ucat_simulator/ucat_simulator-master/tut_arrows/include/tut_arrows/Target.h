#ifndef TUT_ARROWS_TARGET_H_
#define TUT_ARROWS_TARGET_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <cmath>
#include <memory>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "tut_arrows/QuatUtils.h"

class Target
{
public:
	typedef std::shared_ptr<Target> Ptr;

public:
	enum Type
	{
		Step = 0,
		Linear = 1,
		Tanh = 2,
		Hermite = 3,
		Explore = 4,
		BeaconHoming = 5,
		Cos = 6
	};

	static std::shared_ptr<Target> parse(const std::string& targetStr);
	static bool isValidPose(const geometry_msgs::Pose& pose);

public:
	Target(const geometry_msgs::Pose& targetPose);
	Target(Type type, const geometry_msgs::Pose& targetPose, double duration);

	Type getType() const;
	void setType(const std::string& type);

	double getDuration() const;
	void setDuration(double duration);

	bool isElapsed();

	const geometry_msgs::Pose& getTarget() const;

	void start(const geometry_msgs::Pose& currentPose, double currentTime);

	geometry_msgs::Pose getDesired(double currentTime) const;
	Eigen::Matrix<double, 6, 1> getDesiredVelocity(double currentTime) const;
	Eigen::Matrix<double, 6, 1> getDesiredAcceleration(double currentTime) const;

	std::string toString() const;

private:
	geometry_msgs::Pose getDesiredLinear(double currentTime) const;
	geometry_msgs::Pose getDesiredTanh(double currentTime) const;
	geometry_msgs::Pose getDesiredHermite(double currentTime) const;
	geometry_msgs::Pose getDesiredCos(double currentTime) const;

private:
	Type type_;
	geometry_msgs::Pose targetPose_;
	double duration_;

	geometry_msgs::Pose startPose_;
	double startTime_;
};

#endif // TUT_ARROWS_TARGET_H_
