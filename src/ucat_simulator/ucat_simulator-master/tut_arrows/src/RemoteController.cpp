#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

#include "tut_arrows/ControllerFactory.h"
#include "tut_arrows/MathUtils.h"
#include "tut_arrows/TrajectoryUtils.h"

#include "tut_arrows_msgs/FlippersModeCmd.h"

class RemoteController
{
public:
	RemoteController();
	void run();
	void onRcWrench(const geometry_msgs::WrenchStamped::ConstPtr rcWrench);
	void onOdometry(nav_msgs::Odometry::ConstPtr odom);
	void onFlippersModeChange(tut_arrows_msgs::FlippersModeCmd::ConstPtr mode);
	void changeDepth(std_msgs::Float32::ConstPtr depth);

private:
	void control();

	double getRcWrenchDelay() const;
	double getOdomDelay() const;

private:
	ros::NodeHandle nh_;
	double duration_;

	ros::Publisher wrenchPub_;

	ros::Subscriber rcWrenchSub_;
	ros::Subscriber odomSub_;
	ros::Subscriber wrenchModeSub_;
	ros::Subscriber depthSub_;

	geometry_msgs::WrenchStamped::ConstPtr lastRcWrench_;
	nav_msgs::Odometry::ConstPtr odom_;

	geometry_msgs::Pose targetPose_;

	double rcWrenchTimeout_;
	double odomTimeout_;

	std::shared_ptr<IController> depthController_;
	std::shared_ptr<IController> rollController_;
	std::shared_ptr<IController> pitchController_;

	double scaleX_;
	double scaleY_;
	double scaleZ_;
	double scaleRoll_;
	double scalePitch_;
	double scaleYaw_;
};

RemoteController::RemoteController() :
	nh_(),
	duration_(0.1), rcWrenchTimeout_(2.0), odomTimeout_(2.0),
	scaleX_(6.2), scaleY_(3.6), scaleZ_(7.2), scaleRoll_(7.2), scalePitch_(7.2), scaleYaw_(7.2)
{
	ros::NodeHandle nhPrivate("~");

	std::string depthControllerConf;
	nhPrivate.getParam("depth_controller", depthControllerConf);
	depthController_ = ControllerFactory::getControllerFromString(depthControllerConf);
	if (depthController_)
	{
		ROS_INFO("Depth controller: %s", depthController_->toString().c_str());
	}

	std::string rollControllerConf;
	nhPrivate.getParam("roll_controller", rollControllerConf);
	rollController_ = ControllerFactory::getControllerFromString(rollControllerConf);
	if (rollController_)
	{
		ROS_INFO("Roll controller: %s", rollController_->toString().c_str());
	}

	std::string pitchControllerConf;
	nhPrivate.getParam("pitch_controller", pitchControllerConf);
	pitchController_ = ControllerFactory::getControllerFromString(pitchControllerConf);
	if (pitchController_)
	{
		ROS_INFO("Pitch controller: %s", pitchController_->toString().c_str());
	}

	wrenchPub_ = nh_.advertise<geometry_msgs::WrenchStamped>("force_req", 1);

	rcWrenchSub_ = nh_.subscribe("rc_force_req", 1, &RemoteController::onRcWrench, this);
	odomSub_ = nh_.subscribe("odom", 1, &RemoteController::onOdometry, this);
	depthSub_ = nh_.subscribe("depth_setpoint", 1, &RemoteController::changeDepth, this);
	wrenchModeSub_ = nh_.subscribe("force_mode", 1, &RemoteController::onFlippersModeChange, this);

	targetPose_.position.x = 0.0;
	targetPose_.position.y = 0.0;
	targetPose_.position.z = 0.0;
	targetPose_.orientation.x = 0.0;
	targetPose_.orientation.y = 0.0;
	targetPose_.orientation.z = 0.0;
	targetPose_.orientation.w = 1.0;
}

void RemoteController::control()
{
	geometry_msgs::WrenchStamped currentWrench;
	currentWrench.header.stamp = ros::Time::now();
	currentWrench.header.frame_id = "/ucat0";

	if ( odom_ && ( getOdomDelay() < odomTimeout_ ) )
	{
		const Eigen::Matrix<double, 6, 1> poseError = TrajectoryUtils::getPoseError(odom_->pose.pose, targetPose_);

		if (depthController_)
		{
			currentWrench.wrench.force.z = 0.5 *
					scaleZ_ * MathUtils::limitValue(depthController_->update(poseError[2], duration_), -1.0, 1.0);
		}

		if (rollController_)
		{
			currentWrench.wrench.torque.x = 0.5 *
					scaleRoll_ * MathUtils::limitValue(rollController_->update(poseError[3], duration_), -1.0, 1.0);
		}

		if (pitchController_)
		{
			currentWrench.wrench.torque.y = 0.5 *
					scalePitch_ * MathUtils::limitValue(pitchController_->update(poseError[4], duration_), -1.0, 1.0);
		}
	}

	if ( lastRcWrench_ && ( getRcWrenchDelay() < rcWrenchTimeout_ ) )
	{
		if (std::abs(lastRcWrench_->wrench.force.x) > 0)
		{
			currentWrench.wrench.force.x = scaleX_ * lastRcWrench_->wrench.force.x;
		}

		if (std::abs(lastRcWrench_->wrench.force.y) > 0)
		{
			currentWrench.wrench.force.y = scaleY_ * lastRcWrench_->wrench.force.y;
		}

		if (std::abs(lastRcWrench_->wrench.force.z) > 0)
		{
			currentWrench.wrench.force.z = scaleZ_ * lastRcWrench_->wrench.force.z;
		}

		if (std::abs(lastRcWrench_->wrench.torque.x) > 0)
		{
			currentWrench.wrench.torque.x = scaleRoll_ * lastRcWrench_->wrench.torque.x;
		}

		if (std::abs(lastRcWrench_->wrench.torque.y) > 0)
		{
			currentWrench.wrench.torque.y = scalePitch_ * lastRcWrench_->wrench.torque.y;
		}

		if (std::abs(lastRcWrench_->wrench.torque.z) > 0)
		{
			currentWrench.wrench.torque.z = scaleYaw_ * lastRcWrench_->wrench.torque.z;
		}
	}

	wrenchPub_.publish(currentWrench);
}

void RemoteController::run()
{
	ros::Duration duration(duration_);
	while (ros::ok())
	{
		ros::spinOnce();
		control();
		duration.sleep();
	}
}

void RemoteController::onRcWrench(geometry_msgs::WrenchStamped::ConstPtr rcWrench)
{
	lastRcWrench_ = rcWrench;
}

void RemoteController::onOdometry(nav_msgs::Odometry::ConstPtr odom)
{
	odom_ = odom;
}

double RemoteController::getRcWrenchDelay() const
{
	return lastRcWrench_ ? (ros::Time::now() - lastRcWrench_->header.stamp).toSec() : -1;
}

void RemoteController::onFlippersModeChange(const tut_arrows_msgs::FlippersModeCmd::ConstPtr mode)
{
	if (mode->mode == mode->MODE_FAST)
	{
		scaleX_ = 12.0;
		scaleY_ = 7.0;
		scaleZ_ = 14.0;
		scaleRoll_ = 14.0;
		scalePitch_ = 14.0;
		scaleYaw_ = 14.0;
	}
	else if (mode->mode == mode->MODE_SLOW)
	{
		scaleX_ = 6.2;
		scaleY_ = 3.6;
		scaleZ_ = 7.2;
		scaleRoll_ = 7.2;
		scalePitch_ = 7.2;
		scaleYaw_ = 7.2;
	}
}

double RemoteController::getOdomDelay() const
{
	return odom_ ? (ros::Time::now() - odom_->header.stamp).toSec() : -1;
}

void RemoteController::changeDepth(std_msgs::Float32::ConstPtr depth)
{
	targetPose_.position.z = depth->data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "RemoteController");
	RemoteController RemoteController;
	RemoteController.run();
}
