#include <ros/ros.h>
#include <auv_msgs/AcousticModemData.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>

#include "tut_arrows/UcatDynamics.h"
#include "tut_arrows/ekf.h"
#include "tut_arrows_msgs/BeaconPing.h"
#include "tut_arrows_msgs/ModemRange.h"
#include <tut_arrows_msgs/SetTarget.h>

#ifndef TUT_ARROWS_MOTION_ESTIMATOR_H_
#define TUT_ARROWS_MOTION_ESTIMATOR_H_

class MotionEstimator
{
public:
	MotionEstimator();

	bool poseCallback(tut_arrows_msgs::SetTarget::Request& request, tut_arrows_msgs::SetTarget::Response& response);
	void motionCallback(geometry_msgs::WrenchStamped::ConstPtr wrenchStamped);
	void imuCallback(sensor_msgs::Imu::ConstPtr imu);
	void fluidPressureCallback(sensor_msgs::FluidPressure::ConstPtr fluidPressure);
	void beaconPingCallback(const tut_arrows_msgs::BeaconPing::ConstPtr& ping);
	void modemRangeCallback(const tut_arrows_msgs::ModemRange::ConstPtr& range);

	void run();

private:
	void motionUpdate();
	void setPose(double x, double y, double z, double roll, double pitch, double yaw);

	void checkDoubleRange();

	Eigen::Vector3d getEquilibriumVelocity(const Eigen::Vector3d& force) const;
	Eigen::Vector3d getWaterDragForce(const Eigen::Vector3d& localVelocity) const;

	void ekfUpdateOdom();

	void parseBeacons(ros::NodeHandle nh);

private:
	ros::NodeHandle nh_;

	ros::Subscriber motionCallbackSub_;
	ros::Subscriber imuCallbackSub_;
	ros::Subscriber waterPressureCallbackSub_;
	ros::Subscriber beaconSub_;
	ros::Subscriber modemRangeSub_;
	ros::ServiceServer poseService_;

	ros::Publisher odomPub_;

	nav_msgs::Odometry odom_;

	double wrenchTimeout_;
	double imuTimeout_;

	geometry_msgs::WrenchStamped::ConstPtr lastWrench_;
	sensor_msgs::Imu::ConstPtr lastImu_;
	sensor_msgs::FluidPressure::ConstPtr lastFluidPressure_;

	double forceXCoef_;
	double forceYCoef_;
	double torqueZCoef_;

	double mass_;

	double pressureAtSurface_;

	Eigen::Vector3d velocity_;

	int rate_;

	Ekf ekf_;

	UcatDynamics dynamics_;

	double doubleRangeMeasurementTimeout_;
	ros::Time latestProcessed_;
	std::map<std::string, tut_arrows_msgs::ModemRange::ConstPtr> modemRanges_;
};

#endif // TUT_ARROWS_MOTION_ESTIMATOR_H_
