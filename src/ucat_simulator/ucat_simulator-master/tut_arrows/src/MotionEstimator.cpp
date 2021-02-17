#include "tut_arrows/MotionEstimator.h"
#include "tut_arrows/FluidPressure.h"
#include "tut_arrows/QuatUtils.h"

#include <stdexcept>
#include <cmath>

namespace
{
	static double DensityOfWater = 999.1285; // + 15C
	static double DragX = 0.042411501; // 0.3 * (3.14 * 0.30^2) / 2
	static double DragY = 0.144; // 0.8 * (0.6 * 0.3)
	static double DragZ = 0.144; // 0.8 * (0.6 * 0.3)

	template <class T>
	T getRPY(const tf::Quaternion& quaternion)
	{
		double roll, pitch, yaw;
		tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
		return T(roll, pitch, yaw);
	}

	template <class T>
	T getRPY(const geometry_msgs::Quaternion& quaternion)
	{
		return getRPY<T>(tf::Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w));
	}

	template <class T>
	geometry_msgs::Quaternion toQuaternionMsg(const T& rpy)
	{
		tf::Quaternion q;
		q.setRPY(rpy.x(), rpy.y(), rpy.z());
		geometry_msgs::Quaternion qMsg;
		qMsg.x = q.x();
		qMsg.y = q.y();
		qMsg.z = q.z();
		qMsg.w = q.w();
		return qMsg;
	}

	/// Normalize angle to range [-PI ... PI].
	///
	template <class T>
	T normalizeAngle(T angle)
	{
		T x = fmod(angle + M_PI, 2.0 * M_PI);
		if (x < 0)
		{
			x += 2.0 * M_PI;
		}
		return x - M_PI;
	}
}

MotionEstimator::MotionEstimator() : 
	nh_(),
	rate_(10),
	forceXCoef_(1.0), forceYCoef_(1.0), torqueZCoef_(1.0),
	mass_(20.0), velocity_(0.0, 0.0, 0.0),
	ekf_(),
	dynamics_(),
	wrenchTimeout_(5.0), imuTimeout_(2.0),
	lastWrench_(), lastImu_(), lastFluidPressure_(),
	pressureAtSurface_(-1.0),
	doubleRangeMeasurementTimeout_(10.0)
{
	ros::NodeHandle nhPrivate("~"); // private parameters

	nhPrivate.getParam("rate", rate_);

	nhPrivate.getParam("wrench_timeout", wrenchTimeout_);
	nhPrivate.getParam("imu_timeout", imuTimeout_);

	nhPrivate.getParam("mass", mass_);

	double x = 0.0;
	nhPrivate.getParam("position_x", x);

	double y = 0.0;
	nhPrivate.getParam("position_y", y);

	double z = 0.0;
	nhPrivate.getParam("position_z", z);

	double roll = 0.0;
	nhPrivate.getParam("roll", roll);

	double pitch = 0.0;
	nhPrivate.getParam("pitch", pitch);

	double yaw = 0.0;
	nhPrivate.getParam("yaw", yaw);

	setPose(x, y, z, roll, pitch, yaw);

	parseBeacons(nhPrivate);

	double measurementVariance = 0.0;
	if (nhPrivate.getParam("measurementVariance", measurementVariance))
	{
		ekf_.setMeasurementVariance(measurementVariance);
	}

	double rangeMeasurementVariance = 0.0;
	if (nhPrivate.getParam("rangeMeasurementVariance", rangeMeasurementVariance))
	{
		ekf_.setRangeMeasurementVariance(rangeMeasurementVariance);
	}

	nhPrivate.getParam("doubleRangeMeasurementTimeout", doubleRangeMeasurementTimeout_);

	double likelihoodThreshold = 0.0;
	if (nhPrivate.getParam("likelihoodThreshold", likelihoodThreshold))
	{
		ekf_.setLikelihoodThreshold(likelihoodThreshold);
	}

	double alfa1 = 1.0, alfa2 = 1.0, alfa3 = 1.0, alfa4 = 1.0;
	nhPrivate.getParam("alfa1", alfa1);
	nhPrivate.getParam("alfa2", alfa2);
	nhPrivate.getParam("alfa3", alfa3);
	nhPrivate.getParam("alfa4", alfa4);
	ekf_.setMotionVariance(alfa1, alfa2, alfa3, alfa4);

	nhPrivate.getParam("force_x_coef", forceXCoef_);
	nhPrivate.getParam("force_y_coef", forceYCoef_);
	nhPrivate.getParam("torque_z_coef", torqueZCoef_);

	nhPrivate.getParam("surface_pressure", pressureAtSurface_);
	if (pressureAtSurface_ < 0)
	{
		nh_.getParam("surface_pressure", pressureAtSurface_);
	}

	motionCallbackSub_ = nh_.subscribe("wrench", 1, &MotionEstimator::motionCallback, this);
	imuCallbackSub_ = nh_.subscribe("imu", 1, &MotionEstimator::imuCallback, this);
	waterPressureCallbackSub_ = nh_.subscribe("fluid_pressure", 1, &MotionEstimator::fluidPressureCallback, this);
	beaconSub_ = nh_.subscribe<tut_arrows_msgs::BeaconPing>("beacon", 10, &MotionEstimator::beaconPingCallback, this);
	modemRangeSub_ = nh_.subscribe<tut_arrows_msgs::ModemRange>("modem_range", 10, &MotionEstimator::modemRangeCallback, this);

	poseService_ = nh_.advertiseService("set_pose", &MotionEstimator::poseCallback, this);

	odomPub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
}

void MotionEstimator::parseBeacons(ros::NodeHandle nh)
{
	XmlRpc::XmlRpcValue beacons;
	if (nh.getParam("beacons", beacons))
	{
		for (auto b : beacons)
		{
			const std::string& beaconId = b.first;

			std::map<std::string, double> beacon;
			if (nh.getParam("beacons/" + beaconId, beacon))
			{
				const double x = beacon.at("x");
				const double y = beacon.at("y");
				const double z = beacon.at("z");

				ekf_.setCorrespondence(beaconId, Eigen::Vector3d(x, y, z));
			}
		}
	}
}

bool MotionEstimator::poseCallback(tut_arrows_msgs::SetTarget::Request& request, tut_arrows_msgs::SetTarget::Response& response)
{
	const tf::Vector3 orientation = getRPY<tf::Vector3>(request.pose.orientation);
	const geometry_msgs::Point position = request.pose.position;
	setPose(position.x, position.y, position.z, orientation.x(), orientation.y(), orientation.z());
	return true;
}

/// Calculate motion update based on applied force and torque.
///
/// @param wrenchStamped - applied force and torque in local frame (centered at robot)
///
void MotionEstimator::motionCallback(geometry_msgs::WrenchStamped::ConstPtr wrenchStamped)
{
	lastWrench_ = wrenchStamped;
}

void MotionEstimator::imuCallback(sensor_msgs::Imu::ConstPtr imu)
{
	odom_.pose.pose.orientation = imu->orientation;

	//odom_.pose.covariance[21] = imu->orientation_covariance[0]; // roll variance
	//odom_.pose.covariance[22] = imu->orientation_covariance[1]; // roll,pitch covariance
	//odom_.pose.covariance[23] = imu->orientation_covariance[2]; // roll,yaw covariance

	//odom_.pose.covariance[27] = imu->orientation_covariance[3]; // pitch,roll covariance
	//odom_.pose.covariance[28] = imu->orientation_covariance[4]; // pitch variance
	//odom_.pose.covariance[29] = imu->orientation_covariance[5]; // pitch,yaw covariance

	//odom_.pose.covariance[33] = imu->orientation_covariance[6]; // yaw,roll covariance
	//odom_.pose.covariance[34] = imu->orientation_covariance[7]; // yaw,pitch covariance
	//odom_.pose.covariance[35] = imu->orientation_covariance[8]; // yaw variance

	odom_.pose.covariance[21] = 0.05; // roll variance
	odom_.pose.covariance[28] = 0.05; // pitch variance
	odom_.pose.covariance[35] = 0.1; // yaw variance

	odom_.twist.twist.angular = imu->angular_velocity;

	ekf_.setOrientation(getRPY<Eigen::Vector3d>(imu->orientation));

	lastImu_ = imu;
}

void MotionEstimator::fluidPressureCallback(sensor_msgs::FluidPressure::ConstPtr fluidPressure)
{
	// if this is the first fluid pressure measurement then assume that it's at surface and use that value as
	// pressure at surface
	if (!lastFluidPressure_ && pressureAtSurface_ < 0)
	{
		pressureAtSurface_ = fluidPressure->fluid_pressure;

		// surface pressure is stored as global parameter in case this node is restarted when under water
		// then surface value can be restored
		nh_.setParam("surface_pressure", pressureAtSurface_);

		ROS_INFO("Pressure at surface set to %g", pressureAtSurface_);
	}

	if (lastFluidPressure_)
	{
		const double dt = (fluidPressure->header.stamp - lastFluidPressure_->header.stamp).toSec();

		const double zBefore = FluidPressure::getDepth(lastFluidPressure_->fluid_pressure, pressureAtSurface_);
		const double zNow = FluidPressure::getDepth(fluidPressure->fluid_pressure, pressureAtSurface_);
		const double dz = -(zNow - zBefore);

		odom_.pose.pose.position.z = -zNow; // underwater is negative z
		odom_.pose.covariance[14] = FluidPressure::getUnadjustedDepth(fluidPressure->variance);

		odom_.twist.twist.linear.z = dz / dt;

		ekf_.setZ(odom_.pose.pose.position.z);
	}

	lastFluidPressure_ = fluidPressure;
}

void MotionEstimator::beaconPingCallback(const tut_arrows_msgs::BeaconPing::ConstPtr& ping)
{
	//ROS_INFO("Beacon ping received");

	const std::string& id = ping->id;
	const Eigen::Vector3d direction(ping->direction.x, ping->direction.y, ping->direction.z);

	ekf_.measurementUpdate(id, direction);

	//ekf_.iteratedMeasurementUpdate(id, direction);
}

void MotionEstimator::modemRangeCallback(const tut_arrows_msgs::ModemRange::ConstPtr& range)
{
	//ROS_INFO("Modem (%s) range received", range->id.c_str());

	modemRanges_[range->id] = range;

	//ekf_.measurementUpdate(range->id, range->range);
}

void MotionEstimator::setPose(double x, double y, double z, double roll, double pitch, double yaw)
{
	ekf_.setPose(Eigen::Vector3d(x, y, z), Eigen::Vector3d(roll, pitch, yaw));

	odom_.pose.pose.position.x = x;
	odom_.pose.pose.position.y = y;
	odom_.pose.pose.position.z = z;

	tf::Quaternion q;
	q.setRPY(roll, pitch, yaw);

	odom_.pose.pose.orientation.x = q.x();
	odom_.pose.pose.orientation.y = q.y();
	odom_.pose.pose.orientation.z = q.z();
	odom_.pose.pose.orientation.w = q.w();
}

Eigen::Vector3d MotionEstimator::getEquilibriumVelocity(const Eigen::Vector3d& force) const
{
	return Eigen::Vector3d(
		std::sqrt(2.0 * std::abs(force.x()) / ( DensityOfWater * DragX ) ) * (force.x() < 0.0 ? -1.0 : 1.0),
		std::sqrt(2.0 * std::abs(force.y()) / ( DensityOfWater * DragY ) ) * (force.y() < 0.0 ? -1.0 : 1.0),
		std::sqrt(2.0 * std::abs(force.z()) / ( DensityOfWater * DragZ ) ) * (force.z() < 0.0 ? -1.0 : 1.0));
}

Eigen::Vector3d MotionEstimator::getWaterDragForce(const Eigen::Vector3d& localVelocity) const
{
	return Eigen::Vector3d(
		0.5 * DensityOfWater * DragX * localVelocity.x() * localVelocity.x() * (localVelocity.x() < 0.0 ? 1.0 : -1.0),
		0.5 * DensityOfWater * DragY * localVelocity.y() * localVelocity.y() * (localVelocity.y() < 0.0 ? 1.0 : -1.0),
		0.5 * DensityOfWater * DragZ * localVelocity.z() * localVelocity.z() * (localVelocity.z() < 0.0 ? 1.0 : -1.0));
}

void MotionEstimator::motionUpdate()
{
	const double dt = 1.0 / rate_;

	Eigen::Vector3d force(0.0, 0.0, 0.0);
	Eigen::Vector3d torque(0.0, 0.0, 0.0);

	ros::Time now = ros::Time::now();

	if (lastWrench_ && (now - lastWrench_->header.stamp).toSec() < wrenchTimeout_)
	{
		force(0) = lastWrench_->wrench.force.x * forceXCoef_;
		force(1) = lastWrench_->wrench.force.y * forceYCoef_;
		//force(2) = lastWrench_->wrench.force.z;

		//torque(0) = lastWrench_->wrench.torque.x;
		//torque(1) = lastWrench_->wrench.torque.y;
		torque(2) = lastWrench_->wrench.torque.z * torqueZCoef_;
	}

	UcatDynamics::Vector3 linearVelocity(odom_.twist.twist.linear.x, odom_.twist.twist.linear.y, odom_.twist.twist.linear.z);
	UcatDynamics::Vector3 angularVelocity = UcatDynamics::Vector3::Zero();
	UcatDynamics::Quaternion orientation(odom_.pose.pose.orientation.w, odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z);

	//UcatDynamics::Vector3 globalFrameLinearVelocity = QuatUtils::quatRotate(orientation, linearVelocity);

	//odom_.pose.pose.position.x += globalFrameLinearVelocity.x() * dt;
	//odom_.pose.pose.position.y += globalFrameLinearVelocity.y() * dt;

	//UcatDynamics::Quaternion orientationChange;
	//orientationChange.coeffs() = QuatUtils::getRotationFromAngularVelocity(orientation, angularVelocity).coeffs() * dt;

	UcatDynamics::Vector6 acceleration = dynamics_.getAcceleration(orientation, linearVelocity, angularVelocity, force, torque);

	//orientation.coeffs() += orientationChange.coeffs();
	//orientation.normalize();

	ekf_.motionUpdate(linearVelocity, angularVelocity, dt);

	linearVelocity += acceleration.head<3>() * dt;
	angularVelocity += acceleration.tail<3>() * dt;

	odom_.twist.twist.linear.x = linearVelocity.x();
	odom_.twist.twist.linear.y = linearVelocity.y();
	//odom_.twist.twist.linear.z = linearVelocity.z();

	//odom_.twist.twist.angular.x = angularVelocity.x();
	//odom_.twist.twist.angular.y = angularVelocity.y();
	//odom_.twist.twist.angular.z = angularVelocity.z();
}

/// update x,y coordinates and yaw angle
///
void MotionEstimator::ekfUpdateOdom()
{
	//
	// Position (x,y)
	//
	const Eigen::Vector3d& ekfPosEstimate = ekf_.getPositionEstimate();

	odom_.pose.pose.position.x = ekfPosEstimate.x();
	odom_.pose.pose.position.y = ekfPosEstimate.y();


	//
	// Orientation (yaw)
	//
//	const Eigen::Vector3d& ekfOrientationEstimate = ekf_.getOrientationEstimate();
//
//	tf::Vector3 orientation = getRPY<tf::Vector3>(odom_.pose.pose.orientation);
//	orientation.setZ(ekfOrientationEstimate.z());
//
//	odom_.pose.pose.orientation = toQuaternionMsg(orientation);


	//
	// Covariance (x,y,yaw)
	//
	Eigen::Matrix3d cov = ekf_.getCovariance();
	odom_.pose.covariance[0] = cov(0, 0); // x variance
	odom_.pose.covariance[1] = cov(0, 1); // x,y covariance
	odom_.pose.covariance[5] = cov(0, 2); // x,yaw covariance

	odom_.pose.covariance[6] = cov(1, 0); // y,x covariance
	odom_.pose.covariance[7] = cov(1, 1); // y variance
	odom_.pose.covariance[11] = cov(1, 2); // y,yaw covariance

//	odom_.pose.covariance[30] = cov(2, 0); // yaw,x covariance
//	odom_.pose.covariance[31] = cov(2, 1); // yaw,y covariance
//	odom_.pose.covariance[35] = cov(2, 2); // yaw variance
}

void MotionEstimator::checkDoubleRange()
{
	std::vector<tut_arrows_msgs::ModemRange::ConstPtr> updateRanges;

	for (auto& modemRange : modemRanges_)
	{
		if ( (modemRange.second->header.stamp > latestProcessed_)
				&& ((ros::Time::now() - modemRange.second->header.stamp).toSec() < doubleRangeMeasurementTimeout_))
		{
			updateRanges.push_back(modemRange.second);
		}
	}

	if (updateRanges.size() >= 2)
	{
		ROS_INFO("Performing modem range update");

		ekf_.doubleRangeUpdate(updateRanges[0]->id, updateRanges[0]->range, updateRanges[1]->id, updateRanges[1]->range);

		for (auto updateRange : updateRanges)
		{
			if (updateRange->header.stamp > latestProcessed_)
			{
				latestProcessed_ = updateRange->header.stamp;
			}
		}
	}
}

void MotionEstimator::run()
{
	odom_.header.frame_id = "/world";
	odom_.child_frame_id = "/ucat0";

	ros::Rate r(rate_);
	while (ros::ok())
	{
		ros::spinOnce();

		motionUpdate();

		//checkDoubleRange();

		ekfUpdateOdom();

		odom_.header.stamp = ros::Time::now();
		odomPub_.publish(odom_);

		r.sleep();
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "MotionEstimator");
	MotionEstimator motionEstimator;
	motionEstimator.run();
}
