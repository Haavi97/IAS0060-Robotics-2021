#include "tut_arrows/MainControl.h"

#include <tf/transform_datatypes.h>

#include "tut_arrows/FluidPressure.h"
#include "tut_arrows/QuatUtils.h"
#include "tut_arrows/ControllerFactory.h"
#include "tut_arrows/MathUtils.h"

#include "tut_arrows/TrajectoryUtils.h"

#include "tut_arrows_msgs/Switch.h"

#include <memory>
#include <string>

MainControl::MainControl()
	: nh_()
	, rate_(10.0)
	, mode_(NOOP)
	, buoyanceF_(0.0), buoyanceB_(0.0)
	, scaleX_(10.0), scaleY_(10.0), scaleZ_(10.0)
	, scaleRoll_(1.0), scalePitch_(1.0), scaleYaw_(1.0)
	, odom_()
	, trajectoryPoint_()
	, oa_()
	, rateOfChange_(0.25)
	, fluidPressureChangeTime_(-1)
	, changeWrenchModeNow_(false)
	, wrenchModeChangeTime_(2.0)
	, forceWrenchMode_(false)
	, wrenchMode_(NOT_SET)
	, measureInterval_(-1)
	, measureDuration_(60.0)
	, lastMeasureTime_(ros::Time::now())
	, isMeasuring_(false)
	, saveBeaconData_(false)
	, beaconInterval_(16)
	, beaconDuration_(8)
	, beaconMaxReadings_(3)
	, mission_out_beacon_id_("beacon1")
	, mission_out_speed_(3.0)
{
	ros::NodeHandle nhPrivate("~");
	nhPrivate.getParam("rate", rate_);

	std::string mode = "AUTO";
	nhPrivate.getParam("mode", mode);
	setMode(mode);

	std::string wrenchMode = "FAST";
	if (nhPrivate.getParam("force_wrench_mode", wrenchMode))
	{
		forceWrenchMode_ = true;
	}
	setWrenchMode(wrenchMode);

	nhPrivate.getParam("scale_x", scaleX_);
	nhPrivate.getParam("scale_y", scaleY_);
	nhPrivate.getParam("scale_z", scaleZ_);
	nhPrivate.getParam("scale_roll", scaleRoll_);
	nhPrivate.getParam("scale_pitch", scalePitch_);
	nhPrivate.getParam("scale_yaw", scaleYaw_);

	nhPrivate.getParam("rate_of_change", rateOfChange_);

	nhPrivate.getParam("wrench_mode_change_time", wrenchModeChangeTime_);

	nhPrivate.getParam("measure_interval", measureInterval_);
	nhPrivate.getParam("measure_duration", measureDuration_);

	nhPrivate.getParam("mission_out_beacon_id", mission_out_beacon_id_);
	nhPrivate.getParam("beacon_interval", beaconInterval_);
	nhPrivate.getParam("beacon_duration", beaconDuration_);
	nhPrivate.getParam("beacon_max_readings", beaconMaxReadings_);
	nhPrivate.getParam("mission_out_speed", mission_out_speed_);


	wrench_.header.frame_id = "/ucat0"; // robot fixed coordinates
	wrench_.wrench.force.x = 0;
	wrench_.wrench.force.y = 0;
	wrench_.wrench.force.z = 0;
	wrench_.wrench.torque.x = 0;
	wrench_.wrench.torque.y = 0;
	wrench_.wrench.torque.z = 0;

	fastModeControllers_ = loadModeControllers("FAST");
	slowModeControllers_ = loadModeControllers("SLOW");

	ROS_INFO("Initialized controllers for FAST mode");
	logControllers(fastModeControllers_);

	ROS_INFO("Initialized controllers for SLOW mode");
	logControllers(slowModeControllers_);

	acousticModemSub_ = nh_.subscribe("APPLICON_INBOX", 10, &MainControl::acousticModemCallback, this);
	odomSub_ = nh_.subscribe("odom", 1, &MainControl::odometryCallback, this);
	trajectorySub_ = nh_.subscribe("trajectory", 1, &MainControl::trajectoryCallback, this);

	subBuoyanceF_ = nh_.subscribe("hw/buoyance/f", 1, &MainControl::onBuoyanceF, this);
	subBuoyanceB_ = nh_.subscribe("hw/buoyance/b", 1, &MainControl::onBuoyanceF, this);

	pressureSub_ = nh_.subscribe("fluid_pressure", 1, &MainControl::pressureCallback, this);

	pubBuoyanceF_ = nh_.advertise<geometry_msgs::Vector3Stamped>("hw/buoyance/f_cmd", 1, false);
	pubBuoyanceB_ = nh_.advertise<geometry_msgs::Vector3Stamped>("hw/buoyance/b_cmd", 1, false);

	wrenchPub_ = nh_.advertise<geometry_msgs::WrenchStamped>("force_req", 1);
	wrenchModePub_ = nh_.advertise<tut_arrows_msgs::FlippersModeCmd>("force_mode", 1);

	rcWrenchSub_ = nh_.subscribe("rc_force_req", 1, &MainControl::onRcWrenchStamped, this);

	setControllerSrv_ = nh_.advertiseService("SetController", &MainControl::setController, this);

	modeSub_ = nh_.subscribe("control_mode", 1, &MainControl::onModeChange, this);

	switchModemRangeDriver_ = nh_.serviceClient<tut_arrows_msgs::Switch>("EnableModemRangeDriver");

	beaconSub_ = nh_.subscribe<tut_arrows_msgs::BeaconPing>("hw/beaconReceiver", 10, &MainControl::beaconPingCallback, this);
}

void MainControl::run()
{
	int modePubCounter = 0;

	ros::Rate r(rate_);
	while (ros::ok())
	{
		try
		{
			ros::spinOnce();
			if (mode_ != NOOP)
			{
				// re-publishes mode to WrencDriver periodically (if nothing has changed then it will be ignored by WrenchDriver)
				if ( modePubCounter == 0 || changeWrenchModeNow_ )
				{
					wrenchModeCmd_.header.stamp = ros::Time::now();
					wrenchModePub_.publish(wrenchModeCmd_);
					changeWrenchModeNow_ = false;
				}
				modePubCounter = (modePubCounter + 1) % 100;

				if (checkSafety())
				{
					onSafety();
				}
				else
				{
					updateMeasurementState();
					onControl();
				}
			}
		}
		catch (const std::exception& ex)
		{
			ROS_WARN("MainControl: %s", ex.what());
		}

		r.sleep();
	}
}

void MainControl::acousticModemCallback(const auv_msgs::AcousticModemData::ConstPtr& data)
{
	std::istringstream msgStream(std::string(data->payload.begin(), data->payload.end()));

	std::string cmd, arg;
	msgStream >> cmd >> arg;

	if (cmd == "SET_CONTROL_MODE")
	{
		setMode(arg);
	}
}

void MainControl::onRcWrenchStamped(const geometry_msgs::WrenchStampedConstPtr wrenchMsg)
{
	wrench_ = *wrenchMsg;
	applyCurrentWrench();
}

void MainControl::onSafety()
{
	setWrenchMode(SLOW);

	wrench_.header.stamp = ros::Time::now();
	wrench_.wrench.force.x = 0;
	wrench_.wrench.force.y = 0;
	wrench_.wrench.force.z = scaleZ_;
	wrench_.wrench.torque.x = 0;
	wrench_.wrench.torque.y = 0;
	wrench_.wrench.torque.z = 0;

	applyCurrentWrench();
}

void MainControl::beaconPingCallback(const tut_arrows_msgs::BeaconPing::ConstPtr& ping)
{
	if (saveBeaconData_ && (mission_out_beacon_id_=="" || mission_out_beacon_id_== ping->id))
	{
		savedBeaconPings_.push_back(ping);
	}
}

void MainControl::onControl_MISSION_OUT_saveBeaconData()
{
	const bool sizeCriteriaMet = savedBeaconPings_.size()>=beaconMaxReadings_;
	const bool timeCriteriaMet = (ros::Time::now() - lastBeaconDirectionTime_).toSec() > beaconDuration_;

	if (sizeCriteriaMet || timeCriteriaMet)
	{
		if (savedBeaconPings_.size()>0)
		{
			saveBeaconData_ = false;
			ROS_INFO("Got %lu pings in total", savedBeaconPings_.size());

			lastBeaconDirection_ = geometry_msgs::Vector3();
			for(size_t ii=0;ii<savedBeaconPings_.size();++ii)
			{
				lastBeaconDirection_.x += savedBeaconPings_.at(ii)->direction.x;
				lastBeaconDirection_.y += savedBeaconPings_.at(ii)->direction.y;
				lastBeaconDirection_.z += savedBeaconPings_.at(ii)->direction.z;
			}

			lastBeaconDirection_.x /= savedBeaconPings_.size();
			lastBeaconDirection_.y /= savedBeaconPings_.size();
			lastBeaconDirection_.z /= savedBeaconPings_.size();

			ROS_INFO("Calculated beacon direction: [%.4f,%.4f,%.4f]",
					lastBeaconDirection_.x, lastBeaconDirection_.y, lastBeaconDirection_.z);

			const tf::Vector3 localBeaconDirection(lastBeaconDirection_.x, lastBeaconDirection_.y, lastBeaconDirection_.z);
			const tf::Quaternion orientation = QuatUtils::fromMsg(odom_->pose.pose.orientation).normalize();
			const tf::Vector3 globalBeaconDirection = tf::quatRotate(orientation, localBeaconDirection);

			auto trajectoryPoint = boost::make_shared<tut_arrows_msgs::TrajectoryPoint>();
			trajectoryPoint->header.stamp = ros::Time::now();
			trajectoryPoint->pose.position.x = odom_->pose.pose.position.x + globalBeaconDirection.x() * mission_out_speed_;
			trajectoryPoint->pose.position.y = odom_->pose.pose.position.y + globalBeaconDirection.y() * mission_out_speed_;
			trajectoryPoint->pose.position.z = odom_->pose.pose.position.z + globalBeaconDirection.z() * mission_out_speed_;
			trajectoryPoint->pose.orientation.w = 1.0;
			trajectoryPoint->pose.orientation.x = 0.0;
			trajectoryPoint->pose.orientation.y = 0.0;
			trajectoryPoint->pose.orientation.z = 0.0;

			trajectoryPoint_ = trajectoryPoint;

			ROS_INFO("Target x=%g, y=%g, z=%g", trajectoryPoint->pose.position.x, trajectoryPoint->pose.position.y, trajectoryPoint->pose.position.z);

			savedBeaconPings_.clear();
		}
		else
		{
			ROS_INFO("Waiting for at least one beacon ping");
		}
	}
}

void MainControl::onControl_MISSION_OUT_swimTowardsBeacon()
{
	onControl_AUTO_CONTROL();
}

void MainControl::onControl_MISSION_OUT()
{
	//triggering saving of beacon data
	if ((ros::Time::now() - lastBeaconDirectionTime_).toSec() > beaconInterval_ && !saveBeaconData_)
	{
		setWrenchMode(SLOW);
		ROS_INFO("Start saving beacon data");
		savedBeaconPings_.clear();
		saveBeaconData_ = true;
		lastBeaconDirection_ = geometry_msgs::Vector3();
		applyControl(0,0,0,0,0,0);
		lastBeaconDirectionTime_ = ros::Time::now();
	}

	if (saveBeaconData_)
	{
		onControl_MISSION_OUT_saveBeaconData();
	}
	else
	{
		onControl_MISSION_OUT_swimTowardsBeacon();
	}
}

void MainControl::onControl_AUTO_CONTROL()
{
	// first need to have odometry data available
	if (!odom_)
	{
		ROS_WARN("Odometry data unavailable for control.");
		return;
	}

	const double dt = 1.0 / rate_;

	if (!trajectoryPoint_ || (ros::Time::now() - trajectoryPoint_->header.stamp).toSec() > 30.0)
	{
		return;
	}

	const tf::Quaternion orientation = QuatUtils::fromMsg(odom_->pose.pose.orientation).normalize();

	Eigen::Matrix<double, 6, 1> poseError = TrajectoryUtils::getPoseError(odom_->pose.pose, trajectoryPoint_->pose);

	tf::Vector3 trajectoryVelocityGlobal(trajectoryPoint_->linear_velocity.x, trajectoryPoint_->linear_velocity.y, trajectoryPoint_->linear_velocity.z);
	tf::Vector3 trajectoryAngularVelocityGlobal(trajectoryPoint_->angular_velocity.x, trajectoryPoint_->angular_velocity.y, trajectoryPoint_->angular_velocity.z);

	tf::Vector3 linearVelocityError = TrajectoryUtils::getError(odom_->twist.twist.linear, trajectoryVelocityGlobal);

	tf::Vector3 angularVelocityLocal(odom_->twist.twist.angular.x, odom_->twist.twist.angular.y, odom_->twist.twist.angular.z);
	tf::Vector3 angularVelocityError(0, 0, 0); // = TrajectoryUtils::getError(odom_->twist.twist.angular, trajectoryAngularVelocityGlobal);

	tf::Vector3 positionError(0, 0, 0);
	tf::Vector3 orientationError = tf::Vector3(poseError(3), poseError(4), poseError(5));
	//const tf::Quaternion localOrientationError = orientation.inverse() * orientationError * orientation;

	const tf::Vector3 globalPositionError(poseError(0), poseError(1), poseError(2));
	const tf::Vector3 localPositionError = tf::quatRotate(orientation.inverse(), globalPositionError);
	positionError = tf::Vector3(localPositionError.x(), localPositionError.y(), poseError(2));

	//ROS_INFO("pose error (%g, %g, %g), (%g, %g, %g)", poseError.position.x, poseError.position.y, poseError.position.z, rollError, pitchError, yawError);

	if (!forceWrenchMode_)
	{
		if (std::abs(positionError.x()) > 1.0 || std::abs(positionError.y()) > 1.0)
		{
			positionError.setY(0.0);

			double yawTarget = std::atan2(globalPositionError.y(), globalPositionError.x());

			double roll, pitch, yaw;
			tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

			orientationError.setZ(QuatUtils::getSmallestRotation(yawTarget, yaw));

			if (std::abs(orientationError.z()) > 1.0 || std::abs(positionError.z()) > 0.5)
			{
				setWrenchMode(SLOW);

				positionError.setX(0);
			}
			else
			{
				setWrenchMode(FAST);

				if (positionError.x() < 0)
				{
					positionError.setX(0);
				}
			}
		}
		else
		{
			positionError.setX(0);
			positionError.setY(0);
			orientationError.setX(0);
			orientationError.setY(0);
			setWrenchMode(SLOW);
		}
	}

	if (isMeasuring_)
	{
		positionError.setX(0);
		positionError.setY(0);
		orientationError.setX(0);
		orientationError.setY(0);
		orientationError.setZ(0);
		setWrenchMode(SLOW);
	}

	onControlUpdate(positionError, orientationError, dt);
}

void MainControl::onControl()
{
	switch (mode_)
	{
	case AUTO_CONTROL:
		onControl_AUTO_CONTROL();
		break;
	case MISSION_OUT:
		onControl_MISSION_OUT();
		break;
	}
}

void MainControl::onControlUpdate(const tf::Vector3& positionError, const tf::Vector3& orientationError, double dt)
{
	ModeControllers& currentControllers = getCurrentControllers();

	double controlX = 0.0;
	if (currentControllers.x)
	{
		controlX = currentControllers.x->update(positionError.x(), dt);
	}

	double controlY = 0.0;
	if (currentControllers.y)
	{
		controlY = currentControllers.y->update(positionError.y(), dt);
	}

	double controlZ = 0.0;
	if (currentControllers.z)
	{
		controlZ = currentControllers.z->update(positionError.z(), dt);
	}

	double controlRoll = 0.0;
	if (currentControllers.roll)
	{
		controlRoll = currentControllers.roll->update(orientationError.x(), dt);
	}

	double controlPitch = 0.0;
	if (currentControllers.pitch)
	{
		controlPitch = currentControllers.pitch->update(orientationError.y(), dt);
	}

	double controlYaw = 0.0;
	if (currentControllers.yaw)
	{
		controlYaw = currentControllers.yaw->update(orientationError.z(), dt);
	}

	controlX = scaleX_ * MathUtils::limitValue(controlX, -1.0, 1.0);
	controlY = scaleY_ * MathUtils::limitValue(controlY, -1.0, 1.0);
	controlZ = scaleZ_ * MathUtils::limitValue(controlZ, -1.0, 1.0);
	controlRoll = scaleRoll_  * MathUtils::limitValue(controlRoll, -1.0, 1.0);
	controlPitch = scalePitch_ * MathUtils::limitValue(controlPitch, -1.0, 1.0);
	controlYaw = scaleYaw_   * MathUtils::limitValue(controlYaw, -1.0, 1.0);

	applyControl(controlX, controlY, controlZ, controlRoll, controlPitch, controlYaw);
}

void MainControl::applyControl(double controlX, double controlY, double controlZ, double controlRoll, double controlPitch, double controlYaw)
{
	const double dt = 1.0 / rate_;

	const double rateOfChangePct = rateOfChange_ * dt;
	const double ratePctX = rateOfChangePct * scaleX_;
	const double ratePctY = rateOfChangePct * scaleY_;
	const double ratePctZ = rateOfChangePct * scaleZ_;
	const double ratePctRoll = rateOfChangePct * scaleRoll_;
	const double ratePctPitch = rateOfChangePct * scalePitch_;
	const double ratePctYaw = rateOfChangePct * scaleYaw_;

	wrench_.header.stamp = ros::Time::now();
	wrench_.wrench.force.x = wrench_.wrench.force.x + MathUtils::limitValue(controlX - wrench_.wrench.force.x, -ratePctX, ratePctX);
	wrench_.wrench.force.y = wrench_.wrench.force.y + MathUtils::limitValue(controlY - wrench_.wrench.force.y, -ratePctY, ratePctY);
	wrench_.wrench.force.z = wrench_.wrench.force.z + MathUtils::limitValue(controlZ - wrench_.wrench.force.z, -ratePctZ, ratePctZ);
	wrench_.wrench.torque.x = wrench_.wrench.torque.x + MathUtils::limitValue(controlRoll - wrench_.wrench.torque.x, -ratePctRoll, ratePctRoll);
	wrench_.wrench.torque.y = wrench_.wrench.torque.y + MathUtils::limitValue(controlPitch - wrench_.wrench.torque.y, -ratePctPitch, ratePctPitch);
	wrench_.wrench.torque.z = wrench_.wrench.torque.z + MathUtils::limitValue(controlYaw - wrench_.wrench.torque.z, -ratePctYaw, ratePctYaw);

	applyCurrentWrench();
}

void MainControl::applyCurrentWrench()
{
	if ((ros::Time::now() - lastWrenchModeChangeTime_).toSec() < wrenchModeChangeTime_)
	{
		return;
	}

	if (oa_.enabled())
	{
		oa_.applyOA(wrench_.wrench);
	}

	wrench_.wrench.force.z -= getBuoyance(); //keep Z forces in balance

	normalizeWrench();

	wrenchPub_.publish(wrench_);
}

void MainControl::setMode(Mode mode)
{
	switch (mode)
	{
	case AUTO_CONTROL:
		ROS_INFO("Auto control mode activated.");
		break;
	case NOOP:
		ROS_INFO("Auto control deactivated.");
		break;
	case MISSION_OUT:
		ROS_INFO("Mission out mode activated.");
		break;
	default:
		ROS_WARN("Invalid control mode");
		return;
	}

	mode_ = mode;
}

void MainControl::setMode(const std::string& mode)
{
	if (mode == "STOP")
	{
		ROS_INFO("Stop command received.");
		setMode(NOOP);
	}
	else if (mode == "AUTO")
	{
		ROS_INFO("Auto control mode command received.");
		setMode(AUTO_CONTROL);
	}
	else if (mode == "OUT")
	{
		ROS_INFO("Mission out mode command received.");
		setMode(MISSION_OUT);
	}
	else
	{
		ROS_WARN("Invalid control mode command: %s", mode.c_str());
	}
}

void MainControl::setWrenchMode(WrenchMode wrenchMode)
{
	if (wrenchMode == wrenchMode_)
	{
		return;
	}

	switch (wrenchMode)
	{
	case FAST:
		wrenchModeCmd_.mode = "FAST";
		break;
	case SLOW:
		wrenchModeCmd_.mode = "SLOW";
		break;
	}

	//ROS_INFO("wrench mode changing");

	wrenchModeCmd_.mode_change_time = wrenchModeChangeTime_;

	wrenchMode_ = wrenchMode;
	setWrenchModeScales(wrenchMode);
	changeWrenchModeNow_ = true;
	lastWrenchModeChangeTime_ = ros::Time::now();
}

void MainControl::setWrenchMode(const std::string& wrenchMode)
{
	if (wrenchMode == "SLOW")
	{
		setWrenchMode(SLOW);
	}
	else if (wrenchMode == "FAST")
	{
		setWrenchMode(FAST);
	}
	else
	{
		ROS_WARN("Invalid wrench mode command: %s", wrenchMode.c_str());
	}
}

void MainControl::setWrenchModeScales(WrenchMode wrenchMode)
{
	if (wrenchMode_ == FAST)
	{
		scaleX_ = 12.0;
		scaleY_ = 7.0;
		scaleZ_ = 14.0;
		scaleRoll_ = 14.0;
		scalePitch_ = 14.0;
		scaleYaw_ = 14.0;
	}
	else if (wrenchMode_ == SLOW)
	{
		scaleX_ = 6.2;
		scaleY_ = 3.6;
		scaleZ_ = 7.2;
		scaleRoll_ = 7.2;
		scalePitch_ = 7.2;
		scaleYaw_ = 7.2;
	}
}

void MainControl::normalizeWrench()
{
	float maxForce = 4.0 * 3.5f;

	if (wrenchMode_ == SLOW)
	{
		maxForce = 4.0 * 1.8f;
	}

	Eigen::VectorXf wrench(6);
	wrench << wrench_.wrench.force.x, wrench_.wrench.force.y, wrench_.wrench.force.z, wrench_.wrench.torque.x, wrench_.wrench.torque.y, wrench_.wrench.torque.z;

	const float totalForce = wrench.norm();

	if (totalForce > maxForce)
	{
		const float normalizer = totalForce / maxForce;
		wrench /= normalizer;
	}

	wrench_.wrench.force.x = wrench(0);
	wrench_.wrench.force.y = wrench(1);
	wrench_.wrench.force.z = wrench(2);
	wrench_.wrench.torque.x = wrench(3);
	wrench_.wrench.torque.y = wrench(4);
	wrench_.wrench.torque.z = wrench(5);
}

void MainControl::onModeChange(const std_msgs::StringConstPtr mode)
{
	setMode(mode->data);
}

void MainControl::odometryCallback(nav_msgs::Odometry::ConstPtr data)
{
	odom_ = data;
}

void MainControl::trajectoryCallback(tut_arrows_msgs::TrajectoryPoint::ConstPtr data)
{
	trajectoryPoint_ = data;
}

void MainControl::onBuoyanceF(const geometry_msgs::Vector3StampedConstPtr data)
{
	buoyanceF_ = data->vector.z;
}

void MainControl::onBuoyanceB(const geometry_msgs::Vector3StampedConstPtr data)
{
	buoyanceB_ = data->vector.z;
}

double MainControl::getBuoyance() const
{
	return buoyanceF_ + buoyanceB_;
}

void MainControl::setBuoyance(double buoyance)
{
	ros::Time now = ros::Time::now();

	geometry_msgs::Vector3Stamped vf;
	vf.header.stamp = now;
	vf.vector.z = buoyance/2;

	geometry_msgs::Vector3Stamped vb;
	vb.header.stamp = now;
	vb.vector.z = buoyance/2;

	pubBuoyanceF_.publish(vf);
	pubBuoyanceB_.publish(vb);
}

bool MainControl::setController(tut_arrows_msgs::SetController::Request& request, tut_arrows_msgs::SetController::Response& response)
{
	std::shared_ptr<IController> ctrlr = ControllerFactory::getControllerFromString(request.controller);

	if (!ctrlr)
	{
		ROS_INFO("Unable to parse controller: %s", request.controller.c_str());
		return false;
	}

	ModeControllers& currentControllers = getCurrentControllers();

	switch (request.controller_type)
	{
	case tut_arrows_msgs::SetController::Request::CONTROLLER_X:
		currentControllers.x = ctrlr;
		break;
	case tut_arrows_msgs::SetController::Request::CONTROLLER_Y:
		currentControllers.y = ctrlr;
		break;
	case tut_arrows_msgs::SetController::Request::CONTROLLER_Z:
		currentControllers.z = ctrlr;
		break;
	case tut_arrows_msgs::SetController::Request::CONTROLLER_YAW:
		currentControllers.yaw = ctrlr;
		break;
	default:
		ROS_INFO("Unrecognized controller type: %d", request.controller_type);
		return false;
	}

	ROS_INFO("Set new controller %s", ctrlr->toString().c_str());

	return true;
}

void MainControl::pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& fluidPressure)
{
	if (lastFluidPressure_)
	{
		if (fluidPressure->fluid_pressure != lastFluidPressure_->fluid_pressure)
		{
			fluidPressureChangeTime_ = fluidPressure->header.stamp.toSec();
		}
	}

	lastFluidPressure_ = fluidPressure;
}

bool MainControl::checkSafety()
{
	if (fluidPressureChangeTime_ > 0 && (ros::Time::now().toSec() - fluidPressureChangeTime_) > 5.0)
	{
		ROS_ERROR("Pressure sensor failure, going back to surface!");
		return true;
	}

	return false;
}

std::shared_ptr<IController> MainControl::loadModeController(const std::string& mode, const std::string& controllerType)
{
	std::shared_ptr<IController> controller = ControllerFactory::getController(std::string("~/") + mode + "/" + controllerType);
	if (!controller)
	{
		// if no controller in specific namespace then try loading from default namespace
		controller = ControllerFactory::getController(std::string("~/") + controllerType);

		if (controller)
		{
			ROS_INFO("Found default %s", controllerType.c_str());
		}
	}

	if (controller)
	{
		ROS_INFO("%s: %s", controllerType.c_str(), controller->toString().c_str());
	}

	return controller;
}

ModeControllers MainControl::loadModeControllers(const std::string& mode)
{
	ROS_INFO("Initializing controllers for %s mode", mode.c_str());

	ModeControllers modeControllers;
	modeControllers.x = loadModeController(mode, "x_controller");
	modeControllers.y = loadModeController(mode, "y_controller");
	modeControllers.z = loadModeController(mode, "z_controller");
	modeControllers.roll = loadModeController(mode, "roll_controller");
	modeControllers.pitch = loadModeController(mode, "pitch_controller");
	modeControllers.yaw = loadModeController(mode, "yaw_controller");
	return modeControllers;
}

void MainControl::logControllers(const ModeControllers& modeControllers) const
{
	if (modeControllers.x)
	{
		ROS_INFO("X: %s", modeControllers.x->toString().c_str());
	}
	if (modeControllers.y)
	{
		ROS_INFO("Y: %s", modeControllers.y->toString().c_str());
	}
	if (modeControllers.z)
	{
		ROS_INFO("Z: %s", modeControllers.z->toString().c_str());
	}
	if (modeControllers.roll)
	{
		ROS_INFO("Roll: %s", modeControllers.roll->toString().c_str());
	}
	if (modeControllers.pitch)
	{
		ROS_INFO("Pitch: %s", modeControllers.pitch->toString().c_str());
	}
	if (modeControllers.yaw)
	{
		ROS_INFO("Yaw: %s", modeControllers.yaw->toString().c_str());
	}
}

ModeControllers& MainControl::getCurrentControllers()
{
	if (wrenchMode_ == SLOW)
	{
		return slowModeControllers_;
	}
	else if (wrenchMode_ == FAST)
	{
		return fastModeControllers_;
	}
	else
	{
		ROS_ERROR("Invalid wrench mode");
		return slowModeControllers_;
	}
}

void MainControl::updateMeasurementState()
{
	if (measureInterval_ > 0 && measureDuration_ > 0)
	{
		const double timeSinceLastMeasure = (ros::Time::now() - lastMeasureTime_).toSec();

		if ( (timeSinceLastMeasure > measureInterval_) && (timeSinceLastMeasure < (measureInterval_ + measureDuration_) ) )
		{
			if (!isMeasuring_)
			{
				ROS_INFO("Starting measurement");

				tut_arrows_msgs::Switch enableDriver;
				enableDriver.request.enable = true;
				switchModemRangeDriver_.call(enableDriver);

				isMeasuring_ = true;
			}
		}
		else if ( timeSinceLastMeasure > (measureInterval_ + measureDuration_) )
		{
			ROS_INFO("Stopping measurement");

			lastMeasureTime_ = ros::Time::now();

			tut_arrows_msgs::Switch disableDriver;
			disableDriver.request.enable = false;
			switchModemRangeDriver_.call(disableDriver);

			isMeasuring_ = false;
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "MainControl");
	MainControl mainControl;
	mainControl.run();
}
