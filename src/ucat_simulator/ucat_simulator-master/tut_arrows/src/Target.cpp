#include "tut_arrows/Target.h"

Target::Target(const geometry_msgs::Pose& targetPose)
	: type_(Step)
	, targetPose_(targetPose)
	, duration_(-1.0)
	, startTime_(-1.0)
{
	if (!isValidPose(targetPose_))
	{
		throw std::runtime_error("Invalid pose");
	}
}

Target::Target(Type type, const geometry_msgs::Pose& targetPose, double duration)
	: type_(type)
	, targetPose_(targetPose)
	, duration_(duration)
	, startTime_(-1.0)
{
	if (!isValidPose(targetPose_))
	{
		throw std::runtime_error("Invalid pose");
	}
}

Target::Type Target::getType() const
{
	return type_;
}

void Target::setType(const std::string& type)
{
	if (type == "step")
	{
		type_ = Step;
	}
	else if (type == "linear")
	{
		type_ = Linear;
	}
	else if (type == "tanh")
	{
		type_ = Tanh;
	}
	else if (type == "hermite")
	{
		type_ = Hermite;
	}
	else if (type == "explore")
	{
		type_ = Explore;
	}
	else if (type == "beacon_homing")
	{
		type_ = BeaconHoming;
	}
}

double Target::getDuration() const
{
	return duration_;
}

void Target::setDuration(double duration)
{
	if (duration > 0.0)
	{
		duration_ = duration;
	}
}

bool Target::isElapsed()
{
	return (ros::Time::now().toSec() - startTime_) > duration_;
}

const geometry_msgs::Pose& Target::getTarget() const
{
	return targetPose_;
}

bool Target::isValidPose(const geometry_msgs::Pose& pose)
{
	return pose.orientation.w != 0 || pose.orientation.x != 0 || pose.orientation.y != 0 || pose.orientation.z != 0;
}

/// Format: "x y z roll pitch yaw [type duration]", where duration and type are optional.
///
std::shared_ptr<Target> Target::parse(const std::string& targetStr)
{
	std::stringstream targetStream(targetStr);

	double x, y, z, roll, pitch, yaw;
	targetStream >> x >> y >> z >> roll >> pitch >> yaw;

	geometry_msgs::Pose pose;

	pose.position.x = x;
	pose.position.y = y;
	pose.position.z = z;

	pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

	std::shared_ptr<Target> target = std::make_shared<Target>(pose);

	std::string type;
	targetStream >> type;

	if (!targetStream.fail())
	{
		target->setType(type);
	}

	double duration = -1.0; // if less than 0 then duration not defined
	targetStream >> duration;

	if (!targetStream.fail())
	{
		target->setDuration(duration);
	}

	return target;
}

void Target::start(const geometry_msgs::Pose& currentPose, double currentTime)
{
	startPose_ = currentPose;
	startTime_ = currentTime;
}

geometry_msgs::Pose Target::getDesired(double currentTime) const
{
	const double timePct = std::min( (currentTime - startTime_) / duration_, 1.0 );

	switch (type_)
	{
	case Linear:
		return getDesiredLinear(timePct);
	case Tanh:
		return getDesiredTanh(timePct);
	case Hermite:
		return getDesiredHermite(timePct);
	case Step:
		// no break - default behavior is same as Step
	default:
		return targetPose_;
	}
}

Eigen::Matrix<double, 6, 1> Target::getDesiredVelocity(double currentTime) const
{
	static double delta = 0.1;
	geometry_msgs::Pose p1 = getDesired(currentTime);
	geometry_msgs::Pose p2 = getDesired(currentTime + delta);

	Eigen::Vector3d linearVelocity(p2.position.x - p1.position.x, p2.position.y - p1.position.y, p2.position.z - p1.position.z);
	linearVelocity /= delta;

	double p1Roll, p1Pitch, p1Yaw;
	tf::Matrix3x3(QuatUtils::fromMsg(p1.orientation)).getRPY(p1Roll, p1Pitch, p1Yaw);

	double p2Roll, p2Pitch, p2Yaw;
	tf::Matrix3x3(QuatUtils::fromMsg(p2.orientation)).getRPY(p2Roll, p2Pitch, p2Yaw);

	Eigen::Vector3d angularVelocity(
			QuatUtils::getSmallestRotation(p2Roll, p1Roll),
			QuatUtils::getSmallestRotation(p2Pitch, p1Pitch),
			QuatUtils::getSmallestRotation(p2Yaw, p1Yaw));

	angularVelocity /= delta;

	Eigen::Matrix<double, 6, 1> velocity;
	velocity.block<3, 1>(0, 0) = linearVelocity;
	velocity.block<3, 1>(3, 0) = angularVelocity;

	return velocity;
}

Eigen::Matrix<double, 6, 1> Target::getDesiredAcceleration(double currentTime) const
{
	static double delta = 0.1;
	Eigen::Matrix<double, 6, 1> v1 = getDesiredVelocity(currentTime);
	Eigen::Matrix<double, 6, 1> v2 = getDesiredVelocity(currentTime + delta);

	Eigen::Matrix<double, 6, 1> acceleration;
	acceleration = ( v2 - v1 ) / delta;
	return acceleration;
}

geometry_msgs::Pose Target::getDesiredLinear(double timePct) const
{
	geometry_msgs::Pose target;
	target.position.x = (targetPose_.position.x - startPose_.position.x) * timePct + startPose_.position.x;
	target.position.y = (targetPose_.position.y - startPose_.position.y) * timePct + startPose_.position.y;
	target.position.z = (targetPose_.position.z - startPose_.position.z) * timePct + startPose_.position.z;
	target.orientation = QuatUtils::slerp(targetPose_.orientation, startPose_.orientation, timePct);

	return target;
}

/// Desired pose varies according to function (tanh(4 * x - 2) + 1) / 2 from start pose to target pose.
///
/// Input range from start time to end time corresponds to x varying from 0...1.
/// Output of the shifted tanh function varies also in the range from 0...1 when input is in specified range (0...1).
///
geometry_msgs::Pose Target::getDesiredTanh(double timePct) const
{
	const double pct = 6.0 * std::pow(timePct, 5) - 15.0 * std::pow(timePct, 4) + 10.0 * std::pow(timePct, 3);
	//const double pct = ((std::tanh(4.0 * timePct - 2.0) + 1.0) / 2.0 - 0.01798620996209148) / 0.9640275800758169;
	geometry_msgs::Pose target;
	target.position.x = (targetPose_.position.x - startPose_.position.x) * pct + startPose_.position.x;
	target.position.y = (targetPose_.position.y - startPose_.position.y) * pct + startPose_.position.y;
	target.position.z = (targetPose_.position.z - startPose_.position.z) * pct + startPose_.position.z;
	target.orientation = QuatUtils::slerp(targetPose_.orientation, startPose_.orientation, pct);

	return target;
}

/// Desired pose varies according to cubic Hermite spline from 0 to 1 with tangent at both ends set to 0.
///
geometry_msgs::Pose Target::getDesiredHermite(double timePct) const
{
	const double hermitePct = -2.0 * timePct * timePct * timePct + 3 * timePct * timePct;
	geometry_msgs::Pose target;
	target.position.x = (targetPose_.position.x - startPose_.position.x) * hermitePct + startPose_.position.x;
	target.position.y = (targetPose_.position.y - startPose_.position.y) * hermitePct + startPose_.position.y;
	target.position.z = (targetPose_.position.z - startPose_.position.z) * hermitePct + startPose_.position.z;
	target.orientation = QuatUtils::slerp(targetPose_.orientation, startPose_.orientation, hermitePct);

	return target;
}

std::string Target::toString() const
{
	std::stringstream strStream;
	strStream.precision(2);
	strStream << "Position: (" << targetPose_.position.x << " " << targetPose_.position.y << " " << targetPose_.position.z << ")"
			<< " Orientation: (" << targetPose_.orientation.x << ", " << targetPose_.orientation.y << ", " << targetPose_.orientation.z << ", " << targetPose_.orientation.w << ")";

	if (duration_ > 0)
	{
		strStream << " Duration: " << duration_ << " seconds";
	}

	return strStream.str();
}
