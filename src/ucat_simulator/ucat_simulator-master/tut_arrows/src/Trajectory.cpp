#include "tut_arrows/Trajectory.h"
#include "tut_arrows/TrajectoryUtils.h"
#include <tut_arrows_msgs/GetXYZ.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>

Trajectory::Trajectory() : nh_(), positionPrecision_(0.5), orienatationPrecision_(0.2), currentPose_(), currentTime_(0.0), isStarted_(true), startupTime_(30.0), useTimedTargets_(false), 
						yawOffset_(0)
{
	loadTargets();

	trajectoryPointPub_ = nh_.advertise<tut_arrows_msgs::TrajectoryPoint>("trajectory", 1);
	controlModePub_ = nh_.advertise<std_msgs::String>("control_mode", 1);
	odomSub_ = nh_.subscribe("odom", 1, &Trajectory::odometryCallback, this);

	setTargetSrv_ = nh_.advertiseService("SetTarget", &Trajectory::setTarget, this);
	loadTargetsSrv_ = nh_.advertiseService("LoadTargets", &Trajectory::loadTargets, this);
	GPSToLocal_ = nh_.serviceClient<tut_arrows_msgs::GetXYZ>("GetXYZ");
	startExplorer_ = nh_.serviceClient<std_srvs::Empty>("explorer/start");
	stopExplorer_ = nh_.serviceClient<std_srvs::Empty>("explorer/stop");
	setTrajectorySrv_ = nh_.advertiseService("SetTrajectory", &Trajectory::setTrajectory, this);

	ros::NodeHandle nhPrivate("~");
	nhPrivate.param("position_precision", positionPrecision_, positionPrecision_);
	nhPrivate.param("orientation_precision", orienatationPrecision_, orienatationPrecision_);

	nhPrivate.getParam("auto_start", isStarted_);

	nhPrivate.getParam("startup_time", startupTime_);

	nhPrivate.getParam("use_timed_targets", useTimedTargets_);

	acousticModemSub_ = nh_.subscribe("APPLICON_INBOX", 10, &Trajectory::acousticModemCallback, this); 
}

void Trajectory::resetCurrentTarget(Target::Ptr target)
{
	currentTarget_ = target;
}

void Trajectory::addTarget(Target::Ptr target)
{
	targets_.push(target);
}

void Trajectory::update(const geometry_msgs::Pose& currentPose, double currentTime)
{
	currentPose_ = currentPose;
	currentTime_ = currentTime;

	if (!currentTarget_ || isCurrentTargetReached())
	{
		if (currentTarget_ && currentTarget_->getType() == Target::Explore)
		{
			std_srvs::Empty empty;
			stopExplorer_.call(empty);

			std_msgs::String autoCmd;
			autoCmd.data = "AUTO";
			controlModePub_.publish(autoCmd);
		}
		else if (currentTarget_ && currentTarget_->getType() == Target::BeaconHoming)
		{
			std_msgs::String autoCmd;
			autoCmd.data = "AUTO";
			controlModePub_.publish(autoCmd);
		}

		nextTarget(currentPose, currentTime);
	}
}

bool Trajectory::isCurrentTargetReached() const
{
	if (!currentTarget_)
	{
		return true;
	}

	if (useTimedTargets_ || currentTarget_->getType() == Target::Explore || currentTarget_->getType() == Target::BeaconHoming)
	{
		return currentTarget_->isElapsed();
	}
	else
	{
		Eigen::Matrix<double, 6, 1> poseError = TrajectoryUtils::getPoseError(currentPose_, currentTarget_->getDesired(currentTime_));

		const bool xReached = std::abs(poseError(0)) < positionPrecision_;
		const bool yReached = std::abs(poseError(1)) < positionPrecision_;
		const bool zReached = std::abs(poseError(2)) < positionPrecision_;
		const bool rollReached = std::abs(poseError(3)) < orienatationPrecision_;
		const bool pitchReached = std::abs(poseError(4)) < orienatationPrecision_;
		const bool yawReached = std::abs(poseError(5)) < orienatationPrecision_;

		return xReached && yReached && zReached && rollReached && pitchReached && yawReached;
	}
}

bool Trajectory::hasTarget() const
{
	return currentTarget_ != nullptr;
}

bool Trajectory::loadMissionTargets(const std::string& filename){
	//add clientService and call Rasmuses function for local coordinates
	std::ifstream missionFile(filename);
	tut_arrows_msgs::GetXYZ service;
	std::string GPSValues;
	std::string searchCoordinates = "lla_pos:";
	std::vector<double> coordinates;
	std::string readLine;
	double i = 0.0;

	while(!missionFile.eof())
	{

		std::getline(missionFile,readLine);
		std::size_t find = readLine.find(searchCoordinates);
		if(find==std::string::npos){
			continue;
		}
		else{
			GPSValues=readLine.substr(find+searchCoordinates.length()+2,readLine.find(']'));
			GPSValues.erase(GPSValues.find_last_not_of(" \n\r\t"));
			std::stringstream ss(GPSValues);
			while(ss >> i){
				if(ss.peek()==',' || ss.peek()==' '){
					ss.ignore();
				}
				coordinates.push_back(i);
			}

			//for(int i=0; i<coordinates.size(); ++i){
			//		std::cout << coordinates[i] << ' ';
			//}

			if (coordinates.size() == 3)
			{

				service.request.global.x = coordinates[0];
				service.request.global.y = coordinates[1];
				service.request.global.z = coordinates[2];
			}

			if (GPSToLocal_.call(service))
			{
				geometry_msgs::Pose pose;
				pose.position.x = service.response.XYZ.x;
				pose.position.y = service.response.XYZ.y;
				pose.position.z = service.response.XYZ.z;
				pose.orientation.w = 1.0;
				pose.orientation.x = 0.0;
				pose.orientation.y = 0.0;
				pose.orientation.z = 0.0;

				targets_.push(std::make_shared<Target>(pose));
			}
			//std::cout << std::endl;
			coordinates.clear();
		}
	}
	return true;
}

bool Trajectory::loadTargets()
{
	ros::NodeHandle nhPrivate("~");

	std::string targetStr;
	if (nhPrivate.getParam("target_file", targetStr))
	{
		return loadTargets(targetStr);
	}
	else if (nhPrivate.getParam("target", targetStr))
	{
		std::shared_ptr<Target> target = Target::parse(targetStr);

		if (target)
		{
			targets_.push(target);
			return true;
		}
	}
	return false;
}

bool Trajectory::loadTargets(tut_arrows_msgs::LoadTargets::Request& request, tut_arrows_msgs::LoadTargets::Response& response)
{
	return loadTargets(request.filename);
}

bool Trajectory::loadTargets(const std::string& filename)
{
	clearTargets();

	std::ifstream ifs(filename);

	if (!ifs.good())
	{
		return false;
	}

	while (!ifs.eof())
	{
		std::string line;
		std::getline(ifs, line);

		if (!line.empty())
		{
			// if comment then take next line
			if (line[0] == '#')
			{
				continue;
			}

			std::shared_ptr<Target> target = Target::parse(line);
			if (target)
			{
				targets_.push(target);
			}
		}
	}
	return true;
}

bool Trajectory::setTarget(tut_arrows_msgs::SetTarget::Request& request, tut_arrows_msgs::SetTarget::Response& response)
{
	std::shared_ptr<Target> target = std::make_shared<Target>(request.pose);

	if (target)
	{
		clearTargets();
		resetCurrentTarget(Target::Ptr());
		targets_.push(target);
		return true;
	}

	return false;
}

/// If there are more targets then take next one otherwise keep the current target.
///
void Trajectory::nextTarget(const geometry_msgs::Pose& currentPose, double currentTime)
{
	if (!targets_.empty())
	{
		std::shared_ptr<Target> nextTarget = targets_.front();

		if (nextTarget)
		{
			if (currentTarget_)
			{
				nextTarget->start(currentTarget_->getTarget(), currentTime);
				//nextTarget->start(currentPose, currentTime);
			}
			else
			{
				nextTarget->start(currentPose, currentTime);
			}

			if (nextTarget->getType() == Target::Explore)
			{
				std_msgs::String stopCmd;
				stopCmd.data = "STOP";
				controlModePub_.publish(stopCmd);

				std_srvs::Empty empty;
				startExplorer_.call(empty);
			}
			else if (nextTarget->getType() == Target::BeaconHoming)
			{
				std_msgs::String BeaconHomingCmd;
				BeaconHomingCmd.data = "OUT";
				controlModePub_.publish(BeaconHomingCmd);
			}

			currentTarget_ = nextTarget;
			ROS_INFO("New target %s", currentTarget_->toString().c_str());
		}

		targets_.pop();
	}
	else
	{
		currentTarget_ = nullptr;
	}
}

void Trajectory::clearTargets()
{
	targets_ = std::queue< std::shared_ptr<Target> >();
}

bool Trajectory::setTrajectory(tut_arrows_msgs::SetTrajectory::Request& request, tut_arrows_msgs::SetTrajectory::Response& response)
{
	clearTargets();
	resetCurrentTarget(Target::Ptr());

	try
	{
		for (auto& target : request.targets)
		{
			addTarget(std::make_shared<Target>(static_cast<Target::Type>(target.type), target.pose, target.duration));
		}

		ROS_INFO("Starting mission.");
		return true;
	}
	catch (const std::exception& ex)
	{
		clearTargets();
		ROS_ERROR("Error while setting trajectory: %s", ex.what());
		return false;
	}
}

void Trajectory::odometryCallback(nav_msgs::Odometry::ConstPtr data)
{
	odom_ = data;
}

void Trajectory::acousticModemCallback(const auv_msgs::AcousticModemData::ConstPtr& data)
{
	std::string message(data->payload.begin(), data->payload.end());

	if (message == "START")
	{
		ROS_INFO("START command received");
		isStarted_ = true;

		//if (odom_)
		//{
		//	double roll, pitch, yaw;
		//	tf::Matrix3x3(QuatUtils::fromMsg(odom_->pose.pose.orientation)).getRPY(roll, pitch, yaw);
		//	yawOffset_ = yaw;

		//	ROS_INFO("Set yaw offset to %g", yawOffset_);
		//}
	}
}

void Trajectory::run()
{
	ros::Rate r(10);

	ros::Time startTime = ros::Time::now();

	while (ros::ok())
	{
		ros::spinOnce();

		if (odom_ && isStarted_ && ( (ros::Time::now() - startTime).toSec() > startupTime_) )
		{
			double currentTime = ros::Time::now().toSec();

			update(odom_->pose.pose, currentTime);

			if (currentTarget_)
			{
				tut_arrows_msgs::TrajectoryPoint trajectoryPoint;
				trajectoryPoint.header.stamp = ros::Time::now();
				trajectoryPoint.pose = currentTarget_->getDesired(currentTime);

				//tf::Quaternion orientation = QuatUtils::fromMsg(trajectoryPoint.pose.orientation);
				//orientation = orientation * tf::Quaternion(tf::Vector3(0., 0., 1.), yawOffset_);
				//trajectoryPoint.pose.orientation = QuatUtils::toMsg(orientation);

				Eigen::Matrix<double, 6, 1> velocity = currentTarget_->getDesiredVelocity(currentTime);
				trajectoryPoint.linear_velocity.x = velocity(0);
				trajectoryPoint.linear_velocity.y = velocity(1);
				trajectoryPoint.linear_velocity.z = velocity(2);

				trajectoryPoint.angular_velocity.x =  velocity(3);
				trajectoryPoint.angular_velocity.y =  velocity(4);
				trajectoryPoint.angular_velocity.z =  velocity(5);

				Eigen::Matrix<double, 6, 1> acceleration = currentTarget_->getDesiredAcceleration(currentTime);
				trajectoryPoint.linear_acceleration.x = acceleration(0);
				trajectoryPoint.linear_acceleration.y = acceleration(1);
				trajectoryPoint.linear_acceleration.z = acceleration(2);

				trajectoryPoint.angular_acceleration.x = acceleration(3);
				trajectoryPoint.angular_acceleration.y = acceleration(4);
				trajectoryPoint.angular_acceleration.z = acceleration(5);

				trajectoryPointPub_.publish(trajectoryPoint);
			}
		}

		r.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "TrajectoryManager");
	Trajectory trajectory;
	trajectory.run();
}
