#include "tut_arrows/ObstacleAvoidance.h"

ObstacleAvoidance::ObstacleAvoidance() : nh_(), scale_(1.0, 1.0, 1.0)
{
	ros::NodeHandle nhPrivate("~");
	nhPrivate.param("min_oa_distance", oaMinDistance_, 0.2);
	nhPrivate.param("oa_distance", oaDistance_, 2.0);
	nhPrivate.param("oa_timeout", oaTimeout_, 2.0);
	nhPrivate.param("oa_enabled", oaEnabled_, true);

	nhPrivate.getParam("scale_x", scale_.data()[0]);
	nhPrivate.getParam("scale_y", scale_.data()[1]);
	nhPrivate.getParam("scale_z", scale_.data()[2]);

	sonars_.push_back(SonarInfo{"hw/sonar/ff", Eigen::Vector3d(-1.0,    0.0,   0.0), nullptr, "front"});
	//sonars_.push_back(SonarInfo{"hw/sonar/b",  Eigen::Vector3d( 1.0,    0.0,   0.0), nullptr, "back"});
	//sonars_.push_back(SonarInfo{"hw/sonar/l",  Eigen::Vector3d( 0.0,   -1.0,   0.0), nullptr, "left"});
	//sonars_.push_back(SonarInfo{"hw/sonar/r",  Eigen::Vector3d( 0.0,    1.0,   0.0), nullptr, "right"});
	//sonars_.push_back(SonarInfo{"hw/sonar/fu", Eigen::Vector3d( 0.0,    0.0,  -1.0), nullptr, "front-up"});
	//sonars_.push_back(SonarInfo{"hw/sonar/fd", Eigen::Vector3d( 0.0,    0.0,   1.0), nullptr, "front-down"});
	sonars_.push_back(SonarInfo{"hw/sonar/fl", Eigen::Vector3d(-0.707, -0.707, 0.0), nullptr, "front-left"});
	sonars_.push_back(SonarInfo{"hw/sonar/fr", Eigen::Vector3d(-0.707, -0.707, 0.0), nullptr, "front-right"});

	for (auto& sonarInfo : sonars_)
	{
		sonarSubs_.push_back( nh_.subscribe<sensor_msgs::Range>(sonarInfo.topic, 1, boost::bind(&ObstacleAvoidance::onRangeMessage, this, boost::ref(sonarInfo), _1)) );
	}
}

void ObstacleAvoidance::onRangeMessage(sensor_msgs::Range::Ptr data)
{
	ROS_INFO("got range data");
	sonars_[0].rangeMsg = data; //boost::make_shared<sensor_msgs::Range>(*data);
}

void ObstacleAvoidance::onRangeMessage(SonarInfo& sonarInfo, sensor_msgs::RangeConstPtr data)
{
	sonarInfo.rangeMsg = data;
}

Eigen::Vector3d ObstacleAvoidance::getOAForce() const
{
	Eigen::Vector3d oaForce(0,0,0);

	Eigen::Vector3d positiveDirection(0,0,0);
	Eigen::Vector3d negativeDirection(0,0,0);

	const double now = ros::Time::now().toSec();

	for (auto& sonarInfo : sonars_)
	{
		// haven't got any range update for this sonar
		if (!sonarInfo.rangeMsg)
		{
			ROS_WARN("Sonar data missing for %s", sonarInfo.name.c_str());
			continue;
		}

		const double dt = now - sonarInfo.rangeMsg->header.stamp.toSec();

		// last update for this sonar is older than timeout
		if (dt >= oaTimeout_)
		{
			ROS_WARN("Sonar message has timed out for %s.", sonarInfo.name.c_str());
			continue;
		}

		const double range = sonarInfo.rangeMsg->range;

		// if range is smaller than minimum OA distance then it is probably error (e.g. fin obstruction)
		// if range is bigger than OA threshold then it is ignored
		if (range < oaMinDistance_ || range > oaDistance_)
		{
			continue;
		}

		// linear ramp from f(oaDistance_) = 0 to f(oaMinDistance) = 1
		const double magnitude = (oaDistance_ - range) / (oaDistance_ - oaMinDistance_);

		const Eigen::Vector3d force = magnitude * sonarInfo.force;

		// find the closest obstacle along each axis in both directions
		for (int i = 0; i < force.size(); ++i)
		{
			if (force(i) > 0)
			{
				positiveDirection(i) = std::max(positiveDirection(i), force(i));
			}
			else
			{
				negativeDirection(i) = std::min(negativeDirection(i), force(i));
			}
		}
	}

	// obstacles can be in both sides
	oaForce = positiveDirection + negativeDirection;

	oaForce = oaForce.cwiseProduct(scale_);

	return oaForce;
}

bool ObstacleAvoidance::applyOA(geometry_msgs::Wrench& wrench)
{
	Eigen::Vector3d oaForce = getOAForce();

	//ROS_INFO("OA force: (%.2f, %.2f, %.2f)", oaForce(0), oaForce(1), oaForce(2));

	if (oaForce.norm() <= DBL_MIN)
	{
		return false; // OA not applied
	}

	bool xAxisActive = applyOAToAxis(oaForce(0), wrench.force.x, 'x');
	bool yAxisActive = applyOAToAxis(oaForce(1), wrench.force.y, 'y');
	bool zAxisActive = applyOAToAxis(oaForce(2), wrench.force.z, 'z');

	return xAxisActive || yAxisActive || zAxisActive;
}

bool ObstacleAvoidance::applyOAToAxis(double oaForce, double& force, char axis) const
{
	if ( (oaForce > 0 && force < oaForce) || (oaForce < 0 && force > oaForce) )
	{
		ROS_WARN("Obstacle avoidance active at %c (%.4f)", axis, oaForce);
		force = oaForce;
		return true;
	}
	return false;
}

bool ObstacleAvoidance::enabled() const
{
	return oaEnabled_;
}
