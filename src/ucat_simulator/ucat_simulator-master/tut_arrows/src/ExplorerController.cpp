/*
 * ExplorerController.cpp
 *
 *  Created on: 16.07.2015
 *      Author: rasmus
 */

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/FluidPressure.h>
#include <std_srvs/Empty.h>
#include <tut_arrows/FluidPressure.h>
#include <auv_msgs/AcousticModemData.h>
#include <tut_arrows_msgs/FlippersModeCmd.h>

using namespace std;

namespace
{
const float distanceThreshold = 0.5; // metres
float depth = 0;
float targetDepth = 0;
bool started = false;
ros::Publisher wrenchModePublisher;
enum Mode
{
	FORWARD,
	TURN
};
vector<string> sonarNames = {
		"hw/sonar/fl",
		"hw/sonar/ff",
		"hw/sonar/fr"
};

class Sonar
{
public:
	ros::Subscriber subscriber;
	vector<float> ranges;
	bool listening = false;
	Sonar(ros::NodeHandle& n, string topic);
	void receiveMsg(sensor_msgs::RangeConstPtr rangeMsg);
	void startListening();
	float stopListeningAndGetRange();
};

// here's the hard part, doing statistics with too few data points
float filter(vector<float> readings)
{
	// we are interested in the closest obstacle
	auto minimum = min_element(readings.begin(), readings.end());
/*
	// if one or two readings, the lowest value is the range
	if(readings.size() < 3)
		return *minimum;
	// otherwise, require at least two similar readings to eliminate outliers
	const int requiredSimilarCount = 2;
	const float similarityThreshold = 0.5;
	int similar = 0;
	for(auto& reading : readings)
		if(abs(*minimum - reading) < similarityThreshold)
			similar++;
	if(similar >= requiredSimilarCount)
		return *minimum;

	// remove the outlier and try again
	readings.erase(minimum);
	return filter(readings);
*/
	cout << *minimum << " ";
	return *minimum;
}

Sonar::Sonar(ros::NodeHandle& n, string topic)
{
	subscriber = n.subscribe(topic, 100, &Sonar::receiveMsg, this);
}

void Sonar::receiveMsg(sensor_msgs::RangeConstPtr rangeMsg)
{
	//if(listening)
		ranges.push_back(rangeMsg->range);
}
void Sonar::startListening()
{
	ranges.clear();
	listening = true;
}
float Sonar::stopListeningAndGetRange()
{
	listening = false;
	if(!ranges.empty())
		return filter(ranges);

	ROS_INFO("Sonar not working");
	return 10;
}
void start()
{
	started = true;
	targetDepth = depth;
	tut_arrows_msgs::FlippersModeCmd msg;
	msg.header.stamp = ros::Time::now();
	msg.mode = tut_arrows_msgs::FlippersModeCmd::MODE_SLOW;
	msg.mode_change_time = 8;
	wrenchModePublisher.publish(msg);
}
void stop()
{
	started = false;
}
bool startExploring(std_srvs::EmptyRequest& a, std_srvs::EmptyResponse& b)
{
	start();
	return true;
}
bool stopExploring(std_srvs::EmptyRequest& a, std_srvs::EmptyResponse& b)
{
	stop();
	return true;
}
void readPressure(sensor_msgs::FluidPressureConstPtr msg)
{
	const float pressureAtSurface = 101325.0;
	depth = FluidPressure::getDepth(msg->fluid_pressure, 101325.0);
}
float depthControlForce(float depthError)
{
	float K = -20, limit = 50;
	return min(limit, max(-limit, depthError * K));
}
void acousticModemCallback(const auv_msgs::AcousticModemData::ConstPtr& data)
{
	std::string message(data->payload.begin(), data->payload.end());

	if (message == "EXPLORE")
	{
		ROS_INFO("EXPLORE command received");
		start();
	}
	else if (message == "STOP")
	{
		ROS_INFO("STOP command received");
		stop();
	}
}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "explorer");
	ros::NodeHandle n;
	auto testf = [&](){started = true;};
	ros::Publisher wrenchPublisher = n.advertise<geometry_msgs::WrenchStamped>("force_req", 10, false);
	wrenchModePublisher = n.advertise<tut_arrows_msgs::FlippersModeCmd>("force_mode", 10);
	ros::ServiceServer startService = n.advertiseService("explorer/start", startExploring);
	ros::ServiceServer stopService = n.advertiseService("explorer/stop", stopExploring);

	ros::Subscriber depthSubscriber = n.subscribe("hw/pressure", 10, readPressure);
	vector<unique_ptr<Sonar>> sonars;
	for(auto& name : sonarNames)
		sonars.push_back(unique_ptr<Sonar>(new Sonar(n, name)));
	ros::Subscriber acousticModemSubscriber = n.subscribe("APPLICON_INBOX", 10, acousticModemCallback);
	ros::spinOnce();
	ros::Duration(5.0).sleep();

	Mode mode = FORWARD;
	geometry_msgs::WrenchStamped msg;
    while (ros::ok())
	{
    	if(!started)
    	{
    		if(msg.wrench.force.x || msg.wrench.torque.z)
    		{
    			msg.wrench.force.x = 0.0;
    			msg.wrench.force.z = 0.0;
				msg.wrench.torque.z = 0.0;
				wrenchPublisher.publish(msg);
    		}
    		ros::spinOnce();
    		ros::Duration(0.1).sleep();
    		continue;
    	}
    	// listen
    	for(auto& sonar : sonars)
    		sonar->startListening();

    	ros::spinOnce();
    	ros::Duration(1.0).sleep();

    	mode = FORWARD;
    	for(auto& sonar : sonars)
	{
		if(sonar->stopListeningAndGetRange() < distanceThreshold)
		{
			mode = TURN;
			ROS_INFO("Obstacle detected");
		}
	}

	cout << endl;
    	// move
    	msg.header.stamp = ros::Time::now();

    	if(mode == TURN)
    	{
			msg.wrench.force.x = 0.0;
			msg.wrench.torque.z = 5.0;
    	}
    	else
    	{
    		msg.wrench.force.x = 5.0;
			msg.wrench.torque.z = 0.0;
    	}
    	msg.wrench.force.z = depthControlForce(targetDepth - depth);
    	wrenchPublisher.publish(msg);
    	ros::spinOnce();
    	if(mode == TURN)
    		ros::Duration(1.5).sleep();
    	else
    		ros::Duration(1.0).sleep();
    	msg.wrench.force.x = 0.0;
		msg.wrench.force.z = 0.0;
    	msg.wrench.torque.z = 0.0;
		wrenchPublisher.publish(msg);
	}

	return 0;
}
