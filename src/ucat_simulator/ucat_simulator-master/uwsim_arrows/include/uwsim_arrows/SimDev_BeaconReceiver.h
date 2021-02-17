/*
 * SimDev_BeaconReceiver.h
 *
 *  Created on: 10.12.2013
 *      Author: rasmus
 */

#ifndef SIMDEV_BEACONRECEIVER_H_
#define SIMDEV_BEACONRECEIVER_H_
#include "uwsim/SimulatedDevices.h"
#include "uwsim/ConfigXMLParser.h"
#include "uwsim/ROSInterface.h"
#include <ros/ros.h>
using namespace uwsim;

//Driver/ROSInterface configuration
class SimDev_BeaconReceiver_Config: public SimulatedDeviceConfig
{
public:
	//XML members
	std::string relativeTo;
	std::vector<std::string> sensitiveTo; // names of the objects the receiver can see
	std::vector<std::string> notBlockedBy; // names of the objects the signal passes through
	float range;
	// visible turns on debugging graphics, requiresLineOfSight=true means objects can block the signal
	bool visible, requiresLineOfSight;
	std::vector<float> position;
	std::vector<float> orientation;
	double measurementStd;
	double beaconPosStd;
	//constructor
	SimDev_BeaconReceiver_Config(std::string type_) :
			SimulatedDeviceConfig(type_), position(std::vector<float>(3, 0.0)), orientation(
					std::vector<float>(3, 0.0)), requiresLineOfSight(false), visible(
					false), range(0), measurementStd(0.0), beaconPosStd(0.0)
	{
	}
};

class SimDev_BeaconReceiver_Factory: public SimulatedDeviceFactory
{
public:
	SimDev_BeaconReceiver_Factory(std::string type_ = "beaconReceiver") :
			SimulatedDeviceFactory(type_)
	{
	}
	;

	SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node* node,
			ConfigFile * config);

	bool applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars,
			SceneBuilder *sceneBuilder, size_t iteration);

	std::vector<boost::shared_ptr<ROSInterface> > getInterface(
			ROSInterfaceInfo & rosInterface,
			std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile);
};

//Driver class
class SimDev_BeaconReceiver: public SimulatedDevice
{
public:
	typedef std::pair<std::string, std::pair<osg::Vec3d,osg::Vec3d> > Measurement;

public:
	SimDev_BeaconReceiver(SimDev_BeaconReceiver_Config * cfg, osg::Node * root, osg::Node * trackNode);

	std::vector<Measurement> getMeasurements();
    osg::Vec3d robotPos;
private:
	void applyPhysics(BulletPhysics * bulletPhysics)
	{ }

	void setViewBuilder(ViewBuilder * viewBuilder);

private:
    int beaconChanger;
	SimDev_BeaconReceiver_Config cfg;
	std::vector<osg::Node*> beaconNodes;
	osg::ref_ptr<osg::Node> rootNode, receiverNode;

	boost::random::mt19937 rng;
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor;

	std::map<std::string, osg::Vec3d> beaconDisplacement_;
};

class SimDev_BeaconReceiver_ROSPublisher: public ROSPublisherInterface
{
	SimDev_BeaconReceiver * dev;
public:
	SimDev_BeaconReceiver_ROSPublisher(SimDev_BeaconReceiver *dev,
			std::string topic, int rate) :
			ROSPublisherInterface(topic, rate), dev(dev)
	{
	}

	void createPublisher(ros::NodeHandle &nh);
	void publish();

	~SimDev_BeaconReceiver_ROSPublisher()
	{
	}
};

#endif /* SIMDEV_BEACONRECEIVER_H_ */
