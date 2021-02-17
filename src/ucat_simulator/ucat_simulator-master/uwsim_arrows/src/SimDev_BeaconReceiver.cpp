/*
 * SimDev_BeaconReceiver.cpp
 *
 *  Created on: 10.12.2013
 *      Author: rasmus
 */


#include "uwsim_arrows/SimDev_BeaconReceiver.h"
#include "tut_arrows_msgs/BeaconPing.h"
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include "uwsim/SceneBuilder.h"
#include <uwsim/UWSimUtils.h>
#include <vector>

//mart messing
#include <typeinfo>
#include <iostream>
//#include <tgmath.h>
#include <math.h> 

SimDev_BeaconReceiver::SimDev_BeaconReceiver(SimDev_BeaconReceiver_Config * cfg,
		osg::Node * root, osg::Node * trackNode) :
		SimulatedDevice(cfg), cfg(*cfg), rootNode(root), receiverNode(trackNode),beaconChanger(0),
		rng(static_cast<unsigned int>(std::time(0))), var_nor(rng, boost::normal_distribution<>(0.0, cfg->measurementStd))
{
}

void SimDev_BeaconReceiver::setViewBuilder(ViewBuilder * viewBuilder)
{
	for (std::vector<std::string>::iterator beaconName = cfg.sensitiveTo.begin(); beaconName != cfg.sensitiveTo.end(); beaconName++)
	{
		findNodeVisitor findBeaconNode(*beaconName);
		rootNode->accept(findBeaconNode);
		std::vector<osg::Node*> foundBeaconNodes = findBeaconNode.getNodeList();
		beaconNodes.insert(beaconNodes.end(), foundBeaconNodes.begin(), foundBeaconNodes.end());
	}
	ROS_INFO("Found %lu beacons in the scene", beaconNodes.size());

	boost::random::mt19937 rng2;
	rng2.seed(static_cast<unsigned int>(std::time(0)));
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > beacon_var_nor(rng2, boost::normal_distribution<>(0.0, cfg.beaconPosStd));

	for (std::vector<osg::Node*>::iterator beaconIt = beaconNodes.begin(); beaconIt != beaconNodes.end(); ++beaconIt)
	{
		osg::Node* beaconNode = *beaconIt;
		if (beaconNode == NULL)
		{
			continue;
		}

		beaconDisplacement_[beaconNode->getName()] = osg::Vec3d(beacon_var_nor(), beacon_var_nor(), beacon_var_nor());
	}
}

std::vector<SimDev_BeaconReceiver::Measurement> SimDev_BeaconReceiver::getMeasurements()
{
	std::vector<SimDev_BeaconReceiver::Measurement> measurements;

	boost::shared_ptr<osg::Matrixd> receiverCoords = getWorldCoords(receiverNode);
	if (!receiverCoords)
	{
		throw std::runtime_error("Invalid receiver node");
	}

	for (std::vector<osg::Node*>::const_iterator beaconIt = beaconNodes.begin(); beaconIt != beaconNodes.end(); beaconIt++)
	{
		osg::Node* beacon = *beaconIt;
		std::string beaconName = beacon->getName();
		if (beacon == NULL)
		{
			continue;
		}

		boost::shared_ptr<osg::Matrixd> beaconCoords = getWorldCoords(beacon);
		if (!beaconCoords)
		{
			continue;
		}

		osg::Vec3d beaconLocation = beaconCoords->getTrans();
		if (beaconDisplacement_.find(beacon->getName()) != beaconDisplacement_.end())
		{
			beaconLocation += beaconDisplacement_[beacon->getName()];
		}

		//ROS_INFO("Beacon %s at coord (%g, %g, %g)", beacon->getName().c_str(), beaconLocation.x(), beaconLocation.y(), beaconLocation.z());

		/*const*/ 
		osg::Vec3d globalDistance = beaconLocation - receiverCoords->getTrans();
	/*        
	if (pow(globalDistance.x(),2)+
            pow(globalDistance.y(),2) <= .25){
            std::cout << "Changing beacons for the ";
            beaconChanger=beaconName[beaconName.size()-1    ]-'0';
            std::cout << beaconChanger << ". time." << std::endl;
        }
		if (globalDistance.length() > cfg.range)
		{
			continue;
		}
         */       
		osg::Vec3d localDistance = receiverCoords->getRotate().inverse() * globalDistance;

                //mart messing
		//ROS_INFO(typeid(beacon->getName()).name());
		//ROS_INFO("vs");
		//string tmp_string = beacon->getName();
		//string tmp_string2 = "beacon1";
		//string tmp_string3 = "what ";
		//ROS_INFO(typeid(tmp_string2).name());
		//ROS_INFO("this works: "+tmp_string2);
		//ROS_INFO(tmp_string3+tmp_string2);
		//ROS_INFO("this doesn't: "+tmp_string);
		//ROS_INFO((std::string) beacon->getName());
		//std::cout << beacon->getName() << std::endl;
/*		if (beacon-> getName()=="beacon1"){
		    ROS_INFO("Local Distance for beacon1: %g",localDistance.length());
		}*/
		
		//done messing
		/*
		localDistance.normalize();

		localDistance.x() += var_nor();
		localDistance.y() += var_nor();
		localDistance.z() += var_nor();
		
		//ROS_INFO("rnd nr %g",var_nor);
                        

		localDistance.normalize();
                */
                globalDistance.x()+=var_nor();
                globalDistance.y()+=var_nor();
                globalDistance.z()+=var_nor();
                globalDistance.normalize();
		//measurements.push_back( std::make_pair(beacon->getName(),localDistance) );
	    if (beaconChanger == 1){//use second pinger
	        if (beaconName == "beacon1"){
	            beaconName = "beacon2";
	        }
	        else if (beaconName == "beacon2"){
	            beaconName = "beacon1";
	        }
	    }
	    else if(beaconChanger == 2){//use 3rd pinger
	        if (beaconName == "beacon1"){
	            beaconName = "beacon2";
	        }
	        else if (beaconName == "beacon3"){
	            beaconName = "beacon1";
	        }
	    }
	    else if(beaconChanger == 3){//use 4th pinger
	        if (beaconName == "beacon1"){
	            beaconName = "beacon2";
	        }
	        else if (beaconName == "beacon4"){
	            beaconName = "beacon1";
	        }
	    }
		//measurements.push_back( std::make_pair(beacon->getName(),std::make_pair(globalDistance,localDistance)) );
		measurements.push_back( std::make_pair(beaconName,std::make_pair(globalDistance,localDistance)) );
	}
    robotPos = receiverCoords->getTrans();
    //std::cout << "x: " << robotPos.x() << " y: " << robotPos.y() << " z: " << robotPos.z() << std::endl;
	return measurements;
}

SimulatedDeviceConfig::Ptr SimDev_BeaconReceiver_Factory::processConfig(
		const xmlpp::Node* node, ConfigFile * config)
{
	SimDev_BeaconReceiver_Config * cfg = new SimDev_BeaconReceiver_Config(
			getType());
	xmlpp::Node::NodeList list = node->get_children();
	double tmp3[3];
	double tmp;
	int tmpint;
	unsigned int tmpuint;
	std::string tmpstring;
	for (xmlpp::Node::NodeList::iterator iter = list.begin();
			iter != list.end(); ++iter)
	{
		const xmlpp::Node* child = dynamic_cast<const xmlpp::Node*>(*iter);

		if (child->get_name() == "relativeTo")
			config->extractStringChar(child, cfg->relativeTo);
		else if (child->get_name() == "sensitiveTo")
		{
			config->extractStringChar(child, tmpstring);
			boost::trim(tmpstring);
			cfg->sensitiveTo.push_back(tmpstring);
		}
		else if (child->get_name() == "notBlockedBy")
		{
			config->extractStringChar(child, tmpstring);
			boost::trim(tmpstring);
			cfg->notBlockedBy.push_back(tmpstring);
		}
		else if (child->get_name() == "range")
		{
			config->extractFloatChar(child, tmp);
			cfg->range = tmp;
		}
		else if (child->get_name() == "visible")
		{
			config->extractIntChar(child, tmpint);
			cfg->visible = tmpint;
		}
		else if (child->get_name() == "requiresLineOfSight")
		{
			config->extractIntChar(child, tmpint);
			cfg->requiresLineOfSight = tmpint;
		}
		else if (child->get_name() == "position")
		{
			config->extractPositionOrColor(child, tmp3);
			for (int i = 0; i < 3; i++)
				cfg->position[i] = tmp3[i];
		}
		else if (child->get_name() == "orientation")
		{
			config->extractOrientation(child, tmp3);
			for (int i = 0; i < 3; i++)
				cfg->orientation[i] = tmp3[i];
		}
		else if (child->get_name() == "measurementStd")
		{
			config->extractFloatChar(child, tmp);
			cfg->measurementStd = tmp;
		}
		else if (child->get_name() == "beaconPosStd")
		{
			config->extractFloatChar(child, tmp);
			cfg->beaconPosStd = tmp;
		}
	}
	return SimulatedDeviceConfig::Ptr(cfg);
}

bool SimDev_BeaconReceiver_Factory::applyConfig(SimulatedIAUV * auv,
		Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration)
{
	if (iteration > 0)
		return true;
// find all BeaconReceivers
	for (size_t i = 0; i < vehicleChars.simulated_devices.size(); ++i)
		if (vehicleChars.simulated_devices[i]->getType() == this->getType())
		{
			// Get the configuration from XML
			SimDev_BeaconReceiver_Config * cfg =
					dynamic_cast<SimDev_BeaconReceiver_Config *>(vehicleChars.simulated_devices[i].get());
			// Find parent node
			int parentNodeIndex = -1;
			for (int j = 0; j < vehicleChars.nlinks && parentNodeIndex < 0; j++)
				if (vehicleChars.links[j].name == cfg->relativeTo)
					parentNodeIndex = j;

			if (parentNodeIndex >= 0 && cfg)
			{
				// Add transform according to position and orientation
				osg::ref_ptr<osg::Transform> vMr = (osg::Transform*) new osg::PositionAttitudeTransform;

				vMr->asPositionAttitudeTransform()->setPosition(osg::Vec3d(cfg->position[0], cfg->position[1], cfg->position[2]));
				vMr->asPositionAttitudeTransform()->setAttitude(
					  osg::Quat(cfg->orientation[2], osg::Vec3d(0, 0, 1),
								cfg->orientation[1], osg::Vec3d(0, 1, 0),
								cfg->orientation[0], osg::Vec3d(1, 0, 0)));

				auv->urdf->link[parentNodeIndex]->asGroup()->addChild(vMr);

				// Create new device from the configuration
				auv->devices->all.push_back(
						SimDev_BeaconReceiver::Ptr(
								new SimDev_BeaconReceiver(cfg,
										sceneBuilder->scene->localizedWorld,
										vMr)));
			}
			else
			OSG_FATAL<< "SimDev_BeaconReceiver device '" << cfg->relativeTo << " "
			<< vehicleChars.simulated_devices[i]->name
			<< "' inside robot '" << vehicleChars.name
			<< "' has failed, discarding..." << std::endl;
		}
	return true;
}

std::vector<boost::shared_ptr<ROSInterface> > SimDev_BeaconReceiver_Factory::getInterface(
		ROSInterfaceInfo & rosInterface,
		std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile)
{
	std::vector<boost::shared_ptr<ROSInterface> > ifaces;
	for (size_t i = 0; i < iauvFile.size(); ++i)
		for (size_t d = 0; d < iauvFile[i]->devices->all.size(); ++d)
			if (iauvFile[i]->devices->all[d]->getType() == this->getType()
					&& iauvFile[i]->devices->all[d]->name
							== rosInterface.targetName)
			{
				ifaces.push_back(
						boost::shared_ptr<ROSInterface>(
								new SimDev_BeaconReceiver_ROSPublisher(
										dynamic_cast<SimDev_BeaconReceiver*>(iauvFile[i]->devices->all[d].get()),
										rosInterface.topic,
										rosInterface.rate)));
			}
	if (ifaces.size() == 0)
		ROS_WARN("Returning empty ROS interface for device %s...",
				rosInterface.targetName.c_str());
	return ifaces;
}

void SimDev_BeaconReceiver_ROSPublisher::createPublisher(ros::NodeHandle &nh)
{
	ROS_INFO("SimDev_BeaconReceiver_ROSPublisher on topic %s", topic.c_str());
	std::cout << "Topic : " << topic << std::endl;
	pub_ = nh.advertise<tut_arrows_msgs::BeaconPing>(topic, 10);
}

void SimDev_BeaconReceiver_ROSPublisher::publish()
{
	std::vector<SimDev_BeaconReceiver::Measurement> measurements = dev->getMeasurements();
    osg::Vec3d robotPos = dev->robotPos; 
	for (std::vector<SimDev_BeaconReceiver::Measurement>::iterator mIt = measurements.begin(); mIt != measurements.end(); ++mIt)
	{
		const SimDev_BeaconReceiver::Measurement& measurement = *mIt;
		const std::string& name = measurement.first;
		/*const*/ pair<osg::Vec3d,osg::Vec3d> dirs = measurement.second; //COPY!!
		/*const*/ osg::Vec3d& dir = dirs.first;
		/*const*/ osg::Vec3d& relDir = dirs.second;
		relDir.normalize();

		tut_arrows_msgs::BeaconPing msg;
		msg.header.stamp = ros::Time::now();
		msg.id = name;
		msg.direction.x = dir.x();
		msg.direction.y = dir.y();
		msg.direction.z = dir.z();
		msg.absyaw      = std::atan2(dir.y(),dir.x());
		msg.relDir.x = relDir.x();
		msg.relDir.y = relDir.y();
		msg.relDir.z = relDir.z();
		msg.relyaw   = std::atan2(relDir.y(),relDir.x());
		msg.robotPos.x = dev->robotPos.x();
		msg.robotPos.y = dev->robotPos.y();
		msg.robotPos.z = dev->robotPos.z();
		pub_.publish(msg);
	}
}
