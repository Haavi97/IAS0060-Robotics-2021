///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//"DirectionalReceiver" example, SimulatedDevice_DirectionalReceiver.h
#ifndef SIMULATEDDEVICE_DIRECTIONALRECEIVER_H_
#define SIMULATEDDEVICE_DIRECTIONALRECEIVER_H_
#include "uwsim/SimulatedDevices.h"
#include <osgSim/SphereSegment>
using namespace uwsim;
/*
 * Example header of driver/rosinterface configuration/factory
 *
 * Included in SimulatedDevices.cpp
 */

//Driver/ROSInterface configuration
class SimDev_DirectionalReceiver_Config : public SimulatedDeviceConfig
{
public:
  //XML members
  std::string relativeTo;
  std::vector<std::string> sensitiveTo; // names of the objects the receiver can see
  float range;
  float angle;
  bool visible;
  unsigned int numLines;
  std::vector<float> position;
  std::vector<float> orientation;
  //constructor
  SimDev_DirectionalReceiver_Config(std::string type_) :
      SimulatedDeviceConfig(type_), position(std::vector<float>(3, 0.0)), orientation(std::vector<float>(3, 0.0))
  {
  }
};

//Driver/ROSInterface factory class
class SimDev_DirectionalReceiver_Factory : public SimulatedDeviceFactory
{
public:
  //this is the only place the device/interface type is set
  SimDev_DirectionalReceiver_Factory(std::string type_ = "directionalReceiver") :
      SimulatedDeviceFactory(type_)
  {
  }
  ;

  SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node* node, ConfigFile * config);
  bool applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration);
  std::vector<boost::shared_ptr<ROSInterface> > getInterface(ROSInterfaceInfo & rosInterface,
                                                             std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile);
};

//can be a sparate header file for actual implementation classes...

#include "uwsim/ConfigXMLParser.h"
#include "uwsim/ROSInterface.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>

//Driver class
class SimDev_DirectionalReceiver : public SimulatedDevice
{
  void applyPhysics(BulletPhysics * bulletPhysics)
  {
  }
public:
  SimDev_DirectionalReceiver(SimDev_DirectionalReceiver_Config * cfg, osg::Node * root, osg::Node * trackNode);
  class IntersectorUpdateCallback : public osg::NodeTrackerCallback
  {
  public:
    virtual void operator()(osg::Node *node, osg::NodeVisitor *nv);
    double range, angle;
    unsigned int numLines;
    bool transmitterVisible;
    std::vector<std::string> sensitiveTo; // names of the objects the receiver can see
    osg::ref_ptr<osg::Node> root;
    osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector;
    osgUtil::IntersectionVisitor intersectVisitor;

    osg::ref_ptr<osg::Geode> geode;		//Geometry node that draws the beam
    osg::ref_ptr<osg::Geometry> beam;
    osg::ref_ptr<osg::Geode> coneGeode;		//Geometry node that draws the receiver cone
    osg::ref_ptr<osgSim::SphereSegment> cone;

    IntersectorUpdateCallback(double range, double angle, bool visible, osg::Node *root, unsigned int lines);
  };
  osg::ref_ptr<IntersectorUpdateCallback> node_tracker;
};

//ROS publishers and subscribers work exactly as before, no subclassing is needed
class SimDev_DirectionalReceiver_ROSPublisher : public ROSPublisherInterface
{
  //this is just an example, use a pointer to SimulatedIAUV, if only ROSInterface is implemented
  //pointer to a device
  SimDev_DirectionalReceiver * dev;
public:
  SimDev_DirectionalReceiver_ROSPublisher(SimDev_DirectionalReceiver *dev, std::string topic, int rate) :
      ROSPublisherInterface(topic, rate), dev(dev)
  {
  }

  void createPublisher(ros::NodeHandle &nh);
  void publish();

  ~SimDev_DirectionalReceiver_ROSPublisher()
  {
  }
};

#endif
