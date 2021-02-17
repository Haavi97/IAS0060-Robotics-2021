///////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef SimDev_Depth2Sonar_h
#define SimDev_Depth2Sonar_h
#include "uwsim/SimulatedDevices.h"
#include "uwsim/VirtualCamera.h"
#include "uwsim/ROSInterface.h"

using namespace uwsim;

class SimDev_Depth2Sonar_Factory : public SimulatedDeviceFactory
{
public:
  SimDev_Depth2Sonar_Factory(std::string type_ = "depth2Sonar") :
      SimulatedDeviceFactory(type_){};

  SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node* node, ConfigFile * config)
  {
    return SimulatedDeviceConfig::Ptr();
  }

  bool applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration)
  {
    return true;
  }

  std::vector<boost::shared_ptr<ROSInterface> > getInterface(ROSInterfaceInfo & rosInterface,
                                                             std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile);
};

class SimDev_Depth2Sonar_ROSPublisher : public ROSPublisherInterface
{
  VirtualCamera * dev;
  ros::Publisher image_pub;
  std::string infoTopic, frameId;
  int mult;
  bool publishAll;
  std::vector<ros::Publisher> pub_all;
public:
  SimDev_Depth2Sonar_ROSPublisher(VirtualCamera * dev, std::string topic, int rate,std::string infoTopic, int divisor,
                                  std::string frameId, bool publishAll) :
    ROSPublisherInterface(topic, rate), dev(dev)
  {
    this->infoTopic = infoTopic;
    this->mult = divisor;
    this->frameId = frameId;
    this->publishAll = publishAll;
  }

  void createPublisher(ros::NodeHandle &nh);
  void publish();

  ~SimDev_Depth2Sonar_ROSPublisher()
  {
  }
};


#endif
