///////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef SimDev_FluidPressure_h
#define SimDev_FluidPressure_h
#include "uwsim/SimulatedDevices.h"
#include "uwsim/PressureSensor.h"
#include "uwsim/ROSInterface.h"

using namespace uwsim;

class SimDev_FluidPressure_Factory : public SimulatedDeviceFactory
{
public:
  SimDev_FluidPressure_Factory(std::string type_ = "fluidPressure") :
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

class SimDev_FluidPressure_ROSPublisher : public ROSPublisherInterface
{
  std::string frameId;
  PressureSensor * dev;
public:
  SimDev_FluidPressure_ROSPublisher(PressureSensor * dev, std::string topic, int rate, std::string frameId) :
    ROSPublisherInterface(topic, rate), dev(dev), frameId(frameId)
  {
  }

  void createPublisher(ros::NodeHandle &nh);
  void publish();

  ~SimDev_FluidPressure_ROSPublisher()
  {
  }
};


#endif
