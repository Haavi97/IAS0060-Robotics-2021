//"DirectionalReceiver" example, SimulatedDevice_DirectionalReceiver.cpp

#include "uwsim_arrows/SimDev_FluidPressure.h"
#include "uwsim/SceneBuilder.h"
#include "sensor_msgs/FluidPressure.h"
#include <iostream>
#include <vector>


std::vector<boost::shared_ptr<ROSInterface> > SimDev_FluidPressure_Factory::getInterface(
    ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile)
{
  std::vector<boost::shared_ptr<ROSInterface> > ifaces;

  for (size_t i = 0; i < iauvFile.size(); ++i)
    for (size_t d = 0; d < iauvFile[i]->pressure_sensors.size(); ++d)
      if (iauvFile[i]->pressure_sensors[d].name == rosInterface.targetName)
      {
        PressureSensor * cam = &(iauvFile[i]->pressure_sensors[d]);
        ifaces.push_back(
            boost::shared_ptr<ROSInterface>(
                new SimDev_FluidPressure_ROSPublisher(cam, rosInterface.topic, rosInterface.rate,
                                                      rosInterface.values.at("frameId"))
            )
        );
      }
  if (ifaces.size() == 0)
    ROS_WARN("Returning empty ROS interface for device %s...", rosInterface.targetName.c_str());

  return ifaces;
}

void SimDev_FluidPressure_ROSPublisher::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("SimDev_FluidPressure_ROSPublisher on topic %s", topic.c_str());
  pub_ = nh.advertise<sensor_msgs::FluidPressure>(topic, 1);
}

void SimDev_FluidPressure_ROSPublisher::publish()
{
  if (dev!=NULL)
  {
    sensor_msgs::FluidPressure msg;
    msg.header.stamp = getROSTime();
    msg.header.frame_id = frameId;
    msg.fluid_pressure = (1+fabs(dev->getMeasurement())*0.1)*101325;
    msg.variance = dev->getStandardDeviation()*101325;
    pub_.publish(msg);
  }
}
