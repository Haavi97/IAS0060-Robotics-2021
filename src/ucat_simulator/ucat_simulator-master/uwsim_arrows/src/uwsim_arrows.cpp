#include "uwsim_arrows/SimDev_DirectionalReceiver.h"
#include "uwsim_arrows/SimDev_VehicleFin.h"
#include "uwsim_arrows/SimDev_Flipper.h"
#include "uwsim_arrows/SimDev_Depth2Sonar.h"
#include "uwsim_arrows/SimDev_FluidPressure.h"
#include "uwsim_arrows/SimDev_BeaconReceiver.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(SimDev_DirectionalReceiver_Factory, uwsim::SimulatedDeviceFactory)
PLUGINLIB_EXPORT_CLASS(SimDev_VehicleFin_Factory, uwsim::SimulatedDeviceFactory)
PLUGINLIB_EXPORT_CLASS(SimDev_Flipper_Factory, uwsim::SimulatedDeviceFactory)
PLUGINLIB_EXPORT_CLASS(SimDev_Depth2Sonar_Factory, uwsim::SimulatedDeviceFactory)
PLUGINLIB_EXPORT_CLASS(SimDev_FluidPressure_Factory, uwsim::SimulatedDeviceFactory)
PLUGINLIB_EXPORT_CLASS(SimDev_BeaconReceiver_Factory, uwsim::SimulatedDeviceFactory)
