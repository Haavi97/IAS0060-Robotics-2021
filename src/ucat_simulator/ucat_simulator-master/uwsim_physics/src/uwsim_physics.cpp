#include "uwsim_physics/SimDev_VehiclePhysics.h"
#include "uwsim_physics/SimDev_VehiclePhysicsForce.h"
#include "uwsim_physics/SimDev_VehiclePhysicsForces.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(SimDev_VehiclePhysics_Factory, uwsim::SimulatedDeviceFactory)
PLUGINLIB_EXPORT_CLASS(SimDev_VehiclePhysicsForce_Factory, uwsim::SimulatedDeviceFactory)
PLUGINLIB_EXPORT_CLASS(SimDev_VehicleBuoyance_Factory, uwsim::SimulatedDeviceFactory)
PLUGINLIB_EXPORT_CLASS(SimDev_VehicleDrag_Factory, uwsim::SimulatedDeviceFactory)
PLUGINLIB_EXPORT_CLASS(SimDev_VehicleMass_Factory, uwsim::SimulatedDeviceFactory)
PLUGINLIB_EXPORT_CLASS(SimDev_VehicleThruster_Factory, uwsim::SimulatedDeviceFactory)
