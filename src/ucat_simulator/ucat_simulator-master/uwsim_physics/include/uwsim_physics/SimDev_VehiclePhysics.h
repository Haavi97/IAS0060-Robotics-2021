/*
 * Copyright (c) 2013 Tallinn University of Technology.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 *
 * Contributors:
 *     Yuri Gavshin
 */

#ifndef SIMULATEDDEVICE_VEHICLEPHYSICS_H_
#define SIMULATEDDEVICE_VEHICLEPHYSICS_H_

#include "uwsim/SimulatedDevice.h"
using namespace uwsim;
#include "uwsim/ConfigXMLParser.h"
#include "uwsim/ROSInterface.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <osgOcean/OceanScene>
#include "uwsim/BulletPhysics.h"
#include "uwsim/SceneBuilder.h"
#include "SimDev_VehiclePhysicsForce.h"
#include <boost/thread/mutex.hpp>

class SimDev_VehiclePhysics_Config : public SimulatedDeviceConfig
{
public:
  std::string linkName;
  double debugForceRatio;
  std::vector<SimDev_VehiclePhysicsForce> thrusters;
  boost::shared_ptr<PhysicProperties> physicProperties;

  SimDev_VehiclePhysics_Config(std::string type_) :
      SimulatedDeviceConfig(type_)
  {
    linkName = "";
    debugForceRatio = 0;
  }
};

class SimDev_VehiclePhysics : public SimulatedDevice, public btActionInterface
{
  class VehicleState{
  public:
    btTransform pose;
    btVector3 angVel, linVel;
    int count;
    VehicleState(btTransform pose,btVector3 angVel, btVector3 linVel){
      this->pose = pose;
      this->angVel = angVel;
      this->linVel = linVel;
      count = 50;
    }
  };

  btVector3 vehicleLinearSpeed, vehicleAngularSpeed;
  double vehicleLinearSpeedAngle;
  //btQuaternion vehicleOrientation;

  boost::shared_ptr<VehicleState> setVehicleState;
  bool enabled;

  btRigidBody * rigidBody;
  double debugForceRatio;
  osg::Node * node;
  boost::shared_ptr<PhysicProperties> physicProperties;

  BulletPhysics * physics;
  SceneBuilder * sceneBuilder;
  osg::MatrixTransform * vehicleTransform;
  Link xmlLink;
  std::string linkName;

  int iterations;

  void applyPhysics(BulletPhysics * bulletPhysics);

  SimulatedIAUV * auv;

  boost::mutex jointPositionMutex;

public:
  btRigidBody * getRigidBody()
  {
    return rigidBody;
  }

  void setEnabled(bool enabled){
    this->enabled = enabled;
  }
  bool getEnabled()
  {
    return enabled;
  }

  btVector3 getLinearSpeed()
  {
    return vehicleLinearSpeed;
  }
  btVector3 getAngularSpeed()
  {
    return vehicleAngularSpeed;
  }

  double getLinearSpeedAngle()
  {
    return vehicleLinearSpeedAngle;
  }



  std::vector<boost::shared_ptr<SimDev_VehiclePhysicsForce> > forceGenerators;

  SimDev_VehiclePhysics(SimDev_VehiclePhysics_Config * cfg, osg::Node * node, SceneBuilder * sceneBuilder,
                        osg::MatrixTransform * vehicleTransform, Link _xmlLink, SimulatedIAUV * auv);

  //void preTickCallback(btDynamicsWorld* world, btScalar timeStep);
  virtual void stepSimulation(btCollisionWorld* world, btScalar timeStep);

  void updateAction(btCollisionWorld* world, btScalar timeStep);
  virtual void debugDraw(btIDebugDraw* dDraw);

  static boost::shared_ptr<SimDev_VehiclePhysics> getVehiclePhysics(boost::shared_ptr<SimulatedDevices> simdevs)
  {
    if (simdevs)
    {
      for (size_t i = 0; i < simdevs->all.size(); ++i)
        if (simdevs->all.at(i)->getType() == "vehiclePhysics")
          return boost::dynamic_pointer_cast<SimDev_VehiclePhysics, SimulatedDevice>(simdevs->all.at(i));
    }

    return boost::shared_ptr<SimDev_VehiclePhysics>();
  }

  void setJointPosition(std::vector<double> positions, std::vector<std::string> names);

  void setPoseAndSpeeds(btTransform pose, btVector3 angVel, btVector3 linVel);
};

class SimDev_VehiclePhysics_Factory : public SimulatedDeviceFactory
{
public:
  SimDev_VehiclePhysics_Factory(std::string type_ = "vehiclePhysics") :
      SimulatedDeviceFactory(type_)
  {
  }

  virtual SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node* node, ConfigFile * config);
  virtual bool applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *oscene, size_t iteration);

  std::vector<boost::shared_ptr<ROSInterface> >
  getInterface(ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile);
};

#endif /* SIMULATEDDEVICE_VEHICLEPHYSICS_H_ */
