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

#ifndef SIMULATEDDEVICE_VEHICLEPHYSICSFORCE_H
#define SIMULATEDDEVICE_VEHICLEPHYSICSFORCE_H

#include <iostream>
#include <iomanip>
#include <cmath>

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btActionInterface.h>
#include <osgOcean/OceanScene>

#include "uwsim/ConfigXMLParser.h"
#include "uwsim/SimulatedDevice.h"
using namespace uwsim;

#include "uwsim/ROSInterface.h"
#include <uwsim_physics_msgs/ForceCmd.h>

struct SimDev_VehiclePhysics;

class PointWrench
{
public:
  typedef enum ForceDirection
  {
    Zero, Absolute, Global, Local
  } type_t;
  btVector3 force, torque, applicationPoint;
};

struct BulletPhysics;

class SimDev_VehiclePhysicsForce_Config : public SimulatedDeviceConfig
{
public:
  typedef boost::shared_ptr<SimDev_VehiclePhysicsForce_Config> Ptr;
  std::string linkName, forceType;
  double debugForceRatio;

  SimulatedIAUV * auv;

  std::vector<double> radiuses, maxForces, maxTorques;
  std::vector<btVector3> positions, defaultForces, defaultTorques, colors;
  std::vector<btQuaternion> orientations;

  int isROS;

  SimDev_VehiclePhysicsForce_Config(std::string type_) :
      SimulatedDeviceConfig(type_)
  {
    debugForceRatio = 0;
    isROS = 0;
    auv = NULL;
  }

  virtual size_t getSubConfigNum()
  {
    size_t num = 0;
    num = max(radiuses.size(), num);
    num = max(positions.size(), num);
    num = max(orientations.size(), num);
    num = max(defaultForces.size(), num);
    num = max(defaultTorques.size(), num);
    num = max(maxForces.size(), num);
    num = max(maxTorques.size(), num);

    return num;
  }

  template<class T>
    void fillSubConfig(std::vector<T> & v, T d)
    {
      size_t subConfigsTotal = getSubConfigNum();
      size_t subConfigs = v.size();
      for (size_t i = subConfigs; i < subConfigsTotal; ++i)
      {
        v.push_back(subConfigs == 0 ? d : v.at(i % subConfigs));
      }
    }

  virtual void fillSubConfigs()
  {
    fillSubConfig<double>(this->radiuses, -1);
    fillSubConfig<double>(this->maxForces, -1);
    fillSubConfig<double>(this->maxTorques, -1);
    fillSubConfig<btVector3>(this->positions, btVector3(0, 0, 0));
    fillSubConfig<btQuaternion>(this->orientations, btQuaternion(0, 0, 0));
    fillSubConfig<btVector3>(this->defaultForces, btVector3(0, 0, 0));
    fillSubConfig<btVector3>(this->defaultTorques, btVector3(0, 0, 0));
    fillSubConfig<btVector3>(this->colors, btVector3(1, 1, 1));
  }
};

class SimDev_VehiclePhysicsForce : public SimulatedDevice
{
public:
  btVector3 forceVector, torqueVector, color;
  double debugForceRatio;
  int isROS;
  string linkName;
protected:
  btVector3 force, torque, defaultForce, defaultTorque;
  PointWrench::type_t forceType;

private:
  btTransform transform;
  osg::Node * node;
  btRigidBody * rigidBody;
  osgOcean::OceanScene * oceanScene;

  double radius, multiplier, maxForce, maxTorque;
  double impulseLength;
  void applyPhysics(BulletPhysics * bulletPhysics);

protected:

  osgOcean::OceanScene * getOceanScene()
  {
    return oceanScene;
  }

  double getOceanHeight(btVector3 pos)
  {
    return oceanScene->getOceanSurfaceHeightAt(pos.getX(), pos.getY());
  }

  double getMultiplier(const btRigidBody * vehicleRigidBody);

public:
  btRigidBody * getRigidBody()
  {
    return rigidBody;
  }
  void setRigidBody(btRigidBody * rb)
  {
    rigidBody = rb;
  }

  double getMultiplier()
  {
    return multiplier;
  }
  std::string getForceTypeChar()
  {
    switch (forceType)
    {
      case PointWrench::Absolute:
        return "a";
      case PointWrench::Global:
        return "g";
      case PointWrench::Local:
        return "l";
      case PointWrench::Zero:
        return "z";
      default:
        return "";
    }
  }

  btVector3 getForce()
  {
    return force;
  }
  void setForce(btVector3 f, double il)
  {
    force = f;
    impulseLength = il;
  }

  double getImpulseLength()
  {
    return impulseLength;
  }

  double getRadius()
  {
    return radius;
  }
  void setRadius(double r)
  {
    radius = r;
  }

  osg::Node * getNode()
  {
    return node;
  }

  btTransform getTransform()
  {
    return transform;
  }
  void setTransform(btTransform t)
  {
    transform = t;
  }

  double getMaxForce()
  {
    return maxForce;
  }

  SimDev_VehiclePhysicsForce(SimDev_VehiclePhysicsForce_Config * cfg, osg::Node * node,
                             osgOcean::OceanScene * oceanScene, size_t subCfg);

  btVector3 getApplicationPoint(const btRigidBody * vehicleBody);

  virtual void calculateVectors(btRigidBody * vehicleBody, const osg::MatrixTransform * localizedWorld,
                                BulletPhysics * physics, btScalar timeStep, bool isFirstStep);

  virtual ~SimDev_VehiclePhysicsForce()
  {
  }
};

std::string toString(const btVector3 & v, int precision = 6);

class SimDev_VehiclePhysicsForceROSPublisher : public ROSPublisherInterface
{
  SimDev_VehiclePhysics * vehiclePhysics;
  std::vector<boost::shared_ptr<SimDev_VehiclePhysicsForce> > thrusters;
  std::string type;
public:
  void createPublisher(ros::NodeHandle& nh);
  void publish();

  SimDev_VehiclePhysicsForceROSPublisher(std::string type, std::string topic, int rate,
                                         SimDev_VehiclePhysics * vehiclePhysics);
};

class SimDev_VehiclePhysicsForceROSSubscriber : public ROSSubscriberInterface
{
  SimDev_VehiclePhysics * vehiclePhysics;
  std::vector<boost::shared_ptr<SimDev_VehiclePhysicsForce> > thrusters;
  std::string type;
public:
  void processData(const uwsim_physics_msgs::ForceCmd::ConstPtr& cmd);
  void createSubscriber(ros::NodeHandle& nh);

  SimDev_VehiclePhysicsForceROSSubscriber(std::string type, std::string topic, SimDev_VehiclePhysics * vehiclePhysics);
};

class SimDev_VehiclePhysicsForce_Factory : public SimulatedDeviceFactory
{
public:
  typedef boost::shared_ptr<SimDev_VehiclePhysicsForce_Config> Ptr;

  SimDev_VehiclePhysicsForce_Factory(std::string type_ = "vehicleForce") :
      SimulatedDeviceFactory(type_)
  {
  }

  SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node* node, ConfigFile * config);

  virtual Ptr processConfig(const xmlpp::Node* node, ConfigFile * config, Ptr cfg);

  virtual bool applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *oscene, size_t iteration);

  virtual std::vector<boost::shared_ptr<ROSInterface> >
  getInterface(ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile);

  virtual boost::shared_ptr<SimDev_VehiclePhysicsForce> CreateInstance(SimDev_VehiclePhysicsForce_Config * cfg,
                                                                       osg::Node * node,
                                                                       osgOcean::OceanScene * oceanScene,
                                                                       size_t subCfg);
};

#endif // SIMULATEDDEVICE_VEHICLEPHYSICSFORCE_H
