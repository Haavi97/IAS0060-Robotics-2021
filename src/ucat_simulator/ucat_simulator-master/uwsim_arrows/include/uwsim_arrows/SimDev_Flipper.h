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

#ifndef SimDev_Flipper_H_
#define SimDev_Flipper_H_

#include "boost/shared_ptr.hpp"
#include "uwsim/BulletPhysics.h"
#include "uwsim_physics/SimDev_VehiclePhysicsForce.h"
#include "uwsim_physics_msgs/SkeletonMovement.h"
#include "tut_arrows_msgs/Flippers.h"

class SimDev_Flipper_Config : public SimDev_VehiclePhysicsForce_Config
{
public:
  std::string side;
  std::vector<std::string> jointNames;
  double forceAngle, forceMultiplier, maxForce, multPower, borderBack, borderFront;

  SimDev_Flipper_Config(std::string type_) :
      SimDev_VehiclePhysicsForce_Config(type_)
  {
    //values for quite small fin, (approx 4x10cm)
    forceMultiplier = 0.1f;
    maxForce = 1.0;
    multPower = 1.2;

    borderBack = 1.1;
    borderFront = 1.0;
    forceAngle = -0.4;
  }
};

class SimDev_Flipper : public SimDev_VehiclePhysicsForce
{
  SimulatedIAUV * auv;
  double forceAngle, forceMultiplier, maxForce, multPower, zeroDirection;
  int jointIndex;
public:
  boost::shared_ptr<SimDev_VehiclePhysics> vehiclePhysics;

  SimDev_Flipper(SimDev_VehiclePhysicsForce_Config * cfg, osg::Node * node, osgOcean::OceanScene * oceanScene,
                     size_t subCfg);

  std::vector<std::string> jointNames;

  btVector3 calculateForce(float angularSpeed);

  virtual void setFlipperCmd(tut_arrows_msgs::Flipper flipper);
  boost::shared_ptr<tut_arrows_msgs::Flipper> flipperCmd;
  double zeroPhaseMoment;

  virtual void calculateVectors(btRigidBody * vehicleBody, const osg::MatrixTransform * localizedWorld,
                                BulletPhysics * physics, btScalar timeStep, bool isFirstStep);

  void generateFinForceAndPosition(double idealPos, double timeStep);
};

class SimDev_Flipper_Factory : public SimDev_VehiclePhysicsForce_Factory
{
public:
  SimDev_Flipper_Factory(std::string type_ = "flipper") :
      SimDev_VehiclePhysicsForce_Factory(type_)
  {
  }

  virtual boost::shared_ptr<SimDev_VehiclePhysicsForce> CreateInstance(SimDev_VehiclePhysicsForce_Config * cfg,
                                                                       osg::Node * node,
                                                                       osgOcean::OceanScene * oceanScene, size_t subCfg)
  {
    return boost::shared_ptr<SimDev_VehiclePhysicsForce>(new SimDev_Flipper(cfg, node, oceanScene, subCfg));
  }

  Ptr processConfig(const xmlpp::Node* node, ConfigFile* config, Ptr cfg_);

  virtual std::vector<boost::shared_ptr<ROSInterface> > getInterface(
      ROSInterfaceInfo& rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> >& iauvFile);

};

class SimDev_FlipperROSSubscriber : public ROSSubscriberInterface
{
  boost::shared_ptr<SimDev_VehiclePhysics> vehiclePhysics;
  ros::Subscriber flippers_sub_;
  std::vector<boost::shared_ptr<SimDev_Flipper> > flippers;

public:
  void processData(const tut_arrows_msgs::FlippersConstPtr& cmd);
  void createSubscriber(ros::NodeHandle& nh);

  SimDev_FlipperROSSubscriber(std::string topic, SimulatedIAUV * auv);
};

#endif /* SimDev_Flipper_H_ */
