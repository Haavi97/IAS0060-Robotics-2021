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

#ifndef SimDev_VehicleFin_H_
#define SimDev_VehicleFin_H_

#include "boost/shared_ptr.hpp"
#include "uwsim/BulletPhysics.h"
#include "uwsim_physics/SimDev_VehiclePhysicsForce.h"

class SimDev_VehicleFin_Config : public SimDev_VehiclePhysicsForce_Config
{
public:
  std::vector<double> masses;
  SimDev_VehicleFin_Config(std::string type_) :
      SimDev_VehiclePhysicsForce_Config(type_)
  {
  }
};

class FinCmd
{
public:
  std::vector<btVector3> forces;
  btScalar period; //period of transition from first wrench to last, 1/frequency
  btScalar curTime; //0 means first Wrench, period means second wrench, period/2 - in between
  btScalar endTime; //when to finish. <0 means infinity

  FinCmd(btScalar period_, btScalar curTime_, btScalar endTime_)
  {
    this->period = period_;
    this->curTime = curTime_;
    this->endTime = endTime_;
  }

  typedef boost::shared_ptr<FinCmd> Ptr;
};

typedef std::list<FinCmd::Ptr> FinCmds;

class SimDev_VehicleFin : public SimDev_VehiclePhysicsForce
{
  void updateCurrentCommand(btScalar timeStep);
  void setCurrentCommand(FinCmd::Ptr cmd);
public:
  SimDev_VehicleFin(SimDev_VehiclePhysicsForce_Config * cfg, osg::Node * node, osgOcean::OceanScene * oceanScene,
                    size_t subCfg);

  void calculateVectors(btRigidBody* vehicleBody, const osg::MatrixTransform* localizedWorld, BulletPhysics* physics,
                        btScalar timeStep, bool isFirstStep);

  int jointIndex;
  btVector3 jointZeroDir;
  FinCmds finCommands;
  boost::shared_ptr<FinCmd> currentCommand;

  static btVector3 calculateTransition(btVector3 from, btVector3 to, double subPhase);

  double getJointPosition();
};

class SimDev_VehicleFin_Factory : public SimDev_VehiclePhysicsForce_Factory
{
public:
  std::vector<double> masses;
  SimDev_VehicleFin_Factory(std::string type_ = "vehicleFin") :
      SimDev_VehiclePhysicsForce_Factory(type_)
  {
  }

  virtual boost::shared_ptr<SimDev_VehiclePhysicsForce> CreateInstance(SimDev_VehiclePhysicsForce_Config * cfg,
                                                                       osg::Node * node,
                                                                       osgOcean::OceanScene * oceanScene, size_t subCfg)
  {
    return boost::shared_ptr<SimDev_VehiclePhysicsForce>(new SimDev_VehicleFin(cfg, node, oceanScene, subCfg));
  }

  Ptr processConfig(const xmlpp::Node* node, ConfigFile* config, Ptr cfg_);
};

#endif /* SimDev_VehicleFin_H_ */
