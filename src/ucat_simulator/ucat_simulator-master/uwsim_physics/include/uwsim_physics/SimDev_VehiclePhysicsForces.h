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

#ifndef SIMULATEDDEVICE_VEHICLEPHYSICSFORCES_H_
#define SIMULATEDDEVICE_VEHICLEPHYSICSFORCES_H_

#include "SimDev_VehiclePhysicsForce.h"

class SimDev_VehicleMass_Config : public SimDev_VehiclePhysicsForce_Config
{
public:
  std::vector<double> masses;
  SimDev_VehicleMass_Config(std::string type_) :
      SimDev_VehiclePhysicsForce_Config(type_)
  {
  }

  virtual size_t getSubConfigNum()
  {
    return max(SimDev_VehiclePhysicsForce_Config::getSubConfigNum(), masses.size());
  }

  void fillSubConfigs()
  {
    SimDev_VehiclePhysicsForce_Config::fillSubConfigs();
    fillSubConfig<double>(this->masses, 0);
  }
};

class SimDev_VehicleMass : public SimDev_VehiclePhysicsForce
{
  double mass;
  bool isForceSet;
public:
  SimDev_VehicleMass(SimDev_VehiclePhysicsForce_Config * cfg, osg::Node * node, osgOcean::OceanScene * oceanScene,
                     size_t subCfg);

  void calculateVectors(btRigidBody* vehicleBody, const osg::MatrixTransform* localizedWorld, BulletPhysics* physics,
                        btScalar timeStep, bool isFirstStep);
};

class SimDev_VehicleMass_Factory : public SimDev_VehiclePhysicsForce_Factory
{
public:
  std::vector<double> masses;
  SimDev_VehicleMass_Factory(std::string type_ = "vehicleMass") :
      SimDev_VehiclePhysicsForce_Factory(type_)
  {
  }

  virtual boost::shared_ptr<SimDev_VehiclePhysicsForce> CreateInstance(SimDev_VehiclePhysicsForce_Config * cfg,
                                                                       osg::Node * node,
                                                                       osgOcean::OceanScene * oceanScene, size_t subCfg)
  {
    return boost::shared_ptr<SimDev_VehiclePhysicsForce>(new SimDev_VehicleMass(cfg, node, oceanScene, subCfg));
  }

  Ptr processConfig(const xmlpp::Node* node, ConfigFile* config, Ptr cfg_);
};

class SimDev_VehicleBuoyance_Config : public SimDev_VehicleMass_Config
{
public:
  SimDev_VehicleBuoyance_Config(std::string type_) :
      SimDev_VehicleMass_Config(type_)
  {
  }
};

class SimDev_VehicleBuoyance : public SimDev_VehicleMass
{
public:
  SimDev_VehicleBuoyance(SimDev_VehiclePhysicsForce_Config * cfg, osg::Node * node, osgOcean::OceanScene * oceanScene,
                         size_t subCfg) :
      SimDev_VehicleMass(cfg, node, oceanScene, subCfg)
  {
  }
};

class SimDev_VehicleBuoyance_Factory : public SimDev_VehicleMass_Factory
{
public:
  SimDev_VehicleBuoyance_Factory(std::string type_ = "vehicleBuoyance") :
      SimDev_VehicleMass_Factory(type_)
  {
  }

  virtual boost::shared_ptr<SimDev_VehiclePhysicsForce> CreateInstance(SimDev_VehiclePhysicsForce_Config * cfg,
                                                                       osg::Node * node,
                                                                       osgOcean::OceanScene * oceanScene, size_t subCfg)
  {
    return boost::shared_ptr<SimDev_VehiclePhysicsForce>(new SimDev_VehicleBuoyance(cfg, node, oceanScene, subCfg));
  }
};

class SimDev_VehicleDrag_Config : public SimDev_VehiclePhysicsForce_Config
{
public:
  btVector3 translationalDrag, rotationalDrag, translationalLift;

  SimDev_VehicleDrag_Config(std::string type_) :
      SimDev_VehiclePhysicsForce_Config(type_)
  {
    this->translationalDrag = btVector3(0, 0, 0);
    this->rotationalDrag = btVector3(0, 0, 0);
    this->translationalLift = btVector3(0, 0, 0);
  }
};

class SimDev_VehicleDrag : public SimDev_VehiclePhysicsForce
{
  double lastOceanSurface, oceanSurfaceDelta, totalTime;
  int iterations;
  btVector3 translationalDrag, rotationalDrag, translationalLift;
public:
  SimDev_VehicleDrag(SimDev_VehiclePhysicsForce_Config * cfg, osg::Node * node, osgOcean::OceanScene * oceanScene,
                     size_t subCfg);

  void calculateVectors(btRigidBody* vehicleBody, const osg::MatrixTransform* localizedWorld, BulletPhysics* physics,
                        btScalar timeStep, bool isFirstStep);
};

class SimDev_VehicleDrag_Factory : public SimDev_VehiclePhysicsForce_Factory
{
public:
  btVector3 translationalDrag, rotationalDrag;

  SimDev_VehicleDrag_Factory(std::string type_ = "vehicleDrag") :
      SimDev_VehiclePhysicsForce_Factory(type_)
  {
  }

  virtual boost::shared_ptr<SimDev_VehiclePhysicsForce> CreateInstance(SimDev_VehiclePhysicsForce_Config * cfg,
                                                                       osg::Node * node,
                                                                       osgOcean::OceanScene * oceanScene, size_t subCfg)
  {
    return boost::shared_ptr<SimDev_VehiclePhysicsForce>(new SimDev_VehicleDrag(cfg, node, oceanScene, subCfg));
  }

  Ptr processConfig(const xmlpp::Node* node, ConfigFile* config, Ptr cfg_);
};

class SimDev_VehicleThruster_Config : public SimDev_VehiclePhysicsForce_Config
{
public:
  SimDev_VehicleThruster_Config(std::string type_) :
      SimDev_VehiclePhysicsForce_Config(type_)
  {
  }
};

class SimDev_VehicleThruster : public SimDev_VehiclePhysicsForce
{
  double lastOceanSurface;
  btVector3 translationalDrag, rotationalDrag;
public:
  SimDev_VehicleThruster(SimDev_VehiclePhysicsForce_Config * cfg, osg::Node * node, osgOcean::OceanScene * oceanScene,
                         size_t subCfg) :
      SimDev_VehiclePhysicsForce(cfg, node, oceanScene, subCfg)
  {
  }
};

class SimDev_VehicleThruster_Factory : public SimDev_VehiclePhysicsForce_Factory
{
public:
  SimDev_VehicleThruster_Factory(std::string type_ = "vehicleThruster") :
      SimDev_VehiclePhysicsForce_Factory(type_)
  {
  }

  virtual boost::shared_ptr<SimDev_VehiclePhysicsForce> CreateInstance(SimDev_VehiclePhysicsForce_Config * cfg,
                                                                       osg::Node * node,
                                                                       osgOcean::OceanScene * oceanScene, size_t subCfg)
  {
    return boost::shared_ptr<SimDev_VehiclePhysicsForce>(new SimDev_VehicleThruster(cfg, node, oceanScene, subCfg));
  }

  Ptr processConfig(const xmlpp::Node* node, ConfigFile* config, Ptr cfg)
  {
    if (!cfg)
      cfg.reset(new SimDev_VehiclePhysicsForce_Config(getType()));
    cfg->isROS = 1;
    cfg = SimDev_VehiclePhysicsForce_Factory::processConfig(node, config, cfg);
    if (cfg)
      cfg->forceType = "local";
    return cfg;
  }
};

#endif /* SIMULATEDDEVICE_VEHICLEPHYSICSFORCES_H_ */
