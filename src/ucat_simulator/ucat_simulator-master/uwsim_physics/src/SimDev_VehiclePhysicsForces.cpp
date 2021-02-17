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

#include "uwsim_physics/SimDev_VehiclePhysicsForces.h"
#include "uwsim_physics/SimDev_VehiclePhysics.h"
#include "ros/ros.h"
#include "uwsim/SimulatedIAUV.h"
#include "uwsim/SceneBuilder.h"
#include <osg/Notify>
#include <assert.h>

void SimDev_VehicleDrag::calculateVectors(btRigidBody* vehicleBody, const osg::MatrixTransform* localizedWorld,
                                          BulletPhysics* physics, btScalar timeStep, bool isFirstStep)
{
  force.setZero();
  torque.setZero();

  btVector3 p = getRigidBody()->getCenterOfMassPosition();

  if (isFirstStep)
  { //ocean surface is updated once per iteration, but we are inside subiteration
    oceanSurfaceDelta = getOceanHeight(p) - lastOceanSurface;
    if (abs(oceanSurfaceDelta) > 50 || (iterations < 30 && iterations++))
      oceanSurfaceDelta = 0; //ignore huge waves and initial fluctuations
    oceanSurfaceDelta /= totalTime; //we take wave speed from previous iteration

    lastOceanSurface = getOceanHeight(p); //resetting values to calculate oceanSurfaceDelta on next first step
    totalTime = 0;
  }
  totalTime += timeStep;

  btVector3 waveVector(0, 0, -oceanSurfaceDelta); //if wave goes down, an object virtually goes up
  waveVector *= 1 / pow(2.0, fabs(p.getZ() - getOceanHeight(p))); //effect diminishes with depth exponentially 1.5 x depth^2:

  //applying translational drag and lift
  btVector3 velDirection = getRigidBody()->getLinearVelocity() + waveVector;

  if (getRigidBody()->getAngularVelocity().length2() + getTransform().getOrigin().length2() > 0)
  {
    btVector3 pos = getTransform().getOrigin();
    btTransform curRot = btTransform(getRigidBody()->getOrientation());
    btVector3 curRelPos = curRot * pos;
    btVector3 nextRelPos = curRelPos;
    btVector3 localRotation = getRigidBody()->getAngularVelocity() * timeStep;
    nextRelPos = nextRelPos.rotate(btVector3(0, 0, 1), localRotation.getZ());
    nextRelPos = nextRelPos.rotate(btVector3(0, 1, 0), localRotation.getY());
    nextRelPos = nextRelPos.rotate(btVector3(1, 0, 0), localRotation.getX());
    btVector3 velLocal = (nextRelPos - curRelPos) / timeStep;
    //if (!velLocal.fuzzyZero())
    //ROS_INFO("velLocal@%s: %s %.6f %s",this->name.c_str(), toString(velLocal, 10).c_str(), curRelPos.angle(nextRelPos), toString(localRotation,10).c_str());
    velDirection += velLocal;
  }

  btScalar vel2 = velDirection.length2();
  if (vel2 < 1)
    vel2 = pow(sqrt(vel2), 1.5);
  velDirection = velDirection.safeNormalize();

  if (vel2 > 0 && isfinite(vel2) && !this->translationalDrag.fuzzyZero())
  {
    btVector3 velCoeff = (btTransform(getRigidBody()->getOrientation().inverse()) * velDirection);
    velCoeff = velCoeff.safeNormalize();
    //F = 1/2 * p(=1000) * C (drag coefficient) * A(area) * v^2
    force = -500 * fabs(velCoeff.dot(this->translationalDrag)) * vel2 * velDirection;
  }

  if (vel2 > 0 && isfinite(vel2) && !this->translationalLift.fuzzyZero())
  {
    btVector3 velCoeff = (btTransform(getRigidBody()->getOrientation().inverse()) * velDirection);
    velCoeff = velCoeff.safeNormalize();
    btVector3 lift = btVector3(this->translationalLift.getX() * sin(velCoeff.angle(btVector3(1, 0, 0))),
                               this->translationalLift.getY() * sin(velCoeff.angle(btVector3(0, 1, 0))),
                               this->translationalLift.getZ() * sin(velCoeff.angle(btVector3(0, 0, 1))));
    btVector3 liftDir = btTransform(getRigidBody()->getOrientation()) * btVector3(0, 0, 1);
    btScalar CA = velCoeff.dot(lift);
    //F = 1/2 * p(=1000) * C (drag coefficient) * A(area) * v^2
    force += -500 * CA * vel2 * liftDir;
    //ROS_INFO("CA=%.3f, liftDir=%s, Vel=%s, force= %s", CA, toString(liftDir).c_str(), toString(velDirection).c_str(), toString(force).c_str());
  }

  if (force.length() > getMaxForce())
  {
    force = force.normalize() * getMaxForce();
  }

  if (this->rotationalDrag.length2() > 0)
    ROS_INFO("SimDev_VehicleDrag::calculateVectors: torque generation is deprecated, use only tranDrag and tranLift!");

  SimDev_VehiclePhysicsForce::calculateVectors(vehicleBody, localizedWorld, physics, timeStep, isFirstStep);
}

SimDev_VehicleMass::SimDev_VehicleMass(SimDev_VehiclePhysicsForce_Config * cfg, osg::Node * node,
                                       osgOcean::OceanScene * oceanScene, size_t subCfg) :
    SimDev_VehiclePhysicsForce(cfg, node, oceanScene, subCfg)
{
  this->mass = 0;
  this->isForceSet = false;
  SimDev_VehicleMass_Config * config = dynamic_cast<SimDev_VehicleMass_Config*>(cfg);
  if (config == NULL)
  {
    OSG_WARN<< "SimDev_VehicleMass constructor: passed cfg has wrong type"<<std::endl;
  }
  else
  {
    this->mass = config->masses[subCfg];
  }
}

void SimDev_VehicleMass::calculateVectors(btRigidBody* vehicleBody, const osg::MatrixTransform* localizedWorld,
                                          BulletPhysics* physics, btScalar timeStep, bool isFirstStep)
{
  torque.setZero();
  if (!isForceSet)
  {
    force = this->mass * physics->dynamicsWorld->getGravity();
    isForceSet = true;
  }
  SimDev_VehiclePhysicsForce::calculateVectors(vehicleBody, localizedWorld, physics, timeStep, isFirstStep);
}

SimDev_VehiclePhysicsForce_Config::Ptr SimDev_VehicleMass_Factory::processConfig(const xmlpp::Node* node,
                                                                                 ConfigFile* config, Ptr cfg_)
{
  SimDev_VehicleMass_Config * cfg = new SimDev_VehicleMass_Config(getType());
  cfg_.reset(cfg);
  cfg_ = SimDev_VehiclePhysicsForce_Factory::processConfig(node, config, cfg_);

  cfg->forceType = "absolute";

  double tmp;
  xmlpp::Node::NodeList list = node->get_children();
  for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
  {
    const xmlpp::Node* child = dynamic_cast<const xmlpp::Node*>(*iter);
    if (child->get_name() == "mass")
    {
      config->extractFloatChar(child, tmp);
      cfg->masses.push_back(tmp);
    }
  }

  return cfg_;
}

SimDev_VehicleDrag::SimDev_VehicleDrag(SimDev_VehiclePhysicsForce_Config* cfg, osg::Node* node,
                                       osgOcean::OceanScene* oceanScene, size_t subCfg) :
    SimDev_VehiclePhysicsForce(cfg, node, oceanScene, subCfg)
{
  oceanSurfaceDelta = 0;
  lastOceanSurface = 0;
  iterations = 0;
  totalTime = 0;
  this->translationalDrag = btVector3(0, 0, 0);
  this->rotationalDrag = btVector3(0, 0, 0);
  this->translationalLift = btVector3(0, 0, 0);

  SimDev_VehicleDrag_Config * config = dynamic_cast<SimDev_VehicleDrag_Config*>(cfg);
  if (config == NULL)
  {
    OSG_WARN<< "SimDev_VehicleDrag constructor: passed cfg has wrong type"<<std::endl;
  }
  else
  {
    this->translationalDrag = config->translationalDrag;
    this->rotationalDrag = config->rotationalDrag;
    this->translationalLift = config->translationalLift;
  }
}

SimDev_VehiclePhysicsForce_Config::Ptr SimDev_VehicleDrag_Factory::processConfig(
    const xmlpp::Node* node, ConfigFile* config, SimDev_VehiclePhysicsForce_Config::Ptr cfg_)
{
  SimDev_VehicleDrag_Config * cfg = new SimDev_VehicleDrag_Config(getType());
  cfg_.reset(cfg);
  cfg_ = SimDev_VehiclePhysicsForce_Factory::processConfig(node, config, cfg_);

  cfg->forceType = "absolute";

  double tmp3[3];
  xmlpp::Node::NodeList list = node->get_children();
  for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
  {
    const xmlpp::Node* child = dynamic_cast<const xmlpp::Node*>(*iter);
    if (child->get_name() == "tranDrag")
    {
      config->extractPositionOrColor(child, tmp3);
      cfg->translationalDrag = btVector3(tmp3[0], tmp3[1], tmp3[2]);
    }
    else if (child->get_name() == "rotDrag")
    {
      config->extractOrientation(child, tmp3);
      cfg->rotationalDrag = btVector3(tmp3[0], tmp3[1], tmp3[2]);
    }
    else if (child->get_name() == "tranLift")
    {
      config->extractPositionOrColor(child, tmp3);
      cfg->translationalLift = btVector3(tmp3[0], tmp3[1], tmp3[2]);
    }
  }

  return cfg_;
}
