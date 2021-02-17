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

#include "uwsim_physics/SimDev_VehiclePhysics.h"
#include <limits>

SimulatedDeviceConfig::Ptr SimDev_VehiclePhysics_Factory::processConfig(const xmlpp::Node* node, ConfigFile * config)
{
  SimDev_VehiclePhysics_Config * cfg = new SimDev_VehiclePhysics_Config(getType());
  xmlpp::Node::NodeList list = node->get_children();
  for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
  {
    const xmlpp::Node* child = dynamic_cast<const xmlpp::Node*>(*iter);
    if (child->get_name() == "linkName")
      config->extractStringChar(child, cfg->linkName);
    else if (child->get_name() == "debugForceRatio")
      config->extractFloatChar(child, cfg->debugForceRatio);
    else if (child->get_name() == "physics")
    {
      PhysicProperties pp;
      pp.init();
      config->processPhysicProperties(child, pp);
      cfg->physicProperties.reset(new PhysicProperties(pp));
    }
  }
  return SimulatedDeviceConfig::Ptr(cfg);
}

bool SimDev_VehiclePhysics_Factory::applyConfig(SimulatedIAUV * auv, Vehicle &vehicle, SceneBuilder * sceneBuilder,
                                                size_t iteration)
{
  if (iteration > 0)
    return true;
  for (size_t i = 0; i < vehicle.simulated_devices.size(); ++i)
    if (vehicle.simulated_devices[i]->getType() == this->getType())
    {
      SimDev_VehiclePhysics_Config * cfg =
          static_cast<SimDev_VehiclePhysics_Config *>(vehicle.simulated_devices[i].get());

      int nodeIndex = -1;
      for (int j = 0; j < vehicle.nlinks && nodeIndex < 0; j++)
        if (vehicle.links[j].name == cfg->linkName)
          nodeIndex = j;

      if (nodeIndex >= 0)
      {
        osg::Node * node = auv->urdf->link[nodeIndex].get();
        boost::shared_ptr<SimDev_VehiclePhysics> physics = boost::shared_ptr<SimDev_VehiclePhysics>(
            new SimDev_VehiclePhysics(cfg, node, sceneBuilder, auv->baseTransform.get(), vehicle.links[nodeIndex],
                                      auv));
        auv->devices->all.push_back(physics);
      }
      else
      OSG_FATAL<< "VehiclePhysics inside robot '"<<vehicle.name<<"' is attached to unknown link '"<<cfg->linkName<<"', discarding..."<<std::endl;
    }
  return true;
}

std::vector<boost::shared_ptr<ROSInterface> > SimDev_VehiclePhysics_Factory::getInterface(
    ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile)
{
  return std::vector<boost::shared_ptr<ROSInterface> >();
}

void SimDev_VehiclePhysics::applyPhysics(BulletPhysics * bulletPhysics)
{
  this->physics = bulletPhysics;

  osg::ref_ptr<NodeDataType> data = dynamic_cast<NodeDataType*>(node->getUserData());
  btRigidBody * rb = data->rigidBody;

  rigidBody = physics->addObject(vehicleTransform, node, static_cast<CollisionDataType*>(rb->getUserPointer()),
                                 this->physicProperties,
                                 xmlLink.cs == NULL ? NULL : UWSimGeometry::loadGeometry(xmlLink.cs));
  rigidBody->setGravity(btVector3(0, 0, 0)); //gravity is applied with mass force
  physics->dynamicsWorld->addAction(this); //needed for debug

  //reordering force generators, so that drag generators are the last ones
  std::vector<boost::shared_ptr<SimDev_VehiclePhysicsForce> > forceGenerators_;
  for (size_t i = 0; i < forceGenerators.size(); ++i)
    if (forceGenerators.at(i)->getType() != "vehicleDrag")
      forceGenerators_.push_back(forceGenerators.at(i));
  for (size_t i = 0; i < forceGenerators.size(); ++i)
    if (forceGenerators.at(i)->getType() == "vehicleDrag")
      forceGenerators_.push_back(forceGenerators.at(i));
  this->forceGenerators = forceGenerators_;

  for (size_t i = 0; i < forceGenerators.size(); ++i)
    if (forceGenerators.at(i)->linkName == this->linkName)
      forceGenerators.at(i)->setRigidBody(rigidBody);
}

SimDev_VehiclePhysics::SimDev_VehiclePhysics(SimDev_VehiclePhysics_Config * cfg, osg::Node * node,
                                             SceneBuilder * sceneBuilder, osg::MatrixTransform * vehicleTransform,
                                             Link _xmlLink, SimulatedIAUV * auv) :
    SimulatedDevice(cfg)
{
  this->rigidBody = NULL;
  this->physics = NULL;

  this->sceneBuilder = sceneBuilder;
  this->vehicleTransform = vehicleTransform;
  this->node = node;
  this->debugForceRatio = cfg->debugForceRatio;
  this->physicProperties = cfg->physicProperties;
  this->xmlLink = _xmlLink;
  this->linkName = cfg->linkName;

  this->auv = auv;

  iterations = 0;

  setEnabled(true);
  vehicleLinearSpeed.setZero();
  vehicleAngularSpeed.setZero();
  //vehicleOrientation.setValue(0,0,0,0);
  vehicleLinearSpeedAngle = 0;
}

void SimDev_VehiclePhysics::updateAction(btCollisionWorld* world, btScalar timeStep)
{
  stepSimulation(world, timeStep);
}

void SimDev_VehiclePhysics::stepSimulation(btCollisionWorld * world, btScalar timeStep)
{
  rigidBody->clearForces();

  if (setVehicleState)
  {
    this->rigidBody->setWorldTransform(setVehicleState->pose);
    this->rigidBody->setLinearVelocity(setVehicleState->linVel);
    this->rigidBody->setAngularVelocity(setVehicleState->angVel);
    setVehicleState->count--;
    if (setVehicleState->count<=0)
      setVehicleState.reset();
    return;
  }

  if (!getEnabled()){
    this->rigidBody->setLinearVelocity(btVector3(0,0,0));
    this->rigidBody->setAngularVelocity(btVector3(0,0,0));
    vehicleLinearSpeed.setZero();
    vehicleAngularSpeed.setZero();
    vehicleLinearSpeedAngle = 0;
    //vehicleOrientation.setValue(0,0,0,0);
    return;
  }



  std::vector<btVector3> points;
  for (size_t i = 0; i < forceGenerators.size(); ++i)
  {
    points.push_back(forceGenerators[i]->getApplicationPoint(rigidBody));
  }
  for (size_t i = 0; i < forceGenerators.size(); ++i)
  {
    forceGenerators[i]->calculateVectors(rigidBody, sceneBuilder->scene->localizedWorld, this->physics, timeStep,
                                         true);
    rigidBody->applyForce(forceGenerators[i]->forceVector, points.at(i));

    if (forceGenerators[i]->torqueVector.length() > 0)
      ROS_ERROR(
          "SimDev_VehiclePhysics::stepSimulation[%s]: torque generation is deprecated, use only tranDrag and tranLift!",
          forceGenerators[i]->name.c_str());
  }

  btVector3 linear = rigidBody->getLinearVelocity();
  btVector3 angular = rigidBody->getAngularVelocity();

  double newWeight = 0.05;
  double oldWeight = 1 - newWeight;
  vehicleLinearSpeed = oldWeight*vehicleLinearSpeed + newWeight*linear;
  vehicleAngularSpeed = oldWeight*vehicleAngularSpeed + newWeight*angular;
  vehicleLinearSpeedAngle = vehicleLinearSpeed.angle( btTransform(rigidBody->getWorldTransform().getRotation())*btVector3(1,0,0) );
}

void SimDev_VehiclePhysics::debugDraw(btIDebugDraw* dDraw)
{
  for (size_t i = 0; i < forceGenerators.size(); ++i)
  {
    double dbg = forceGenerators[i]->debugForceRatio;
    if (dbg < 0)
      dbg = debugForceRatio;
    if (dbg <= 0)
      continue;

    btVector3 p0 = this->rigidBody->getWorldTransform().getOrigin()
            + forceGenerators[i]->getApplicationPoint(rigidBody);
    btVector3 p1 = p0 + forceGenerators[i]->forceVector * dbg;

    dDraw->drawLine(p0, p1, forceGenerators[i]->color);

    if (debugForceRatio > 0)
      OSG_ALWAYS<<"Forces['"<<this->name<<"']['"<<forceGenerators[i]->getType()<<"'.'"<<forceGenerators[i]->name<<"']="
      <<toString(forceGenerators[i]->forceVector,12) <<" at "<<toString(forceGenerators[i]->getApplicationPoint(rigidBody), 3)<<std::endl;
  }

  btVector3 p0 = this->rigidBody->getWorldTransform().getOrigin()+btVector3(0,0,0);
  btVector3 p1 = p0 + vehicleLinearSpeed*debugForceRatio;
  dDraw->drawLine(p0, p1, btVector3(0,0,1));
}


void SimDev_VehiclePhysics::setJointPosition(std::vector<double> positions, std::vector<std::string> names)
{
  boost::mutex::scoped_lock scoped_lock(jointPositionMutex);

  auv->urdf->setJointPosition(positions, names);
}

void SimDev_VehiclePhysics::setPoseAndSpeeds(btTransform pose, btVector3 angVel, btVector3 linVel)
{
  setVehicleState.reset(new VehicleState(pose, angVel, linVel));
}

