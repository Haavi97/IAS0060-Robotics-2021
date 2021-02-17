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

#include "uwsim_physics/SimDev_VehiclePhysicsForce.h"
#include "uwsim_physics/SimDev_VehiclePhysics.h"
#include "ros/ros.h"
#include "uwsim/SimulatedIAUV.h"
#include "uwsim/SceneBuilder.h"
#include <osg/Notify>

float ITERATE = 0;
int longueur = 0;
int start = 0; 
SimulatedDeviceConfig::Ptr SimDev_VehiclePhysicsForce_Factory::processConfig(const xmlpp::Node* node,
                                                                             ConfigFile * config)
{
  Ptr cfg;
  return processConfig(node, config, cfg);
}


SimDev_VehiclePhysicsForce_Config::Ptr SimDev_VehiclePhysicsForce_Factory::processConfig(const xmlpp::Node* node,
                                                                                         ConfigFile * config, Ptr cfg)
{
  if (!cfg)
    cfg.reset(new SimDev_VehiclePhysicsForce_Config(this->getType()));

  double tmp3[3];
  double tmp;
  xmlpp::Node::NodeList list = node->get_children();
  for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
  {
    const xmlpp::Node* child = dynamic_cast<const xmlpp::Node*>(*iter);
    if (child->get_name() == "relativeTo")
      config->extractStringChar(child, cfg->linkName);
    else if (child->get_name() == "forceType")
      config->extractStringChar(child, cfg->forceType);
    else if (child->get_name() == "radius")
    {
      config->extractFloatChar(child, tmp);
      cfg->radiuses.push_back(tmp);
    }
    else if (child->get_name() == "maxForce")
    {
      config->extractFloatChar(child, tmp);
      cfg->maxForces.push_back(tmp);
    }
    else if (child->get_name() == "maxTorque")
    {
      config->extractFloatChar(child, tmp);
      cfg->maxTorques.push_back(tmp);
    }
    else if (child->get_name() == "debugForceRatio")
      config->extractFloatChar(child, cfg->debugForceRatio);
    else if (child->get_name() == "position")
    {
      config->extractPositionOrColor(child, tmp3);
      cfg->positions.push_back(btVector3(tmp3[0], tmp3[1], tmp3[2]));
    }
    else if (child->get_name() == "orientation")
    {
      config->extractPositionOrColor(child, tmp3);
      cfg->orientations.push_back(btQuaternion(tmp3[0], tmp3[1], tmp3[2]));
    }
    else if (child->get_name() == "defaultForce")
    {
      config->extractPositionOrColor(child, tmp3);
      cfg->defaultForces.push_back(btVector3(tmp3[0], tmp3[1], tmp3[2]));
    }
    else if (child->get_name() == "defaultTorque")
    {
      config->extractPositionOrColor(child, tmp3);
      cfg->defaultTorques.push_back(btVector3(tmp3[0], tmp3[1], tmp3[2]));
    }
    else if (child->get_name() == "color")
    {
      config->extractPositionOrColor(child, tmp3);
      cfg->colors.push_back(btVector3(tmp3[0], tmp3[1], tmp3[2]));
    }
    else if (child->get_name() == "isROS")
    {
      config->extractIntChar(child, cfg->isROS);
    }
  }

  return cfg;
}

bool SimDev_VehiclePhysicsForce_Factory::applyConfig(SimulatedIAUV * auv, Vehicle &vehicle, SceneBuilder * sceneBuilder,
                                                     size_t iteration)
{
  if (iteration < 1)
    return false;
  if (iteration > 1)
    return true;
  for (size_t i = 0; i < vehicle.simulated_devices.size(); ++i)
    if (vehicle.simulated_devices[i]->getType() == this->getType())
    {
      SimDev_VehiclePhysicsForce_Config * cfg =
          static_cast<SimDev_VehiclePhysicsForce_Config *>(vehicle.simulated_devices[i].get());

      int nodeIndex = -1;
      for (int j = 0; j < vehicle.nlinks && nodeIndex < 0; j++)
        if (vehicle.links[j].name == cfg->linkName)
          nodeIndex = j;

      if (nodeIndex >= 0)
      {
        osg::Node * node = auv->urdf->link[nodeIndex].get();

        SimDev_VehiclePhysicsForce_Config * cfg =
            dynamic_cast<SimDev_VehiclePhysicsForce_Config *>(vehicle.simulated_devices[i].get());
        cfg->fillSubConfigs();
        cfg->auv = auv;

        size_t subConfigsTotal = cfg->getSubConfigNum();
        for (size_t i = 0; i < subConfigsTotal; ++i)
        {
          boost::shared_ptr<SimDev_VehiclePhysicsForce> dev = this->CreateInstance(cfg, node,
                                                                                   sceneBuilder->scene->getOceanScene(),
                                                                                   i);
          if (subConfigsTotal > 1)
          {
            std::stringstream ss;
            ss << "_" << i;
            dev->name += ss.str();
          }
          if (!dev)
          {
            OSG_FATAL<<"SimDev_VehiclePhysicsForce: CreateInstance returned nothing for device '"<<vehicle.simulated_devices[i]->name<<"' in vehicle '"<<vehicle.name<<"'"<<std::endl;
          }
          else
          {
            if (!SimDev_VehiclePhysics::getVehiclePhysics(auv->devices))
            {
              OSG_FATAL<<"SimDev_VehiclePhysicsForce_Config::applyConfig: VehiclePhysics is not set. Cannot add force generator '"<<dev->name<<"' to '"<< auv->name <<"'."<<std::endl;
            }
            else
            {
              SimDev_VehiclePhysics::getVehiclePhysics(auv->devices)->forceGenerators.push_back(dev);
              auv->devices->all.push_back(dev);
              OSG_NOTICE << "Added force generator '"<<dev->name<<"' to '"<< auv->name <<"'.";
            }
          }
        }

        if (subConfigsTotal == 0)
          OSG_FATAL<< "ForceGenerator '"<<cfg->name<<"' in '"<<auv->name<<"' has 0 sub-configurations"<<std::endl;
        }
        else
        OSG_FATAL<<"VehiclePhysics inside robot '"<<vehicle.name<<"' is attached to unknown link '"<<cfg->linkName<<"', discarding..."<<std::endl;
      }
  return true;
}

std::vector<boost::shared_ptr<ROSInterface> > SimDev_VehiclePhysicsForce_Factory::getInterface(
    ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile)
{
  std::vector<boost::shared_ptr<ROSInterface> > ifaces;

  SimDev_VehiclePhysics * vehicle = NULL;

  for (size_t i = 0; i < iauvFile.size(); ++i)
    if (iauvFile[i]->name == rosInterface.targetName)
      vehicle = SimDev_VehiclePhysics::getVehiclePhysics(iauvFile[i]->devices).get();

  if (vehicle != NULL)
  {
    if (rosInterface.topic.length() > 0)
    {
      ifaces.push_back(
          boost::shared_ptr<ROSInterface>(
              new SimDev_VehiclePhysicsForceROSSubscriber(rosInterface.subtype, rosInterface.topic, vehicle)));
    }
    if (rosInterface.infoTopic.length() > 0)
    {
      ifaces.push_back(
          boost::shared_ptr<ROSInterface>(
              new SimDev_VehiclePhysicsForceROSPublisher(rosInterface.subtype, rosInterface.infoTopic,
                                                         rosInterface.rate, vehicle)));
    }
  }
  else
    ROS_WARN("%sROS: vehicle '%s' not found, skipping...", rosInterface.subtype.c_str(),
             rosInterface.targetName.c_str());
  return ifaces;
}

void SimDev_VehiclePhysicsForce::applyPhysics(BulletPhysics * bulletPhysics)
{
  if (rigidBody == NULL)
  { //it is possible that physics is already set by a VehiclePhysics class
    osg::ref_ptr<NodeDataType> data = dynamic_cast<NodeDataType*>(node->getUserData());
    rigidBody = data->rigidBody;

  }
}

double SimDev_VehiclePhysicsForce::getMultiplier(const btRigidBody * vehicleRigidBody)
{
  multiplier = 1;
  //double d = 0;

  if (radius >= 0)
  {
    btVector3 p = vehicleRigidBody->getCenterOfMassPosition() + getApplicationPoint(vehicleRigidBody);

    double topDepth = p.getZ() + radius - getOceanHeight(p);
    //d = topDepth;

    if (topDepth <= 0) //entirely below surface
      multiplier = 1;
    else if (topDepth >= 2 * radius) //entirely above surface
      multiplier = 0;
    else
    {
      double m = ((topDepth) / (2 * radius));
      multiplier = cos(m * 1.570796327);
      //      std::cout<<"intermediate position: d=" <<rigidBody->getCenterOfMassPosition().getZ()<<", h="<<getOceanHeight(p)<< ", r="<<radius<<", m="<<(1-m)<<", mm="<<multiplier<<std::endl;
    }
  }

  //std::cout<<std::endl<<this->getType()<<"::"<<this->name<<": topDepth=" << d<<", radius="<<radius<<",multiplier="<<multiplier<<std::endl;

  return 0.001 + multiplier * 0.999;
}

btVector3 SimDev_VehiclePhysicsForce::getApplicationPoint(const btRigidBody * vehicleBody)
{
  btVector3 pos = transform.getOrigin();
  btVector3 absoluteApplicationPoint = rigidBody->getWorldTransform() * pos;
  btVector3 bodyPos = vehicleBody->getWorldTransform().getOrigin();
  btVector3 p = absoluteApplicationPoint - bodyPos;
  //std::cout<< toString(pos)<<","<<toString(absoluteApplicationPoint)<<","<<toString(bodyPos)<<","<<toString(p)<<std::endl;
  return p;
}

PointWrench::type_t parseForce(std::string forceType)
{
  if (forceType.length() > 0)
    switch (forceType.at(0))
    {
      case 'a':
      case 'A':
        return PointWrench::Absolute;
      case 'g':
      case 'G':
        return PointWrench::Global;
      case 'l':
      case 'L':
        return PointWrench::Local;
    }
  return PointWrench::Zero;
}

SimDev_VehiclePhysicsForce::SimDev_VehiclePhysicsForce(SimDev_VehiclePhysicsForce_Config * cfg, osg::Node * node,
                                                       osgOcean::OceanScene * oceanScene, size_t subCfg) :
    SimulatedDevice(cfg)
{
  this->node = node;
  this->oceanScene = oceanScene;
  rigidBody = NULL;

  this->color = cfg->colors[subCfg];
  this->debugForceRatio = cfg->debugForceRatio; //-1=default, 0-disabled
  this->transform = btTransform(cfg->orientations[subCfg], cfg->positions[subCfg]);
  this->forceType = parseForce(cfg->forceType);
  this->radius = cfg->radiuses[subCfg];
  this->maxForce = cfg->maxForces[subCfg];
  this->maxTorque = cfg->maxTorques[subCfg];

  this->defaultForce = cfg->defaultForces[subCfg];
  this->defaultTorque = cfg->defaultTorques[subCfg];
  this->force = this->defaultForce;
  this->torque = this->defaultTorque;

  this->isROS = cfg->isROS != 0;

  this->linkName = cfg->linkName;
  this->multiplier = 0;
  this->impulseLength = -1;
  this->forceVector = btVector3(0,0,0);
  this->torqueVector = btVector3(0,0,0);
  //this->translationalDrag = btVector3(cfg->translationalDrag[0],cfg->translationalDrag[1],cfg->translationalDrag[2]);
  //this->rotationalDrag = btVector3(cfg->rotationalDrag[0],cfg->rotationalDrag[1],cfg->rotationalDrag[2]);
}

void SimDev_VehiclePhysicsForce::calculateVectors(btRigidBody * vehicleBody,
                                                  const osg::MatrixTransform * localizedWorld, BulletPhysics * physics,
                                                  btScalar timeStep, bool isFirstStep)
{

/*
// ADDING SIMULATION OF MARINE CURRENT
//--------------------------------------------------------------
  ITERATE++;
  longueur = 4000;
  start = 100000;
//  std::cout<<ITERATE<<"...";
    if(ITERATE > start && ITERATE < 4221240000)
    {
    if(ITERATE>start && ITERATE < start+2)
    {std::cout<<"Gravity along X axis STARTED...\n \n \n";}
    
    physics->setGravity(btVector3(150, 0, 0));
    if(ITERATE > start + longueur && ITERATE < start + longueur + 2)
    {std::cout<<"Gravity along X axis ENDED...\n \n \n";}

    }
    
    if (ITERATE > start+longueur){
    physics->setGravity(btVector3(0, 0, 0));
    }
//--------------------------------------------------------------    
*/

/*
// ADDING SIMULATION OF Buoyancy
//--------------------------------------------------------------
  ITERATE++;
//  std::cout<<ITERATE<<"...";
    if(ITERATE > 40000)
    {
    if(ITERATE==40001)
    {std::cout<<"STARTED...\n \n \n";}
    
    physics->setGravity(btVector3(0, 0, 0.2));
    
    }
//--------------------------------------------------------------    
*/  

  forceVector.setZero();
  torqueVector.setZero();

  if (impulseLength >= 0)
  {
    impulseLength -= timeStep;
    if (impulseLength < 0)
    {
      force = defaultForce;
      torque = defaultTorque;
      impulseLength = -1;
    }
  }

  if (forceType == PointWrench::Zero)
    return;

  btTransform rotation(btQuaternion(0, 0, 0));  //zero rotation for absolute coordinates
  if (this->forceType == PointWrench::Local)
  {
    rotation = btTransform(this->rigidBody->getWorldTransform().getRotation());
  }
  else if (this->forceType == PointWrench::Global)
  {
    osg::Quat r = localizedWorld->getMatrix().getRotate();
    rotation = btTransform(btQuaternion(r.x(), r.y(), r.z(), r.w()));
  }

  double multiplier = getMultiplier(vehicleBody);

  assert(isfinite(force.length2()));
  assert(isfinite(torque.length2()));

  if (torque.length() > 0)
    ROS_INFO(
        "SimDev_VehiclePhysicsForce::calculateVectors: torque generation is deprecated, use only tranDrag and tranLift!");

  forceVector = (rotation * force);
  //torqueVector = (rotation*torque);
  if (maxForce > 0 && forceVector.length() > maxForce)
  {
    //ROS_INFO("maxForce=%.1f, %.1f", maxForce, forceVector.length());
    forceVector = forceVector.normalize() * maxForce;
  }

  forceVector *= multiplier;

}

std::string toString(const btVector3 & v, int precision)
{
  stringstream ss;
  ss << std::fixed << std::setprecision(precision) << "(" << v.getX() << "," << v.getY() << "," << v.getZ() << ")";
  return ss.str();
}

boost::shared_ptr<SimDev_VehiclePhysicsForce> SimDev_VehiclePhysicsForce_Factory::CreateInstance(
    SimDev_VehiclePhysicsForce_Config * cfg, osg::Node * node, osgOcean::OceanScene * oceanScene, size_t subCfg)
{
  return boost::shared_ptr<SimDev_VehiclePhysicsForce>(new SimDev_VehiclePhysicsForce(cfg, node, oceanScene, subCfg));
}

void SimDev_VehiclePhysicsForceROSPublisher::createPublisher(ros::NodeHandle& nh)
{
  ROS_INFO("%sROSPublisher on topic %s", type.c_str(), topic.c_str());
  pub_ = nh.advertise<uwsim_physics_msgs::ForceCmd>(topic, 1);
}
void SimDev_VehiclePhysicsForceROSPublisher::publish()
{
  uwsim_physics_msgs::ForceCmd forces;
  forces.header.stamp = getROSTime();
  for (size_t i = 0; i < thrusters.size(); ++i)
  {
    btVector3 force = thrusters[i]->getForce();
    geometry_msgs::Vector3 v;
    v.x = force.getX();
    v.y = force.getY();
    v.z = force.getZ();

    btTransform transform = thrusters[i]->getTransform();
    geometry_msgs::Pose p;
    p.position.x = transform.getOrigin().getX();
    p.position.y = transform.getOrigin().getY();
    p.position.z = transform.getOrigin().getZ();
    p.orientation.x = transform.getRotation().getX();
    p.orientation.y = transform.getRotation().getY();
    p.orientation.z = transform.getRotation().getZ();
    p.orientation.w = transform.getRotation().getW();

    forces.name.push_back(thrusters[i]->name);
    forces.radius.push_back(thrusters[i]->getRadius());
    forces.multiplier.push_back(thrusters[i]->getMultiplier());
    forces.link.push_back(thrusters[i]->linkName);
    forces.pose.push_back(p);
    forces.force.push_back(v);
    forces.duration.push_back(thrusters[i]->getImpulseLength());
    forces.forceType.push_back(thrusters[i]->getForceTypeChar());
  }
  pub_.publish(forces);
}

SimDev_VehiclePhysicsForceROSPublisher::SimDev_VehiclePhysicsForceROSPublisher(std::string type, std::string topic,
                                                                               int rate,
                                                                               SimDev_VehiclePhysics * vehiclePhysics) :
    ROSPublisherInterface(topic, rate)
{
  this->vehiclePhysics = vehiclePhysics;
  this->type = type;
  if (vehiclePhysics)
  {
    for (size_t i = 0; i < vehiclePhysics->forceGenerators.size(); ++i)
      if (type == vehiclePhysics->forceGenerators[i]->getType() && vehiclePhysics->forceGenerators[i]->isROS)
        thrusters.push_back(vehiclePhysics->forceGenerators[i]);

    ROS_DEBUG("%sROSPublisher: found %d force generators", type.c_str(), (int )thrusters.size());
  }
}

void SimDev_VehiclePhysicsForceROSSubscriber::processData(const uwsim_physics_msgs::ForceCmd::ConstPtr& cmd)
{
  for (size_t n = 0; n < cmd->name.size(); ++n)
  {
    int thrusterFound = 0;

    for (size_t i = 0; i < thrusters.size(); ++i)
    {
      if (thrusters[i]->name == cmd->name[n])
      {
        thrusterFound = thrusterFound | 1;

        if (cmd->force.size() > n)
        {
          thrusterFound = thrusterFound | 2;
          geometry_msgs::Vector3 v = cmd->force[n];
          double duration = 1;
          if (cmd->duration.size() > n)
            duration = cmd->duration[n];
          thrusters[i]->setForce(btVector3(v.x, v.y, v.z), duration);
          ROS_INFO("Set force %g for duration %g to thruster: %s", v.z, duration, thrusters[i]->name.c_str());
        }

        if (cmd->pose.size() > n)
        {
          thrusterFound = thrusterFound | 4;
          geometry_msgs::Pose p = cmd->pose[n];
          thrusters[i]->setTransform(
              btTransform(btQuaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
                          btVector3(p.position.x, p.position.y, p.position.z)));
        }

        if (cmd->radius.size() > n)
        {
          thrusterFound = thrusterFound | 8;
          thrusters[i]->setRadius(cmd->radius[n]);
        }
      }
    }

    if (thrusterFound == 0)
      ROS_WARN("%sROSSubscriber::processData: Thruster '%s' not found", type.c_str(), cmd->name[n].c_str());
  }
}

void SimDev_VehiclePhysicsForceROSSubscriber::createSubscriber(ros::NodeHandle& nh)
{
  ROS_INFO("%sROSSubscriber on topic %s", type.c_str(), topic.c_str());
  sub_ = nh.subscribe<uwsim_physics_msgs::ForceCmd>(topic, 10, &SimDev_VehiclePhysicsForceROSSubscriber::processData,
                                                    this);
  if (sub_ == ros::Subscriber())
  {
    ROS_ERROR("%sROSSubscriber cannot subscribe to topic %s", type.c_str(), topic.c_str());
  }
}

SimDev_VehiclePhysicsForceROSSubscriber::SimDev_VehiclePhysicsForceROSSubscriber(std::string type, std::string topic,
                                                                                 SimDev_VehiclePhysics * vehiclePhysics) :
    ROSSubscriberInterface(topic)
{
  this->vehiclePhysics = vehiclePhysics;
  this->type = type;
  if (vehiclePhysics)
  {
    for (size_t i = 0; i < vehiclePhysics->forceGenerators.size(); ++i)
      if (type == vehiclePhysics->forceGenerators[i]->getType() && vehiclePhysics->forceGenerators[i]->isROS)
      {
        thrusters.push_back(vehiclePhysics->forceGenerators[i]);
      }

    ROS_DEBUG("%sROSSubscriber: found %d force generators", type.c_str(), (int )thrusters.size());
  }
}
