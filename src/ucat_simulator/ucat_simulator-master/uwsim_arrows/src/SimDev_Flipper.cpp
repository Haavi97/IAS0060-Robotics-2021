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

#include "uwsim_arrows/SimDev_Flipper.h"
#include "uwsim_physics/SimDev_VehiclePhysics.h"
#include <iostream>
#include "uwsim/ConfigXMLParser.h"
#include <cmath>

SimDev_VehiclePhysicsForce_Config::Ptr SimDev_Flipper_Factory::processConfig(
    const xmlpp::Node* node, ConfigFile* config, SimDev_VehiclePhysicsForce_Config::Ptr cfg_)
{
  if (!cfg_)
    cfg_.reset(new SimDev_Flipper_Config(this->getType()));

  SimDev_Flipper_Config * cfg = dynamic_cast<SimDev_Flipper_Config *> (cfg_.get());
  if (cfg==NULL)
    ROS_ERROR("SimDev_Flipper_Config::processConfig got wrong Config");

  cfg_ = SimDev_VehiclePhysicsForce_Factory::processConfig(node, config, cfg_);

  cfg->forceType = "local";

  std::string tmpStr;
  xmlpp::Node::NodeList list = node->get_children();
  for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
  {
    const xmlpp::Node* child = dynamic_cast<const xmlpp::Node*>(*iter);

    if (child->get_name() == "jointName")
    {
      config->extractStringChar(child, tmpStr);
      cfg->jointNames.push_back(tmpStr);
    }
    else if (child->get_name() == "side")
    {
      config->extractStringChar(child, cfg->side);
    }
    else if (child->get_name() == "forceAngle")
    {
      config->extractFloatChar(child, cfg->forceAngle);
    }
    else if (child->get_name() == "forceMultiplier")
    {
      config->extractFloatChar(child, cfg->forceMultiplier);
    }
    else if (child->get_name() == "maxForce")
    {
      config->extractFloatChar(child, cfg->maxForce);
    }
    else if (child->get_name() == "multPower")
    {
      config->extractFloatChar(child, cfg->multPower);
    }
  }

  return cfg_;
}

SimDev_Flipper::SimDev_Flipper(SimDev_VehiclePhysicsForce_Config* cfg, osg::Node* node,
                                       osgOcean::OceanScene* oceanScene, size_t subCfg) :
    SimDev_VehiclePhysicsForce(cfg, node, oceanScene, subCfg)
{
  SimDev_Flipper_Config* cfg_ = dynamic_cast<SimDev_Flipper_Config*>(cfg);
  zeroPhaseMoment = 0;
  zeroDirection = 0;
  auv = NULL;
  if (cfg_ != NULL)
  {
    this->jointNames = cfg_->jointNames;
    this->forceAngle = cfg_->forceAngle;
    this->forceMultiplier = cfg_->forceMultiplier;
    this->maxForce = cfg_->maxForce;
    this->multPower = cfg_->multPower;
    this->auv = cfg_->auv;

    if (this->auv)
    {
      int idx = -1;
      for (int i = 0; i < auv->urdf->getNumberOfJoints(); i++)
      {
        if (not (auv->urdf->jointType[i] == 0 || auv->urdf->mimic[i].joint != i))
        { //Check it is not fixed nor mimic
          idx++;
          int found = 0;
          for (int j = 0; j < auv->urdf->names.size() && !found; j++)
          {
            if (auv->urdf->names[i] == this->jointNames.at(0))
            {
              found = 1;
              jointIndex = idx;
            }
          }
        }
      }
    }

    //jointIndex
  }
}

btVector3 SimDev_Flipper::calculateForce(float angularSpeed)
{
  //float angularSpeed = (new_position - old_position) / time_difference;
  double sign = angularSpeed < 0 ? -1 : 1;
  double magnitude = sign * pow(fabs(angularSpeed), multPower) * forceMultiplier;
  magnitude = min(magnitude, maxForce);
  magnitude = max(magnitude, -maxForce);
  btVector3 force(abs(magnitude * cos(forceAngle)), 0.0, magnitude * sin(forceAngle));
  //ROS_INFO("calculateForce: as=%.4f, m=%.4f, f=%.4f", angularSpeed, magnitude, force.length());
  return force;
}

double normalizeAngle(double a)
{
  while (a>M_PI) a-=2*M_PI;
  while (a<-M_PI) a+=2*M_PI;
  return a;
}

void SimDev_Flipper::calculateVectors(btRigidBody * vehicleBody, const osg::MatrixTransform * localizedWorld,
                              BulletPhysics * physics, btScalar timeStep, bool isFirstStep)
{
  if (flipperCmd && flipperCmd->frequency > 0)
  {
    double period = 1.0/flipperCmd->frequency;
    double phase01 = fmod(ros::Time::now().toSec()-zeroPhaseMoment, period)/period;
    double phase = phase01*2*M_PI + flipperCmd->phaseOffset;

    double toZero = normalizeAngle(flipperCmd->zeroDirection - zeroDirection);
    if (fabs(toZero)>0.01)
      toZero = (toZero<0?-1:1)*0.01;
    zeroDirection += toZero;

    double idealPos = normalizeAngle(zeroDirection+sin(phase)*flipperCmd->amplitude*0.5);
    generateFinForceAndPosition(idealPos, timeStep);
  }
  else if (flipperCmd)
    force.setZero();

  SimDev_VehiclePhysicsForce::calculateVectors(vehicleBody, localizedWorld, physics, timeStep, isFirstStep);
  //if (flipperCmd)// && flipperCmd->motorNumber==0)
  //  ROS_INFO("Force[%u] = %.2fm, dir=%.2f", flipperCmd->motorNumber, force.length(), zeroDirection);
}

void SimDev_Flipper::generateFinForceAndPosition(double idealPos, double timeStep)
{
  double maxSpeed = 2 * 2*M_PI; // rad/sec
  std::vector<double> jointPositions = auv->urdf->getJointPosition();
  if (jointIndex < jointPositions.size())
  {
    double curPos = normalizeAngle(jointPositions.at(jointIndex));
    double delta = normalizeAngle(idealPos-curPos)/timeStep; // rad/sec
    if (fabs(delta) > maxSpeed)
    {
      //if (flipperCmd->motorNumber==0)
      //  ROS_INFO("motor[%u] speed = %.2f rad/sec at %.4f, ideal=%.4f j=%s", flipperCmd->motorNumber, delta, curPos, idealPos, jointNames.at(0).c_str());
        //ROS_INFO("limit[%u] delta=%.4f, 0..1=%.4f, ideal = %.4f, cur=%.4f", flipperCmd->motorNumber, delta, phase01, idealPos, curPos);
      delta = (delta<0?-1:1)*maxSpeed;
    }

    setForce(calculateForce(delta), 0.1);

    double nextPos =normalizeAngle(curPos + delta*timeStep);
    std::vector<double> p;
    p.push_back(nextPos);
    std::vector<std::string> n;
    n.push_back(jointNames.at(0));
    vehiclePhysics->setJointPosition(p,n);
  }
  else
     force.setZero();
}




std::vector<boost::shared_ptr<ROSInterface> > SimDev_Flipper_Factory::getInterface(
    ROSInterfaceInfo& rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> >& iauvFile)
{
  std::vector<boost::shared_ptr<ROSInterface> > ifaces;

  if (rosInterface.topic.length() > 0)
  {
    for (size_t i = 0; i < iauvFile.size(); ++i)
      if (iauvFile[i]->name == rosInterface.targetName && iauvFile[i]->devices)
      {
        ifaces.push_back(
            boost::shared_ptr<ROSInterface>(
                new SimDev_FlipperROSSubscriber(rosInterface.topic, iauvFile[i].get())));
      }
  }

  return ifaces;
}

void SimDev_Flipper::setFlipperCmd(tut_arrows_msgs::Flipper flipper)
{
  flipperCmd.reset(new tut_arrows_msgs::Flipper(flipper));
  zeroPhaseMoment = 0;
}

void SimDev_FlipperROSSubscriber::createSubscriber(ros::NodeHandle& nh)
{
  flippers_sub_ = nh.subscribe<tut_arrows_msgs::Flippers>(
      this->topic, 100, &SimDev_FlipperROSSubscriber::processData, this);
}

void SimDev_FlipperROSSubscriber::processData(const tut_arrows_msgs::FlippersConstPtr& cmd)
{
  for (size_t f = 0; f < cmd->flippers.size(); ++f)
  {
    tut_arrows_msgs::Flipper flipper = cmd->flippers.at(f);
    if (flipper.motorNumber<this->flippers.size())
      this->flippers.at(flipper.motorNumber)->setFlipperCmd(flipper);
  }
}

SimDev_FlipperROSSubscriber::SimDev_FlipperROSSubscriber(string topic, SimulatedIAUV * auv) :
    ROSSubscriberInterface(topic)
{
  boost::shared_ptr<SimulatedDevices> simdevs = auv->devices;
  vehiclePhysics = SimDev_VehiclePhysics::getVehiclePhysics(simdevs);
  if (!vehiclePhysics)
    ROS_ERROR("SimDev_FlipperROSSubscriber requires VehiclePhysics");

  for (size_t i = 0; i < simdevs->all.size(); ++i)
    {
      boost::shared_ptr<SimDev_Flipper> fin = boost::dynamic_pointer_cast<SimDev_Flipper, SimulatedDevice>(
          simdevs->all.at(i));
      if (fin)
      {
        fin->vehiclePhysics = vehiclePhysics;
        flippers.push_back(fin);
      }
    }
}
