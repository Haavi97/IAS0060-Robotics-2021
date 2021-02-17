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

#include "uwsim_arrows/SimDev_VehicleFin.h"
#include <iostream>

double SimDev_VehicleFin::getJointPosition()
{
  if (jointZeroDir.fuzzyZero() || force.fuzzyZero())
    return 0;
  else
  {
    double a = force.angle(jointZeroDir);
    return jointZeroDir.cross(force).getX() < 0 ? a : 0 - a;
  }
}

btVector3 SimDev_VehicleFin::calculateTransition(btVector3 from, btVector3 to, double subPhase)
{
  if (from == to)
    return from;
  else if (from.isZero())
  {
    subPhase = 1 - subPhase;
    std::swap(from, to);
  }

  btVector3 ret;
  if (from.cross(to).isZero())
  {
    ret = (from - to) * subPhase + to;
  }
  else
  {
    ret = from.rotate(from.cross(to).safeNormalize(), subPhase * from.angle(to));
  }

  if (false)
  {
    std::cerr
        << // "Thruster '"<<this->name<<"': "
        "Transition from " << toString(from) << " to " << toString(to) << " at " << subPhase << " = " << toString(ret)
        << ", len=" << (ret.length() / from.length()) << ", angle=" << from.angle(ret) << "\n"; //<<std::endl;
  }
  return ret;
}

SimDev_VehiclePhysicsForce_Factory::Ptr SimDev_VehicleFin_Factory::processConfig(
    const xmlpp::Node* node, ConfigFile* config, SimDev_VehiclePhysicsForce_Factory::Ptr cfg_)
{
  SimDev_VehicleFin_Config * cfg = new SimDev_VehicleFin_Config(getType());
  cfg_.reset(cfg);
  cfg_ = SimDev_VehiclePhysicsForce_Factory::processConfig(node, config, cfg_);

  cfg->forceType = "local";

  double tmp;
  xmlpp::Node::NodeList list = node->get_children();
  for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
  {
    const xmlpp::Node* child = dynamic_cast<const xmlpp::Node*>(*iter);
    //TODO: parse config values
    // 		if(child->get_name()=="mass"){
    // 			config->extractFloatChar(child,tmp);
    // 			cfg->masses.push_back(tmp);
    // 	}
  }

  return cfg_;
}

SimDev_VehicleFin::SimDev_VehicleFin(SimDev_VehiclePhysicsForce_Config* cfg, osg::Node* node,
                                     osgOcean::OceanScene* oceanScene, size_t subCfg) :
    SimDev_VehiclePhysicsForce(cfg, node, oceanScene, subCfg)
{
  //TODO: take values from config
  jointIndex = -1;
  jointZeroDir = btVector3(0, 0, 0);

  //finCommands.front();
  //
}

void SimDev_VehicleFin::setCurrentCommand(FinCmd::Ptr cmd)
{
  if (!cmd && currentCommand)
    OSG_DEBUG<< "VehicleFin '"<<this->name<<"': resetting to defaults..." <<std::endl;

    currentCommand = cmd;
  }

void SimDev_VehicleFin::updateCurrentCommand(btScalar timeStep)
{
  if (currentCommand)
    currentCommand->curTime += timeStep;

  if (!currentCommand || (currentCommand->endTime >= 0 && currentCommand->curTime > currentCommand->endTime))
  {
    FinCmd::Ptr cmd;
    while (!finCommands.empty() && !cmd)
    {
      cmd = finCommands.front();
      finCommands.pop_front();
      if (cmd && cmd->forces.size() < 1)
      { //checking FinCmd for correctness
        OSG_ALWAYS<< "VehicleFin '"<<this->name<<"': skipping incorrect FinCmd..." <<std::endl;
        cmd.reset();
      }
    }
    setCurrentCommand(cmd);
  }

}

void SimDev_VehicleFin::calculateVectors(btRigidBody* vehicleBody, const osg::MatrixTransform* localizedWorld,
                                         BulletPhysics* physics, btScalar timeStep, bool isFirstStep)
{
  force = defaultForce;
  torque = defaultTorque;

  updateCurrentCommand(timeStep);
  if (currentCommand)
  {
    if (currentCommand->period <= 0 || currentCommand->forces.size() == 1)
    {
      force = currentCommand->forces.at(0);
    }
    else
    { //period>0 && wrenches.size()>1
      double gPhase = currentCommand->curTime / currentCommand->period;
      double phase0to1 = gPhase - floor(gPhase);

      assert(phase0to1 >= 0 && phase0to1 < 1);
      double phase = phase0to1 * (currentCommand->forces.size() - 1);
      size_t base = (size_t)floor(phase);
      double subPhase = phase - base;
      force = calculateTransition(currentCommand->forces.at(base), currentCommand->forces.at(base + 1), subPhase);
    }
  }

  SimDev_VehiclePhysicsForce::calculateVectors(vehicleBody, localizedWorld, physics, timeStep, isFirstStep);
}
