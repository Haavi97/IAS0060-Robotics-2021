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

/* Change the force applied with a specific thruster.
 */

#include <ros/ros.h>
#include <underwater_sensor_msgs/ForceCmd.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <iostream>

int main(int argc, char **argv)
{
  if (argc < 6 || argc % 3 != 0)
  {
    std::cerr << "USAGE: " << argv[0] << " <vehicleForceCmdTopic> <thrusterName> <x0> <y0> <z0>  [<x0> <y0> <z0> ...]"
        << std::endl;
    std::cerr << "units are newtons." << std::endl;
    std::cerr << "If only x0,y0,z0 are given, command will be sent to 'thrusterName'," << std::endl;
    std::cerr << "if more sets of forces are given, command will be sent to 'thrusterName_0', 'thrusterName_1', etc."
        << std::endl;
    return 0;
  }

  std::string controlTopic(argv[1]);
  std::string thrusterName(argv[2]);

  std::vector<geometry_msgs::Vector3> forces;
  for (int i = 5; i < argc; i += 3)
  {
    double x = atof(argv[i - 2]);
    double y = atof(argv[i - 1]);
    double z = atof(argv[i]);
    geometry_msgs::Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    forces.push_back(v);
  }

  std::string nodeName("setThrusterForce_" + thrusterName);
  ros::init(argc, argv, nodeName);
  ros::NodeHandle nh;

  ros::Publisher force_pub = nh.advertise<underwater_sensor_msgs::ForceCmd>(controlTopic, 1);

  ros::Rate r(2);
  while (ros::ok())
  {
    underwater_sensor_msgs::ForceCmd force;
    for (size_t i = 0; i < forces.size(); ++i)
    {
      std::stringstream ss;
      ss << thrusterName;
      if (forces.size() > 1)
        ss << "_" << i;
      force.name.push_back(ss.str());
      force.force.push_back(forces.at(i));
      force.duration.push_back(1);

      /*
       *     //It is also possible to set thruster pose (in local link coordinates) and thruster radius
       *     //Radius marks the size of a thruster sphere. Resulting thrust is proportional to the volume inside water.
       *     //Negative radius means that thruster is always active (both in the water and in the air).
       *     //Pose setting here is not recommended, use it on your own risk...
       *     //Dynamic thruster position is easier to implement by attaching it to a movable link.
       *     geometry_msgs::Pose p;
       *     p.orientation.x = ?;
       *     p.orientation.y = ?;
       *     p.orientation.z = ?;
       *     p.orientation.w = ?;
       *     p.position.x = ?;
       *     p.position.y = ?;
       *     p.position.z = ?;
       *     force.pose.push_back(p);
       *     
       *     double r = 0.05;
       *     force.radius.push_back(r); 
       */
    }

    force_pub.publish(force);
    ros::spinOnce();
    r.sleep();
  }

  //ros::shutdown();

  return 0;
}
