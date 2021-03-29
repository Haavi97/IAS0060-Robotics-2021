/*
 * Copyright (c) 2013 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 *
 * Contributors:
 *     Mario Prats
 *     Javier Perez
 *     Walid REMMAS
 */


/* This node spawns a Mesh marker in the uwsim scene through a marker service*/

#include <ros/ros.h>
#include <underwater_sensor_msgs/SpawnMarker.h>
#include <tf/transform_datatypes.h>
#include <math.h>



int main( int argc, char** argv )
{
  ros::init(argc, argv, "spawnObject");
  ros::NodeHandle n;

  
  ros::ServiceClient client = n.serviceClient<underwater_sensor_msgs::SpawnMarker>("SpawnMarker");
  underwater_sensor_msgs::SpawnMarker srv;

  // Initialize a "Point" message publisher, to publish the coordinates of the moving target 


  srv.request.marker.header.frame_id = "/localizedWorld";
  srv.request.marker.header.stamp = ros::Time::now();

  srv.request.marker.ns = "Object";
  srv.request.marker.id = 1;

  srv.request.marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  srv.request.marker.mesh_resource="package://uwsim/pingeryellow.ive";

  srv.request.marker.action = visualization_msgs::Marker::ADD;
  ros::Rate loop_rate(10);


  float i = 0; // increment (meters per iteration)
  float x,y,z = 0; // position of the target

  x = 8;  
  y = 8;
  z = -10;

  while (ros::ok())
  {	  
	  // Create a trajectory
	  //--------------------------------
	  // x = some_equation;
	  // y = some_equation;
	  // z = some_equation;


	  srv.request.marker.pose.position.x = x;
	  srv.request.marker.pose.position.y = y;
	  srv.request.marker.pose.position.z = z;

	  tf::Quaternion quat;
	  quat.setRPY( 0,0,0);
	  quat.normalize();

	  srv.request.marker.pose.orientation.x = quat.getX();
	  srv.request.marker.pose.orientation.y = quat.getY();
	  srv.request.marker.pose.orientation.z = quat.getZ();
	  srv.request.marker.pose.orientation.w = quat.getW();


	  srv.request.marker.scale.x = 1.0;
	  srv.request.marker.scale.y = 1.0;
	  srv.request.marker.scale.z = 1.0;

	  srv.request.marker.color.r = 1.0f;
	  srv.request.marker.color.g = 1.0f;
	  srv.request.marker.color.b = 1.0f;
	  srv.request.marker.color.a = 1.0;

	  if (client.call(srv))
	  {
	   // Publish the coordinates of the target
	  }
	  
	  
	  ros::spinOnce();

	  loop_rate.sleep();
  }
  return 0;
}
