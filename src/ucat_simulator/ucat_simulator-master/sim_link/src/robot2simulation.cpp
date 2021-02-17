/*
 * robot2simulation.cpp
 *
 *  Created on: Sep 11, 2013
 *      Author: arrows
 */

#include <ros/ros.h>
#include <uwsim_physics_msgs/ForceCmd.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <tut_arrows_msgs/Flipper.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <motor.h>
#include <cmath>
#include <vector>

using namespace std;

unique_ptr<ros::NodeHandle> node;
unique_ptr<ros::Subscriber> robot_subscriber;
unique_ptr<ros::Publisher> simulator_force_publisher;
unique_ptr<ros::Publisher> simulator_visual_publisher;

#define NUM_OF_MOTORS 4 // Robbe FS 70 MG
#define SIM_UPDATE_RATE 50.0f
#define FORCE_ANGLE -0.5f
#define FORCE_MULTIPLIER 0.03f
#define MAX_ANGULAR_VELOCITY 6.5f // speed without load is 1.28 revolutions per second
#define MAX_ANGULAR_ACCELERATION 500.0f // max torque is 3.65 kg/cm
#define MAX_FORCE 10.0f

vector<string> jointnames = {"baselink_to_frontrightflipper", "baselink_to_backrightflipper",
		"baselink_to_backleftflipper", "baselink_to_frontleftflipper"};
vector<string> thrusternames = {"thruster1", "thruster2", "thruster3", "thruster4"};
vector<int> motordirections = {1, 1, 1, 1};

vector<FlipperMotor> motors(NUM_OF_MOTORS);

void FlipperCallback(const tut_arrows_msgs::FlipperConstPtr msg)
{
	if(msg->motorNumber < NUM_OF_MOTORS || msg->motorNumber >= 0)
	{
		motors[msg->motorNumber].frequency = msg->frequency;
		motors[msg->motorNumber].zeroDirection = msg->zeroDirection;
		motors[msg->motorNumber].amplitude = msg->amplitude;
		motors[msg->motorNumber].phaseOffset = msg->phaseOffset;
	}
	else
		ROS_ERROR("Motor number %d is not between 0 and %d", msg->motorNumber, NUM_OF_MOTORS-1);
}

geometry_msgs::Vector3 calculateForce(float old_position, float new_position, float time_difference)
{
	float angularSpeed = (new_position - old_position) / time_difference;
	float magnitude = angularSpeed*FORCE_MULTIPLIER;
	magnitude = min(magnitude, MAX_FORCE);
	magnitude = max(magnitude, -MAX_FORCE);
	geometry_msgs::Vector3 force;
	force.x = abs(magnitude * cos(FORCE_ANGLE));
	force.y = 0.0;
	force.z = magnitude * sin(FORCE_ANGLE);
	return force;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot2simulation");
	node = unique_ptr<ros::NodeHandle>(new ros::NodeHandle);

	robot_subscriber = unique_ptr<ros::Subscriber> (new ros::Subscriber);
	*robot_subscriber = node->subscribe("/robot2simulation/flipper_cmd", 100, FlipperCallback);

	simulator_force_publisher = unique_ptr<ros::Publisher> (new ros::Publisher);
	*simulator_force_publisher = node->advertise<uwsim_physics_msgs::ForceCmd>("robot2simulation/thrusters_cmd_out", 100);

	simulator_visual_publisher = unique_ptr<ros::Publisher> (new ros::Publisher);
	*simulator_visual_publisher = node->advertise<sensor_msgs::JointState>("robot2simulation/thrusters_visual_out", 100);

	// configure motors
	for(int motornum = 0; motornum < NUM_OF_MOTORS; motornum++)
	{
		motors[motornum].jointName = jointnames[motornum];
		motors[motornum].thrusterName = thrusternames[motornum];
		motors[motornum].positiveDirection = motordirections[motornum];
	}
	ros::Rate r(SIM_UPDATE_RATE);

//	motors[1].frequency = 1.0;
//	motors[1].amplitude = 2.0;
//	motors[1].zeroDirection = -1.0;
//	motors[2].frequency = 1.0;
//	motors[2].amplitude = 2.0;
//	motors[2].zeroDirection = -1.0;
//	motors[0].frequency = 1.0;
//	motors[0].amplitude = 2.0;
//	motors[0].zeroDirection = 1.0;
//	motors[3].frequency = 1.0;
//	motors[3].amplitude = 2.0;
//	motors[3].zeroDirection = 1.0;
//	motors[0].phaseOffset = M_PI;
//	motors[1].phaseOffset = M_PI;

//	ros::Duration(7).sleep();
	vector<float> oldvelocity(NUM_OF_MOTORS, 0);

	while (ros::ok())
	{
		uwsim_physics_msgs::ForceCmd force_cmd;
		sensor_msgs::JointState flippervisual;
		flippervisual.header.stamp = ros::Time::now();
		ros::Time currentTime = ros::Time::now();
		int motornum = 0;
		for(auto& motor : motors)
		{
			float oldposition = motor.position;

			float phase = (fmod(currentTime.toSec(),1.0/motor.frequency))*motor.frequency*2*M_PI + motor.phaseOffset;
			float desired_position = motor.zeroDirection + motor.positiveDirection*sin(phase)*motor.amplitude/2;
			float shortest_path = fmod(float(desired_position - oldposition), float(2*M_PI));
			// shortest path is always in the -3.14...3.14 range
			if(shortest_path > M_PI)
				shortest_path -= 2*M_PI;
			else if(shortest_path < -M_PI)
				shortest_path += 2*M_PI;
			float angular_velocity = shortest_path * SIM_UPDATE_RATE;
			angular_velocity = min(angular_velocity, MAX_ANGULAR_VELOCITY);
			angular_velocity = max(angular_velocity, -MAX_ANGULAR_VELOCITY);
			float angular_acceleration = (angular_velocity - oldvelocity[motornum])*SIM_UPDATE_RATE;
			angular_acceleration = min(angular_acceleration, MAX_ANGULAR_ACCELERATION);
			angular_acceleration = max(angular_acceleration, -MAX_ANGULAR_ACCELERATION);
			angular_velocity = angular_acceleration/SIM_UPDATE_RATE + oldvelocity[motornum];
			motor.position = oldposition + angular_velocity/SIM_UPDATE_RATE;
			while(motor.position > M_PI)
				motor.position -= 2*M_PI;
			while(motor.position < -M_PI)
				motor.position += 2*M_PI;

			flippervisual.name.push_back(motor.jointName);
			flippervisual.position.push_back(motor.position);

			geometry_msgs::Vector3 resultingForce = calculateForce(oldposition, motor.position, 1.0/SIM_UPDATE_RATE);

			force_cmd.name.push_back(motor.thrusterName);
			force_cmd.force.push_back(resultingForce);
			force_cmd.duration.push_back(10.0/SIM_UPDATE_RATE); // 10 times overlap

			oldvelocity[motornum] = angular_velocity;
			motornum++;
		}
		simulator_visual_publisher->publish(flippervisual);
		simulator_force_publisher->publish(force_cmd);
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
