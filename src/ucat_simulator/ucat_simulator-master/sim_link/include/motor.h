/*
 * motor.h
 *
 *  Created on: Sep 11, 2013
 *      Author: arrows
 */

#ifndef MOTOR_H_
#define MOTOR_H_

using namespace std;

class FlipperMotor
{
public:
	float frequency;
	float zeroDirection;
	float amplitude;
	float position;
	float phaseOffset;
	int positiveDirection;
	string jointName, thrusterName;
	vector<float> thrust; // X Y Z
	FlipperMotor() : frequency(0), zeroDirection(0), amplitude(0), position(0), positiveDirection(1) {};
};


#endif /* MOTOR_H_ */
