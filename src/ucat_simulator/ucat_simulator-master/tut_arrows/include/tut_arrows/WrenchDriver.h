#ifndef WRENCH_DRIVER_H_
#define WRENCH_DRIVER_H_

#include <ros/ros.h>

#include <auv_msgs/AcousticModemData.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Range.h>

#include "tut_arrows_msgs/FlippersModeCmd.h"
#include "tut_arrows_msgs/Flippers.h"

class WrenchDriver
{
public:
	WrenchDriver();

	void setMode(const tut_arrows_msgs::FlippersModeCmd& modeCmd);

	void setFrequency(float frequency);

	void applyWrench(const geometry_msgs::Wrench& wrench);

	void controlByAmplitude(double fx, double fy, double fz, double fRoll, double fPitch, double fYaw);

	void moveFastYaw(double fx, double fYaw);
	void moveFast(double fx, double fz, double fRoll, double fPitch, double fYaw);

	void moveSlowX(double fx);
	void moveSlowY(double fy);
	void moveSlowYaw(double fYaw);
	float moveSlowVertical(double fz, double fRoll, double fPitch);
	void moveSlow(double fx, double fy, double fz, double fRoll, double fPitch, double fYaw);

private:
	void ProcessWrenchStamped(const geometry_msgs::WrenchStampedConstPtr data);
	void ProcessModeCmd(const tut_arrows_msgs::FlippersModeCmdConstPtr data);

	void acousticModemCallback(const auv_msgs::AcousticModemData::ConstPtr& data);

	float forceToAmplitude(float force) const;
	float getMaxForce() const;

	void limitAmplitudes();

	void convertForcesToAmplitudes();

private:
	ros::NodeHandle nh_;

	ros::Subscriber sub_force_req, sub_mode, sub_obs_req;
	ros::Subscriber acousticModemSub_;
	ros::Publisher pub_flippers;

	tut_arrows_msgs::FlippersPtr flippersMsg_;
	tut_arrows_msgs::FlippersModeCmd modeCmd_;

	ros::Time lastModeChangeTime_;
	double modeChangeTime_;

	bool useFastV2_;
};

#endif /* WRENCH_DRIVER_H_ */
