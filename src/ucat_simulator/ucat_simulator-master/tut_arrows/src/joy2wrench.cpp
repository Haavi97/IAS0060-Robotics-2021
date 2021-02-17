#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tut_arrows_msgs/FlippersModeCmd.h>
#include <tut_arrows_msgs/Flippers.h>
#include "tut_arrows/Ps3Config.h"

class TeleopWrench
{
public:
  TeleopWrench();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  tut_arrows_msgs::Flippers createFlippersMsg(double zeroDirRF, double zeroDirRB, double zeroDirLB, double zeroDirLF, double frequency, double amplitude) const;
  bool pressedButton(int button, const sensor_msgs::Joy::ConstPtr& joy) const;
  bool releasedButton(int button, const sensor_msgs::Joy::ConstPtr& joy) const;
  static void addToFlippersMsg(tut_arrows_msgs::Flippers& flippers, double zeroDirRF, double zeroDirRB, double zeroDirLB, double zeroDirLF);

  ros::NodeHandle nh_;
  ros::NodeHandle nhPrivate_;

  int axis_x,axis_y, axis_z, axis_yaw, button_fast, button_slow;
  double scale_x, scale_y, scale_z, scale_yaw;
  ros::Publisher wrench_pub_, mode_pub_, flippers_cmd_pub_;
  ros::Subscriber joy_sub_;
  bool isFastMode;
  bool useMsgTimeStamp_;

  sensor_msgs::Joy::Ptr lastJoy_;
};

TeleopWrench::TeleopWrench():
	nh_(), nhPrivate_("~"),
    axis_x(1), axis_y(0),axis_z(3),axis_yaw(2),
    scale_x(4), scale_y(8), scale_z(4), scale_yaw(0.4),
    button_fast(12),button_slow(14),isFastMode(false),
    useMsgTimeStamp_(true)
{

  nhPrivate_.param("axis_x", axis_x, axis_x);
  nhPrivate_.param("axis_y", axis_y, axis_y);
  nhPrivate_.param("axis_z", axis_z, axis_z);
  nhPrivate_.param("axis_yaw", axis_yaw, axis_yaw);

  nhPrivate_.param("scale_x", scale_x, scale_x);
  nhPrivate_.param("scale_y", scale_y, scale_y);
  nhPrivate_.param("scale_z", scale_z, scale_z);
  nhPrivate_.param("scale_yaw", scale_yaw, scale_yaw);

  nhPrivate_.param("button_fast", button_fast, button_fast);
  nhPrivate_.param("button_slow", button_slow, button_slow);

  nhPrivate_.param("use_msgs_timestamp", useMsgTimeStamp_, useMsgTimeStamp_);

  wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("force_req", 1);
  mode_pub_ = nh_.advertise<tut_arrows_msgs::FlippersModeCmd>("force_mode", 1);
  flippers_cmd_pub_ = nh_.advertise<tut_arrows_msgs::Flippers>("hw/flippers_cmd", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopWrench::joyCallback, this);
}

//void TeleopWrench::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
//{
//  if (joy->buttons.at(button_fast) > 0 || joy->buttons.at(button_slow) > 0)
//    {
//      tut_arrows_msgs::FlippersModeCmd mode;
//      mode.header.stamp = ros::Time::now();
//      if (joy->buttons.at(button_fast) > 0)
//      {
//        mode.mode = tut_arrows_msgs::FlippersModeCmd::MODE_FAST;
//        isFastMode = true;
//      }
//      if (joy->buttons.at(button_slow) > 0)
//      {
//        mode.mode = tut_arrows_msgs::FlippersModeCmd::MODE_SLOW;
//        isFastMode = false;
//      }
//      mode_pub_.publish(mode);
//    }
//
//  double mult = isFastMode?4:2;
//  geometry_msgs::WrenchStamped wrench;
//  // Using Time::now() instead of joy->header.stamp since default ros joystick driver
//  // doesn't update timestamp when using autorepeat. Perhaps this is not a problem when
//  // using xboxdrv or even causes problem then, if so just add it back.
//  wrench.header.stamp = useMsgTimeStamp_ ? joy->header.stamp : ros::Time::now();
//  wrench.wrench.force.x = joy->axes.at(axis_x)*scale_x*mult;
//  wrench.wrench.force.y = joy->axes.at(axis_y)*scale_y*mult;
//  wrench.wrench.force.z = joy->axes.at(axis_z)*scale_z*mult;
//  wrench.wrench.torque.z = joy->axes.at(axis_yaw)*scale_yaw*mult;
//  wrench_pub_.publish(wrench);
//}

tut_arrows_msgs::Flippers TeleopWrench::createFlippersMsg(double zeroDirRF, double zeroDirRB, double zeroDirLB, double zeroDirLF, double frequency, double amplitude) const
{
	tut_arrows_msgs::Flippers flippersCmd;

	{
		tut_arrows_msgs::Flipper flipper_RF;
		flipper_RF.motorNumber = 0;
		flipper_RF.phaseOffset = 0;
		flipper_RF.zeroDirection = zeroDirRF;

		flippersCmd.flippers.push_back(flipper_RF);
	}

	{
		tut_arrows_msgs::Flipper flipper_RB;
		flipper_RB.motorNumber = 1;
		flipper_RB.phaseOffset = 0;
		flipper_RB.zeroDirection = zeroDirRB;

		flippersCmd.flippers.push_back(flipper_RB);
	}

	{
		tut_arrows_msgs::Flipper flipper_LB;
		flipper_LB.motorNumber = 2;
		flipper_LB.phaseOffset = -3.14;
		flipper_LB.zeroDirection = zeroDirLB;

		flippersCmd.flippers.push_back(flipper_LB);
	}

	{
		tut_arrows_msgs::Flipper flipper_LF;
		flipper_LF.motorNumber = 3;
		flipper_LF.phaseOffset = -3.14;
		flipper_LF.zeroDirection = zeroDirLF;

		flippersCmd.flippers.push_back(flipper_LF);
	}

	for (auto& flipper : flippersCmd.flippers)
	{
		flipper.frequency = frequency;
		flipper.amplitude = amplitude;
	}

	return flippersCmd;
}

void TeleopWrench::addToFlippersMsg(tut_arrows_msgs::Flippers& flippers, double zeroDirRF, double zeroDirRB, double zeroDirLB, double zeroDirLF)
{
	if (flippers.flippers.size() != 4)
	{
		return;
	}

	flippers.flippers[0].zeroDirection += zeroDirRF;
	flippers.flippers[1].zeroDirection += zeroDirRB;
	flippers.flippers[2].zeroDirection += zeroDirLB;
	flippers.flippers[3].zeroDirection += zeroDirLF;
}

bool TeleopWrench::pressedButton(int button, const sensor_msgs::Joy::ConstPtr& joy) const
{
	if (lastJoy_)
	{
		return !lastJoy_->buttons.at(button) && joy->buttons.at(button);
	}
	return false;
}

bool TeleopWrench::releasedButton(int button, const sensor_msgs::Joy::ConstPtr& joy) const
{
	if (lastJoy_)
	{
		return lastJoy_->buttons.at(button) && !joy->buttons.at(button);
	}
	return false;
}

void TeleopWrench::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	if (releasedButton(PS3_BUTTON_CROSS_UP, joy) ||
		releasedButton(PS3_BUTTON_CROSS_DOWN, joy) ||
		releasedButton(PS3_BUTTON_REAR_RIGHT_2, joy) ||
		releasedButton(PS3_BUTTON_REAR_LEFT_2, joy) ||
		releasedButton(PS3_BUTTON_CROSS_LEFT, joy) ||
		releasedButton(PS3_BUTTON_CROSS_RIGHT, joy))
	{
		const tut_arrows_msgs::Flippers flippers = createFlippersMsg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		flippers_cmd_pub_.publish(flippers);
		lastJoy_ = sensor_msgs::Joy::Ptr(new sensor_msgs::Joy(*joy));
		return;
	}

	tut_arrows_msgs::Flippers flippersMsg;

	if (pressedButton(PS3_BUTTON_CROSS_UP, joy)) // move forward
	{
		flippersMsg = createFlippersMsg(0.0, 0.0, 0.0, 0.0, 2.5, 1.12);

		if (joy->buttons.at(PS3_BUTTON_ACTION_TRIANGLE)) // move forward-up
		{
			addToFlippersMsg(flippersMsg, 0.785, 0.785, -0.785, -0.785);
		}
		else if (joy->buttons.at(PS3_BUTTON_ACTION_CROSS)) // move forward-down
		{
			addToFlippersMsg(flippersMsg, -0.785, -0.785, 0.785, 0.785);
		}
	}

	else if (pressedButton(PS3_BUTTON_CROSS_DOWN, joy)) // move back
	{
		flippersMsg = createFlippersMsg(3.14, 3.14, 3.14, 3.14, 2.5, 1.12);

		if (joy->buttons.at(PS3_BUTTON_ACTION_TRIANGLE)) // move backward-up
		{
			addToFlippersMsg(flippersMsg, -0.785, -0.785, 0.785, 0.785);
		}
		else if (joy->buttons.at(PS3_BUTTON_ACTION_CROSS)) // move backward-down
		{
			addToFlippersMsg(flippersMsg, 0.785, 0.785, -0.785, -0.785);
		}
	}

	else if (pressedButton(PS3_BUTTON_REAR_RIGHT_2, joy)) // turn right
	{
		flippersMsg = createFlippersMsg(3.14, 3.14, 0.0, 0.0, 2.5, 1.12);
	}

	else if (pressedButton(PS3_BUTTON_REAR_LEFT_2, joy)) // turn left
	{
		flippersMsg = createFlippersMsg(0.0, 0.0, 3.14, 3.14, 2.5, 1.12);
	}

	else if (pressedButton(PS3_BUTTON_CROSS_LEFT, joy)) // move left
	{
		flippersMsg = createFlippersMsg(0.0, -3.14, 0.0, 3.14, 2.5, 1.12);

		if (joy->buttons.at(PS3_BUTTON_ACTION_TRIANGLE)) // move up
		{
			addToFlippersMsg(flippersMsg, 0.785, -0.785, -0.785, 0.785);
		}
		else if (joy->buttons.at(PS3_BUTTON_ACTION_CROSS)) // move down
		{
			addToFlippersMsg(flippersMsg, -0.785, 0.785, 0.785, -0.785);
		}
	}

	else if (pressedButton(PS3_BUTTON_CROSS_RIGHT, joy)) // move left
	{
		flippersMsg = createFlippersMsg(-3.14, 0.0, 3.14, 0.0, 2.5, 1.12);

		if (joy->buttons.at(PS3_BUTTON_ACTION_TRIANGLE)) // move up
		{
			addToFlippersMsg(flippersMsg, -0.785, 0.785, 0.785, -0.785);
		}
		else if (joy->buttons.at(PS3_BUTTON_ACTION_CROSS)) // move down
		{
			addToFlippersMsg(flippersMsg, 0.785, -0.785, -0.785, 0.785);
		}
	}

	// -------------------------------------------------------------------------------------------------------

	if (flippersMsg.flippers.size() == 4)
	{
		flippers_cmd_pub_.publish(flippersMsg);
	}

	lastJoy_ = sensor_msgs::Joy::Ptr(new sensor_msgs::Joy(*joy));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy2wrench");
  TeleopWrench teleop_wrench;

  ros::spin();
}
