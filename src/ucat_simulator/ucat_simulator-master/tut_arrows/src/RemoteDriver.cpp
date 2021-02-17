#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tut_arrows_msgs/FlippersModeCmd.h>
#include <std_msgs/Float32.h>

class RemoteDriver
{
public:
	RemoteDriver();
	void run();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

private:
	ros::NodeHandle nh_;

	double rate_;

	ros::Publisher wrench_pub_, mode_pub_, depth_pub_;
	ros::Subscriber joy_sub_;

	geometry_msgs::WrenchStamped wrench_;

	int axis_x,axis_y, axis_z, axis_roll, axis_pitch, axis_yaw;
	int button_fast, button_slow, button_depth_up, button_depth_down;

	double depth_;
};

RemoteDriver::RemoteDriver():
	nh_(), rate_(5.0),
    axis_x(1), axis_y(0),axis_z(3),axis_yaw(2),
    button_fast(12), button_slow(14), button_depth_up(5), button_depth_down(7),
	depth_(0.0)
{
	ros::NodeHandle nhPrivate("~");

	nhPrivate.getParam("rate", rate_);

	nhPrivate.getParam("axis_x", axis_x);
	nhPrivate.getParam("axis_y", axis_y);
	nhPrivate.getParam("axis_z", axis_z);
	nhPrivate.getParam("axis_roll", axis_roll);
	nhPrivate.getParam("axis_pitch", axis_pitch);
	nhPrivate.getParam("axis_yaw", axis_yaw);

	nhPrivate.getParam("button_fast", button_fast);
	nhPrivate.getParam("button_slow", button_slow);
	nhPrivate.getParam("button_depth_up", button_depth_up);
	nhPrivate.getParam("button_depth_down", button_depth_down);

	wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("force_req", 1);
	mode_pub_ = nh_.advertise<tut_arrows_msgs::FlippersModeCmd>("force_mode", 1);
	depth_pub_ = nh_.advertise<std_msgs::Float32>("depth_setpoint", 1);

	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RemoteDriver::joyCallback, this);
}

void RemoteDriver::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	if (joy->buttons.at(button_fast) > 0 || joy->buttons.at(button_slow) > 0)
    {
		tut_arrows_msgs::FlippersModeCmd mode;
		mode.header.stamp = ros::Time::now();

		if (joy->buttons.at(button_fast) > 0)
		{
			mode.mode = tut_arrows_msgs::FlippersModeCmd::MODE_FAST;
		}
		else if (joy->buttons.at(button_slow) > 0)
		{
			mode.mode = tut_arrows_msgs::FlippersModeCmd::MODE_SLOW;
		}

		mode_pub_.publish(mode);
    }

	wrench_.wrench.force.x = joy->axes.at(axis_x);
	wrench_.wrench.force.y = joy->axes.at(axis_y);
	wrench_.wrench.force.z = joy->axes.at(axis_z);
	wrench_.wrench.torque.x = joy->axes.at(axis_roll);
	wrench_.wrench.torque.y = joy->axes.at(axis_pitch);
	wrench_.wrench.torque.z = joy->axes.at(axis_yaw);

	if (joy->buttons.at(button_depth_down) > 0)
	{
		depth_ -= 0.1;
	}

	if (joy->buttons.at(button_depth_up) > 0)
	{
		depth_ += 0.1;

		if (depth_ > 0)
		{
			depth_ = 0;
		}
	}
}

void RemoteDriver::run()
{
	ros::Rate r(rate_);
	while (ros::ok())
	{
		ros::spinOnce();

		wrench_.header.stamp = ros::Time::now();
		wrench_pub_.publish(wrench_);

		std_msgs::Float32 depth;
		depth.data = depth_;
		depth_pub_.publish(depth);

		r.sleep();
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "RemoteDriver");
	RemoteDriver remoteDriver;
	remoteDriver.run();
}
