#include <ros/ros.h>

#include "auv_msgs/AcousticModemData.h"
#include <geometry_msgs/WrenchStamped.h>

class WrenchToAcousticMsg
{
public:
	WrenchToAcousticMsg();

	void onAcousticModemData(const auv_msgs::AcousticModemData::ConstPtr data);
	void onWrench(const geometry_msgs::WrenchStampedConstPtr wrenchStamped);

private:
	ros::NodeHandle nh_;

	ros::Publisher wrenchPub_;
	ros::Subscriber wrenchSub_;

	ros::Publisher acousticPub_;
	ros::Subscriber acousticSub_;

	double scale_x, scale_y, scale_z, scale_yaw;

	bool sendDefaultZero_;
};

WrenchToAcousticMsg::WrenchToAcousticMsg() : nh_(), scale_x(20.0), scale_y(20.0), scale_z(20.0), scale_yaw(5.0), sendDefaultZero_(true)
{
	ros::NodeHandle nhPrivate("~");

	nhPrivate.param("scale_x", scale_x, scale_x);
	nhPrivate.param("scale_y", scale_y, scale_y);
	nhPrivate.param("scale_z", scale_z, scale_z);
	nhPrivate.param("scale_yaw", scale_yaw, scale_yaw);
	nhPrivate.param("send_default_zero", sendDefaultZero_, sendDefaultZero_);

	wrenchPub_ = nh_.advertise<geometry_msgs::WrenchStamped>("force_req", 1);
	wrenchSub_ = nh_.subscribe("force_req", 1, &WrenchToAcousticMsg::onWrench, this);

	acousticPub_ = nh_.advertise<auv_msgs::AcousticModemData>("send_buffer", 100);
	acousticSub_ = nh_.subscribe("receive_buffer", 1, &WrenchToAcousticMsg::onAcousticModemData, this);
}

void WrenchToAcousticMsg::onAcousticModemData(const auv_msgs::AcousticModemData::ConstPtr data)
{
	const std::string msg(data->payload.begin(), data->payload.end());

	geometry_msgs::WrenchStamped wrench;
	wrench.header.stamp = ros::Time::now();
	wrench.wrench.force.x = 0.0;
	wrench.wrench.force.y = 0.0;
	wrench.wrench.force.z = 0.0;
	wrench.wrench.torque.x = 0.0;
	wrench.wrench.torque.y = 0.0;
	wrench.wrench.torque.z = 0.0;

	if (msg == "U") // up
	{
		wrench.wrench.force.z = scale_z;
	}
	else if (msg == "D") // down
	{
		wrench.wrench.force.z = -scale_z;
	}
	else if (msg == "F") // forward
	{
		wrench.wrench.force.x = scale_x;
	}
	else if (msg == "B") // backward
	{
		wrench.wrench.force.x = -scale_x;
	}
	else if (msg == "L") // left
	{
		wrench.wrench.force.y = -scale_y;
	}
	else if (msg == "R") // right
	{
		wrench.wrench.force.y = scale_y;
	}
	else if (msg == "Q") // rotate counter clockwise
	{
		wrench.wrench.torque.z = -scale_yaw;
	}
	else if (msg == "W") // rotate clockwise
	{
		wrench.wrench.torque.z = scale_yaw;
	}
	else if (msg == "0") // stop
	{
		// sending zero wrench to stop
	}
	else
	{
		return; // to prevent message being sent when command is not recognized
	}

	wrenchPub_.publish(wrench);
}

void WrenchToAcousticMsg::onWrench(const geometry_msgs::WrenchStampedConstPtr wrenchStamped)
{
	const geometry_msgs::Vector3& force = wrenchStamped->wrench.force;
	const geometry_msgs::Vector3& torque = wrenchStamped->wrench.torque;

	auv_msgs::AcousticModemData msg;

	if (force.z > 0) // up
	{
		msg.payload.push_back('U');
	}
	else if (force.z < 0) // down
	{
		msg.payload.push_back('D');
	}
	else if (force.x > 0) // forward
	{
		msg.payload.push_back('F');
	}
	else if (force.x < 0) // backward
	{
		msg.payload.push_back('B');
	}
	else if (force.y < 0) // left
	{
		msg.payload.push_back('L');
	}
	else if (force.y > 0) // right
	{
		msg.payload.push_back('R');
	}
	else if (torque.z < 0) // rotate counter clockwise
	{
		msg.payload.push_back('Q');
	}
	else if (torque.z > 0) // rotate clockwise
	{
		msg.payload.push_back('W');
	}
	else
	{
		if (sendDefaultZero_) // by default send stop command
		{
			msg.payload.push_back('0');
		}
		else // or don't sent any command
		{
			return;
		}
	}

	acousticPub_.publish(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "WrenchToAcousticMsg");
	WrenchToAcousticMsg wrenchToAcoustic;
	ros::spin();
	return 0;
}
