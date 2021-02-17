#include <ros/ros.h>
#include "tut_arrows_msgs/ModemRange.h"
#include "tut_arrows_msgs/SendModemRange.h"
#include "tut_arrows_msgs/Switch.h"

class ModemRangeDriver
{
public:
	ModemRangeDriver();
	void onModemRange(tut_arrows_msgs::ModemRange::ConstPtr modemRange);
	void run();

	bool setEnabled(tut_arrows_msgs::Switch::Request& request, tut_arrows_msgs::Switch::Response& response);

private:
	void sendNext();

private:
	ros::NodeHandle nh_;
	std::vector<int> modemIds_;
	int currentIndex_;
	double cmdTimeout_;

	ros::Publisher modemRangeCmdPub_;
	ros::Subscriber modemRangeSub_;
	tut_arrows_msgs::SendModemRange cmd_;

	ros::ServiceServer enableSrv_;
	bool isEnabled_;
};

ModemRangeDriver::ModemRangeDriver() : nh_(), modemIds_(), currentIndex_(0), cmdTimeout_(10.0), isEnabled_(true)
{
	ros::NodeHandle nhPrivate("~");

	nhPrivate.getParam("modem_ids", modemIds_);
	nhPrivate.getParam("cmd_timeout", cmdTimeout_);
	nhPrivate.getParam("enabled", isEnabled_);

	if (modemIds_.size() == 0)
	{
		throw std::runtime_error("ModemRangeDriver node requires modem id-s to be set");
	}

	modemRangeCmdPub_ = nh_.advertise<tut_arrows_msgs::SendModemRange>("send_modem_range", 1);
	modemRangeSub_ = nh_.subscribe("modem_range", 1, &ModemRangeDriver::onModemRange, this);
	cmd_.header.stamp = ros::Time(0);

	enableSrv_ = nh_.advertiseService("EnableModemRangeDriver", &ModemRangeDriver::setEnabled, this);
}

void ModemRangeDriver::onModemRange(tut_arrows_msgs::ModemRange::ConstPtr modemRange)
{
	sendNext();
}

void ModemRangeDriver::sendNext()
{
	currentIndex_ = (currentIndex_ + 1) % modemIds_.size();

	cmd_.header.stamp = ros::Time::now();
	cmd_.modem_id = modemIds_[currentIndex_];

	modemRangeCmdPub_.publish(cmd_);
}

void ModemRangeDriver::run()
{
	ros::Rate rate(5);

	while (ros::ok())
	{
		ros::spinOnce();

		if ( isEnabled_ && (ros::Time::now() - cmd_.header.stamp).toSec() > cmdTimeout_ )
		{
			sendNext();
		}

		rate.sleep();
	}
}

bool ModemRangeDriver::setEnabled(tut_arrows_msgs::Switch::Request& request, tut_arrows_msgs::Switch::Response& response)
{
	isEnabled_ = request.enable;
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ModemRangeDriver");
	ModemRangeDriver modemRangeDrv;
	modemRangeDrv.run();
}
