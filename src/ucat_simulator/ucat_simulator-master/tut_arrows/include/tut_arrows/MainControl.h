#include <ros/ros.h>

#include <auv_msgs/AcousticModemData.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>

#include "tut_arrows/ObstacleAvoidance.h"
#include "tut_arrows/Controllers/IController.h"
#include "tut_arrows_msgs/FlippersModeCmd.h"
#include "tut_arrows_msgs/SetController.h"
#include "tut_arrows_msgs/TrajectoryPoint.h"
#include "tut_arrows_msgs/BeaconPing.h"

#include "tut_arrows/UcatDynamics.h"

struct ModeControllers
{
	std::shared_ptr<IController> x;
	std::shared_ptr<IController> y;
	std::shared_ptr<IController> z;
	std::shared_ptr<IController> roll;
	std::shared_ptr<IController> pitch;
	std::shared_ptr<IController> yaw;
};

class MainControl
{
public:
	enum Mode
	{
		NOOP,
		AUTO_CONTROL,
		MISSION_OUT
	};

	enum WrenchMode
	{
		NOT_SET,
		SLOW,
		FAST
	};

public:
	MainControl();

	void run();

	void acousticModemCallback(const auv_msgs::AcousticModemData::ConstPtr& data);

	void odometryCallback(nav_msgs::Odometry::ConstPtr data);

	void trajectoryCallback(tut_arrows_msgs::TrajectoryPoint::ConstPtr data);

	void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
	void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& pressure);

	void onBuoyanceF(const geometry_msgs::Vector3StampedConstPtr data);
	void onBuoyanceB(const geometry_msgs::Vector3StampedConstPtr data);

	void onRcWrenchStamped(const geometry_msgs::WrenchStampedConstPtr wrenchMsg);

	void onModeChange(const std_msgs::StringConstPtr mode);

	bool setController(tut_arrows_msgs::SetController::Request& request, tut_arrows_msgs::SetController::Response& response);

	void beaconPingCallback(const tut_arrows_msgs::BeaconPing::ConstPtr& ping);

private:
	void onControl();
	void onControl_AUTO_CONTROL();
	void onControl_MISSION_OUT();
	void onControlUpdate(const tf::Vector3& positionError, const tf::Vector3& orientationError, double dt);

	//void onSMC(const Eigen::Matrix<double, 6, 1>& poseError, const Eigen::Matrix<double, 6, 1>& velocityError, double dt);
	//void onNLPD(const Eigen::Matrix<double, 6, 1>& poseError, const Eigen::Matrix<double, 6, 1>& velocityError, UcatState state, double dt);

	void onSafety();

	bool checkSafety();

	double getBuoyance() const;
	void setBuoyance(double buoyance);

	void applyControl(double controlX, double controlY, double controlZ, double controlRoll, double controlPitch, double controlYaw);
	void applyCurrentWrench();

	void setMode(Mode mode);
	void setMode(const std::string& mode);
	void setWrenchMode(WrenchMode wrenchMode);
	void setWrenchMode(const std::string& wrenchMode);
	void setWrenchModeScales(WrenchMode wrenchMode);

	ModeControllers loadModeControllers(const std::string& mode);
	std::shared_ptr<IController> loadModeController(const std::string& mode, const std::string& controllerType);
	void logControllers(const ModeControllers& modeControllers) const;
	ModeControllers& getCurrentControllers();

	void normalizeWrench();

	void updateMeasurementState();

	void onControl_MISSION_OUT_saveBeaconData();

	void onControl_MISSION_OUT_swimTowardsBeacon();

private:
	ros::NodeHandle nh_;

	ros::Publisher wrenchPub_;
	ros::Publisher wrenchModePub_;
	ros::Publisher pubBuoyanceF_, pubBuoyanceB_;

	ros::Subscriber rcWrenchSub_;
	ros::Subscriber acousticModemSub_;
	ros::Subscriber odomSub_;
	ros::Subscriber subBuoyanceF_, subBuoyanceB_;
	ros::Subscriber modeSub_;
	ros::Subscriber pressureSub_;
	ros::Subscriber trajectorySub_;
	ros::Subscriber beaconSub_;

	ros::ServiceServer setControllerSrv_;

	double rate_;

	Mode mode_;
	WrenchMode wrenchMode_;
	double wrenchModeChangeTime_;
	ros::Time lastWrenchModeChangeTime_;

	geometry_msgs::WrenchStamped wrench_;
	tut_arrows_msgs::FlippersModeCmd wrenchModeCmd_;

	double buoyanceF_, buoyanceB_;

	double scaleX_, scaleY_, scaleZ_;
	double scaleRoll_, scalePitch_, scaleYaw_;

	double rateOfChange_;

	nav_msgs::Odometry::ConstPtr odom_;

	sensor_msgs::FluidPressure::ConstPtr lastFluidPressure_;
	double fluidPressureChangeTime_;

	ModeControllers fastModeControllers_;
	ModeControllers slowModeControllers_;

	tut_arrows_msgs::TrajectoryPoint::ConstPtr trajectoryPoint_;

	ObstacleAvoidance oa_;

	bool changeWrenchModeNow_;

	bool forceWrenchMode_;

	double measureInterval_;
	double measureDuration_;
	ros::Time lastMeasureTime_;
	bool isMeasuring_;
	ros::ServiceClient switchModemRangeDriver_;

	std::string mission_out_beacon_id_;
	bool saveBeaconData_;
	std::vector<tut_arrows_msgs::BeaconPing::ConstPtr> savedBeaconPings_;
	geometry_msgs::Vector3 lastBeaconDirection_;
	ros::Time lastBeaconDirectionTime_;
	double beaconInterval_;
	double beaconDuration_;
	int beaconMaxReadings_;
	double mission_out_speed_;
};
