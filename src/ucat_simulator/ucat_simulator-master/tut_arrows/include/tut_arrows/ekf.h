#ifndef TUT_ARROWS_EKF_H_
#define TUT_ARROWS_EKF_H_

#include <map>
#include <string>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class Ekf
{
public:
	Ekf();

	void setPose(const Eigen::Vector3d& position, const Eigen::Vector3d& rpy);
	void setOrientation(const Eigen::Vector3d& rpy);
	void setZ(double z);
	void setCorrespondence(const std::string& id, const Eigen::Vector3d& location);
	const Eigen::Vector3d& getCorrespondence(const std::string& id) const;

	void setLikelihoodThreshold(double likelihoodThreshold);
	void setMeasurementVariance(double measurementVariance);
	void setRangeMeasurementVariance(double rangeMeasurementVariance);
	void setMotionVariance(double alfa1, double alfa2, double alfa3, double alfa4);

	void motionUpdate(const Eigen::Vector3d& velocity, const Eigen::Vector3d& angularVelocity, double timeStep);
	void measurementUpdate(const std::string& id ,const Eigen::Vector3d& direction);

	void measurementUpdate(const std::string& id, double range);
	void doubleRangeUpdate(const std::string& id1, double range1, const std::string& id2, double range2);

	void iteratedMeasurementUpdate(const std::string& id ,const Eigen::Vector3d& direction);
	Eigen::Vector3d measurementUpdateIteration(const Eigen::Vector3d& beaconPos, const Eigen::Vector3d& direction, Eigen::Vector3d const& initialState, Eigen::Vector3d const& state, bool update);

	const Eigen::Vector3d& getPositionEstimate() const;
	const Eigen::Vector3d& getOrientationEstimate() const;
	const Eigen::Matrix3d& getCovariance() const;

	std::string toString() const;

private:
	void setYaw(double newYaw);

private:
	// in world reference frame
	Eigen::Vector3d pos_; //(x, y, z)
	Eigen::Vector3d orientation_; // (roll, pitch, yaw)
	Eigen::Matrix3d cov_;

	//
	// Motion estimation
	//

	// control uncertainty parameters
	double alfa1_;
	double alfa2_;
	double alfa3_;
	double alfa4_;

	//
	// Measurement
	//

	std::map<std::string, Eigen::Vector3d> beacons_;

	double rangeMeasurementVariance_;
	double measurementVariance_; // measured angle variance
	double likelihoodThreshold_;
};

#endif //TUT_ARROWS_EKF_H_

