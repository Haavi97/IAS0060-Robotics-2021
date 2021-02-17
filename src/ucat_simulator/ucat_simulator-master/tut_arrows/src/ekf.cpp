#include "tut_arrows/ekf.h"

#include <cmath>
#include <cfloat>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include "CppNumericalSolvers/BfgsSolver.h"

namespace RotationUtils
{
	/// Normalize angle to range [-PI ... PI].
	///
	template <class T>
	T normalizeAngle(T angle)
	{
		T x = fmod(angle + M_PI, 2.0 * M_PI);
		if (x < 0)
		{
			x += 2.0 * M_PI;
		}
		return x - M_PI;
	}

	/// Get smallest rotation (in range [-PI, PI]) between target and source.
	///
	template <class T>
	T getSmallestRotation(T target, T source)
	{
		target = normalizeAngle(target);
		source = normalizeAngle(source);

		T diff = target - source;

		if (diff > M_PI)
		{
			return diff - 2 * M_PI;
		}
		else if (diff < -M_PI)
		{
			return diff + 2 * M_PI;
		}

		return diff;
	}
}

namespace
{
	Eigen::Matrix3d getDefaultCovariance()
	{
		Eigen::Matrix3d defaultCov;
		defaultCov <<
					1.00, 0.00, 0.00,
					0.00, 1.00, 0.00,
					0.00, 0.00, 0.25;
		return defaultCov;
	}
}

Ekf::Ekf()
	: pos_(0.0, 0.0, 0.0)
	, orientation_(0.0, 0.0, 0.0)
	, cov_(getDefaultCovariance())
	, alfa1_(0.25)
	, alfa2_(0.05)
	, alfa3_(0.05)
	, alfa4_(0.25)
	, beacons_()
	, measurementVariance_(0.0034) // 99.7% probability of true angle being +-10 degrees of measured angle
	, rangeMeasurementVariance_(0.1)
	, likelihoodThreshold_(0.01)
{ }

void Ekf::setPose(const Eigen::Vector3d& position, const Eigen::Vector3d& rpy)
{
	pos_ = position;
	orientation_ = rpy;
	cov_ = getDefaultCovariance();
}

void Ekf::setOrientation(const Eigen::Vector3d& rpy)
{
	orientation_ = rpy;

	cov_(0,2) = 0;
	cov_(1,2) = 0;
	cov_(2,0) = 0;
	cov_(2,1) = 0;
	cov_(2,2) = 0.10;
}

void Ekf::setZ(double z)
{
	pos_(2) = z;
}

void Ekf::setCorrespondence(const std::string& id, const Eigen::Vector3d& location)
{
	beacons_[id] = location;
}

const Eigen::Vector3d& Ekf::getCorrespondence(const std::string& id) const
{
	return beacons_.at(id);
}

void Ekf::setLikelihoodThreshold(double likelihoodThreshold)
{
	likelihoodThreshold_ = likelihoodThreshold;
}

void Ekf::setMeasurementVariance(double measurementVariance)
{
	measurementVariance_ = measurementVariance;
}

void Ekf::setRangeMeasurementVariance(double rangeMeasurementVariance)
{
	rangeMeasurementVariance_ = rangeMeasurementVariance;
}

void Ekf::setMotionVariance(double alfa1, double alfa2, double alfa3, double alfa4)
{
	alfa1_ = alfa1;
	alfa2_ = alfa2;
	alfa3_ = alfa3;
	alfa4_ = alfa4;
}

/// yaw is always set to angle in range [-PI, PI]
///
void Ekf::setYaw(double newYaw)
{
	orientation_(2) = RotationUtils::normalizeAngle(newYaw);
}

/// @param velocity in local reference frame
/// @param angularVelocity in local reference frame
///
void Ekf::motionUpdate(const Eigen::Vector3d& velocity, const Eigen::Vector3d& angularVelocity, double dt)
{
	const double yaw = orientation_.z();

	Eigen::Matrix3d G;
	Eigen::Matrix3d V;

	double dx = 0.0;
	double dy = 0.0;
	double dYaw = 0.0;

	if (angularVelocity.z() != 0)
	{
		const double vt_wt = velocity.norm() / angularVelocity.z();

		// angle between velocity vector and estimated global reference frame
		const double yaw2 = yaw + std::atan2(velocity.y(), velocity.x());

		dYaw = angularVelocity.z() * dt;
		dx = vt_wt * ( -std::sin(yaw2) + std::sin(yaw2 + dYaw) );
		dy = vt_wt * (  std::cos(yaw2) - std::cos(yaw2 + dYaw) );

		const double g13 = vt_wt * ( -std::cos(yaw2) + std::cos(yaw2 + dYaw) );
		const double g23 = vt_wt * ( -std::sin(yaw2) + std::sin(yaw2 + dYaw) );

		G << 1.0, 0.0, g13,
			 0.0, 1.0, g23,
			 0.0, 0.0, 1.0;

		const double v11 = ( -std::sin(yaw) + std::sin(yaw + dYaw) ) / angularVelocity.z();
		const double v12 = ( -std::cos(yaw) + std::cos(yaw + dYaw) ) / angularVelocity.z();
		const double v13 =
				( velocity.norm() * ( std::sin(yaw2) - std::sin(yaw2+ dYaw) ) / ( angularVelocity.z() * angularVelocity.z() ) ) +
				( velocity.norm() * std::cos(yaw2 + dYaw) * dt / angularVelocity.z() );

		const double v21 = (  std::cos(yaw) - std::cos(yaw + dYaw) ) / angularVelocity.z();
		const double v22 = ( -std::sin(yaw) + std::sin(yaw + dYaw) ) / angularVelocity.z();
		const double v23 =
				-( velocity.norm() * ( std::cos(yaw2) - std::cos(yaw2 + dYaw) ) / ( angularVelocity.z() * angularVelocity.z() ) ) +
				 ( velocity.norm() * std::sin(yaw2 + dYaw) * dt / angularVelocity.z() );

		V << v11, v12, v13,
			 v21, v22, v23,
			 0.0, 0.0,  dt;
	}
	else
	{
		dx = ( velocity(0) * std::cos(yaw) - velocity(1) * std::sin(yaw) ) * dt;
		dy = ( velocity(0) * std::sin(yaw) + velocity(1) * std::cos(yaw) ) * dt;

		const double g13 = ( -velocity(0) * std::sin(yaw) - velocity(1) * std::cos(yaw) ) * dt;
		const double g23 = (  velocity(0) * std::cos(yaw) - velocity(1) * std::sin(yaw) ) * dt;

		G << 1.0, 0.0, g13,
			 0.0, 1.0, g23,
			 0.0, 0.0, 1.0;

		const double v11 =  std::cos(yaw) * dt;
		const double v12 = -std::sin(yaw) * dt;
		const double v21 =  std::sin(yaw) * dt;
		const double v22 =  std::cos(yaw) * dt;

		V << v11, v12, 0.0,
			 v21, v22, 0.0,
			 0.0, 0.0, 0.0;
	}

	const double m11 = alfa1_ * velocity.squaredNorm() + alfa2_ * angularVelocity.z() * angularVelocity.z();
	const double m22 = m11;
	const double m33 = alfa3_ * velocity.squaredNorm() + alfa4_ * angularVelocity.z() * angularVelocity.z();

	Eigen::Matrix3d M;
	M << m11, 0.0, 0.0,
		 0.0, m22, 0.0,
		 0.0, 0.0, m33;

	const Eigen::Matrix3d stateCov = G * cov_ * G.transpose();
	const Eigen::Matrix3d controlCov = V * M * V.transpose();

	//std::cout << "state cov: " << std::endl << stateCov << std::endl;
	//std::cout << "control cov: " << std::endl << controlCov << std::endl;

	pos_(0) = pos_.x() + dx;
	pos_(1) = pos_.y() + dy;
	setYaw(yaw + dYaw);

	cov_ = stateCov + controlCov;
}

void Ekf::doubleRangeUpdate(const std::string& id1, double range1, const std::string& id2, double range2)
{
	std::cout << "Double range update " << id1 << ", " << id2 << std::endl;

	auto modem1It = beacons_.find(id1);
	auto modem2It = beacons_.find(id2);
	// if unknown beacon then discard it
	if (modem1It == beacons_.end() || modem2It == beacons_.end())
	{
		return;
	}

	const Eigen::Vector3d& modem1 = modem1It->second;
	const Eigen::Vector3d& modem2 = modem2It->second;

	std::cout << "Modem 1: " << modem1.transpose() << " (" << range1 <<  ")" << std::endl;
	std::cout << "Modem 2: " << modem2.transpose() << " (" << range2 <<  ")" << std::endl;

	//std::cout << "Initial pos: " << pos_.transpose() << std::endl;

	Eigen::VectorXd m1(2);
	m1 << modem1[0], modem1[1];

	Eigen::VectorXd m2(2);
	m2 << modem2[0], modem2[1];

	// Project to modem plane using Pythagorean Theorem
	const double r1 = std::sqrt(std::pow(range1, 2) - std::pow(modem1.z() - pos_.z(), 2));
	const double r2 = std::sqrt(std::pow(range2, 2) - std::pow(modem2.z() - pos_.z(), 2));

    auto objectiveFunction = [&](const Eigen::VectorXd& x) -> double
    {
        const double c1 = std::pow((m1 - x).squaredNorm() - r1 * r1, 2);
        const double c2 = std::pow((m2 - x).squaredNorm() - r2 * r2, 2);
		const double cost = c1 + c2;
        return cost;
    };

    // create derivative of function
    auto partialDerivatives = [&](const Eigen::VectorXd x, Eigen::VectorXd& grad) -> void
    {
		const double e1 = (m1 - x).squaredNorm() - r1 * r1;
		const double e2 = (m2 - x).squaredNorm() - r2 * r2;

        grad[0]  = -4.0 * (m1[0] - x[0]) * e1 - 4.0 * (m2[0] - x[0]) * e2;
        grad[1]  = -4.0 * (m1[1] - x[1]) * e1 - 4.0 * (m2[1] - x[1]) * e2;
    };

    Eigen::VectorXd initialPos(2);
    initialPos << pos_[0], pos_[1];

    Eigen::VectorXd estPos = initialPos;

    pwie::BfgsSolver g;
    g.solve(estPos, objectiveFunction, partialDerivatives);

    Eigen::VectorXd change(2);
    change = estPos - initialPos;

    const double stdDev = 5.0;

    const double innovation = change.norm();

    const double multiplier = innovation < stdDev ? 1.0 : stdDev / innovation;

    std::cout << "New position = (" << estPos[0] << ", " << estPos[1] << ")" << std::endl;

    if (multiplier > 0.1)
    {
    	pos_(0) += change[0] * multiplier;
    	pos_(1) += change[1] * multiplier;
    }
}


//void Ekf::doubleRangeUpdate(const std::string& id1, double range1, const std::string& id2, double range2)
//{
//	auto modem1It = beacons_.find(id1);
//	auto modem2It = beacons_.find(id2);
//	// if unknown beacon then discard it
//	if (modem1It == beacons_.end() || modem2It == beacons_.end())
//	{
//		return;
//	}
//
//	const Eigen::Vector3d& modem1Pos = modem1It->second;
//	const Eigen::Vector3d& modem2Pos = modem2It->second;
//
//	Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(modem2Pos - modem1Pos, Eigen::Vector3d(1.0, 0.0, 0.0));
//	Eigen::Vector3d translation(modem1Pos);
//
//	Eigen::Vector3d posInBeaconCoord = rotation.toRotationMatrix() * (pos_ - translation); // rotate to modem coordinate system
//
//	const double distance = (modem1Pos - modem2Pos).norm();
//	const double distanceSquared = distance * distance;
//
//	const double range1Squared = range1 * range1;
//	const double range2Squared = range2 * range2;
//
//	const double x = (distanceSquared - range2Squared + range1Squared) / (2.0 * distance);
//
//	const double radius_1 = 4.0 * distanceSquared * range1Squared - std::pow(distanceSquared - range2Squared + range1Squared, 2);
//	if (radius_1 < 0)
//	{
//		return;
//	}
//	const double radius = std::sqrt( radius_1 ) / (2.0 * distance);
//
//	Eigen::Vector2d projectedPoint(posInBeaconCoord.y(), posInBeaconCoord.z()); // project to y-z plane
//	Eigen::Vector2d closestPoint = radius * projectedPoint / projectedPoint.norm();
//	Eigen::Vector3d closestPoint3d(x, closestPoint.x(), closestPoint.y());
//
//	Eigen::Vector3d newPos = rotation.inverse().toRotationMatrix() * closestPoint3d + translation;
//
//	Eigen::Vector3d innovation = newPos - pos_;
//
//	double innovationDistance = innovation.norm();
//
//	//std::cout << "innovationDistance = " << innovationDistance << std::endl;
//	//std::cout << "rangeMeasurementVariance = " << rangeMeasurementVariance_ << std::endl;
//
//	if (innovationDistance <= 0.0)
//	{
//		return;
//	}
//
//	double dist = innovation.norm() / rangeMeasurementVariance_;
//
//	Eigen::Matrix3d K = Eigen::Vector3d(1.0/dist, 1.0/dist, 1.0/dist).asDiagonal();
//
//	const Eigen::Vector3d change = K * innovation;
//
//	//std::cout << "change (" << change.x() << ", " << change.y() << ")" << std::endl;
//
//	pos_(0) = pos_.x() + change.x();
//	pos_(1) = pos_.y() + change.y();
//}

void Ekf::measurementUpdate(const std::string& id, double range)
{
	auto beaconIt = beacons_.find(id);
	// if unknown beacon then discard it
	if (beaconIt == beacons_.end())
	{
		return;
	}

	const Eigen::Vector3d beaconPos = beaconIt->second;

	// calculating how far the beacon is from currently estimated position
	const Eigen::Vector3d diff = beaconPos - pos_;
	const double expected_range = diff.norm();

	if (expected_range < 0.0001)
	{
		//std::cout << "Calculated length to beacon is 0, canceling measurement update." << std::endl;
		return;
	}

	const double innovation = range - expected_range;

	//std::cout << "Innovation = " << innovation << std::endl;

	// measurement model Jacobian in respect to state vector
	const Eigen::RowVector3d H(-diff.x()/expected_range, -diff.y()/expected_range, -diff.z()/expected_range);

	// innovation variance
	const double S = H * cov_ * H.transpose() + rangeMeasurementVariance_;

	if (S <= 0)
	{
		//std::cout << "Innovation variance is 0 or negative, canceling measurement update." << std::endl;
		return;
	}

	const double S_inv = 1.0 / S;

	// given current pose estimate how likely is the new measurement
	const double likelihood = (1.0 / std::sqrt(2.0 * M_PI * S)) * std::exp(-0.5*innovation*S_inv*innovation);

	//std::cout << "Likelihood = " << likelihood << std::endl;

	// if likelihood of this measurement is too low then discard it
	if (likelihood < likelihoodThreshold_)
	{
		std::cout << "Discarding measurement (" << likelihood << ")" << std::endl;
		return;
	}

	// Kalman gain =
	//   cov_ (state covariance) - measures how accurate is current state
	//   H (measurement model) - how change in state variables affects measurement outcome
	//   S (innovation covariance) - measures how accurate is measurement given uncertainty about current state and uncertainty caused by measurement itself
	const Eigen::Vector3d K = cov_ * H.transpose() * S_inv;

	//std::cout << "K:" << std::endl << K << std::endl;
	//std::cout << "Covariance:" << std::endl << cov_ << std::endl;

	const Eigen::Vector3d change = K * innovation;

	//std::cout << std::setprecision(5) << std::fixed << "Change (" << change.x() << ", " << change.y() << ", " << change.z() << ")" << std::endl;

	pos_(0) = pos_.x() + change.x();
	pos_(1) = pos_.y() + change.y();
	setYaw(orientation_.z() + change.z());

	Eigen::Matrix3d IKH = (Eigen::Matrix3d::Identity() - K*H);

	// use Joseph form covariance update to make sure that the symmetry and
	// positive definiteness is preserved
	cov_ = IKH * cov_ * IKH.transpose() + K * measurementVariance_ * K.transpose();
}

void Ekf::measurementUpdate(const std::string& id ,const Eigen::Vector3d& direction)
{
	auto beaconIt = beacons_.find(id);
	// if unknown beacon then discard it
	if (beaconIt == beacons_.end())
	{
		return;
	}

	const Eigen::Vector3d beaconPos = beaconIt->second;

	// calculating how far the beacon is from currently estimated position
	const double dx = beaconPos.x() - pos_.x();
	const double dy = beaconPos.y() - pos_.y();
	const double q = dx * dx + dy * dy;

	if (q == 0)
	{
		//std::cout << "Calculated length to beacon is 0, canceling measurement update." << std::endl;
		return;
	}

	const double yaw = RotationUtils::normalizeAngle(orientation_.z());

	// expected angle at which the beacon signal should be measured
	const double z_expected = std::atan2(dy, dx) - yaw;

	const double z_actual = std::atan2(direction.y(), direction.x());

	// innovation is difference between expected and actual measurement
	const double innovation = RotationUtils::getSmallestRotation(z_actual, z_expected);

	//std::cout << "Innovation = " << innovation << std::endl;

	// measurement model Jacobian in respect to state vector
	const Eigen::RowVector3d H(dy/q, -dx/q, -1.0);

	// innovation variance
	const double S = H * cov_ * H.transpose() + measurementVariance_;

	if (S <= 0)
	{
		//std::cout << "Innovation variance is 0 or negative, canceling measurement update." << std::endl;
		return;
	}

	const double S_inv = 1.0 / S;

	// given current pose estimate how likely is the new measurement
	const double likelihood = (1.0 / std::sqrt(2.0 * M_PI * S)) * std::exp(-0.5*innovation*S_inv*innovation);

	//std::cout << "Likelihood = " << likelihood << std::endl;

	// if likelihood of this measurement is too low then discard it
	if (likelihood < likelihoodThreshold_)
	{
		std::cout << "Discarding measurement (" << likelihood << ")" << std::endl;
		return;
	}

	// Kalman gain =
	//   cov_ (state covariance) - measures how accurate is current state
	//   H (measurement model) - how change in state variables affects measurement outcome
	//   S (innovation covariance) - measures how accurate is measurement given uncertainty about current state and uncertainty caused by measurement itself
	const Eigen::Vector3d K = cov_ * H.transpose() * S_inv;

	//std::cout << "K:" << std::endl << K << std::endl;
	//std::cout << "Covariance:" << std::endl << cov_ << std::endl;

	pos_(0) = pos_.x() + K.x() * innovation;
	pos_(1) = pos_.y() + K.y() * innovation;
	setYaw(orientation_.z() + K.z() * innovation);

	Eigen::Matrix3d IKH = (Eigen::Matrix3d::Identity() - K*H);

	// use Joseph form covariance update to make sure that the symmetry and
	// positive definiteness is preserved
	cov_ = IKH * cov_ * IKH.transpose() + K * measurementVariance_ * K.transpose();
}

void Ekf::iteratedMeasurementUpdate(const std::string& id ,const Eigen::Vector3d& direction)
{
	auto beaconIt = beacons_.find(id);
	// if unknown beacon then discard it
	if (beaconIt == beacons_.end())
	{
		return;
	}

	const Eigen::Vector3d beaconPos = beaconIt->second;

	Eigen::Vector3d initialState(pos_.x(), pos_.y(), orientation_.z());
	Eigen::Vector3d state = initialState;

	try
	{
		for (int i = 0; i < 20; ++i)
		{
			Eigen::Vector3d newState = measurementUpdateIteration(beaconPos, direction, initialState, state, false);

			const double change = (newState - state).norm();
			//std::cout << i << "-th iteration change " << change << std::endl;
			if (change < 0.00001)
			{
				break;
			}
			state = newState;
		}

		measurementUpdateIteration(beaconPos, direction, initialState, state, true);
	}
	catch (std::exception const& ex)
	{
		std::cout << ex.what() << std::endl;
	}
}

Eigen::Vector3d Ekf::measurementUpdateIteration(const Eigen::Vector3d& beaconPos, const Eigen::Vector3d& direction, Eigen::Vector3d const& initialState, Eigen::Vector3d const& state, bool update)
{
	// calculating how far the beacon is from currently estimated position
	const double dx = beaconPos.x() - state.x();
	const double dy = beaconPos.y() - state.y();
	const double q = dx * dx + dy * dy;

	if (q == 0)
	{
		throw std::logic_error("Calculated length to beacon is 0, canceling measurement update.");
	}

	// expected angle at which the beacon signal should be measured
	const double z_expected = std::atan2(dy, dx) - state.z();

	const double z_actual = std::atan2(direction.y(), direction.x());

	// innovation is difference between expected and actual measurement
	const double innovation = RotationUtils::getSmallestRotation(z_actual, z_expected);

	const Eigen::RowVector3d H_i(dy/q, -dx/q, -1.0);

	// innovation variance
	const double S = H_i * cov_ * H_i.transpose() + measurementVariance_;

	if (S <= 0)
	{
		throw std::logic_error("Innovation variance is 0 or negative, canceling measurement update.");
	}

	const double S_inv = 1.0 / S;

	const Eigen::Vector3d K_i = cov_ * H_i.transpose() * S_inv;

	Eigen::Vector3d newState = initialState + K_i * (innovation - H_i * (initialState - state));

	newState(2) = RotationUtils::normalizeAngle(newState.z());

	if (update)
	{
		pos_(0) = newState.x();
		pos_(1) = newState.y();
		orientation_(2) = newState(2);

		Eigen::Matrix3d IKH = (Eigen::Matrix3d::Identity() - K_i*H_i);
		cov_ = IKH * cov_ * IKH.transpose() + K_i * measurementVariance_ * K_i.transpose();
	}

	return newState;
}

const Eigen::Vector3d& Ekf::getPositionEstimate() const
{
	return pos_;
}

const Eigen::Vector3d& Ekf::getOrientationEstimate() const
{
	return orientation_;
}

const Eigen::Matrix3d& Ekf::getCovariance() const
{
	return cov_;
}

std::string Ekf::toString() const
{
	std::stringstream strStream;
	strStream << std::fixed << std::showpoint;
	strStream.precision(3);
	strStream << "x: " << pos_.x() << " y: " << pos_.y() << " yaw: " << orientation_.z() << std::endl;
	strStream << "covariance: " << std::endl << cov_ << std::endl;

	return strStream.str();
}
