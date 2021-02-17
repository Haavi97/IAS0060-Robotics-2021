#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace QuatUtils
{
	template <typename ScalarType>
	Eigen::Matrix<ScalarType, 3, 1> quatRotate(const Eigen::Quaternion<ScalarType>& rotation, const Eigen::Matrix<ScalarType, 3, 1>& vector)
	{
		Eigen::Quaternion<ScalarType> q = rotation * Eigen::Quaternion<ScalarType>(0, vector.x(), vector.y(), vector.z());
		q *= rotation.inverse();
		return Eigen::Matrix<ScalarType, 3, 1>(q.x(), q.y(), q.z());
	}

	template <typename ScalarType>
	Eigen::Matrix<ScalarType, 3, 1> quatToEuler(const Eigen::Quaternion<ScalarType>& rotation)
	{
		return rotation.toRotationMatrix().eulerAngles(0, 1, 2);
	}

	template <typename ScalarType>
	Eigen::Quaternion<ScalarType> eulerToQuat(Eigen::Matrix<ScalarType, 3, 1> rpy)
	{
		Eigen::AngleAxis<ScalarType> rollAngle(rpy(0), Eigen::Vector3d::UnitX());
		Eigen::AngleAxis<ScalarType> pitchAngle(rpy(1), Eigen::Vector3d::UnitY());
		Eigen::AngleAxis<ScalarType> yawAngle(rpy(2), Eigen::Vector3d::UnitZ());

		return Eigen::Quaternion<ScalarType>(rollAngle * pitchAngle * yawAngle);
	}

	template <typename ScalarType>
	Eigen::Quaternion<ScalarType> getRotationFromAngularVelocity(const Eigen::Quaternion<ScalarType>& orientation, const Eigen::Matrix<ScalarType, 3, 1>& angularVelocity)
	{
		const ScalarType& w = orientation.w();
		const ScalarType& x = orientation.x();
		const ScalarType& y = orientation.y();
		const ScalarType& z = orientation.z();

		Eigen::Matrix<ScalarType, 4, 3> Tq;
		Tq << -x, -y, -z,
			w, -z, y,
			z, w, -x,
			-y, x, w;

		// quaternion initialized in form [w, x, y, z]
		Eigen::Matrix<ScalarType, 4, 1> quatMat = Tq * angularVelocity;
		Eigen::Quaternion<ScalarType> quat;
		quat.w() = quatMat(0);
		quat.x() = quatMat(1);
		quat.y() = quatMat(2);
		quat.z() = quatMat(3);
		return quat;
	}
}

class UcatDynamics
{
public:
	typedef double ScalarType;
	typedef Eigen::Matrix<ScalarType, 3, 1> Vector3;
	typedef Eigen::Matrix<ScalarType, 6, 1> Vector6;
	typedef Eigen::Matrix<ScalarType, 12, 1> Vector12;
	typedef Eigen::Matrix<ScalarType, 3, 3> Matrix3;
	typedef Eigen::Matrix<ScalarType, 6, 6> Matrix6;
	typedef Eigen::Quaternion<ScalarType> Quaternion;

public:
	UcatDynamics();

	const Matrix6& getM() const;
	const Matrix6& getMInverse() const;

	const Matrix6& getC(const Vector3& linearVelocity, const Vector3& angularVelocity);

	const Matrix6& getD(const Vector3& linearVelocity, const Vector3& angularVelocity);

	const Vector6& getG(const Quaternion& orientation);

	Vector6 getAcceleration(const Quaternion& orientation, const Vector3& linearVelocity, const Vector3& angularVelocity, const Vector3& force, const Vector3& torque);

private:
	void initM();

private:
	ScalarType Mass;
	ScalarType WaterDensity;
	Vector3 CentreOfMass;
	ScalarType Gamma;

	// Inertia matrix
	Matrix3 Ib;

	// Gravity matrix constants
	Vector3 RG; // Centre of gravity
	Vector3 RB; // Centre of buoyancy
	ScalarType We; // Gravity
	ScalarType Bo;

	// Damping coef - s
	ScalarType Cd1, Cd2, Cd3, Cd4, Cd5, Cd6;

	ScalarType FinLength;
	ScalarType FinWidth;

	// force radius - fins
	ScalarType RFx, RFy, RFz;

	Matrix3 M11, M12;
	Matrix6 MAdded;

	Matrix6 M_, MInverse_;
	Matrix6 C_;
	Matrix6 D_;
    Matrix6 External_;
	Vector6 G_;
};
