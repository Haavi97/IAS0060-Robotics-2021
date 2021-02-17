#include "tut_arrows/UcatDynamics.h"

#include <cmath>
#include <iostream>

namespace
{
	/// skew - symmetry matrix (cross product operator)
	///
	Eigen::Matrix3d S(const Eigen::Vector3d& k)
	{
		Eigen::Matrix3d s;
		s << 0, -k(2), k(1),
			k(2), 0, -k(0),
			-k(1), k(0), 0;
		return s;
	}
}

UcatDynamics::UcatDynamics() 
	: M_(Matrix6::Zero()), MInverse_(Matrix6::Zero())
	, C_(Matrix6::Zero()), D_(Matrix6::Zero()), G_(Vector6::Zero())
{
	Mass = 19;
	WaterDensity = 1000;
	CentreOfMass = Vector3(0, 0, 0); // Vector3(0, 0, 0.02);
	Gamma = 0.5236; // 30 degrees

	// Moment of inertia U - CAT(solidworks)
	ScalarType Iyy = 464600000 * 1e-9;
	ScalarType Iyz = -253500 * 1e-9;
	ScalarType Iyx = 88700 * 1e-9;

	ScalarType Izy = Iyz;
	ScalarType Izz = 511510000 * 1e-9;
	ScalarType Izx = -549900 * 1e-9;

	ScalarType Ixy = Iyx;
	ScalarType Ixz = Izx;
	ScalarType Ixx = 157400000 * 1e-9;

	// Inertia matrix
	Ib << Ixx, -Ixy, -Ixz,
		-Iyx, Iyy, -Iyz,
		-Izx, -Izy, Izz;

	// Gravity matrix constants
	RG = Vector3(0, 0, 0); //Vector3(0, 0, 0.02); // Centre of gravity
	RB = Vector3(0, 0, 0); // Centre of buoyancy
	We = Mass * 9.81; // Gravity
	Bo = We * 1.00;

	// Damping coef - s
	Cd1 = 0.46;
	Cd2 = 0.7;
	Cd3 = 0.91;
	Cd4 = 0.9;
	Cd5 = 0.8;
	Cd6 = 0.7;

	FinLength = 0.18;
	FinWidth = 0.1;

	// force radius - fins
	RFx = 0.23;
	RFy = 0.1;
	RFz = 0;

	// initialize system inertia matrix
	initM();
}

void UcatDynamics::initM()
{
	M11 = Mass * Matrix3::Identity();
	M12 = -Mass * S(CentreOfMass);

	Matrix6 MRB;
	MRB.block<3, 3>(0, 0) = M11;
	MRB.block<3, 3>(0, 3) = M12;
	MRB.block<3, 3>(3, 0) = -M12;
	MRB.block<3, 3>(3, 3) = Ib;

	//std::cout << "MRB" << std::endl << MRB << std::endl;

	// Added mass - strip theory
	ScalarType Xuh = MRB(0, 0) - 59.0;
	ScalarType Yvh = MRB(1, 1) - 40.0;
	ScalarType Zwh = MRB(2, 2) - 168.0;
	ScalarType Kph = 0.1269;
	ScalarType Mqh = -1.2052;
	ScalarType Nrh = MRB(5, 5) - 2.8179;
	ScalarType Mwh = -1.0154;

	// Added mass matrix
	Vector6 MAdded_temp;
	MAdded_temp << Xuh, Yvh, Zwh, Kph, Mqh, Nrh;

	MAdded = MAdded_temp.asDiagonal() * -1.0;
	MAdded(2, 4) = Mwh;
	MAdded(4, 2) = Mwh;

	//std::cout << "MAdded" << std::endl << MAdded << std::endl;

	M_ = MAdded + MRB;
	MInverse_ = M_.inverse();

	//std::cout << "M added" << std::endl << MAdded << std::endl;
	//std::cout << "MRB" << std::endl << MRB << std::endl;
	//std::cout << "M" << std::endl << M_ << std::endl;
	//std::cout << "M inverse:" << std::endl << MInverse_ << std::endl;
}

// System inertia matrix (including added mass)
//
const UcatDynamics::Matrix6& UcatDynamics::getM() const
{
	return M_;
}

const UcatDynamics::Matrix6& UcatDynamics::getMInverse() const
{
	return MInverse_;
}

// Coriolis-centripedal matrix (including added mass)
//
// @param linearVelocity [in] - in body fixed frame
// @param angularVelocity [in] - in body fixed frame
//
const UcatDynamics::Matrix6& UcatDynamics::getC(const Vector3& linearVelocity, const Vector3& angularVelocity)
{
	Matrix3 C12 = -S(M11 * linearVelocity + M12 * angularVelocity);
	Matrix3 C22 = -S(M12 * linearVelocity + Ib * angularVelocity);

	Matrix6 CRB;
	CRB.block<3, 3>(0, 0) = Matrix3::Zero();
	CRB.block<3, 3>(0, 3) = -C12;
	CRB.block<3, 3>(3, 0) = -C12;
	CRB.block<3, 3>(3, 3) = -C22;

	const Matrix3& A11 = MAdded.block<3, 3>(0, 0);
	const Matrix3& A12 = MAdded.block<3, 3>(0, 3);
	const Matrix3& A21 = MAdded.block<3, 3>(3, 0);
	const Matrix3& A22 = MAdded.block<3, 3>(3, 3);

	Matrix3 CA12 = -S(A11 * linearVelocity + A12 * angularVelocity);
	Matrix3 CA22 = -S(A21 * linearVelocity + A22 * angularVelocity);

	Matrix6 CAdded;
	CAdded.block<3, 3>(0, 0) = Matrix3::Zero();
	CAdded.block<3, 3>(0, 3) = CA12;
	CAdded.block<3, 3>(3, 0) = CA12;
	CAdded.block<3, 3>(3, 3) = CA22;

	C_ = CRB; //+ CAdded;

	//std::cout << "CRB: " << std::endl << CRB << std::endl;
	//std::cout << "CAdded: " << std::endl << CAdded << std::endl;
	//std::cout << "C: " << std::endl << C_ << std::endl;

	return C_;
}

// Damping matrix
//
// @param linearVelocity [in] - in body fixed frame
// @param angularVelocity [in] - in body fixed frame
//
const UcatDynamics::Matrix6& UcatDynamics::getD(const Vector3& linearVelocity, const Vector3& angularVelocity)
{
	ScalarType finAngle1 = 1.57;
	ScalarType finAngle2 = 1.57;
	ScalarType finAngle3 = 1.57;
	ScalarType finAngle4 = 1.57;

	ScalarType Syz = 56200.0 * 1e-6 + FinWidth * std::sin(Gamma) * FinLength *
		(std::sin(finAngle1) + std::sin(finAngle2) + std::sin(finAngle3) + std::sin(finAngle4));

	ScalarType Sxy = 164000.0 * 1e-6 + FinWidth * FinLength *
		(std::cos(finAngle1) + std::cos(finAngle2) + std::cos(finAngle3) + std::cos(finAngle4));

	ScalarType Sxz = 124000.0 * 1e-6;

	ScalarType SRx = 35000.0 * 1e-6 + FinWidth * FinLength *
		(std::cos(finAngle1) + std::cos(finAngle2) + std::cos(finAngle3) + std::cos(finAngle4));

	ScalarType SRy = Sxy;
	ScalarType SRz = Sxz;

	D_(0, 0) = 56.0 * std::abs(linearVelocity(0));
	D_(1, 1) = 551.0 * abs(linearVelocity(1));
	D_(2, 2) = 361.0 * abs(linearVelocity(2));
	D_(3, 3) = 0.5 * WaterDensity * Cd4 * SRx * std::pow(0.57*0.288, 2) * std::abs(angularVelocity(0));
	D_(4, 4) = 0.5 * WaterDensity * Cd5 * SRy * std::pow(0.57*0.314, 2) * std::abs(angularVelocity(1));
	D_(5, 5) = 0.7226 * abs(angularVelocity(2));

	//std::cout << "D: " << std::endl << D_ << std::endl;

	return D_;
}

// Vector of gravitational/buoyancy forces and moments
//
const UcatDynamics::Vector6& UcatDynamics::getG(const Quaternion& orientation)
{
	Vector3 fGravityEarth(0, 0, We);
	Vector3 fBuoyancyEarth(0, 0, -Bo);

	Vector3 fGravityBody = QuatUtils::quatRotate(orientation.inverse(), fGravityEarth);
	Vector3 fBuoyancyBody = QuatUtils::quatRotate(orientation.inverse(), fBuoyancyEarth);

	G_.block<3, 1>(0, 0) = fGravityBody + fBuoyancyBody;
	G_.block<3, 1>(3, 0) = RG.cross(fGravityBody) + RB.cross(fBuoyancyBody);

	return G_;
}

UcatDynamics::Vector6 UcatDynamics::getAcceleration(const Quaternion& orientation, const Vector3& linearVelocity, const Vector3& angularVelocity, const Vector3& force, const Vector3& torque)
{
	const Matrix6& C = getC(linearVelocity, angularVelocity);
	const Matrix6& D = getD(linearVelocity, angularVelocity);

	Vector6 velocity;
	velocity << linearVelocity, angularVelocity;

	Vector6 wrench;
	wrench << force, torque;

	const Vector6 fC = C * velocity;
	const Vector6 fD = D * velocity;
	const Vector6& fG = getG(orientation);

	return -MInverse_ * (fC + fD + fG - wrench);
}
