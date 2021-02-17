#ifndef CONTROLLERS_NPD_H_
#define CONTROLLERS_NPD_H_

#include <sstream>
#include <cmath>

#include "tut_arrows/ControllerUtils.h"
#include "tut_arrows/Controllers/IController.h"

class NPD : public IController
{
public:
	NPD(double alfa1, double alfa2, double delta1, double delta2, double Kp, double Ki, double Kd, double windup_limit)
	: alfa1_(alfa1), alfa2_(alfa2)
	, delta1_(delta1), delta2_(delta2)
	, Kp_(Kp), Ki_(Ki), Kd_(Kd)
	, windupGuard_(windup_limit)
	, intError_(0.0), errors_(2)
	{ }

	virtual ~NPD() { }

	void limitIntError()
	{
		if (windupGuard_ < 0)
		{
			// disabled
			return;
		}

		if (intError_ < -windupGuard_)
		{
			intError_ = -windupGuard_;
		}
		else if (intError_ > windupGuard_)
		{
			intError_ = windupGuard_;
		}
	}

	virtual double update(double error, double dt)
	{
		// integration
		intError_ += error * dt;

		// limit integrative error
		limitIntError();

		//ROS_INFO("intError: %g", intError_);

		errors_.push(error);
		const double derivative = ControllerUtils::getDerivative(errors_, dt, 1);

		const double output = updateNpd(error, derivative);

		return output;
	}

	double getKp(double error) const
	{
		if (std::abs(error) > delta1_)
		{
			return Kp_ * std::pow( std::abs(error), (alfa1_ - 1.0) );
		}
		else
		{
			return Kp_ * std::pow( delta1_, (alfa1_ - 1.0) );
		}
	}

	double getKd(double errorDerivative) const
	{
		if (std::abs(errorDerivative) > delta2_)
		{
			return Kd_ * std::pow( std::abs(errorDerivative), (alfa2_ - 1.0) );
		}
		else
		{
			return Kd_ * std::pow( delta2_, (alfa2_ - 1.0) );
		}
	}

	double updateNpd(double error, double errorDerivative) const
	{
		double Kp = getKp(error);
		double Kd = getKd(errorDerivative);

		//ROS_INFO("Kp = %g\tKd = %g", Kp, Kd);

		return Kp * error + Ki_ * intError_ + Kd * errorDerivative;
	}

	virtual std::string toString() const
	{
		std::stringstream strStream;
		strStream << "NPD: alfa_1 = " << alfa1_ << " alfa_2 = " << alfa2_ << " delta_1 = " << delta1_ << " delta_2 = " << delta2_ << " Ki = " << Ki_;

		if (windupGuard_ >= 0)
		{
			strStream << " windup limit = " << windupGuard_;
		}

		return strStream.str();
	}

private:
	double intError_;
	ControllerUtils::RingBuffer<double> errors_;

	double windupGuard_;

	double delta1_, delta2_;
	double alfa1_, alfa2_;

	double Kp_;
	double Ki_;
	double Kd_;
};

#endif // CONTROLLERS_NPD_H_
