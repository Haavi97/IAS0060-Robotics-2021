#ifndef CONTROLLERS_PID_H_
#define CONTROLLERS_PID_H_

#include <sstream>

#include "tut_arrows/ControllerUtils.h"
#include "tut_arrows/Controllers/IController.h"

class PID : public IController
{
public:
	PID(double Kp, double Ki, double Kd, double windupLimit)
	: Kp_(Kp), Ki_(Ki), Kd_(Kd), windupGuard_(windupLimit)
	, errors_(2), intError_(0)
	{ }

	virtual ~PID() { }

	void limitIntError()
	{
		if (windupGuard_ <= 0)
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

		//ROS_INFO("Integral error %g", intError_);

		errors_.push(error);
		const double derivative = ControllerUtils::getDerivative(errors_, dt, 1);

		return Kp_ * error + Ki_ * intError_ + Kd_ * derivative;
	}

	virtual std::string toString() const
	{
		std::stringstream strStream;
		strStream << "PID: Kp = " << Kp_ << " Ki = " << Ki_ << " Kd = " << Kd_;

		if (windupGuard_ >= 0.0)
		{
			strStream << " windup limit = " << windupGuard_;
		}

		return strStream.str();
	}

private:
	double Kp_;
	double Ki_;
	double Kd_;

	double windupGuard_;

	ControllerUtils::RingBuffer<double> errors_;
	double intError_;
};

#endif // CONTROLLERS_PID_H_
