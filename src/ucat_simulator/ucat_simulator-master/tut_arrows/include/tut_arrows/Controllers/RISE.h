#ifndef CONTROLLERS_RISE_H_
#define CONTROLLERS_RISE_H_

#include <sstream>

#include "tut_arrows/ControllerUtils.h"
#include "tut_arrows/Controllers/IController.h"

class RISE : public IController
{
public:
	RISE(double ks, double alpha1, double alpha2, double beta, double windupLimit)
	: ks_(ks), alpha1_(alpha1), alpha2_(alpha2), beta_(beta), vf_(0), e20_(0)
	, errors_(2), windupGuard_(windupLimit)
	, isFirstUpdate_(true)
	{ }

	virtual ~RISE() { }

	void limitIntError()
	{
		if (windupGuard_ <= 0)
		{
			// disabled
			return;
		}

		if (vf_ < -windupGuard_)
		{
			vf_ = -windupGuard_;
		}
		else if (vf_ > windupGuard_)
		{
			vf_ = windupGuard_;
		}
	}

	virtual double update(double error, double dt)
	{
		errors_.push(error);
		const double derivative = ControllerUtils::getDerivative(errors_, dt, 1);

		if (isFirstUpdate_ && errors_.isFull())
		{
			e20_ = derivative + alpha1_ * error;
			//ROS_INFO("e20 = %g (%g, %g)", e20_, derivative, error);
			isFirstUpdate_ = false;
		}

		const double e2 = derivative + alpha1_ * error;

		const double vfd = (ks_ + 1.0) * e2 * alpha2_ + beta_ * (e2 >= 0 ? 1.0 : -1.0);
		vf_ += vfd * dt;
		limitIntError();

		return (ks_ + 1.0) * e2 /* - (ks_ + 1.0) * e20_; */ + vf_;
	}

	virtual std::string toString() const
	{
		std::stringstream strStream;
		strStream << "RISE: ks = " << ks_ << " alpha1 = " << alpha1_ << " alpha2 = " << alpha2_ << " beta = " << beta_;
		return strStream.str();
	}

private:
	double ks_;
	double alpha1_;
	double alpha2_;
	double beta_;

	double e20_;
	double vf_;

	double windupGuard_;

	ControllerUtils::RingBuffer<double> errors_;

	bool isFirstUpdate_;
};

#endif // CONTROLLERS_RISE_H_
