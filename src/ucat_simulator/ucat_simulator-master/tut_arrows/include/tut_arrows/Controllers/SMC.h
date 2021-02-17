#ifndef CONTROLLERS_SLIDING_MODE_H_
#define CONTROLLERS_SLIDING_MODE_H_

#include <sstream>

#include "tut_arrows/ControllerUtils.h"
#include "tut_arrows/Controllers/IController.h"

/// Sliding mode controller
///
class SMC : public IController
{
public:
	SMC(double K, double lambda, double slope)
	: K_(K), lambda_(lambda), slope_(slope)
	, errors_(2)
	{ }

	virtual ~SMC() { };

	virtual double update(double error, double dt)
	{
		errors_.push(error);
		const double derivative = ControllerUtils::getDerivative(errors_, dt, 1);

		double s = derivative  + lambda_ * error;

		//const double output = K_ * ControllerUtils::sign(s);
		const double output = K_ * (2 * ControllerUtils::sigmoid(s, slope_) - 1);

		return output;
	}

	virtual std::string toString() const
	{
		std::stringstream strStream;
		strStream << "SMC: K = " << K_ << " lambda = " << lambda_ << " slope = " << slope_;
		return strStream.str();
	}

private:
	double K_;
	double lambda_;
	double slope_;

	ControllerUtils::RingBuffer<double> errors_;
};

#endif // CONTROLLERS_SLIDING_MODE_H_
