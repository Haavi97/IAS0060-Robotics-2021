#ifndef CONTROLLERS_SAWTOOTH_H_
#define CONTROLLERS_SAWTOOTH_H_

#include <sstream>

#include "tut_arrows/ControllerUtils.h"
#include "tut_arrows/Controllers/IController.h"

class SawtoothController : public IController
{
public:
	SawtoothController(double frequency, double min, double max)
	: step_(0.0), min_(min), max_(max), output_(0.0)
	{
		step_ = frequency * 2 * (max_ - min_);
	}

	virtual ~SawtoothController() { }

	virtual double update(double error, double dt)
	{
		output_ += step_ * dt;

		if (output_ > max_)
		{
			output_ = max_;
			step_ = step_ > 0.0 ? -step_ : step_;
		}
		else if (output_ < min_)
		{
			output_ = min_;
			step_ = step_ < 0.0 ? -step_ : step_;
		}

		return output_;
	}

	virtual std::string toString() const
	{
		std::stringstream strStream;
		strStream << "Sawtooth: step = " << step_ << " min = " << min_ << " max = " << max_;

		return strStream.str();
	}

private:
	double step_;
	double min_;
	double max_;
	double output_;
};

#endif // CONTROLLERS_SAWTOOTH_H_
