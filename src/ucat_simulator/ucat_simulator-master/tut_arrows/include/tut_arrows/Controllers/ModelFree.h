#ifndef CONTROLLERS_I_PID_H_
#define CONTROLLERS_I_PID_H_

#include <sstream>
#include <memory>

#include "tut_arrows/ControllerUtils.h"
#include "tut_arrows/Controllers/IController.h"

class ModelFree : public IController
{
public:
	ModelFree(double alfa, int v, std::shared_ptr<IController> secondary_controller)
	: alfa_(alfa), v_(v), controller_(secondary_controller)
	, prevControl_(0.0), states_(3), desiredStates_(3)
	{ }

	virtual ~ModelFree() { }

	virtual double update(double error, double dt)
	{
		throw std::runtime_error("Doesn't work!!!");

//		const double desiredState = state + error;
//		desiredStates_.push(desiredState);
//
//		const double y_n_d = ControllerUtils::getDerivative(desiredStates_, dt, v_); // v-th derivative from desired output path
//
//		states_.push(state);
//		const double stateDerivative = ControllerUtils::getDerivative(states_, dt, v_);
//
//		const double F = stateDerivative - alfa_ * prevControl_;
//
//		double otherControl = 0.0;
//		if (controller_)
//		{
//			otherControl = controller_->update(error, dt);
//		}
//
//		const double control = - ( F - y_n_d - otherControl ) / alfa_;
//
//		prevControl_ = control;
//		return control;
	}

	virtual std::string toString() const
	{
		throw std::runtime_error("Doesn't work!!!");

//		std::stringstream strStream;
//		strStream << "Model free: alfa = " << alfa_ << " v = " << v_;
//		if (controller_)
//		{
//			strStream << " with " << controller_->toString();
//		}
//
//		return strStream.str();
	}

private:
	double alfa_;
	int v_;
	std::shared_ptr<IController> controller_;

	double prevControl_;
	ControllerUtils::RingBuffer<double> states_;
	ControllerUtils::RingBuffer<double> desiredStates_;
};

#endif // CONTROLLERS_I_PID_H_
