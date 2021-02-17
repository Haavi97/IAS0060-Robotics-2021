#ifndef CONTROLLERS_CONSTANT_H_
#define CONTROLLERS_CONSTANT_H_

#include <sstream>

#include "tut_arrows/ControllerUtils.h"
#include "tut_arrows/Controllers/IController.h"

class ConstantController : public IController
{
public:
	ConstantController(double K)
	: K_(K)
	{ }

	virtual ~ConstantController() { }

	virtual double update(double /*error*/, double /*dt*/)
	{
		return K_;
	}

	virtual std::string toString() const
	{
		std::stringstream strStream;
		strStream << "Constant: K = " << K_;

		return strStream.str();
	}

private:
	double K_;
};

#endif // CONTROLLERS_CONSTANT_H_
