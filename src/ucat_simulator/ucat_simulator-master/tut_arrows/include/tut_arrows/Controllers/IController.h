#ifndef CONTROLLERS_I_CONTROLLER_H_
#define CONTROLLERS_I_CONTROLLER_H_

/// 1 DOF controller interface
///
class IController
{
public:
	virtual ~IController() {}

	/// Calculates controller output.
	///
	virtual double update(double error, double dt) = 0;

	/// Get description of controller setup (e.g. "PID with Kp = 1.0, Ki = 0.0, Kd = 0.0").
	///
	virtual std::string toString() const = 0;
};

#endif // CONTROLLERS_I_CONTROLLER_H_
