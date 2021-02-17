#ifndef TUT_ARROWS_CONTROLLER_FACTORY_H_
#define TUT_ARROWS_CONTROLLER_FACTORY_H_

#include <ros/ros.h>

#include "tut_arrows/Controllers/IController.h"
#include "tut_arrows/Controllers/PID.h"
#include "tut_arrows/Controllers/NPD.h"
#include "tut_arrows/Controllers/SMC.h"
#include "tut_arrows/Controllers/RISE.h"
#include "tut_arrows/Controllers/ModelFree.h"
#include "tut_arrows/Controllers/ConstantController.h"
#include "tut_arrows/Controllers/SawtoothController.h"

namespace ControllerFactory
{
	std::shared_ptr<IController> getController(const std::string& ns)
	{
		ros::NodeHandle nh(ns);

		std::string controllerType = "";
		nh.param("type", controllerType, controllerType);

		if (controllerType == "PID")
		{
			double Kp = 0.0, Ki = 0.0, Kd = 0.0, windup_limit = 0.0;
			nh.param("Kp", Kp, Kp);
			nh.param("Ki", Ki, Ki);
			nh.param("Kd", Kd, Kd);
			nh.param("windup_limit", windup_limit, windup_limit);
			return std::make_shared<PID>(Kp, Ki, Kd, windup_limit);
		}
		else if (controllerType == "SMC")
		{
			double K = 0.0, lambda = 0.0, slope = 0.0;
			nh.param("K", K, K);
			nh.param("lambda", lambda, lambda);
			nh.param("slope", slope, slope);
			return std::make_shared<SMC>(K, lambda, slope);
		}
		else if (controllerType == "NPD")
		{
			double alfa1 = 0.0, alfa2 = 0.0, delta1 = 0.0, delta2 = 0.0, Kp = 0.0, Ki = 0.0, Kd = 0.0, windup_limit = 0.0;
			nh.param("alfa1", alfa1, alfa1);
			nh.param("alfa2", alfa2, alfa2);
			nh.param("delta1", delta1, delta1);
			nh.param("delta2", delta2, delta2);
			nh.param("Kp", Kp, Kp);
			nh.param("Ki", Ki, Ki);
			nh.param("Kd", Kd, Kd);
			nh.param("windup_limit", windup_limit, windup_limit);

			return std::make_shared<NPD>(alfa1, alfa2, delta1, delta2, Kp, Ki, Kd, windup_limit);
		}
		else if (controllerType == "RISE")
		{
			double ks = 0.0, alpha1 = 0.0, alpha2 = 0.0, beta = 0.0, windup_limit = 0.0;
			nh.param("ks", ks, ks);
			nh.param("alpha1", alpha1, alpha1);
			nh.param("alpha2", alpha2, alpha2);
			nh.param("beta", beta, beta);
			nh.param("windup_limit", windup_limit, windup_limit);

			return std::make_shared<RISE>(ks, alpha1, alpha2, beta, windup_limit);
		}
		else if (controllerType == "ModelFree")
		{
			double alfa = 0.0;
			int v = 0;
			nh.param("alfa", alfa, alfa);
			nh.param("v", v, v);

			std::shared_ptr<IController> secondaryController = getController(ns + "/secondary_controller");

			return std::make_shared<ModelFree>(alfa, v, secondaryController);
		}
		else if (controllerType == "Constant")
		{
			double K = 0.0;
			nh.param("K", K, K);
			return std::make_shared<ConstantController>(K);
		}
		else if (controllerType == "Sawtooth")
		{
			double frequency = 0.0, min = 0.0, max = 0.0;
			nh.param("frequency", frequency, frequency);
			nh.param("min", min, min);
			nh.param("max", max, max);
			return std::make_shared<SawtoothController>(frequency, min, max);
		}
		else
		{
			ROS_INFO("No controller set in %s", ns.c_str());
			return std::shared_ptr<IController>();
		}
	}

	std::shared_ptr<IController> getControllerFromString(const std::string& input)
	{
		std::string controllerType;

		std::stringstream ss(input);
		ss >> controllerType;

		if (controllerType == "PID")
		{
			double Kp = 0.0, Ki = 0.0, Kd = 0.0, windup_limit = 0.0;
			ss >> Kp >> Ki >> Kd >> windup_limit;
			return std::make_shared<PID>(Kp, Ki, Kd, windup_limit);
		}
		else if (controllerType == "NPD")
		{
			double alfa1 = 0.0, alfa2 = 0.0, delta1 = 0.0, delta2 = 0.0, Kp = 0.0, Ki = 0.0, Kd = 0.0, windup_limit = 0.0;
			ss >> alfa1 >> alfa2 >> delta1 >> delta2 >> Kp >> Ki >> Kd >> windup_limit;
			return std::make_shared<NPD>(alfa1, alfa2, delta1, delta2, Kp, Ki, Kd, windup_limit);
		}
		else if (controllerType == "SMC")
		{
			double K = 0.0, lambda = 0.0, slope = 0.0;
			ss >> K >> lambda >> slope;
			return std::make_shared<SMC>(K, lambda, slope);
		}
		else
		{
			return std::shared_ptr<IController>();
		}
	}
}

#endif
