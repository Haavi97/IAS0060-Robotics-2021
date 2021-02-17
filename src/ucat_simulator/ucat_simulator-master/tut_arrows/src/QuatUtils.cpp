#include "tut_arrows/QuatUtils.h"

tf::Quaternion QuatUtils::fromMsg(const geometry_msgs::Quaternion& quatMsg)
{
	return tf::Quaternion(quatMsg.x, quatMsg.y, quatMsg.z, quatMsg.w);
}

geometry_msgs::Quaternion QuatUtils::toMsg(const tf::Quaternion& quat)
{
	geometry_msgs::Quaternion q;
	q.x = quat.x();
	q.y = quat.y();
	q.z = quat.z();
	q.w = quat.w();
	return q;
}

geometry_msgs::Quaternion QuatUtils::slerp(const geometry_msgs::Quaternion& qFinal, const geometry_msgs::Quaternion& qStart, double t)
{
	tf::Quaternion q0 = fromMsg(qStart);
	tf::Quaternion q1 = fromMsg(qFinal);

	tf::Quaternion qDesired = q0.slerp(q1, t);

	return toMsg(qDesired);
}

/// get shortest path angle in range [-PI, PI]
///
double QuatUtils::getAngleShortestPath(const tf::Quaternion& q)
{
	return q.getAngleShortestPath() - M_PI;
}

/// get shortest path angle between to quaternions in range [-PI, PI]
///
double QuatUtils::getAngleShortestPath(const tf::Quaternion& q1, const tf::Quaternion& q2)
{
	return q1.angleShortestPath(q2) - M_PI;
}

double QuatUtils::getSmallestRotation(double target, double source)
{
	double diff = target - source;
	if (diff > M_PI)
	{
		diff -= 2.0 * M_PI;
	}
	else if (diff < -M_PI)
	{
		diff += 2.0 * M_PI;
	}
	return diff;
}

