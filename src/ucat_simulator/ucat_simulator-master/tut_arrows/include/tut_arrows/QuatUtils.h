#ifndef TUT_ARROWS_QUAT_UTILS_H_
#define TUT_ARROWS_QUAT_UTILS_H_

#include <cmath>

#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>

namespace QuatUtils
{
	tf::Quaternion fromMsg(const geometry_msgs::Quaternion& quatMsg);

	geometry_msgs::Quaternion toMsg(const tf::Quaternion& quat);

	geometry_msgs::Quaternion slerp(const geometry_msgs::Quaternion& qFinal, const geometry_msgs::Quaternion& qStart, double t);

	double getAngleShortestPath(const tf::Quaternion& q);

	double getAngleShortestPath(const tf::Quaternion& q1, const tf::Quaternion& q2);

	double getSmallestRotation(double target, double source);

	/// Normalize angle to range [-PI ... PI].
	///
	template <class T>
	T normalizeAngle(T angle)
	{
		T x = fmod(angle + M_PI, 2.0 * M_PI);
		if (x < 0)
		{
			x += 2.0 * M_PI;
		}
		return x - M_PI;
	}
}

#endif // TUT_ARROWS_QUAT_UTILS_H_
