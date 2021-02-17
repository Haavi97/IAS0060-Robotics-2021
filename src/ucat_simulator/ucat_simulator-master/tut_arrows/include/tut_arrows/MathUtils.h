#ifndef TUT_ARROWS_MATH_UTILS_H_
#define TUT_ARROWS_MATH_UTILS_H_

namespace MathUtils
{
	template <class T>
	T limitValue(T value, T min, T max)
	{
		if (value > max)
		{
			return max;
		}
		else if (value < min)
		{
			return min;
		}
		return value;
	}
}

#endif // TUT_ARROWS_MATH_UTILS_H_
