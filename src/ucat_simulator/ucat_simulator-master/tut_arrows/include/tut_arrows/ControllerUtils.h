#ifndef TUT_ARROWS_CONTROLLER_UTILS_H_
#define TUT_ARROWS_CONTROLLER_UTILS_H_

#include <cmath>
#include <vector>

namespace ControllerUtils
{
	template<class T>
	class RingBuffer
	{
	public:
		RingBuffer(size_t size) : size_(size), data_(size), currentIndex_(0), count_(0)
		{ }

		const T& head(int offset = 0) const
		{
			const int index = (currentIndex_ + (size_ + (offset % size_))) % size_;
			return data_[index];
		}

		void push(const T& element)
		{
			currentIndex_ = (currentIndex_ + 1) % size_;
			data_[currentIndex_] = element;

			if (count_ < size_)
			{
				count_++;
			}
		}

		bool isFull() const
		{
			return count_ == size_;
		}

	private:
		const int size_;
		std::vector<T> data_;
		int currentIndex_;
		int count_;
	};

	double get1stDerivative(const RingBuffer<double>& values, double dt)
	{
		return ( values.head(0) - values.head(-1) ) / dt;
	}

	double get2ndDerivative(const RingBuffer<double>& values, double dt)
	{
		double d1 = ( values.head() - values.head(-1) ) / dt;
		double d2 = ( values.head(-1) - values.head(-2) ) / dt;
		return ( d1 - d2 ) / dt;
	}

	double getDerivative(const RingBuffer<double>& values, double dt, int order)
	{
		if (!values.isFull())
		{
			return 0.0;
		}

		switch (order)
		{
		case 1:
			return get1stDerivative(values, dt);
		case 2:
			return get2ndDerivative(values, dt);
		}
		return 0.0;
	}

	double sign(double x)
	{
		if (x < 0)
		{
			return -1.0;
		}
		else if (x > 0)
		{
			return 1.0;
		}
		return 0.0;
	}

	double sigmoid(double x, double gain)
	{
		return 1.0 / (1.0 + std::exp(-gain*x));
	}
}

#endif // TUT_ARROWS_CONTROLLER_UTILS_H_
