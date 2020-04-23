#pragma once
/************************************************
 * @file Timestamp.h
 * @short helper class for time measurement
 ***********************************************/

#include <sys/time.h>
#include <exception>

namespace FrameAssembler
{

class Timestamp
{
 public:
	Timestamp()
	{ tv.tv_sec = tv.tv_usec = 0; }

	void latchNow()
	{
		if (gettimeofday(&tv, NULL) != 0)
			throw std::runtime_error("Error in gettimeofday");
	}

 private:
	friend double operator -(const Timestamp& t1, const Timestamp& t2);

	struct timeval tv;
};

inline double operator -(const Timestamp& t1, const Timestamp& t2)
{
	return ((t1.tv.tv_sec - t2.tv.tv_sec)
		+ (t1.tv.tv_usec - t2.tv.tv_usec) * 1e-6);
}
 
} // namespace FrameAssembler
