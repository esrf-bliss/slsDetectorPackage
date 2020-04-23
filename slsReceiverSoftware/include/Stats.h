#pragma once
/************************************************
 * @file Stats.h
 * @short helper classes calculating statistics
 ***********************************************/

#include "ThreadUtils.h"
#include <iostream>

namespace FrameAssembler
{

/**
 *@short X-Y linear regression statistics
 */

struct XYStat {
	double xacc, xacc2, yacc, xyacc;
	int xn;
	double factor;
	mutable Mutex lock;

	XYStat(double f = 1);
	void reset();
	void add(double x, double y);
	XYStat& operator =(const XYStat& o);
	XYStat& operator += (const XYStat& o);

	// Linear Regression
	struct LinRegress {
		int n;
		double slope;
		double offset;

		LinRegress() : n(0), slope(0), offset(0)
		{}
	};

	int n() const;
	LinRegress calcLinRegress() const;
};

std::ostream& operator <<(std::ostream& os, const XYStat::LinRegress& r);

inline XYStat::XYStat(double f)
	: factor(f)
{
	reset();
}

inline void XYStat::reset()
{
	MutexLock l(lock);
	xacc = xacc2 = yacc = xyacc = 0;
	xn = 0;
}

inline void XYStat::add(double x, double y) {
	MutexLock l(lock);
	y *= factor;
	xacc += x;
	xacc2 += pow(x, 2);
	yacc += y;
	xyacc += x * y;
	++xn;
}

inline XYStat& XYStat::operator =(const XYStat& o)
{
	if (&o == this)
		return *this;

	MutexLock l(o.lock);
	xacc = o.xacc;
	xacc2 = o.xacc2;
	yacc = o.yacc;
	xyacc = o.xyacc;
	xn = o.xn;
	factor = o.factor;
	return *this;
}

inline XYStat& XYStat::operator +=(const XYStat& o)
{
	if (o.factor != factor)
		throw std::runtime_error("Cannot add different XYStats");

	MutexLock l(o.lock);
	xacc += o.xacc;
	xacc2 += o.xacc2;
	yacc += o.yacc;
	xyacc += o.xyacc;
	xn += o.xn;
	return *this;
}

inline int XYStat::n() const
{ 
	MutexLock l(lock);
	return xn;
}

inline XYStat::LinRegress XYStat::calcLinRegress() const
{
	MutexLock l(lock);
	LinRegress r;
	if (!xn)
		return r;
	r.n = xn;
	r.slope = (xn * xyacc - xacc * yacc) / (xn * xacc2 - xacc * xacc);
	r.offset = (yacc - r.slope * xacc) / xn;
	return r;
}

inline std::ostream& operator <<(std::ostream& os, const XYStat::LinRegress& r)
{
	os << "<";
	os << "slope=" << r.slope << ", "
	   << "offset=" << r.offset << ", "
	   << "n=" << r.n;
	return os << ">";
}

} // namespace FrameAssembler
