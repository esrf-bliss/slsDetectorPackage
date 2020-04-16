#pragma once
/************************************************
 * @file AutoPtr.h
 * @short helper class implementing RAII for allocated objects
 ***********************************************/

#include <algorithm>  // for std::swap - until C++11, then in <utility>

namespace FrameAssembler
{

template <typename T>
class AutoPtr
{
public:
	AutoPtr(T *p = NULL) : ptr(p)
	{}

	~AutoPtr()
	{ delete ptr; }

	AutoPtr& operator =(T *p)
	{
		if (p != ptr) {
			std::swap(ptr, p);
			delete p;
		}
		return *this;
	}

	T *forget()
	{
		T *p = NULL;
		std::swap(ptr, p);
		return p;
	}

	operator T*() const
	{ return ptr; }

	T *operator ->() const
	{ return ptr; }

private:
	AutoPtr& operator =(const AutoPtr& o);

	T *ptr;
};

} // namespace FrameAssembler
