#pragma once
/************************************************
 * @file Stats.h
 * @short helper classes calculating statistics
 ***********************************************/

#include <cmath>
#include <iostream>
#include <mutex>

/**
 *@short Basic statistics
 */

struct XStat {
    double xacc, xacc2, xmin, xmax;
    int xn;
    double factor;
    mutable std::mutex lock;

    XStat(double f = 1);
    void reset();
    void add(double x);
    XStat &operator=(const XStat &o);
    XStat &operator+=(const XStat &o);

    // Linear Regression
    struct Stats {
        int n{0};
        double min_val{0};
        double max_val{0};
        double ave{0};
        double std_dev{0};
    };

    int n() const;
    Stats calcStats() const;
};

std::ostream &operator<<(std::ostream &os, const XStat::Stats &s);

inline XStat::XStat(double f) : factor(f) { reset(); }

inline void XStat::reset() {
    std::lock_guard<std::mutex> l(lock);
    xacc = xacc2 = xmin = xmax = 0;
    xn = 0;
}

inline void XStat::add(double x) {
    x *= factor;
    std::lock_guard<std::mutex> l(lock);
    xacc += x;
    xacc2 += pow(x, 2);
    if (!xn || (x < xmin))
        xmin = x;
    if (!xn || (x > xmax))
        xmax = x;
    ++xn;
}

inline XStat &XStat::operator=(const XStat &o) {
    if (&o == this)
        return *this;

    std::lock_guard<std::mutex> l(o.lock);
    xacc = o.xacc;
    xacc2 = o.xacc2;
    xmin = o.xmin;
    xmax = o.xmax;
    xn = o.xn;
    factor = o.factor;
    return *this;
}

inline XStat &XStat::operator+=(const XStat &o) {
    if (o.factor != factor)
        throw std::runtime_error("Cannot add different XStats");

    std::lock_guard<std::mutex> l(o.lock);
    if (!o.xn)
        return *this;

    xacc += o.xacc;
    xacc2 += o.xacc2;
    if (!xn || (o.xmin < xmin))
        xmin = o.xmin;
    if (!xn || (o.xmax > xmax))
        xmax = o.xmax;
    xn += o.xn;
    return *this;
}

inline int XStat::n() const {
    std::lock_guard<std::mutex> l(lock);
    return xn;
}

inline XStat::Stats XStat::calcStats() const {
    std::lock_guard<std::mutex> l(lock);
    Stats s;
    if (!xn)
        return s;
    s.n = xn;
    s.ave = xacc / xn;
    s.std_dev = std::sqrt(xacc2 / xn - s.ave * s.ave);
    s.min_val = xmin;
    s.max_val = xmax;
    return s;
}

inline std::ostream &operator<<(std::ostream &os, const XStat::Stats &s) {
    os << "<";
    os << "ave=" << s.ave << ", "
       << "std=" << s.std_dev << ", "
       << "min=" << s.min_val << ", "
       << "max=" << s.max_val << ", "
       << "n=" << s.n;
    return os << ">";
}

/**
 *@short X-Y linear regression statistics
 */

struct XYStat {
    double xacc, xacc2, yacc, xyacc;
    int xn;
    double factor;
    mutable std::mutex lock;

    XYStat(double f = 1);
    void reset();
    void add(double x, double y);
    XYStat &operator=(const XYStat &o);
    XYStat &operator+=(const XYStat &o);

    // Linear Regression
    struct LinRegress {
        int n;
        double slope;
        double offset;

        LinRegress() : n(0), slope(0), offset(0) {}
    };

    int n() const;
    LinRegress calcLinRegress() const;
};

std::ostream &operator<<(std::ostream &os, const XYStat::LinRegress &r);

inline XYStat::XYStat(double f) : factor(f) { reset(); }

inline void XYStat::reset() {
    std::lock_guard<std::mutex> l(lock);
    xacc = xacc2 = yacc = xyacc = 0;
    xn = 0;
}

inline void XYStat::add(double x, double y) {
    y *= factor;
    std::lock_guard<std::mutex> l(lock);
    xacc += x;
    xacc2 += pow(x, 2);
    yacc += y;
    xyacc += x * y;
    ++xn;
}

inline XYStat &XYStat::operator=(const XYStat &o) {
    if (&o == this)
        return *this;

    std::lock_guard<std::mutex> l(o.lock);
    xacc = o.xacc;
    xacc2 = o.xacc2;
    yacc = o.yacc;
    xyacc = o.xyacc;
    xn = o.xn;
    factor = o.factor;
    return *this;
}

inline XYStat &XYStat::operator+=(const XYStat &o) {
    if (o.factor != factor)
        throw std::runtime_error("Cannot add different XYStats");

    std::lock_guard<std::mutex> l(o.lock);
    xacc += o.xacc;
    xacc2 += o.xacc2;
    yacc += o.yacc;
    xyacc += o.xyacc;
    xn += o.xn;
    return *this;
}

inline int XYStat::n() const {
    std::lock_guard<std::mutex> l(lock);
    return xn;
}

inline XYStat::LinRegress XYStat::calcLinRegress() const {
    std::lock_guard<std::mutex> l(lock);
    LinRegress r;
    if (!xn)
        return r;
    r.n = xn;
    r.slope = (xn * xyacc - xacc * yacc) / (xn * xacc2 - xacc * xacc);
    r.offset = (yacc - r.slope * xacc) / xn;
    return r;
}

inline std::ostream &operator<<(std::ostream &os, const XYStat::LinRegress &r) {
    os << "<";
    os << "slope=" << r.slope << ", "
       << "offset=" << r.offset << ", "
       << "n=" << r.n;
    return os << ">";
}
