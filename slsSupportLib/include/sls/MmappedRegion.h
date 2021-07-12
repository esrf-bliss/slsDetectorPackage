#pragma once
/************************************************
 * @file MmappedRegion.h
 * @short helper class for allocating memory with mmap
 ***********************************************/

#include <memory>
#include <string>

/**
 *@short MmappedRegion
 */

class bad_mmap_alloc : public std::bad_alloc {
  public:
    bad_mmap_alloc(const char *m = "") : msg(m) {}
    virtual ~bad_mmap_alloc() throw() {}
    virtual const char *what() const throw() { return msg.c_str(); }

  private:
    std::string msg;
};

template <typename T> class MmappedRegion {
  public:
    MmappedRegion(size_t size = 0, const unsigned long *node_mask = nullptr,
                  int max_node = 0);
    ~MmappedRegion();

    void alloc(size_t size, const unsigned long *node_mask = nullptr,
               int max_node = 0);
    void release();
    T *getPtr() { return ptr; }

    void clear();

    long long getMemorySize();

  private:
    T *ptr;
    size_t len;
};

#include "MmappedRegion.cxx"
