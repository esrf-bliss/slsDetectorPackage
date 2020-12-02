#pragma once
/************************************************
 * @file MmappedRegion.h
 * @short helper class for allocating memory with mmap
 ***********************************************/

#include <memory>
#include <string>

namespace FrameAssembler {

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

class MmappedRegion {
  public:
    MmappedRegion(size_t size = 0, unsigned long node_mask = 0,
                  int max_node = 0);
    ~MmappedRegion();

    void alloc(size_t size, unsigned long node_mask = 0, int max_node = 0);
    void release();
    char *getPtr() { return ptr; }

    void clear();

  private:
    char *ptr;
    size_t len;
};

} // namespace FrameAssembler
