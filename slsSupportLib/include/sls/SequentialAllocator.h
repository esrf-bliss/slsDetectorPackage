#pragma once
/************************************************
 * @file SequentialAllocator.h
 * @short abstract class allocating memory sequentially
 ***********************************************/

/**
 *@short SequentialAllocator
 */

class SequentialAllocator {
  public:
    virtual ~SequentialAllocator() {}

    virtual void alloc(std::size_t item_size, std::size_t nb_items) = 0;
    virtual void release() = 0;

    virtual std::size_t getNbItems() = 0;
    virtual void *getItemPtr(std::size_t idx) = 0;

    virtual void clear() = 0;

    virtual long long getMemorySize() = 0;
};
