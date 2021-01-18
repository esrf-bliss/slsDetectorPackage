/************************************************
 * @file MmappedRegion.cpp
 * @short helper class for allocating memory with mmap
 ***********************************************/

#include "MmappedRegion.h"

#include <memory.h>
#include <numaif.h>
#include <sys/mman.h>
#include <unistd.h>

namespace FrameAssembler {

/**
 * MmappedRegion
 */

template <typename T>
MmappedRegion<T>::MmappedRegion(size_t size, unsigned long node_mask,
                                int max_node)
    : ptr(NULL), len(0) {
    alloc(size, node_mask, max_node);
}

template <typename T> MmappedRegion<T>::~MmappedRegion() { release(); }

template <typename T>
void MmappedRegion<T>::alloc(size_t size, unsigned long node_mask,
                             int max_node) {
    release();
    if (size == 0)
        return;

    size_t page_size = sysconf(_SC_PAGESIZE);
    len = size * sizeof(T);
    size_t misalign = len % page_size;
    len += misalign ? (page_size - misalign) : 0;

    ptr = static_cast<T *>(mmap(0, len, PROT_READ | PROT_WRITE,
                                MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));
    if (ptr == NULL)
        throw bad_mmap_alloc("Could not allocate packet memory");

    if (node_mask && max_node) {
        int ret = mbind(ptr, len, MPOL_BIND, &node_mask, max_node, 0);
        if (ret != 0) {
            release();
            throw bad_mmap_alloc("Could not bind packet memory");
        }
    }

    clear();
}

template <typename T> void MmappedRegion<T>::release() {
    if (len) {
        munmap(ptr, len);
        len = 0;
    }
}

template <typename T> void MmappedRegion<T>::clear() { memset(ptr, 0, len); }

} // namespace FrameAssembler
