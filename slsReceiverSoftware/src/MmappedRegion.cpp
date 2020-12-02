/************************************************
 * @file MmappedRegion.cpp
 * @short helper class for allocating memory with mmap
 ***********************************************/

#include "MmappedRegion.h"

#include <memory.h>
#include <numaif.h>
#include <sys/mman.h>
#include <unistd.h>

using namespace FrameAssembler;

/**
 * MmappedRegion
 */

MmappedRegion::MmappedRegion(size_t size, unsigned long node_mask, int max_node)
    : ptr(NULL), len(0) {
    alloc(size, node_mask, max_node);
}

MmappedRegion::~MmappedRegion() { release(); }

void MmappedRegion::alloc(size_t size, unsigned long node_mask, int max_node) {
    release();
    if (size == 0)
        return;

    size_t page_size = sysconf(_SC_PAGESIZE);
    size_t misalign = size % page_size;
    size += misalign ? (page_size - misalign) : 0;

    ptr = (char *)mmap(0, size, PROT_READ | PROT_WRITE,
                       MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (ptr == NULL)
        throw bad_mmap_alloc("Could not allocate packet memory");
    len = size;

    if (node_mask && max_node) {
        int ret = mbind(ptr, len, MPOL_BIND, &node_mask, max_node, 0);
        if (ret != 0) {
            release();
            throw bad_mmap_alloc("Could not bind packet memory");
        }
    }

    clear();
}

void MmappedRegion::release() {
    if (len) {
        munmap(ptr, len);
        len = 0;
    }
}

void MmappedRegion::clear() { memset(ptr, 0, len); }
