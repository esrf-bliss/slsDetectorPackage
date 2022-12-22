#pragma once
/************************************************
 * @file PacketBlockAllocator
 * @short High-level API for PacketBlock allocation
 ***********************************************/

#include <memory>
#include <vector>

#include "sls/SequentialAllocator.h"

using PacketBlockAllocator = SequentialAllocator;
using PacketBlockAllocPtr = std::shared_ptr<PacketBlockAllocator>;
using PacketBlockAllocList = std::vector<PacketBlockAllocPtr>;
