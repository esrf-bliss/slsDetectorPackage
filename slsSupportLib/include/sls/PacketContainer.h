#pragma once
/************************************************
 * @file PacketContainer.h
 * @short low-level udp packet container classes
 ***********************************************/

#include "sls/CPUAffinity.h"
#include "sls/PacketTypedefs.h"
#include "sls/logger.h"

#include <condition_variable>
#include <map>
#include <mutex>
#include <queue>
#include <variant>

#include "sls/MmappedRegion.h"

/**
 *@short container managing packet blocks to/from stream
 */

// P: Packet

template <class P> class PacketContainer {

  public:
    using NUMAMask = sls::CPUAffinity::NUMAMask;

    PacketContainer(int frames, const NUMAMask &numa_mask);
    ~PacketContainer();

    using Packet = P;
    using Block = sls::PacketBlock<Packet>;
    using BlockPtr = sls::PacketBlockPtr<Packet>;
    using BlockLayout = typename Block::Layout;

    BlockPtr getReadyPacketBlock(uint64_t frame = uint64_t(-1));

    unsigned int getPendingPackets();

    void clearBuffers();
    long long getMemorySize();

    void prepare();

    BlockPtr getFreePacketBlock(uint64_t frame);
    void putReadyPacketBlock(BlockPtr block);
    void setMissingFrame(uint64_t frame);

    void stop();
    void cleanUp();

  private:
    using MmappedBlockRegion = MmappedRegion<BlockLayout>;

    using FreeBlockMap = std::vector<BlockLayout *>;
    using ReadyBlockMap = std::map<uint64_t, BlockPtr>;

    unsigned int getBufferIdx(uint64_t frame) {
        return (frame - 1) % num_frames;
    }

    void releaseReadyPacketBlocks();
    void waitUsedPacketBlocks();

    const unsigned int num_frames;
    MmappedBlockRegion packet_buffer_array;
    std::mutex free_mutex;
    std::condition_variable free_cond;
    FreeBlockMap free_map;
    int pending_packets{0};
    std::mutex block_mutex;
    std::condition_variable block_cond;
    ReadyBlockMap ready_block_map;
    int waiting_reader_count{0};
    bool stopped;
};

using AnyPacketContainer =
    std::variant<PacketContainer<sls::EigerPacketPixel4TenGigaDisable>,
                 PacketContainer<sls::EigerPacketPixel4TenGigaEnable>,
                 PacketContainer<sls::EigerPacketPixel8TenGigaDisable>,
                 PacketContainer<sls::EigerPacketPixel8TenGigaEnable>,
                 PacketContainer<sls::EigerPacketPixel16TenGigaDisable>,
                 PacketContainer<sls::EigerPacketPixel16TenGigaEnable>,
                 PacketContainer<sls::EigerPacketPixel32TenGigaDisable>,
                 PacketContainer<sls::EigerPacketPixel32TenGigaEnable>,
                 PacketContainer<sls::JungfrauPacketOneIface>,
                 PacketContainer<sls::JungfrauPacketTwoIface>>;

using AnyPacketContainerPtr = std::shared_ptr<AnyPacketContainer>;

template <class P>
std::shared_ptr<PacketContainer<P>>
PacketContainerPtrFromAny(AnyPacketContainerPtr any_pc) {
    return {any_pc, &std::get<PacketContainer<P>>(*any_pc)};
}

AnyPacketContainerPtr
CreatePacketContainer(slsDetectorDefs::detectorType det_type, bool tg_enable,
                      int num_udp_ifaces, uint32_t dr, int frames,
                      const sls::CPUAffinity::NUMAMask &numa_mask);

#include "PacketContainer.cxx"
