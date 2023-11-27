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
#include "sls/PacketBlockAllocator.h"

/**
 *@short default packet block allocator
 */

class MmappedPacketAllocator : public PacketBlockAllocator {

  public:
    using NUMAMask = sls::CPUAffinity::NUMAMask;

    MmappedPacketAllocator(const NUMAMask &numa_mask = {});

    void alloc(std::size_t item_size, std::size_t nb_items) override;
    void release() override;

    std::size_t getNbItems() override { return nb_blocks; }
    void *getItemPtr(std::size_t idx) override;

    void clear() override { block_array.clear(); }

    long long getMemorySize() override { return block_array.getMemorySize(); }

  protected:
    MmappedRegion<char> block_array;
    NUMAMask block_numa_mask;
    std::size_t block_size{0};
    std::size_t nb_blocks{0};
};

inline void *MmappedPacketAllocator::getItemPtr(std::size_t idx) {
    if (idx >= nb_blocks)
        throw std::out_of_range("MmappedPacketAllocator: index out of range: " +
                                std::to_string(idx) +
                                " (max=" + std::to_string(nb_blocks - 1) + ")");
    return block_array.getPtr() + block_size * idx;
}

/**
 *@short container managing packet blocks to/from stream
 */

// P: Packet

template <class P> class PacketContainer {

  public:
    PacketContainer(int frames, PacketBlockAllocPtr alloc_ptr);
    ~PacketContainer();

    using Packet = P;
    using Block = sls::PacketBlock<Packet>;
    using BlockPtr = sls::PacketBlockPtr<Packet>;
    using BlockLayout = typename Block::Layout;

    uint64_t getNextReadyFrameNumber();
    BlockPtr getReadyPacketBlock(uint64_t frame = uint64_t(-1));

    unsigned int getPendingPackets();

    void clearBuffers();
    long long getMemorySize();

    void prepare();

    BlockPtr getFreePacketBlock(uint64_t frame);
    void putReadyPacketBlock(BlockPtr block);

    void stop();
    void cleanUp();

  private:
    using FreeBlockMap = std::vector<BlockLayout *>;
    using ReadyBlockMap = std::map<uint64_t, BlockPtr>;

    unsigned int getBufferIdx(uint64_t frame) {
        return (frame - 1) % num_frames;
    }

    class WaitingCountHelper {
      public:
        using Lock = std::unique_lock<std::mutex>;

        WaitingCountHelper(PacketContainer &c, Lock &l) : pc(c) {
            if (!l)
                throw std::runtime_error("WaitingCounterHelper without lock");
            ++pc.waiting_reader_count;
        }
        ~WaitingCountHelper() { --pc.waiting_reader_count; }

      private:
        PacketContainer &pc;
    };

    void releaseReadyPacketBlocks();
    void waitUsedPacketBlocks();

    const unsigned int num_frames;
    PacketBlockAllocPtr block_alloc_ptr;
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
                      PacketBlockAllocPtr alloc_ptr);

#include "PacketContainer.cxx"
