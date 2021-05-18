#pragma once
/************************************************
 * @file PacketContainer.h
 * @short low-level udp packet container classes
 ***********************************************/

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
    PacketContainer(int frames, unsigned long node_mask, int max_node);
    ~PacketContainer();

    using Packet = P;
    using Block = sls::PacketBlock<Packet>;
    using BlockPtr = sls::PacketBlockPtr<Packet>;
    using BlockLayout = typename Block::Layout;

    using Ptr = std::shared_ptr<PacketContainer>;

    class StreamIface {
      public:
        StreamIface(Ptr c) : pc(c) {}

        void prepare() { pc->prepare(); }

        BlockPtr getFreePacketBlock() { return pc->getFreePacketBlock(); }
        void putReadyPacketBlock(BlockPtr block) {
            pc->putReadyPacketBlock(std::move(block));
        }

        void stop() { pc->stop(); }
        void cleanUp() { pc->cleanUp(); }

      private:
        Ptr pc;
    };

    BlockPtr getReadyPacketBlock(uint64_t frame = uint64_t(-1));

    unsigned int getPendingPackets();

    void clearBuffers();
    long long getMemorySize();

  private:
    friend class StreamIface;

    using MmappedBlockRegion = MmappedRegion<BlockLayout>;

    using PacketBlockMap = std::map<uint64_t, BlockPtr>;
    using MapIterator = typename PacketBlockMap::iterator;
    using FramePacketBlock = typename PacketBlockMap::value_type;

    void prepare();

    BlockPtr getFreePacketBlock();
    void putReadyPacketBlock(BlockPtr block);
    void releaseReadyPacketBlocks();
    void waitUsedPacketBlocks();

    void stop();
    void cleanUp();

    const unsigned int num_frames;
    MmappedBlockRegion packet_buffer_array;
    std::mutex free_mutex;
    std::condition_variable free_cond;
    std::queue<BlockLayout *> free_queue;
    std::mutex block_mutex;
    std::condition_variable block_cond;
    PacketBlockMap packet_block_map;
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
typename PacketContainer<P>::Ptr
PacketContainerPtrFromAny(AnyPacketContainerPtr any_pc) {
    return {any_pc, &std::get<PacketContainer<P>>(*any_pc)};
}

AnyPacketContainerPtr
CreatePacketContainer(slsDetectorDefs::detectorType det_type, bool tg_enable,
                      int num_udp_ifaces, uint32_t dr, int frames,
                      unsigned long node_mask, int max_node);

#include "PacketContainer.cxx"
