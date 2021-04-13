#pragma once
/************************************************
 * @file PacketContainer.h
 * @short low-level udp packet container classes
 ***********************************************/

#include "sls/logger.h"

#include <condition_variable>
#include <map>
#include <mutex>
#include <queue>
#include <variant>

#include "GeneralData.h"
#include "MmappedRegion.h"
#include "Packet.h"

/**
 *@short container managing packet blocks to/from stream
 */

// P: Packet

template <class P> class PacketContainer {

  public:
    PacketContainer(int frames, unsigned long node_mask, int max_node);
    ~PacketContainer();

    using Packet = P;
    using Block = PacketBlock<Packet>;
    using BlockPtr = PacketBlockPtr<Packet>;
    using BlockLayout = typename Block::Layout;

    using Ptr = std::shared_ptr<PacketContainer>;

    class StreamIface {
      public:
        StreamIface(Ptr c) : pc(c) {}

        void prepare() { pc->prepare(); }

        BlockPtr getFreePacketBlock() { return pc->getFreePacketBlock(); }
        void putReadyPacketBlock(BlockPtr &&block) {
            pc->putReadyPacketBlock(std::move(block));
        }

        void stop() { pc->stop(); }
        void cleanup() { pc->cleanup(); }

      private:
        Ptr pc;
    };

    BlockPtr getReadyPacketBlock(uint64_t frame = uint64_t(-1));

    bool hasPendingPacket();

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
    void putReadyPacketBlock(BlockPtr &&block);
    void releaseReadyPacketBlocks();
    void waitUsedPacketBlocks();

    void stop();
    void cleanup();

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

// TODO: Automatic definition from AnyPacketBlockPtr
using AnyPacketContainer =
    std::variant<PacketContainer<::Eiger::Packet<Pixel4>>,
                 PacketContainer<::Eiger::Packet<Pixel8>>,
                 PacketContainer<::Eiger::Packet<Pixel16>>,
                 PacketContainer<::Eiger::Packet<Pixel32>>,
                 PacketContainer<::Jungfrau::Packet<1>>,
                 PacketContainer<::Jungfrau::Packet<2>>>;

using AnyPacketContainerPtr = std::shared_ptr<AnyPacketContainer>;

template <class P>
typename PacketContainer<P>::Ptr
PacketContainerPtrFromAny(AnyPacketContainerPtr any_pc) {
    return {any_pc, &std::get<PacketContainer<P>>(*any_pc)};
}

AnyPacketContainerPtr
CreatePacketContainer(GeneralDataPtr d, unsigned long node_mask, int max_node);

#include "PacketContainer.cxx"
