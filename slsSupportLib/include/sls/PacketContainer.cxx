/************************************************
 * @file PacketContainer.cxx
 * @short low-level udp packet container classes
 ***********************************************/

#include <cassert>
#include <chrono>
#include <thread>

using namespace sls::Geom;

using Clock = std::chrono::high_resolution_clock;
using Seconds = std::chrono::duration<double>;

template <class Duration> Seconds ToSeconds(const Duration &d) {
    return std::chrono::duration_cast<Seconds>(d);
}

/**
 * PacketContainer
 */

template <class P>
PacketContainer<P>::PacketContainer(int frames, PacketBlockAllocPtr alloc_ptr)
    : num_frames(frames), block_alloc_ptr(alloc_ptr),
      free_map(num_frames, nullptr) {
    block_alloc_ptr->alloc(sizeof(BlockLayout), num_frames);
    for (unsigned int i = 0; i < num_frames; ++i)
        free_map[i] =
            static_cast<BlockLayout *>(block_alloc_ptr->getItemPtr(i));
}

template <class P> PacketContainer<P>::~PacketContainer() {
    stop();
    cleanUp();
    waitUsedPacketBlocks();
    block_alloc_ptr->release();
}

template <class P>
sls::PacketBlockPtr<P> PacketContainer<P>::getFreePacketBlock(uint64_t frame) {
    auto idx = getBufferIdx(frame);
    auto releaser = [&, idx = idx](BlockLayout *layout) {
        std::lock_guard<std::mutex> l(free_mutex);
        std::swap(layout, free_map[idx]);
        --pending_packets;
        free_cond.notify_one();
    };
    using LayoutPtr = typename Block::LayoutPtr;
    auto allocator = [&]() -> LayoutPtr {
        std::unique_lock<std::mutex> l(free_mutex);
        while (!stopped && !free_map[idx])
            free_cond.wait(l);
        if (stopped)
            return nullptr;
        BlockLayout *layout = nullptr;
        std::swap(layout, free_map[idx]);
        ++pending_packets;
        return {layout, releaser};
    };
    auto layout = allocator();
    return layout ? std::make_unique<Block>(std::move(layout)) : nullptr;
}

template <class P>
sls::PacketBlockPtr<P> PacketContainer<P>::getReadyPacketBlock(uint64_t frame) {

    class WaitingCountHelper {
      public:
        WaitingCountHelper(PacketContainer &c) : pc(c) {
            ++pc.waiting_reader_count;
        }
        ~WaitingCountHelper() { --pc.waiting_reader_count; }

      private:
        PacketContainer &pc;
    };

    std::unique_lock<std::mutex> l(block_mutex);
    WaitingCountHelper h(*this);
    typename ReadyBlockMap::iterator it;
    bool any = (frame == uint64_t(-1));
    while (!stopped) {
        if (!ready_block_map.empty()) {
            it = ready_block_map.begin();
            if (any)
                break;
            bool too_old = (it->first > frame);
            if (too_old)
                return nullptr;
            it = ready_block_map.find(frame);
            if (it != ready_block_map.end())
                break;
        }
        block_cond.wait(l);
    }
    if (stopped)
        return nullptr;

    BlockPtr block = std::move(it->second);
    ready_block_map.erase(it);
    return block;
}

template <class P>
void PacketContainer<P>::putReadyPacketBlock(BlockPtr block) {
    using FramePacketBlock = typename ReadyBlockMap::value_type;
    std::lock_guard<std::mutex> l(block_mutex);
    ready_block_map.emplace(
        FramePacketBlock(block->getRecvFrameNumber(), std::move(block)));
    block_cond.notify_all();
}

template <class P> void PacketContainer<P>::setMissingFrame(uint64_t frame) {
    using FramePacketBlock = typename ReadyBlockMap::value_type;
    std::lock_guard<std::mutex> l(block_mutex);
    ready_block_map.emplace(FramePacketBlock(frame, nullptr));
    block_cond.notify_all();
}

template <class P> unsigned int PacketContainer<P>::getPendingPackets() {
    std::lock_guard<std::mutex> l(free_mutex);
    return pending_packets;
}

template <class P> void PacketContainer<P>::releaseReadyPacketBlocks() {
    using namespace std::chrono_literals;
    std::unique_lock<std::mutex> l(block_mutex);
    while (waiting_reader_count > 0)
        block_cond.wait_for(l, 5ms);
    ReadyBlockMap old_map = std::move(ready_block_map);
    assert(ready_block_map.empty());
    l.unlock();
}

template <class P> void PacketContainer<P>::waitUsedPacketBlocks() {
    using namespace std::chrono_literals;
    Clock::duration wait_reader_timeout = 10s;
    Clock::time_point t0 = Clock::now();
    while (getPendingPackets() > 0) {
        Clock::time_point t = Clock::now();
        if (t - t0 > wait_reader_timeout)
            break;
        std::this_thread::sleep_for(100ms);
    }
    auto missing = getPendingPackets();
    if (missing > 0) {
        std::lock_guard<std::mutex> l(free_mutex);
        std::ostringstream error;
        error << "PacketContainer: Missing " << missing << " free frames "
              << "after " << ToSeconds(wait_reader_timeout).count() << " sec";
        std::cerr << error.str() << std::endl;
    }
}

template <class P> void PacketContainer<P>::prepare() {
    waitUsedPacketBlocks();
    stopped = false;
}

template <class P> void PacketContainer<P>::stop() {
    stopped = true;
    {
        std::lock_guard<std::mutex> l(block_mutex);
        block_cond.notify_all();
    }
    {
        std::lock_guard<std::mutex> l(free_mutex);
        free_cond.notify_one();
    }
}

template <class P> void PacketContainer<P>::cleanUp() {
    releaseReadyPacketBlocks();
}

template <class P> void PacketContainer<P>::clearBuffers() {
    block_alloc_ptr->clear();
}

template <class P> long long PacketContainer<P>::getMemorySize() {
    return block_alloc_ptr->getMemorySize();
}
