/************************************************
 * @file PacketContainer.cxx
 * @short low-level udp packet container classes
 ***********************************************/

#include "PacketContainer.h"

#include <chrono>
#include <thread>

using Clock = std::chrono::high_resolution_clock;
using Seconds = std::chrono::duration<double>;

template <class Duration> Seconds ToSeconds(const Duration &d) {
    return std::chrono::duration_cast<Seconds>(d);
}

/**
 * PacketContainer
 */

template <class P>
PacketContainer<P>::PacketContainer(int frames, unsigned long node_mask,
                                    int max_node)
    : num_frames(frames) {
    packet_buffer_array.alloc(num_frames, node_mask, max_node);
    BlockLayout *p = packet_buffer_array.getPtr();
    for (unsigned int i = 0; i < num_frames; ++i, ++p)
        free_queue.push(p);
}

template <class P> PacketContainer<P>::~PacketContainer() {
    stop();
    cleanup();
}

template <class P> PacketBlockPtr<P> PacketContainer<P>::getFreePacketBlock() {
    auto releaser = [&](BlockLayout *layout) {
        std::lock_guard<std::mutex> l(free_mutex);
        free_queue.push(layout);
        free_cond.notify_one();
    };
    using LayoutPtr = typename Block::LayoutPtr;
    auto allocator = [&]() -> LayoutPtr {
        std::unique_lock<std::mutex> l(free_mutex);
        while (!stopped && free_queue.empty())
            free_cond.wait(l);
        if (stopped)
            return nullptr;
        BlockLayout *layout = free_queue.front();
        free_queue.pop();
        return {layout, releaser};
    };
    auto layout = allocator();
    return layout ? std::make_unique<Block>(std::move(layout)) : nullptr;
}

template <class P>
PacketBlockPtr<P> PacketContainer<P>::getReadyPacketBlock(uint64_t frame) {

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
    MapIterator it;
    bool any = (frame == uint64_t(-1));
    while (!stopped) {
        if (!packet_block_map.empty()) {
            it = packet_block_map.begin();
            if (any)
                break;
            bool too_old = (it->first > frame);
            if (too_old)
                return nullptr;
        }
        if (!any) {
            it = packet_block_map.find(frame);
            if (it != packet_block_map.end())
                break;
        }
        block_cond.wait(l);
    }
    if (stopped)
        return nullptr;

    BlockPtr block = std::move(it->second);
    packet_block_map.erase(it);
    return block;
}

template <class P>
void PacketContainer<P>::putReadyPacketBlock(BlockPtr &&block) {
    std::lock_guard<std::mutex> l(block_mutex);
    packet_block_map.emplace(
        FramePacketBlock(block->frame_number, std::move(block)));
    block_cond.notify_all();
}

template <class P> bool PacketContainer<P>::hasPendingPacket() {
    std::lock_guard<std::mutex> l(free_mutex);
    return (free_queue.size() != num_frames);
}

template <class P> void PacketContainer<P>::releaseReadyPacketBlocks() {
    using namespace std::chrono_literals;
    std::unique_lock<std::mutex> l(block_mutex);
    while (waiting_reader_count > 0)
        block_cond.wait_for(l, 5ms);
    PacketBlockMap old_map = std::move(packet_block_map);
    l.unlock();
    old_map.clear();
}

template <class P> void PacketContainer<P>::waitUsedPacketBlocks() {
    using namespace std::chrono_literals;
    Clock::duration wait_reader_timeout = 1s;
    Clock::time_point t0 = Clock::now();
    while (hasPendingPacket()) {
        Clock::time_point t = Clock::now();
        if (t - t0 > wait_reader_timeout)
            break;
        std::this_thread::sleep_for(5ms);
    }
    if (hasPendingPacket()) {
        std::lock_guard<std::mutex> l(free_mutex);
        std::ostringstream error;
        error << "PacketContainer: Missing free frames after "
              << ToSeconds(wait_reader_timeout).count() << " sec: "
              << "expected " << num_frames << ", "
              << "got " << free_queue.size();
        std::cerr << error.str() << std::endl;
    }
}

template <class P> void PacketContainer<P>::prepare() { stopped = false; }

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

template <class P> void PacketContainer<P>::cleanup() {
    releaseReadyPacketBlocks();
    waitUsedPacketBlocks();
}

template <class P> void PacketContainer<P>::clearBuffers() {
    packet_buffer_array.clear();
}

template <class P> long long PacketContainer<P>::getMemorySize() {
    return packet_buffer_array.getMemorySize();
}

/**
 * PacketContainer factory
 */

template <class P, class... Args> auto PCFactory(Args &&... args) {
    using PC = PacketContainer<P>;
    return std::make_shared<AnyPacketContainer>(std::in_place_type_t<PC>(),
                                                std::forward<Args>(args)...);
}

inline AnyPacketContainerPtr CreatePacketContainer(GeneralDataPtr d, int frames,
                                                   unsigned long node_mask,
                                                   int max_node) {

    auto any_pixel = AnyPixelFromBpp(d->dynamicRange);

    return std::visit(
        [&](auto pixel) {
            using P = decltype(pixel);

#define args frames, node_mask, max_node

            if (d->myDetectorType == slsDetectorDefs::EIGER) {
                if (!d->tgEnable) {
                    const char *error = "10 Giga not enabled!";
                    std::cerr << error << std::endl;
                    throw std::runtime_error(error);
                }
                return PCFactory<::Eiger::Packet<P>>(args);
            } else if (d->myDetectorType == slsDetectorDefs::JUNGFRAU) {
                if (d->numUDPInterfaces == 1)
                    return PCFactory<::Jungfrau::Packet<1>>(args);
                else
                    return PCFactory<::Jungfrau::Packet<2>>(args);
            } else
                throw sls::RuntimeError("Detector not supported: " +
                                        std::to_string(d->myDetectorType));
#undef args
        },
        any_pixel);
}
