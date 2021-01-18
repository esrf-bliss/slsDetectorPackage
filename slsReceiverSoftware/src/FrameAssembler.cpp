/************************************************
 * @file FrameAssembler.cpp
 * @short helper classes assembling frames
 * from udp packets
 ***********************************************/

#include "FrameAssembler.h"

#include <emmintrin.h>

#include <chrono>
#include <thread>

using namespace FrameAssembler;

using Clock = std::chrono::high_resolution_clock;
using Seconds = std::chrono::duration<double>;

template <class Duration> Seconds ToSeconds(const Duration &d) {
    return std::chrono::duration_cast<Seconds>(d);
}

/**
 * PacketBlock
 */

template <class P> void PacketBlock<P>::setValid(unsigned int i, bool valid) {
    (*this)[i].softHeader()->valid = valid;
    if (valid)
        ++valid_packets;
}

template <class P> void PacketBlock<P>::moveToGood(P &p) {
    P dst = (*this)[p.number()];
    *dst.buffer = *p.buffer;
    p.softHeader()->valid = false;
    dst.softHeader()->valid = true;
}

/**
 * PacketStream
 */

template <class P, class SD, class FP>
PacketBlockPtr<P> PacketStream<P, SD, FP>::getPacketBlock(uint64_t frame) {
    class WaitingCountHelper {
      public:
        WaitingCountHelper(PacketStream &s) : ps(s) {
            ++ps.waiting_reader_count;
        }
        ~WaitingCountHelper() { --ps.waiting_reader_count; }

      private:
        PacketStream &ps;
    };

    BlockPtr block;
    {
        std::unique_lock<std::mutex> l(block_mutex);
        WaitingCountHelper h(*this);
        MapIterator it;
        while (!stopped) {
            if (!packet_block_map.empty()) {
                it = packet_block_map.begin();
                bool too_old = (it->first > frame);
                if (too_old)
                    return nullptr;
            }
            it = packet_block_map.find(frame);
            if (it != packet_block_map.end())
                break;
            block_cond.wait(l);
        }
        if (stopped)
            return nullptr;

        block = std::move(it->second);
        packet_block_map.erase(it);
    }

    bool full_frame = block->hasFullFrame();
    {
        std::lock_guard<std::mutex> l(mutex);
        if (full_frame)
            ++frames_caught;
        if (frame > last_frame)
            last_frame = frame;
    }
    if (!full_frame && FP::canDiscardFrame(block->getValidPackets()))
        block.reset();
    return block;
}

template <class P, class SD, class FP>
bool PacketStream<P, SD, FP>::hasPendingPacket() {
    std::lock_guard<std::mutex> l(free_mutex);
    return (free_queue.size() != num_frames);
}

template <class P, class SD, class FP>
PacketStream<P, SD, FP>::PacketStream(UdpRxSocketPtr s, cpu_set_t cpu_mask,
                                      unsigned long node_mask, int max_node)
    : socket(s), num_frames(MaxBufferFrames), cpu_aff_mask(cpu_mask) {

    packet_buffer_array.alloc(num_frames, node_mask, max_node);
    BlockLayout *p = packet_buffer_array.getPtr();
    for (unsigned int i = 0; i < num_frames; ++i, ++p)
        free_queue.push(p);

    thread = std::make_unique<WriterThread>(*this);
}

template <class P, class SD, class FP>
PacketStream<P, SD, FP>::~PacketStream() {
    stop();
    thread.reset();
    releaseReadyPacketBlocks();
    waitUsedPacketBlocks();
}

template <class P, class SD, class FP>
void PacketStream<P, SD, FP>::releaseReadyPacketBlocks() {
    using namespace std::chrono_literals;
    std::unique_lock<std::mutex> l(block_mutex);
    while (waiting_reader_count > 0)
        block_cond.wait_for(l, 5ms);
    PacketBlockMap old_map = std::move(packet_block_map);
    l.unlock();
    old_map.clear();
}

template <class P, class SD, class FP>
void PacketStream<P, SD, FP>::waitUsedPacketBlocks() {
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
        error << "[" << socket->getPortNumber() << "]: "
              << "Missing free frames after "
              << ToSeconds(wait_reader_timeout).count() << " sec: "
              << "expected " << num_frames << ", "
              << "got " << free_queue.size();
        std::cerr << error.str() << std::endl;
    }
}

template <class P, class SD, class FP>
void PacketStream<P, SD, FP>::printStats() {
    std::lock_guard<std::mutex> l(mutex);
    std::ostringstream msg;
    msg << "[" << socket->getPortNumber() << "]: "
        << "packet_delay_stat=" << packet_delay_stat.calcLinRegress();
    std::cout << msg.str() << std::endl;
}

template <class P, class SD, class FP> void PacketStream<P, SD, FP>::stop() {
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

template <class P, class SD, class FP>
bool PacketStream<P, SD, FP>::wasStopped() {
    std::lock_guard<std::mutex> l(free_mutex);
    return stopped;
}

template <class P, class SD, class FP>
void PacketStream<P, SD, FP>::clearBuffer() {
    packet_buffer_array.clear();
}

template <class P, class SD, class FP>
int PacketStream<P, SD, FP>::getNumPacketsCaught() {
    std::lock_guard<std::mutex> l(mutex);
    return packets_caught;
}

template <class P, class SD, class FP>
uint64_t PacketStream<P, SD, FP>::getNumFramesCaught() {
    std::lock_guard<std::mutex> l(mutex);
    return frames_caught;
}

template <class P, class SD, class FP>
uint64_t PacketStream<P, SD, FP>::getLastFrameIndex() {
    std::lock_guard<std::mutex> l(mutex);
    return last_frame;
}

template <class P, class SD, class FP>
PacketBlockPtr<P> PacketStream<P, SD, FP>::getEmptyBlock() {
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

template <class P, class SD, class FP>
void PacketStream<P, SD, FP>::addPacketBlock(FramePacketBlock &&frame_block) {
    std::lock_guard<std::mutex> l(block_mutex);
    packet_block_map.emplace(std::move(frame_block));
    block_cond.notify_all();
}

template <class P, class SD, class FP>
class PacketStream<P, SD, FP>::WriterThread {
  public:
    WriterThread(PacketStream &s) : ps(s), thread(threadFunctionStatic, this) {
        struct sched_param param;
        param.sched_priority = 90;
        int ret =
            pthread_setschedparam(thread.native_handle(), SCHED_FIFO, &param);
        if (ret != 0)
            std::cerr << "Could not set packet thread RT priority!"
                      << std::endl;
    }

    ~WriterThread() { thread.join(); }

  private:
    bool checkBlock() {
        if (!block)
            block = std::move(ps.getEmptyBlock());
        return bool(block);
    }

    uint32_t getPacketNumber(uint32_t idx) {
        return ps.stream_data.getPacketNumber(idx);
    }

    std::pair<uint32_t, uint32_t> incPacketCounters() {
        curr_packet = getPacketNumber(++curr_idx);
        return {curr_idx, curr_packet};
    }

    P getNextPacket() { return (*block)[incPacketCounters().second]; }

    FramePacketBlock finishPacketBlock() {
        curr_idx = curr_packet = -1;
        return FramePacketBlock(curr_frame, std::move(block));
    }

    void setInvalidPacketsUntil(uint32_t good_packet) {
        // curr_packet validity was already set
        while (incPacketCounters().second != good_packet)
            block->setValid(curr_packet, false);
    }

    void setInvalidRemainingPackets() {
        // curr_packet validity was already set
        while (incPacketCounters().first != ps.FramePackets)
            block->setValid(curr_packet, false);
    }

    void addPacketDelayStat(P &packet, uint32_t index) {
        Clock::time_point t = Clock::now();
        long packet_idx = ((packet.frame() - 1) * ps.FramePackets + index);
        if (packet_idx == 0)
            t0 = t;
        std::lock_guard<std::mutex> l(ps.mutex);
        double sec = ToSeconds(t - t0).count();
        ps.packet_delay_stat.add(packet_idx, sec);
    }

    bool addPacket(P &packet) {
        addPacketDelayStat(packet, curr_idx);

        uint64_t packet_frame = packet.frame();
        uint32_t packet_number = packet.number();

        auto trace_unexpected = [&](auto msg) {
            std::cout << "*** [" << ps.socket->getPortNumber()
                      << "] unexpected " << msg << ": "
                      << "packet_frame=" << packet_frame << ", "
                      << "packet_number=" << packet_number << ", "
                      << "curr_frame=" << curr_frame << ", "
                      << "curr_packet=" << curr_packet << ", "
                      << "curr_idx=" << curr_idx << std::endl;
        };

        // moveToGood manages both src & dst valid flags
        if (curr_idx == 0)
            curr_frame = packet_frame;
        if (packet_frame != curr_frame) {
            trace_unexpected("new frame");
            BlockPtr new_block = ps.getEmptyBlock();
            if (new_block)
                new_block->moveToGood(packet);
            else
                block->setValid(curr_packet, false);
            setInvalidRemainingPackets();
            ps.addPacketBlock(finishPacketBlock());
            if (!new_block)
                return false;
            block = std::move(new_block);
            curr_frame = packet_frame;
            setInvalidPacketsUntil(packet_number);
        } else if (packet_number != curr_packet) {
            trace_unexpected("bad frame");
            block->moveToGood(packet);
            setInvalidPacketsUntil(packet_number);
        } else {
            block->setValid(curr_packet, true);
        }

        if (curr_idx == (ps.FramePackets - 1))
            ps.addPacketBlock(finishPacketBlock());
        return true;
    }

    bool processOnePacket() {
        if (!checkBlock())
            return false;

        P packet = getNextPacket();
        char *b = static_cast<char *>(packet.networkBuffer());
        int ret = ps.socket->ReceiveDataOnly(b);
        if ((ret < 0) || ((packet.frame() == 0) && ps.wasStopped()))
            return false;

        packet.initSoftHeader();

        return addPacket(packet);
    }

    static void threadFunctionStatic(WriterThread *wt) { wt->threadFunction(); }

    void threadFunction() {
        cpu_set_t &cpu_aff_mask = ps.cpu_aff_mask;
        if (CPU_COUNT(&cpu_aff_mask) != 0) {
            int size = sizeof(cpu_aff_mask);
            int ret = sched_setaffinity(0, size, &cpu_aff_mask);
            if (ret != 0)
                std::cerr << "Could not set writer thread "
                          << "cpu affinity mask" << std::endl;
        }

        while (true) {
            if (!processOnePacket())
                break;

            std::lock_guard<std::mutex> l(ps.mutex);
            ++ps.packets_caught;
        }
    }

    PacketStream &ps;
    Clock::time_point t0;
    BlockPtr block;
    uint64_t curr_frame{0};
    uint32_t curr_idx{uint32_t(-1)};
    uint32_t curr_packet{uint32_t(-1)};
    std::thread thread;
};

/**
 * DefaultFrameAssembler
 */

template <class P, class SD, class FP, class SP, class DP>
DefaultFrameAssembler<P, SD, FP, SP, DP>::DefaultFrameAssembler(
    UdpRxSocketPtr s, cpu_set_t cpu_mask, unsigned long node_mask, int max_node)
    : packet_stream(std::make_unique<PacketStream<P, SD, FP>>(
          s, cpu_mask, node_mask, max_node)) {}

template <class P, class SD, class FP, class SP, class DP>
void DefaultFrameAssembler<P, SD, FP, SP, DP>::stop() {
    packet_stream->stop();
}

template <class P, class SD, class FP, class SP, class DP>
bool DefaultFrameAssembler<P, SD, FP, SP, DP>::hasPendingPacket() {
    return packet_stream->hasPendingPacket();
}

template <class P, class SD, class FP, class SP, class DP>
int DefaultFrameAssembler<P, SD, FP, SP, DP>::getImageSize() {
    return P::Data::FrameLen / SP::depth() * DP::depth();
}

template <class P, class SD, class FP, class SP, class DP>
void DefaultFrameAssembler<P, SD, FP, SP, DP>::expand4Bits(char *dst, char *src,
                                                           int src_size) {
    unsigned long s = (unsigned long)src;
    unsigned long d = (unsigned long)dst;
    if ((s & 15) != 0) {
        LOG(logERROR) << "Missaligned source";
        return;
    } else if ((d & 15) != 0) {
        LOG(logERROR) << "Missaligned destination";
        return;
    }

    const int blk = sizeof(__m128i);
    if ((src_size % blk) != 0) {
        LOG(logWARNING) << "len misalignment: "
                        << "src_size=" << src_size << ", "
                        << "blk=" << blk;
    }
    int num_blocks = src_size / blk;
    const __m128i *src128 = (const __m128i *)src;
    __m128i *dst128 = (__m128i *)dst;
    const __m128i mask = _mm_set1_epi8(0xf);
    for (int i = 0; i < num_blocks; ++i) {
        __m128i pack4_raw = _mm_load_si128(src128++);
        __m128i pack4_shr = _mm_srli_epi16(pack4_raw, 4);
        __m128i ilace8_0 = _mm_and_si128(pack4_raw, mask);
        __m128i ilace8_1 = _mm_and_si128(pack4_shr, mask);
        __m128i pack8_0 = _mm_unpacklo_epi8(ilace8_0, ilace8_1);
        __m128i pack8_1 = _mm_unpackhi_epi8(ilace8_0, ilace8_1);
        _mm_store_si128(dst128++, pack8_0);
        _mm_store_si128(dst128++, pack8_1);
    }
}

template <class P, class SD, class FP, class SP, class DP>
Result DefaultFrameAssembler<P, SD, FP, SP, DP>::assembleFrame(
    uint64_t frame, RecvHeader *recv_header, char *buf) {
    bool header_empty = true;
    constexpr int packets_per_frame = Stream::FramePackets;
    DetHeader *det_header = &recv_header->detHeader;
    constexpr uint32_t src_dsize = P::Data::PacketDataLen;
    constexpr uint32_t frame_size = P::Data::FrameLen;
#define check_last(i, p) (((i) % (p)) ? ((i) % (p)) : (p))
    constexpr uint32_t last_dsize = check_last(frame_size, src_dsize);
#undef check_last
    constexpr uint32_t dst_dsize =
        P::Data::PacketDataLen / SP::depth() * DP::depth();

    recv_header->packetsMask.reset();

    using BlockPtr = typename Stream::BlockPtr;
    BlockPtr block = packet_stream->getPacketBlock(frame);
    if (!block || (block->getValidPackets() == 0))
        return Result{1, 0};

    uint32_t prev_adjust = 0;
    for (int i = 0; i < packets_per_frame; ++i) {
        P packet = (*block)[i];
        if (!packet.valid())
            continue;

        int pnum = packet.number();
        recv_header->packetsMask[pnum] = 1;

        // write header
        if (header_empty) {
            packet.fillDetHeader(det_header);
            header_empty = false;
        }

        if (!buf)
            continue;

        // copy packet
        char *dst = buf + pnum * dst_dsize;
        bool last_packet = (pnum == (packets_per_frame - 1));
        uint32_t copy_dsize = last_packet ? last_dsize : src_dsize;
        uint32_t size_adjust = packet.sizeAdjust();
        copy_dsize += size_adjust;
        dst += prev_adjust;
        prev_adjust = size_adjust;
        if (Expand4Bits)
            expand4Bits(dst, packet.data(), copy_dsize);
        else
            memcpy(dst, packet.data(), copy_dsize);
    }

    if (header_empty)
        det_header->frameNumber = frame;
    det_header->packetNumber = block->getValidPackets();

    return Result{1, 1};
}

template <class P, class SD, class FP, class SP, class DP>
int DefaultFrameAssembler<P, SD, FP, SP, DP>::getNumPacketsCaught() {
    return packet_stream->getNumPacketsCaught();
}

template <class P, class SD, class FP, class SP, class DP>
uint64_t DefaultFrameAssembler<P, SD, FP, SP, DP>::getNumFramesCaught() {
    return packet_stream->getNumFramesCaught();
}

template <class P, class SD, class FP, class SP, class DP>
uint64_t DefaultFrameAssembler<P, SD, FP, SP, DP>::getLastFrameIndex() {
    return packet_stream->getLastFrameIndex();
}

template <class P, class SD, class FP, class SP, class DP>
void DefaultFrameAssembler<P, SD, FP, SP, DP>::clearBuffers() {
    packet_stream->clearBuffer();
}

template <class P, class SD, class FP, class SP, class DP>
void DefaultFrameAssembler<P, SD, FP, SP, DP>::printStreamStats() {
    packet_stream->printStats();
}

DefaultFrameAssemblerPtr FrameAssembler::CreateDefaultFrameAssembler(
    UdpRxSocketPtr s, GeneralDataPtr d, int idx, cpu_set_t cpu_mask,
    unsigned long node_mask, int max_node, FramePolicy fp, bool e4b) {

    auto any_fp = AnyFramePolicyFromFP(fp);

    AnyPixel src_pixel = AnyPixelFromBpp(d->dynamicRange);
    AnyPixel dst_pixel = src_pixel;
    if ((d->dynamicRange == 4) && e4b)
        dst_pixel = Pixel8();

    return std::visit(
        [&](auto fp, auto src_pixel, auto dst_pixel) {
            using FP = decltype(fp);
            using SP = decltype(src_pixel);
            using DP = decltype(dst_pixel);

            DefaultFrameAssemblerPtr a;

#define args s, cpu_mask, node_mask, max_node

            if (d->myDetectorType == slsDetectorDefs::EIGER) {
                a = std::make_shared<Eiger::Assembler<SP, FP, DP>>(args);
            } else if (d->myDetectorType == slsDetectorDefs::JUNGFRAU) {
                if (d->numUDPInterfaces == 1)
                    a = std::make_shared<Jungfrau::Assembler<1, 0, FP>>(args);
                else if (idx == 0)
                    a = std::make_shared<Jungfrau::Assembler<2, 0, FP>>(args);
                else
                    a = std::make_shared<Jungfrau::Assembler<2, 1, FP>>(args);

            } else
                throw sls::RuntimeError("Detector not supported: " +
                                        std::to_string(d->myDetectorType));
#undef args

            return a;
        },
        any_fp, src_pixel, dst_pixel);
}

/**
 * RawFrameAssembler
 */

Result RawFrameAssembler::assembleFrame(uint64_t frame, RecvHeader *recv_header,
                                        char *buf) {
    const int NbIfaces = assembler.size();
    Result res{NbIfaces, 0};
    for (int i = 0; i < NbIfaces; ++i) {
        Result r = assembler[i]->assembleFrame(frame, recv_header, buf);
        res.valid_data[i] = r.valid_data[0];
        if (buf)
            buf += assembler[i]->getImageSize();
    }
    return res;
}

#include "FrameAssemblerEiger.cxx"
#include "FrameAssemblerJungfrau.cxx"
