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
 * StdPacketImpl
 */

template <class PD> void Packet<PD>::checkConsistency(GeneralDataPtr gd) {}

/**
 * StdPacketImpl
 */

template <class PD>
void StdPacketImpl<PD>::fillDetHeader(DetHeader *det_header) {
    memcpy(det_header, header(), sizeof(*det_header));
}

/**
 * LegacyPacket
 */

template <class F> void LegacyPacketImpl<F>::initSoftHeader() {
    int thread_idx = 0;
    uint64_t packet_bid;
    generalData()->GetHeaderInfo(thread_idx, Base::networkHeader(),
                                 stream_data.odd_numbering,
                                 Base::softHeader()->packet_frame,
                                 Base::softHeader()->packet_number, packet_bid);
}

template <class F>
void LegacyPacketImpl<F>::fillDetHeader(DetHeader *det_header) {
    memset(det_header, 0, sizeof(*det_header));
    det_header->frameNumber = frame();
    det_header->detType = (uint8_t)generalData()->myDetectorType;
    det_header->version = (uint8_t)SLS_DETECTOR_HEADER_VERSION;
}

/**
 * GotthardPacket
 */

inline void GotthardPacket::initSoftHeader() {
    if (stream_data.first_packet) {
        bool &odd_numbering = stream_data.odd_numbering;
        GeneralDataPtr gd = generalData();
        int thread_idx = 0;
        char *start = networkHeader();
        odd_numbering = gd->SetOddStartingPacket(thread_idx, start);
        stream_data.first_packet = false;
    }
    Base::initSoftHeader();
}

/**
 * PacketBlock
 */

template <class P>
PacketBlock<P>::PacketBlock(PacketStream<P> &s, char *b) : ps(s), buf(b) {}

template <class P> PacketBlock<P>::~PacketBlock() {
    ps.releasePacketBlock(*this);
}

template <class P> P PacketBlock<P>::operator[](unsigned int i) {
    return P(buf + ps.packet_len * i, ps.stream_data);
}

template <class P> void PacketBlock<P>::setValid(unsigned int i, bool valid) {
    (*this)[i].softHeader()->valid = valid;
    if (valid)
        ++valid_packets;
}

template <class P> void PacketBlock<P>::moveToGood(P &p) {
    P dst = (*this)[p.number()];
    using SoftHeader = typename P::SoftHeader;
    int len = sizeof(SoftHeader) + ps.general_data->packetSize;
    memcpy(dst.buffer, p.buffer, len);
    p.softHeader()->valid = false;
    dst.softHeader()->valid = true;
}

/**
 * PacketStream
 */

const int MaxBufferFrames = 4;

template <class P>
void PacketStream<P>::releasePacketBlock(PacketBlock<P> &block) {
    std::lock_guard<std::mutex> l(free_mutex);
    free_queue.push(block.buf);
    free_cond.notify_one();
}

template <class P> bool PacketStream<P>::canDiscardFrame(int received_packets) {
    return ((frame_policy == slsDetectorDefs::DISCARD_PARTIAL_FRAMES) ||
            ((frame_policy == slsDetectorDefs::DISCARD_EMPTY_FRAMES) &&
             !received_packets));
}

template <class P>
PacketBlockPtr<P> PacketStream<P>::getPacketBlock(uint64_t frame) {
    class WaitingCountHelper {
      public:
        WaitingCountHelper(PacketStream &s) : ps(s) {
            ++ps.waiting_reader_count;
        }
        ~WaitingCountHelper() { --ps.waiting_reader_count; }

      private:
        PacketStream &ps;
    };

    PacketBlockPtr<P> block;
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
    if (!full_frame && canDiscardFrame(block->getValidPackets()))
        block.reset();
    return block;
}

template <class P> bool PacketStream<P>::hasPendingPacket() {
    std::lock_guard<std::mutex> l(free_mutex);
    return (free_queue.size() != num_frames);
}

template <class P>
PacketStream<P>::PacketStream(UdpRxSocketPtr s, GeneralDataPtr d,
                              FramePolicy fp, cpu_set_t cpu_mask,
                              unsigned long node_mask, int max_node)
    : socket(s), general_data(d), frame_policy(fp), num_frames(MaxBufferFrames),
      stream_data(general_data), cpu_aff_mask(cpu_mask) {
    P::checkConsistency(general_data);
    initMem(node_mask, max_node);
    thread = std::make_unique<WriterThread>(*this);
}

template <class P> PacketStream<P>::~PacketStream() {
    stop();
    thread.reset();
    releaseReadyPacketBlocks();
    waitUsedPacketBlocks();
}

template <class P> void PacketStream<P>::releaseReadyPacketBlocks() {
    using namespace std::chrono_literals;
    std::unique_lock<std::mutex> l(block_mutex);
    while (waiting_reader_count > 0)
        block_cond.wait_for(l, 5ms);
    PacketBlockMap old_map = std::move(packet_block_map);
    l.unlock();
    old_map.clear();
}

template <class P> void PacketStream<P>::waitUsedPacketBlocks() {
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

template <class P> void PacketStream<P>::printStats() {
    std::lock_guard<std::mutex> l(mutex);
    std::ostringstream msg;
    msg << "[" << socket->getPortNumber() << "]: "
        << "packet_delay_stat=" << packet_delay_stat.calcLinRegress();
    std::cout << msg.str() << std::endl;
}

template <class P> void PacketStream<P>::stop() {
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

template <class P> void PacketStream<P>::clearBuffer() {
    packet_buffer_array.clear();
}

template <class P> int PacketStream<P>::getNumPacketsCaught() {
    std::lock_guard<std::mutex> l(mutex);
    return packets_caught;
}

template <class P> uint64_t PacketStream<P>::getNumFramesCaught() {
    std::lock_guard<std::mutex> l(mutex);
    return frames_caught;
}

template <class P> uint64_t PacketStream<P>::getLastFrameIndex() {
    std::lock_guard<std::mutex> l(mutex);
    return last_frame;
}

template <class P> uint32_t PacketStream<P>::getNbPacketFrames() {
    return general_data->packetsPerFrame;
}

template <class P>
void PacketStream<P>::initMem(unsigned long node_mask, int max_node) {
    const int data_align = 128 / 8;
    GeneralDataPtr gd = general_data;
    packet_len = gd->packetSize;
    using SoftHeader = typename P::SoftHeader;
    size_t header_len = sizeof(SoftHeader) + gd->headerSizeinPacket;
    size_t misalign = header_len % data_align;
    header_pad = misalign ? (data_align - misalign) : 0;
    packet_len += header_pad;
    misalign = packet_len % data_align;
    packet_len += misalign ? (data_align - misalign) : 0;
    int frame_len = packet_len * getNbPacketFrames();
    packet_buffer_array.alloc(frame_len * num_frames, node_mask, max_node);

    char *p = packet_buffer_array.getPtr() + header_pad;
    for (unsigned int i = 0; i < num_frames; ++i, p += frame_len)
        free_queue.push(p);
}

template <class P> PacketBlockPtr<P> PacketStream<P>::getEmptyBlock() {
    std::unique_lock<std::mutex> l(free_mutex);
    while (!stopped && free_queue.empty())
        free_cond.wait(l);
    if (stopped)
        return nullptr;
    char *b = free_queue.front();
    free_queue.pop();
    l.unlock();
    return std::make_unique<PacketBlock<P>>(*this, b);
}

template <class P>
void PacketStream<P>::addPacketBlock(FramePacketBlock &&frame_block) {
    std::lock_guard<std::mutex> l(block_mutex);
    packet_block_map.emplace(std::move(frame_block));
    block_cond.notify_all();
}

template <class P> class PacketStream<P>::WriterThread {
  public:
    WriterThread(PacketStream &s)
        : ps(s), frame_packets(ps.getNbPacketFrames()),
          thread(threadFunctionStatic, this) {
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

    P getNextPacket() {
        return (*block)[curr_packet = getPacketNumber(++curr_idx)];
    }

    FramePacketBlock finishPacketBlock() {
        curr_idx = curr_packet = -1;
        return FramePacketBlock(curr_frame, std::move(block));
    }

    void setInvalidPacketsUntil(uint32_t good_packet) {
        // curr_packet validity was already set
        while ((curr_packet = getPacketNumber(++curr_idx)) != good_packet)
            block->setValid(curr_packet, false);
    }

    void addPacketDelayStat(P &packet) {
        Clock::time_point t = Clock::now();
        long packet_idx =
            ((packet.frame() - 1) * frame_packets + packet.index());
        if (packet_idx == 0)
            t0 = t;
        std::lock_guard<std::mutex> l(ps.mutex);
        double sec = ToSeconds(t - t0).count();
        ps.packet_delay_stat.add(packet_idx, sec);
    }

    bool addPacket(P &packet) {
        packet.setIndex(curr_idx);
        addPacketDelayStat(packet);

        uint64_t packet_frame = packet.frame();
        uint32_t packet_number = packet.number();
        // moveToGood manages both src & dst valid flags
        if (curr_idx == 0)
            curr_frame = packet_frame;
        if (packet_frame != curr_frame) {
            PacketBlockPtr<P> new_block = ps.getEmptyBlock();
            if (new_block)
                new_block->moveToGood(packet);
            else
                block->setValid(curr_packet, false);
            setInvalidPacketsUntil(frame_packets);
            ps.addPacketBlock(finishPacketBlock());
            if (!new_block)
                return false;
            block = std::move(new_block);
            curr_frame = packet_frame;
            setInvalidPacketsUntil(packet_number);
        } else if (packet_number != curr_packet) {
            block->moveToGood(packet);
            setInvalidPacketsUntil(packet_number);
        } else {
            block->setValid(curr_packet, true);
        }

        if (curr_idx == (frame_packets - 1))
            ps.addPacketBlock(finishPacketBlock());
        return true;
    }

    bool processOnePacket() {
        if (!checkBlock())
            return false;

        P packet = getNextPacket();
        char *b = packet.networkBuffer();
        int ret = ps.socket->ReceiveDataOnly(b);
        if (ret < 0)
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
    const uint32_t frame_packets;
    Clock::time_point t0;
    PacketBlockPtr<P> block;
    uint64_t curr_frame{0};
    uint32_t curr_idx{uint32_t(-1)};
    uint32_t curr_packet{uint32_t(-1)};
    std::thread thread;
};

/**
 * DefaultFrameAssemblerBase
 */

DefaultFrameAssemblerBase::DefaultFrameAssemblerBase(GeneralDataPtr d, bool e4b)
    : general_data(d), expand_4bits(e4b) {}

inline GeneralDataPtr DefaultFrameAssemblerBase::getGeneralData() {
    return general_data;
}

inline bool DefaultFrameAssemblerBase::doExpand4Bits() {
    return (general_data->dynamicRange == 4) && expand_4bits;
}

/**
 * DefaultFrameAssembler
 */

template <class P>
DefaultFrameAssembler<P>::DefaultFrameAssembler(
    UdpRxSocketPtr s, GeneralDataPtr d, cpu_set_t cpu_mask,
    unsigned long node_mask, int max_node, FramePolicy fp, bool e4b)
    : DefaultFrameAssemblerBase(d, e4b),
      packet_stream(std::make_unique<PacketStream<P>>(s, d, fp, cpu_mask,
                                                      node_mask, max_node)),
      frame_policy(fp) {}

template <class P> void DefaultFrameAssembler<P>::stop() {
    packet_stream->stop();
}

template <class P> bool DefaultFrameAssembler<P>::hasPendingPacket() {
    return packet_stream->hasPendingPacket();
}

template <class P> int DefaultFrameAssembler<P>::getImageSize() {
    return general_data->imageSize * (doExpand4Bits() ? 2 : 1);
}

template <class P>
void DefaultFrameAssembler<P>::expand4Bits(char *dst, char *src, int src_size) {
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

template <class P>
Result DefaultFrameAssembler<P>::assembleFrame(uint64_t frame,
                                               RecvHeader *recv_header,
                                               char *buf) {
    bool header_empty = true;
    int packets_per_frame = general_data->packetsPerFrame;
    DetHeader *det_header = &recv_header->detHeader;
    bool do_expand_4bits = doExpand4Bits();
    uint32_t src_dsize = general_data->dataSize;
    uint32_t last_dsize = general_data->imageSize % src_dsize;
    if (last_dsize == 0)
        last_dsize = src_dsize;
    uint32_t dst_dsize = src_dsize * (do_expand_4bits ? 2 : 1);

    recv_header->packetsMask.reset();

    PacketBlockPtr<P> block = packet_stream->getPacketBlock(frame);
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
        if (do_expand_4bits)
            expand4Bits(dst, packet.data(), copy_dsize);
        else
            memcpy(dst, packet.data(), copy_dsize);
    }

    if (header_empty)
        det_header->frameNumber = frame;
    det_header->packetNumber = block->getValidPackets();

    return Result{1, 1};
}

template <class P> int DefaultFrameAssembler<P>::getNumPacketsCaught() {
    return packet_stream->getNumPacketsCaught();
}

template <class P> uint64_t DefaultFrameAssembler<P>::getNumFramesCaught() {
    return packet_stream->getNumFramesCaught();
}

template <class P> uint64_t DefaultFrameAssembler<P>::getLastFrameIndex() {
    return packet_stream->getLastFrameIndex();
}

template <class P> void DefaultFrameAssembler<P>::clearBuffers() {
    packet_stream->clearBuffer();
}

template <class P> void DefaultFrameAssembler<P>::printStreamStats() {
    packet_stream->printStats();
}

DefaultFrameAssemblerBase::Ptr
DefaultFrameAssemblerBase::create(UdpRxSocketPtr s, GeneralDataPtr d,
                                  cpu_set_t cpu_mask, unsigned long node_mask,
                                  int max_node, FramePolicy fp, bool e4b) {
    DefaultFrameAssemblerBase::Ptr a;

#define args s, d, cpu_mask, node_mask, max_node, fp, e4b

    switch (d->myDetectorType) {
    case slsDetectorDefs::EIGER:
        a = std::make_shared<Eiger::Assembler>(args);
        break;
    case slsDetectorDefs::JUNGFRAU:
        a = std::make_shared<Jungfrau::Assembler>(args);
        break;
    case slsDetectorDefs::GOTTHARD:
        a = std::make_shared<GotthardAssembler>(args);
        break;
    default:
        a = std::make_shared<LegacyAssembler>(args);
    }

#undef args

    return a;
}

/**
 * DualPortFrameAssembler
 */

DualPortFrameAssembler::DualPortFrameAssembler(
    DefaultFrameAssemblerBase::Ptr a[2])
    : assembler{a[0], a[1]} {}

void DualPortFrameAssembler::stop() { stopAssemblers(); }

void DualPortFrameAssembler::stopAssemblers() {
    assembler[0]->stop(), assembler[1]->stop();
}

#include "FrameAssemblerEiger.cxx"
#include "FrameAssemblerJungfrau.cxx"
