/************************************************
 * @file PacketStream.cxx
 * @short low-level udp packet reception classes
 ***********************************************/

#include <cassert>

#include "PacketStream.h"
#include "sls/logger.h"

/**
 * PacketStream
 */

template <class PC, class SD, class FP>
PacketStream<PC, SD, FP>::PacketStream(UdpRxSocketPtr s, int rr_nb, int rr_idx,
                                       AnyCPUAffinity cpu_affinity,
                                       AnyPacketContainerPtr any_pc)
    : socket(s), rr_nb_recvs(rr_nb), rr_recv_idx(rr_idx),
      any_cpu_affinity(cpu_affinity),
      packet_cont(PacketContainerPtrFromAny<Packet>(any_pc)) {
    packet_cont->prepare();
    thread = std::make_unique<WriterThread>(*this);
}

template <class PC, class SD, class FP>
PacketStream<PC, SD, FP>::~PacketStream() {
    stop();
    thread.reset();
    packet_cont->cleanUp();
}

template <class PC, class SD, class FP>
void PacketStream<PC, SD, FP>::printStats() {
    std::ostringstream msg;
    msg << "[" << socket->getPortNumber() << "] "
        << "packet_delay_stat=" << packet_delay_stat.calcLinRegress() << ", "
        << "packet_push_stat=" << packet_push_stat.calcStats();
    LOG(logINFO) << msg.str();
}

template <class PC, class SD, class FP> void PacketStream<PC, SD, FP>::stop() {
    {
        std::lock_guard<std::mutex> l(mutex);
        stopped = true;
    }
    packet_cont->stop();
}

template <class PC, class SD, class FP>
bool PacketStream<PC, SD, FP>::wasStopped() {
    std::lock_guard<std::mutex> l(mutex);
    return stopped;
}

template <class PC, class SD, class FP>
int PacketStream<PC, SD, FP>::getNumPacketsCaught() {
    std::lock_guard<std::mutex> l(mutex);
    return packets_caught;
}

template <class PC, class SD, class FP>
uint64_t PacketStream<PC, SD, FP>::getFirstFrameCaught() {
    std::lock_guard<std::mutex> l(mutex);
    return first_frame;
}

template <class PC, class SD, class FP>
uint64_t PacketStream<PC, SD, FP>::getNumFramesCaught() {
    std::lock_guard<std::mutex> l(mutex);
    return frames_caught;
}

template <class PC, class SD, class FP>
uint64_t PacketStream<PC, SD, FP>::getNumCompleteFramesCaught() {
    std::lock_guard<std::mutex> l(mutex);
    return complete_frames_caught;
}

template <class PC, class SD, class FP>
uint64_t PacketStream<PC, SD, FP>::getLastFrameIndex() {
    std::lock_guard<std::mutex> l(mutex);
    return last_frame;
}

template <class PC, class SD, class FP>
void PacketStream<PC, SD, FP>::addPacketBlock(BlockPtr block,
                                              uint64_t det_frame) {
    bool full_frame = block->hasFullFrame();
    int valid_packets = block->getValidPackets();
    {
        uint64_t recv_frame = calcRecvFrameNumber(det_frame);
        block->setRecvFrameNumber(recv_frame);
        std::lock_guard<std::mutex> l(mutex);
        if (first_frame == uint64_t(-1))
            first_frame = recv_frame;
        if (valid_packets > 0)
            ++frames_caught;
        if (full_frame)
            ++complete_frames_caught;
        if (recv_frame > last_frame)
            last_frame = recv_frame;
    }
    // insert nullptr if frame packet block can be discarded
    if (!full_frame && FP::canDiscardFrame(valid_packets))
        block->discard();
    packet_cont->putReadyPacketBlock(std::move(block));
}

template <class PC, class SD, class FP>
void PacketStream<PC, SD, FP>::threadFunction() {
    thread->threadFunction();
}

template <class PC, class SD, class FP>
class PacketStream<PC, SD, FP>::WriterThread {
  public:
    WriterThread(PacketStream &s) : ps(s) { curr_frame += ps.rr_recv_idx; }

    ~WriterThread() {
        std::unique_lock<std::mutex> l(ps.mutex);
        while (running)
            cond.wait(l);
    }

    void threadFunction() {
        setThreadAffinity();
        {
            std::lock_guard<std::mutex> l(ps.mutex);
            running = true;
        }
        while (true) {
            if (!processOnePacket())
                break;

            std::lock_guard<std::mutex> l(ps.mutex);
            ++ps.packets_caught;
        }
        // release current block
        block.reset();
        {
            std::lock_guard<std::mutex> l(ps.mutex);
            running = false;
            cond.notify_one();
        }
    }

  private:
    void setThreadAffinity() {
        using FixedCPUSetAffinity = sls::CPUAffinity::FixedCPUSetAffinityMask;
        if (std::holds_alternative<FixedCPUSetAffinity>(ps.any_cpu_affinity)) {
            auto cpu_mask = std::get<FixedCPUSetAffinity>(ps.any_cpu_affinity);
            if (cpu_mask.count() > 0) {
                try {
                    cpu_mask.apply_to_this_thread();
                } catch (sls::RuntimeError &e) {
                    LOG(logERROR) << "Could not set writer thread "
                                  << "cpu affinity mask: " << e.what();
                }
            }
        }
    }

    bool checkBlock() {
        if (!block)
            block = std::move(ps.getEmptyBlock(curr_frame));
        return bool(block);
    }

    uint32_t getPacketNumber(uint32_t idx) {
        return ps.stream_data.getPacketNumber(idx);
    }

    void incFrameCounter() { curr_frame += ps.rr_nb_recvs; }

    std::pair<uint32_t, uint32_t> incPacketCounters() {
        curr_packet = getPacketNumber(++curr_idx);
        return {curr_idx, curr_packet};
    }

    Packet getNextPacket() { return (*block)[incPacketCounters().second]; }

    void finishPacketBlock() {
        Clock::time_point t0 = Clock::now();
        ps.addPacketBlock(std::move(block), curr_frame);
        Clock::time_point t = Clock::now();
        double sec = ToSeconds(t - t0).count();
        ps.packet_push_stat.add(sec);
        assert(!block);
        curr_idx = curr_packet = -1;
        incFrameCounter();
    }

    bool setMissingFramesUntil(uint64_t frame) {
        while (curr_frame != frame) {
            block = std::move(ps.getEmptyBlock(curr_frame));
            if (!block)
                return false;
            getNextPacket();
            setInvalidRemainingPackets();
            finishPacketBlock();
        }
        return true;
    }

    void setInvalidPacketsUntil(uint32_t good_packet) {
        for (; curr_packet != good_packet; incPacketCounters())
            block->setValid(curr_packet, false);
    }

    void setInvalidRemainingPackets() {
        for (; curr_idx != ps.FramePackets; incPacketCounters())
            block->setValid(curr_packet, false);
    }

    void addPacketDelayStat(Packet &packet, uint32_t index) {
        Clock::time_point t = Clock::now();
        uint64_t packet_frame = ps.calcRecvFrameNumber(packet.frame());
        long packet_idx = ((packet_frame - 1) * ps.FramePackets + index);
        if (packet_idx == 0)
            t0 = t;
        double sec = ToSeconds(t - t0).count();
        ps.packet_delay_stat.add(packet_idx, sec);
    }

    bool addPacket(Packet &packet) {
        addPacketDelayStat(packet, curr_idx);

        uint64_t packet_frame = packet.frame();
        uint32_t packet_number = packet.number();

        bool skip_trace_unexpected = true;
        auto trace_unexpected = [&](auto msg) {
            if (skip_trace_unexpected)
                return;
            LOG(logERROR) << "[" << ps.socket->getPortNumber() << "] "
                          << "unexpected " << msg << ": "
                          << "packet_frame=" << packet_frame << ", "
                          << "packet_number=" << packet_number << ", "
                          << "curr_frame=" << long(curr_frame) << ", "
                          << "curr_packet=" << curr_packet << ", "
                          << "curr_idx=" << curr_idx;
        };

        // moveToGood manages dst valid flag, src must be invalidated
        if (packet_frame != curr_frame) {
            trace_unexpected("new frame");
            BlockPtr new_block = ps.getEmptyBlock(packet_frame);
            if (new_block)
                new_block->moveToGood(packet);
            // Finish current block if it's got some data
            if (curr_idx > 0) {
                setInvalidRemainingPackets();
                finishPacketBlock();
            } else {
                block.reset();
                curr_idx = curr_packet = -1;
            }
            if (!setMissingFramesUntil(packet_frame) || !new_block)
                return false;
            // initialize new block
            block = std::move(new_block);
            incPacketCounters();
            setInvalidPacketsUntil(packet_number);
        } else if (packet_number != curr_packet) {
            trace_unexpected("bad frame");
            block->moveToGood(packet);
            setInvalidPacketsUntil(packet_number);
        } else {
            block->setValid(curr_packet, true);
        }

        if (curr_idx == (ps.FramePackets - 1))
            finishPacketBlock();
        return true;
    }

    bool processOnePacket() {
        if (!checkBlock())
            return false;

        Packet packet = getNextPacket();
        char *b = static_cast<char *>(packet.networkBuffer());
        int ret = ps.socket->ReceiveDataOnly(b);
        if (ps.wasStopped() || (ret < 0))
            return false;

        packet.initSoftHeader();

        return addPacket(packet);
    }

    PacketStream &ps;
    Clock::time_point t0;
    BlockPtr block;
    uint64_t curr_frame{DefaultFirstFrameIdx};
    uint32_t curr_idx{uint32_t(-1)};
    uint32_t curr_packet{uint32_t(-1)};
    bool running{false};
    std::condition_variable cond;
};
