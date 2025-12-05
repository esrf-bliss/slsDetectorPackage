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
std::mutex PacketStream<PC, SD, FP>::frame_ts_map_mutex;

template <class PC, class SD, class FP>
typename PacketStream<PC, SD, FP>::FrameTimestampMap
    PacketStream<PC, SD, FP>::frame_ts_map;

template <class PC, class SD, class FP>
PacketStream<PC, SD, FP>::PacketStream(UdpRxSocketPtr s, int rr_nb, int rr_idx,
                                       AnyCPUAffinity cpu_affinity,
                                       AnyPacketContainerPtr any_pc)
    : socket(s), rr_nb_recvs(rr_nb), rr_recv_idx(rr_idx),
      any_cpu_affinity(cpu_affinity),
      packet_cont(PacketContainerPtrFromAny<Packet>(any_pc)) {
    initFrameTimestamp();
    packet_cont->prepare();
    thread = std::make_unique<WriterThread>(*this);
}

template <class PC, class SD, class FP>
PacketStream<PC, SD, FP>::~PacketStream() {
    stop();
    thread.reset();
    packet_cont->cleanUp();
    cleanUpFrameTimestamp();
}

template <class PC, class SD, class FP>
void PacketStream<PC, SD, FP>::initFrameTimestamp() {
    std::lock_guard<std::mutex> l(frame_ts_map_mutex);
    auto [it, inserted] = frame_ts_map.emplace(
        std::piecewise_construct_t{}, std::make_tuple(socket->getPortNumber()),
        std::make_tuple());
    assert(inserted);
    frame_ts_it = it;
}

template <class PC, class SD, class FP>
void PacketStream<PC, SD, FP>::updateFrameTimestamp(uint64_t frame) {
    FrameTimestampData &frame_ts = frame_ts_it->second;
    std::lock_guard<std::mutex> l(frame_ts.mutex);
    frame_ts.ts = std::make_pair(frame, std::chrono::steady_clock::now());
}

template <class PC, class SD, class FP>
sls::FrameTimestamp PacketStream<PC, SD, FP>::getLastFrameTimestamp() {
    sls::FrameTimestamp ts{0, {}};
    auto check_latest = [&ts](auto &frame_ts) {
        std::lock_guard<std::mutex> l(frame_ts.mutex);
        if (frame_ts.ts > ts)
            ts = frame_ts.ts;
    };
    std::unique_lock<std::mutex> l(frame_ts_map_mutex);
    for (auto &[port, frame_ts] : frame_ts_map)
        check_latest(frame_ts);
    return ts;
}

template <class PC, class SD, class FP>
void PacketStream<PC, SD, FP>::cleanUpFrameTimestamp() {
    std::lock_guard<std::mutex> l(frame_ts_map_mutex);
    frame_ts_map.erase(frame_ts_it);
}

template <class PC, class SD, class FP>
void PacketStream<PC, SD, FP>::printStats() {
    std::ostringstream msg;
    msg << "[" << socket->getPortNumber() << "] "
        << "packet_delay_stat=" << packet_delay_stat.calcLinRegress() << ", "
        << "packet_push_stat=" << packet_push_stat.calcStats();
    LOG(sls::logINFO) << msg.str();
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
        if (!isValid(first_frame))
            first_frame = recv_frame;
        if (valid_packets > 0)
            ++frames_caught;
        if (full_frame)
            ++complete_frames_caught;
        if (recv_frame > last_frame)
            last_frame = recv_frame;
    }
    // insert empty (invalid) data if frame packet block can be discarded
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
                    LOG(sls::logERROR) << "Could not set writer thread "
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

    uint32_t getPacketIndex(uint32_t nb) {
        return ps.stream_data.getPacketIndex(nb);
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
        assert(frame >= curr_frame);
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

    void setInvalidPacketsUntilIdx(uint32_t good_idx) {
        assert(good_idx >= curr_idx);
        for (; curr_idx != good_idx; incPacketCounters())
            block->setValid(curr_packet, false);
    }

    void setInvalidRemainingPackets() {
        for (; curr_idx != ps.FramePackets; incPacketCounters())
            block->setValid(curr_packet, false);
    }

    void decPacketCounters() { --curr_idx; }

    void addPacketDelayStat(Packet &packet, uint32_t index) {
        Clock::time_point t = Clock::now();
        uint64_t packet_frame = ps.calcRecvFrameNumber(packet.frame());
        long packet_idx = ((packet_frame - 1) * ps.FramePackets + index);
        if (packet_idx == 0)
            t0 = t;
        double sec = ToSeconds(t - t0).count();
        ps.packet_delay_stat.add(packet_idx, sec);
    }

    template <bool valid> bool addPacket(Packet &packet) {
        uint64_t packet_frame;
        uint32_t packet_number, packet_idx;
        if constexpr (valid) {
            addPacketDelayStat(packet, curr_idx);
            packet_frame = packet.frame();
            packet_number = packet.number();
            packet_idx = getPacketIndex(packet_number);
            ps.updateFrameTimestamp(packet_frame);
        } else {
            // refer to last packet from frame received by other PacketStreams
            auto [last_frame, last_ts] = ps.getLastFrameTimestamp();
            packet_frame = last_frame;
            packet_idx = ps.FramePackets - 1;
            packet_number = getPacketNumber(packet_idx);
        }

        bool skip_trace_unexpected = true;
        auto trace_unexpected = [&](auto msg, bool force = false) {
            if (skip_trace_unexpected && !force)
                return;
            LOG(sls::logERROR) << "[" << ps.socket->getPortNumber() << "] "
                          << "unexpected " << msg << ": "
                          << "valid=" << valid << ", "
                          << "packet_frame=" << packet_frame << ", "
                          << "packet_number=" << packet_number << ", "
                          << "packet_idx=" << packet_idx << ", "
                          << "curr_frame=" << long(curr_frame) << ", "
                          << "curr_packet=" << curr_packet << ", "
                          << "curr_idx=" << curr_idx;
        };
        auto force_trace_unexpected = [&](auto msg) {
            trace_unexpected(msg, true);
        };

        // moveToGood manages dst valid flag, src must be invalidated
        if (packet_frame < curr_frame) {
            force_trace_unexpected("older frame");
            decPacketCounters();
            return true;
        } else if (packet_frame > curr_frame) {
            trace_unexpected("newer frame");
            BlockPtr new_block = ps.getEmptyBlock(packet_frame);
            if constexpr (valid) {
                if (new_block)
                    new_block->moveToGood(packet);
            }
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
            setInvalidPacketsUntilIdx(packet_idx);
        } else if (packet_idx < curr_idx) {
            force_trace_unexpected("older packet");
            if constexpr (valid)
                block->moveToGood(packet);
            decPacketCounters();
            return true;
        } else if (packet_idx > curr_idx) {
            trace_unexpected("newer packet");
            if constexpr (valid)
                block->moveToGood(packet);
            setInvalidPacketsUntilIdx(packet_idx);
        }

        block->setValid(curr_packet, valid);

        if (curr_idx == (ps.FramePackets - 1))
            finishPacketBlock();
        return true;
    }

    bool isTooDelayed(int timeout = 10) {
        using namespace std::chrono;
        auto [last_frame, last_ts] = ps.getLastFrameTimestamp();
        if (last_frame < curr_frame)
            return false;
        auto elapsed =
            duration_cast<seconds>(steady_clock::now() - last_ts).count();
        bool delayed = (elapsed > timeout);
        if (delayed)
            LOG(sls::logERROR) << "[" << ps.socket->getPortNumber() << "] "
			       << "is too delayed: "
			       << "last_frame=" << last_frame << ", "
			       << "elapsed=" << elapsed << " sec";
        return delayed;
    }

    bool processOnePacket() {
        if (!checkBlock())
            return false;

        Packet packet = getNextPacket();
        char *b = static_cast<char *>(packet.networkBuffer());
        while (true) {
            int ret = ps.socket->ReceiveDataOnly(b);
            if (ps.wasStopped() || (ret < 0))
                return false;
            else if (ret > 0) {
                int expected = sizeof(typename Packet::Data::NetworkPacket);
                if (ret != expected) {
                    LOG(sls::logERROR) << "[" << ps.socket->getPortNumber() << "] "
				       << "invalid packet size: " << ret << ", "
				       << "expected " << expected;
                }
                break;
            }
            LOG(sls::logERROR) << "[" << ps.socket->getPortNumber() << "] "
			       << "got no packet data";
            if (isTooDelayed())
                return addPacket<false>(packet);
        }

        packet.initSoftHeader();

        return addPacket<true>(packet);
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
