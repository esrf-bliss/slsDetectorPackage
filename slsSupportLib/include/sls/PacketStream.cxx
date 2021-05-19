/************************************************
 * @file PacketStream.cxx
 * @short low-level udp packet reception classes
 ***********************************************/

#include <cassert>

/**
 * PacketStream
 */

template <class P, class SD, class FP>
PacketStream<P, SD, FP>::PacketStream(UdpRxSocketPtr s, cpu_set_t cpu_mask,
                                      pid_t thread_id,
                                      AnyPacketContainerPtr any_pc)
    : socket(s), packet_cont(PacketContainerPtrFromAny<Packet>(any_pc)),
      cpu_aff_mask(cpu_mask) {
    packet_cont.prepare();
    thread = std::make_unique<WriterThread>(*this, thread_id);
}

template <class P, class SD, class FP>
PacketStream<P, SD, FP>::~PacketStream() {
    stop();
    thread.reset();
    packet_cont.cleanUp();
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
    {
        std::lock_guard<std::mutex> l(mutex);
        stopped = true;
    }
    packet_cont.stop();
}

template <class P, class SD, class FP>
bool PacketStream<P, SD, FP>::wasStopped() {
    std::lock_guard<std::mutex> l(mutex);
    return stopped;
}

template <class P, class SD, class FP>
int PacketStream<P, SD, FP>::getNumPacketsCaught() {
    std::lock_guard<std::mutex> l(mutex);
    return packets_caught;
}

template <class P, class SD, class FP>
uint64_t PacketStream<P, SD, FP>::getFirstFrameCaught() {
    std::lock_guard<std::mutex> l(mutex);
    return first_frame;
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
void PacketStream<P, SD, FP>::addPacketBlock(BlockPtr block) {
    bool full_frame = block->hasFullFrame();
    {
        uint64_t frame = block->getFrameNumber();
        std::lock_guard<std::mutex> l(mutex);
        if (first_frame == uint64_t(-1))
            first_frame = frame;
        if (full_frame)
            ++frames_caught;
        if (frame > last_frame)
            last_frame = frame;
    }
    if (full_frame || !FP::canDiscardFrame(block->getValidPackets()))
        packet_cont.putReadyPacketBlock(std::move(block));
}

template <class P, class SD, class FP>
void PacketStream<P, SD, FP>::threadFunction() {
    thread->threadFunction();
}

template <class P, class SD, class FP>
class PacketStream<P, SD, FP>::WriterThread {
  public:
    WriterThread(PacketStream &s, pid_t thread_id) : ps(s) {
        struct sched_param param;
        param.sched_priority = 90;
        int ret = sched_setscheduler(thread_id, SCHED_FIFO, &param);
        if (ret != 0)
            std::cerr << "Could not set packet thread RT priority!"
                      << std::endl;
    }

    ~WriterThread() {
        std::unique_lock<std::mutex> l(ps.mutex);
        while (running)
            cond.wait(l);
    }

    void threadFunction() {
        cpu_set_t &cpu_aff_mask = ps.cpu_aff_mask;
        if (CPU_COUNT(&cpu_aff_mask) != 0) {
            int size = sizeof(cpu_aff_mask);
            int ret = sched_setaffinity(0, size, &cpu_aff_mask);
            if (ret != 0)
                std::cerr << "Could not set writer thread "
                          << "cpu affinity mask" << std::endl;
        }
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

    void finishPacketBlock() {
        ps.addPacketBlock(std::move(block));
        assert(!block);
        curr_idx = curr_packet = -1;
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
                      << "curr_frame=" << block->getFrameNumber() << ", "
                      << "curr_packet=" << curr_packet << ", "
                      << "curr_idx=" << curr_idx << std::endl;
        };

        // moveToGood manages both src & dst valid flags
        bool first_packet = !block->getNetworkHeader();
        if (!first_packet && (packet_frame != block->getFrameNumber())) {
            trace_unexpected("new frame");
            BlockPtr new_block = ps.getEmptyBlock();
            if (new_block)
                new_block->moveToGood(packet);
            else
                block->setValid(curr_packet, false);
            setInvalidRemainingPackets();
            finishPacketBlock();
            if (!new_block)
                return false;
            block = std::move(new_block);
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

        P packet = getNextPacket();
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
    uint32_t curr_idx{uint32_t(-1)};
    uint32_t curr_packet{uint32_t(-1)};
    bool running{false};
    std::condition_variable cond;
};
