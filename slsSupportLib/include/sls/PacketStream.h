#pragma once
/************************************************
 * @file PacketStream.h
 * @short low-level udp packet reception classes
 ***********************************************/

#include "sls/UdpRxSocket.h"
#include "sls/logger.h"

#include <mutex>
#include <variant>

#include "CPUAffinity.h"
#include "PacketContainer.h"
#include "Stats.h"

using FramePolicy = slsDetectorDefs::frameDiscardPolicy;

using UdpRxSocket = sls::UdpRxSocket;
using UdpRxSocketPtr = std::shared_ptr<UdpRxSocket>;

/**
 *@short Frame discard policies
 */

struct NoFrameDiscard {
    static bool canDiscardFrame(int received_packets) { return false; }
};

struct EmptyFrameDiscard {
    static bool canDiscardFrame(int received_packets) {
        return !received_packets;
    }
};

struct PartialFrameDiscard {
    static bool canDiscardFrame(int received_packets) { return true; }
};

using AnyFramePolicy =
    std::variant<NoFrameDiscard, EmptyFrameDiscard, PartialFrameDiscard>;

inline AnyFramePolicy AnyFramePolicyFromFP(FramePolicy fp) {
    switch (fp) {
    case slsDetectorDefs::DISCARD_PARTIAL_FRAMES:
        return PartialFrameDiscard();
    case slsDetectorDefs::DISCARD_EMPTY_FRAMES:
        return EmptyFrameDiscard();
    default:
        return NoFrameDiscard();
    }
}

/**
 *@short manages packet stream with buffer & parallel read functionality
 */

// PC: Packet Container, SD: Stream Data, FP: Frame discard policy
template <class PC, class SD, class FP> class PacketStream {

  public:
    using PacketContainer = PC;
    using Packet = typename PacketContainer::Packet;
    using StreamData = SD;
    using FramePolicy = FP;
    using Block = typename PacketContainer::Block;
    using BlockPtr = typename PacketContainer::BlockPtr;
    using BlockLayout = typename PacketContainer::BlockLayout;
    static constexpr int FramePackets = Block::NbPackets;

    using AnyCPUAffinity = sls::CPUAffinity::AnyCPUAffinity;

    PacketStream(UdpRxSocketPtr s, int rr_nb, int rr_idx,
                 AnyCPUAffinity cpu_affinity, AnyPacketContainerPtr any_pc);
    ~PacketStream();

    void threadFunction();

    void stop();

    int getNumPacketsCaught();
    uint64_t getFirstFrameCaught();
    uint64_t getNumFramesCaught();
    uint64_t getNumCompleteFramesCaught();
    uint64_t getLastFrameIndex();

    void printStats();

    static constexpr int64_t DefaultFirstFrameIdx = 1;

    uint64_t calcDetFrameNumber(uint64_t recv_frame,
                                int64_t first_idx = DefaultFirstFrameIdx) {
        if (!isValid(recv_frame))
            return -1;
        return (recv_frame - first_idx) * rr_nb_recvs + rr_recv_idx + first_idx;
    }

    uint64_t calcRecvFrameNumber(uint64_t det_frame,
                                 int64_t first_idx = DefaultFirstFrameIdx) {
        if (!isValid(det_frame))
            return -1;
        return (det_frame - first_idx - rr_recv_idx) / rr_nb_recvs + first_idx;
    }

  private:
    struct WriterThread;

    BlockPtr getEmptyBlock() { return packet_cont->getFreePacketBlock(); }
    void addPacketBlock(BlockPtr block);

    bool isValid(uint64_t frame) { return frame != uint64_t(-1); }

    bool wasStopped();

    UdpRxSocketPtr socket;
    std::mutex mutex;
    int rr_nb_recvs;
    int rr_recv_idx;
    int packets_caught{0};
    uint64_t first_frame{uint64_t(-1)};
    uint64_t frames_caught{0};
    uint64_t complete_frames_caught{0};
    uint64_t last_frame{0};
    StreamData stream_data;
    int header_pad;
    int packet_len;
    AnyCPUAffinity any_cpu_affinity;
    typename PacketContainer::Ptr packet_cont;
    bool stopped{false};
    XYStat packet_delay_stat{1e6};
    std::unique_ptr<WriterThread> thread;
};

// UGLY
#include "sls/detectors/eiger/StreamData.h"
#include "sls/detectors/jungfrau/StreamData.h"

#define SLS_DEFINE_EIGER_PACKET_STREAM(P)                                      \
    PacketStream<PacketContainer<P>, sls::StreamData<P>, NoFrameDiscard>,      \
        PacketStream<PacketContainer<P>, sls::StreamData<P>,                   \
                     EmptyFrameDiscard>,                                       \
        PacketStream<PacketContainer<P>, sls::StreamData<P>,                   \
                     PartialFrameDiscard>

#define SLS_DEFINE_JUNGFRAU_PACKET_STREAM(P, SD)                               \
    PacketStream<PacketContainer<P>, SD, NoFrameDiscard>,                      \
        PacketStream<PacketContainer<P>, SD, EmptyFrameDiscard>,               \
        PacketStream<PacketContainer<P>, SD, PartialFrameDiscard>

using JungfrauStreamDataOneIface =
    sls::Jungfrau::StreamData<sls::Jungfrau::Geom::OneIface, 0>;
using JungfrauStreamDataTwoIface1 =
    sls::Jungfrau::StreamData<sls::Jungfrau::Geom::TwoIface, 0>;
using JungfrauStreamDataTwoIface2 =
    sls::Jungfrau::StreamData<sls::Jungfrau::Geom::TwoIface, 1>;

using AnyPacketStream = std::variant<
    SLS_DEFINE_EIGER_PACKET_STREAM(sls::EigerPacketPixel4TenGigaDisable),
    SLS_DEFINE_EIGER_PACKET_STREAM(sls::EigerPacketPixel4TenGigaEnable),
    SLS_DEFINE_EIGER_PACKET_STREAM(sls::EigerPacketPixel8TenGigaDisable),
    SLS_DEFINE_EIGER_PACKET_STREAM(sls::EigerPacketPixel8TenGigaEnable),
    SLS_DEFINE_EIGER_PACKET_STREAM(sls::EigerPacketPixel16TenGigaDisable),
    SLS_DEFINE_EIGER_PACKET_STREAM(sls::EigerPacketPixel16TenGigaEnable),
    SLS_DEFINE_EIGER_PACKET_STREAM(sls::EigerPacketPixel32TenGigaDisable),
    SLS_DEFINE_EIGER_PACKET_STREAM(sls::EigerPacketPixel32TenGigaEnable),
    SLS_DEFINE_JUNGFRAU_PACKET_STREAM(sls::JungfrauPacketOneIface,
                                      JungfrauStreamDataOneIface),
    SLS_DEFINE_JUNGFRAU_PACKET_STREAM(sls::JungfrauPacketTwoIface,
                                      JungfrauStreamDataTwoIface1),
    SLS_DEFINE_JUNGFRAU_PACKET_STREAM(sls::JungfrauPacketTwoIface,
                                      JungfrauStreamDataTwoIface2)>;

std::shared_ptr<AnyPacketStream>
CreatePacketStream(UdpRxSocketPtr s, slsDetectorDefs::detectorType det_type,
                   bool tg_enable, int num_udp_ifaces, uint32_t dr, int idx,
                   int rr_nb, int rr_idx,
                   sls::CPUAffinity::AnyCPUAffinity cpu_affinity,
                   FramePolicy fp, AnyPacketContainerPtr any_pc);

#include "PacketStream.cxx"
