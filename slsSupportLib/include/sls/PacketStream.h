#pragma once
/************************************************
 * @file PacketStream.h
 * @short low-level udp packet reception classes
 ***********************************************/

#include "sls/UdpRxSocket.h"
#include "sls/logger.h"

#include <mutex>
#include <variant>

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

    PacketStream(UdpRxSocketPtr s, cpu_set_t cpu_mask,
                 AnyPacketContainerPtr any_pc);
    ~PacketStream();

    void threadFunction();

    void stop();

    int getNumPacketsCaught();
    uint64_t getFirstFrameCaught();
    uint64_t getNumFramesCaught();
    uint64_t getLastFrameIndex();

    void printStats();

  private:
    struct WriterThread;

    BlockPtr getEmptyBlock() { return packet_cont->getFreePacketBlock(); }
    void addPacketBlock(BlockPtr block);

    bool wasStopped();

    UdpRxSocketPtr socket;
    std::mutex mutex;
    int packets_caught{0};
    uint64_t first_frame{uint64_t(-1)};
    uint64_t frames_caught{0};
    uint64_t last_frame{0};
    StreamData stream_data;
    int header_pad;
    int packet_len;
    typename PacketContainer::Ptr packet_cont;
    bool stopped{false};
    cpu_set_t cpu_aff_mask;
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
                   cpu_set_t cpu_mask, FramePolicy fp,
                   AnyPacketContainerPtr any_pc);

#include "PacketStream.cxx"
