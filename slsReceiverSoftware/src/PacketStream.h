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

// An instance of StreamData is included in the PacketStream
template <class Packet> struct StreamData {
    // Describes the sequence of the packets in the stream
    uint32_t getPacketNumber(uint32_t packet_idx) { return packet_idx; }
};

/**
 *@short manages packet stream with buffer & parallel read functionality
 */

// P: Packet, SD: Stream Data, FP: Frame discard policy
template <class P, class SD, class FP> class PacketStream {

  public:
    using Packet = P;
    using StreamData = SD;
    using FramePolicy = FP;
    using Block = PacketBlock<Packet>;
    using BlockPtr = PacketBlockPtr<Packet>;
    using BlockLayout = typename Block::Layout;
    static constexpr int FramePackets = Block::NbPackets;

    PacketStream(UdpRxSocketPtr s, cpu_set_t cpu_mask, pid_t thread_id,
                 AnyPacketContainerPtr pc);
    ~PacketStream();

    void threadFunction();

    void stop();

    int getNumPacketsCaught();
    uint64_t getNumFramesCaught();
    uint64_t getLastFrameIndex();

    void printStats();

  private:
    struct WriterThread;

    BlockPtr getEmptyBlock() { return packet_cont.getFreePacketBlock(); }
    void addPacketBlock(BlockPtr &&block);

    bool wasStopped();

    UdpRxSocketPtr socket;
    std::mutex mutex;
    int packets_caught{0};
    uint64_t frames_caught{0};
    uint64_t last_frame{0};
    StreamData stream_data;
    int header_pad;
    int packet_len;
    typename PacketContainer<Packet>::StreamIface packet_cont;
    bool stopped{false};
    cpu_set_t cpu_aff_mask;
    XYStat packet_delay_stat{1e6};
    std::unique_ptr<WriterThread> thread;
};

#define AllPacketStreamsFor(P, SD)                                             \
    PacketStream<P, SD, NoFrameDiscard>,                                       \
        PacketStream<P, SD, EmptyFrameDiscard>,                                \
        PacketStream<P, SD, PartialFrameDiscard>

/*
 * Eiger packet stream definitions
 */

namespace Eiger {

template <class Pixel, class FP, class TenGiga = TenGigaEnable>
using PacketStream = ::PacketStream<Packet<Pixel, TenGiga>,
                                    StreamData<Packet<Pixel, TenGiga>>, FP>;

// Only 10G supported so far
#define EigerPacketStreamsFor(P)                                               \
    AllPacketStreamsFor(EigerPacketFor(P), StreamData<EigerPacketFor(P)>)

#define EigerPacketStreams                                                     \
    EigerPacketStreamsFor(Pixel4), EigerPacketStreamsFor(Pixel8),              \
        EigerPacketStreamsFor(Pixel16), EigerPacketStreamsFor(Pixel32)

} // namespace Eiger

/*
 * Jungfrau packet stream definitions
 */

namespace Jungfrau {

template <int NbUDPIfaces, int Idx>
struct StreamData : ::StreamData<Packet<NbUDPIfaces>> {
    uint32_t getPacketNumber(uint32_t packet_idx);
};

template <int NbUDPIfaces, int Idx, class FP>
using PacketStream =
    ::PacketStream<Packet<NbUDPIfaces>, StreamData<NbUDPIfaces, Idx>, FP>;

#define JungfrauPSData1  ::Jungfrau::StreamData<1, 0>
#define JungfrauPSData20 ::Jungfrau::StreamData<2, 0>
#define JungfrauPSData21 ::Jungfrau::StreamData<2, 1>

#define JungfrauPacketStreams                                                  \
    AllPacketStreamsFor(::Jungfrau::Packet<1>, JungfrauPSData1),               \
        AllPacketStreamsFor(::Jungfrau::Packet<2>, JungfrauPSData20),          \
        AllPacketStreamsFor(::Jungfrau::Packet<2>, JungfrauPSData21)

} // namespace Jungfrau

// AnyPacketStream
using AnyPacketStream = std::variant<EigerPacketStreams, JungfrauPacketStreams>;

template <class PS, class... Ps> constexpr bool IsGroupPacketStream() {
    using a = std::tuple<std::bool_constant<std::is_same_v<PS, Ps>>...>;
    using b = std::tuple<std::bool_constant<std::is_same_v<int, Ps>>...>;
    return !std::is_same_v<a, b>;
}

template <class PS> constexpr bool IsEigerPacketStream() {
    return IsGroupPacketStream<PS, EigerPacketStreams>();
}

template <class PS> constexpr bool IsJungfrauPacketStream() {
    return IsGroupPacketStream<PS, JungfrauPacketStreams>();
}

using AnyPacketStreamPtr = std::shared_ptr<AnyPacketStream>;
using PacketStreamList = std::vector<AnyPacketStreamPtr>;

#undef JungfrauPacketStreams
#undef JungfrauPSData21
#undef JungfrauPSData20
#undef JungfrauPSData1
#undef EigerPacketStreams
#undef EigerPacketStreamsFor
#undef AllPacketStreamsFor

AnyPacketStreamPtr CreatePacketStream(UdpRxSocketPtr s, GeneralDataPtr d,
                                      int idx, cpu_set_t cpu_mask,
                                      pid_t thread_id, FramePolicy fp,
                                      AnyPacketContainerPtr any_pc);

#include "PacketStream.cxx"
