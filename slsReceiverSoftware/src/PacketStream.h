#pragma once
/************************************************
 * @file PacketStream.h
 * @short low-level udp packet reception classes
 ***********************************************/

#include "sls/UdpRxSocket.h"
#include "sls/logger.h"

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <map>
#include <mutex>
#include <numeric>
#include <queue>
#include <variant>

#include "GeneralData.h"
#include "MmappedRegion.h"
#include "Packet.h"
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

    static constexpr int MaxBufferFrames = 4;

  public:
    using Packet = P;
    using StreamData = SD;
    using FramePolicy = FP;
    using Block = PacketBlock<P>;
    using BlockPtr = PacketBlockPtr<P>;
    using BlockLayout = typename Block::Layout;
    static constexpr int FramePackets = Block::NbPackets;

    using MmappedBlockRegion = MmappedRegion<BlockLayout>;

    PacketStream(UdpRxSocketPtr s, cpu_set_t cpu_mask, unsigned long node_mask,
                 int max_node);
    ~PacketStream();

    BlockPtr getPacketBlock(uint64_t frame);

    bool hasPendingPacket();
    void stop();
    bool wasStopped();

    int getNumPacketsCaught();
    uint64_t getNumFramesCaught();
    uint64_t getLastFrameIndex();

    void clearBuffers();

    void printStats();

  private:
    struct WriterThread;
    using PacketBlockMap = std::map<uint64_t, BlockPtr>;
    using MapIterator = typename PacketBlockMap::iterator;
    using FramePacketBlock = typename PacketBlockMap::value_type;

    BlockPtr getEmptyBlock();
    void addPacketBlock(FramePacketBlock &&frame_block);
    void releaseReadyPacketBlocks();
    void waitUsedPacketBlocks();

    UdpRxSocketPtr socket;
    const unsigned int num_frames;
    std::mutex mutex;
    int packets_caught{0};
    uint64_t frames_caught{0};
    uint64_t last_frame{0};
    StreamData stream_data;
    int header_pad;
    int packet_len;
    MmappedBlockRegion packet_buffer_array;
    std::mutex free_mutex;
    std::condition_variable free_cond;
    std::queue<BlockLayout *> free_queue;
    bool stopped{false};
    int waiting_reader_count{0};
    std::mutex block_mutex;
    std::condition_variable block_cond;
    cpu_set_t cpu_aff_mask;
    XYStat packet_delay_stat{1e6};
    PacketBlockMap packet_block_map;
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
                                      unsigned long node_mask, int max_node,
                                      FramePolicy fp);

#include "PacketStream.cxx"
