#pragma once
/************************************************
 * @file FrameAssembler.h
 * @short helper classes assembling frames
 * from udp packets
 ***********************************************/

#include "sls/Geometry.h"
#include "sls/UdpRxSocket.h"
#include "sls/logger.h"
#include "sls/sls_detector_defs.h"

#include "GeneralData.h"

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <variant>
#include <vector>

#include "MmappedRegion.h"
#include "Stats.h"

namespace FrameAssembler {

using namespace sls::Geom;

using DetHeader = slsDetectorDefs::sls_detector_header;
using RecvHeader = slsDetectorDefs::sls_receiver_header;
using FramePolicy = slsDetectorDefs::frameDiscardPolicy;

using UdpRxSocket = sls::UdpRxSocket;
using UdpRxSocketPtr = std::shared_ptr<UdpRxSocket>;
typedef GeneralData *GeneralDataPtr;

/**
 * Memory layout of each element of the packet buffer array
 *
 * < pad > < --------------------- buffer --------------------------- >
 *         < soft_header > < ------------ network_buffer ------------ >
 *                         < -- network_header -- > < ---- data ----- >
 *
 * Note: <pad> is calculated in order to have <data> aligned to 128-bits
 */

/**
 *@short The PacketData base struct
 */

template <int DataLen, class NetworkHeader, int FramePackets, int Align = 16>
struct PacketData {
    static constexpr int PacketDataLen = DataLen;
    static constexpr int PacketsPerFrame = FramePackets;
    static constexpr int FrameLen = PacketDataLen * PacketsPerFrame;

    using NetworkPacketHeader = NetworkHeader;

    struct NetworkPacket {
        NetworkPacketHeader header;
        char data[PacketDataLen];
    } __attribute__((packed));

    // An instance of <derived>::SoftHeader prepends each network packet
    struct SoftHeader {
        bool valid;
    };

    // The Packet structure in the (software) buffer
    struct SoftwarePacket {
        SoftHeader soft_header;
        NetworkPacket net_packet;
    } __attribute__((packed));

    static constexpr int DataOffset =
        offsetof(SoftwarePacket, net_packet) + offsetof(NetworkPacket, data);
    static constexpr int Pad = AlignCeil(DataOffset, Align) - DataOffset;

    struct Layout {
        char pad[Pad];
        SoftwarePacket soft_packet;
    } __attribute__((packed));
};

/**
 *@short The Packet base struct
 */

template <class PD> struct Packet {
    using Data = PD;
    using SoftwarePacket = typename Data::SoftwarePacket;
    using SoftHeader = typename Data::SoftHeader;
    using NetworkPacketHeader = typename Data::NetworkPacketHeader;
    using Layout = typename Data::Layout;

    SoftwarePacket *buffer;

    Packet(Layout *l) : buffer(&l->soft_packet) {}

    SoftHeader *softHeader() { return &buffer->soft_header; }
    void initSoftHeader() {}

    bool valid() { return softHeader()->valid; }

    void *networkBuffer() { return &buffer->net_packet; }

    NetworkPacketHeader *networkHeader() { return &buffer->net_packet.header; }

    char *data() { return buffer->net_packet.data; }
};

/**
 *@short StdPacket class
 */

template <int DataLen, int FramePackets>
using StdPacketData = PacketData<DataLen, DetHeader, FramePackets>;

template <class PacketData> struct StdPacket : Packet<PacketData> {
    using Base = Packet<PacketData>;
    using Layout = typename Base::Layout;

    StdPacket(Layout *l) : Base(l) {}

    uint64_t frame() { return Base::networkHeader()->frameNumber; }

    uint32_t number() { return Base::networkHeader()->packetNumber; }

    uint32_t sizeAdjust() { return 0; }

    void fillDetHeader(DetHeader *det_header) {
        memcpy(det_header, Base::networkHeader(), sizeof(*det_header));
    }
};

/**
 *@short Packet Block: packets in a frame
 */

// P: Packet
template <class P> class PacketBlock {
  public:
    static constexpr int NbPackets = P::Data::PacketsPerFrame;
    using Layout = std::array<typename P::Layout, NbPackets>;
    using LayoutPtr = std::unique_ptr<Layout, std::function<void(Layout *)>>;

    PacketBlock(LayoutPtr &&l) : layout(std::move(l)){};

    P operator[](unsigned int i) { return P(&(*layout)[i]); }

    void setValid(unsigned int i, bool valid);

    void moveToGood(P &p);

    bool hasFullFrame() { return valid_packets == NbPackets; }

    int getValidPackets() { return valid_packets; }

  private:
    LayoutPtr layout;
    int valid_packets{0};
};

template <class P> using PacketBlockPtr = std::unique_ptr<PacketBlock<P>>;

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

    void clearBuffer();

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

/**
 *@short Multi-port frame assembler result
 */

constexpr int MaxNbPorts = 2;

using PortsMask = std::bitset<MaxNbPorts>;

struct Result {
    int nb_ports;
    PortsMask valid_data;
};

/**
 *@short Frame assembler interface
 */

class FrameAssemblerBase {

  public:
    virtual ~FrameAssemblerBase() {}

    virtual Result assembleFrame(uint64_t frame, RecvHeader *header,
                                 char *buf) = 0;

    virtual void stop() = 0;
};

using FrameAssemblerPtr = std::shared_ptr<FrameAssemblerBase>;

/**
 *@short Default frame assembler in Listener
 */

class DefaultFrameAssemblerBase : public FrameAssemblerBase {

  public:
    virtual bool hasPendingPacket() = 0;
    virtual int getImageSize() = 0;

    virtual int getNumPacketsCaught() = 0;
    virtual uint64_t getNumFramesCaught() = 0;
    virtual uint64_t getLastFrameIndex() = 0;

    virtual void clearBuffers() = 0;

    virtual void printStreamStats() = 0;
};
using DefaultFrameAssemblerPtr = std::shared_ptr<DefaultFrameAssemblerBase>;
using DefaultFrameAssemblerList = std::vector<DefaultFrameAssemblerPtr>;

DefaultFrameAssemblerPtr
CreateDefaultFrameAssembler(UdpRxSocketPtr s, GeneralDataPtr d, int idx,
                            cpu_set_t cpu_mask, unsigned long node_mask,
                            int max_node, FramePolicy fp, bool e4b);
/*
 * DefaultFrameAssembler:
 *   P: Packet, SD: Stream Data, FP: Frame discard Policy,
 *   SP: Src Pixel, DP: Dst Pixel
 */

template <class P, class SD, class FP, class SP, class DP = SP>
class DefaultFrameAssembler : public DefaultFrameAssemblerBase {

  public:
    static constexpr bool Expand4Bits =
        (std::is_same_v<SP, Pixel4> && std::is_same_v<DP, Pixel8>);

    using Stream = PacketStream<P, SD, FP>;
    using StreamPtr = std::unique_ptr<Stream>;

    DefaultFrameAssembler(UdpRxSocketPtr s, cpu_set_t cpu_mask,
                          unsigned long node_mask, int max_node);

    Result assembleFrame(uint64_t frame, RecvHeader *header,
                         char *buf) override;

    void stop() override;

    bool hasPendingPacket() override;
    int getImageSize() override;

    int getNumPacketsCaught() override;
    uint64_t getNumFramesCaught() override;
    uint64_t getLastFrameIndex() override;

    void clearBuffers() override;

    void printStreamStats() override;

    Stream &getStream() { return *packet_stream.get(); }

  protected:
    void expand4Bits(char *dst, char *src, int src_size);

    StreamPtr packet_stream;
};

/**
 *@short Raw frame assembler: vertical concatenation of default assemblers
 */

class RawFrameAssembler : public FrameAssemblerBase {

  public:
    RawFrameAssembler(DefaultFrameAssemblerList a, int recv_idx)
        : assembler(a) {
        int iface_size = assembler[0]->getImageSize();
        data_offset = assembler.size() * iface_size * recv_idx;
    }

    Result assembleFrame(uint64_t frame, RecvHeader *recv_header,
                         char *buf) override;
    void stop() override {
        for (auto &a : assembler)
            a->stop();
    }

  private:
    DefaultFrameAssemblerList assembler;
    int data_offset;
};

} // namespace FrameAssembler

#include "FrameAssemblerEiger.hxx"
#include "FrameAssemblerGotthard.hxx"
#include "FrameAssemblerJungfrau.hxx"
