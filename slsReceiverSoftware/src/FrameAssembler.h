#pragma once
/************************************************
 * @file FrameAssembler.h
 * @short helper classes assembling frames
 * from udp packets
 ***********************************************/

#include "sls/UdpRxSocket.h"
#include "sls/logger.h"
#include "sls/sls_detector_defs.h"

#include "GeneralData.h"

#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

#include "MmappedRegion.h"
#include "Stats.h"

namespace FrameAssembler {

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

struct PacketDataBase {
    // An instance of <derived>::StreamData is included in the PacketStream
    struct StreamData {
        StreamData(GeneralDataPtr /*d*/, int idx = 0) : stream_idx(idx) {}

        // Describes the sequence of the packets in the stream
        uint32_t getPacketNumber(uint32_t packet_idx) { return packet_idx; }

        // Index of stream in receiver
        int stream_idx;
    };

    // An instance of <derived>::SoftHeader prepends each network packet
    struct SoftHeader {
        bool valid;
        int index;
    };
};

/**
 *@short The Packet base struct
 */

template <class PD> struct Packet {
    using StreamData = typename PD::StreamData;
    using SoftHeader = typename PD::SoftHeader;

    char *buffer;

    Packet(char *b, StreamData & /*sd*/) : buffer(b) {}

    SoftHeader *softHeader() { return reinterpret_cast<SoftHeader *>(buffer); }

    bool valid() { return softHeader()->valid; }

    void setIndex(int index) { softHeader()->index = index; }
    int index() { return softHeader()->index; }

    char *networkBuffer() { return buffer + sizeof(SoftHeader); }

    char *networkHeader() { return networkBuffer(); }

    static void checkConsistency(GeneralDataPtr gd);
};

/**
 *@short StdPacket class
 */

template <class PD> struct StdPacketImpl : public Packet<PD> {
    using Base = Packet<PD>;
    using StreamData = typename Base::StreamData;

    StdPacketImpl(char *b, StreamData &sd) : Base(b, sd) {}

    void initSoftHeader() {}

    DetHeader *header() {
        return reinterpret_cast<DetHeader *>(Base::networkHeader());
    }

    uint64_t frame() { return header()->frameNumber; }

    uint32_t number() { return header()->packetNumber; }

    char *data() { return Base::networkHeader() + sizeof(DetHeader); }

    uint32_t sizeAdjust() { return 0; }

    void fillDetHeader(DetHeader *det_header);
};

/**
 *@short LegacyPacket class
 */

struct LegacyPacketData : public PacketDataBase {
    struct StreamData : public PacketDataBase::StreamData {
        GeneralDataPtr gd;
        bool odd_numbering{true};
        StreamData(GeneralDataPtr d, int idx)
            : PacketDataBase::StreamData(d, idx), gd(d) {}
    };

    struct SoftHeader : public PacketDataBase::SoftHeader {
        uint64_t packet_frame;
        uint32_t packet_number;
    };
};

template <class PD> struct LegacyPacketImpl : public Packet<PD> {
    using Base = Packet<PD>;
    using StreamData = typename Base::StreamData;

    StreamData &stream_data;

    LegacyPacketImpl(char *b, StreamData &sd) : Base(b, sd), stream_data(sd) {}

    GeneralDataPtr generalData() { return stream_data.gd; }

    void initSoftHeader();

    uint64_t frame() { return Base::softHeader()->packet_frame; }

    uint32_t number() { return Base::softHeader()->packet_number; }

    char *data() {
        return (Base::networkHeader() + generalData()->headerSizeinPacket);
    }

    uint32_t sizeAdjust() { return 0; }

    void fillDetHeader(DetHeader *det_header);
};

using LegacyPacket = LegacyPacketImpl<LegacyPacketData>;

/**
 *@short GotthardPacket class
 */

struct GotthardPacketData : public LegacyPacketData {
    struct StreamData : public LegacyPacketData::StreamData {
        bool first_packet{true};
        StreamData(GeneralDataPtr d, int idx)
            : LegacyPacketData::StreamData(d, idx) {}
    };
};

struct GotthardPacket : public LegacyPacketImpl<GotthardPacketData> {
    using Base = LegacyPacketImpl<GotthardPacketData>;

    GotthardPacket(char *b, StreamData &sd) : Base(b, sd) {}

    void initSoftHeader();

    // Gotthard data:
    //   1st packet: CACA + CACA, (640 - 1) * 2 bytes data
    //   2nd packet: (1 + 640) * 2 bytes data

    char *data() { return Base::data() + ((number() == 0) ? 4 : 0); }

    uint32_t sizeAdjust() { return (number() == 0) ? -2 : 2; }
};

/**
 *@short Packet Block: reference packets in a frame
 */

template <class P> class PacketStream;

template <class P> class PacketBlock {
  public:
    PacketBlock(PacketStream<P> &s, char *b);
    ~PacketBlock();

    int getNbPackets() { return ps.getNbPacketFrames(); }

    P operator[](unsigned int i);

    void setValid(unsigned int i, bool valid);

    void moveToGood(P &p);

    bool hasFullFrame() { return valid_packets == getNbPackets(); }

    int getValidPackets() { return valid_packets; }

  private:
    friend class PacketStream<P>;

    PacketStream<P> &ps;
    char *buf;
    int valid_packets{0};
};

template <class P> using PacketBlockPtr = std::unique_ptr<PacketBlock<P>>;

/**
 *@short manages packet stream with buffer & parallel read functionality
 */

template <class P> class PacketStream {

  public:
    PacketStream(UdpRxSocketPtr s, GeneralDataPtr d, FramePolicy fp, int idx,
                 cpu_set_t cpu_mask, unsigned long node_mask, int max_node);
    ~PacketStream();

    PacketBlockPtr<P> getPacketBlock(uint64_t frame);

    bool hasPendingPacket();
    void stop();
    bool wasStopped();

    int getNumPacketsCaught();
    uint64_t getNumFramesCaught();
    uint64_t getLastFrameIndex();

    void clearBuffer();

    void printStats();

  private:
    using StreamData = typename P::StreamData;
    friend class PacketBlock<P>;

    struct WriterThread;
    using PacketBlockMap = std::map<uint64_t, PacketBlockPtr<P>>;
    using MapIterator = typename PacketBlockMap::iterator;
    using FramePacketBlock = typename PacketBlockMap::value_type;

    uint32_t getNbPacketFrames();

    void initMem(unsigned long node_mask, int max_node);

    bool canDiscardFrame(int received_packets);

    PacketBlockPtr<P> getEmptyBlock();
    void addPacketBlock(FramePacketBlock &&frame_block);
    void releasePacketBlock(PacketBlock<P> &block);
    void releaseReadyPacketBlocks();
    void waitUsedPacketBlocks();

    UdpRxSocketPtr socket;
    GeneralDataPtr general_data;
    FramePolicy frame_policy;
    const unsigned int num_frames;
    std::mutex mutex;
    int packets_caught{0};
    uint64_t frames_caught{0};
    uint64_t last_frame{0};
    StreamData stream_data;
    int header_pad;
    int packet_len;
    MmappedRegion packet_buffer_array;
    std::mutex free_mutex;
    std::condition_variable free_cond;
    std::queue<char *> free_queue;
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

const int MaxNbPorts = 2;

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

/**
 *@short Default frame assembler in Listener
 */

namespace Eiger {
class StdFrameAssembler;
}
namespace Jungfrau {
template <int NbUDPIfaces> class StdFrameAssembler;
}

class DefaultFrameAssemblerBase : public FrameAssemblerBase {

  public:
    using Ptr = std::shared_ptr<DefaultFrameAssemblerBase>;

    DefaultFrameAssemblerBase(GeneralDataPtr d, bool e4b);

    GeneralDataPtr getGeneralData();
    bool doExpand4Bits();

    virtual bool hasPendingPacket() = 0;
    virtual int getImageSize() = 0;

    virtual int getNumPacketsCaught() = 0;
    virtual uint64_t getNumFramesCaught() = 0;
    virtual uint64_t getLastFrameIndex() = 0;

    virtual void clearBuffers() = 0;

    virtual void printStreamStats() = 0;

    static Ptr create(UdpRxSocketPtr s, GeneralDataPtr d, int idx,
                      cpu_set_t cpu_mask, unsigned long node_mask, int max_node,
                      FramePolicy fp, bool e4b);

  protected:
    GeneralDataPtr general_data;
    bool expand_4bits;
};

template <class P>
class DefaultFrameAssembler : public DefaultFrameAssemblerBase {

  public:
    DefaultFrameAssembler(UdpRxSocketPtr s, GeneralDataPtr d, int idx,
                          cpu_set_t cpu_mask, unsigned long node_mask,
                          int max_node, FramePolicy fp, bool e4b);

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

  protected:
    friend class Eiger::StdFrameAssembler;
    template <int NbUDPIfaces> friend class Jungfrau::StdFrameAssembler;

    using PacketStreamPtr = std::unique_ptr<PacketStream<P>>;

    void expand4Bits(char *dst, char *src, int src_size);

    PacketStreamPtr packet_stream;
    FramePolicy frame_policy;
};

using LegacyAssembler = DefaultFrameAssembler<LegacyPacket>;
using GotthardAssembler = DefaultFrameAssembler<GotthardPacket>;

/**
 *@short assembles frame data from 2 udp sockets into memory
 */

class DualPortFrameAssembler : public FrameAssemblerBase {

  public:
    using PortsMask = std::bitset<2>;

    DualPortFrameAssembler(DefaultFrameAssemblerBase::Ptr a[2]);

    void stop() override;

  protected:
    void stopAssemblers();

    DefaultFrameAssemblerBase::Ptr assembler[2];
};

} // namespace FrameAssembler

#include "FrameAssemblerEiger.hxx"
#include "FrameAssemblerJungfrau.hxx"
