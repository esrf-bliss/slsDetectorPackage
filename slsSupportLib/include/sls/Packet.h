#pragma once
/************************************************
 * @file Packet.h
 * @short low-level udp packet definition classes
 ***********************************************/

#include "sls/Geometry.h"
#include "sls/sls_detector_defs.h"

#include <memory>

using DetHeader = slsDetectorDefs::sls_detector_header;
using RecvHeader = slsDetectorDefs::sls_receiver_header;

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

template <class P, int DataLen, class NetworkHeader, int FramePixels,
          int Align = 16>
struct PacketData {
    using Pixel = P;
    static constexpr int PacketDataLen = DataLen;
    static constexpr int PacketPixels = PacketDataLen / Pixel::depth();
    static constexpr int FrameLen = FramePixels * Pixel::depth();
    static constexpr int PacketsPerFrame = FrameLen / PacketDataLen;

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
    static constexpr int Pad =
        sls::Geom::AlignCeil(DataOffset, Align) - DataOffset;

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

template <class Pixel, int DataLen, int FramePixels>
using StdPacketData = PacketData<Pixel, DataLen, DetHeader, FramePixels>;

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
    using Packet = P;
    static constexpr int NbPackets = Packet::Data::PacketsPerFrame;
    using Layout = std::array<typename Packet::Layout, NbPackets>;
    using LayoutPtr = std::unique_ptr<Layout, std::function<void(Layout *)>>;

    PacketBlock(LayoutPtr &&l) : layout(std::move(l)){};

    Packet operator[](unsigned int i) { return Packet(&(*layout)[i]); }

    void setValid(unsigned int i, bool valid);

    void moveToGood(Packet &p);

    bool hasFullFrame() { return valid_packets == NbPackets; }

    int getValidPackets() { return valid_packets; }

    uint64_t frame_number{0};

  private:
    LayoutPtr layout;
    int valid_packets{0};
};

template <class P> using PacketBlockPtr = std::unique_ptr<PacketBlock<P>>;

/*
 * Eiger packet definitions
 */

namespace Eiger {

constexpr int NbIfaces = sls::Geom::Eiger::RecvIfaces.x;

struct TenGigaDisable {
    static constexpr int PacketDataLen = 1024;
};
struct TenGigaEnable {
    static constexpr int PacketDataLen = 4096;
};

using AnyTenGiga = std::variant<TenGigaDisable, TenGigaEnable>;

inline AnyTenGiga AnyTenGigaFromTgEnable(bool tg_enable) {
    if (tg_enable)
        return TenGigaEnable();
    else
        return TenGigaDisable();
}

using Eiger500kGeom = sls::Geom::Eiger::Eiger500kGeom;
constexpr auto RawIfaceGeom = Eiger500kGeom::RawIfaceGeom::geom;

constexpr auto FramePixels = RawIfaceGeom.pixels();

template <class Pixel, class TenGiga>
using PacketData = ::StdPacketData<Pixel, TenGiga::PacketDataLen, FramePixels>;

template <class Pixel, class TenGiga>
using Packet = ::StdPacket<PacketData<Pixel, TenGiga>>;

// Only 10G supported so far
#define EigerPacketFor(P, T) ::Eiger::Packet<P, T>

#define EigerPacketBlockPtrsFor(P)                                             \
    PacketBlockPtr<EigerPacketFor(P, ::Eiger::TenGigaDisable)>,                \
        PacketBlockPtr<EigerPacketFor(P, ::Eiger::TenGigaEnable)>

#define EigerPacketBlockPtrs                                                   \
    EigerPacketBlockPtrsFor(sls::Geom::Pixel4),                                \
        EigerPacketBlockPtrsFor(sls::Geom::Pixel8),                            \
        EigerPacketBlockPtrsFor(sls::Geom::Pixel16),                           \
        EigerPacketBlockPtrsFor(sls::Geom::Pixel32)

} // namespace Eiger

/*
 * Jungfrau packet definitions
 */

namespace Jungfrau {

constexpr int PacketDataLen = 8192;

using Pixel = sls::Geom::Pixel16;

template <int NbUDPIfaces>
using Jungfrau500kGeom = sls::Geom::Jungfrau::Jungfrau500kGeom<NbUDPIfaces>;

template <int NbUDPIfaces, int Idx>
constexpr auto RawIfaceGeom =
    Jungfrau500kGeom<NbUDPIfaces>::template RawIfaceGeom<Idx>::geom;

template <int NbUDPIfaces>
constexpr auto FramePixels = RawIfaceGeom<NbUDPIfaces, 0>.pixels();

template <int NbUDPIfaces>
struct PacketData
    : StdPacketData<Pixel, PacketDataLen, FramePixels<NbUDPIfaces>> {
    static constexpr int NbIfaces = NbUDPIfaces;
};

template <int NbUDPIfaces> using Packet = StdPacket<PacketData<NbUDPIfaces>>;

#define JungfrauPacketBlockPtrs                                                \
    PacketBlockPtr<::Jungfrau::Packet<1>>, PacketBlockPtr<::Jungfrau::Packet<2>>

} // namespace Jungfrau

/*
 * Gotthard packet definitions
 * TODO: update to new Packet/Data/StreamData interface
 */

namespace Gotthard {

/**
 *@short Gotthard Packet class
 *
 * Gotthard Full mode data:
 *   1st packet: CACA + CACA, (640 - 1) * 2 bytes data
 *   2nd packet: (2 + 640 - 1) * 2 bytes data
 *
 * Gotthard Roi mode data:
 *   1st packet: CACA + CACA, (256 - 1) * 2 bytes data
 */

using Pixel = sls::Geom::Pixel16;

struct NetworkHeader {
    uint32_t packet_number;
    uint32_t sign_data;
} __attribute__((packed));

struct FullMode {
    static constexpr int PacketDataLen = (640 - 1) * 2;
    static constexpr int PacketsPerFrame = 2;
    static constexpr int FramePixels = 640 * PacketsPerFrame;

    using PacketDataBase =
        ::PacketData<Pixel, PacketDataLen, NetworkHeader, FramePixels>;
    struct StreamData {
        bool inited{false};
        int packet_offset;

        void init(NetworkHeader *network_header) {
            if (!inited) {
                bool first_packet = (network_header->sign_data == 0xCACACACA);
                bool odd_number = network_header->packet_number & 1;
                packet_offset = (first_packet == odd_number) ? 1 : 0;
                inited = true;
            }
        }

        uint32_t correctFramePacket(NetworkHeader *network_header) {
            return network_header->packet_number + packet_offset;
        }
        int getFrameNumber(NetworkHeader *network_header) {
            return correctFramePacket(network_header) / PacketsPerFrame;
        }
        int getPacketNumber(NetworkHeader *network_header) {
            return correctFramePacket(network_header) % PacketsPerFrame;
        }
    };
};

struct RoiMode {
    static constexpr int PacketDataLen = (256 - 1) * 2;
    static constexpr int FramePixels = 256;

    using PacketDataBase =
        ::PacketData<Pixel, PacketDataLen, NetworkHeader, FramePixels>;
    struct StreamData {
        int getFrameNumber(NetworkHeader *network_header) {
            return network_header->packet_number;
        }
        int getPacketNumber(NetworkHeader * /*network_header*/) { return 0; }
    };
};

template <class Mode> using PacketData = typename Mode::PacketDataBase;

template <class Mode> struct PacketImpl : ::Packet<PacketData<Mode>> {
    using Base = ::Packet<PacketData<Mode>>;
    using StreamData = typename Mode::StreamData;

    StreamData &stream_data;

    PacketImpl(char *b, StreamData &sd) : Base(b, sd), stream_data(sd) {}

    void initSoftHeader() { stream_data.init(Base::networkHeader()); }

    uint64_t frame() {
        return stream_data.getFrameNumber(Base::networkHeader());
    }

    uint32_t number() {
        return stream_data.getPacketNumber(Base::networkHeader());
    }

    uint32_t sizeAdjust() { return (number() == 0) ? 0 : (2 * 2); }

    void fillDetHeader(::DetHeader *det_header);
};

using FullPacket = PacketImpl<FullMode>;
using RoiPacket = PacketImpl<RoiMode>;

} // namespace Gotthard

// AnyPacketBlockPtr, AnyPacketBlockList & AnyPacketStream
using AnyPacketBlockPtr =
    std::variant<EigerPacketBlockPtrs, JungfrauPacketBlockPtrs>;
using AnyPacketBlockList = std::vector<AnyPacketBlockPtr>;

#undef JungfrauPacketBlockPtrs
#undef EigerPacketBlockPtrs
#undef EigerPacketBlockPtrsFor

#include "Packet.cxx"
