#pragma once
/************************************************
 * @file Packet.h
 * @short low-level udp packet definition classes
 ***********************************************/

#include "sls/Align.h"
#include "sls/sls_detector_defs.h"

#include <memory>

namespace sls {

using DetHeader = slsDetectorDefs::sls_detector_header;

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
    } __attribute__((packed));
    // The Packet structure in the (software) buffer
    struct SoftwarePacket {
        SoftHeader soft_header;
        NetworkPacket net_packet;
    } __attribute__((packed));

    static constexpr int DataOffset =
        offsetof(SoftwarePacket, net_packet) + offsetof(NetworkPacket, data);
    static constexpr int Pad = sls::alignCeil(DataOffset, Align) - DataOffset;

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

    SoftHeader const *softHeader() const { return &buffer->soft_header; }
    SoftHeader *softHeader() { return &buffer->soft_header; }

    void initSoftHeader() {}

    bool isValid() const { return softHeader()->valid; }

    void *networkBuffer() { return &buffer->net_packet; }

    NetworkPacketHeader *networkHeader() { return &buffer->net_packet.header; }

    char *data() { return buffer->net_packet.data; }
};

/**
 *@short StdPacket class
 */

// StdPacketData
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

    void setValid(unsigned int i, bool valid) {
        (*this)[i].softHeader()->valid = valid;
        if (valid)
            ++valid_packets;
    }

    void moveToGood(Packet &p) {
        P dst = (*this)[p.number()];
        *dst.buffer = *p.buffer;
        p.softHeader()->valid = false;
        dst.softHeader()->valid = true;
    }

    bool hasFullFrame() { return valid_packets == NbPackets; }

    int getValidPackets() { return valid_packets; }

    uint64_t frame_number{0};

  private:
    LayoutPtr layout;
    int valid_packets{0};
};

template <class P> using PacketBlockPtr = std::unique_ptr<PacketBlock<P>>;

} // namespace sls
