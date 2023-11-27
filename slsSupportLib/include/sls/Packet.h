#pragma once
/************************************************
 * @file Packet.h
 * @short low-level udp packet definition classes
 ***********************************************/

#include "sls/Align.h"
#include "sls/sls_detector_defs.h"

#include <functional>
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
        int valid;
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

    Layout *layout;
    SoftwarePacket *buffer;

    Packet(Layout *l) : layout(l), buffer(&l->soft_packet) {}

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
    using NetworkHeader = typename Packet::NetworkPacketHeader;
    using sls_bitset = slsDetectorDefs::sls_bitset;

    PacketBlock(LayoutPtr &&l) : layout(std::move(l)) {}

    Packet operator[](unsigned int i) const { return Packet(&(*layout)[i]); }

    void setValid(unsigned int i, bool valid) {
        Packet p = (*this)[i];
        p.softHeader()->valid = valid;
        if (!valid) // default valid mask is false: nothing to do
            return;
        valid_packet_mask[i] = true;
        if (!header || (p.number() < header->packetNumber))
            header = p.networkHeader();
    }

    void moveToGood(Packet &p) {
        P dst = (*this)[p.number()];
        *dst.buffer = *p.buffer;
        setValid(p.number(), true);
    }

    bool hasFullFrame() const { return getValidPackets() == NbPackets; }

    int getValidPackets() const { return valid_packet_mask.count(); }

    const sls_bitset &getValidPacketMask() const { return valid_packet_mask; }

    NetworkHeader *getNetworkHeader() const { return header; }

    uint64_t getDetFrameNumber() const {
        return header ? header->frameNumber : -1;
    }

    uint64_t getRecvFrameNumber() const { return recv_frame_number; }
    void setRecvFrameNumber(uint64_t frame) { recv_frame_number = frame; }

    void discard() { valid_packet_mask.reset(); }

  private:
    LayoutPtr layout;
    sls_bitset valid_packet_mask;
    NetworkHeader *header{nullptr};
    uint64_t recv_frame_number{uint64_t(-1)};
};

template <class P> using PacketBlockPtr = std::unique_ptr<PacketBlock<P>>;

} // namespace sls
