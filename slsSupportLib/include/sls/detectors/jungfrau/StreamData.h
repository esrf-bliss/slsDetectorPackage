#pragma once
/************************************************
 * @file StreamData.h
 * @short Jungfrau stream data definitions
 ***********************************************/

#include <sls/StreamData.h>

namespace sls {
namespace Jungfrau {

/**
 * Sequence of Jungfrau packets
 */
template <typename NbUDPIfaces, int Idx>
struct StreamData : sls::StreamData<Packet<NbUDPIfaces>> {

    static constexpr bool OneIface = (NbUDPIfaces::NbIfaces == 1);
    static constexpr int IfacePackets =
        PacketData<NbUDPIfaces>::PacketsPerFrame;
    static constexpr int DirBottom = 1;
    static constexpr int DirTop = -1;
    static constexpr int FirstBottom = OneIface ? (IfacePackets / 2) : 0;
    static constexpr int FirstTop = (OneIface ? FirstBottom : IfacePackets) - 1;

    constexpr uint32_t getAbsPacket(uint32_t idx, int first, int dir) {
        return first + idx * dir;
    }

    constexpr uint32_t getPacketNumber(uint32_t packet_idx) {
        if (OneIface) {
            bool top = ((packet_idx % 2) == 0);
            int rel_row = packet_idx / 2;
            int first = top ? FirstTop : FirstBottom;
            int dir = top ? DirTop : DirBottom;
            return getAbsPacket(rel_row, first, dir);
        } else if (Idx == 0) {
            return getAbsPacket(packet_idx, FirstTop, DirTop);
        } else {
            return getAbsPacket(packet_idx, FirstBottom, DirBottom);
        }
    }

    constexpr uint32_t getRelIndex(uint32_t packet, int first, int dir) {
        return (int(packet) - first) / dir;
    }

    constexpr uint32_t getPacketIndex(uint32_t packet_nb) {
        if (OneIface) {
            bool top = packet_nb <= FirstTop;
            int first = top ? FirstTop : FirstBottom;
            int dir = top ? DirTop : DirBottom;
            int rel_row = getRelIndex(packet_nb, first, dir);
            return rel_row * 2 + (top ? 0 : 1);
        } else if (Idx == 0) {
            return getRelIndex(packet_nb, FirstTop, DirTop);
        } else {
            return getRelIndex(packet_nb, FirstBottom, DirBottom);
        }
    }
};

template <typename NbUDPIfaces, int Idx, class FP>
using PacketStream = ::PacketStream<PacketContainer<Packet<NbUDPIfaces>>,
                                    StreamData<NbUDPIfaces, Idx>, FP>;

} // namespace Jungfrau
} // namespace sls
