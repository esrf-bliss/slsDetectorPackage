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
    uint32_t getPacketNumber(uint32_t packet_idx) {
        constexpr int FramePackets = PacketData<NbUDPIfaces>::PacketsPerFrame;
        constexpr int DirBottom = 1;
        constexpr int DirTop = -1;
        if constexpr (NbUDPIfaces::NbIfaces == 1) {
            constexpr int FirstBottom = FramePackets / 2;
            constexpr int FirstTop = FirstBottom - 1;
            bool top = ((packet_idx % 2) == 0);
            int rel_row = packet_idx / 2;
            int first = top ? FirstTop : FirstBottom;
            int dir = top ? DirTop : DirBottom;
            return first + rel_row * dir;
        } else if constexpr (Idx == 0) {
            constexpr int FirstTop = FramePackets - 1;
            return FirstTop + packet_idx * DirTop;
        } else {
            return packet_idx;
        }
    }
};

template <typename NbUDPIfaces, int Idx, class FP>
using PacketStream =
    ::PacketStream<Packet<NbUDPIfaces>, StreamData<NbUDPIfaces, Idx>, FP>;

} // namespace Jungfrau
} // namespace sls
