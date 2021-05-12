#pragma once
/************************************************
 * @file PacketJungFrau.h
 * @short Jungfrau packet definitions
 ***********************************************/

#include "sls/Packet.h"
#include "sls/detectors/jungfrau/Geometry.h"

namespace sls {
namespace Jungfrau {

constexpr int PacketDataLen = 8192;

using Pixel = Pixel16;

template <typename NbUDPIfaces>
using Jungfrau500kGeom = Geom::Jungfrau500kGeom<NbUDPIfaces>;

template <typename NbUDPIfaces, int Idx>
constexpr auto RawIfaceGeom =
    Geom::Jungfrau500kGeom<NbUDPIfaces>::template RawIfaceGeom<Idx>::geom;

template <typename NbUDPIfaces>
constexpr auto FramePixels = RawIfaceGeom<NbUDPIfaces, 0>.pixels();

template <typename NbUDPIfaces>
struct PacketData
    : StdPacketData<Pixel, PacketDataLen, FramePixels<NbUDPIfaces>> {
    static constexpr int NbIfaces = NbUDPIfaces::NbIfaces;
};

template <typename NbUDPIfaces>
using Packet = StdPacket<PacketData<NbUDPIfaces>>;

} // namespace Jungfrau
} // namespace sls
