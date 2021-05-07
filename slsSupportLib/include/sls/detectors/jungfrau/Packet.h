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

using Pixel = Geom::Pixel16;

struct OneIface {
    static constexpr int NbIfaces = 1;
};
struct TwoIface {
    static constexpr int NbIfaces = 2;
};

template <int NbUDPIfaces>
using Jungfrau500kGeom = Geom::Jungfrau500kGeom<NbUDPIfaces>;

template <int NbUDPIfaces, int Idx>
constexpr auto RawIfaceGeom =
    Geom::Jungfrau500kGeom<NbUDPIfaces>::template RawIfaceGeom<Idx>::geom;

template <int NbUDPIfaces>
constexpr auto FramePixels = RawIfaceGeom<NbUDPIfaces, 0>.pixels();

template <typename NbUDPIfaces>
struct PacketData
    : StdPacketData<Pixel, PacketDataLen, FramePixels<NbUDPIfaces::NbIfaces>> {
    static constexpr int NbIfaces = NbUDPIfaces::NbIfaces;
};

template <typename NbUDPIfaces>
using Packet = StdPacket<PacketData<NbUDPIfaces>>;

} // namespace Jungfrau
} // namespace sls
