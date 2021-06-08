#pragma once
/************************************************
 * @file Packet.h
 * @short Eiger packet definitions
 ***********************************************/

#include "sls/Packet.h"
#include "sls/detectors/eiger/Geometry.h"

namespace sls {
namespace Eiger {

constexpr int NbIfaces = 2;

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

using Eiger500kGeom = Geom::Eiger500kGeom;
constexpr auto RawIfaceGeom = Eiger500kGeom::RawIfaceGeom::geom;

constexpr auto FramePixels = RawIfaceGeom.pixels();

template <class Pixel, class TenGiga>
using PacketData = StdPacketData<Pixel, TenGiga::PacketDataLen, FramePixels>;

template <class Pixel, class TenGiga>
using Packet = StdPacket<PacketData<Pixel, TenGiga>>;

} // namespace Eiger
} // namespace sls
