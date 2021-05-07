#pragma once
/************************************************
 * @file PacketTypedefs.h
 * @short low-level udp packet definition classes
 ***********************************************/

#include "sls/Packet.h"
#include "sls/detectors/eiger/Packet.h"
#include "sls/detectors/gotthard/Packet.h"
#include "sls/detectors/jungfrau/Packet.h"

namespace sls {

// Only 10G supported so far
#define SLS_EIGER_DEFINE(Pixel, Interface)                                     \
    using EigerPacket##Pixel##Interface =                                      \
        Eiger::Packet<Geom::Pixel, Eiger::Interface>;                          \
    using EigerPacketBlock##Pixel##Interface =                                 \
        PacketBlock<Eiger::Packet<Geom::Pixel, Eiger::Interface>>;             \
    using EigerPacketBlockPtr##Pixel##Interface =                              \
        PacketBlockPtr<Eiger::Packet<Geom::Pixel, Eiger::Interface>>;

#define SLS_EIGER_DEFINE_PIXEL(Pixel)                                          \
    SLS_EIGER_DEFINE(Pixel, TenGigaDisable)                                    \
    SLS_EIGER_DEFINE(Pixel, TenGigaEnable)

SLS_EIGER_DEFINE_PIXEL(Pixel4);
SLS_EIGER_DEFINE_PIXEL(Pixel8);
SLS_EIGER_DEFINE_PIXEL(Pixel16);
SLS_EIGER_DEFINE_PIXEL(Pixel32);

#define SLS_JUNGFRAU_DEFINE(NbUDPIfaces)                                       \
    using JungfrauPacket##NbUDPIfaces =                                        \
        Jungfrau::Packet<Jungfrau::NbUDPIfaces>;                               \
    using JungfrauPacketBlock##NbUDPIfaces =                                   \
        PacketBlock<Jungfrau::Packet<Jungfrau::NbUDPIfaces>>;                  \
    using JungfrauPacketBlockPtr##NbUDPIfaces =                                \
        PacketBlockPtr<Jungfrau::Packet<Jungfrau::NbUDPIfaces>>;

SLS_JUNGFRAU_DEFINE(OneIface);
SLS_JUNGFRAU_DEFINE(TwoIface);

using AnyPacketBlockPtr = std::variant<EigerPacketBlockPtrPixel4TenGigaDisable,
                                       EigerPacketBlockPtrPixel4TenGigaEnable,
                                       EigerPacketBlockPtrPixel8TenGigaDisable,
                                       EigerPacketBlockPtrPixel8TenGigaEnable,
                                       EigerPacketBlockPtrPixel16TenGigaDisable,
                                       EigerPacketBlockPtrPixel16TenGigaEnable,
                                       EigerPacketBlockPtrPixel32TenGigaDisable,
                                       EigerPacketBlockPtrPixel32TenGigaEnable,
                                       JungfrauPacketBlockPtrOneIface,
                                       JungfrauPacketBlockPtrTwoIface>;

using AnyPacketBlockList = std::vector<AnyPacketBlockPtr>;

} // namespace sls
