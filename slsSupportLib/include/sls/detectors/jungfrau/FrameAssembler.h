#pragma once
/************************************************
 * @file Jungfrau/FrameAssembler.h
 * @short helper classes assembling Jungfrau frames
 * from udp packets
 ***********************************************/

#include "sls/FrameAssembler.h"
#include "sls/detectors/jungfrau/Packet.h"

namespace sls {
namespace Jungfrau {
namespace FrameAssembler {

using namespace sls::FrameAssembler;

/**
 *@short Jungfrau frame assembler in standard mode: Default frame assembler
 */

//  GD: Geom data, MGX/Y: Module gap X/Y
template <class GD, bool MGX, bool MGY, int Idx> struct CopyHelper;

template <class GD, bool MGX, bool MGY>
class FrameAssembler : public MPFrameAssembler {
  public:
    using NbUDPIfaces = typename GD::num_udp_ifaces;
    static constexpr int NbIfaces = NbUDPIfaces::NbIfaces;

    FrameAssembler(int offset) : data_offset(offset) {}

    Result assembleFrame(const AnyPacketBlockList &blocks, char *buf) override;
    FrameDims getAssembledFrameDims() override;

  private:
    template <int Idx>
    bool assembleIface(const AnyPacketBlockPtr &block, char *buf);

    int data_offset;
};

using XY = sls::Geom::XY;
MPFrameAssemblerPtr CreateFrameAssembler(int mod_ifaces, XY det_ifaces,
                                         XY mod_pos);

} // namespace FrameAssembler
} // namespace Jungfrau
} // namespace sls

#include "sls/detectors/jungfrau/FrameAssembler.cxx"
