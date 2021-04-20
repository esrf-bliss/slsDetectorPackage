/************************************************
 * @file FrameAssemblerEiger.hxx
 * @short helper classes assembling Eiger frames
 * from udp packets
 * DO NOT INCLUDE THIS FILE DIRECTLY IN YOUR CODE
 * include "FrameAssembler.h" instead
 ***********************************************/

namespace FrameAssembler {
namespace Eiger {

// Import Eiger definitions
constexpr auto NbIfaces = ::Eiger::NbIfaces;

using TenGigaDisable = ::Eiger::TenGigaDisable;
using TenGigaEnable = ::Eiger::TenGigaEnable;

template <class Pixel, class TenGiga>
using Packet = ::Eiger::Packet<Pixel, TenGiga>;

/**
 *@short Eiger frame assembler in std mode: port interleaving
 */

//  P: Src Pixel, TG: Ten Giga, GD: Geom data, MGX/Y: Module gap X/Y
template <class P, class TG, class GD, bool MGX, bool MGY, int Idx>
struct Expand4BitsHelper;
template <class P, class TG, class GD, bool MGX, bool MGY, int Idx>
struct CopyHelper;

template <class P, class TG, class GD, bool MGX, bool MGY, int Idx>
class FrameAssembler : public MPFrameAssembler {
  public:
    using Helper =
        std::conditional_t<std::is_same_v<P, Pixel4>,
                           Expand4BitsHelper<P, TG, GD, MGX, MGY, Idx>,
                           CopyHelper<P, TG, GD, MGX, MGY, Idx>>;
    using DP = typename Helper::DstPixel;
    using BlockPtr = typename Helper::BlockPtr;

    FrameAssembler(int offset) : data_offset(offset) {}

    Result assembleFrame(AnyPacketBlockList &&blocks, RecvHeader *recv_header,
                         char *buf) override;

  private:
    Helper helper;
    int data_offset;
};

using XY = sls::Geom::XY;
MPFrameAssemblerPtr CreateFrameAssembler(uint32_t src_dr, bool tg_enable,
                                         XY det_ifaces, XY mod_pos,
                                         int recv_idx);

} // namespace Eiger
} // namespace FrameAssembler
