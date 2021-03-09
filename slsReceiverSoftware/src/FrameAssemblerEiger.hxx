/************************************************
 * @file FrameAssemblerEiger.hxx
 * @short helper classes assembling Eiger frames
 * from udp packets
 * DO NOT INCLUDE THIS FILE DIRECTLY IN YOUR CODE
 * include "FrameAssembler.h" instead
 ***********************************************/

namespace FrameAssembler {

namespace Eiger {

constexpr int NbIfaces = sls::Geom::Eiger::RecvIfaces.x;

struct TenGigaDisable {
    static constexpr int PacketDataLen = 1024;
};
struct TenGigaEnable {
    static constexpr int PacketDataLen = 4096;
};

using AnyTenGiga = std::variant<TenGigaDisable, TenGigaEnable>;

using Eiger500kGeom = sls::Geom::Eiger::Eiger500kGeom;
constexpr auto RawIfaceGeom = Eiger500kGeom::RawIfaceGeom::geom;

template <class Pixel, class TenGiga = TenGigaEnable>
constexpr int PacketPixels = TenGiga::PacketDataLen / Pixel::depth();

template <class Pixel, class TenGiga = TenGigaEnable>
constexpr int
    FramePackets = RawIfaceGeom.calcFramePackets(PacketPixels<Pixel, TenGiga>);

template <class Pixel, class TenGiga = TenGigaEnable>
using PacketData = FrameAssembler::StdPacketData<TenGiga::PacketDataLen,
                                                 FramePackets<Pixel, TenGiga>>;

template <class Pixel, class TenGiga = TenGigaEnable>
using Packet = FrameAssembler::StdPacket<PacketData<Pixel, TenGiga>>;

//  SP: Src Pixel, FP: Frame discard policy, DP: DstPixel
template <class SP, class FP, class DP = SP, class TenGiga = TenGigaEnable>
using Assembler = FrameAssembler::DefaultFrameAssembler<
    Packet<SP, TenGiga>, StreamData<Packet<SP, TenGiga>>, FP, SP, DP>;

/**
 *@short Eiger frame assembler in std mode: port interleaving
 */

//  P: Src Pixel, FP: Frame discard policy, GD: Geom data, MGX/Y: Module gap X/Y
template <class P, class GD, bool MGX, bool MGY, int Idx>
struct Expand4BitsHelper;
template <class P, class GD, bool MGX, bool MGY, int Idx> struct CopyHelper;

template <class P, class FP, class GD, bool MGX, bool MGY, int Idx>
class FrameAssembler : public FrameAssemblerBase {
  public:
    using Helper = std::conditional_t<std::is_same_v<P, Pixel4>,
                                      Expand4BitsHelper<P, GD, MGX, MGY, Idx>,
                                      CopyHelper<P, GD, MGX, MGY, Idx>>;
    using DP = typename Helper::DstPixel;
    using BlockPtr = typename Helper::BlockPtr;
    using RealAssembler = Assembler<P, FP, DP>;
    using Stream =
        std::decay_t<decltype(std::declval<RealAssembler>().getStream())>;
    using StreamList = std::array<Stream *, NbIfaces>;

    FrameAssembler(StreamList s, int offset) : stream(s), data_offset(offset) {}

    Result assembleFrame(uint64_t frame, RecvHeader *recv_header,
                         char *buf) override;
    void stop() override {
        for (auto s : stream)
            s->stop();
    }

    static auto rawAssemblerStream(DefaultFrameAssemblerPtr a) {
        return &dynamic_cast<RealAssembler *>(a.get())->getStream();
    }

  private:
    StreamList stream;
    Helper helper;
    int data_offset;
};

using XY = sls::Geom::XY;
FrameAssemblerPtr CreateFrameAssembler(int pixel_bpp, FramePolicy fp,
                                       bool enable_tg, XY det_ifaces,
                                       XY mod_pos, int recv_idx,
                                       DefaultFrameAssemblerList a);

} // namespace Eiger
} // namespace FrameAssembler