/************************************************
 * @file FrameAssemblerJungfrau.hxx
 * @short helper classes assembling Jungfrau frames
 * from udp packets
 * DO NOT INCLUDE THIS FILE DIRECTLY IN YOUR CODE
 * include "FrameAssembler.h" instead
 ***********************************************/

namespace FrameAssembler {
namespace Jungfrau {

constexpr int PacketDataLen = 8192;

using Pixel = Pixel16;

constexpr int PacketPixels = PacketDataLen / Pixel::depth();

template <int NbUDPIfaces>
using Jungfrau500kGeom = sls::Geom::Jungfrau::Jungfrau500kGeom<NbUDPIfaces>;

template <int NbUDPIfaces, int Idx>
constexpr auto RawIfaceGeom =
    Jungfrau500kGeom<NbUDPIfaces>::template RawIfaceGeom<Idx>::geom;

template <int NbUDPIfaces>
constexpr int
    FramePackets = RawIfaceGeom<NbUDPIfaces, 0>.calcFramePackets(PacketPixels);

template <int NbUDPIfaces>
using PacketData = StdPacketData<PacketDataLen, FramePackets<NbUDPIfaces>>;

template <int NbUDPIfaces> using Packet = StdPacket<PacketData<NbUDPIfaces>>;

template <int NbUDPIfaces, int Idx>
struct StreamData : FrameAssembler::StreamData<Packet<NbUDPIfaces>> {
    uint32_t getPacketNumber(uint32_t packet_idx);
};

template <int NbUDPIfaces, int Idx, class FP>
using AssemblerBase =
    DefaultFrameAssembler<Packet<NbUDPIfaces>, StreamData<NbUDPIfaces, Idx>, FP,
                          Pixel>;

template <int NbUDPIfaces, int Idx, class FP>
struct Assembler : AssemblerBase<NbUDPIfaces, Idx, FP> {
    using AssemblerBase<NbUDPIfaces, Idx, FP>::AssemblerBase;
};

/**
 *@short Jungfrau frame assembler in standard mode: Default frame assembler
 */

//  FP: Frame discard policy, GD: Geom data, MGX/Y: Module gap X/Y
template <class GD, bool MGX, bool MGY, int Idx> struct CopyHelper;

template <class FP, class GD, bool MGX, bool MGY>
class FrameAssembler : public FrameAssemblerBase {
  public:
    static constexpr int NbUDPIfaces = GD::num_udp_ifaces;

    template <int Idx> using RealAssembler = Assembler<NbUDPIfaces, Idx, FP>;
    template <int Idx>
    using Stream =
        std::decay_t<decltype(std::declval<RealAssembler<Idx>>().getStream())>;

    using StreamList1 = std::tuple<Stream<0> *>;
    using StreamList2 = std::tuple<Stream<0> *, Stream<1> *>;
    using StreamList =
        std::conditional_t<NbUDPIfaces == 1, StreamList1, StreamList2>;

    FrameAssembler(StreamList s, int offset) : stream(s), data_offset(offset) {}

    Result assembleFrame(uint64_t frame, RecvHeader *recv_header,
                         char *buf) override;
    void stop() override;

    template <int Idx>
    static auto rawAssemblerStream(DefaultFrameAssemblerPtr a) {
        return &dynamic_cast<RealAssembler<Idx> *>(a.get())->getStream();
    }

  private:
    StreamList stream;
    int data_offset;

    struct Worker {
        PortsMask mask;
        bool header_empty{true};
        uint64_t frame;
        DetHeader *det_header;
        char *buf;

        Worker(uint64_t f, RecvHeader *rh, char *b)
            : frame(f), det_header(&rh->detHeader), buf(b) {
            det_header->frameNumber = frame;
            det_header->packetNumber = 0;
        }

        template <int Idx> void assembleIface(Stream<Idx> *s);

        Result result();
    };
};

using XY = sls::Geom::XY;
FrameAssemblerPtr CreateFrameAssembler(XY det_ifaces, XY mod_pos,
                                       int num_udp_ifaces, FramePolicy fp,
                                       DefaultFrameAssemblerList a);

} // namespace Jungfrau
} // namespace FrameAssembler
