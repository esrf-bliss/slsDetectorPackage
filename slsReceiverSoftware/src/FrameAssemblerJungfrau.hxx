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

template <int NbUDPIfaces, int Idx>
constexpr auto IfaceGeom =
    sls::Geom::Jungfrau::IfaceGeom<NbUDPIfaces, Idx, RawFmt>;

template <int NbUDPIfaces>
constexpr int
    FramePackets = IfaceGeom<NbUDPIfaces, 0>.calcFramePackets(PacketPixels);

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

template <int NbUDPIfaces, int Idx> struct CopyHelper;

template <int NbUDPIfaces, class FP>
class FrameAssembler : public FrameAssemblerBase {
  public:
    template <int Idx> using RealAssembler = Assembler<NbUDPIfaces, Idx, FP>;
    template <int Idx>
    using Stream =
        std::decay_t<decltype(std::declval<RealAssembler<Idx>>().getStream())>;

    using StreamList1 = std::tuple<Stream<0> *>;
    using StreamList2 = std::tuple<Stream<0> *, Stream<1> *>;
    using StreamList =
        std::conditional_t<NbUDPIfaces == 1, StreamList1, StreamList2>;

    FrameAssembler(StreamList s) : stream(s) {}

    Result assembleFrame(uint64_t frame, RecvHeader *recv_header,
                         char *buf) override;
    void stop() override;

    template <int Idx>
    static auto rawAssemblerStream(DefaultFrameAssemblerPtr a) {
        return &dynamic_cast<RealAssembler<Idx> *>(a.get())->getStream();
    }

  private:
    StreamList stream;

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

FrameAssemblerPtr CreateFrameAssembler(int num_udp_ifaces, FramePolicy fp,
                                       DefaultFrameAssemblerList a);

} // namespace Jungfrau
} // namespace FrameAssembler
