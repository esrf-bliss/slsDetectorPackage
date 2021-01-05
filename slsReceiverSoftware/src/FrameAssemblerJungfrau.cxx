/************************************************
 * @file FrameAssemblerJungfrau.cxx
 * @short helper classes assembling Jungfrau frames
 * from udp packets
 * This file is include in FrameAssembler.cpp
 ***********************************************/

namespace FrameAssembler {
namespace Jungfrau {

/**
 * Sequence of Jungfrau packets
 */

template <int NbUDPIfaces>
inline uint32_t
PacketData<NbUDPIfaces>::StreamData::getPacketNumber(uint32_t packet_idx) {
    const int FramePackets = 128;
    const int FirstBottom = FramePackets / 2;
    const int FirstTop = FirstBottom - 1;
    const int DirBottom = 1;
    const int DirTop = -1;
    if (NbUDPIfaces == 1) {
        bool top = ((packet_idx % 2) == 0);
        int rel_row = packet_idx / 2;
        int first = top ? FirstTop : FirstBottom;
        int dir = top ? DirTop : DirBottom;
        return first + rel_row * dir;
    } else if (stream_idx == 0) {
        return FirstTop + packet_idx * DirTop;
    } else {
        return packet_idx;
    }
}

/**
 * RawFrameAssembler
 */

template <int NbUDPIfaces>
RawFrameAssembler<NbUDPIfaces>::RawFrameAssembler(
    DefaultFrameAssemblerBase::Ptr a[NbUDPIfaces]) {
    for (int i = 0; i < NbUDPIfaces; ++i)
        assembler[i] = a[i];
}

template <int NbUDPIfaces>
Result RawFrameAssembler<NbUDPIfaces>::assembleFrame(uint64_t frame,
                                                     RecvHeader *recv_header,
                                                     char *buf) {
    Result res{NbUDPIfaces, 0};
    for (int i = 0; i < NbUDPIfaces; ++i) {
        Result r = assembler[i]->assembleFrame(frame, recv_header, buf);
        res.valid_data[i] = r.valid_data[0];
        if ((NbUDPIfaces == 2) && buf)
            buf += assembler[i]->getImageSize();
    }
    return res;
}

template <int NbUDPIfaces> void RawFrameAssembler<NbUDPIfaces>::stop() {
    for (int i = 0; i < NbUDPIfaces; ++i)
        assembler[i]->stop();
}

// template instantiation
template class RawFrameAssembler<1>;
template class RawFrameAssembler<2>;

/**
 * StdFrameAssembler
 */

template <int NbUDPIfaces> class StdFrameAssembler<NbUDPIfaces>::Helper {
  public:
    using Packet = Jungfrau::Packet<NbUDPIfaces>;
    using BlockPtr = PacketBlockPtr<Packet>;
    using Stream = PacketStream<Packet>;

    Helper(GeneralDataPtr gd);
    virtual ~Helper() {}

    float getSrcPixelBytes() { return general_data->GetPixelDepth(); }

    float getDstPixelBytes() {
        int factor = (general_data->dynamicRange == 4) ? 2 : 1;
        return getSrcPixelBytes() * factor;
    }

    virtual void assemblePackets(BlockPtr block[NbUDPIfaces], char *buf) = 0;

  protected:
    static const int chip_size = 256;
    static const int chip_gap = 2;
    static const int nb_horz_chips = 4;
    static const int nb_vert_chips = 2;

    struct Geometry {
        float pixel_size;
        int chip_size;
        int line_size;
    };

    GeneralDataPtr general_data;
    int frame_packets;
    int packet_lines;
    Geometry src;
    Geometry dst;
};

template <int NbUDPIfaces>
StdFrameAssembler<NbUDPIfaces>::Helper::Helper(GeneralDataPtr gd)
    : general_data(gd) {
    frame_packets = gd->packetsPerFrame;
    int port_vert_chips = nb_vert_chips / NbUDPIfaces;
    packet_lines = port_vert_chips * chip_size / frame_packets;
    src.pixel_size = getSrcPixelBytes();
    src.chip_size = chip_size * src.pixel_size;
    src.line_size = nb_horz_chips * src.chip_size;
    dst.pixel_size = getDstPixelBytes();
    dst.chip_size = (chip_size + chip_gap) * dst.pixel_size;
    dst.line_size = gd->nPixelsX * dst.pixel_size;
}

template <int NbUDPIfaces>
class CopyHelper : public StdFrameAssembler<NbUDPIfaces>::Helper {
  public:
    using Helper = typename StdFrameAssembler<NbUDPIfaces>::Helper;
    using BlockPtr = typename Helper::BlockPtr;

    CopyHelper(GeneralDataPtr gd) : Helper(gd) {}

    virtual void assemblePackets(BlockPtr block[NbUDPIfaces],
                                 char *buf) override;
};

template <int NbUDPIfaces>
void CopyHelper<NbUDPIfaces>::assemblePackets(BlockPtr block[NbUDPIfaces],
                                              char *buf) {
    using H = Helper;
    char *d = buf;
    int line = 0;
    for (int i = 0; i < NbUDPIfaces; ++i) {
        BlockPtr b = std::move(block[i]);
        for (int p = 0; p < H::frame_packets; ++p) {
            if ((line > 0) && ((line % H::chip_size) == 0))
                d += H::dst.line_size * H::chip_gap;
            typename H::Packet line_packet = (*b)[p];
            char *s = line_packet.data();
            for (int l = 0; l < H::packet_lines; ++l, ++line) {
                char *ld = d;
                char *ls = s;
                for (int c = 0; c < H::nb_horz_chips; ++c) {
                    if (line_packet.valid())
                        memcpy(ld, ls, H::src.chip_size);
                    else
                        memset(ld, 0xff, H::src.chip_size);
                    ls += H::src.chip_size;
                    ld += H::dst.chip_size;
                }
                s += H::src.line_size;
                d += H::dst.line_size;
            }
        }
    }
}

template <int NbUDPIfaces>
StdFrameAssembler<NbUDPIfaces>::StdFrameAssembler(
    DefaultFrameAssemblerBase::Ptr a[NbUDPIfaces]) {
    for (int i = 0; i < NbUDPIfaces; ++i)
        assembler[i] = a[i];
    helper = new CopyHelper<NbUDPIfaces>(assembler[0]->getGeneralData());
}

template <int NbUDPIfaces>
StdFrameAssembler<NbUDPIfaces>::~StdFrameAssembler() {
    delete helper;
}

template <int NbUDPIfaces>
Result StdFrameAssembler<NbUDPIfaces>::assembleFrame(uint64_t frame,
                                                     RecvHeader *recv_header,
                                                     char *buf) {
    Result res{NbUDPIfaces, 0};

    DetHeader *det_header = &recv_header->detHeader;
    det_header->frameNumber = frame;
    det_header->packetNumber = 0;

    using BlockPtr = typename Helper::BlockPtr;
    using Packet = typename Helper::Packet;
    using Stream = typename Helper::Stream;
    using Assembler = Jungfrau::Assembler<NbUDPIfaces>;

    BlockPtr block[NbUDPIfaces];

    for (int i = 0; i < NbUDPIfaces; ++i) {
        Assembler *a = reinterpret_cast<Assembler *>(assembler[i].get());
        Stream *ps = a->packet_stream.get();
        block[i] = std::move(ps->getPacketBlock(frame));
        int packet_count = block[i] ? block[i]->getValidPackets() : 0;
        res.valid_data[i] = (packet_count > 0);
        if (!res.valid_data[i])
            continue;

        det_header->packetNumber += packet_count;

        // write header
        Packet p = (*block[i])[0];
        p.fillDetHeader(det_header);
    }

    if (res.valid_data.any() && buf)
        helper->assemblePackets(block, buf);

    return res;
}

template <int NbUDPIfaces> void StdFrameAssembler<NbUDPIfaces>::stop() {
    for (int i = 0; i < NbUDPIfaces; ++i)
        assembler[i]->stop();
}

// template instantiation
template class StdFrameAssembler<1>;
template class StdFrameAssembler<2>;

} // namespace Jungfrau
} // namespace FrameAssembler
