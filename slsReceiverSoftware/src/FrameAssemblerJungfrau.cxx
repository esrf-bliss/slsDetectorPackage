/************************************************
 * @file FrameAssemblerJungfrau.cxx
 * @short helper classes assembling Jungfrau frames
 * from udp packets
 * This file is include in FrameAssembler.cpp
 ***********************************************/

/**
 * Sequence of Jungfrau packets
 */

inline uint32_t
Jungfrau::PacketData::StreamData::getPacketNumber(uint32_t packet_idx) {
    const int FramePackets = 128;
    bool top = ((packet_idx % 2) == 0);
    int rel_row = packet_idx / 2;
    int first = FramePackets / 2 - (top ? 1 : 0);
    int dir = top ? -1 : 1;
    return first + rel_row * dir;
}

/**
 * Jungfrau::RawFrameAssembler
 */

Jungfrau::RawFrameAssembler::RawFrameAssembler(DefaultFrameAssemblerBase::Ptr a)
    : assembler(a) {}

Result Jungfrau::RawFrameAssembler::assembleFrame(uint64_t frame,
                                                  RecvHeader *recv_header,
                                                  char *buf) {
    return assembler->assembleFrame(frame, recv_header, buf);
}

void Jungfrau::RawFrameAssembler::stop() { assembler->stop(); }

namespace FrameAssembler {
namespace Jungfrau {

/**
 * StdFrameAssembler
 */

class StdFrameAssembler::Helper {
  public:
    using BlockPtr = PacketBlockPtr<Packet>;
    using Stream = PacketStream<Packet>;

    Helper(GeneralDataPtr gd, bool f);
    virtual ~Helper() {}

    float getSrcPixelBytes() { return float(general_data->dynamicRange) / 8; }

    float getDstPixelBytes() {
        int factor = (general_data->dynamicRange == 4) ? 2 : 1;
        return getSrcPixelBytes() * factor;
    }

    virtual void assemblePackets(BlockPtr block, char *buf) = 0;

  protected:
    static const int chip_size = 256;
    static const int chip_gap = 2;
    static const int nb_horz_chips = 4;
    static const int nb_vert_chips = 2;

    struct Geometry {
        float pixel_size;
        int chip_size;
        int line_size;
        int offset;
    };

    GeneralDataPtr general_data;
    bool flipped;
    int nb_ifaces;
    int frame_packets;
    int packet_lines;
    Geometry src;
    Geometry dst;
    int first_idx;
    int idx_inc;
};

} // namespace Jungfrau
} // namespace FrameAssembler

Jungfrau::StdFrameAssembler::Helper::Helper(GeneralDataPtr gd, bool f)
    : general_data(gd), flipped(f) {
    int nb_ifaces = static_cast<JungfrauData *>(gd)->numUDPInterfaces;
    frame_packets = gd->packetsPerFrame;
    int port_vert_chips = nb_vert_chips / nb_ifaces;
    packet_lines = port_vert_chips * chip_size / frame_packets;
    src.pixel_size = getSrcPixelBytes();
    src.chip_size = chip_size * src.pixel_size;
    src.line_size = nb_horz_chips * src.chip_size;
    dst.pixel_size = getDstPixelBytes();
    dst.chip_size = (chip_size + chip_gap) * dst.pixel_size;
    dst.line_size = gd->nPixelsX * dst.pixel_size;
    if (flipped) {
        src.offset = (packet_lines - 1) * src.line_size;
        src.line_size *= -1;
        dst.offset = 0;
        first_idx = frame_packets - 1;
        idx_inc = -1;
    } else {
        src.offset = 0;
        dst.offset = (nb_ifaces == 2) ? dst.line_size : 0;
        first_idx = 0;
        idx_inc = 1;
    }
}

namespace FrameAssembler {
namespace Jungfrau {

class CopyHelper : public StdFrameAssembler::Helper {
  public:
    CopyHelper(GeneralDataPtr gd, bool flipped) : Helper(gd, flipped) {}

    virtual void assemblePackets(BlockPtr block, char *buf) override;
};

} // namespace Jungfrau
} // namespace FrameAssembler

void Jungfrau::CopyHelper::assemblePackets(BlockPtr block, char *buf) {
    int packet = first_idx;
    char *d = buf + dst.offset;
    int line = 0;
    for (int i = 0; i < frame_packets; ++i, packet += idx_inc) {
        if ((line > 0) && (line % chip_size) == 0)
            d += dst.line_size * chip_gap;
        Packet line_packet = (*block)[packet];
        char *s = line_packet.data() + src.offset;
        for (int l = 0; l < packet_lines; ++l, ++line) {
            char *ld = d;
            char *ls = s;
            for (int c = 0; c < nb_horz_chips; ++c) {
                if (line_packet.valid())
                    memcpy(ld, ls, src.chip_size);
                else
                    memset(ld, 0xff, src.chip_size);
                ls += src.chip_size;
                ld += dst.chip_size;
            }
            s += src.line_size;
            d += dst.line_size;
        }
    }
}

Jungfrau::StdFrameAssembler::StdFrameAssembler(DefaultFrameAssemblerBase::Ptr a)
    : assembler(a), helper(new CopyHelper(assembler->getGeneralData(), false)) {
}

Jungfrau::StdFrameAssembler::~StdFrameAssembler() { delete helper; }

Result Jungfrau::StdFrameAssembler::assembleFrame(uint64_t frame,
                                                  RecvHeader *recv_header,
                                                  char *buf) {

    DetHeader *det_header = &recv_header->detHeader;
    det_header->frameNumber = frame;
    det_header->packetNumber = 0;

    Assembler *a = static_cast<Assembler *>(assembler.get());
    Helper::Stream *ps = a->packet_stream.get();
    Helper::BlockPtr block = ps->getPacketBlock(frame);
    int packet_count = block ? block->getValidPackets() : 0;
    if (packet_count == 0)
        return Result{1, 0};

    det_header->packetNumber += packet_count;

    // write header
    Packet p = (*block)[0];
    p.fillDetHeader(det_header);

    if (buf)
        helper->assemblePackets(std::move(block), buf);

    return Result{1, 1};
}

void Jungfrau::StdFrameAssembler::stop() { assembler->stop(); }
