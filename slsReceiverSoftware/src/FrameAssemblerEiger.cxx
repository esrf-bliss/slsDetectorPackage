/************************************************
 * @file FrameAssemblerEiger.cxx
 * @short helper classes assembling Eiger frames
 * from udp packets
 * This file is include in FrameAssembler.cpp
 ***********************************************/

/**
 * Eiger::RawFrameAssembler
 */

Eiger::RawFrameAssembler::RawFrameAssembler(DefaultFrameAssemblerBase::Ptr a[2])
    : DualPortFrameAssembler(a) {}

Result Eiger::RawFrameAssembler::assembleFrame(uint64_t frame,
                                               RecvHeader *recv_header,
                                               char *buf) {
    const int num_ports = 2;
    const int image_size = assembler[0]->getImageSize();
    Result dual{num_ports, 0};
    for (int i = 0; i < num_ports; ++i) {
        Result single = assembler[i]->assembleFrame(frame, recv_header, buf);
        dual.valid_data.set(i, single.valid_data[0]);
        if (buf)
            buf += image_size;
    }
    return dual;
}

/**
 * Eiger::StdFrameAssembler::Helper
 */

class Eiger::StdFrameAssembler::Helper {
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

    virtual void assemblePackets(BlockPtr block[2], char *buf) = 0;

  protected:
    static const int chip_size = 256;
    static const int chip_gap = 2;
    static const int num_ports = 2;
    static const int port_horz_chips = 2;
    static const int port_vert_chips = 1;

    struct Geometry {
        float pixel_size;
        int chip_size;
        int line_size;
        int offset;
    };

    GeneralDataPtr general_data;
    bool flipped;
    int frame_packets;
    int packet_lines;
    Geometry src;
    Geometry dst;
    int first_idx;
    int idx_inc;
};

Eiger::StdFrameAssembler::Helper::Helper(GeneralDataPtr gd, bool f)
    : general_data(gd), flipped(f) {
    frame_packets = gd->packetsPerFrame;
    packet_lines = port_vert_chips * chip_size / frame_packets;
    src.pixel_size = getSrcPixelBytes();
    src.chip_size = chip_size * src.pixel_size;
    src.line_size = port_horz_chips * src.chip_size;
    dst.pixel_size = getDstPixelBytes();
    dst.chip_size = (chip_size + chip_gap) * dst.pixel_size;
    dst.line_size = num_ports * gd->nPixelsX * dst.pixel_size;
    if (flipped) {
        src.offset = (packet_lines - 1) * src.line_size;
        src.line_size *= -1;
        dst.offset = 0;
        first_idx = frame_packets - 1;
        idx_inc = -1;
    } else {
        src.offset = 0;
        dst.offset = dst.line_size;
        first_idx = 0;
        idx_inc = 1;
    }
}

/**
 * Expand4BitsHelper
 */
namespace FrameAssembler {
namespace Eiger {

class Expand4BitsHelper : public Eiger::StdFrameAssembler::Helper {
  public:
    Expand4BitsHelper(GeneralDataPtr gd, bool flipped);

    virtual void assemblePackets(BlockPtr block[2], char *buf) override;

  private:
    static const int half_module_chips = num_ports * port_horz_chips;
    static const int block_len = sizeof(__m128i);
    static const int block_bits = block_len * 8;
    static const int gap_bits = chip_gap * 8;

    const int chip_blocks;
    const int port_blocks;

    const __m128i m;
    const __m128i m64_0, m64_1;
    const __m128i gap_bits128;
    const __m128i block64_bits128;

    struct Worker {
        Expand4BitsHelper &h;

        bool v[num_ports], valid_data;
        const __m128i *s[num_ports], *src128;
        __m128i *dst128;
        int dest_misalign;
        int shift_l;
        __m128i shift_l128, shift_r128;
        __m128i prev;

        Worker(Expand4BitsHelper &helper);

        int load_packet(BlockPtr block[2], int packet);
        void load_dst128(char *buf);
        void load_shift_store128();
        void pad_dst128();
        void sync_dst128();

        void assemblePackets(BlockPtr block[2], char *buf);
    };
};

} // namespace Eiger
} // namespace FrameAssembler

Eiger::Expand4BitsHelper::Expand4BitsHelper(GeneralDataPtr gd, bool flipped)
    : Helper(gd, flipped), chip_blocks(src.chip_size / block_len),
      port_blocks(src.line_size / block_len), m(_mm_set1_epi8(0xf)),
      m64_0(_mm_set_epi64x(0, -1)), m64_1(_mm_set_epi64x(-1, 0)),
      gap_bits128(_mm_set_epi64x(0, gap_bits)),
      block64_bits128(_mm_set_epi64x(0, 64)) {}

inline Eiger::Expand4BitsHelper::Worker::Worker(Expand4BitsHelper &helper)
    : h(helper) {}

inline int Eiger::Expand4BitsHelper::Worker::load_packet(BlockPtr block[2],
                                                         int packet) {
    Packet p0 = (*block[0])[packet];
    Packet p1 = (*block[1])[packet];
    s[0] = (const __m128i *)(p0.data() + h.src.offset);
    s[1] = (const __m128i *)(p1.data() + h.src.offset);
    if ((((unsigned long)s[0] | (unsigned long)s[1]) & 15) != 0) {
        LOG(logERROR) << "Missaligned src";
        return -1;
    }
    v[0] = p0.valid();
    v[1] = p1.valid();
    return 0;
}

inline void Eiger::Expand4BitsHelper::Worker::load_dst128(char *buf) {
    char *d = buf + h.dst.offset;
    dest_misalign = ((unsigned long)d & 15);
    dst128 = (__m128i *)(d - dest_misalign);
    shift_l = dest_misalign * 8;
    shift_l128 = _mm_set_epi64x(0, shift_l % 64);
    shift_r128 = _mm_sub_epi64(h.block64_bits128, shift_l128);
    if (shift_l != 0) {
        __m128i m0;
        m0 = _mm_srl_epi64(_mm_set1_epi8(0xff), shift_r128);
        if (shift_l < 64)
            m0 = _mm_and_si128(m0, h.m64_0);
        prev = _mm_and_si128(_mm_load_si128(dst128), m0);
    } else {
        prev = _mm_setzero_si128();
    }
}

inline void Eiger::Expand4BitsHelper::Worker::load_shift_store128() {
    __m128i p4_raw;
    if (valid_data)
        p4_raw = _mm_load_si128(src128);
    else
        p4_raw = _mm_set1_epi8(0xff);
    ++src128;
    __m128i p4_shr = _mm_srli_epi16(p4_raw, 4);
    __m128i i8_0 = _mm_and_si128(p4_raw, h.m);
    __m128i i8_1 = _mm_and_si128(p4_shr, h.m);
    __m128i p8_0 = _mm_unpacklo_epi8(i8_0, i8_1);
    __m128i p8_1 = _mm_unpackhi_epi8(i8_0, i8_1);
    __m128i p8_0l = _mm_sll_epi64(p8_0, shift_l128);
    __m128i p8_0r = _mm_srl_epi64(p8_0, shift_r128);
    __m128i p8_1l = _mm_sll_epi64(p8_1, shift_l128);
    __m128i p8_1r = _mm_srl_epi64(p8_1, shift_r128);
    __m128i d0, d1, d2, d3, d4;
    if (shift_l < 64) {
        d0 = p8_0l;
        d1 = p8_0r;
        d2 = p8_1l;
        d3 = p8_1r;
        d4 = _mm_setzero_si128();
    } else {
        d0 = _mm_setzero_si128();
        d1 = p8_0l;
        d2 = p8_0r;
        d3 = p8_1l;
        d4 = p8_1r;
    }
    __m128i d10 = _mm_slli_si128(d1, 8);
    __m128i d11 = _mm_srli_si128(d1, 8);
    __m128i d30 = _mm_slli_si128(d3, 8);
    __m128i d31 = _mm_srli_si128(d3, 8);
    prev = _mm_or_si128(prev, d0);
    _mm_store_si128(dst128++, _mm_or_si128(prev, d10));
    prev = _mm_or_si128(d11, d2);
    _mm_store_si128(dst128++, _mm_or_si128(prev, d30));
    prev = _mm_or_si128(d31, d4);
}

inline void Eiger::Expand4BitsHelper::Worker::pad_dst128() {
    shift_l += h.gap_bits;
    if (shift_l % 64 == 0)
        shift_l128 = _mm_setzero_si128();
    else
        shift_l128 = _mm_add_epi64(shift_l128, h.gap_bits128);
    shift_r128 = _mm_sub_epi64(h.block64_bits128, shift_l128);
    if (shift_l == h.block_bits) {
        _mm_store_si128(dst128++, prev);
        prev = _mm_setzero_si128();
        shift_l = 0;
    }
}

inline void Eiger::Expand4BitsHelper::Worker::sync_dst128() {
    if (shift_l != 0) {
        __m128i m0;
        m0 = _mm_sll_epi64(_mm_set1_epi8(0xff), shift_l128);
        if (shift_l >= 64)
            m0 = _mm_and_si128(m0, h.m64_1);
        __m128i a = _mm_and_si128(_mm_load_si128(dst128), m0);
        _mm_store_si128(dst128, _mm_or_si128(prev, a));
    }
}

inline void Eiger::Expand4BitsHelper::Worker::assemblePackets(BlockPtr block[2],
                                                              char *buf) {
    const int &hm_chips = h.half_module_chips;
    int packet = h.first_idx;
    load_dst128(buf);
    for (int i = 0; i < h.frame_packets; ++i, packet += h.idx_inc) {
        if (load_packet(block, packet) < 0)
            return;
        for (int l = 0; l < h.packet_lines; ++l) {
            int chip_count = 0;
            for (int p = 0; p < h.num_ports; ++p) {
                valid_data = v[p];
                src128 = s[p];
                for (int c = 0; c < h.port_horz_chips; ++c) {
                    for (int b = 0; b < h.chip_blocks; ++b)
                        load_shift_store128();
                    if (++chip_count % hm_chips > 0)
                        pad_dst128();
                }
                s[p] += h.port_blocks;
            }
        }
    }
    sync_dst128();
}

void Eiger::Expand4BitsHelper::assemblePackets(BlockPtr block[2], char *buf) {
    Worker w(*this);
    w.assemblePackets(block, buf);
}

namespace FrameAssembler {
namespace Eiger {

/**
 * CopyHelper
 */

class CopyHelper : public StdFrameAssembler::Helper {
  public:
    CopyHelper(GeneralDataPtr gd, bool flipped) : Helper(gd, flipped) {}

    virtual void assemblePackets(BlockPtr block[2], char *buf) override;
};

} // namespace Eiger
} // namespace FrameAssembler

void Eiger::CopyHelper::assemblePackets(BlockPtr block[2], char *buf) {
    int packet = first_idx;
    char *d = buf + dst.offset;
    for (int i = 0; i < frame_packets; ++i, packet += idx_inc) {
        Packet line_packet[2] = {(*block[0])[packet], (*block[1])[packet]};
        char *s[num_ports] = {line_packet[0].data() + src.offset,
                              line_packet[1].data() + src.offset};
        for (int l = 0; l < packet_lines; ++l) {
            char *ld = d;
            for (int p = 0; p < num_ports; ++p) {
                char *ls = s[p];
                for (int c = 0; c < port_horz_chips; ++c) {
                    if (line_packet[p].valid())
                        memcpy(ld, ls, src.chip_size);
                    else
                        memset(ld, 0xff, src.chip_size);
                    ls += src.chip_size;
                    ld += dst.chip_size;
                }
                s[p] += src.line_size;
            }
            d += dst.line_size;
        }
    }
}

/**
 * Eiger::StdFrameAssembler
 */

Eiger::StdFrameAssembler::StdFrameAssembler(DefaultFrameAssemblerBase::Ptr a[2],
                                            bool flipped)
    : DualPortFrameAssembler(a) {
    GeneralDataPtr gd = a[0]->getGeneralData();
    if (!gd->tgEnable) {
        const char *error = "10 Giga not enabled!";
        std::cerr << error << std::endl;
        throw std::runtime_error(error);
    }

    Helper *h;
    if (a[0]->doExpand4Bits())
        h = new Expand4BitsHelper(gd, flipped);
    else
        h = new CopyHelper(gd, flipped);
    helper = h;
}

Eiger::StdFrameAssembler::~StdFrameAssembler() { delete helper; }

Result Eiger::StdFrameAssembler::assembleFrame(uint64_t frame,
                                               RecvHeader *recv_header,
                                               char *buf) {
    PortsMask mask;

    const int num_ports = 2;

    Assembler *a[num_ports] = {
        reinterpret_cast<Assembler *>(assembler[0].get()),
        reinterpret_cast<Assembler *>(assembler[1].get())};
    bool header_empty = true;

    DetHeader *det_header = &recv_header->detHeader;
    det_header->frameNumber = frame;
    det_header->packetNumber = 0;

    Helper::BlockPtr block[num_ports];
    for (int i = 0; i < num_ports; ++i) {
        Helper::Stream *ps = a[i]->packet_stream.get();
        block[i] = std::move(ps->getPacketBlock(frame));
        int packet_count = block[i] ? block[i]->getValidPackets() : 0;
        if (packet_count == 0)
            continue;
        mask.set(i, true);
        det_header->packetNumber += packet_count;

        // write header
        if (header_empty) {
            Packet p = (*block[i])[0];
            p.fillDetHeader(det_header);
            header_empty = false;
        }
    }

    FramePolicy policy = a[0]->frame_policy;
    bool fp_partial = (policy == slsDetectorDefs::DISCARD_PARTIAL_FRAMES);
    if (fp_partial && (mask.count() != num_ports))
        return Result{num_ports, 0};

    if (mask.any() && buf)
        helper->assemblePackets(block, buf);

    return Result{num_ports, mask};
}
