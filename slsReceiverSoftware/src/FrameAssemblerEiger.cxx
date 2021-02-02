/************************************************
 * @file FrameAssemblerEiger.cxx
 * @short helper classes assembling Eiger frames
 * from udp packets
 * This file is include in FrameAssembler.cpp
 ***********************************************/

namespace FAEiger = FrameAssembler::Eiger;
namespace GeomEiger = sls::Geom::Eiger;

namespace FrameAssembler {
namespace Eiger {

constexpr int IfaceHorzChips = GeomEiger::IfaceChips.x;

/**
 * GeomHelper
 */

// P: Pixel Type, RG: RecvGeom
template <class P, class RG> struct GeomHelper {

#define SCA static constexpr auto
#define SCI static constexpr int
#define SCF static constexpr float

    using SrcPixel = P;
    using DstPixel = std::conditional_t<std::is_same_v<P, Pixel4>, Pixel8, P>;

    SCA RecvGeom = RG::geom;
    using BlockPtr = PacketBlockPtr<Packet<SrcPixel>>;

    SCI PacketDataLen = Packet<SrcPixel>::Data::PacketDataLen;
    SCI PacketPixels = PacketDataLen / SrcPixel::depth();

    // raw (packet) geometry
    SCA RawIfaceSize = RawIfaceGeom.size;
    // std (image) geometry
    SCA RecvView = RecvGeom.view;
    SCA IfaceGeom1 = RecvGeom.getIfaceGeom(XY{0, 0});
    SCA IfaceView1 = IfaceGeom1.view;

    SCA getPacketView(int PacketIdx) {
        return IfaceGeom1.getPacketView(PacketPixels, PacketIdx);
    }

    SCI frame_packets = FramePackets<SrcPixel>;
    SCI packet_lines = RawIfaceSize.y / frame_packets;
    SCI flipped = (RecvView.pixelDir().y < 0);
    SCF src_pixel_size = SrcPixel::depth();
    SCI src_chip_size = GeomEiger::ChipPixels.x * src_pixel_size;
    SCI src_line_size = RawIfaceSize.x * src_pixel_size;
    SCI src_dir = flipped ? -1 : 1;
    SCI src_line_step = src_line_size * src_dir;
    SCI dst_pixel_size = DstPixel::depth();
    SCI dst_chip_pixels = IfaceGeom1.chip_step.x;
    SCI dst_chip_size = dst_chip_pixels * dst_pixel_size;
    SCI dst_line_size = RecvView.pixelStep().y * dst_pixel_size * src_dir;
    SCI chip_gap_cols = GeomEiger::ChipGap.x;
    SCI chip_gap_lines = GeomEiger::ChipGap.y;
    SCI gap_cols_size = chip_gap_cols * dst_pixel_size;
    SCI gap_lines_size = chip_gap_lines * dst_line_size;
    SCI src_first_line = IfaceView1.calcViewOrigin().y;
    SCI src_first_packet = src_first_line / packet_lines;
    SCA first_packet_view = getPacketView(src_first_packet);
    SCA first_packet_offset = first_packet_view.calcViewOrigin();
    SCI src_offset = first_packet_offset.y * src_line_size;
    SCI dst_iface_step = RecvGeom.iface_step.y * dst_line_size;

#undef SCF
#undef SCI
#undef SCA
};

/**
 * Expand4BitsHelper
 */

template <class P, class RG> struct Expand4BitsHelper : GeomHelper<P, RG> {

    using H = GeomHelper<P, RG>;
    using BlockPtr = typename H::BlockPtr;

#define SCI static constexpr int

    SCI half_module_chips = NbIfaces * IfaceHorzChips;
    SCI block_len = sizeof(__m128i);
    SCI block_bits = block_len * 8;
    SCI gap_bits = GeomEiger::ChipGap.x * 8;

    SCI chip_blocks = H::src_chip_size / block_len;
    SCI iface_blocks = H::src_line_step / block_len;

#undef SCI

    const __m128i m{_mm_set1_epi8(0xf)};
    const __m128i m64_0{_mm_set_epi64x(0, -1)};
    const __m128i m64_1{_mm_set_epi64x(-1, 0)};
    const __m128i gap_bits128{_mm_set_epi64x(0, gap_bits)};
    const __m128i block64_bits128{_mm_set_epi64x(0, 64)};

    struct Worker {
        Expand4BitsHelper &h;

        bool v[NbIfaces], valid_data;
        const __m128i *s[NbIfaces], *src128;
        __m128i *dst128;
        int dest_misalign;
        int shift_l;
        __m128i shift_l128, shift_r128;
        __m128i prev;

        Worker(Expand4BitsHelper &helper) : h(helper){};

        int load_packet(BlockPtr block[NbIfaces], int packet);
        void load_dst128(char *buf);
        void load_shift_store128();
        void pad_dst128();
        void sync_dst128();

        void assemblePackets(BlockPtr block[NbIfaces], char *buf);
    };

    void assemblePackets(BlockPtr block[NbIfaces], char *buf) {
        Worker w(*this);
        w.assemblePackets(block, buf);
    }
};

template <class P, class RG>
int Expand4BitsHelper<P, RG>::Worker::load_packet(BlockPtr block[NbIfaces],
                                                  int packet) {
    Packet<P> p0 = (*block[0])[packet];
    Packet<P> p1 = (*block[1])[packet];
    s[0] = (const __m128i *)(p0.data() + h.src_offset);
    s[1] = (const __m128i *)(p1.data() + h.src_offset);
    if ((((unsigned long)s[0] | (unsigned long)s[1]) & 15) != 0) {
        LOG(logERROR) << "Missaligned src";
        return -1;
    }
    v[0] = p0.valid();
    v[1] = p1.valid();
    return 0;
}

template <class P, class RG>
void Expand4BitsHelper<P, RG>::Worker::load_dst128(char *buf) {
    char *d = buf;
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

template <class P, class RG>
void Expand4BitsHelper<P, RG>::Worker::load_shift_store128() {
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

template <class P, class RG>
void Expand4BitsHelper<P, RG>::Worker::pad_dst128() {
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

template <class P, class RG>
void Expand4BitsHelper<P, RG>::Worker::sync_dst128() {
    if (shift_l != 0) {
        __m128i m0;
        m0 = _mm_sll_epi64(_mm_set1_epi8(0xff), shift_l128);
        if (shift_l >= 64)
            m0 = _mm_and_si128(m0, h.m64_1);
        __m128i a = _mm_and_si128(_mm_load_si128(dst128), m0);
        _mm_store_si128(dst128, _mm_or_si128(prev, a));
    }
}

template <class P, class RG>
void Expand4BitsHelper<P, RG>::Worker::assemblePackets(BlockPtr block[NbIfaces],
                                                       char *buf) {
    const int &hm_chips = h.half_module_chips;
    int packet = h.src_first_packet;
    load_dst128(buf);
    for (int p = 0; p < h.frame_packets; ++p, packet += h.src_dir) {
        if (load_packet(block, packet) < 0)
            return;
        for (int l = 0; l < h.packet_lines; ++l) {
            int chip_count = 0;
            for (int i = 0; i < NbIfaces; ++i) {
                valid_data = v[i];
                src128 = s[i];
                for (int c = 0; c < IfaceHorzChips; ++c) {
                    for (int b = 0; b < h.chip_blocks; ++b)
                        load_shift_store128();
                    if (++chip_count % hm_chips > 0)
                        pad_dst128();
                }
                s[i] += h.iface_blocks;
            }
        }
    }
    sync_dst128();

    constexpr bool top_recv = h.flipped;
    bool fill_chip_gap_lines = top_recv;
    if (fill_chip_gap_lines)
        memset(buf + h.dst_iface_step - h.gap_lines_size, 0, h.gap_lines_size);
}

/**
 * CopyHelper
 */

template <class P, class RG> struct CopyHelper : GeomHelper<P, RG> {

    using H = GeomHelper<P, RG>;
    using BlockPtr = typename H::BlockPtr;

    void assemblePackets(BlockPtr block[NbIfaces], char *buf);
};

template <class P, class RG>
void CopyHelper<P, RG>::assemblePackets(BlockPtr block[NbIfaces], char *buf) {
    H h;
    int packet = h.src_first_packet;
    char *d = buf;
    for (int p = 0; p < h.frame_packets; ++p, packet += h.src_dir) {
        Packet<P> line_packet[NbIfaces] = {(*block[0])[packet],
                                           (*block[1])[packet]};
        char *s[NbIfaces] = {line_packet[0].data() + h.src_offset,
                             line_packet[1].data() + h.src_offset};
        for (int l = 0; l < h.packet_lines; ++l) {
            char *ld = d;
            for (int i = 0; i < NbIfaces; ++i) {
                char *ls = s[i];
                for (int c = 0; c < IfaceHorzChips; ++c) {
                    if (line_packet[i].valid())
                        memcpy(ld, ls, h.src_chip_size);
                    else
                        memset(ld, 0xff, h.src_chip_size);
                    bool fill_chip_gap_cols =
                        ((i == 0) || (c < IfaceHorzChips - 1));
                    if (fill_chip_gap_cols)
                        memset(ld + h.src_chip_size, 0, h.gap_cols_size);
                    ls += h.src_chip_size;
                    ld += h.dst_chip_size;
                }
                s[i] += h.src_line_step;
            }
            d += h.dst_line_size;
        }
    }
    constexpr bool top_recv = h.flipped;
    bool fill_chip_gap_lines = top_recv;
    if (fill_chip_gap_lines)
        memset(d, 0, h.gap_lines_size);
}

/**
 * FrameAssembler
 */

template <class P, class FP, class RG>
Result FrameAssembler<P, FP, RG>::assembleFrame(uint64_t frame,
                                                RecvHeader *recv_header,
                                                char *buf) {
    PortsMask mask;
    bool header_empty = true;

    DetHeader *det_header = &recv_header->detHeader;
    det_header->frameNumber = frame;
    det_header->packetNumber = 0;

    BlockPtr block[NbIfaces] = {stream[0]->getPacketBlock(frame),
                                stream[1]->getPacketBlock(frame)};
    for (int i = 0; i < NbIfaces; ++i) {
        int packet_count = block[i] ? block[i]->getValidPackets() : 0;
        if (packet_count == 0)
            continue;
        mask.set(i, true);
        det_header->packetNumber += packet_count;

        // write header
        if (header_empty) {
            Packet<P> p = (*block[i])[0];
            p.fillDetHeader(det_header);
            header_empty = false;
        }
    }

    constexpr bool fp_partial = std::is_same_v<FP, PartialFrameDiscard>;
    if (fp_partial && (mask.count() != NbIfaces))
        return Result{NbIfaces, 0};

    if (mask.any() && buf)
        helper.assemblePackets(block, buf);

    return Result{NbIfaces, mask};
}

} // namespace Eiger
} // namespace FrameAssembler

FrameAssemblerPtr FAEiger::CreateFrameAssembler(int pixel_bpp, FramePolicy fp,
                                                bool enable_tg, int recv_idx,
                                                DefaultFrameAssemblerList a) {

    if (!enable_tg) {
        const char *error = "10 Giga not enabled!";
        std::cerr << error << std::endl;
        throw std::runtime_error(error);
    }

    auto any_pixel = AnyPixelFromBpp(pixel_bpp);
    auto any_fp = AnyFramePolicyFromFP(fp);
    auto any_geom = GeomEiger::AnyRecvGeomFromIndex<AsmWithGapFmt>(recv_idx);

    return std::visit(
        [&](auto pixel, auto fp, auto geom) -> FrameAssemblerPtr {
            using P = decltype(pixel);
            using FP = decltype(fp);
            using RG = decltype(geom);
            using Assembler = FrameAssembler<P, FP, RG>;
            typename Assembler::StreamList s;
            auto f = [](auto a) { return Assembler::rawAssemblerStream(a); };
            std::transform(std::begin(a), std::end(a), std::begin(s), f);
            return std::make_shared<Assembler>(s);
        },
        any_pixel, any_fp, any_geom);
}
