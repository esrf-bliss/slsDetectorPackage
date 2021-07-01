/************************************************
 * @file FrameAssembler.cpp
 * @short helper classes assembling frames
 * from udp packets
 ***********************************************/

#include "sls/detectors/eiger/FrameAssembler.h"
#include "sls/detectors/jungfrau/FrameAssembler.h"
#include "sls/logger.h"

#include <emmintrin.h>
#include <string.h>

using namespace sls::FrameAssembler;

/**
 * DefaultFrameAssembler
 */

template <class Packet, class DP>
FrameDims DefaultFrameAssembler<Packet, DP>::getAssembledFrameDims() {
    auto nb_pixels = iface_size.x * iface_size.y;
    return {iface_size, int(nb_pixels * DP::depth())};
}

template <class Packet, class DP>
void DefaultFrameAssembler<Packet, DP>::expand4Bits(char *dst, char *src,
                                                    int src_size) {
    unsigned long s = (unsigned long)src;
    unsigned long d = (unsigned long)dst;
    if ((s & 15) != 0) {
        LOG(logERROR) << "Missaligned source";
        return;
    } else if ((d & 15) != 0) {
        LOG(logERROR) << "Missaligned destination";
        return;
    }

    const int blk = sizeof(__m128i);
    if ((src_size % blk) != 0) {
        LOG(logWARNING) << "len misalignment: "
                        << "src_size=" << src_size << ", "
                        << "blk=" << blk;
    }
    int num_blocks = src_size / blk;
    const __m128i *src128 = (const __m128i *)src;
    __m128i *dst128 = (__m128i *)dst;
    const __m128i mask = _mm_set1_epi8(0xf);
    for (int i = 0; i < num_blocks; ++i) {
        __m128i pack4_raw = _mm_load_si128(src128++);
        __m128i pack4_shr = _mm_srli_epi16(pack4_raw, 4);
        __m128i ilace8_0 = _mm_and_si128(pack4_raw, mask);
        __m128i ilace8_1 = _mm_and_si128(pack4_shr, mask);
        __m128i pack8_0 = _mm_unpacklo_epi8(ilace8_0, ilace8_1);
        __m128i pack8_1 = _mm_unpackhi_epi8(ilace8_0, ilace8_1);
        _mm_store_si128(dst128++, pack8_0);
        _mm_store_si128(dst128++, pack8_1);
    }
}

template <class Packet, class DP>
bool DefaultFrameAssembler<Packet, DP>::assembleFrame(
    const AnyPacketBlockPtr &block, char *buf) {
    if (!std::holds_alternative<BlockPtr>(block))
        throw std::runtime_error("Invalid packet block");

    auto &b = std::get<BlockPtr>(block);
    if (!b || (b->getValidPackets() == 0))
        return false;
    else if (!buf)
        return true;

    constexpr int packets_per_frame = Packet::Data::PacketsPerFrame;
    constexpr uint32_t src_dsize = PacketData::PacketDataLen;
    constexpr uint32_t frame_size = PacketData::FrameLen;
#define check_last(i, p) (((i) % (p)) ? ((i) % (p)) : (p))
    constexpr uint32_t last_dsize = check_last(frame_size, src_dsize);
#undef check_last
    constexpr uint32_t dst_dsize =
        PacketData::PacketDataLen / SP::depth() * DP::depth();

    uint32_t prev_adjust = 0;
    auto valid_packet_mask = b->getValidPacketMask();
    for (int i = 0; i < packets_per_frame; ++i) {
        // copy packet
        auto packet = (*b)[i];
        char *dst = buf + i * dst_dsize;
        bool last_packet = (i == (packets_per_frame - 1));
        uint32_t copy_dsize = last_packet ? last_dsize : src_dsize;
        uint32_t size_adjust = packet.sizeAdjust();
        copy_dsize += size_adjust;
        dst += prev_adjust;
        prev_adjust = size_adjust;
        if (!valid_packet_mask[i])
            memset(dst, 0xff, copy_dsize);
        else if (Expand4Bits)
            expand4Bits(dst, packet.data(), copy_dsize);
        else
            memcpy(dst, packet.data(), copy_dsize);
    }

    return true;
}

template <class DetGeom, class Packet, class DP = typename Packet::Data::Pixel>
DefaultFrameAssemblerPtr DefaultFrameAssemblerFactory() {
    using Assembler = DefaultFrameAssembler<Packet, DP>;
    constexpr auto mod_geom = DetGeom::raw_geom.getModGeom(XY0);
    constexpr auto recv_geom = mod_geom.getRecvGeom(XY0);
    constexpr auto iface_size = recv_geom.getIfaceGeom(XY0).size;
    slsDetectorDefs::xy iface_dims{int(iface_size.x), int(iface_size.y)};
    return std::make_shared<Assembler>(iface_dims);
}

template <class NbUDPIfaces> auto JungfrauFrameAssemblerFactory() {
    using Packet = sls::Jungfrau::Packet<NbUDPIfaces>;
    using DG = sls::Jungfrau::Geom::Jungfrau500kGeom<NbUDPIfaces>;
    return DefaultFrameAssemblerFactory<DG, Packet>();
}

DefaultFrameAssemblerPtr sls::FrameAssembler::CreateDefaultFrameAssembler(
    slsDetectorDefs::detectorType det_type, bool tg_enable, int num_udp_ifaces,
    uint32_t src_dr, uint32_t dst_dr) {
    if (dst_dr == 0)
        dst_dr = src_dr;

    auto any_src_pixel = AnyPixelFromBpp(src_dr);
    auto any_dst_pixel = AnyPixelFromBpp(dst_dr);

    return std::visit(
        [&](auto src_pixel, auto dst_pixel) {
            using SP = decltype(src_pixel);
            using DP = decltype(dst_pixel);

            if (det_type == slsDetectorDefs::EIGER) {
                auto any_tg = Eiger::AnyTenGigaFromTgEnable(tg_enable);
                return std::visit(
                    [&](auto tg) {
                        using TG = decltype(tg);
                        using Packet = Eiger::Packet<SP, TG>;
                        using DG = Eiger::Geom::Eiger500kGeom;
                        return DefaultFrameAssemblerFactory<DG, Packet, DP>();
                    },
                    any_tg);
            } else if (det_type == slsDetectorDefs::JUNGFRAU) {
                if (num_udp_ifaces == 1)
                    return JungfrauFrameAssemblerFactory<
                        Jungfrau::Geom::OneIface>();
                else
                    return JungfrauFrameAssemblerFactory<
                        Jungfrau::Geom::TwoIface>();
            } else
                throw sls::RuntimeError("Detector not supported: " +
                                        std::to_string(det_type));
        },
        any_src_pixel, any_dst_pixel);
}

/**
 * RawFrameAssembler
 */

Result RawFrameAssembler::assembleFrame(const AnyPacketBlockList &blocks,
                                        char *buf) {
    const int NbIfaces = assembler.size();
    if (blocks.size() != std::size_t(NbIfaces))
        throw std::runtime_error("Invalid packet block list");

    if (buf)
        buf += data_offset;
    Result res{NbIfaces, 0};
    for (int i = 0; i < NbIfaces; ++i) {
        res.valid_data[i] = assembler[i]->assembleFrame(blocks[i], buf);
        if (buf)
            buf += assembler[i]->getAssembledFrameDims().size;
    }
    return res;
}

FrameDims RawFrameAssembler::getAssembledFrameDims() {
    const int NbIfaces = assembler.size();
    auto dims = assembler[0]->getAssembledFrameDims();
    auto det_ifaces = NbIfaces * nb_recvs;
    dims.dim.y *= det_ifaces;
    dims.size *= det_ifaces;
    return dims;
}
