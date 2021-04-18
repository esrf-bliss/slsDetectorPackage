/************************************************
 * @file FrameAssembler.cpp
 * @short helper classes assembling frames
 * from udp packets
 ***********************************************/

#include "sls/FrameAssembler.h"
#include "sls/logger.h"

#include <emmintrin.h>
#include <string.h>

using namespace FrameAssembler;

/**
 * DefaultFrameAssembler
 */

template <class Packet, class DP>
int DefaultFrameAssembler<Packet, DP>::getImageSize() {
    return PacketData::FrameLen / SP::depth() * DP::depth();
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
bool DefaultFrameAssembler<Packet, DP>::assembleFrame(AnyPacketBlockPtr &&block,
                                                      RecvHeader *recv_header,
                                                      char *buf) {
    if (!std::holds_alternative<BlockPtr>(block))
        throw std::runtime_error("Invalid packet block");

    BlockPtr b = std::get<BlockPtr>(std::move(block));
    if (!b || (b->getValidPackets() == 0))
        return false;

    constexpr int packets_per_frame = Packet::Data::PacketsPerFrame;
    DetHeader *det_header = &recv_header->detHeader;
    constexpr uint32_t src_dsize = PacketData::PacketDataLen;
    constexpr uint32_t frame_size = PacketData::FrameLen;
#define check_last(i, p) (((i) % (p)) ? ((i) % (p)) : (p))
    constexpr uint32_t last_dsize = check_last(frame_size, src_dsize);
#undef check_last
    constexpr uint32_t dst_dsize =
        PacketData::PacketDataLen / SP::depth() * DP::depth();

    recv_header->packetsMask.reset();
    bool header_empty = true;

    uint32_t prev_adjust = 0;
    for (int i = 0; i < packets_per_frame; ++i) {
        Packet packet = (*b)[i];
        if (!packet.valid())
            continue;

        int pnum = packet.number();
        recv_header->packetsMask[pnum] = 1;

        // write header
        if (header_empty) {
            packet.fillDetHeader(det_header);
            header_empty = false;
        }

        if (!buf)
            continue;

        // copy packet
        char *dst = buf + pnum * dst_dsize;
        bool last_packet = (pnum == (packets_per_frame - 1));
        uint32_t copy_dsize = last_packet ? last_dsize : src_dsize;
        uint32_t size_adjust = packet.sizeAdjust();
        copy_dsize += size_adjust;
        dst += prev_adjust;
        prev_adjust = size_adjust;
        if (Expand4Bits)
            expand4Bits(dst, packet.data(), copy_dsize);
        else
            memcpy(dst, packet.data(), copy_dsize);
    }

    det_header->packetNumber = b->getValidPackets();

    return true;
}

DefaultFrameAssemblerPtr FrameAssembler::CreateDefaultFrameAssembler(
    slsDetectorDefs::detectorType det_type, int num_udp_ifaces, uint32_t src_dr,
    uint32_t dst_dr) {
    if (dst_dr == 0)
        dst_dr = src_dr;

    auto any_src_pixel = AnyPixelFromBpp(src_dr);
    auto any_dst_pixel = AnyPixelFromBpp(dst_dr);

    return std::visit(
        [&](auto src_pixel, auto dst_pixel) -> DefaultFrameAssemblerPtr {
            using SP = decltype(src_pixel);
            using DP = decltype(dst_pixel);

            if (det_type == slsDetectorDefs::EIGER) {
                using Packet = ::Eiger::Packet<SP>;
                using Assembler = DefaultFrameAssembler<Packet, DP>;
                return std::make_shared<Assembler>();
            } else if (det_type == slsDetectorDefs::JUNGFRAU) {
                if (num_udp_ifaces == 1) {
                    using Packet = ::Jungfrau::Packet<1>;
                    using Assembler = DefaultFrameAssembler<Packet>;
                    return std::make_shared<Assembler>();
                } else {
                    using Packet = ::Jungfrau::Packet<2>;
                    using Assembler = DefaultFrameAssembler<Packet>;
                    return std::make_shared<Assembler>();
                }
            } else
                throw sls::RuntimeError("Detector not supported: " +
                                        std::to_string(det_type));
        },
        any_src_pixel, any_dst_pixel);
}

/**
 * RawFrameAssembler
 */

Result RawFrameAssembler::assembleFrame(AnyPacketBlockList &&blocks,
                                        RecvHeader *recv_header, char *buf) {
    const int NbIfaces = assembler.size();
    if (blocks.size() != std::size_t(NbIfaces))
        throw std::runtime_error("Invalid packet block list");

    if (buf)
        buf += data_offset;
    Result res{NbIfaces, 0};
    for (int i = 0; i < NbIfaces; ++i) {
        res.valid_data[i] =
            assembler[i]->assembleFrame(std::move(blocks[i]), recv_header, buf);
        if (buf)
            buf += assembler[i]->getImageSize();
    }
    return res;
}

#include "FrameAssemblerEiger.cxx"
#include "FrameAssemblerJungfrau.cxx"
