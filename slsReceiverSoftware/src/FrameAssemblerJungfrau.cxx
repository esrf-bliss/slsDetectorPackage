/************************************************
 * @file FrameAssemblerJungfrau.cxx
 * @short helper classes assembling Jungfrau frames
 * from udp packets
 * This file is include in FrameAssembler.cpp
 ***********************************************/

namespace GeomJungfrau = sls::Geom::Jungfrau;

namespace FrameAssembler {
namespace Jungfrau {

constexpr int IfaceHorzChips = GeomJungfrau::IfaceChips<1>.x;

template <int NbUDPIfaces, int Idx>
constexpr auto RawIfaceGeom = ::Jungfrau::RawIfaceGeom<NbUDPIfaces, Idx>;

/**
 * GeomHelper
 */

//  GD: Geom data, MGX/Y: Module gap X/Y
template <class GD, bool MGX, bool MGY, int Idx> struct GeomHelper {

    using SrcPixel = Pixel;
    using DstPixel = Pixel;

#define SCA static constexpr auto
#define SCI static constexpr int

    SCI NbUDPIfaces = GD::num_udp_ifaces;

    using BlockPtr = PacketBlockPtr<Packet<NbUDPIfaces>>;

    using PacketData = typename Packet<NbUDPIfaces>::Data;

    // raw (packet) geometry
    SCA RawIfaceSize = RawIfaceGeom<NbUDPIfaces, Idx>.size;
    // std (image) geometry
    SCA RecvGeom = GD::RecvGeom::geom;
    SCA RecvView = RecvGeom.view;
    SCA IfaceGeom1 = RecvGeom.getIfaceGeom(XY0);
    SCA IfaceView1 = IfaceGeom1.view;

    SCA getPacketView(int PacketIdx) {
        return IfaceGeom1.getPacketView(PacketData::PacketPixels, PacketIdx);
    }

    SCI chip_cols = GeomJungfrau::ChipPixels.x;
    SCI chip_lines = GeomJungfrau::ChipPixels.y;
    SCA chip_gap_pixels = GeomJungfrau::ChipGap;
    SCA mod_gap_pixels = GeomJungfrau::ModGap;
    SCI frame_packets = PacketData::PacketsPerFrame;
    SCI packet_lines = RawIfaceSize.y / frame_packets;
    SCI flipped = (RecvView.pixelDir().y < 0);
    SCI src_pixel_size = SrcPixel::depth();
    SCI src_chip_size = chip_cols * src_pixel_size;
    SCI src_line_size = RawIfaceSize.x * src_pixel_size;
    SCI src_dir = flipped ? -1 : 1;
    SCI src_line_step = src_line_size * src_dir;
    SCI dst_pixel_size = DstPixel::depth();
    SCI dst_chip_pixels = IfaceGeom1.chip_step.x;
    SCI dst_chip_size = dst_chip_pixels * dst_pixel_size;
    SCI dst_line_size = RecvView.pixelStep().y * dst_pixel_size * src_dir;
    SCI dst_iface_cols = IfaceView1.size.x + (MGX ? mod_gap_pixels.x : 0);
    SCI dst_iface_line_size = dst_iface_cols * dst_pixel_size;
    SCI cg_cols_size = chip_gap_pixels.x * dst_pixel_size;
    SCI mg_cols_size = mod_gap_pixels.x * dst_pixel_size;
    SCI src_first_line = IfaceView1.calcViewOrigin().y;
    SCI src_first_packet = src_first_line / packet_lines;
    SCA first_packet_view = getPacketView(src_first_packet);
    SCA first_packet_offset = first_packet_view.calcViewOrigin();
    SCI src_offset = first_packet_offset.y * src_line_size;
    SCI dst_iface_step = RecvGeom.iface_step.y * dst_line_size;

#undef SCI
#undef SCA
};

/**
 * CopyHelper
 */

template <class GD, bool MGX, bool MGY, int Idx>
struct CopyHelper : GeomHelper<GD, MGX, MGY, Idx> {

    using H = GeomHelper<GD, MGX, MGY, Idx>;
    using BlockPtr = typename H::BlockPtr;

    static void assemblePackets(BlockPtr block, char *buf);
};

template <class GD, bool MGX, bool MGY, int Idx>
void CopyHelper<GD, MGX, MGY, Idx>::assemblePackets(BlockPtr block, char *buf) {
    H h;
    char *d = buf;
    int line = 0;
    int packet = h.src_first_packet;
    for (int p = 0; p < h.frame_packets; ++p, packet += h.src_dir) {
        auto line_packet = (*block)[packet];
        char *s = line_packet.data() + h.src_offset;
        for (int l = 0; l < h.packet_lines; ++l, ++line) {
            char *ld = d;
            char *ls = s;
            for (int c = 0; c < IfaceHorzChips; ++c) {
                if (line_packet.valid())
                    memcpy(ld, ls, h.src_chip_size);
                else
                    memset(ld, 0xff, h.src_chip_size);
                ls += h.src_chip_size;
                ld += h.src_chip_size;
                bool fill_chip_gap_cols = (c < IfaceHorzChips - 1);
                constexpr bool fill_mod_gap_cols = MGX;
                if (fill_chip_gap_cols)
                    memset(ld, 0, h.cg_cols_size);
                else if constexpr (fill_mod_gap_cols)
                    memset(ld, 0, h.mg_cols_size);
                ld += h.dst_chip_size - h.src_chip_size;
            }
            s += h.src_line_step;
            d += h.dst_line_size;
        }
        bool fill_chip_gap_lines = ((Idx == 0) && (line == h.chip_lines));
        if (fill_chip_gap_lines) {
            for (int i = 0; i < h.chip_gap_pixels.y; ++i) {
                memset(d, 0, h.dst_iface_line_size);
                d += h.dst_line_size;
            }
        }
    }
    constexpr bool fill_mod_gap_lines = MGY;
    if constexpr (fill_mod_gap_lines) {
        for (int i = 0; i < h.mod_gap_pixels.y; ++i) {
            memset(d, 0, h.dst_iface_line_size);
            d += h.dst_line_size;
        }
    }
}

/**
 * FrameAssembler
 */

template <class GD, bool MGX, bool MGY>
template <int Idx>
void FrameAssembler<GD, MGX, MGY>::Worker::assembleIface(
    AnyPacketBlockPtr &&block) {
    using Helper = CopyHelper<GD, MGX, MGY, Idx>;
    using BlockPtr = typename Helper::BlockPtr;

    if (!std::holds_alternative<BlockPtr>(block))
        throw std::runtime_error("Invalid packet block");

    BlockPtr b = std::get<BlockPtr>(std::move(block));
    int packet_count = b ? b->getValidPackets() : 0;
    if (packet_count == 0)
        return;

    mask.set(Idx, true);
    det_header->packetNumber += packet_count;

    // write header
    if (header_empty) {
        auto p = (*b)[0];
        p.fillDetHeader(det_header);
        header_empty = false;
    }

    Helper::assemblePackets(std::move(b), buf);
    if (buf)
        buf += Helper::dst_iface_step;
}

template <class GD, bool MGX, bool MGY>
Result FrameAssembler<GD, MGX, MGY>::Worker::result() {
    return Result{NbUDPIfaces, mask};
}

template <class GD, bool MGX, bool MGY>
Result FrameAssembler<GD, MGX, MGY>::assembleFrame(AnyPacketBlockList &&blocks,
                                                   RecvHeader *recv_header,
                                                   char *buf) {
    if (blocks.size() != std::size_t(NbUDPIfaces))
        throw std::runtime_error("Invalid packet block list");

    if (buf)
        buf += data_offset;

    Worker w(recv_header, buf);

    for (int i = 0; i < NbUDPIfaces; ++i) {
        if (i == 0)
            w.template assembleIface<0>(std::move(blocks[0]));
        else if constexpr (NbUDPIfaces == 2)
            w.template assembleIface<1>(std::move(blocks[1]));
    }

    return w.result();
}

} // namespace Jungfrau
} // namespace FrameAssembler

MPFrameAssemblerPtr
FrameAssembler::Jungfrau::CreateFrameAssembler(GeneralDataPtr gd, XY det_ifaces,
                                               XY mod_pos) {

    auto any_nb_ifaces =
        GeomJungfrau::AnyNbUDPIfacesFromNbUDPIfaces(gd->numUDPInterfaces);

    return std::visit(
        [&](auto nb_ifaces) {
            constexpr int NbUDPIfaces = nb_ifaces;
            constexpr XY iface_size = RawIfaceGeom<NbUDPIfaces, 0>.size;
            XY det_size = iface_size * det_ifaces;
            auto any_det_geom =
                GeomJungfrau::AnyDetGeomFromDetSize<NbUDPIfaces>(det_size);

            return std::visit(
                [&](auto gd) {
                    using GD = decltype(gd);
                    constexpr auto det_geom = GD::asm_wg_geom;
                    auto mod_view = det_geom.getModView(mod_pos);
                    auto origin = mod_view.calcViewOrigin();
                    int pixel_offset = mod_view.calcMapPixelIndex(origin);
                    int data_offset = pixel_offset * Pixel::depth();
                    auto any_fill =
                        AnyModGapFillingFromModPos(det_geom, mod_pos);
                    return std::visit(
                        [&, NbUDPIfaces](auto gx,
                                         auto gy) -> MPFrameAssemblerPtr {
                            constexpr bool MGX = gx, MGY = gy;
                            using Assembler = FrameAssembler<GD, MGX, MGY>;
                            return std::make_shared<Assembler>(data_offset);
                        },
                        any_fill.x, any_fill.y);
                },
                any_det_geom);
        },
        any_nb_ifaces);
}
