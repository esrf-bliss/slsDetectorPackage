/************************************************
 * @file Jungfrau/FrameAssembler.cxx
 * @short helper classes assembling Jungfrau frames
 * from udp packets
 ***********************************************/

namespace sls {
namespace Jungfrau {
namespace FrameAssembler {

constexpr int IfaceHorzChips =
    Geom::IfaceChips<sls::Jungfrau::Geom::OneIface>.x;

template <typename NbUDPIfaces, int Idx>
constexpr auto RawIfaceGeom = sls::Jungfrau::RawIfaceGeom<NbUDPIfaces, Idx>;

/**
 * GeomHelper
 */

//  GD: Geom data, MGX/Y: Module gap X/Y
template <class GD, bool MGX, bool MGY, int Idx> struct GeomHelper {

    using SrcPixel = Pixel;
    using DstPixel = Pixel;

#define SCA static constexpr auto
#define SCI static constexpr int

    using NbUDPIfaces = typename GD::num_udp_ifaces;
    SCI single_udp_iface = (NbUDPIfaces::NbIfaces == 1);

    using BlockPtr = PacketBlockPtr<Packet<NbUDPIfaces>>;
    using ConstBlockPtr =
        std::add_pointer_t<std::add_const_t<typename BlockPtr::element_type>>;

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

    SCI chip_cols = Geom::ChipPixels.x;
    SCI chip_lines = Geom::ChipPixels.y;
    SCA chip_gap_pixels = Geom::ChipGap;
    SCA mod_gap_pixels = Geom::ModGap;
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
    SCI correct_src_first_line = (single_udp_iface && flipped);
    SCI src_first_line_offset = correct_src_first_line ? chip_gap_pixels.y : 0;
    SCI src_first_line = IfaceView1.calcViewOrigin().y - src_first_line_offset;
    SCI src_first_packet = src_first_line / packet_lines;
    SCA first_packet_view = getPacketView(src_first_packet);
    SCA first_packet_offset = first_packet_view.calcViewOrigin();
    SCI src_offset = first_packet_offset.y * src_line_size;
    SCI dst_iface_step = RecvGeom.iface_step.y * dst_line_size;
    SCI dst_iface_pos = RecvGeom.getIfacePos(XY{0, Idx}).y;
    SCI dst_iface_offset = dst_iface_pos * dst_iface_step;
    SCA fill_mod_gap_cols = MGX;
    SCA bottom_iface = (single_udp_iface || (dst_iface_pos == 1));
    SCA fill_mod_gap_lines = (MGY && bottom_iface);

#undef SCI
#undef SCA
};

/**
 * CopyHelper
 */

template <class GD, bool MGX, bool MGY, int Idx>
struct CopyHelper : GeomHelper<GD, MGX, MGY, Idx> {

    using H = GeomHelper<GD, MGX, MGY, Idx>;
    using ConstBlockPtr = typename H::ConstBlockPtr;

    static void assemblePackets(ConstBlockPtr block, char *buf);
};

template <class GD, bool MGX, bool MGY, int Idx>
void CopyHelper<GD, MGX, MGY, Idx>::assemblePackets(ConstBlockPtr block,
                                                    char *buf) {
    H h;
    char *d = buf;
    int line = 0;
    int packet = h.src_first_packet;
    constexpr int pos = h.dst_iface_pos;
    using sls_bitset = slsDetectorDefs::sls_bitset;
    auto valid_packet_mask = block ? block->getValidPacketMask() : sls_bitset();
    for (int p = 0; p < h.frame_packets; ++p, packet += h.src_dir) {
        char *s = block ? ((*block)[packet].data() + h.src_offset) : nullptr;
        for (int l = 0; l < h.packet_lines; ++l, ++line) {
            char *ld = d;
            char *ls = s;
            for (int c = 0; c < IfaceHorzChips; ++c) {
                if (valid_packet_mask[packet])
                    memcpy(ld, ls, h.src_chip_size);
                else
                    memset(ld, 0xff, h.src_chip_size);
                ls += h.src_chip_size;
                ld += h.src_chip_size;
                bool fill_chip_gap_cols = (c < IfaceHorzChips - 1);
                if (fill_chip_gap_cols)
                    memset(ld, 0, h.cg_cols_size);
                else if constexpr (h.fill_mod_gap_cols)
                    memset(ld, 0, h.mg_cols_size);
                ld += h.dst_chip_size - h.src_chip_size;
            }
            s += h.src_line_step;
            d += h.dst_line_size;
        }
        bool fill_chip_gap_lines = ((pos == 0) && (line == h.chip_lines));
        if (fill_chip_gap_lines) {
            for (int i = 0; i < h.chip_gap_pixels.y; ++i) {
                memset(d, 0, h.dst_iface_line_size);
                d += h.dst_line_size;
            }
        }
    }
    if constexpr (h.fill_mod_gap_lines) {
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
bool FrameAssembler<GD, MGX, MGY>::assembleIface(const AnyPacketBlockPtr &block,
                                                 char *buf) {
    using Helper = CopyHelper<GD, MGX, MGY, Idx>;
    using BlockPtr = typename Helper::BlockPtr;
    using ConstBlockPtr = typename Helper::ConstBlockPtr;

    if (!std::holds_alternative<BlockPtr>(block))
        throw std::runtime_error("Invalid packet block");

    ConstBlockPtr b = std::get<BlockPtr>(block).get();

    if (buf) {
        auto offset = data_offset + Helper::dst_iface_offset;
        Helper::assemblePackets(b, buf + offset);
    }

    return b && (b->getValidPackets() > 0);
}

template <class GD, bool MGX, bool MGY>
Result
FrameAssembler<GD, MGX, MGY>::assembleFrame(const AnyPacketBlockList &blocks,
                                            char *buf) {
    if (blocks.size() != std::size_t(NbIfaces))
        throw std::runtime_error("Invalid packet block list");

    PortsMask mask;
    for (int i = 0; i < NbIfaces; ++i) {
        bool ok;
        if (i == 0)
            ok = assembleIface<0>(blocks[0], buf);
        else if constexpr (NbIfaces == 2)
            ok = assembleIface<1>(blocks[1], buf);
        mask.set(i, ok);
    }

    return Result{NbIfaces, mask};
}

template <class GD, bool MGX, bool MGY>
void FrameAssembler<GD, MGX, MGY>::fillMissingFrame(char *buf) {
    using BlockPtr = typename CopyHelper<GD, MGX, MGY, 0>::BlockPtr;
    assembleIface<0>(BlockPtr(nullptr), buf);
    if constexpr (NbIfaces == 2)
        assembleIface<1>(BlockPtr(nullptr), buf);
}

template <class GD, bool MGX, bool MGY>
FrameDims FrameAssembler<GD, MGX, MGY>::getAssembledFrameDims() {
    constexpr auto geom_size = GD::asm_wg_geom.size;
    slsDetectorDefs::xy det_size{int(geom_size.x), int(geom_size.y)};
    auto nb_pixels = det_size.x * det_size.y;
    return {det_size, int(nb_pixels * Pixel::depth())};
}

inline MPFrameAssemblerPtr CreateFrameAssembler(int mod_ifaces, XY det_ifaces,
                                                XY mod_pos) {

    auto any_nb_ifaces = Geom::AnyNbUDPIfacesFromNbUDPIfaces(mod_ifaces);

    return std::visit(
        [&](auto nb_ifaces) {
            using NbUDPIfaces = decltype(nb_ifaces);
            constexpr XY iface_size = RawIfaceGeom<NbUDPIfaces, 0>.size;
            XY det_size = iface_size * det_ifaces;
            auto any_det_geom =
                Geom::AnyDetGeomFromDetSize<NbUDPIfaces>(det_size);

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
                        [&](auto gx, auto gy) -> MPFrameAssemblerPtr {
                            constexpr bool MGX = gx, MGY = gy;
                            using Assembler = FrameAssembler<GD, MGX, MGY>;
                            return std::make_unique<Assembler>(data_offset);
                        },
                        any_fill.x, any_fill.y);
                },
                any_det_geom);
        },
        any_nb_ifaces);
}

} // namespace FrameAssembler
} // namespace Jungfrau
} // namespace sls
