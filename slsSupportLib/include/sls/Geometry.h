#pragma once
/************************************************
 * @file Geometry.h
 * @short Detector geometry helpers for image reconstruction.
 * Compile-time (constexpr) arithmetics is used in order to
 * optimize frame assembling tasks
 ***********************************************/

#include <iostream>
#include <type_traits>
#include <utility>
#include <variant>

namespace sls {
namespace Geom {

// XY: the basic class for 2D arithmetics
struct XY {
    int x{0};
    int y{0};

    constexpr int area() const { return x * y; }
};

// 2D arithmetics
constexpr auto operator+(const XY &a, const XY &b) {
    return XY{a.x + b.x, a.y + b.y};
}

constexpr auto operator-(const XY &a, const XY &b) {
    return XY{a.x - b.x, a.y - b.y};
}

constexpr auto operator*(const XY &a, const XY &b) {
    return XY{a.x * b.x, a.y * b.y};
}

constexpr auto operator/(const XY &a, const XY &b) {
    return XY{a.x / b.x, a.y / b.y};
}

constexpr auto operator^(const XY &a, const XY &b) {
    return XY{a.x ^ b.x, a.y ^ b.y};
}

constexpr auto operator==(const XY &a, const XY &b) {
    return (a.x == b.x) && (a.y == b.y);
}

inline std::ostream &operator<<(std::ostream &os, const XY &a) {
    return os << "XY<" << a.x << "," << a.y << ">";
}

// Element index in a 2D array: first index is the fast one: RowWise
constexpr int RowWiseElementIndex(const XY &size, const XY &element) {
    return element.y * size.x + element.x;
}

// Element index in a 2D array: second index the fast one: ColWise
constexpr int ColWiseElementIndex(const XY &size, const XY &element) {
    return element.x * size.y + element.y;
}

constexpr XY XY0{0, 0};
constexpr XY XY1{1, 1};

// Flip modes
constexpr XY NoFlip{0, 0};
constexpr XY HorzFlip{1, 0};
constexpr XY VertFlip{0, 1};
constexpr XY HorzVertFlip{1, 1};

// View (port) of a map
struct MapView {
    XY map_size;
    XY view_origin;
    XY size;
    XY flipped;

    constexpr MapView(const XY &ms, const XY &vo, const XY &s, const XY &f)
        : map_size(ms), view_origin(vo), size(s), flipped(f) {}

    constexpr auto pixelOffset() const {
        return view_origin + flipped * (size - XY1);
    }
    constexpr auto pixelDir() const {
        return XY{!flipped.x ? 1 : -1, !flipped.y ? 1 : -1};
    }
    constexpr auto pixelStep() const { return pixelDir() * XY{1, map_size.x}; }

    constexpr auto calcMapPixel(const XY &view_pixel) const {
        return pixelOffset() + view_pixel * pixelDir();
    }
    constexpr auto calcViewPixel(const XY &map_pixel) const {
        return (map_pixel - pixelOffset()) / pixelDir();
    }
    constexpr auto calcViewOrigin() const { return calcViewPixel(view_origin); }
    constexpr auto calcMapPixelIndex(const XY &view_pixel) const {
        return RowWiseElementIndex(map_size, calcMapPixel(view_pixel));
    }

    // sub view (sv) flip is relative to current flip
    constexpr auto getSubView(const XY &sv_origin, const XY &sv_size,
                              const XY &sv_flip = NoFlip) const {
        auto c1 = calcMapPixel(sv_origin);
        auto c2 = calcMapPixel(sv_origin + sv_size - XY1);
        XY real_origin{!flipped.x ? c1.x : c2.x, !flipped.y ? c1.y : c2.y};
        XY real_flip = flipped ^ sv_flip;
        return MapView(map_size, real_origin, sv_size, real_flip);
    }
};

constexpr auto ViewFromMap(const XY &map_size) {
    return MapView(map_size, {0, 0}, map_size, {0, 0});
}

constexpr auto EmptyView = ViewFromMap({0, 0});

/*
 * Raw format: all the network Ifaces (ports) are vertically concatenated
 */

struct RawFmt {
    static constexpr auto calcArraySize(const XY &elem_size,
                                        const XY &array_size,
                                        const XY & /*gap*/) {
        return elem_size * XY{1, array_size.area()};
    }

    static constexpr auto getElementView(const XY &elem_size,
                                         const XY &array_size,
                                         const XY & /*gap*/, const XY &idx,
                                         const MapView &view,
                                         const XY &flip = NoFlip) {
        XY raw_idx{0, RowWiseElementIndex(array_size, idx)};
        return view.getSubView(elem_size * raw_idx, elem_size, flip);
    }
};

/*
 * Std format: the geometry is reconstructed including chip/mod gap pixels
 */

struct StdFmt {
    static constexpr auto calcArraySize(const XY &elem_size,
                                        const XY &array_size, const XY &gap) {
        return elem_size * array_size + (array_size - XY1) * gap;
    }

    static constexpr auto getElementView(const XY &elem_size,
                                         const XY &array_size, const XY &gap,
                                         const XY &idx, const MapView &view,
                                         const XY &flip = NoFlip) {
        return view.getSubView((elem_size + gap) * idx, elem_size, flip);
    }
};

template <class Fmt> constexpr bool IsRaw() {
    return std::is_same<Fmt, RawFmt>::value;
}

// IfaceGeom: multi-chip UDP interface geometry
template <class Fmt> struct IfaceGeom {
    XY chip_pixels, chip_gap, iface_chips;
    MapView view;
    XY iface_idx, det_ifaces;

    XY raw_size, std_size, size;
    XY chip_step;

    constexpr IfaceGeom(const XY &cp, const XY &cg, const XY &ic,
                        const MapView &iv, const XY &ii = XY0,
                        const XY &di = XY1)
        : chip_pixels(cp), chip_gap(cg), iface_chips(ic), view(iv),
          iface_idx(ii), det_ifaces(di), raw_size(chip_pixels * iface_chips),
          std_size(raw_size + (iface_chips - XY1) * chip_gap),
          size(IsRaw<Fmt>() ? raw_size : std_size),
          chip_step(chip_pixels + (IsRaw<Fmt>() ? XY0 : chip_gap)) {}

    constexpr auto getChipView(const XY &chip_idx) const {
        auto offset = (IsRaw<Fmt>() ? XY0 : (chip_gap * chip_idx));
        return view.getSubView(chip_idx * chip_pixels + offset, chip_pixels);
    }

    // PacketPixels: PacketDataLen / Pixel::depth()
    constexpr int calcFramePackets(int packet_pixels) const {
        return raw_size.area() / packet_pixels;
    }
    constexpr int calcPacketLines(int packet_pixels) const {
        return raw_size.y / calcFramePackets(packet_pixels);
    }
    constexpr auto getPacketView(int packet_pixels, int packet_number) const {
        XY packet_geom{size.x, calcPacketLines(packet_pixels)};
        return view.getSubView(packet_geom * XY{0, packet_number}, packet_geom);
    }
};

// RecvGeom: multi-UDP interface receiver geometry
template <class Fmt> struct RecvGeom {
    XY chip_pixels, chip_gap, iface_chips, recv_ifaces;
    MapView view;
    XY recv_idx, det_recvs;

    XY iface_geom_size, size;
    XY iface_step;

    constexpr RecvGeom(const XY &cp, const XY &cg, const XY &ic, const XY &ri,
                       const MapView &rv, const XY &ridx = XY0,
                       const XY &dr = XY1)
        : chip_pixels(cp), chip_gap(cg), iface_chips(ic), recv_ifaces(ri),
          view(rv), recv_idx(ridx), det_recvs(dr),
          iface_geom_size(
              IfaceGeom<Fmt>(chip_pixels, chip_gap, iface_chips, EmptyView)
                  .size),
          size(Fmt::calcArraySize(iface_geom_size, recv_ifaces, chip_gap)),
          iface_step(iface_geom_size + (IsRaw<Fmt>() ? XY0 : chip_gap)) {}

    constexpr auto getIfaceView(const XY &iface_idx) const {
        return Fmt::getElementView(iface_geom_size, recv_ifaces, chip_gap,
                                   iface_idx, view);
    }

    constexpr auto getIfaceGeom(const XY &iface_idx) const {
        auto det_iface_idx = recv_ifaces * recv_idx + iface_idx;
        auto det_ifaces = det_recvs * recv_ifaces;
        return IfaceGeom<Fmt>(chip_pixels, chip_gap, iface_chips,
                              getIfaceView(iface_idx), det_iface_idx,
                              det_ifaces);
    }
};

struct DefaultModRecvFlip {
    static constexpr auto getRecvFlip(const XY & /*recv_idx*/) {
        return NoFlip;
    }
};

// ModGeom: multi-receiver module geometry
template <class Fmt, class ModRecvFlip> struct ModGeom {
    XY chip_pixels, chip_gap, iface_chips, recv_ifaces, mod_recvs;
    MapView view;
    XY mod_idx, det_mods;

    XY recv_geom_size, size;
    XY recv_step;

    constexpr ModGeom(const XY &cp, const XY &cg, const XY &ic, const XY &ri,
                      const XY &mr, const MapView &mv, const XY &mi = XY0,
                      const XY &dm = XY1)
        : chip_pixels(cp), chip_gap(cg), iface_chips(ic), recv_ifaces(ri),
          mod_recvs(mr), view(mv), mod_idx(mi), det_mods(dm),
          recv_geom_size(RecvGeom<Fmt>(chip_pixels, chip_gap, iface_chips,
                                       recv_ifaces, EmptyView)
                             .size),
          size(Fmt::calcArraySize(recv_geom_size, mod_recvs, chip_gap)),
          recv_step(recv_geom_size + (IsRaw<Fmt>() ? XY0 : chip_gap)) {}

    constexpr auto getRecvView(const XY &recv_idx) const {
        using RecvFlip =
            std::conditional_t<IsRaw<Fmt>(), DefaultModRecvFlip, ModRecvFlip>;
        return Fmt::getElementView(recv_geom_size, mod_recvs, chip_gap,
                                   recv_idx, view,
                                   RecvFlip::getRecvFlip(recv_idx));
    }

    constexpr auto getRecvGeom(const XY &recv_idx) const {
        auto det_recv_idx = mod_recvs * mod_idx + recv_idx;
        auto det_recvs = det_mods * mod_recvs;
        return RecvGeom<Fmt>(chip_pixels, chip_gap, iface_chips, recv_ifaces,
                             getRecvView(recv_idx), det_recv_idx, det_recvs);
    }
};

// DetGeom: multi-module detector geometry
template <class Fmt, class ModRecvFlip> struct DetGeom {
    XY chip_pixels, chip_gap, iface_chips, recv_ifaces, mod_recvs, mod_gap;
    XY det_mods;

    XY mod_geom_size, size;
    XY mod_step;
    MapView view;

    constexpr DetGeom(const XY &cp, const XY &cg, const XY &ic, const XY &ri,
                      const XY &mr, const XY &mg, const XY &dm)
        : chip_pixels(cp), chip_gap(cg), iface_chips(ic), recv_ifaces(ri),
          mod_recvs(mr), mod_gap(mg), det_mods(dm),
          mod_geom_size(ModGeom<Fmt, ModRecvFlip>(chip_pixels, chip_gap,
                                                  iface_chips, recv_ifaces,
                                                  mod_recvs, EmptyView)
                            .size),
          size(Fmt::calcArraySize(mod_geom_size, det_mods, mod_gap)),
          mod_step(mod_geom_size + (IsRaw<Fmt>() ? XY0 : mod_gap)),
          view(ViewFromMap(size)) {}

    constexpr auto getModView(const XY &mod_idx) const {
        return Fmt::getElementView(mod_geom_size, det_mods, mod_gap, mod_idx,
                                   view);
    }

    constexpr auto getModGeom(const XY &mod_idx) const {
        return ModGeom<Fmt, ModRecvFlip>(
            chip_pixels, chip_gap, iface_chips, recv_ifaces, mod_recvs,
            getModView(mod_idx), mod_idx, det_mods);
    }
};

// Pixel types
template <typename T> struct PixelIterator {
    using pointer_type = std::add_pointer_t<T>;
    pointer_type ptr{nullptr};

    PixelIterator(pointer_type p) : ptr(p) {}

    T value() { return *ptr; }

    PixelIterator &operator++() { return ++ptr, *this; }
    PixelIterator &operator--() { return --ptr, *this; }
    PixelIterator operator++(int) {
        PixelIterator x(*this);
        operator++();
        return x;
    }
    PixelIterator operator--(int) {
        PixelIterator x(*this);
        operator--();
        return x;
    }
};

struct Pixel4Iterator {
    using pointer_type = std::add_pointer_t<uint8_t>;
    pointer_type ptr{nullptr};
    bool low{false};

    Pixel4Iterator(uint8_t *p) : ptr(p) {}

    Pixel4Iterator &operator++() {
        if (!(low = !low))
            ++ptr;
        return *this;
    }
    Pixel4Iterator &operator--() {
        if ((low = !low))
            --ptr;
        return *this;
    }
    Pixel4Iterator operator++(int) {
        Pixel4Iterator x(*this);
        operator++();
        return x;
    }
    Pixel4Iterator operator--(int) {
        Pixel4Iterator x(*this);
        operator--();
        return x;
    }

    uint8_t value() { return low ? low_bits(*ptr) : high_bits(*ptr); }

    static uint8_t low_bits(uint8_t v) { return v & 15; }
    static uint8_t high_bits(uint8_t v) { return v >> 4; }
};

template <int Bpp, class V> struct Pixel {
    using bits_per_pixel = std::integral_constant<int, Bpp>;
    using iterator =
        std::conditional_t<Bpp == 4, Pixel4Iterator, PixelIterator<V>>;

    using val_type = V;
    using pointer_type = typename iterator::pointer_type;

    static constexpr float depth() { return float(Bpp) / 8; }
};

using Pixel4 = Pixel<4, uint8_t>;
using Pixel8 = Pixel<8, uint8_t>;
using Pixel16 = Pixel<16, uint16_t>;
using Pixel32 = Pixel<32, uint32_t>;

using AnyPixel = std::variant<Pixel4, Pixel8, Pixel16, Pixel32>;

inline AnyPixel AnyPixelFromBpp(int bpp) {
    switch (bpp) {
    case 4:
        return Pixel4();
    case 8:
        return Pixel8();
    case 16:
        return Pixel16();
    case 32:
        return Pixel32();
    default:
        throw std::runtime_error("Invalid Bpp: " + std::to_string(bpp));
    }
};

// Alignment helpers
constexpr int AlignCeil(int x, int align) {
    int missalign = x % align;
    return x + (!missalign ? 0 : (align - missalign));
}

constexpr int AlignFloor(int x, int align) { return x - (x % align); }

/*
// Image strides
template <class Pixel, class Size, int RowSize = Pixel::depth() * Size::x(),
          int Align = 1>
struct DefaultPixel2DArray {
    using pixel = Pixel;
    using size = Size;
    static constexpr int row_size = RowSize;
    static constexpr int stride = AlignCeil<row_size, Align>();

    template <int X, int Y> constexpr auto calcPixelByteOffset(XY<X, Y> pos) {
        return Y * stride + X * Pixel::depth();
    }
};

// Image/View: 2D pixel array in memory

template <class Image, class MapView> struct ImageView {
    using image = Image;
    using pixel = typename image::pixel;
    using pixel_pointer = typename pixel::pointer_type;
    using map_view = MapView;

    template <class Pos> constexpr auto calcPixelByteOffset(Pos pos) {
        return image::pixel_array::calcPixelByteOffset(pos);
    }

    struct iterator {
        typename pixel::iterator it;
        int col{0};

        auto operator++() {
            ++it;
            constexpr int cols = map_view::size::x();
            if (++col == cols) {
                char *p = reinterpret_cast<char *>(it.ptr);
                p += image::stride - cols * pixel::depth;
                it.ptr = reinterpret_cast<pixel_pointer>(p);
                col = 0;
            }
            return *this;
        }
    };

    image *im{nullptr};

    template <class MapSubView> auto getSubView(MapSubView map_sub_view) {
        return ImageView<Image, MapSubView>{im};
    }

    template <class Pos = XY0> auto getIterator(Pos pos = XY0()) {
        char *p = im->buffer + calcPixelByteOffset(pos);
        return iterator{reinterpret_cast<pixel_pointer>(p)};
    }
};

template <class Geom, class Pixel,
          class Pixel2DArray = DefaultPixel2DArray<Pixel, typename Geom::size>>
struct Image {
    using geom = Geom;
    using pixel = Pixel;
    using pixel_array = Pixel2DArray;
    using stride = typename pixel_array::stride;

    using view = ImageView<Image, typename Geom::view>;

    char *buffer{nullptr};

    auto getView() { return view{this}; }
};
*/

namespace Eiger {

// Eiger definitions and helpers
constexpr XY ChipPixels{256, 256};
constexpr XY ChipGap{2, 2};
constexpr XY IfaceChips{2, 1};
constexpr XY RecvIfaces{2, 1};
struct ModRecvFlip {
    // Eiger top recv is vertically flipped
    static constexpr auto getRecvFlip(const XY &recv_idx) {
        return (recv_idx == XY{0, 0}) ? VertFlip : NoFlip;
    }
};
constexpr XY ModRecvs{1, 2};
constexpr XY ModGap{36, 36};
constexpr XY Eiger500kDetMods{1, 1};
constexpr XY Eiger2MDetMods{1, 4};

template <class Fmt>
constexpr Geom::DetGeom<Fmt, ModRecvFlip>
    Eiger500kDetGeom(ChipPixels, ChipGap, IfaceChips, RecvIfaces, ModRecvs,
                     ModGap, Eiger500kDetMods);

template <class Fmt>
constexpr auto ModGeom = Eiger500kDetGeom<Fmt>.getModGeom(XY{0, 0});

template <class Fmt>
constexpr auto TopRecvGeom = ModGeom<Fmt>.getRecvGeom(XY{0, 0});
template <class Fmt>
constexpr auto BottomRecvGeom = ModGeom<Fmt>.getRecvGeom(XY{0, 1});

template <class Fmt, int Idx> struct RecvGeom {
    static constexpr auto geom = ModGeom<Fmt>.getRecvGeom(XY{0, Idx});
};

template <class Fmt>
using AnyRecvGeom = std::variant<RecvGeom<Fmt, 0>, RecvGeom<Fmt, 1>>;

template <class Fmt> AnyRecvGeom<Fmt> AnyRecvGeomFromIndex(int recv_idx) {
    if (recv_idx % ModRecvs.y == 0)
        return AnyRecvGeom<Fmt>(std::in_place_index_t<0>());
    else
        return AnyRecvGeom<Fmt>(std::in_place_index_t<1>());
}

template <class Fmt>
constexpr auto TopLeftIfaceGeom = TopRecvGeom<Fmt>.getIfaceGeom(XY{0, 0});
template <class Fmt>
constexpr auto TopRightIfaceGeom = TopRecvGeom<Fmt>.getIfaceGeom(XY{1, 0});
template <class Fmt>
constexpr auto BottomLeftIfaceGeom = BottomRecvGeom<Fmt>.getIfaceGeom(XY{0, 0});
template <class Fmt>
constexpr auto BottomRightIfaceGeom = BottomRecvGeom<Fmt>.getIfaceGeom(XY{1,
                                                                          0});

constexpr auto RawIfaceGeom = TopLeftIfaceGeom<RawFmt>;

template <class Fmt>
constexpr Geom::DetGeom<Fmt, ModRecvFlip>
    Eiger2MDetGeom(ChipPixels, ChipGap, IfaceChips, RecvIfaces, ModRecvs,
                   ModGap, Eiger2MDetMods);

}; // namespace Eiger

namespace Jungfrau {

// Jungfrau definitions and helpers
constexpr XY ChipPixels{256, 256};
constexpr XY ChipGap{2, 2};
template <int NbUDPIfaces> constexpr XY IfaceChips{4, 2 / NbUDPIfaces};
template <int NbUDPIfaces> constexpr XY RecvIfaces{1, NbUDPIfaces};
using ModRecvFlip = DefaultModRecvFlip;
constexpr XY ModRecvs{1, 1};
constexpr XY ModGap{36, 36};
constexpr XY Jungfrau500kDetMods{1, 1};
constexpr XY Jungfrau4MDetMods{2, 4};

template <int NbUDPIfaces, class Fmt>
constexpr Geom::DetGeom<Fmt, ModRecvFlip>
    Jungfrau500kDetGeom(ChipPixels, ChipGap, IfaceChips<NbUDPIfaces>,
                        RecvIfaces<NbUDPIfaces>, ModRecvs, ModGap,
                        Jungfrau500kDetMods);

template <int NbUDPIfaces, class Fmt>
constexpr auto
    ModGeom = Jungfrau500kDetGeom<NbUDPIfaces, Fmt>.getModGeom(XY{0, 0});

template <int NbUDPIfaces, class Fmt>
constexpr auto RecvGeom = ModGeom<NbUDPIfaces, Fmt>.getRecvGeom(XY{0, 0});

template <int NbUDPIfaces, int Idx, class Fmt>
constexpr auto IfaceGeom = RecvGeom<NbUDPIfaces, Fmt>.getIfaceGeom(XY{0, Idx});

template <int NbUDPIfaces, int Idx, class Fmt> struct Iface {
    static constexpr auto Geom = IfaceGeom<NbUDPIfaces, Idx, Fmt>;
};

template <class Fmt> using Iface1 = Iface<1, 0, Fmt>;
template <class Fmt> using Iface2Top = Iface<2, 0, Fmt>;
template <class Fmt> using Iface2Bottom = Iface<2, 0, Fmt>;

template <class Fmt>
using AnyIface = std::variant<Iface1<Fmt>, Iface2Top<Fmt>, Iface2Bottom<Fmt>>;

using AnyNbUDPIfaces = std::variant<std::integral_constant<int, 1>,
                                    std::integral_constant<int, 2>>;

inline auto AnyNbUDPIfacesFromNbUDPIfaces(int num_udp_ifaces) {
    if (num_udp_ifaces == 1)
        return AnyNbUDPIfaces(std::in_place_index_t<0>());
    else
        return AnyNbUDPIfaces(std::in_place_index_t<1>());
}

template <int NbUDPIfaces, class Fmt>
constexpr Geom::DetGeom<Fmt, ModRecvFlip>
    Jungfrau4MDetGeom(ChipPixels, ChipGap, IfaceChips<NbUDPIfaces>,
                      RecvIfaces<NbUDPIfaces>, ModRecvs, ModGap,
                      Jungfrau4MDetMods);
}; // namespace Jungfrau

}; // namespace Geom
}; // namespace sls
