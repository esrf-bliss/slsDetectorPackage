#pragma once
/************************************************
 * @file Geometry.h
 * @short Detector geometry helpers for image reconstruction.
 * Compile-time (constexpr) arithmetics is used in order to
 * optimize frame assembling tasks
 ***********************************************/

#include <functional>
#include <iostream>
#include <type_traits>
#include <utility>
#include <variant>

namespace sls {
namespace Geom {

// XY: the basic class for 2D arithmetics
struct XY {
    std::ptrdiff_t x{0};
    std::ptrdiff_t y{0};

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

// Helper pixel iterator
template <typename UnaryFunction>
UnaryFunction view_for_each_pixel(const MapView &view, UnaryFunction f) {
    for (int pixely = 0; pixely < view.size.y; ++pixely)
        for (int pixelx = 0; pixelx < view.size.x; ++pixelx)
            f(view, XY{pixelx, pixely});

    return f;
}

template <typename BinaryFunction>
BinaryFunction view_for_each_pixel(const MapView &view1, const MapView &view2,
                                   BinaryFunction f) {
    for (int pixely = 0; pixely < view1.size.y; ++pixely)
        for (int pixelx = 0; pixelx < view1.size.x; ++pixelx)
            f(view1, view2, XY{pixelx, pixely});

    return f;
}

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
 * Asm format: the geometry is reconstructed
 */

template <bool WithGap> constexpr auto EffectiveGap(const XY &gap) {
    return WithGap ? gap : XY0;
}

constexpr auto CalcAsmArraySize(const XY &elem_size, const XY &array_size,
                                const XY &gap) {
    return (elem_size * array_size + (array_size - XY1) * gap);
}

constexpr auto GetAsmElementView(const XY &elem_size, const XY &gap,
                                 const XY &idx, const MapView &view,
                                 const XY &flip) {
    return view.getSubView((elem_size + gap) * idx, elem_size, flip);
}

/*
 * AsmWithNoGap format: the geometry is reconstructed, no chip/mod gap pixels
 */

struct AsmWithNoGapFmt {
    static constexpr auto calcArraySize(const XY &elem_size,
                                        const XY &array_size, const XY &gap) {
        return CalcAsmArraySize(elem_size, array_size, gap);
    }

    static constexpr auto getElementView(const XY &elem_size,
                                         const XY & /*array_size*/,
                                         const XY &gap, const XY &idx,
                                         const MapView &view,
                                         const XY &flip = NoFlip) {
        return GetAsmElementView(elem_size, gap, idx, view, flip);
    }
};

/*
 * AsmWithGap format: the geometry is reconstructed with chip/mod gap pixels
 */

struct AsmWithGapFmt {
    static constexpr auto calcArraySize(const XY &elem_size,
                                        const XY &array_size, const XY &gap) {
        return CalcAsmArraySize(elem_size, array_size, gap);
    }

    static constexpr auto getElementView(const XY &elem_size,
                                         const XY & /*array_size*/,
                                         const XY &gap, const XY &idx,
                                         const MapView &view,
                                         const XY &flip = NoFlip) {
        return GetAsmElementView(elem_size, gap, idx, view, flip);
    }
};

template <class Fmt> constexpr bool IsRaw() {
    return std::is_same<Fmt, RawFmt>::value;
}

template <class Fmt> constexpr auto EffectiveFmtGap(const XY &gap) {
    return EffectiveGap<std::is_same<Fmt, AsmWithGapFmt>::value>(gap);
}

// IfaceGeom: multi-chip UDP interface geometry
template <class Fmt> struct IfaceGeom {
    XY chip_pixels, chip_gap, iface_chips;
    MapView view;
    XY iface_idx, det_ifaces;

    XY raw_size, asm_size, size;
    XY chip_step;

    constexpr IfaceGeom(const XY &cp, const XY &cg, const XY &ic,
                        const MapView &iv, const XY &ii = XY0,
                        const XY &di = XY1)
        : chip_pixels(cp), chip_gap(EffectiveFmtGap<Fmt>(cg)), iface_chips(ic),
          view(iv), iface_idx(ii), det_ifaces(di),
          raw_size(chip_pixels * iface_chips),
          asm_size(raw_size + (iface_chips - XY1) * chip_gap),
          size(IsRaw<Fmt>() ? raw_size : asm_size),
          chip_step(chip_pixels + (IsRaw<Fmt>() ? XY0 : chip_gap)) {}

    constexpr auto getChipView(const XY &chip_idx) const {
        auto offset = (IsRaw<Fmt>() ? XY0 : (chip_gap * chip_idx));
        return view.getSubView(chip_idx * chip_pixels + offset, chip_pixels);
    }

    constexpr int pixels() const { return raw_size.area(); }

    constexpr int calcPacketLines(int packet_pixels) const {
        return raw_size.y * packet_pixels / pixels();
    }
    constexpr auto getPacketView(int packet_pixels, int packet_number) const {
        XY packet_geom{size.x, calcPacketLines(packet_pixels)};
        return view.getSubView(packet_geom * XY{0, packet_number}, packet_geom);
    }
};

// Helper chip iterator
template <class IG, typename UnaryFunction>
void iface_for_each_chip(const IG &iface_geom, UnaryFunction f) {
    for (int chipy = 0; chipy < iface_geom.iface_chips.y; ++chipy) {
        for (int chipx = 0; chipx < iface_geom.iface_chips.x; ++chipx) {
            XY chip = {chipx, chipy};
            f(chip, iface_geom.getChipView(chip));
        }
    }
}

template <class IG1, class IG2, typename BinaryFunction>
void iface_for_each_chip(const IG1 &iface_geom1, const IG2 &iface_geom2,
                         BinaryFunction f) {
    for (int chipy = 0; chipy < iface_geom1.iface_chips.y; ++chipy) {
        for (int chipx = 0; chipx < iface_geom1.iface_chips.x; ++chipx) {
            XY chip = {chipx, chipy};
            auto chip_view1 = iface_geom1.getChipView(chip);
            auto chip_view2 = iface_geom2.getChipView(chip);
            f(chip, chip_view1, chip_view2);
        }
    }
}

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
        : chip_pixels(cp), chip_gap(EffectiveFmtGap<Fmt>(cg)), iface_chips(ic),
          recv_ifaces(ri), view(rv), recv_idx(ridx), det_recvs(dr),
          iface_geom_size(
              IfaceGeom<Fmt>(chip_pixels, chip_gap, iface_chips, EmptyView)
                  .size),
          size(Fmt::calcArraySize(iface_geom_size, recv_ifaces, chip_gap)),
          iface_step(iface_geom_size + (IsRaw<Fmt>() ? XY0 : chip_gap)) {}

    constexpr auto getIfacePos(const XY &iface_idx) const {
        auto recv_flip = view.flipped;
        auto offset = recv_flip * (recv_ifaces - XY1);
        XY dir{!recv_flip.x ? 1 : -1, !recv_flip.y ? 1 : -1};
        return iface_idx * dir + offset;
    }

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

// Helper iface iterator
template <class RG, typename UnaryFunction>
UnaryFunction recv_for_each_iface(const RG &recv_geom, UnaryFunction f) {
    for (int ifacey = 0; ifacey < recv_geom.recv_ifaces.y; ++ifacey) {
        for (int ifacex = 0; ifacex < recv_geom.recv_ifaces.x; ++ifacex) {
            XY iface = {ifacex, ifacey};
            f(iface, recv_geom.getIfaceGeom(iface));
        }
    }

    return f;
}

template <class RG1, class RG2, typename BinaryFunction>
BinaryFunction recv_for_each_iface(const RG1 &recv_geom1, const RG2 &recv_geom2,
                                   BinaryFunction f) {
    for (int ifacey = 0; ifacey < recv_geom1.recv_ifaces.y; ++ifacey) {
        for (int ifacex = 0; ifacex < recv_geom1.recv_ifaces.x; ++ifacex) {
            XY iface = {ifacex, ifacey};
            auto iface_geom1 = recv_geom1.getIfaceGeom(iface);
            auto iface_geom2 = recv_geom2.getIfaceGeom(iface);
            f(iface, iface_geom1, iface_geom2);
        }
    }

    return f;
}

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
        : chip_pixels(cp), chip_gap(EffectiveFmtGap<Fmt>(cg)), iface_chips(ic),
          recv_ifaces(ri), mod_recvs(mr), view(mv), mod_idx(mi), det_mods(dm),
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

// Helper recv iterator
template <class MG, typename UnaryFunction>
UnaryFunction mod_for_each_recv(const MG &mod_geom, UnaryFunction f) {
    for (int recvy = 0; recvy < mod_geom.mod_recvs.y; ++recvy) {
        for (int recvx = 0; recvx < mod_geom.mod_recvs.x; ++recvx) {
            XY recv = {recvx, recvy};
            f(recv, mod_geom.getRecvGeom(recv));
        }
    }

    return f;
}

template <class MG1, class MG2, typename BinaryFunction>
BinaryFunction mod_for_each_recv(const MG1 &mod_geom1, const MG2 &mod_geom2,
                                 BinaryFunction f) {
    for (int recvy = 0; recvy < mod_geom1.mod_recvs.y; ++recvy) {
        for (int recvx = 0; recvx < mod_geom1.mod_recvs.x; ++recvx) {
            XY recv = {recvx, recvy};
            auto recv_geom1 = mod_geom1.getRecvGeom(recv);
            auto recv_geom2 = mod_geom2.getRecvGeom(recv);
            f(recv, recv_geom1, recv_geom2);
        }
    }

    return f;
}

// DetGeom: multi-module detector geometry
template <class Fmt, class ModRecvFlip> struct DetGeom {
    XY chip_pixels, chip_gap, iface_chips, recv_ifaces, mod_recvs, mod_gap;
    XY det_mods;

    XY mod_geom_size, size;
    XY mod_step;
    MapView view;

    constexpr DetGeom(const XY &cp, const XY &cg, const XY &ic, const XY &ri,
                      const XY &mr, const XY &mg, const XY &dm)
        : chip_pixels(cp), chip_gap(EffectiveFmtGap<Fmt>(cg)), iface_chips(ic),
          recv_ifaces(ri), mod_recvs(mr), mod_gap(EffectiveFmtGap<Fmt>(mg)),
          det_mods(dm),
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

// Helper module iterator
template <class DG, typename UnaryFunction>
UnaryFunction det_for_each_mod(const DG &det_geom, UnaryFunction f) {
    for (int mody = 0; mody < det_geom.det_mods.y; ++mody) {
        for (int modx = 0; modx < det_geom.det_mods.x; ++modx) {
            XY mod = {modx, mody};
            f(mod, det_geom.getModGeom(mod));
        }
    }

    return f;
}

template <class DG1, class DG2, typename BinaryFunction>
BinaryFunction det_for_each_mod(const DG1 &det_geom1, const DG2 &det_geom2,
                                BinaryFunction f) {
    for (int mody = 0; mody < det_geom1.det_mods.y; ++mody) {
        for (int modx = 0; modx < det_geom1.det_mods.x; ++modx) {
            XY mod = {modx, mody};
            auto mod_geom1 = det_geom1.getModGeom(mod);
            auto mod_geom2 = det_geom2.getModGeom(mod);
            f(mod, mod_geom1, mod_geom2);
        }
    }

    return f;
}

// Helper detector iterator

template <class DG, typename UnaryFunction>
UnaryFunction det_for_each_chip(const DG &det_geom, UnaryFunction f) {
    det_for_each_mod(det_geom, [&](auto const &mod, auto const &mod_geom) {
        mod_for_each_recv(
            mod_geom, [&](auto const &recv, auto const &recv_geom) {
                recv_for_each_iface(recv_geom, [&](auto const &iface,
                                                   auto const &iface_geom) {
                    iface_for_each_chip(iface_geom, [&](auto const &chip,
                                                        auto const &chip_view) {
                        auto first_chip =
                            (iface_geom.iface_idx * iface_geom.iface_chips);
                        f(first_chip + chip, chip_view);
                    });
                });
            });
    });

    return f;
}

template <class DG, typename UnaryFunction>
UnaryFunction det_for_each_pixel(const DG &det_geom, UnaryFunction f) {
    det_for_each_chip(det_geom, [&](auto const &chip, auto const &chip_view) {
        view_for_each_pixel(chip_view, f);
    });

    return f;
}

template <class DG1, class DG2, typename BinaryFunction>
BinaryFunction det_for_each_chip(const DG1 &det_geom1, const DG2 &det_geom2,
                                 BinaryFunction f) {
    det_for_each_mod(
        det_geom1, det_geom2,
        [&](auto const &mod, auto const &mod_geom1, auto const &mod_geom2) {
            mod_for_each_recv(
                mod_geom1, mod_geom2,
                [&](auto const &recv, auto const &recv_geom1,
                    auto const &recv_geom2) {
                    recv_for_each_iface(
                        recv_geom1, recv_geom2,
                        [&](auto const &iface, auto const &iface_geom1,
                            auto const &iface_geom2) {
                            iface_for_each_chip(
                                iface_geom1, iface_geom2,
                                [&](auto const &chip, auto const &chip_view1,
                                    auto const &chip_view2) {
                                    auto first_chip = (iface_geom1.iface_idx *
                                                       iface_geom1.iface_chips);
                                    f(first_chip + chip, chip_view1,
                                      chip_view2);
                                });
                        });
                });
        });

    return f;
}

template <class DG1, class DG2, typename BinaryFunction>
BinaryFunction det_for_each_pixel(const DG1 &det_geom1, const DG2 &det_geom2,
                                  BinaryFunction f) {
    det_for_each_chip(
        det_geom1, det_geom2,
        [&](auto const &chip, auto const &chip_view1, auto const &chip_view2) {
            view_for_each_pixel(chip_view1, chip_view2, f);
        });

    return f;
}

// Detector geometry data: stores all geometry for a particular detector
// MX, MY: detector modules, G: detector geometry generator
template <int MX, int MY, template <int x, int y, class Fmt> class G>
struct DetGeomData {
    static constexpr auto raw_geom = G<MX, MY, RawFmt>()();
    static constexpr auto asm_ng_geom = G<MX, MY, AsmWithNoGapFmt>()();
    static constexpr auto asm_wg_geom = G<MX, MY, AsmWithGapFmt>()();
};

// Detector collections: variants of all possible detector geometries

template <class V, std::size_t N = std::variant_size_v<V> - 1>
constexpr V GetValidVariant(std::size_t i) {
    if (i == N)
        return V(std::in_place_index_t<N>());
    if constexpr (N > 0)
        return GetValidVariant<V, N - 1>(i);
    else
        return V(std::in_place_index_t<N>());
}

template <class V, std::size_t I>
constexpr bool DetCollectIdxMatchesDetSize(const XY &det_size) {
    using geom_data = std::variant_alternative_t<I, V>;
    return (geom_data::asm_ng_geom.size == det_size);
}

template <class V, std::size_t I = std::variant_size_v<V> - 1>
constexpr std::size_t GetDetCollectIdxFromDetSize(const XY &det_size) {
    if (DetCollectIdxMatchesDetSize<V, I>(det_size))
        return I;
    if constexpr (I > 0)
        return GetDetCollectIdxFromDetSize<V, I - 1>(det_size);
    else
        return -1;
}

// Module gap filling

using AnyGapFilling = std::variant<std::false_type, std::true_type>;
struct AnyModGapFilling {
    AnyGapFilling x, y;
};

template <class DG>
constexpr AnyModGapFilling AnyModGapFillingFromModPos(const DG &det_geom,
                                                      const XY &mod_pos) {
    auto det_mods = det_geom.det_mods;
    XY has_gap{mod_pos.x < det_mods.x - 1, mod_pos.y < det_mods.y - 1};
    return {GetValidVariant<AnyGapFilling>(has_gap.x),
            GetValidVariant<AnyGapFilling>(has_gap.y)};
}
}; // namespace Geom
}; // namespace sls
