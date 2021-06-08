#pragma once
/************************************************
 * @file Geometry.h
 * @short Eiger geometry definitions
 ***********************************************/

#include "sls/Geometry.h"

namespace sls {

namespace Eiger {
namespace Geom {

using namespace sls::Geom;

// Eiger definitions and helpers
constexpr XY ChipPixels{256, 256};
constexpr XY ChipGap{2, 2};
constexpr XY IfaceChips{2, 1};
constexpr XY RecvIfaces{2, 1};

struct ModRecvFlip : DefaultModRecvFlip {
    // Eiger top recv is vertically flipped
    static constexpr auto getRecvFlip(const XY &recv_idx) {
        return (recv_idx == XY{0, 0}) ? VertFlip : NoFlip;
    }
};
constexpr XY ModRecvs{1, 2};
constexpr XY ModGap{9, 36};

template <int MX, int MY, class Fmt> struct TiledDetGeom {
    constexpr auto operator()() {
        return Geom::DetGeom<Fmt, ModRecvFlip>(ChipPixels, ChipGap, IfaceChips,
                                               RecvIfaces, ModRecvs, ModGap,
                                               XY{MX, MY});
    }
};

template <int MX, int MY>
using GeomDataBase = DetGeomData<MX, MY, TiledDetGeom>;

template <int MX, int MY> struct GeomData : GeomDataBase<MX, MY> {
    using B = GeomDataBase<MX, MY>;

    struct RawIfaceGeom {
        static constexpr auto geom =
            B::raw_geom.getModGeom(XY0).getRecvGeom(XY0).getIfaceGeom(XY0);
    };

    template <int Idx> struct RecvGeom {
        static constexpr auto geom =
            B::asm_wg_geom.getModGeom(XY0).getRecvGeom(XY{0, Idx});
    };
};

using Eiger500kGeom = GeomData<1, 1>;
using Eiger2MGeom = GeomData<1, 4>;

using AnyDetGeom = std::variant<Eiger500kGeom, Eiger2MGeom>;

constexpr auto AnyDetGeomFromDetSize(const XY &det_size) {
    auto idx = GetDetCollectIdxFromDetSize<AnyDetGeom>(det_size);
    if (idx < std::variant_size_v<AnyDetGeom>)
        return GetValidVariant<AnyDetGeom>(idx);
    else
        throw std::runtime_error(
            "Invalid detector size: " + std::to_string(det_size.x) + "," +
            std::to_string(det_size.y));
}

using AnyRecvIdx = std::variant<std::integral_constant<int, 0>,
                                std::integral_constant<int, 1>>;

constexpr auto AnyRecvIdxFromRecvIdx(int recv_idx) {
    return GetValidVariant<AnyRecvIdx>(recv_idx);
}

} // namespace Geom
} // namespace Eiger
} // namespace sls
