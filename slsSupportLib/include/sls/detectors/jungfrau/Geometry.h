#pragma once
/************************************************
 * @file Geometry.h
 * @short Eiger geometry definitions
 ***********************************************/

#include "sls/Geometry.h"

namespace sls {
namespace Jungfrau {
namespace Geom {

using namespace sls::Geom;

// Jungfrau definitions and helpers
constexpr XY ChipPixels{256, 256};
constexpr XY ChipGap{2, 2};
template <int NbUDPIfaces> constexpr XY IfaceChips{4, 2 / NbUDPIfaces};
template <int NbUDPIfaces> constexpr XY RecvIfaces{1, NbUDPIfaces};
using ModRecvFlip = DefaultModRecvFlip;
constexpr XY ModRecvs{1, 1};
constexpr XY ModGap{9, 36};

template <int NbUDPIfaces> struct TiledDetGeom {
    template <int MX, int MY, class Fmt> struct Generator {
        constexpr auto operator()() {
            return Geom::DetGeom<Fmt, ModRecvFlip>(
                ChipPixels, ChipGap, IfaceChips<NbUDPIfaces>,
                RecvIfaces<NbUDPIfaces>, ModRecvs, ModGap, XY{MX, MY});
        }
    };
};

template <int NbUDPIfaces, int MX, int MY>
using GeomDataBase =
    DetGeomData<MX, MY, TiledDetGeom<NbUDPIfaces>::template Generator>;

template <int NbUDPIfaces, int MX, int MY>
struct GeomData : GeomDataBase<NbUDPIfaces, MX, MY> {
    using B = GeomDataBase<NbUDPIfaces, MX, MY>;

    static constexpr int num_udp_ifaces = NbUDPIfaces;

    template <int Idx> struct RawIfaceGeom {
        static constexpr auto geom =
            B::raw_geom.getModGeom(XY0).getRecvGeom(XY0).getIfaceGeom(
                XY{0, Idx});
    };

    struct RecvGeom {
        static constexpr auto geom =
            B::asm_wg_geom.getModGeom(XY0).getRecvGeom(XY0);
    };

    template <int Idx> struct IfaceGeom {
        static constexpr auto geom = RecvGeom::geom.getIfaceGeom(XY{0, Idx});
    };
};

template <int NbUDPIfaces> using Jungfrau500kGeom = GeomData<NbUDPIfaces, 1, 1>;
template <int NbUDPIfaces> using Jungfrau1MGeom = GeomData<NbUDPIfaces, 1, 2>;
template <int NbUDPIfaces> using Jungfrau1MWGeom = GeomData<NbUDPIfaces, 2, 1>;
template <int NbUDPIfaces> using Jungfrau4MGeom = GeomData<NbUDPIfaces, 2, 4>;
template <int NbUDPIfaces> using Jungfrau16MGeom = GeomData<NbUDPIfaces, 4, 8>;

template <int NbUDPIfaces>
using AnyDetGeom =
    std::variant<Jungfrau500kGeom<NbUDPIfaces>, Jungfrau1MGeom<NbUDPIfaces>,
                 Jungfrau1MWGeom<NbUDPIfaces>, Jungfrau4MGeom<NbUDPIfaces>,
                 Jungfrau16MGeom<NbUDPIfaces>>;

template <int NbUDPIfaces>
constexpr auto AnyDetGeomFromDetSize(const XY &det_size) {
    using AnyDet = AnyDetGeom<NbUDPIfaces>;
    auto idx = GetDetCollectIdxFromDetSize<AnyDet>(det_size);
    if (idx < std::variant_size_v<AnyDet>)
        return GetValidVariant<AnyDet>(idx);
    else
        throw std::runtime_error(
            "Invalid detector size: " + std::to_string(det_size.x) + "," +
            std::to_string(det_size.y));
}

using AnyNbUDPIfaces = std::variant<std::integral_constant<int, 1>,
                                    std::integral_constant<int, 2>>;

constexpr auto AnyNbUDPIfacesFromNbUDPIfaces(int num_udp_ifaces) {
    return GetValidVariant<AnyNbUDPIfaces>(num_udp_ifaces - 1);
}

} // namespace Geom
} // namespace Jungfrau
} // namespace sls
