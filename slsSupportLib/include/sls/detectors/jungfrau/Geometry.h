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

// Jungfrau types defining number of UDP interfaces
struct OneIface {
    static constexpr int NbIfaces = 1;
};
struct TwoIface {
    static constexpr int NbIfaces = 2;
};

// Jungfrau definitions and helpers
constexpr XY ChipPixels{256, 256};
constexpr XY ChipGap{2, 2};
template <typename NbUDPIfaces>
constexpr XY IfaceChips{4, 2 / NbUDPIfaces::NbIfaces};
template <typename NbUDPIfaces>
constexpr XY RecvIfaces{1, NbUDPIfaces::NbIfaces};
struct ModRecvFlip : DefaultModRecvFlip {
    static constexpr auto getRecvFlip(const XY & /*recv_idx*/) {
        return VertFlip;
    }
};

constexpr XY ModRecvs{1, 1};
constexpr XY ModGap{8, 36};

template <typename NbUDPIfaces> struct TiledDetGeom {
    template <int MX, int MY, class Fmt> struct Generator {
        constexpr auto operator()() {
            return Geom::DetGeom<Fmt, ModRecvFlip>(
                ChipPixels, ChipGap, IfaceChips<NbUDPIfaces>,
                RecvIfaces<NbUDPIfaces>, ModRecvs, ModGap, XY{MX, MY});
        }
    };
};

template <typename NbUDPIfaces, int MX, int MY>
using GeomDataBase =
    DetGeomData<MX, MY, TiledDetGeom<NbUDPIfaces>::template Generator>;

template <typename NbUDPIfaces, int MX, int MY>
struct GeomData : GeomDataBase<NbUDPIfaces, MX, MY> {
    using B = GeomDataBase<NbUDPIfaces, MX, MY>;

    using num_udp_ifaces = NbUDPIfaces;

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

template <typename NbUDPIfaces>
using Jungfrau500kGeom = GeomData<NbUDPIfaces, 1, 1>;
template <typename NbUDPIfaces>
using Jungfrau1MGeom = GeomData<NbUDPIfaces, 1, 2>;
template <typename NbUDPIfaces>
using Jungfrau1MWGeom = GeomData<NbUDPIfaces, 2, 1>;
template <typename NbUDPIfaces>
using Jungfrau4MGeom = GeomData<NbUDPIfaces, 2, 4>;
template <typename NbUDPIfaces>
using Jungfrau16MGeom = GeomData<NbUDPIfaces, 4, 8>;

template <typename NbUDPIfaces>
using AnyDetGeom =
    std::variant<Jungfrau500kGeom<NbUDPIfaces>, Jungfrau1MGeom<NbUDPIfaces>,
                 Jungfrau1MWGeom<NbUDPIfaces>, Jungfrau4MGeom<NbUDPIfaces>,
                 Jungfrau16MGeom<NbUDPIfaces>>;

template <typename NbUDPIfaces>
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

using AnyNbUDPIfaces = std::variant<OneIface, TwoIface>;

constexpr AnyNbUDPIfaces AnyNbUDPIfacesFromNbUDPIfaces(int num_udp_ifaces) {
    if (num_udp_ifaces == 1)
        return OneIface();
    else
        return TwoIface();
}

} // namespace Geom
} // namespace Jungfrau
} // namespace sls
