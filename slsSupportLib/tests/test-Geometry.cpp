#include "catch.hpp"
#include "sls/Geometry.h"

namespace sls {
namespace Geom {

namespace Eiger {

template <class DetGeom> struct GeomData {
    int chip_size, iface_chips, recv_ifaces, mod_recvs, chip_gap, mod_gap,
        recv_chips;
    XY det_mods, raw_mod_size, raw_size, std_mod_size, std_size;

    constexpr GeomData(const DetGeom &det_geom)
        : chip_size(ChipPixels.x), iface_chips(IfaceChips.x),
          recv_ifaces(RecvIfaces.x), mod_recvs(ModRecvs.y), chip_gap(ChipGap.x),
          mod_gap(ModGap.x), recv_chips(iface_chips * recv_ifaces),
          det_mods(det_geom.det_mods),
          // Raw mode: vertical concatenation of 4x dual-chip ports (512x256)
          raw_mod_size(
              {chip_size * iface_chips, chip_size * recv_ifaces * mod_recvs}),
          raw_size(raw_mod_size * det_mods),
          std_mod_size({chipsWithoutLastGap(recv_chips),
                        chipsWithoutLastGap(mod_recvs)}),
          std_size({std_mod_size.x, modsWithoutLastGap(det_mods.y)}) {}

    constexpr int rawPixelIdx(int mod, int recv, int iface, int chip, int x,
                              int y) const {
        int recv_idx = mod_recvs * mod + recv;
        int chip_idx = recv_ifaces * recv_idx + iface;
        int row = chip_size * chip_idx + y;
        int col = chip_size * chip + x;
        return RowWiseElementIndex(raw_size, XY{col, row});
    }

    // Std mode: image reconstruction with gap pixel insertion: (2x2) x (3x1)
    constexpr int chipsWithLastGap(int chips) const {
        return (chip_size + chip_gap) * chips;
    }
    constexpr int chipsWithoutLastGap(int chips) const {
        return chipsWithLastGap(chips) - chip_gap;
    }

    // Std mode: image reconstruction with module gap insertion: (36x36) x (0x3)
    constexpr int modsWithLastGap(int mods) const {
        return (std_mod_size.y + mod_gap) * mods;
    }
    constexpr int modsWithoutLastGap(int mods) const {
        return modsWithLastGap(mods) - mod_gap;
    }

    constexpr int stdPixelIdx(int mod, int recv, int iface, int chip, int x,
                              int y) const {
        int row_base = modsWithLastGap(mod) + chipsWithLastGap(recv);
        // std bottom receiver: vertical flip
        bool flipped = !recv;
        int row_dir = !flipped ? 1 : -1;
        int row_offset = row_base + (!flipped ? 0 : (chip_size - 1));
        int row = row_offset + row_dir * y;
        int recv_chip = iface * iface_chips + chip;
        int col = chipsWithLastGap(recv_chip) + x;
        return RowWiseElementIndex(std_size, XY{col, row});
    }
};

#define CA constexpr auto

template <class Fmt> struct TestEiger500kDetGeom {
    static CA geom = Eiger500kDetGeom<Fmt>;
};

template <class Fmt> struct TestEiger2MDetGeom {
    static CA geom = Eiger2MDetGeom<Fmt>;
};

template <template <class Fmt> class TestGeom> struct TestSuite {

    template <class Fmt> static CA det_geom = TestGeom<Fmt>::geom;

    template <int mod_idx> static void run() {
        std::cout << "In sls::Geom::Eiger::TestSuite::run<" << mod_idx
                  << ">:" << std::endl;
        run_raw<mod_idx>();
        run_std<mod_idx>();
    }

#define assert_size(a, b)                                                      \
    static_assert(a.size == b);                                                \
    CHECK(a.size == b);                                                        \
    std::cout << #a ".size == " #b " (" << b << ")" << std::endl

#define assert_iface_idx(in, i)                                                \
    static_assert(in.iface_idx == i);                                          \
    CHECK(in.iface_idx == i);                                                  \
    std::cout << #in ".iface_idx == " #i " (" << i << ")" << std::endl

#define assert_equal(a, b)                                                     \
    static_assert(a == b);                                                     \
    CHECK(int(a) == int(b));                                                   \
    std::cout << #a " == " #b " (" << int(a) << ")" << std::endl

#define assert_chip_pixel(cn, x, y, v)                                         \
    assert_equal(cn.calcMapPixelIndex(XY{x, y}), v)

    template <int mod_idx> static void run_raw() {
        CA raw_det = det_geom<RawFmt>;
        constexpr GeomData d(raw_det);
        assert_size(raw_det, d.raw_size);

#define assert_raw_pixel(cn, r, i, c, x, y)                                    \
    assert_chip_pixel(cn, x, y, (d.rawPixelIdx(mod_idx, r, i, c, x, y)))

        CA raw_mod = raw_det.getModGeom(XY{0, mod_idx});
        assert_size(raw_mod, d.raw_mod_size);
        CA raw_top_recv = raw_mod.getRecvGeom(XY{0, 0});
        CA raw_top_recv_size = XY{d.raw_size.x, d.chip_size * d.recv_ifaces};
        assert_size(raw_top_recv, raw_top_recv_size);
        CA raw_top_left_iface = raw_top_recv.getIfaceGeom(XY{0, 0});
        CA raw_top_left_iface_size = XY{d.raw_size.x, d.chip_size};
        assert_size(raw_top_left_iface, raw_top_left_iface_size);
        CA raw_top_left_iface_idx = XY{0, mod_idx * d.mod_recvs};
        assert_iface_idx(raw_top_left_iface, raw_top_left_iface_idx);
        CA raw_top_left_chip_r = raw_top_left_iface.getChipView(XY{1, 0});
        CA raw_top_left_chip_r_size = XY{d.chip_size, d.chip_size};
        assert_size(raw_top_left_chip_r, raw_top_left_chip_r_size);
        assert_raw_pixel(raw_top_left_chip_r, 0, 0, 1, 1, 0);
        assert_raw_pixel(raw_top_left_chip_r, 0, 0, 1, 1, 31);
        CA raw_top_right_iface = raw_top_recv.getIfaceGeom(XY{1, 0});
        CA raw_top_right_iface_idx = XY{1, mod_idx * d.mod_recvs};
        assert_iface_idx(raw_top_right_iface, raw_top_right_iface_idx);
        CA raw_top_right_chip_r = raw_top_right_iface.getChipView(XY{1, 0});
        assert_raw_pixel(raw_top_right_chip_r, 0, 1, 1, 1, 0);
        assert_raw_pixel(raw_top_right_chip_r, 0, 1, 1, 1, 31);

        CA raw_bot_recv = raw_mod.getRecvGeom(XY{0, 1});
        CA raw_bot_right_iface = raw_bot_recv.getIfaceGeom(XY{1, 0});
        CA raw_bot_right_iface_idx = XY{1, mod_idx * d.mod_recvs + 1};
        assert_iface_idx(raw_bot_right_iface, raw_bot_right_iface_idx);
        CA raw_bot_right_chip_r = raw_bot_right_iface.getChipView(XY{1, 0});
        assert_raw_pixel(raw_bot_right_chip_r, 1, 1, 1, 1, 0);
        assert_raw_pixel(raw_bot_right_chip_r, 1, 1, 1, 1, 31);

#undef assert_raw_pixel
    }

    template <int mod_idx> static void run_std() {
        CA std_det = det_geom<StdFmt>;
        constexpr GeomData d(std_det);
        assert_size(std_det, d.std_size);

#define assert_std_pixel(cn, r, i, c, x, y)                                    \
    assert_chip_pixel(cn, x, y, (d.stdPixelIdx(mod_idx, r, i, c, x, y)));

        CA std_mod = std_det.getModGeom(XY{0, mod_idx});
        assert_size(std_mod, d.std_mod_size);
        CA std_top_recv = std_mod.getRecvGeom(XY{0, 0});
        CA std_top_recv_size = XY{d.std_size.x, d.chip_size};
        assert_size(std_top_recv, std_top_recv_size);
        CA std_top_left_iface = std_top_recv.getIfaceGeom(XY{0, 0});
        CA std_top_left_chip_r = std_top_left_iface.getChipView(XY{1, 0});
        assert_std_pixel(std_top_left_chip_r, 0, 0, 1, 1, 0);
        assert_std_pixel(std_top_left_chip_r, 0, 0, 1, 1, 31);

        CA std_top_right_iface = std_top_recv.getIfaceGeom(XY{1, 0});
        CA std_top_right_chip_r = std_top_right_iface.getChipView(XY{1, 0});
        assert_std_pixel(std_top_right_chip_r, 0, 1, 1, 1, 0);
        assert_std_pixel(std_top_right_chip_r, 0, 1, 1, 1, 31);

        CA std_bot_recv = std_mod.getRecvGeom(XY{0, 1});
        CA std_bot_right_iface = std_bot_recv.getIfaceGeom(XY{1, 0});
        CA std_bot_right_chip_r = std_bot_right_iface.getChipView(XY{1, 0});
        assert_std_pixel(std_bot_right_chip_r, 1, 1, 1, 1, 0);
        assert_std_pixel(std_bot_right_chip_r, 1, 1, 1, 1, 31);

#undef assert_std_pixel
    }

#undef assert_chip_pixel
#undef assert_equal
#undef assert_iface_idx
#undef assert_size
};

inline void test() {
    TestSuite<TestEiger500kDetGeom>::run<0>();
    TestSuite<TestEiger2MDetGeom>::run<3>();
}

} // namespace Eiger

inline void test() {
    std::cout << "In sls::Geom::test():" << std::endl;
    constexpr XY c1{128, 256};
    std::cout << "c1=" << c1 << ", "
              << "c1.area()=" << c1.area() << std::endl;
    static_assert((c1 * XY{2, 4}) == XY{256, 1024});
    static_assert(c1.area() == (128 * 256));

    Eiger::test();
}

} // namespace Geom
} // namespace sls

TEST_CASE("The Geometry code behaves as expected", "[support]") {
    sls::Geom::test();
}
