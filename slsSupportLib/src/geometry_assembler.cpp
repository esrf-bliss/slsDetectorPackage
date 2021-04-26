//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2011
// European Synchrotron Radiation Facility
// BP 220, Grenoble 38043
// FRANCE
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//###########################################################################

#include "sls/Geometry.h"
#include "sls/logger.h"
#include "sls/sls_detector_exceptions.h"

#include <cstring>
#include <fstream>
#include <memory>
#include <type_traits>
#include <vector>

using namespace std::literals::string_literals;

using XY = sls::Geom::XY;
using MapView = sls::Geom::MapView;

/*
 * Helper functions
 */

template <class TDG, class T>
void fill_gap_pixel_value(const TDG &tgt_det, T *tgt, T gap_pixel_val) {
    int det_pixels = tgt_det.size.area();
    for (int i = 0; i < det_pixels; ++i)
        tgt[i] = gap_pixel_val;
}

/*
 * Generator types
 */

template <class SDG, class T> struct GenChipIdx {
    static const std::string name;
    GenChipIdx(const SDG &src_geom)
        : det_chips(src_geom.det_mods * src_geom.mod_recvs *
                    src_geom.recv_ifaces * src_geom.iface_chips) {}
    T get_val(T * /*src*/, const XY &chip, const MapView & /*src_chip*/,
              const XY & /*pixel*/) {
        return RowWiseElementIndex(det_chips, chip);
    }
    XY det_chips;
};
template <class SDG, class T>
const std::string GenChipIdx<SDG, T>::name = "chip_idx";

template <class SDG, class T> struct GenPixelIdx {
    static const std::string name;
    GenPixelIdx(const SDG & /*src_geom*/) {}
    T get_val(T * /*src*/, const XY & /*chip*/, const MapView &src_chip,
              const XY &pixel) {
        return src_chip.calcMapPixelIndex(pixel);
    }
};
template <class SDG, class T>
const std::string GenPixelIdx<SDG, T>::name = "pixel_idx";

template <class SDG, class T> struct GenPixelVal {
    static const std::string name;
    GenPixelVal(const SDG & /*src_geom*/) {}
    T get_val(T *src, const XY & /*chip*/, const MapView &src_chip,
              const XY &pixel) {
        int si = src_chip.calcMapPixelIndex(pixel);
        return src[si];
    }
};
template <class SDG, class T>
const std::string GenPixelVal<SDG, T>::name = "pixel_val";

template <class SDG, class T>
using AnyGenType =
    std::variant<GenChipIdx<SDG, T>, GenPixelIdx<SDG, T>, GenPixelVal<SDG, T>>;

template <class SDG, class T>
AnyGenType<SDG, T> AnyGenTypeFromStr(const std::string &gen_type,
                                     const SDG &src_geom) {

#define test(g)                                                                \
    if (gen_type == g<SDG, T>::name)                                           \
    return g<SDG, T>(src_geom)

    test(GenChipIdx);
    test(GenPixelIdx);
    test(GenPixelVal);

#undef test

    throw sls::RuntimeError("Invalid gen_type: " + gen_type);
}

/*
 * Generate map
 */

template <class Gen, class SDG, class TDG, class T>
void generate_map(Gen &gen, const SDG &src_det, T *src, const TDG &tgt_det,
                  T *tgt, T gap_pixel_val = -1) {
    fill_gap_pixel_value(tgt_det, tgt, gap_pixel_val);

    det2_for_each_chip(
        src_det, tgt_det, [&](auto &chip, auto &src_chip, auto &tgt_chip) {
            using SV = decltype(src_chip);
            using TV = decltype(tgt_chip);
            view2_for_each_pixel(
                src_chip, tgt_chip,
                [&](const SV &src_chip, const TV &tgt_chip, const XY &pixel) {
                    int ti = tgt_chip.calcMapPixelIndex(pixel);
                    tgt[ti] = gen.get_val(src, chip, src_chip, pixel);
                });
        });
}

/*
 * Data format
 *
 * Packet - UDP packet block with 128-bit-aligned data
 * Raw - Data as it arrives from UDP packets
 * AsmWithNoGap - Assembled with no gaps
 * AsmWithGap - Assembled with gaps
 */

struct RawFmt : sls::Geom::RawFmt {
    static const std::string name;
    template <class GD> static constexpr auto getGeom(const GD &geom_data) {
        return geom_data.raw_geom;
    };
};
const std::string RawFmt::name = "Raw";

struct AsmWithNoGapFmt : sls::Geom::AsmWithNoGapFmt {
    static const std::string name;
    template <class GD> static constexpr auto getGeom(const GD &geom_data) {
        return geom_data.asm_ng_geom;
    };
};
const std::string AsmWithNoGapFmt::name = "AsmNG";

struct AsmWithGapFmt : sls::Geom::AsmWithGapFmt {
    static const std::string name;
    template <class GD> static constexpr auto getGeom(const GD &geom_data) {
        return geom_data.asm_wg_geom;
    };
};
const std::string AsmWithGapFmt::name = "AsmWG";

using AnyDataFmt = std::variant<RawFmt, AsmWithNoGapFmt, AsmWithGapFmt>;

AnyDataFmt AnyDataFmtFromStr(const std::string &data_fmt) {

#define test(f)                                                                \
    if (data_fmt == f::name)                                                   \
    return f()

    test(RawFmt);
    test(AsmWithNoGapFmt);
    test(AsmWithGapFmt);

#undef test

    throw sls::RuntimeError("Invalid data_format: " + data_fmt);
}

/*
 * Data type
 */

template <class T> struct TypeData {
    using type = T;
    static const std::string name;
};

template <> const std::string TypeData<std::uint8_t>::name = "uint8"s;
template <> const std::string TypeData<std::int8_t>::name = "int8"s;
template <> const std::string TypeData<std::uint16_t>::name = "uint16"s;
template <> const std::string TypeData<std::int16_t>::name = "int16"s;
template <> const std::string TypeData<std::uint32_t>::name = "uint32"s;
template <> const std::string TypeData<std::int32_t>::name = "int32"s;
template <> const std::string TypeData<float>::name = "float32"s;
template <> const std::string TypeData<double>::name = "float64"s;

using AnyTypeData =
    std::variant<TypeData<std::uint8_t>, TypeData<std::int8_t>,
                 TypeData<std::uint16_t>, TypeData<std::int16_t>,
                 TypeData<std::uint32_t>, TypeData<std::int32_t>,
                 TypeData<float>, TypeData<double>>;

AnyTypeData AnyTypeDataFromStr(const std::string &data_type) {

#define test(t)                                                                \
    if (data_type == TypeData<t>::name)                                        \
    return TypeData<t>()

    test(std::uint8_t);
    test(std::int8_t);
    test(std::uint16_t);
    test(std::int16_t);
    test(std::uint32_t);
    test(std::int32_t);
    test(float);
    test(double);

#undef test

    throw sls::RuntimeError("Invalid data_type: " + data_type);
}

/*
 * Detector type
 */

struct EigerData {
    static const std::string name;
    using AnyGeom = sls::Geom::Eiger::AnyDetGeom;
    static constexpr auto from_size(const XY &xy) {
        return sls::Geom::Eiger::AnyDetGeomFromDetSize(xy);
    }
    AnyGeom any_geom;
};
const std::string EigerData::name = "eiger"s;

struct Jungfraux1Data {
    static const std::string name;
    using AnyGeom = sls::Geom::Jungfrau::AnyDetGeom<1>;
    static constexpr auto from_size(const XY &xy) {
        return sls::Geom::Jungfrau::AnyDetGeomFromDetSize<1>(xy);
    }
    AnyGeom any_geom;
};
const std::string Jungfraux1Data::name = "jungfraux1"s;

struct Jungfraux2Data {
    static const std::string name;
    using AnyGeom = sls::Geom::Jungfrau::AnyDetGeom<2>;
    static constexpr auto from_size(const XY &xy) {
        return sls::Geom::Jungfrau::AnyDetGeomFromDetSize<2>(xy);
    }
    AnyGeom any_geom;
};
const std::string Jungfraux2Data::name = "jungfraux2"s;

using AnyDet = std::variant<EigerData, Jungfraux1Data, Jungfraux2Data>;

AnyDet AnyDetFromStrAndSize(const std::string &det_type, const XY &det_size) {
#define test(d)                                                                \
    if (det_type == d::name)                                                   \
    return d{d::from_size(det_size)}

    test(EigerData);
    test(Jungfraux1Data);
    test(Jungfraux2Data);

#undef test
    throw sls::RuntimeError("Invalid det_type: " + det_type);
}

/*
 * High level geometry_assembler
 */

template <class DT, class SDG, class TDG, class TD>
void geometry_assembler(DT det_type, std::string gen_type, const SDG &src_geom,
                        const TDG &tgt_geom, TD type_data,
                        std::size_t nb_frames, std::ifstream *ifile,
                        std::ofstream *ofile, int gap_pixel_val) {
    using T = typename TD::type;
    LOG(logINFO) << "det_type.name=" << det_type.name << ", "
                 << "src_geom.size=" << src_geom.size << ", "
                 << "tgt_geom.size=" << tgt_geom.size << ", "
                 << "sizeof(T)=" << sizeof(T) << ", "
                 << "type_data.name=" << type_data.name << ", "
                 << "nb_frames=" << nb_frames;

    std::size_t src_pixels = src_geom.size.area();
    std::size_t tgt_pixels = tgt_geom.size.area();
    std::unique_ptr<T> src(ifile ? new T[src_pixels] : nullptr);
    std::unique_ptr<T> tgt(ofile ? new T[tgt_pixels] : nullptr);

    auto any_gen_type = AnyGenTypeFromStr<SDG, T>(gen_type, src_geom);
    std::visit(
        [&](auto &gen) {
            for (std::size_t f = 0; f < nb_frames; ++f) {
                if (ifile) {
                    char *sp = reinterpret_cast<char *>(src.get());
                    size_t isize = src_pixels * sizeof(T);
                    LOG(logINFO) << "Reading " << isize << " bytes";
                    ifile->read(sp, isize);
                }
                generate_map(gen, src_geom, src.get(), tgt_geom, tgt.get(),
                             T(gap_pixel_val));
                if (ofile) {
                    char *tp = reinterpret_cast<char *>(tgt.get());
                    size_t osize = tgt_pixels * sizeof(T);
                    LOG(logINFO) << "Writing " << osize << " bytes";
                    ofile->write(tp, osize);
                }
            }
        },
        any_gen_type);
}

template <class SDG, class TDG>
void print_geom(const SDG &src_geom, const TDG &tgt_geom) {
    std::cout << src_geom.size << " " << tgt_geom.size << std::endl;
    det_for_each_mod(tgt_geom, [&](auto &mod, auto &mod_geom) {
        auto &v = mod_geom.view;
        std::cout << " " << v.view_origin << "x" << v.size;
    });
}

void usage(std::string prog_name) {
    LOG(logERROR) << "Usage: " << prog_name << " "
                  << "<det_type> <width,height> <nb_frames> <gen_type> "
                  << "<src_format> <src_data_type> <src_file_name> "
                  << "<target_format> <target_data_type> <target_file_name> "
                  << "<gap_pixel_val>";
    exit(1);
}

void main_funct(int argc, char *argv[]) {
    if (argc != 12)
        usage(argv[0]);

    int argi = 0;
    std::string det_type = argv[++argi];
    std::string size_str = argv[++argi];
    std::string nb_frames_str = argv[++argi];
    std::string gen_type = argv[++argi];
    std::string src_data_fmt = argv[++argi];
    std::string src_data_type = argv[++argi];
    std::string src_fname = argv[++argi];
    std::string tgt_data_fmt = argv[++argi];
    std::string tgt_data_type = argv[++argi];
    std::string tgt_fname = argv[++argi];
    std::string gap_pixel_val_str = argv[++argi];

    LOG(logINFO) << "det_type=" << det_type << ", "
                 << "size_str=" << size_str << ", "
                 << "nb_frames_str=" << nb_frames_str << ", "
                 << "gen_type=" << gen_type << ", "
                 << "src_data_fmt=" << src_data_fmt << ", "
                 << "src_data_type=" << src_data_type << ", "
                 << "src_fname=" << src_fname << ", "
                 << "tgt_data_fmt=" << tgt_data_fmt << ", "
                 << "tgt_data_type=" << tgt_data_type << ", "
                 << "tgt_fname=" << tgt_fname << ", "
                 << "gap_pixel_val_str=" << gap_pixel_val_str;

    if (tgt_data_type != src_data_type)
        throw sls::RuntimeError("Src & Tgt data type must match");

    XY size;
    {
        std::vector<int> sl(2);
        std::istringstream is(size_str);
        bool check_sep = false;
        for (auto &s : sl) {
            if (check_sep) {
                char sep;
                is >> sep;
                if (sep != ',')
                    throw sls::RuntimeError("Invalid size_str: " + size_str);
            }
            is >> s;
            check_sep = true;
        }
        size = XY{sl[0], sl[1]};
    }

    std::size_t nb_frames = std::stoul(nb_frames_str);

    AnyDet any_det = AnyDetFromStrAndSize(det_type, size);
    AnyDataFmt src_any_data_fmt = AnyDataFmtFromStr(src_data_fmt);
    AnyDataFmt tgt_any_data_fmt = AnyDataFmtFromStr(tgt_data_fmt);
    AnyTypeData any_type_data = AnyTypeDataFromStr(tgt_data_type);

    bool do_print_geom = (src_fname.empty() && tgt_fname.empty());

    int gap_pixel_val = std::stoi(gap_pixel_val_str);

    std::visit(
        [&](auto det_type, auto src_data_fmt, auto tgt_data_fmt,
            auto type_data) {
            std::visit(
                [&](auto geom_data) {
                    constexpr auto src_geom = src_data_fmt.getGeom(geom_data);
                    constexpr auto tgt_geom = tgt_data_fmt.getGeom(geom_data);

                    if (do_print_geom) {
                        print_geom(src_geom, tgt_geom);
                        return;
                    }

                    auto in_mode = std::ios_base::in | std::ios_base::binary;
                    std::unique_ptr<std::ifstream> ifile;
                    if (!src_fname.empty())
                        ifile =
                            std::make_unique<std::ifstream>(src_fname, in_mode);
                    auto out_mode = std::ios_base::out | std::ios_base::binary;
                    std::unique_ptr<std::ofstream> ofile;
                    if (!tgt_fname.empty())
                        ofile = std::make_unique<std::ofstream>(tgt_fname,
                                                                out_mode);

                    geometry_assembler(det_type, gen_type, src_geom, tgt_geom,
                                       type_data, nb_frames, ifile.get(),
                                       ofile.get(), gap_pixel_val);
                },
                det_type.any_geom);
        },
        any_det, src_any_data_fmt, tgt_any_data_fmt, any_type_data);
}

int main(int argc, char *argv[]) {
    try {
        main_funct(argc, argv);
    } catch (std::exception &e) {
        LOG(logERROR) << "Exeption: " << e.what();
    } catch (...) {
        LOG(logERROR) << "Unknown exception";
    }
    return 0;
};
