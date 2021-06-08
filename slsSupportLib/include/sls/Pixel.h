#pragma once
/************************************************
 * @file Pixel.h
 * @short Pixel concept definition
 ***********************************************/

#include <cstdint>
// #include <functional>
// #include <iostream>
#include <type_traits>
// #include <utility>
#include <variant>

namespace sls {

///
/// \template Bpp Bits per pixel
/// \template T Underlying type of the pixel
template <int Bpp, class T> struct Pixel {
    static constexpr int nb_bits_per_byte = 8;
    using bits_per_pixel_type = std::integral_constant<int, Bpp>;
    // using iterator =
    //     std::conditional_t<Bpp == 4, Pixel4Iterator, PixelIterator<V>>;

    using value_type = T;
    // using pointer_type = typename iterator::pointer_type;

    /// Returns the pixel depth in byte per pixel (4bit being 0.5 byte)
    static constexpr float depth() { return float(Bpp) / nb_bits_per_byte; }
};

using Pixel4 = Pixel<4, std::uint8_t>;
using Pixel8 = Pixel<8, std::uint8_t>;
using Pixel16 = Pixel<16, std::uint16_t>;
using Pixel32 = Pixel<32, std::uint32_t>;

using AnyPixel = std::variant<Pixel4, Pixel8, Pixel16, Pixel32>;

/// Constructs a pixel from Bits er Pixel
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

}; // namespace sls
