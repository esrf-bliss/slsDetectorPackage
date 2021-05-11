#pragma once
/************************************************
 * @file Pixel.h
 * @short Pixel concept definition
 ***********************************************/

namespace sls {

/// Returns the first aligned number with res >= x
constexpr int alignCeil(int x, int align) {
    int missalign = x % align;
    return x + (!missalign ? 0 : (align - missalign));
}

/// Returns the first aligned number with res <= x
constexpr int alignFloor(int x, int align) { return x - (x % align); }

}; // namespace sls
