#pragma once
/************************************************
 * @file StreamData.h
 * @short Eiger stream data definitions
 ***********************************************/

#include <sls/StreamData.h>

namespace sls {
namespace Eiger {

template <class Pixel, class TenGiga, class FP>
using PacketStream =
    ::PacketStream<Packet<Pixel, TenGiga>,
                   sls::StreamData<Packet<Pixel, TenGiga>>, FP>;

} // namespace Eiger
} // namespace sls
