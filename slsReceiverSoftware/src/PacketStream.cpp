/************************************************
 * @file PacketStream.cpp
 * @short low-level udp packet reception classes
 ***********************************************/

#include "PacketStream.h"
#include "sls/detectors/eiger/StreamData.h"
#include "sls/detectors/jungfrau/StreamData.h"

/**
 * PacketStream factory
 */

template <class PS, class... Args> auto PSFactory(Args &&...args) {
    return std::make_shared<AnyPacketStream>(std::in_place_type_t<PS>(),
                                             std::forward<Args>(args)...);
}

std::shared_ptr<AnyPacketStream>
CreatePacketStream(UdpRxSocketPtr s, GeneralDataPtr d, int idx,
                   cpu_set_t cpu_mask, pid_t thread_id, FramePolicy fp,
                   AnyPacketContainerPtr any_pc) {

    auto any_pixel = sls::AnyPixelFromBpp(d->dynamicRange);
    auto any_fp = AnyFramePolicyFromFP(fp);

    return std::visit(
        [&](auto pixel, auto fp) {
            using P = decltype(pixel);
            using FP = decltype(fp);

#define args s, cpu_mask, thread_id, any_pc

            if (d->myDetectorType == slsDetectorDefs::EIGER) {
                auto any_tg = sls::Eiger::AnyTenGigaFromTgEnable(d->tgEnable);
                return std::visit(
                    [&](auto tg) {
                        using TG = decltype(tg);
                        return PSFactory<sls::Eiger::PacketStream<P, TG, FP>>(
                            args);
                    },
                    any_tg);
            } else if (d->myDetectorType == slsDetectorDefs::JUNGFRAU) {
                if (d->numUDPInterfaces == 1)
                    return PSFactory<sls::Jungfrau::PacketStream<
                        sls::Jungfrau::Geom::OneIface, 0, FP>>(args);
                else if (idx == 0)
                    return PSFactory<sls::Jungfrau::PacketStream<
                        sls::Jungfrau::Geom::TwoIface, 0, FP>>(args);
                else
                    return PSFactory<sls::Jungfrau::PacketStream<
                        sls::Jungfrau::Geom::TwoIface, 1, FP>>(args);
            } else
                throw sls::RuntimeError("Detector not supported: " +
                                        std::to_string(d->myDetectorType));
#undef args
        },
        any_pixel, any_fp);
}
