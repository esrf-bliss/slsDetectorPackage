/************************************************
 * @file PacketContainer.cpp
 * @short low-level udp packet container classes
 ***********************************************/

#include "sls/PacketContainer.h"

/**
 * MmappedPacketAllocator
 */

MmappedPacketAllocator::MmappedPacketAllocator(const NUMAMask &numa_mask)
    : block_numa_mask(numa_mask) {}

void MmappedPacketAllocator::alloc(std::size_t item_size,
                                   std::size_t nb_items) {
    release();

    auto size = item_size * nb_items;
    auto &&[node_mask, max_node] = block_numa_mask.get_os_mask();
    block_array.alloc(size, node_mask, max_node);

    block_size = item_size;
    nb_blocks = nb_items;
}

void MmappedPacketAllocator::release() {
    block_array.release();
    block_size = 0;
    nb_blocks = 0;
}

/**
 * PacketContainer factory
 */

template <class P, class... Args> auto PCFactory(Args &&...args) {
    using PC = PacketContainer<P>;
    return std::make_shared<AnyPacketContainer>(std::in_place_type_t<PC>(),
                                                std::forward<Args>(args)...);
}

AnyPacketContainerPtr
CreatePacketContainer(slsDetectorDefs::detectorType det_type, bool tg_enable,
                      int num_udp_ifaces, uint32_t dr, int frames,
                      PacketBlockAllocPtr alloc_ptr) {

    auto any_pixel = sls::AnyPixelFromBpp(dr);

    return std::visit(
        [&](auto pixel) {
            using P = decltype(pixel);

#define args frames, alloc_ptr

            if (det_type == slsDetectorDefs::EIGER) {
                auto any_tg = sls::Eiger::AnyTenGigaFromTgEnable(tg_enable);
                return std::visit(
                    [&](auto tg) {
                        using TG = decltype(tg);
                        return PCFactory<sls::Eiger::Packet<P, TG>>(args);
                    },
                    any_tg);
            } else if (det_type == slsDetectorDefs::JUNGFRAU) {
                if (num_udp_ifaces == 1)
                    return PCFactory<
                        sls::Jungfrau::Packet<sls::Jungfrau::Geom::OneIface>>(
                        args);
                else
                    return PCFactory<
                        sls::Jungfrau::Packet<sls::Jungfrau::Geom::TwoIface>>(
                        args);
            } else
                throw sls::RuntimeError("Detector not supported: " +
                                        std::to_string(det_type));
#undef args
        },
        any_pixel);
}
