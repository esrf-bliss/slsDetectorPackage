#pragma once
/************************************************
 * @file FrameAssembler.h
 * @short helper classes assembling frames
 * from udp packets
 ***********************************************/

#include "PacketTypedefs.h"

namespace sls {
namespace FrameAssembler {

using namespace sls::Geom;

/**
 *@short Dimensions of (assembled) frames
 */

struct FrameDims {
    slsDetectorDefs::xy dim;
    int size;
};

/**
 *@short Default frame assembler in DataProcessor
 */

class DefaultFrameAssemblerBase {
  public:
    virtual ~DefaultFrameAssemblerBase() {}

    virtual bool assembleFrame(AnyPacketBlockPtr block, char *buf) = 0;
    virtual FrameDims getAssembledFrameDims() = 0;
};
using DefaultFrameAssemblerPtr = std::shared_ptr<DefaultFrameAssemblerBase>;

/*
 * DefaultFrameAssembler:
 *   Packet: Packet type, DP: Dst Pixel
 */

template <class Packet, class DP = typename Packet::Data::Pixel>
class DefaultFrameAssembler : public DefaultFrameAssemblerBase {

  public:
    using PacketData = typename Packet::Data;
    using SP = typename PacketData::Pixel;

    static constexpr bool Expand4Bits =
        (std::is_same_v<SP, Pixel4> && std::is_same_v<DP, Pixel8>);

    using Block = PacketBlock<Packet>;
    using BlockPtr = PacketBlockPtr<Packet>;

    DefaultFrameAssembler(slsDetectorDefs::xy iface_dims)
        : iface_size(iface_dims) {}

    bool assembleFrame(AnyPacketBlockPtr block, char *buf) override;

    FrameDims getAssembledFrameDims() override;

  protected:
    void expand4Bits(char *dst, char *src, int src_size);

    slsDetectorDefs::xy iface_size;
};

DefaultFrameAssemblerPtr
CreateDefaultFrameAssembler(slsDetectorDefs::detectorType det_type,
                            bool tg_enable, int num_udp_ifaces, uint32_t src_dr,
                            uint32_t dst_dr = 0);

/**
 *@short Multi-port frame assembler result
 */

constexpr int MaxNbPorts = 2;

using PortsMask = std::bitset<MaxNbPorts>;

struct Result {
    int nb_ports;
    PortsMask valid_data;
};

class MPFrameAssembler {

  public:
    virtual ~MPFrameAssembler() {}

    virtual Result assembleFrame(AnyPacketBlockList blocks, char *buf) = 0;
    virtual FrameDims getAssembledFrameDims() = 0;
};

using MPFrameAssemblerPtr = std::unique_ptr<MPFrameAssembler>;

/**
 *@short Raw frame assembler: vertical concatenation of default assemblers
 */

class RawFrameAssembler : public MPFrameAssembler {

  public:
    RawFrameAssembler(slsDetectorDefs::detectorType det_type, int recv_idx,
                      int det_recvs, int num_udp_ifaces, uint32_t src_dr,
                      uint32_t dst_dr = 0)
        : nb_recvs(det_recvs) {
        for (int i = 0; i < num_udp_ifaces; ++i)
            assembler.emplace_back(CreateDefaultFrameAssembler(
                det_type, num_udp_ifaces, src_dr, dst_dr));
        int iface_size = assembler[0]->getAssembledFrameDims().size;
        data_offset = assembler.size() * iface_size * recv_idx;
    }

    Result assembleFrame(AnyPacketBlockList blocks, char *buf) override;
    FrameDims getAssembledFrameDims() override;

  private:
    using DefaultFrameAssemblerList = std::vector<DefaultFrameAssemblerPtr>;

    DefaultFrameAssemblerList assembler;
    int nb_recvs;
    int data_offset;
};

enum AssemblerType {
    AsmRaw,
    AsmWithGap,
};

} // namespace FrameAssembler
} // namespace sls
