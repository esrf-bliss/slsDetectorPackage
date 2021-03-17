#pragma once
/************************************************
 * @file FrameAssembler.h
 * @short helper classes assembling frames
 * from udp packets
 ***********************************************/

#include "GeneralData.h"
#include "Packet.h"

namespace FrameAssembler {

/**
 *@short Default frame assembler in Listener
 */

class DefaultFrameAssemblerBase {
  public:
    virtual ~DefaultFrameAssemblerBase() {}

    virtual bool assembleFrame(AnyPacketBlockPtr &&block, RecvHeader *header,
                               char *buf) = 0;

    virtual int getImageSize() = 0;
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

    bool assembleFrame(AnyPacketBlockPtr &&block, RecvHeader *header,
                       char *buf) override;

    int getImageSize() override;

  protected:
    void expand4Bits(char *dst, char *src, int src_size);
};

DefaultFrameAssemblerPtr CreateDefaultFrameAssembler(GeneralDataPtr gd,
                                                     bool e4b);

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

    virtual Result assembleFrame(AnyPacketBlockList &&blocks,
                                 RecvHeader *header, char *buf) = 0;
};

using MPFrameAssemblerPtr = std::shared_ptr<MPFrameAssembler>;

/**
 *@short Raw frame assembler: vertical concatenation of default assemblers
 */

class RawFrameAssembler : public MPFrameAssembler {

  public:
    RawFrameAssembler(GeneralDataPtr gd, int recv_idx, bool e4b) {
        for (std::size_t i = 0; i < gd->numUDPInterfaces; ++i)
            assembler.emplace_back(CreateDefaultFrameAssembler(gd, e4b));
        int iface_size = assembler[0]->getImageSize();
        data_offset = assembler.size() * iface_size * recv_idx;
    }

    Result assembleFrame(AnyPacketBlockList &&blocks, RecvHeader *recv_header,
                         char *buf) override;

  private:
    using DefaultFrameAssemblerList = std::vector<DefaultFrameAssemblerPtr>;

    DefaultFrameAssemblerList assembler;
    int data_offset;
};

} // namespace FrameAssembler

#include "FrameAssemblerEiger.hxx"
#include "FrameAssemblerJungfrau.hxx"
