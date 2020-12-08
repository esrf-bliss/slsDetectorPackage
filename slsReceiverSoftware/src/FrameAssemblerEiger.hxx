/************************************************
 * @file FrameAssemblerEiger.hxx
 * @short helper classes assembling Eiger frames
 * from udp packets
 * DO NOT INCLUDE THIS FILE DIRECTLY IN YOUR CODE
 * include "FrameAssembler.h" instead
 ***********************************************/

namespace FrameAssembler {

namespace Eiger {

struct PacketData : public PacketDataBase {};

using Packet = StdPacketImpl<PacketData>;
using Assembler = DefaultFrameAssembler<Packet>;

/**
 *@short Eiger frame assembler in raw mode: 2x Default frame assemblers
 */

class RawFrameAssembler : public DualPortFrameAssembler {

  public:
    RawFrameAssembler(DefaultFrameAssemblerBase::Ptr a[2]);

    Result assembleFrame(uint64_t frame, RecvHeader *recv_header,
                         char *buf) override;
};

/**
 *@short Eiger frame assembler in std mode: port interleaving
 */

class StdFrameAssembler : public DualPortFrameAssembler {

  public:
    StdFrameAssembler(DefaultFrameAssemblerBase::Ptr a[2], bool flipped);
    ~StdFrameAssembler();

    Result assembleFrame(uint64_t frame, RecvHeader *recv_header,
                         char *buf) override;

    class Helper;

  protected:
    Helper *helper;
};

} // namespace Eiger
} // namespace FrameAssembler
