/************************************************
 * @file FrameAssemblerEiger.hxx
 * @short helper classes assembling Eiger frames
 * from udp packets
 * DO NOT INCLUDE THIS FILE DIRECTLY IN YOUR CODE
 * include "FrameAssembler.h" instead
 ***********************************************/

struct EigerPacketData : public PacketDataBase {};

using EigerPacket = StdPacketImpl<EigerPacketData>;
using EigerAssembler = DefaultFrameAssembler<EigerPacket>;

/**
 *@short Eiger frame assembler in raw mode: 2x Default frame assemblers
 */

class EigerRawFrameAssembler : public DualPortFrameAssembler {

  public:
    EigerRawFrameAssembler(DefaultFrameAssemblerBase::Ptr a[2]);

    virtual PortsMask assembleFrame(uint64_t frame, RecvHeader *recv_header,
                                    char *buf);
};

/**
 *@short Eiger frame assembler in std mode: port interleaving
 */

class EigerStdFrameAssembler : public DualPortFrameAssembler {

  public:
    EigerStdFrameAssembler(DefaultFrameAssemblerBase::Ptr a[2], bool flipped);
    ~EigerStdFrameAssembler();

    virtual PortsMask assembleFrame(uint64_t frame, RecvHeader *recv_header,
                                    char *buf);

    class Helper;

  protected:
    Helper *helper;
};
