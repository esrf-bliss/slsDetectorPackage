/************************************************
 * @file FrameAssemblerJungfrau.hxx
 * @short helper classes assembling Jungfrau frames
 * from udp packets
 * DO NOT INCLUDE THIS FILE DIRECTLY IN YOUR CODE
 * include "FrameAssembler.h" instead
 ***********************************************/

namespace FrameAssembler {
namespace Jungfrau {

struct PacketData : PacketDataBase {
    struct StreamData : PacketDataBase::StreamData {
        StreamData(GeneralDataPtr d) : PacketDataBase::StreamData(d) {}

        uint32_t getPacketNumber(uint32_t packet_idx);
    };
};

using Packet = StdPacketImpl<PacketData>;
using Assembler = DefaultFrameAssembler<Packet>;

/**
 *@short Jungfrau frame assembler in raw mode: Default frame assembler
 */

class RawFrameAssembler : public FrameAssemblerBase {

  public:
    RawFrameAssembler(DefaultFrameAssemblerBase::Ptr a);

    Result assembleFrame(uint64_t frame, RecvHeader *recv_header,
                         char *buf) override;

    void stop() override;

  private:
    DefaultFrameAssemblerBase::Ptr assembler;
};

/**
 *@short Jungfrau frame assembler in standard mode: Default frame assembler
 */

class StdFrameAssembler : public FrameAssemblerBase {

  public:
    StdFrameAssembler(DefaultFrameAssemblerBase::Ptr a);
    ~StdFrameAssembler();

    Result assembleFrame(uint64_t frame, RecvHeader *recv_header,
                         char *buf) override;

    void stop() override;

    class Helper;

  private:
    DefaultFrameAssemblerBase::Ptr assembler;
    Helper *helper;
};

} // namespace Jungfrau
} // namespace FrameAssembler
