/************************************************
 * @file FrameAssemblerJungfrau.hxx
 * @short helper classes assembling Jungfrau frames
 * from udp packets
 * DO NOT INCLUDE THIS FILE DIRECTLY IN YOUR CODE
 * include "FrameAssembler.h" instead
 ***********************************************/

namespace FrameAssembler {
namespace Jungfrau {

template <int NbUDPIfaces> struct PacketData : PacketDataBase {
    struct StreamData : PacketDataBase::StreamData {
        StreamData(GeneralDataPtr d, int idx)
            : PacketDataBase::StreamData(d, idx) {}

        uint32_t getPacketNumber(uint32_t packet_idx);
    };
};

template <int NbUDPIfaces>
using Packet = StdPacketImpl<PacketData<NbUDPIfaces>>;
template <int NbUDPIfaces>
using Assembler = DefaultFrameAssembler<Packet<NbUDPIfaces>>;

/**
 *@short Jungfrau frame assembler in raw mode: Default frame assembler
 */

template <int NbUDPIfaces> class RawFrameAssembler : public FrameAssemblerBase {

  public:
    RawFrameAssembler(DefaultFrameAssemblerBase::Ptr a[NbUDPIfaces]);

    Result assembleFrame(uint64_t frame, RecvHeader *recv_header,
                         char *buf) override;

    void stop() override;

  private:
    DefaultFrameAssemblerBase::Ptr assembler[NbUDPIfaces];
};

/**
 *@short Jungfrau frame assembler in standard mode: Default frame assembler
 */

template <int NbUDPIfaces> class StdFrameAssembler : public FrameAssemblerBase {

  public:
    StdFrameAssembler(DefaultFrameAssemblerBase::Ptr a[NbUDPIfaces]);
    ~StdFrameAssembler();

    Result assembleFrame(uint64_t frame, RecvHeader *recv_header,
                         char *buf) override;

    void stop() override;

    class Helper;

  private:
    DefaultFrameAssemblerBase::Ptr assembler[NbUDPIfaces];
    Helper *helper;
};

} // namespace Jungfrau
} // namespace FrameAssembler
