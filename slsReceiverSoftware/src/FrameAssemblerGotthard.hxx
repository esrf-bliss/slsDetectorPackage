/************************************************
 * @file FrameAssemblerGotthard.hxx
 * @short helper classes assembling Gotthard frames
 * from udp packets
 * DO NOT INCLUDE THIS FILE DIRECTLY IN YOUR CODE
 * include "FrameAssembler.h" instead
 ***********************************************/

namespace FrameAssembler {
namespace Gotthard {

/**
 *@short Gotthard Packet class
 *
 * Gotthard Full mode data:
 *   1st packet: CACA + CACA, (640 - 1) * 2 bytes data
 *   2nd packet: (2 + 640 - 1) * 2 bytes data
 *
 * Gotthard Roi mode data:
 *   1st packet: CACA + CACA, (256 - 1) * 2 bytes data
 */

struct NetworkHeader {
    uint32_t packet_number;
    uint32_t sign_data;
} __attribute__((packed));

struct FullMode {
    static constexpr int PacketDataLen = (640 - 1) * 2;
    static constexpr int PacketsPerFrame = 2;

    using PacketDataBase =
        FrameAssembler::PacketData<PacketDataLen, NetworkHeader,
                                   PacketsPerFrame>;
    struct StreamData {
        bool inited{false};
        int packet_offset;

        void init(NetworkHeader *network_header) {
            if (!inited) {
                bool first_packet = (network_header->sign_data == 0xCACACACA);
                bool odd_number = network_header->packet_number & 1;
                packet_offset = (first_packet == odd_number) ? 1 : 0;
                inited = true;
            }
        }

        uint32_t correctFramePacket(NetworkHeader *network_header) {
            return network_header->packet_number + packet_offset;
        }
        int getFrameNumber(NetworkHeader *network_header) {
            return correctFramePacket(network_header) / PacketsPerFrame;
        }
        int getPacketNumber(NetworkHeader *network_header) {
            return correctFramePacket(network_header) % PacketsPerFrame;
        }
    };
};

struct RoiMode {
    static constexpr int PacketDataLen = (256 - 1) * 2;
    static constexpr int PacketsPerFrame = 1;

    using PacketDataBase =
        FrameAssembler::PacketData<PacketDataLen, NetworkHeader,
                                   PacketsPerFrame>;
    struct StreamData {
        int getFrameNumber(NetworkHeader *network_header) {
            return network_header->packet_number;
        }
        int getPacketNumber(NetworkHeader *network_header) { return 0; }
    };
};

template <class Mode> using PacketData = typename Mode::PacketDataBase;

template <class Mode>
struct PacketImpl : FrameAssembler::Packet<PacketData<Mode>> {
    using Base = FrameAssembler::Packet<PacketData<Mode>>;
    using StreamData = typename Mode::StreamData;

    StreamData &stream_data;

    PacketImpl(char *b, StreamData &sd) : Base(b, sd), stream_data(sd) {}

    void initSoftHeader() { stream_data.init(Base::networkHeader()); }

    uint64_t frame() {
        return stream_data.getFrameNumber(Base::networkHeader());
    }

    uint32_t number() {
        return stream_data.getPacketNumber(Base::networkHeader());
    }

    uint32_t sizeAdjust() { return (number() == 0) ? 0 : (2 * 2); }

    void fillDetHeader(FrameAssembler::DetHeader *det_header);
};

using FullPacket = PacketImpl<FullMode>;
using RoiPacket = PacketImpl<RoiMode>;

} // namespace Gotthard
} // namespace FrameAssembler
