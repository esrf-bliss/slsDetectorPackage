#pragma once

#include <cstdint>

namespace sls {

// StreamDaTa is a LUT that returns the target packet position in memory from
// the given packet_idx In other words, it gives the expected order of the
// packet sequence \note An instance of StreamData is included in the
// PacketStream
template <class Packet> struct StreamData {
    // Describes the sequence of the packets in the stream
    constexpr std::uint32_t getPacketNumber(std::uint32_t packet_idx) {
        return packet_idx;
    }
    // Reverse sequence of the packets in the stream
    constexpr std::uint32_t getPacketIndex(std::uint32_t packet_nb) {
        return packet_nb;
    }
};

} // namespace sls
