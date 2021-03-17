/************************************************
 * @file Packet.cxx
 * @short low-level udp packet definition classes
 ***********************************************/

#include "Packet.h"

/**
 * PacketBlock
 */

template <class P> void PacketBlock<P>::setValid(unsigned int i, bool valid) {
    (*this)[i].softHeader()->valid = valid;
    if (valid)
        ++valid_packets;
}

template <class P> void PacketBlock<P>::moveToGood(P &p) {
    P dst = (*this)[p.number()];
    *dst.buffer = *p.buffer;
    p.softHeader()->valid = false;
    dst.softHeader()->valid = true;
}
