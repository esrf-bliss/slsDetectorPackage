// SPDX-License-Identifier: LGPL-3.0-or-other
// Copyright (C) 2021 Contributors to the SLS Detector Package
/************************************************
 * @file Listener.cpp
 * @short creates the listener thread that
 * listens to udp sockets, writes data to memory
 * & puts pointers to their memory addresses into fifos
 ***********************************************/

#include "Listener.h"
#include "Fifo.h"
#include "GeneralData.h"
#include "sls/UdpRxSocket.h"
#include "sls/container_utils.h" // For sls::make_unique<>
#include "sls/network_utils.h"
#include "sls/sls_detector_exceptions.h"

#include <cerrno>
#include <cstring>
#include <iostream>

const std::string Listener::TypeName = "Listener";

Listener::Listener(int ind, detectorType dtype, Fifo *f,
                   std::atomic<runStatus> *s, uint32_t *portno, std::string *e,
                   int *us, int *as, frameDiscardPolicy *fdp, bool *sm)
    : ThreadObject(ind, TypeName), fifo(f), myDetectorType(dtype), status(s),
      udpPortNumber(portno), eth(e), udpSocketBufferSize(us),
      actualUDPSocketBufferSize(as), frameDiscardMode(fdp), silentMode(sm) {
    LOG(logDEBUG) << "Listener " << ind << " created";
}

Listener::~Listener() = default;

uint64_t Listener::GetPacketsCaught() const {
    if (!packetStream)
        return packetsCaught;
    return std::visit([&](auto &ps) { return ps.getNumPacketsCaught(); },
                      *packetStream);
}

uint64_t Listener::GetFirstFrameCaught() const {
    if (!packetStream)
        return firstFrameCaught;
    return std::visit([&](auto &ps) { return ps.getFirstFrameCaught(); },
                      *packetStream);
}

uint64_t Listener::GetNumFramesCaught() const {
    if (!packetStream)
        return numFramesCaught;
    return std::visit([&](auto &ps) { return ps.getNumFramesCaught(); },
                      *packetStream);
}

uint64_t Listener::GetNumCompleteFramesCaught() const {
    if (!packetStream)
        return numCompleteFramesCaught;
    return std::visit([&](auto &ps) { return ps.getNumCompleteFramesCaught(); },
                      *packetStream);
}

uint64_t Listener::GetLastFrameIndexCaught() const {
    if (!packetStream)
        return lastFrameIndexCaught;
    return std::visit([&](auto &ps) { return ps.getLastFrameIndex(); },
                      *packetStream);
}

uint64_t Listener::GetNumMissingPacket(bool stoppedFlag, uint64_t numPackets) {
    uint64_t numPacketsCaught = GetPacketsCaught();
    if (!stoppedFlag) {
        return (numPackets - numPacketsCaught);
    }
    if (numPacketsCaught == 0) {
        return numPacketsCaught;
    }
    uint64_t frames = GetLastFrameIndexCaught() - GetFirstFrameCaught() + 1;
    return frames * generalData->packetsPerFrame - numPacketsCaught;
}

void Listener::SetFifo(Fifo *f) { fifo = f; }

void Listener::ResetParametersforNewAcquisition() { StopRunning(); }

void Listener::SetThreadCPUAffinity(AnyCPUAffinity cpu_affinity) {
    cpuAffinity = cpu_affinity;
}

void Listener::SetGeneralData(GeneralData *g) { generalData = g; }

void Listener::CreateUDPSockets() {
    // if eth is mistaken with ip address
    if ((*eth).find('.') != std::string::npos) {
        (*eth) = "";
    }
    if (!(*eth).length()) {
        LOG(logWARNING) << "eth is empty. Listening to all";
    }

    ShutDownUDPSocket();

    uint32_t packetSize = generalData->packetSize;
    if (myDetectorType == GOTTHARD2 && index != 0) {
        packetSize = generalData->vetoPacketSize;
    }

    // InterfaceNameToIp(eth).str().c_str()
    try {
        udpSocket = std::make_shared<sls::UdpRxSocket>(
            *udpPortNumber, packetSize,
            ((*eth).length() ? sls::InterfaceNameToIp(*eth).str().c_str()
                             : nullptr),
            *udpSocketBufferSize);
        LOG(logINFO) << index << ": UDP port opened at port " << *udpPortNumber;
    } catch (...) {
        throw sls::RuntimeError("Could not create UDP socket on port " +
                                std::to_string(*udpPortNumber));
    }

    auto packetContainer = fifo->GetPacketContainer();

    try {
        packetStream = CreatePacketStream(
            udpSocket, generalData->myDetectorType, generalData->tgEnable,
            generalData->numUDPInterfaces, generalData->dynamicRange, index,
            cpuAffinity, *frameDiscardMode, packetContainer);
        LOG(logINFO) << index << ": PacketStream for port " << *udpPortNumber;
    } catch (...) {
        throw sls::RuntimeError("Could not create PacketStream on port " +
                                std::to_string(*udpPortNumber));
    }

    udpSocketAlive = true;

    // doubled due to kernel bookkeeping (could also be less due to permissions)
    *actualUDPSocketBufferSize = udpSocket->getBufferSize();
}

void Listener::ShutDownUDPSocket() {
    if (!udpSocket)
        return;

    udpSocketAlive = false;
    if (packetStream)
        std::visit(
            [&](auto &ps) {
                ps.stop();
                ps.printStats();

                packetsCaught = ps.getNumPacketsCaught();
                firstFrameCaught = ps.getFirstFrameCaught();
                numFramesCaught = ps.getNumFramesCaught();
                numCompleteFramesCaught = ps.getNumCompleteFramesCaught();
                lastFrameIndexCaught = ps.getLastFrameIndex();
            },
            *packetStream);
    udpSocket->Shutdown();
    packetStream.reset();
    udpSocket.reset();
    LOG(logINFO) << "Shut down of UDP port " << *udpPortNumber;
}

void Listener::CreateDummySocketForUDPSocketBufferSize(int s) {
    LOG(logINFO) << "Testing UDP Socket Buffer size " << s << " with test port "
                 << *udpPortNumber;

    int temp = *udpSocketBufferSize;
    *udpSocketBufferSize = s;

    // if eth is mistaken with ip address
    if ((*eth).find('.') != std::string::npos) {
        (*eth) = "";
    }

    uint32_t packetSize = generalData->packetSize;
    if (myDetectorType == GOTTHARD2 && index != 0) {
        packetSize = generalData->vetoPacketSize;
    }

    // create dummy socket
    try {
        sls::UdpRxSocket g(*udpPortNumber, packetSize,
                           ((*eth).length()
                                ? sls::InterfaceNameToIp(*eth).str().c_str()
                                : nullptr),
                           *udpSocketBufferSize);

        // doubled due to kernel bookkeeping (could also be less due to
        // permissions)
        *actualUDPSocketBufferSize = g.getBufferSize();
        if (*actualUDPSocketBufferSize == -1) {
            *udpSocketBufferSize = temp;
        } else {
            *udpSocketBufferSize = (*actualUDPSocketBufferSize) / 2;
        }

    } catch (...) {
        throw sls::RuntimeError("Could not create a test UDP socket on port " +
                                std::to_string(*udpPortNumber));
    }
}

void Listener::ThreadExecution() {
    std::visit([&](auto &ps) { ps.threadFunction(); }, *packetStream);
    StopRunning();

    if (!(*silentMode) && !index) {
        LOG(logINFOBLUE) << index << " First Index: " << GetFirstFrameCaught();
    }
}
