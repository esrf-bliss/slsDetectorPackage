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
                   uint64_t *nf, int *us, int *as, uint32_t *fpf,
                   frameDiscardPolicy *fdp, bool *act, bool *depaden, bool *sm,
                   int *flx, bool push_to_fifo)
    : ThreadObject(ind, TypeName), fifo(f), myDetectorType(dtype), status(s),
      udpPortNumber(portno), eth(e), numImages(nf), udpSocketBufferSize(us),
      actualUDPSocketBufferSize(as), framesPerFile(fpf), frameDiscardMode(fdp),
      activated(act), deactivatedPaddingEnable(depaden), silentMode(sm),
      flippedDataX(flx), pushFramesToFifo(push_to_fifo) {
    LOG(logDEBUG) << "Listener " << ind << " created";
    CPU_ZERO(&cpuMask);
}

Listener::~Listener() = default;

uint64_t Listener::GetPacketsCaught() const {
    if (!packetStream)
        return 0;
    return std::visit([&](auto &ps) { return ps.getNumPacketsCaught(); },
                      *packetStream);
}

uint64_t Listener::GetNumFramesCaught() {
    if (!packetStream)
        return 0;
    return std::visit([&](auto &ps) { return ps.getNumFramesCaught(); },
                      *packetStream);
}

uint64_t Listener::GetLastFrameIndexCaught() const {
    if (!packetStream)
        return 0;
    return std::visit([&](auto &ps) { return ps.getLastFrameIndex(); },
                      *packetStream);
}

uint64_t Listener::GetNumMissingPacket(bool stoppedFlag,
                                       uint64_t numPackets) const {
    uint64_t numPacketsCaught = GetPacketsCaught();
    if (!stoppedFlag) {
        return (numPackets - numPacketsCaught);
    }
    if (numPacketsCaught == 0) {
        return numPacketsCaught;
    }
    return ((GetLastFrameIndexCaught() - firstIndex + 1) *
                generalData->packetsPerFrame -
            numPacketsCaught);
}

void Listener::SetFifo(Fifo *f) { fifo = f; }

void Listener::ResetParametersforNewAcquisition() {
    StopRunning();
    startedFlag = false;
    firstIndex = 0;
    currentFrameIndex = 0;
    numPacketsStatistic = 0;
    numFramesStatistic = 0;
    // reset fifo statistic
    fifo->GetMaxLevelForFifoBound();
    fifo->GetMinLevelForFifoFree();
}

void Listener::SetThreadCPUAffinity(const cpu_set_t &cpu_mask) {
    cpuMask = cpu_mask;
}

void Listener::SetFifoNodeAffinity(unsigned long fifo_node_mask, int max_node) {
    fifoNodeMask = fifo_node_mask;
    maxNode = max_node;
    LOG(logINFO) << "Node mask: " << std::hex << std::showbase << fifo_node_mask
                 << std::dec << ", max_node: " << max_node;
}

void Listener::RecordFirstIndex(uint64_t fnum) {
    // listen to this fnum, later +1
    currentFrameIndex = fnum;

    startedFlag = true;
    firstIndex = fnum;

    if (!(*silentMode)) {
        if (!index) {
            LOG(logINFOBLUE) << index << " First Index: " << firstIndex;
        }
    }
}

void Listener::SetGeneralData(GeneralData *g) { generalData = g; }

void Listener::CreateUDPSockets() {
    if (!(*activated)) {
        return;
    }

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

    try {
        packetStream = CreatePacketStream(udpSocket, generalData, index,
                                          cpuMask, fifoNodeMask, maxNode,
                                          GetThreadId(), *frameDiscardMode);
        bool e4b = !pushFramesToFifo;
        frameAssembler =
            FrameAssembler::CreateDefaultFrameAssembler(generalData, e4b);
        LOG(logINFO) << index << ": Default FrameAssembler for port "
                     << *udpPortNumber;
    } catch (...) {
        throw sls::RuntimeError("Could not create FrameAssembler on port " +
                                std::to_string(*udpPortNumber));
    }

    udpSocketAlive = true;

    // doubled due to kernel bookkeeping (could also be less due to permissions)
    *actualUDPSocketBufferSize = udpSocket->getBufferSize();
}

void Listener::Stop() {
    if (packetStream)
        std::visit([&](auto &ps) { ps.stop(); }, *packetStream);
}

void Listener::ShutDownUDPSocket() {
    if (udpSocket) {
        bool was_alive = udpSocketAlive;
        udpSocketAlive = false;
        Stop();
        if (packetStream && was_alive)
            std::visit([&](auto &ps) { ps.printStats(); }, *packetStream);
        udpSocket->Shutdown();
        LOG(logINFO) << "Shut down of UDP port " << *udpPortNumber;
    }
}

void Listener::CreateDummySocketForUDPSocketBufferSize(int s) {
    LOG(logINFO) << "Testing UDP Socket Buffer size " << s << " with test port "
                 << *udpPortNumber;

    if (!(*activated)) {
        *actualUDPSocketBufferSize = (s * 2);
        return;
    }

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

void Listener::SetHardCodedPosition(uint16_t r, uint16_t c) {
    row = r;
    column = c;
}

void Listener::ThreadExecution() {
    if (!pushFramesToFifo) {
        std::visit([&](auto &ps) { ps.threadFunction(); }, *packetStream);
        StopRunning();
        return;
    }

    FifoFrame *frame;
    int rc = 0;

    fifo->GetNewFrame(frame);
    LOG(logDEBUG5) << "Listener " << index << ", " << std::hex << "pop 0x"
                   << (void *)frame << " "
                   << "[data: 0x" << (void *)frame->recvFrame.data << "]"
                   << std::dec;

    sls_receiver_header *recv_header = &frame->recvFrame.header;
    char *image_data = frame->recvFrame.data;

    // udpsocket doesnt exist
    bool carryOverFlag;
    std::visit([&](auto &ps) { carryOverFlag = ps.hasPendingPacket(); },
               *packetStream);
    if (*activated && !udpSocketAlive && !carryOverFlag) {
        StopListening(frame);
        return;
    }

    // get data
    if ((*status != TRANSMITTING && (!(*activated) || udpSocketAlive)) ||
        carryOverFlag) {
        rc = ListenToAnImage(recv_header, image_data);
    }

    // error check, (should not be here) if not transmitting yet (previous if)
    // rc should be > 0
    if (rc == 0) {
        if (!udpSocketAlive) {
            StopListening(frame);
        } else
            fifo->FreeFrame(frame);
        return;
    } else if (rc < 0) { // discarding image
        LOG(logDEBUG) << index << " discarding fnum:" << currentFrameIndex;
        fifo->FreeFrame(frame);
        currentFrameIndex++;
        return;
    }

    frame->recvFrame.numBytes = rc;
    // for those returning earlier
    recv_header->detHeader.frameNumber = currentFrameIndex;
    currentFrameIndex++;

    // push into fifo
    fifo->PushFrame(frame);

    // Statistics
    if (!(*silentMode)) {
        numFramesStatistic++;
        if (numFramesStatistic >=
            // second condition also for infinite #number of frames
            (((*framesPerFile) == 0) ? STATISTIC_FRAMENUMBER_INFINITE
                                     : (*framesPerFile)))
            PrintFifoStatistics();
    }
}

void Listener::ClearAllBuffers() {
    if (packetStream)
        std::visit([&](auto &ps) { ps.clearBuffers(); }, *packetStream);
}

void Listener::StopListening(FifoFrame *frame) {
    frame->end = true;
    fifo->PushFrame(frame);
    StopRunning();
}

/* buf includes the fifo header and packet header */
int Listener::ListenToAnImage(sls_receiver_header *recv_header, char *buf) {

    uint64_t fnum = 0;
    uint32_t numpackets = 0;
    uint32_t imageSize = generalData->imageSize;

    // deactivated (eiger)
    if (!(*activated)) {
        // no padding
        if (!(*deactivatedPaddingEnable))
            return 0;
        // padding without setting bitmask (all missing packets padded in
        // dataProcessor)
        if (currentFrameIndex >= *numImages)
            return 0;

        //(eiger) first fnum starts at 1
        if (!currentFrameIndex) {
            ++currentFrameIndex;
        }
        memset(recv_header, 0, sizeof(sls_receiver_header));
        recv_header->detHeader.frameNumber = currentFrameIndex;
        recv_header->detHeader.row = row;
        recv_header->detHeader.column = column;
        recv_header->detHeader.detType = (uint8_t)generalData->myDetectorType;
        recv_header->detHeader.version = (uint8_t)SLS_DETECTOR_HEADER_VERSION;
        return imageSize;
    }

    fnum = currentFrameIndex;
    auto block = GetFramePackets(fnum);
    bool ok;
    ok = frameAssembler->assembleFrame(std::move(block), recv_header, buf);
    recv_header->detHeader.row = row;
    recv_header->detHeader.column = column;
    if (!ok)
        return -1;

    // update parameters
    numpackets = recv_header->detHeader.packetNumber;
    numPacketsStatistic += numpackets;
    if (!startedFlag)
        RecordFirstIndex(fnum);

    return imageSize;
}

AnyPacketBlockPtr Listener::GetFramePackets(uint64_t frame) {
    return std::visit(
        [&](auto &ps) -> AnyPacketBlockPtr { return ps.getPacketBlock(frame); },
        *packetStream);
}

void Listener::PrintFifoStatistics() {
    LOG(logDEBUG1) << "numFramesStatistic:" << numFramesStatistic
                   << " numPacketsStatistic:" << numPacketsStatistic
                   << " packetsperframe:" << generalData->packetsPerFrame;

    // calculate packet loss
    int64_t totalP = numFramesStatistic * (generalData->packetsPerFrame);
    int64_t loss = totalP - numPacketsStatistic;
    int lossPercent = ((double)loss / (double)totalP) * 100.00;
    numPacketsStatistic = 0;
    numFramesStatistic = 0;

    const auto color = loss ? logINFORED : logINFOGREEN;
    LOG(color) << "[" << *udpPortNumber
               << "]:  "
                  "Packet_Loss:"
               << loss << " (" << lossPercent << "%)"
               << "  Used_Fifo_Max_Level:" << fifo->GetMaxLevelForFifoBound()
               << " \tFree_Slots_Min_Level:" << fifo->GetMinLevelForFifoFree()
               << " \tCurrent_Frame#:" << currentFrameIndex;
}
