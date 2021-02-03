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
                   int *flx, bool do_udp_read)
    : ThreadObject(ind, TypeName, do_udp_read), fifo(f), myDetectorType(dtype),
      status(s), udpPortNumber(portno), eth(e), numImages(nf),
      udpSocketBufferSize(us), actualUDPSocketBufferSize(as),
      framesPerFile(fpf), frameDiscardMode(fdp), activated(act),
      deactivatedPaddingEnable(depaden), silentMode(sm), flippedDataX(flx),
      doUdpRead(do_udp_read) {
    LOG(logDEBUG) << "Listener " << ind << " created";
    CPU_ZERO(&cpuMask);
}

Listener::~Listener() = default;

uint64_t Listener::GetPacketsCaught() const {
    return frameAssembler ? frameAssembler->getNumPacketsCaught() : 0;
}

uint64_t Listener::GetNumFramesCaught() {
    return frameAssembler ? frameAssembler->getNumFramesCaught() : 0;
}

uint64_t Listener::GetLastFrameIndexCaught() const {
    return frameAssembler ? frameAssembler->getLastFrameIndex() : 0;
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
        frameAssembler = FrameAssembler::CreateDefaultFrameAssembler(
            udpSocket, generalData, index, cpuMask, fifoNodeMask, maxNode,
            *frameDiscardMode, !doUdpRead);
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

void Listener::ShutDownUDPSocket() {
    if (udpSocket) {
        bool was_alive = udpSocketAlive;
        udpSocketAlive = false;
        if (!doUdpRead)
            StopRunning();
        if (frameAssembler) {
            frameAssembler->stop();
            if (was_alive)
                frameAssembler->printStreamStats();
        }
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
    char *buffer;
    int rc = 0;

    fifo->GetNewAddress(buffer);
    LOG(logDEBUG5) << "Listener " << index
                   << ", "
                      "pop 0x"
                   << std::hex << (void *)(buffer) << std::dec << ":" << buffer;

    uint32_t *byte_count = (uint32_t *)buffer;
    char *header_ptr = buffer + FIFO_HEADER_NUMBYTES;
    sls_receiver_header *recv_header = (sls_receiver_header *)header_ptr;
    char *image_data = (header_ptr + sizeof(sls_receiver_header));

    // udpsocket doesnt exist
    bool carryOverFlag = frameAssembler->hasPendingPacket();
    if (*activated && !udpSocketAlive && !carryOverFlag) {
        byte_count = 0;
        StopListening(buffer);
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
            (*((uint32_t *)buffer)) = 0;
            StopListening(buffer);
        } else
            fifo->FreeAddress(buffer);
        return;
    }

    // discarding image
    else if (rc < 0) {
        LOG(logDEBUG) << index << " discarding fnum:" << currentFrameIndex;
        fifo->FreeAddress(buffer);
        currentFrameIndex++;
        return;
    }

    *byte_count = rc;
    recv_header->detHeader.frameNumber =
        currentFrameIndex; // for those returning earlier
    currentFrameIndex++;

    // push into fifo
    fifo->PushAddress(buffer);

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

Listener::FrameAssemblerPtr
Listener::CreateFrameAssembler(std::vector<Ptr> &listener, int det_ifaces[2]) {
    FrameAssemblerPtr fa;
    GeneralData *gd = listener[0]->generalData;
    detectorType d = listener[0]->myDetectorType;
    int nb_ports = listener.size();
    bool raw = !gd->gapEnable;
    frameDiscardPolicy fp = *listener[0]->frameDiscardMode;
    using namespace FrameAssembler;
    DefaultFrameAssemblerList a;
    for (auto &l : listener)
        a.push_back(l->frameAssembler);
    if (raw) {
        fa = std::make_shared<FrameAssembler::RawFrameAssembler>(a);
    } else if (d == slsDetectorDefs::EIGER) {
        int pixel_bpp = gd->dynamicRange;
        int recv_idx = *listener[0]->flippedDataX ? 1 : 0;
        fa = FrameAssembler::Eiger::CreateFrameAssembler(
            pixel_bpp, fp, gd->tgEnable, det_ifaces, recv_idx, a);
    } else if (d == slsDetectorDefs::JUNGFRAU) {
        fa = FrameAssembler::Jungfrau::CreateFrameAssembler(det_ifaces,
                                                            nb_ports, fp, a);
    } else
        throw sls::RuntimeError("FrameAssembler not available for " +
                                sls::ToString(d));

    return fa;
}

void Listener::ClearAllBuffers() {
    if (frameAssembler)
        frameAssembler->clearBuffers();
}

void Listener::StopListening(char *buf) {
    uint32_t *byte_count = (uint32_t *)buf;
    *byte_count = DUMMY_PACKET_VALUE;
    fifo->PushAddress(buf);
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
    FrameAssembler::Result res;
    res = frameAssembler->assembleFrame(fnum, recv_header, buf);
    recv_header->detHeader.row = row;
    recv_header->detHeader.column = column;
    if (res.valid_data.none())
        return -1;

    // update parameters
    numpackets = recv_header->detHeader.packetNumber;
    numPacketsStatistic += numpackets;
    if (!startedFlag)
        RecordFirstIndex(fnum);

    return imageSize;
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
