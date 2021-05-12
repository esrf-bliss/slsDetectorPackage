/************************************************
 * @file Fifo.cpp
 * @short constructs the fifo structure
 * which is a circular buffer with pointers to
 * parts of allocated memory
 ***********************************************/

#include "Fifo.h"
#include "sls/sls_detector_exceptions.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <unistd.h>

Fifo::Fifo(int ind, GeneralDataPtr gd, uint32_t depth, unsigned long node_mask,
           int max_node)
    : index(ind), memory(nullptr), fifoFree(nullptr), fifoStream(nullptr),
      fifoDepth(depth), status_fifoFree(depth) {
    LOG(logDEBUG3) << __SHORT_AT__ << " called";
    CreateFifos(gd, node_mask, max_node);
}

Fifo::~Fifo() {
    LOG(logDEBUG3) << __SHORT_AT__ << " called";
    DestroyFifos();
}

void Fifo::CreateFifos(GeneralDataPtr gd, unsigned long node_mask,
                       int max_node) {
    LOG(logDEBUG3) << __SHORT_AT__ << " called";

    // destroy if not already
    DestroyFifos();

    try {
        packetContainer = CreatePacketContainer(
            gd->myDetectorType, gd->tgEnable, gd->numUDPInterfaces,
            gd->dynamicRange, fifoDepth, node_mask, max_node);
        LOG(logINFO) << "Fifo " << index
                     << " packet Depth (rx_fifodepth): " << fifoDepth;
        long long mem_len;
        std::visit([&](auto &pc) { mem_len = pc.getMemorySize(); },
                   *packetContainer);
        LOG(logDEBUG) << "Memory Allocated " << index << ": "
                      << mem_len / (double)(1024 * 1024) << " MB";
    } catch (...) {
        throw sls::RuntimeError("Could not create PacketContainer");
    }

    fifoDepth = 10;
    uint32_t imageSize = gd->imageSize;
    // veto data size
    if ((gd->myDetectorType == GOTTHARD2) && (index != 0)) {
        imageSize = gd->vetoImageSize;
    }
    // create fifos
    fifoFree = new sls::CircularFifo<FifoFrame *>(fifoDepth);
    fifoStream = new sls::CircularFifo<FifoFrame *>(fifoDepth);

    // allocate memory
    fifoFrameSize = offsetof(FifoFrame, recvFrame.data[0]) + imageSize;

    size_t mem_len = fifoFrameSize * fifoDepth;
    memory = (char *)malloc(mem_len);
    if (memory == nullptr) {
        throw sls::RuntimeError("Could not allocate memory for fifos");
    }
    memset(memory, 0, mem_len);
    int pagesize = getpagesize();
    const char *t = "memory";
    for (size_t i = 0; (i + strlen(t)) < mem_len; i += pagesize)
        strcpy(memory + i, t);
    LOG(logDEBUG) << "Memory Allocated for Streamer " << index << ": "
                  << (double)mem_len / (double)(1024 * 1024) << " MB";

    { // push free addresses into fifoFree fifo
        char *buffer = memory;
        for (int i = 0; i < fifoDepth; ++i) {
            FifoFrame *frame =
                static_cast<FifoFrame *>(static_cast<void *>(buffer));
            FreeFrame(frame);
            buffer += fifoFrameSize;
        }
    }
    LOG(logINFO) << "Fifo " << index
                 << " frame-4-streamer Depth: " << fifoFree->getDataValue();
}

void Fifo::DestroyFifos() {
    LOG(logDEBUG3) << __SHORT_AT__ << " called";

    if (memory) {
        free(memory);
        memory = nullptr;
    }
    delete fifoFree;
    fifoFree = nullptr;
    delete fifoStream;
    fifoStream = nullptr;

    packetContainer.reset();
}

AnyPacketContainerPtr Fifo::GetPacketContainer() { return packetContainer; }

sls::AnyPacketBlockPtr Fifo::GetFramePackets(uint64_t frame) {
    return std::visit(
        [&](auto &pc) -> sls::AnyPacketBlockPtr {
            return pc.getReadyPacketBlock(frame);
        },
        *packetContainer);
}

void Fifo::FreeFrame(FifoFrame *frame) { fifoFree->push(frame); }

void Fifo::GetNewFrame(FifoFrame *&frame) {
    int temp = fifoFree->getDataValue();
    if (temp < status_fifoFree)
        status_fifoFree = temp;
    fifoFree->pop(frame);
}

void Fifo::PushFrameToStream(FifoFrame *frame) { fifoStream->push(frame); }

void Fifo::PopFrameToStream(FifoFrame *&frame) { fifoStream->pop(frame); }

int Fifo::GetMaxLevelForFifoStream() {
    int temp = status_fifoStream;
    status_fifoStream = 0;
    return temp;
}

int Fifo::GetMinLevelForFifoFree() {
    int temp = status_fifoFree;
    status_fifoFree = fifoDepth;
    return temp;
}

size_t Fifo::GetFifoFrameSize() { return fifoFrameSize; }

void Fifo::ClearAllBuffers() {
    if (packetContainer)
        std::visit([&](auto &pc) { pc.clearBuffers(); }, *packetContainer);
}
