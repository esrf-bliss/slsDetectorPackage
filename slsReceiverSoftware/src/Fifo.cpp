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

Fifo::Fifo(int ind, uint32_t imageSize, uint32_t depth)
    : index(ind), memory(nullptr), fifoBound(nullptr), fifoFree(nullptr),
      fifoStream(nullptr), fifoDepth(depth), status_fifoBound(0),
      status_fifoFree(depth) {
    LOG(logDEBUG3) << __SHORT_AT__ << " called";
    CreateFifos(imageSize);
}

Fifo::~Fifo() {
    LOG(logDEBUG3) << __SHORT_AT__ << " called";
    DestroyFifos();
}

void Fifo::CreateFifos(uint32_t imageSize) {
    LOG(logDEBUG3) << __SHORT_AT__ << " called";

    // destroy if not already
    DestroyFifos();

    // create fifos
    fifoBound = new sls::CircularFifo<FifoFrame *>(fifoDepth);
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
    LOG(logDEBUG) << "Memory Allocated " << index << ": "
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
    LOG(logINFO) << "Fifo " << index << " reconstructed Depth (rx_fifodepth): "
                 << fifoFree->getDataValue();
}

void Fifo::DestroyFifos() {
    LOG(logDEBUG3) << __SHORT_AT__ << " called";

    if (memory) {
        free(memory);
        memory = nullptr;
    }
    delete fifoBound;
    fifoBound = nullptr;
    delete fifoFree;
    fifoFree = nullptr;
    delete fifoStream;
    fifoStream = nullptr;
}

void Fifo::FreeFrame(FifoFrame *frame) { fifoFree->push(frame); }

void Fifo::GetNewFrame(FifoFrame *&frame) {
    int temp = fifoFree->getDataValue();
    if (temp < status_fifoFree)
        status_fifoFree = temp;
    fifoFree->pop(frame);
}

void Fifo::PushFrame(FifoFrame *frame) {
    int temp = fifoBound->getDataValue();
    if (temp > status_fifoBound)
        status_fifoBound = temp;
    while (!fifoBound->push(frame))
        ;
}

void Fifo::PopFrame(FifoFrame *&frame) { fifoBound->pop(frame); }

void Fifo::PushFrameToStream(FifoFrame *frame) { fifoStream->push(frame); }

void Fifo::PopFrameToStream(FifoFrame *&frame) { fifoStream->pop(frame); }

int Fifo::GetMaxLevelForFifoBound() {
    int temp = status_fifoBound;
    status_fifoBound = 0;
    return temp;
}

int Fifo::GetMinLevelForFifoFree() {
    int temp = status_fifoFree;
    status_fifoFree = fifoDepth;
    return temp;
}

size_t Fifo::GetFifoFrameSize() { return fifoFrameSize; }
