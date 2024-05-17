// SPDX-License-Identifier: LGPL-3.0-or-other
// Copyright (C) 2021 Contributors to the SLS Detector Package
/************************************************
 * @file DataStreamer.cpp
 * @short streams data from receiver via ZMQ
 ***********************************************/

#include "DataStreamer.h"
#include "Fifo.h"
#include "GeneralData.h"
#include "sls/ZmqSocket.h"
#include "sls/sls_detector_exceptions.h"

#include <cerrno>
#include <iostream>

const std::string DataStreamer::TypeName = "DataStreamer";

DataStreamer::DataStreamer(int ind, Fifo *f, uint32_t *dr, ROI *r, uint64_t *fi,
                           bool fr, int *nm, bool *qe, uint64_t *tot)
    : ThreadObject(ind, TypeName), fifo(f), dynamicRange(dr), roi(r),
      fileIndex(fi), flipRows(fr), quadEnable(qe), totalNumFrames(tot) {
    numMods[0] = nm[0];
    numMods[1] = nm[1];

    LOG(sls::logDEBUG) << "DataStreamer " << ind << " created";
}

DataStreamer::~DataStreamer() {
    CloseZmqSocket();
    delete[] completeBuffer;
}

void DataStreamer::SetFifo(Fifo *f) { fifo = f; }

void DataStreamer::ResetParametersforNewAcquisition(const std::string &fname) {
    StopRunning();
    startedFlag = false;
    firstIndex = 0;

    fileNametoStream = fname;
    if (completeBuffer) {
        delete[] completeBuffer;
        completeBuffer = nullptr;
    }
    if (generalData->myDetectorType == GOTTHARD && roi->xmin != -1) {
        adcConfigured = generalData->GetAdcConfigured(index, *roi);
        completeBuffer = new char[generalData->imageSizeComplete];
        memset(completeBuffer, 0, generalData->imageSizeComplete);
    }
}

void DataStreamer::RecordFirstIndex(uint64_t fnum, FifoFrame *frame) {
    startedFlag = true;
    // streamer first index needn't be
    firstIndex = fnum - frame->firstStreamerFrame;
    LOG(sls::logDEBUG1) << index << " First Index: " << firstIndex
                   << ", First Streamer Index:" << fnum;
}

void DataStreamer::SetGeneralData(GeneralData *g) { generalData = g; }

void DataStreamer::SetNumberofModules(int *nm) {
    numMods[0] = nm[0];
    numMods[1] = nm[1];
}

void DataStreamer::SetFlipRows(bool fd) { flipRows = fd; }

void DataStreamer::SetAdditionalJsonHeader(
    const std::map<std::string, std::string> &json) {
    std::lock_guard<std::mutex> lock(additionalJsonMutex);
    additionalJsonHeader = json;
    isAdditionalJsonUpdated = true;
}

void DataStreamer::CreateZmqSockets(int *nunits, uint32_t port,
                                    const sls::IpAddr ip, int hwm) {
    uint32_t portnum = port + index;
    std::string sip = ip.str();
    try {
        zmqSocket = new sls::ZmqSocket(portnum, (ip != 0 ? sip.c_str() : nullptr));
        // set if custom
        if (hwm >= 0) {
            zmqSocket->SetSendHighWaterMark(hwm);
            if (zmqSocket->GetSendHighWaterMark() != hwm) {
                throw sls::RuntimeError(
                    "Could not set zmq send high water mark to " +
                    std::to_string(hwm));
            }
        }
    } catch (...) {
        LOG(sls::logERROR) << "Could not create Zmq socket on port " << portnum
                      << " for Streamer " << index;
        throw;
    }
    LOG(sls::logINFO) << index << " Streamer: Zmq Server started at "
                 << zmqSocket->GetZmqServerAddress()
                 << "[hwm: " << zmqSocket->GetSendHighWaterMark() << "]";
}

void DataStreamer::CloseZmqSocket() {
    if (zmqSocket) {
        delete zmqSocket;
        zmqSocket = nullptr;
    }
}

void DataStreamer::ThreadExecution() {
    FifoFrame *frame;
    fifo->PopFrameToStream(frame);
    LOG(sls::logDEBUG5) << "DataStreamer " << index << ", " << std::hex << "pop 0x"
                   << (void *)frame << " "
                   << "[data: 0x" << (void *)frame->recvFrame.data << "]"
                   << std::dec;

    // check dummy
    auto &numBytes = frame->recvFrame.numBytes;
    LOG(sls::logDEBUG1) << "DataStreamer " << index << ", Numbytes:" << numBytes;
    if (frame->end) {
        StopProcessing(frame);
        return;
    }

    ProcessAnImage(frame);

    // free
    fifo->FreeFrame(frame);
}

void DataStreamer::StopProcessing(FifoFrame *frame) {
    LOG(sls::logDEBUG1) << "DataStreamer " << index << ": Dummy";

    sls_receiver_header *header = &frame->recvFrame.header;
    // send dummy header and data
    if (!SendHeader(header, 0, 0, 0, true)) {
        LOG(sls::logERROR) << "Could not send zmq dummy header for streamer "
                      << index;
    }

    fifo->FreeFrame(frame);
    StopRunning();
    LOG(sls::logDEBUG1) << index << ": Streaming Completed";
}

/** buf includes only the standard header */
void DataStreamer::ProcessAnImage(FifoFrame *frame) {

    sls_receiver_header *header = &frame->recvFrame.header;
    uint64_t fnum = header->detHeader.frameNumber;
    LOG(sls::logDEBUG1) << "DataStreamer " << index << ": fnum:" << fnum;

    if (!startedFlag)
        RecordFirstIndex(fnum, frame);

    char *buf = frame->recvFrame.data;
    auto &numBytes = frame->recvFrame.numBytes;
    if (completeBuffer) { // shortframe gotthard

        // disregarding the size modified from callback (always using
        // imageSizeComplete
        // instead of buf (32 bit) because gui needs imagesizecomplete and
        // listener
        // write imagesize

        if (!SendHeader(header, generalData->imageSizeComplete,
                        generalData->nPixelsXComplete,
                        generalData->nPixelsYComplete, false)) {
            LOG(sls::logERROR) << "Could not send zmq header for fnum " << fnum
                          << " and streamer " << index;
        }
        memcpy(completeBuffer + ((generalData->imageSize) * adcConfigured), buf,
               numBytes);

        if (!zmqSocket->SendData(completeBuffer,
                                 generalData->imageSizeComplete)) {
            LOG(sls::logERROR) << "Could not send zmq data for fnum " << fnum
                          << " and streamer " << index;
        }
    } else { // normal
             // new size possibly from callback
        if (!SendHeader(header, numBytes, generalData->nPixelsX,
                        generalData->nPixelsY, false)) {
            LOG(sls::logERROR) << "Could not send zmq header for fnum " << fnum
                          << " and streamer " << index;
        }
        // new size possibly from callback
        if (!zmqSocket->SendData(buf, numBytes)) {
            LOG(sls::logERROR) << "Could not send zmq data for fnum " << fnum
                          << " and streamer " << index;
        }
    }
}

int DataStreamer::SendHeader(sls_receiver_header *rheader, uint32_t size,
                             uint32_t nx, uint32_t ny, bool dummy) {

    sls::zmqHeader zHeader;
    zHeader.data = !dummy;
    zHeader.jsonversion = SLS_DETECTOR_JSON_HEADER_VERSION;

    if (dummy) {
        return zmqSocket->SendHeader(index, zHeader);
    }

    sls_detector_header header = rheader->detHeader;

    uint64_t frameIndex = header.frameNumber - firstIndex;
    uint64_t acquisitionIndex = header.frameNumber;

    zHeader.dynamicRange = *dynamicRange;
    zHeader.fileIndex = *fileIndex;
    zHeader.ndetx = numMods[0];
    zHeader.ndety = numMods[1];
    zHeader.npixelsx = nx;
    zHeader.npixelsy = ny;
    zHeader.imageSize = size;
    zHeader.acqIndex = acquisitionIndex;
    zHeader.frameIndex = frameIndex;
    zHeader.progress =
        100 * ((double)(frameIndex + 1) / (double)(*totalNumFrames));
    zHeader.fname = fileNametoStream;
    zHeader.frameNumber = header.frameNumber;
    zHeader.expLength = header.expLength;
    zHeader.packetNumber = header.packetNumber;
    zHeader.detSpec1 = header.detSpec1;
    zHeader.timestamp = header.timestamp;
    zHeader.modId = header.modId;
    zHeader.row = header.row;
    zHeader.column = header.column;
    zHeader.detSpec2 = header.detSpec2;
    zHeader.detSpec3 = header.detSpec3;
    zHeader.detSpec4 = header.detSpec4;
    zHeader.detType = header.detType;
    zHeader.version = header.version;
    zHeader.flipRows = static_cast<int>(flipRows);
    zHeader.quad = *quadEnable;
    zHeader.completeImage =
        (header.packetNumber < generalData->packetsPerFrame ? false : true);

    // update local copy only if it was updated (to prevent locking each time)
    if (isAdditionalJsonUpdated) {
        std::lock_guard<std::mutex> lock(additionalJsonMutex);
        localAdditionalJsonHeader = additionalJsonHeader;
        isAdditionalJsonUpdated = false;
    }
    zHeader.addJsonHeader = localAdditionalJsonHeader;

    return zmqSocket->SendHeader(index, zHeader);
}

void DataStreamer::RestreamStop() {
    // send dummy header
    sls::zmqHeader zHeader;
    zHeader.data = false;
    zHeader.jsonversion = SLS_DETECTOR_JSON_HEADER_VERSION;
    int ret = zmqSocket->SendHeader(index, zHeader);
    if (!ret) {
        throw sls::RuntimeError(
            "Could not restream Dummy Header via ZMQ for port " +
            std::to_string(zmqSocket->GetPortNumber()));
    }
}
