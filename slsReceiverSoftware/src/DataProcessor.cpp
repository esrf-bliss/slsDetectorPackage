// SPDX-License-Identifier: LGPL-3.0-or-other
// Copyright (C) 2021 Contributors to the SLS Detector Package
/************************************************
 * @file DataProcessor.cpp
 * @short creates data processor thread that
 * pulls pointers to memory addresses from fifos
 * and processes data stored in them & writes them to file
 ***********************************************/

#include "DataProcessor.h"
#include "BinaryDataFile.h"
#include "BinaryMasterFile.h"
#include "Fifo.h"
#include "GeneralData.h"
#include "MasterAttributes.h"
#ifdef HDF5C
#include "HDF5DataFile.h"
#include "HDF5MasterFile.h"
#include "HDF5VirtualFile.h"
#endif
#include "DataStreamer.h"
#include "sls/container_utils.h"
#include "sls/sls_detector_exceptions.h"

#include <cerrno>
#include <cstring>
#include <iostream>

const std::string DataProcessor::typeName_ = "DataProcessor";

DataProcessor::DataProcessor(int index, detectorType detectorType, Fifo *fifo,
                             uint64_t *nimages, uint32_t *framesperfile,
                             bool *dataStreamEnable,
                             uint32_t *streamingFrequency,
                             uint32_t *streamingTimerInMs,
                             uint32_t *streamingStartFnum, bool *framePadding,
                             bool *silentMode, std::vector<int> *ctbDbitList,
                             int *ctbDbitOffset, int *ctbAnalogDataBytes,
                             std::mutex *hdf5Lib)
    : ThreadObject(index, typeName_), fifo_(fifo), detectorType_(detectorType),
      numImages_(nimages), streamingFrequency_(streamingFrequency),
      streamingTimerInMs_(streamingTimerInMs),
      streamingStartFnum_(streamingStartFnum), framePadding_(framePadding),
      silentMode_(silentMode), ctbDbitList_(ctbDbitList),
      ctbDbitOffset_(ctbDbitOffset), ctbAnalogDataBytes_(ctbAnalogDataBytes),
      firstStreamerFrame_(false), hdf5Lib_(hdf5Lib),
      framesPerFile_(framesperfile) {

    LOG(sls::logDEBUG) << "DataProcessor " << index << " created";

    memset((void *)&timerbegin_, 0, sizeof(timespec));
}

DataProcessor::~DataProcessor() { DeleteFiles(); }

/** getters */

bool DataProcessor::GetStartedFlag() { return startedFlag_; }

uint64_t DataProcessor::GetCurrentFrameIndex() { return currentFrameIndex_; }

uint64_t DataProcessor::GetProcessedIndex() {
    return currentFrameIndex_ - firstIndex_;
}

void DataProcessor::SetFifo(Fifo *fifo) { fifo_ = fifo; }

void DataProcessor::SetHardCodedPosition(uint16_t r, uint16_t c) {
    row_ = r;
    column_ = c;
}

void DataProcessor::ResetParametersforNewAcquisition() {
    StopRunning();
    startedFlag_ = false;
    numFramesCaught_ = 0;
    firstIndex_ = 0;
    currentFrameIndex_ = 0;
    firstStreamerFrame_ = true;
    numPacketsStatistic_ = 0;
    numFramesStatistic_ = 0;
    // reset fifo statistic
    fifo_->GetMaxLevelForFifoStream();
    fifo_->GetMinLevelForFifoFree();
}

void DataProcessor::RecordFirstIndex(uint64_t fnum) {
    // listen to this fnum, later +1
    currentFrameIndex_ = fnum;

    startedFlag_ = true;
    firstIndex_ = fnum;

    LOG(sls::logDEBUG1) << index << " First Index:" << firstIndex_;
}

void DataProcessor::SetGeneralData(GeneralData *generalData) {
    generalData_ = generalData;

    try {
        frameAssembler_ = sls::FrameAssembler::CreateDefaultFrameAssembler(
            generalData_->myDetectorType, generalData_->tgEnable,
            generalData_->numUDPInterfaces, generalData_->dynamicRange);
        LOG(sls::logINFO) << index << ": Default FrameAssembler created";
    } catch (...) {
        throw sls::RuntimeError("Could not create FrameAssembler #" +
                                std::to_string(index));
    }
}

void DataProcessor::CloseFiles() {
    if (dataFile_)
        dataFile_->CloseFile();
    if (masterFile_)
        masterFile_->CloseFile();
#ifdef HDF5C
    if (virtualFile_)
        virtualFile_->CloseFile();
#endif
}

void DataProcessor::DeleteFiles() {
    CloseFiles();
    if (dataFile_) {
        delete dataFile_;
        dataFile_ = nullptr;
    }
    if (masterFile_) {
        delete masterFile_;
        masterFile_ = nullptr;
    }
#ifdef HDF5C
    if (virtualFile_) {
        delete virtualFile_;
        virtualFile_ = nullptr;
    }
#endif
}
void DataProcessor::SetupFileWriter(const bool filewriteEnable,
                                    const bool masterFilewriteEnable,
                                    const fileFormat fileFormatType,
                                    const int modulePos) {
    DeleteFiles();
    if (filewriteEnable) {
        switch (fileFormatType) {
#ifdef HDF5C
        case HDF5:
            dataFile_ = new HDF5DataFile(index, hdf5Lib_);
            if (modulePos == 0 && index == 0) {
                if (masterFilewriteEnable) {
                    masterFile_ = new HDF5MasterFile(hdf5Lib_);
                }
            }
            break;
#endif
        case BINARY:
            dataFile_ = new BinaryDataFile(index);
            if (modulePos == 0 && index == 0 && masterFilewriteEnable) {
                masterFile_ = new BinaryMasterFile();
            }
            break;
        default:
            throw sls::RuntimeError(
                "Unknown file format (compile with hdf5 flags");
        }
    }
}

void DataProcessor::CreateFirstFiles(
    MasterAttributes *attr, const std::string filePath,
    const std::string fileNamePrefix, const uint64_t fileIndex,
    const bool overWriteEnable, const bool silentMode, const int modulePos,
    const int numUnitsPerReadout, const uint32_t udpPortNumber,
    const uint32_t maxFramesPerFile, const uint64_t numImages,
    const uint32_t dynamicRange) {
    if (dataFile_ == nullptr) {
        throw sls::RuntimeError("file object not contstructed");
    }
    CloseFiles();

    // master file write enabled
    if (masterFile_) {
        masterFile_->CreateMasterFile(filePath, fileNamePrefix, fileIndex,
                                      overWriteEnable, silentMode, attr);
    }

    switch (dataFile_->GetFileFormat()) {
#ifdef HDF5C
    case HDF5:
        dataFile_->CreateFirstHDF5DataFile(
            filePath, fileNamePrefix, fileIndex, overWriteEnable, silentMode,
            modulePos, numUnitsPerReadout, udpPortNumber, maxFramesPerFile,
            numImages, generalData_->nPixelsX, generalData_->nPixelsY,
            dynamicRange);
        break;
#endif
    case BINARY:
        dataFile_->CreateFirstBinaryDataFile(
            filePath, fileNamePrefix, fileIndex, overWriteEnable, silentMode,
            modulePos, numUnitsPerReadout, udpPortNumber, maxFramesPerFile);
        break;
    default:
        throw sls::RuntimeError("Unknown file format (compile with hdf5 flags");
    }
}

#ifdef HDF5C
uint32_t DataProcessor::GetFilesInAcquisition() const {
    if (dataFile_ == nullptr) {
        throw sls::RuntimeError("No data file object created to get number of "
                                "files in acquiistion");
    }
    return dataFile_->GetFilesInAcquisition();
}

void DataProcessor::CreateVirtualFile(
    const std::string filePath, const std::string fileNamePrefix,
    const uint64_t fileIndex, const bool overWriteEnable, const bool silentMode,
    const int modulePos, const int numUnitsPerReadout,
    const uint32_t maxFramesPerFile, const uint64_t numImages,
    const uint32_t dynamicRange, const int numModX, const int numModY) {

    if (virtualFile_) {
        delete virtualFile_;
    }
    virtualFile_ = new HDF5VirtualFile(hdf5Lib_);

    uint64_t numImagesProcessed = GetProcessedIndex() + 1;
    // maxframesperfile = 0 for infinite files
    uint32_t framesPerFile =
        ((maxFramesPerFile == 0) ? numImagesProcessed + 1 : maxFramesPerFile);

    // TODO: assumption 1: create virtual file even if no data in other
    // files (they exist anyway) assumption2: virtual file max frame index
    // is from R0 P0 (difference from others when missing frames or for a
    // stop acquisition)
    virtualFile_->CreateVirtualFile(
        filePath, fileNamePrefix, fileIndex, overWriteEnable, silentMode,
        modulePos, numUnitsPerReadout, framesPerFile, numImages,
        generalData_->nPixelsX, generalData_->nPixelsY, dynamicRange,
        numImagesProcessed, numModX, numModY, dataFile_->GetPDataType(),
        dataFile_->GetParameterNames(), dataFile_->GetParameterDataTypes());
}

void DataProcessor::LinkDataInMasterFile(const bool silentMode) {
    std::string fname, datasetName;
    if (virtualFile_) {
        auto res = virtualFile_->GetFileAndDatasetName();
        fname = res[0];
        datasetName = res[1];
    } else {
        auto res = dataFile_->GetFileAndDatasetName();
        fname = res[0];
        datasetName = res[1];
    }
    // link in master
    masterFile_->LinkDataFile(fname, datasetName,
                              dataFile_->GetParameterNames(), silentMode);
}
#endif

void DataProcessor::UpdateMasterFile(bool silentMode) {
    if (masterFile_) {
        // final attributes
        std::unique_ptr<MasterAttributes> masterAttributes;
        switch (detectorType_) {
        case GOTTHARD:
            masterAttributes = sls::make_unique<GotthardMasterAttributes>();
            break;
        case JUNGFRAU:
            masterAttributes = sls::make_unique<JungfrauMasterAttributes>();
            break;
        case EIGER:
            masterAttributes = sls::make_unique<EigerMasterAttributes>();
            break;
        case MYTHEN3:
            masterAttributes = sls::make_unique<Mythen3MasterAttributes>();
            break;
        case GOTTHARD2:
            masterAttributes = sls::make_unique<Gotthard2MasterAttributes>();
            break;
        case MOENCH:
            masterAttributes = sls::make_unique<MoenchMasterAttributes>();
            break;
        case CHIPTESTBOARD:
            masterAttributes = sls::make_unique<CtbMasterAttributes>();
            break;
        default:
            throw sls::RuntimeError(
                "Unknown detector type to set up master file attributes");
        }
        masterAttributes->framesInFile = numFramesCaught_;
        masterFile_->UpdateMasterFile(masterAttributes.get(), silentMode);
    }
}

void DataProcessor::ThreadExecution() {
    FifoFrame *frame;
    fifo_->GetNewFrame(frame);
    LOG(sls::logDEBUG5) << "DataProcessor " << index << ", " << std::hex << "pop 0x"
                   << (void *)frame << " "
                   << "[data: 0x" << (void *)frame->recvFrame.data << "]"
                   << std::dec;

    int rc = AssembleAnImage(frame);
    if (rc < 0) {
        StopProcessing(frame);
        return;
    } else if (rc == 0) {
        fifo_->FreeFrame(frame);
        return;
    }

    auto &numBytes = frame->recvFrame.numBytes;
    numBytes = rc;
    LOG(sls::logDEBUG1) << "DataProcessor " << index << ", Numbytes:" << numBytes;

    uint64_t fnum = 0;
    try {
        fnum = ProcessAnImage(frame);
    } catch (const std::exception &e) {
        fifo_->FreeFrame(frame);
        return;
    }
    // stream (if time/freq to stream) or free
    if (*dataStreamEnable_ && SendToStreamer()) {
        // if first frame to stream, add frame index to fifo header (might
        // not be the first)
        if (firstStreamerFrame_) {
            firstStreamerFrame_ = false;
            frame->firstStreamerFrame = (uint32_t)(fnum - firstIndex_);
        }
        fifo_->PushFrameToStream(frame);
    } else {
        fifo_->FreeFrame(frame);
    }

    // Statistics
    if (!(*silentMode_)) {
        numFramesStatistic_++;
        if (numFramesStatistic_ >=
            // second condition also for infinite #number of frames
            (((*framesPerFile_) == 0) ? STATISTIC_FRAMENUMBER_INFINITE
                                      : (*framesPerFile_)))
            PrintFifoStatistics();
    }
}

int DataProcessor::AssembleAnImage(FifoFrame *frame) {

    sls_receiver_header *recv_header = &frame->recvFrame.header;
    char *buf = frame->recvFrame.data;
    uint32_t imageSize = generalData_->imageSize;

    auto block = fifo_->GetFramePackets();
    std::visit(
        [&](auto &b) {
            if (!b)
                return;
            recv_header->packetsMask = b->getValidPacketMask();
            auto *header = b->getNetworkHeader();
            if (header)
                recv_header->detHeader = *header;
        },
        block);
    recv_header->detHeader.row = row_;
    recv_header->detHeader.column = column_;

    bool ok = frameAssembler_->assembleFrame(block, buf);
    if (!ok)
        return -1;

    // update parameters
    numPacketsStatistic_ += recv_header->detHeader.packetNumber;

    return imageSize;
}

void DataProcessor::StopProcessing(FifoFrame *frame) {
    LOG(sls::logDEBUG1) << "DataProcessing " << index << ": Dummy";
    frame->end = true;

    // stream or free
    if (*dataStreamEnable_)
        fifo_->PushFrameToStream(frame);
    else
        fifo_->FreeFrame(frame);

    CloseFiles();
    StopRunning();
    LOG(sls::logDEBUG1) << index << ": Processing Completed";
}

uint64_t DataProcessor::ProcessAnImage(FifoFrame *frame) {

    auto *rheader = &frame->recvFrame.header;
    sls_detector_header header = rheader->detHeader;
    uint64_t fnum = header.frameNumber;
    currentFrameIndex_ = fnum;
    numFramesCaught_++;
    uint32_t nump = header.packetNumber;

    LOG(sls::logDEBUG1) << "DataProcessing " << index << ": fnum:" << fnum;

    if (!startedFlag_) {
        RecordFirstIndex(fnum);
        if (*dataStreamEnable_) {
            // restart timer
            clock_gettime(CLOCK_REALTIME, &timerbegin_);
            timerbegin_.tv_sec -= (*streamingTimerInMs_) / 1000;
            timerbegin_.tv_nsec -= ((*streamingTimerInMs_) % 1000) * 1000000;

            // to send first image
            currentFreqCount_ = *streamingFrequency_ - *streamingStartFnum_;
        }
    }

    // frame padding
    if (*framePadding_ && nump < generalData_->packetsPerFrame)
        PadMissingPackets(frame);

    // rearrange ctb digital bits (if ctbDbitlist is not empty)
    if (!(*ctbDbitList_).empty())
        RearrangeDbitData(frame);

    char *buf = frame->recvFrame.data;
    auto &numBytes = frame->recvFrame.numBytes;
    try {
        // normal call back
        if (rawDataReadyCallBack != nullptr) {
            rawDataReadyCallBack((char *)rheader, buf, numBytes, pRawDataReady);
        }

        // call back with modified size
        else if (rawDataModifyReadyCallBack != nullptr) {
            rawDataModifyReadyCallBack((char *)rheader, buf, numBytes,
                                       pRawDataReady);
        }
    } catch (const std::exception &e) {
        throw sls::RuntimeError("Get Data Callback Error: " +
                                std::string(e.what()));
    }

    // write to file
    if (dataFile_) {
        try {
            // size of data (resizable from previous call back)
            dataFile_->WriteToFile(rheader, buf, numBytes, fnum - firstIndex_,
                                   nump);
        } catch (const sls::RuntimeError &e) {
            ; // ignore write exception for now (TODO: send error message
              // via stopReceiver tcp)
        }
    }
    return fnum;
}

bool DataProcessor::SendToStreamer() {
    // skip
    if ((*streamingFrequency_) == 0u) {
        if (!CheckTimer())
            return false;
    } else {
        if (!CheckCount())
            return false;
    }
    return true;
}

bool DataProcessor::CheckTimer() {
    struct timespec end;
    clock_gettime(CLOCK_REALTIME, &end);

    LOG(sls::logDEBUG1) << index << " Timer elapsed time:"
                   << ((end.tv_sec - timerbegin_.tv_sec) +
                       (end.tv_nsec - timerbegin_.tv_nsec) / 1000000000.0)
                   << " seconds";
    // still less than streaming timer, keep waiting
    if (((end.tv_sec - timerbegin_.tv_sec) +
         (end.tv_nsec - timerbegin_.tv_nsec) / 1000000000.0) <
        ((double)*streamingTimerInMs_ / 1000.00))
        return false;

    // restart timer
    clock_gettime(CLOCK_REALTIME, &timerbegin_);
    return true;
}

bool DataProcessor::CheckCount() {
    if (currentFreqCount_ == *streamingFrequency_) {
        currentFreqCount_ = 1;
        return true;
    }
    currentFreqCount_++;
    return false;
}

void DataProcessor::registerCallBackRawDataReady(void (*func)(char *, char *,
                                                              uint32_t, void *),
                                                 void *arg) {
    rawDataReadyCallBack = func;
    pRawDataReady = arg;
}

void DataProcessor::registerCallBackRawDataModifyReady(
    void (*func)(char *, char *, uint32_t &, void *), void *arg) {
    rawDataModifyReadyCallBack = func;
    pRawDataReady = arg;
}

void DataProcessor::PadMissingPackets(FifoFrame *frame) {
    LOG(sls::logDEBUG) << index << ": Padding Missing Packets";

    uint32_t pperFrame = generalData_->packetsPerFrame;
    auto *header = &frame->recvFrame.header;
    uint32_t nmissing = pperFrame - header->detHeader.packetNumber;
    sls_bitset pmask = header->packetsMask;
    LOG(sls::logDEBUG1) << "bitmask: " << pmask.to_string();

    uint32_t dsize = generalData_->dataSize;
    if (detectorType_ == GOTTHARD2 && index != 0) {
        dsize = generalData_->vetoDataSize;
    }
    uint32_t corrected_dsize =
        dsize - ((pperFrame * dsize) - generalData_->imageSize);

    char *buf = frame->recvFrame.data;
    for (unsigned int pnum = 0; pnum < pperFrame; ++pnum) {

        // not missing packet
        if (pmask[pnum])
            continue;

        // done with padding, exit loop earlier
        if (nmissing == 0u)
            break;

        LOG(sls::logDEBUG) << "padding for " << index << " for pnum: " << pnum
                      << std::endl;

        // missing packet
        switch (detectorType_) {
        // for gotthard, 1st packet: 4 bytes fnum, CACA + CACA, 639*2 bytes
        // data
        //              2nd packet: 4 bytes fnum, previous 1*2 bytes data  +
        //              640*2 bytes data !!
        case GOTTHARD:
            if (pnum == 0u)
                memset(buf + (pnum * dsize), 0xFF, dsize - 2);
            else
                memset(buf + (pnum * dsize), 0xFF, dsize + 2);
            break;
        case CHIPTESTBOARD:
        case MOENCH:
            if (pnum == (pperFrame - 1))
                memset(buf + (pnum * dsize), 0xFF, corrected_dsize);
            else
                memset(buf + (pnum * dsize), 0xFF, dsize);
            break;
        default:
            memset(buf + (pnum * dsize), 0xFF, dsize);
            break;
        }
        --nmissing;
    }
}

/** ctb specific */
void DataProcessor::RearrangeDbitData(FifoFrame *frame) {
    char *buf = frame->recvFrame.data;
    // TODO! (Erik) Refactor and add tests
    auto &totalSize = frame->recvFrame.numBytes;
    int ctbDigitalDataBytes =
        totalSize - (*ctbAnalogDataBytes_) - (*ctbDbitOffset_);

    // no digital data
    if (ctbDigitalDataBytes == 0) {
        LOG(sls::logWARNING)
            << "No digital data for call back, yet dbitlist is not empty.";
        return;
    }

    const int numSamples = (ctbDigitalDataBytes / sizeof(uint64_t));
    const int digOffset = *ctbAnalogDataBytes_;

    // ceil as numResult8Bits could be decimal
    const int numResult8Bits =
        ceil((double)(numSamples * (*ctbDbitList_).size()) / 8.00);
    std::vector<uint8_t> result(numResult8Bits);
    uint8_t *dest = &result[0];

    auto *source = (uint64_t *)(buf + digOffset + (*ctbDbitOffset_));

    // loop through digital bit enable vector
    int bitoffset = 0;
    for (auto bi : (*ctbDbitList_)) {
        // where numbits * numsamples is not a multiple of 8
        if (bitoffset != 0) {
            bitoffset = 0;
            ++dest;
        }

        // loop through the frame digital data
        for (auto ptr = source; ptr < (source + numSamples);) {
            // get selected bit from each 8 bit
            uint8_t bit = (*ptr++ >> bi) & 1;
            *dest |= bit << bitoffset;
            ++bitoffset;
            // extract destination in 8 bit batches
            if (bitoffset == 8) {
                bitoffset = 0;
                ++dest;
            }
        }
    }

    // copy back to buf and update size
    memcpy(buf + digOffset, result.data(), numResult8Bits * sizeof(uint8_t));
    totalSize = numResult8Bits * sizeof(uint8_t);
}

// TODO: Include packet fifo statistics
void DataProcessor::PrintFifoStatistics() {
    LOG(sls::logDEBUG1) << "numFramesStatistic:" << numFramesStatistic_
                   << " numPacketsStatistic:" << numPacketsStatistic_
                   << " packetsperframe:" << generalData_->packetsPerFrame;

    // calculate packet loss
    int64_t totalP = numFramesStatistic_ * (generalData_->packetsPerFrame);
    int64_t loss = totalP - numPacketsStatistic_;
    int lossPercent = ((double)loss / (double)totalP) * 100.00;
    numPacketsStatistic_ = 0;
    numFramesStatistic_ = 0;

    const auto color = loss ? sls::logINFORED : sls::logINFOGREEN;
    LOG(color) << "DataProcessor " << index << ":  Packet_Loss:" << loss << " ("
               << lossPercent << "%)"
               << "  Used_Fifo_Max_Level:" << fifo_->GetMaxLevelForFifoStream()
               << " \tFree_Slots_Min_Level:" << fifo_->GetMinLevelForFifoFree()
               << " \tCurrent_Frame#:" << currentFrameIndex_;
}
