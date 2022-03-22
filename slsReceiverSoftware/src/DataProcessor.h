// SPDX-License-Identifier: LGPL-3.0-or-other
// Copyright (C) 2021 Contributors to the SLS Detector Package
#pragma once
/************************************************
 * @file DataProcessor.h
 * @short creates data processor thread that
 * pulls pointers to memory addresses from fifos
 * and processes data stored in them & writes them to file
 ***********************************************/
/**
 *@short creates & manages a data processor thread each
 */

#include "ThreadObject.h"
#include "receiver_defs.h"
#include "sls/FrameAssembler.h"

class GeneralData;
class Fifo;
class File;
class DataStreamer;
struct MasterAttributes;

#include <atomic>
#include <mutex>
#include <vector>

class DataProcessor : private virtual slsDetectorDefs, public ThreadObject {

  public:
    using DefaultFrameAssemblerPtr =
        sls::FrameAssembler::DefaultFrameAssemblerPtr;

    DataProcessor(int index, detectorType detectorType, Fifo *fifo,
                  uint64_t *nimages, uint32_t *framesperfile,
                  bool *dataStreamEnable, uint32_t *streamingFrequency,
                  uint32_t *streamingTimerInMs, uint32_t *streamingStartFnum,
                  bool *framePadding, bool *silentMode,
                  std::vector<int> *ctbDbitList, int *ctbDbitOffset,
                  int *ctbAnalogDataBytes);

    ~DataProcessor() override;

    bool GetStartedFlag();
    /** (-1 if no frames have been caught */
    uint64_t GetCurrentFrameIndex();
    /** (-1 if no frames have been caught) */
    uint64_t GetProcessedIndex();

    void SetFifo(Fifo *f);

    /**
     * Set hard coded (calculated but not from detector) row and column
     * r is in row index if detector has not send them yet in firmware,
     * c is in col index for jungfrau and eiger (for missing packets/deactivated
     * eiger) c when used is in 2d
     */
    void SetHardCodedPosition(uint16_t r, uint16_t c);

    void ResetParametersforNewAcquisition();
    void SetGeneralData(GeneralData *generalData);

    void CloseFiles();
    void DeleteFiles();
    void SetupFileWriter(const bool filewriteEnable,
                         const bool masterFilewriteEnable,
                         const fileFormat fileFormatType, const int modulePos,
                         std::mutex *hdf5Lib);

    void CreateFirstFiles(MasterAttributes *attr, const std::string filePath,
                          const std::string fileNamePrefix,
                          const uint64_t fileIndex, const bool overWriteEnable,
                          const bool silentMode, const int modulePos,
                          const int numUnitsPerReadout,
                          const uint32_t udpPortNumber,
                          const uint32_t maxFramesPerFile,
                          const uint64_t numImages,
                          const uint32_t dynamicRange);
#ifdef HDF5C
    uint32_t GetFilesInAcquisition() const;
    void CreateVirtualFile(const std::string filePath,
                           const std::string fileNamePrefix,
                           const uint64_t fileIndex, const bool overWriteEnable,
                           const bool silentMode, const int modulePos,
                           const int numUnitsPerReadout,
                           const uint32_t maxFramesPerFile,
                           const uint64_t numImages,
                           const uint32_t dynamicRange, const int numModX,
                           const int numModY, std::mutex *hdf5Lib);
    void LinkDataInMasterFile(const bool silentMode);
#endif
    void UpdateMasterFile(bool silentMode);
    /**
     * Call back for raw data
     * args to raw data ready callback are
     * sls_receiver_header frame metadata
     * dataPointer is the pointer to the data
     * dataSize in bytes is the size of the data in bytes.
     */
    void registerCallBackRawDataReady(void (*func)(char *, char *, uint32_t,
                                                   void *),
                                      void *arg);

    /**
     * Call back for raw data (modified)
     * args to raw data ready callback are
     * sls_receiver_header frame metadata
     * dataPointer is the pointer to the data
     * revDatasize is the reference of data size in bytes.
     * Can be modified to the new size to be written/streamed. (only smaller
     * value).
     */
    void registerCallBackRawDataModifyReady(void (*func)(char *, char *,
                                                         uint32_t &, void *),
                                            void *arg);

  private:
    void RecordFirstIndex(uint64_t fnum);

    /**
     * Thread Exeution for DataProcessor Class
     * Pop bound addresses, process them,
     * write to file if needed & free the address
     */
    void ThreadExecution() override;

    /**
     * Assemble an image from UDP packets
     * @param frame pointer to frame
     * @returns number of bytes of relevant data, can be image size or -1 (stop
     * acquisition) or 0 to discard image
     */
    int AssembleAnImage(FifoFrame *frame);

    /**
     * Frees dummy buffer,
     * reset running mask by calling StopRunning()
     * @param frame pointer to frame
     */
    void StopProcessing(FifoFrame *frame);

    /**
     * Process an image popped from fifo,
     * write to file if fw enabled & update parameters
     * @param frame pointer to frame
     * @returns frame number
     */
    uint64_t ProcessAnImage(FifoFrame *frame);

    /**
     * Calls CheckTimer and CheckCount for streaming frequency and timer
     * and determines if the current image should be sent to streamer
     * @returns true if it should to streamer, else false
     */
    bool SendToStreamer();

    /**
     * This function should be called only in random frequency mode
     * Checks if timer is done and ready to send to stream
     * @returns true if ready to send to stream, else false
     */
    bool CheckTimer();

    /**
     * This function should be called only in non random frequency mode
     * Checks if count is done and ready to send to stream
     * @returns true if ready to send to stream, else false
     */
    bool CheckCount();

    /**
     * Pad Missing Packets from the bit mask
     * @param frame pointer to frame
     */
    void PadMissingPackets(FifoFrame *frame);

    /**
     * Align corresponding digital bits together (CTB only if ctbDbitlist is not
     * empty)
     */
    void RearrangeDbitData(FifoFrame *frame);

    /**
     * Print Fifo Statistics
     */
    void PrintFifoStatistics();

    static const std::string typeName_;

    const GeneralData *generalData_{nullptr};
    Fifo *fifo_;
    detectorType detectorType_;
    uint64_t *numImages_;
    uint16_t row_{0};
    uint16_t column_{0};
    DefaultFrameAssemblerPtr frameAssembler_;
    bool *dataStreamEnable_;
    /** if 0, sending random images with a timer */
    uint32_t *streamingFrequency_;
    uint32_t *streamingTimerInMs_;
    uint32_t *streamingStartFnum_;
    uint32_t currentFreqCount_{0};
    struct timespec timerbegin_;
    bool *framePadding_;
    bool *silentMode_;
    std::vector<int> *ctbDbitList_;
    int *ctbDbitOffset_;
    int *ctbAnalogDataBytes_;
    std::atomic<bool> startedFlag_{false};
    std::atomic<uint64_t> firstIndex_{0};

    // for statistics
    /** Number of frames caught */
    uint64_t numFramesCaught_{0};

    /** Frame Number of latest processed frame number */
    std::atomic<uint64_t> currentFrameIndex_{0};

    /** first streamer frame to add frame index in fifo header */
    bool firstStreamerFrame_{false};

    File *dataFile_{nullptr};
    File *masterFile_{nullptr};
#ifdef HDF5C
    File *virtualFile_{nullptr};
#endif

    // for print progress during acquisition
    uint32_t *framesPerFile_;
    uint32_t numPacketsStatistic_{0};
    uint32_t numFramesStatistic_{0};

    // call back
    /**
     * Call back for raw data
     * args to raw data ready callback are
     * sls_receiver_header frame metadata
     * dataPointer is the pointer to the data
     * dataSize in bytes is the size of the data in bytes.
     */
    void (*rawDataReadyCallBack)(char *, char *, uint32_t, void *) = nullptr;

    /**
     * Call back for raw data (modified)
     * args to raw data ready callback are
     * sls_receiver_header frame metadata
     * dataPointer is the pointer to the data
     * revDatasize is the reference of data size in bytes. Can be modified to
     * the new size to be written/streamed. (only smaller value).
     */
    void (*rawDataModifyReadyCallBack)(char *, char *, uint32_t &,
                                       void *) = nullptr;

    void *pRawDataReady{nullptr};
};
