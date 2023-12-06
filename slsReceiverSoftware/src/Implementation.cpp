// SPDX-License-Identifier: LGPL-3.0-or-other
// Copyright (C) 2021 Contributors to the SLS Detector Package
#include "Implementation.h"
#include "DataProcessor.h"
#include "DataStreamer.h"
#include "Fifo.h"
#include "GeneralData.h"
#include "Listener.h"
#include "MasterAttributes.h"
#include "sls/ToString.h"
#include "sls/ZmqSocket.h" //just for the zmq port define
#include "sls/detectors/eiger/FrameAssembler.h"
#include "sls/detectors/jungfrau/FrameAssembler.h"
#include "sls/file_utils.h"

#include <cerrno> //eperm
#include <chrono>
#include <cstdlib> //system
#include <cstring>
#include <cstring> //strcpy
#include <fstream>
#include <iostream>
#include <sys/stat.h> // stat
#include <thread>
#include <unistd.h>

/** cosntructor & destructor */

Implementation::Implementation(const detectorType d, bool passive)
    : passiveMode(passive) {
    setDetectorType(d);
}

Implementation::~Implementation() {
    delete generalData;
    generalData = nullptr;
}

void Implementation::SetLocalNetworkParameters() {
    // to increase Max length of input packet queue
    int max_back_log;
    const char *proc_file_name = "/proc/sys/net/core/netdev_max_backlog";
    {
        std::ifstream proc_file(proc_file_name);
        proc_file >> max_back_log;
    }

    if (max_back_log < MAX_SOCKET_INPUT_PACKET_QUEUE) {
        std::ofstream proc_file(proc_file_name);
        if (proc_file.good()) {
            proc_file << MAX_SOCKET_INPUT_PACKET_QUEUE << std::endl;
            LOG(logINFOBLUE)
                << "Max length of input packet queue "
                   "[/proc/sys/net/core/netdev_max_backlog] modified to "
                << MAX_SOCKET_INPUT_PACKET_QUEUE;
        } else {
            LOG(logWARNING)
                << "Could not change max length of "
                   "input packet queue [net.core.netdev_max_backlog]. (No Root "
                   "Privileges?)";
        }
    }
}

void Implementation::SetThreadPriorities() {
    for (const auto &it : listener)
        if (IsValidThread(it))
            it->SetThreadPriority(LISTENER_PRIORITY);
}

void Implementation::SetupFifoStructure() {
    fifo.clear();
    frameAssembler.reset();
    for (int i = 0; i < numThreads; ++i) {
        // create fifo structure
        NUMAMask numa_mask;
        try {
            PacketBlockAllocPtr alloc_ptr;
            if (HasValidThread(packetAllocPtr, i)) {
                alloc_ptr = packetAllocPtr[i];
            } else {
                if (HasValidThread(numaMask, i))
                    numa_mask = *numaMask[i];
                alloc_ptr = std::make_shared<MmappedPacketAllocator>(numa_mask);
            }
            fifo.push_back(
                sls::make_unique<Fifo>(i, generalData, fifoDepth, alloc_ptr));
        } catch (...) {
            fifo.clear();
            fifoDepth = 0;
            throw sls::RuntimeError(
                "Could not allocate memory for fifo structure " +
                std::to_string(i) + ". FifoDepth is now 0.");
        }
        // set the listener & dataprocessor threads to point to the right fifo
        Fifo *f = fifo[i].get();
        if (HasValidThread(listener, i))
            listener[i]->SetFifo(f);
        if (HasValidThread(dataProcessor, i))
            dataProcessor[i]->SetFifo(f);
        if (HasValidThread(dataStreamer, i))
            dataStreamer[i]->SetFifo(f);

        size_t framesize = f->GetFifoFrameSize();
        std::string numa_str;
        if (numa_mask.count() > 0) {
            std::ostringstream os;
            os << " - NUMA mask: " << numa_mask;
            numa_str = os.str();
        }
        LOG(logINFO) << "Memory Allocated for Fifo " << i << ": "
                     << (double)(framesize * fifoDepth) / (double)(1024 * 1024)
                     << " MB" << numa_str;
    }
    LOG(logINFO) << numThreads << " Fifo structure(s) reconstructed";
}

MPFrameAssemblerPtr
Implementation::CreateFrameAssembler(AssemblerType asm_type) {
    MPFrameAssemblerPtr fa;
    GeneralDataPtr gd = generalData;
    detectorType d = detType;
    bool tg_enable = tengigaEnable;
    int nb_ports = listener.size();
    int recv_idx = modulePos;
    uint32_t src_dr = gd->dynamicRange;
    using XY = sls::Geom::XY;
    XY det_ifaces{numMods[0], numMods[1]};
    auto port_geom = GetPortGeometry();
    XY recv_ifaces{port_geom[X], port_geom[Y]};
    XY mod_recvs;
    if (d == slsDetectorDefs::EIGER)
        mod_recvs = sls::Eiger::Geom::ModRecvs;
    else if (d == slsDetectorDefs::JUNGFRAU)
        mod_recvs = sls::Jungfrau::Geom::ModRecvs;
    else
        throw sls::RuntimeError("FrameAssembler not available for " +
                                sls::ToString(d));
    int recvs_per_mod = mod_recvs.area();
    int mod_idx = recv_idx / recvs_per_mod;
    int mod_recv_idx = recv_idx % recvs_per_mod;
    XY det_mods = det_ifaces / (mod_recvs * recv_ifaces);
    XY mod_pos = ColWiseElementFromIndex(det_mods, mod_idx);

    if (asm_type == AsmRaw) {
        int det_recvs = det_ifaces.area() / recv_ifaces.area();
        uint32_t dst_dr = (src_dr == 4) ? 8 : src_dr;
        fa = std::make_unique<RawFrameAssembler>(
            d, recv_idx, det_recvs, tg_enable, nb_ports, src_dr, dst_dr);
    } else if (d == slsDetectorDefs::EIGER) {
        fa = sls::Eiger::FrameAssembler::CreateFrameAssembler(
            src_dr, gd->tgEnable, det_ifaces, mod_pos, mod_recv_idx);
    } else if (d == slsDetectorDefs::JUNGFRAU) {
        fa = sls::Jungfrau::FrameAssembler::CreateFrameAssembler(
            nb_ports, det_ifaces, mod_pos);
    }

    return fa;
}

/**************************************************
 *                                                 *
 *   Threads
 *                                                 *
 * ************************************************/

void Implementation::CreateThreads() {
    for (int i = 0; i < numThreads; ++i) {
        auto fifo_ptr = fifo[i].get();
        try {
            if (!(activated && detectorDataStream[i])) {
                listener.push_back(nullptr);
                dataProcessor.push_back(nullptr);
                dataStreamer.push_back(nullptr);
                continue;
            }

            // listener threads
            listener.push_back(std::make_shared<Listener>(
                i, detType, fifo_ptr, &status, &udpPortNum[i], &eth[i],
                &udpSocketBufferSize, &actualUDPSocketBufferSize,
                &frameDiscardMode, &silentMode, rrNbRecvs, rrRecvIdx));
            listener[i]->SetGeneralData(generalData);

            if (passiveMode) {
                dataProcessor.push_back(nullptr);
                dataStreamer.push_back(nullptr);
                continue;
            }

            // dataprocessor threads
            dataProcessor.push_back(sls::make_unique<DataProcessor>(
                i, detType, fifo_ptr, &numberOfTotalFrames, &framesPerFile,
                &dataStreamEnable, &streamingFrequency, &streamingTimerInMs,
                &streamingStartFnum, &framePadding, &silentMode, &ctbDbitList,
                &ctbDbitOffset, &ctbAnalogDataBytes, &hdf5Lib));
            dataProcessor[i]->SetGeneralData(generalData);

            if (!dataStreamEnable) {
                dataStreamer.push_back(nullptr);
                continue;
            }

            // streamer threads
            bool flip = flipRows;
            int nm[2] = {numMods[0], numMods[1]};
            if (quadEnable) {
                flip = (i == 1 ? true : false);
                nm[0] = 1;
                nm[1] = 2;
            }
            dataStreamer.push_back(sls::make_unique<DataStreamer>(
                i, fifo[i].get(), &dynamicRange, &roi, &fileIndex, flip,
                (int *)nm, &quadEnable, &numberOfTotalFrames));
            dataStreamer[i]->SetGeneralData(generalData);
            dataStreamer[i]->CreateZmqSockets(&numThreads, streamingPort,
                                              streamingSrcIP, streamingHwm);
            dataStreamer[i]->SetAdditionalJsonHeader(additionalJsonHeader);
        } catch (...) {
            DestroyThreads();
            dataStreamEnable = false;
            throw sls::RuntimeError(
                "Could not create listener/dataprocessor/streamer threads "
                "(index:" +
                std::to_string(i) + ")");
        }
    }

    listenerStatistics.resize(numThreads);

    SetThreadPriorities();
}

void Implementation::DestroyThreads() {
    listener.clear();
    dataProcessor.clear();
    dataStreamer.clear();
}

/**************************************************
 *                                                 *
 *   Configuration Parameters                      *
 *                                                 *
 * ************************************************/

void Implementation::setDetectorType(const detectorType d) {
    detType = d;
    switch (detType) {
    case GOTTHARD:
    case EIGER:
    case JUNGFRAU:
    case CHIPTESTBOARD:
    case MOENCH:
    case MYTHEN3:
    case GOTTHARD2:
        LOG(logINFO) << " ***** " << sls::ToString(d) << " Receiver *****";
        break;
    default:
        throw sls::RuntimeError("This is an unknown receiver type " +
                                std::to_string(static_cast<int>(d)));
    }

    delete generalData;
    generalData = nullptr;

    // set detector specific variables
    switch (detType) {
    case GOTTHARD:
        generalData = new GotthardData();
        break;
    case EIGER:
        generalData = new EigerData();
        break;
    case JUNGFRAU:
        generalData = new JungfrauData();
        break;
    case CHIPTESTBOARD:
        generalData = new ChipTestBoardData();
        break;
    case MOENCH:
        generalData = new MoenchData();
        break;
    case MYTHEN3:
        generalData = new Mythen3Data();
        break;
    case GOTTHARD2:
        generalData = new Gotthard2Data();
        break;
    default:
        break;
    }
    int n = generalData->numUDPInterfaces;
    if (n > MAX_NUM_PORTS)
        throw sls::RuntimeError("Invalid numUDPInterfaces: " +
                                std::to_string(n));
    numUDPInterfaces = n;
    numThreads = n;
    fifoDepth = generalData->defaultFifoDepth;
    udpSocketBufferSize = generalData->defaultUdpSocketBufferSize;
    framesPerFile = generalData->maxFramesPerFile;

    SetLocalNetworkParameters();
    SetupFifoStructure();
    CreateThreads();

    LOG(logDEBUG) << " Detector type set to " << sls::ToString(d);
}

Implementation::PortGeometry Implementation::GetPortGeometry() {
    PortGeometry port_geom{{1, 1}};
    if (detType == EIGER)
        port_geom[X] = numUDPInterfaces;
    else if (detType == JUNGFRAU)
        port_geom[Y] = numUDPInterfaces;
    return port_geom;
}

int *Implementation::getDetectorSize() const { return (int *)numMods; }

void Implementation::setDetectorSize(const int *size) {
    PortGeometry port_geom = GetPortGeometry();
    std::string log_message = "Detector Size (ports): (";
    for (int i = 0; i < MAX_DIMENSIONS; ++i) {
        numMods[i] = size[i] * port_geom[i];
        log_message += std::to_string(numMods[i]);
        if (i < MAX_DIMENSIONS - 1)
            log_message += ", ";
    }
    log_message += ")";

    int nm[2] = {numMods[0], numMods[1]};
    if (quadEnable) {
        nm[0] = 1;
        nm[1] = 2;
    }
    for (const auto &it : dataStreamer)
        if (IsValidThread(it))
            it->SetNumberofModules(nm);

    LOG(logINFO) << log_message;
}

int Implementation::getModulePositionId() const { return modulePos; }

void Implementation::setModulePositionId(const int id) {
    modulePos = id;
    LOG(logINFO) << "Module Position Id:" << modulePos;

    // update zmq port
    PortGeometry port_geom = GetPortGeometry();
    streamingPort = DEFAULT_ZMQ_RX_PORTNO + modulePos * port_geom[X];

    for (const auto &it : dataProcessor)
        if (IsValidThread(it))
            it->SetupFileWriter(fileWriteEnable, masterFileWriteEnable,
                                fileFormatType, modulePos);
    assert(numMods[1] != 0);
    for (int i = 0; i < numThreads; ++i) {
        uint16_t row = 0, col = 0;
        row = (modulePos % numMods[1]) * port_geom[Y];     // row
        col = (modulePos / numMods[1]) * port_geom[X] + i; // col
        if (HasValidThread(dataProcessor, i))
            dataProcessor[i]->SetHardCodedPosition(row, col);
    }
}

std::string Implementation::getDetectorHostname() const { return detHostname; }

void Implementation::setDetectorHostname(const std::string &c) {
    if (!c.empty())
        detHostname = c;
    LOG(logINFO) << "Detector Hostname: " << detHostname;
}

bool Implementation::getSilentMode() const { return silentMode; }

void Implementation::setSilentMode(const bool i) {
    silentMode = i;
    LOG(logINFO) << "Silent Mode: " << i;
}

uint32_t Implementation::getFifoDepth() const { return fifoDepth; }

void Implementation::setFifoDepth(const uint32_t i) {
    if (fifoDepth != i) {
        fifoDepth = i;
        SetupFifoStructure();
    }
    LOG(logINFO) << "Fifo Depth: " << i;
}

slsDetectorDefs::frameDiscardPolicy
Implementation::getFrameDiscardPolicy() const {
    return frameDiscardMode;
}

void Implementation::setFrameDiscardPolicy(const frameDiscardPolicy i) {
    frameDiscardMode = i;
    LOG(logINFO) << "Frame Discard Policy: " << sls::ToString(frameDiscardMode);
}

bool Implementation::getFramePaddingEnable() const { return framePadding; }

void Implementation::setFramePaddingEnable(const bool i) {
    framePadding = i;
    LOG(logINFO) << "Frame Padding: " << framePadding;
}

void Implementation::setThreadIds(const pid_t parentTid, const pid_t tcpTid) {
    parentThreadId = parentTid;
    tcpThreadId = tcpTid;
}

std::array<pid_t, NUM_RX_THREAD_IDS> Implementation::getThreadIds() const {
    std::array<pid_t, NUM_RX_THREAD_IDS> retval{};
    int id = 0;
    retval[id++] = parentThreadId;
    retval[id++] = tcpThreadId;
    retval[id++] = HasValidThread(listener, 0) ? listener[0]->GetThreadId() : 0;
    retval[id++] =
        HasValidThread(dataProcessor, 0) ? dataProcessor[0]->GetThreadId() : 0;
    retval[id++] =
        HasValidThread(dataStreamer, 0) ? dataStreamer[0]->GetThreadId() : 0;
    if (numThreads == 2) {
        retval[id++] =
            HasValidThread(listener, 1) ? listener[1]->GetThreadId() : 0;
        retval[id++] = HasValidThread(dataProcessor, 1)
                           ? dataProcessor[1]->GetThreadId()
                           : 0;
        retval[id++] = HasValidThread(dataStreamer, 1)
                           ? dataStreamer[1]->GetThreadId()
                           : 0;
    }
    return retval;
}

/**************************************************
 *                                                 *
 *   File Parameters                               *
 *                                                 *
 * ************************************************/
slsDetectorDefs::fileFormat Implementation::getFileFormat() const {
    return fileFormatType;
}

void Implementation::setFileFormat(const fileFormat f) {
    if (f != fileFormatType) {
        switch (f) {
#ifdef HDF5C
        case HDF5:
            fileFormatType = HDF5;
            break;
#endif
        case BINARY:
            fileFormatType = BINARY;
            break;
        default:
            throw sls::RuntimeError("Unknown file format");
        }
        for (const auto &it : dataProcessor)
            if (IsValidThread(it))
                it->SetupFileWriter(fileWriteEnable, masterFileWriteEnable,
                                    fileFormatType, modulePos);
    }

    LOG(logINFO) << "File Format: " << sls::ToString(fileFormatType);
}

std::string Implementation::getFilePath() const { return filePath; }

void Implementation::setFilePath(const std::string &c) {
    if (!c.empty()) {
        mkdir_p(c); // throws if it can't create
        filePath = c;
    }
    LOG(logINFO) << "File path: " << filePath;
}

std::string Implementation::getFileName() const { return fileName; }

void Implementation::setFileName(const std::string &c) {
    fileName = c;
    LOG(logINFO) << "File name: " << fileName;
}

uint64_t Implementation::getFileIndex() const { return fileIndex; }

void Implementation::setFileIndex(const uint64_t i) {
    fileIndex = i;
    LOG(logINFO) << "File Index: " << fileIndex;
}

bool Implementation::getFileWriteEnable() const { return fileWriteEnable; }

void Implementation::setFileWriteEnable(const bool b) {
    if (fileWriteEnable != b) {
        fileWriteEnable = b;
        for (const auto &it : dataProcessor)
            if (IsValidThread(it))
                it->SetupFileWriter(fileWriteEnable, masterFileWriteEnable,
                                    fileFormatType, modulePos);
    }
    LOG(logINFO) << "File Write Enable: "
                 << (fileWriteEnable ? "enabled" : "disabled");
}

bool Implementation::getMasterFileWriteEnable() const {
    return masterFileWriteEnable;
}

void Implementation::setMasterFileWriteEnable(const bool b) {
    if (masterFileWriteEnable != b) {
        masterFileWriteEnable = b;
        for (const auto &it : dataProcessor)
            if (IsValidThread(it))
                it->SetupFileWriter(fileWriteEnable, masterFileWriteEnable,
                                    fileFormatType, modulePos);
    }
    LOG(logINFO) << "Master File Write Enable: "
                 << (masterFileWriteEnable ? "enabled" : "disabled");
}

bool Implementation::getOverwriteEnable() const { return overwriteEnable; }

void Implementation::setOverwriteEnable(const bool b) {
    overwriteEnable = b;
    LOG(logINFO) << "Overwrite Enable: "
                 << (overwriteEnable ? "enabled" : "disabled");
}

uint32_t Implementation::getFramesPerFile() const { return framesPerFile; }

void Implementation::setFramesPerFile(const uint32_t i) {
    framesPerFile = i;
    LOG(logINFO) << "Frames per file: " << framesPerFile;
}

/**************************************************
 *                                                 *
 *   Acquisition                                   *
 *                                                 *
 * ************************************************/
slsDetectorDefs::runStatus Implementation::getStatus() const { return status; }

uint64_t Implementation::getFramesCaught() const {
    uint64_t min = -1;
    for (const auto &it : listener)
        if (IsValidThread(it))
            min = std::min(min, it->GetNumFramesCaught());
    return min;
}

uint64_t Implementation::getCurrentFrameIndex() const {
    uint64_t max = 0;
    for (const auto &it : listener)
        if (IsValidThread(it))
            max = std::max(max, it->GetCurrentFrameIndex());
    return max;
}

double Implementation::getProgress() const {
    // get maximum of processed frame indices
    uint64_t currentFrameIndex = 0;

    for (const auto &it : listener)
        if (IsValidThread(it))
            currentFrameIndex =
                std::max(currentFrameIndex, it->GetListenedIndex());

    return (100.00 *
            ((double)(currentFrameIndex + 1) / (double)numberOfTotalFrames));
}

std::vector<uint64_t> Implementation::getNumMissingPackets() const {
    std::vector<uint64_t> mp(numThreads);
    for (int i = 0; i < numThreads; i++) {
        // not running, read stored statistics
        if (status != RUNNING) {
            mp[i] = listenerStatistics[i].packets_missing;
            continue;
        }
        int np = generalData->packetsPerFrame;
        uint64_t totnp = np;
        // ReadNRows
        if (readNRows != (int)generalData->maxRowsPerReadout) {
            totnp = ((readNRows * np) / generalData->maxRowsPerReadout);
        }
        totnp *= numberOfTotalFrames / rrNbRecvs;
        if (HasValidThread(listener, i))
            mp[i] = listener[i]->GetNumMissingPacket(stoppedFlag, totnp);
    }
    return mp;
}

void Implementation::setScan(slsDetectorDefs::scanParameters s) {
    scanParams = s;
    LOG(logINFO) << "Scan parameters: " << sls::ToString(scanParams);
}

void Implementation::startReceiver() {
    LOG(logINFO) << "Starting Receiver";
    stoppedFlag = false;
    ResetParametersforNewAcquisition();

    // listener
    CreateUDPSockets();

    // callbacks
    if (startAcquisitionCallBack) {
        try {
            startAcquisitionCallBack(filePath, fileName, fileIndex,
                                     generalData->imageSize, pStartAcquisition);
        } catch (const std::exception &e) {
            throw sls::RuntimeError("Start Acquisition Callback Error: " +
                                    std::string(e.what()));
        }
        if (rawDataReadyCallBack != nullptr) {
            LOG(logINFO) << "Data Write has been defined externally";
        }
    }

    // processor->writer
    if (fileWriteEnable) {
        SetupWriter();
    } else
        LOG(logINFO) << "File Write Disabled";

    LOG(logINFO) << "Ready ...";

    // status
    status = RUNNING;

    // Let Threads continue to be ready for acquisition
    StartRunning();

    LOG(logINFO) << "Receiver Started";
    LOG(logINFO) << "Status: " << sls::ToString(status);
}

void Implementation::setStoppedFlag(bool stopped) { stoppedFlag = stopped; }

void Implementation::stopReceiver() {
    LOG(logINFO) << "Stopping Receiver";

    // set status to transmitting
    if (activated) {
        startReadout();
    } else if (status == RUNNING) {
        status = TRANSMITTING;
        LOG(logINFO) << "Status: Transmitting";
    }

    // wait for the processes (Listener and DataProcessor) to be done
    bool running = true;
    while (running) {
        running = false;
        for (const auto &it : listener)
            if (IsValidThread(it) && it->IsRunning())
                running = true;

        for (const auto &it : dataProcessor)
            if (IsValidThread(it) && it->IsRunning())
                running = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

#ifdef HDF5C
    if (fileWriteEnable && fileFormatType == HDF5) {
        if ((modulePos == 0) && HasValidThread(dataProcessor, 0)) {
            // more than 1 file, create virtual file
            if (dataProcessor[0]->GetFilesInAcquisition() > 1 ||
                (numMods[X] * numMods[Y]) > 1) {
                dataProcessor[0]->CreateVirtualFile(
                    filePath, fileName, fileIndex, overwriteEnable, silentMode,
                    modulePos, numThreads, framesPerFile, numberOfTotalFrames,
                    dynamicRange, numMods[X], numMods[Y]);
            }
            // link file in master
            dataProcessor[0]->LinkDataInMasterFile(silentMode);
        }
    }
#endif
    if (fileWriteEnable && masterFileWriteEnable && modulePos == 0) {
        try {
            dataProcessor[0]->UpdateMasterFile(silentMode);
        } catch (...) {
            ; // ignore it and just print it
        }
    }

    // wait for the processes (dataStreamer) to be done
    running = true;
    while (running) {
        running = false;
        for (const auto &it : dataStreamer)
            if (IsValidThread(it) && it->IsRunning())
                running = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    status = RUN_FINISHED;
    LOG(logINFO) << "Status: " << sls::ToString(status);

    { // statistics
        uint64_t tot = 0;
        for (int i = 0; i < numThreads; i++) {
            const ListenerStatistics &ls = listenerStatistics[i];
            uint64_t mp = ls.packets_missing;
            auto signed_mp = int(mp);
            int nf = ls.frames_caught;
            tot += nf;
            std::string mpMessage = std::to_string(signed_mp);
            if (signed_mp < 0) {
                mpMessage =
                    std::to_string(abs(signed_mp)) + std::string(" (Extra)");
            }

            TLogLevel lev = (signed_mp > 0) ? logINFORED : logINFOGREEN;
            LOG(lev) <<
                // udp port number could be the second if selected interface is
                // 2 for jungfrau
                "Summary of Port " << udpPortNum[i]
                     << "\n\tMissing Packets\t\t: " << mpMessage
                     << "\n\tComplete Frames\t\t: " << nf
                     << "\n\tLast Frame Caught\t: " << ls.last_frame;
        }

        if (!passiveMode) {
            if (!activated) {
                LOG(logINFORED) << "Deactivated Receiver";
            }
            if (!detectorDataStream[0]) {
                LOG(logINFORED) << "Deactivated Left Port";
            }
            if (!detectorDataStream[1]) {
                LOG(logINFORED) << "Deactivated Right Port";
            }
            // callback
            if (acquisitionFinishedCallBack) {
                try {
                    acquisitionFinishedCallBack((tot / numThreads),
                                                pAcquisitionFinished);
                } catch (const std::exception &e) {
                    // change status
                    status = IDLE;
                    LOG(logINFO) << "Receiver Stopped";
                    LOG(logINFO) << "Status: " << sls::ToString(status);
                    throw sls::RuntimeError(
                        "Acquisition Finished Callback Error: " +
                        std::string(e.what()));
                }
            }
        }
    }

    // change status
    status = IDLE;
    LOG(logINFO) << "Receiver Stopped";
    LOG(logINFO) << "Status: " << sls::ToString(status);
}

void Implementation::startReadout() {
    if (status == RUNNING) {
        // wait for incoming delayed packets
        int totalPacketsReceived = 0;
        int previousValue = -1;
        int active_listeners = 0;
        for (const auto &it : listener)
            if (IsValidThread(it)) {
                totalPacketsReceived += it->GetPacketsCaught();
                ++active_listeners;
            }

        // wait for all packets
        const int numPacketsToReceive = numberOfTotalFrames *
                                        generalData->packetsPerFrame *
                                        active_listeners / rrNbRecvs;
        if (totalPacketsReceived != numPacketsToReceive) {
            while (totalPacketsReceived != previousValue) {
                LOG(logDEBUG3)
                    << "waiting for all packets, previousValue:"
                    << previousValue
                    << " totalPacketsReceived: " << totalPacketsReceived;
                /* TODO! Need to find optimal time **/
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                previousValue = totalPacketsReceived;
                totalPacketsReceived = 0;
                for (const auto &it : listener)
                    if (IsValidThread(it))
                        totalPacketsReceived += it->GetPacketsCaught();

                LOG(logDEBUG3) << "\tupdated:  totalPacketsReceived:"
                               << totalPacketsReceived;
            }
        }

        std::vector<uint64_t> missing_packets = getNumMissingPackets();

        // store listener statistics
        for (int i = 0; i < numThreads; ++i) {
            ListenerStatistics &ls = listenerStatistics[i];
            ls.packets_missing = missing_packets[i];
            if (HasValidThread(listener, i)) {
                const auto &l = listener[i];
                ls.packets_caught = l->GetPacketsCaught();
                ls.frames_caught = l->GetNumFramesCaught();
                ls.last_frame = l->GetLastFrameIndexCaught();
            }
        }

        status = TRANSMITTING;
        LOG(logINFO) << "Status: Transmitting";
    }

    // shut down udp sockets to make listeners push dummy (end) packets for
    // processors
    shutDownUDPSockets();
}

void Implementation::shutDownUDPSockets() {
    for (const auto &it : listener)
        if (IsValidThread(it))
            it->ShutDownUDPSocket();
    frameAssembler.reset();
}

void Implementation::restreamStop() {
    for (const auto &it : dataStreamer)
        if (IsValidThread(it))
            it->RestreamStop();
    LOG(logINFO) << "Restreaming Dummy Header via ZMQ successful";
}

void Implementation::ResetParametersforNewAcquisition() {
    for (const auto &it : listener)
        if (IsValidThread(it))
            it->ResetParametersforNewAcquisition();
    for (auto &it : listenerStatistics)
        it.reset();
    for (const auto &it : dataProcessor)
        if (IsValidThread(it))
            it->ResetParametersforNewAcquisition();

    std::ostringstream os;
    os << filePath << '/' << fileName;
    std::string fnametostream = os.str();
    for (const auto &it : dataStreamer)
        if (IsValidThread(it))
            it->ResetParametersforNewAcquisition(fnametostream);
}

void Implementation::CreateUDPSockets() {
    try {
        for (const auto &it : listener)
            if (IsValidThread(it))
                it->CreateUDPSockets();
    } catch (const sls::RuntimeError &e) {
        shutDownUDPSockets();
        throw sls::RuntimeError("Could not create UDP Socket(s).");
    }
    LOG(logDEBUG) << "UDP socket(s) created successfully.";
}

void Implementation::SetupWriter() {
    // master file
    std::unique_ptr<MasterAttributes> masterAttributes;
    if (masterFileWriteEnable && modulePos == 0) {
        switch (detType) {
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
        masterAttributes->detType = detType;
        masterAttributes->timingMode = timingMode;
        masterAttributes->imageSize = generalData->imageSize;
        masterAttributes->nPixels =
            xy(generalData->nPixelsX, generalData->nPixelsY);
        masterAttributes->maxFramesPerFile = framesPerFile;
        masterAttributes->frameDiscardMode = frameDiscardMode;
        masterAttributes->framePadding = framePadding;
        masterAttributes->scanParams = scanParams;
        masterAttributes->totalFrames = numberOfTotalFrames;
        masterAttributes->exptime = acquisitionTime;
        masterAttributes->period = acquisitionPeriod;
        masterAttributes->burstMode = burstMode;
        masterAttributes->numUDPInterfaces = numUDPInterfaces;
        masterAttributes->dynamicRange = dynamicRange;
        masterAttributes->tenGiga = tengigaEnable;
        masterAttributes->thresholdEnergyeV = thresholdEnergyeV;
        masterAttributes->thresholdAllEnergyeV = thresholdAllEnergyeV;
        masterAttributes->subExptime = subExpTime;
        masterAttributes->subPeriod = subPeriod;
        masterAttributes->quad = quadEnable;
        masterAttributes->readNRows = readNRows;
        masterAttributes->ratecorr = rateCorrections;
        masterAttributes->adcmask =
            tengigaEnable ? adcEnableMaskTenGiga : adcEnableMaskOneGiga;
        masterAttributes->analog =
            (readoutType == ANALOG_ONLY || readoutType == ANALOG_AND_DIGITAL)
                ? 1
                : 0;
        masterAttributes->analogSamples = numberOfAnalogSamples;
        masterAttributes->digital =
            (readoutType == DIGITAL_ONLY || readoutType == ANALOG_AND_DIGITAL)
                ? 1
                : 0;
        masterAttributes->digitalSamples = numberOfDigitalSamples;
        masterAttributes->dbitoffset = ctbDbitOffset;
        masterAttributes->dbitlist = 0;
        for (auto &i : ctbDbitList) {
            masterAttributes->dbitlist |= (1 << i);
        }
        masterAttributes->roi = roi;
        masterAttributes->counterMask = counterMask;
        masterAttributes->exptime1 = acquisitionTime1;
        masterAttributes->exptime2 = acquisitionTime2;
        masterAttributes->exptime3 = acquisitionTime3;
        masterAttributes->gateDelay1 = gateDelay1;
        masterAttributes->gateDelay2 = gateDelay2;
        masterAttributes->gateDelay3 = gateDelay3;
        masterAttributes->gates = numberOfGates;
        masterAttributes->additionalJsonHeader = additionalJsonHeader;
    }

    try {
        for (int i = 0; i < numThreads; ++i) {
            if (HasValidThread(dataProcessor, i))
                dataProcessor[i]->CreateFirstFiles(
                    masterAttributes.get(), filePath, fileName, fileIndex,
                    overwriteEnable, silentMode, modulePos, numThreads,
                    udpPortNum[i], framesPerFile, numberOfTotalFrames,
                    dynamicRange);
        }
    } catch (const sls::RuntimeError &e) {
        shutDownUDPSockets();
        for (const auto &it : dataProcessor)
            if (IsValidThread(it))
                it->CloseFiles();
        throw sls::RuntimeError("Could not create first data file.");
    }
}

void Implementation::StartRunning() {

    // set running mask and post semaphore to start the inner loop in execution
    // thread
    for (const auto &it : listener)
        if (IsValidThread(it)) {
            it->StartRunning();
            it->Continue();
        }
    for (const auto &it : dataProcessor)
        if (IsValidThread(it)) {
            it->StartRunning();
            it->Continue();
        }
    for (const auto &it : dataStreamer)
        if (IsValidThread(it)) {
            it->StartRunning();
            it->Continue();
        }
}

/**************************************************
 *                                                 *
 *   Network Configuration (UDP)                   *
 *                                                 *
 * ************************************************/
int Implementation::getNumberofUDPInterfaces() const {
    return numUDPInterfaces;
}

void Implementation::setNumberofUDPInterfaces(const int n) {

    if (numUDPInterfaces != n) {
        // remove previous correction of interface geometry (ports) in x/y dirs
        // the new port geometry correction will be done in setDetectorSize
        PortGeometry prev_geom = GetPortGeometry();
        numMods[X] /= prev_geom[X];
        numMods[Y] /= prev_geom[Y];

        // clear all threads and fifos
        DestroyThreads();
        fifo.clear();

        // set local variables
        generalData->SetNumberofInterfaces(n);
        numUDPInterfaces = n;
        numThreads = n;

        // fifo
        udpSocketBufferSize = generalData->defaultUdpSocketBufferSize;
        SetupFifoStructure();

        // create threads
        CreateThreads();

        // update (from 1 to 2 interface) & also for printout
        setDetectorSize(numMods);
        // update row and column in dataprocessor
        setModulePositionId(modulePos);

        // update call backs
        if (rawDataReadyCallBack) {
            for (const auto &it : dataProcessor)
                if (IsValidThread(it))
                    it->registerCallBackRawDataReady(rawDataReadyCallBack,
                                                     pRawDataReady);
        }
        if (rawDataModifyReadyCallBack) {
            for (const auto &it : dataProcessor)
                if (IsValidThread(it))
                    it->registerCallBackRawDataModifyReady(
                        rawDataModifyReadyCallBack, pRawDataReady);
        }

        // test socket buffer size with current set up
        setUDPSocketBufferSize(0);
    }

    LOG(logINFO) << "Number of Interfaces: " << numUDPInterfaces;
}

std::string Implementation::getEthernetInterface() const { return eth[0]; }

void Implementation::setEthernetInterface(const std::string &c) {
    eth[0] = c;
    LOG(logINFO) << "Ethernet Interface: " << eth[0];
}

std::string Implementation::getEthernetInterface2() const { return eth[1]; }

void Implementation::setEthernetInterface2(const std::string &c) {
    eth[1] = c;
    LOG(logINFO) << "Ethernet Interface 2: " << eth[1];
}

uint32_t Implementation::getUDPPortNumber() const { return udpPortNum[0]; }

void Implementation::setUDPPortNumber(const uint32_t i) {
    udpPortNum[0] = i;
    LOG(logINFO) << "UDP Port Number[0]: " << udpPortNum[0];
}

uint32_t Implementation::getUDPPortNumber2() const { return udpPortNum[1]; }

void Implementation::setUDPPortNumber2(const uint32_t i) {
    udpPortNum[1] = i;
    LOG(logINFO) << "UDP Port Number[1]: " << udpPortNum[1];
}

int Implementation::getUDPSocketBufferSize() const {
    return udpSocketBufferSize;
}

void Implementation::setUDPSocketBufferSize(const int s) {
    // custom setup is not 0 (must complain if set up didnt work)
    // testing default setup at startup, argument is 0 to use default values
    int size = (s == 0) ? udpSocketBufferSize : s;
    size_t listSize = listener.size();
    bool has_port_geometry = ((detType == EIGER) || (detType == JUNGFRAU));
    if (has_port_geometry && int(listSize) != numUDPInterfaces) {
        throw sls::RuntimeError(
            "Number of Interfaces " + std::to_string(numUDPInterfaces) +
            " do not match listener size " + std::to_string(listSize));
    }

    for (auto &it : listener)
        if (IsValidThread(it))
            it->CreateDummySocketForUDPSocketBufferSize(size);

    // custom and didnt set, throw error
    if (s != 0 && udpSocketBufferSize != s) {
        throw sls::RuntimeError("Could not set udp socket buffer size. (No "
                                "CAP_NET_ADMIN privileges?)");
    }
}

int Implementation::getActualUDPSocketBufferSize() const {
    return actualUDPSocketBufferSize;
}

/**************************************************
 *                                                 *
 *   ZMQ Streaming Parameters (ZMQ)                *
 *                                                 *
 * ************************************************/
bool Implementation::getDataStreamEnable() const { return dataStreamEnable; }

void Implementation::setDataStreamEnable(const bool enable) {
    if (dataStreamEnable != enable) {
        DestroyThreads();
        dataStreamEnable = enable;
        CreateThreads();
    }
    LOG(logINFO) << "Data Send to Gui: " << dataStreamEnable;
}

uint32_t Implementation::getStreamingFrequency() const {
    return streamingFrequency;
}

void Implementation::setStreamingFrequency(const uint32_t freq) {
    streamingFrequency = freq;
    LOG(logINFO) << "Streaming Frequency: " << streamingFrequency;
}

uint32_t Implementation::getStreamingTimer() const {
    return streamingTimerInMs;
}

void Implementation::setStreamingTimer(const uint32_t time_in_ms) {
    streamingTimerInMs = time_in_ms;
    LOG(logINFO) << "Streamer Timer: " << streamingTimerInMs;
}

uint32_t Implementation::getStreamingStartingFrameNumber() const {
    return streamingStartFnum;
}

void Implementation::setStreamingStartingFrameNumber(const uint32_t fnum) {
    streamingStartFnum = fnum;
    LOG(logINFO) << "Streaming Start Frame num: " << streamingStartFnum;
}

uint32_t Implementation::getStreamingPort() const { return streamingPort; }

void Implementation::setStreamingPort(const uint32_t i) {
    streamingPort = i;
    LOG(logINFO) << "Streaming Port: " << streamingPort;
}

sls::IpAddr Implementation::getStreamingSourceIP() const {
    return streamingSrcIP;
}

void Implementation::setStreamingSourceIP(const sls::IpAddr ip) {
    streamingSrcIP = ip;
    LOG(logINFO) << "Streaming Source IP: " << streamingSrcIP;
}

int Implementation::getStreamingHwm() const { return streamingHwm; }

void Implementation::setStreamingHwm(const int i) {
    streamingHwm = i;
    LOG(logINFO) << "Streaming Hwm: "
                 << (i == -1 ? "Default (-1)" : std::to_string(streamingHwm));
}

std::map<std::string, std::string>
Implementation::getAdditionalJsonHeader() const {
    return additionalJsonHeader;
}

void Implementation::setAdditionalJsonHeader(
    const std::map<std::string, std::string> &c) {

    additionalJsonHeader = c;
    for (const auto &it : dataStreamer)
        if (IsValidThread(it))
            it->SetAdditionalJsonHeader(c);
    LOG(logINFO) << "Additional JSON Header: "
                 << sls::ToString(additionalJsonHeader);
}

std::string
Implementation::getAdditionalJsonParameter(const std::string &key) const {
    if (additionalJsonHeader.find(key) != additionalJsonHeader.end()) {
        return additionalJsonHeader.at(key);
    }
    throw sls::RuntimeError("No key " + key +
                            " found in additional json header");
}

void Implementation::setAdditionalJsonParameter(const std::string &key,
                                                const std::string &value) {
    auto pos = additionalJsonHeader.find(key);
    // if value is empty, delete
    if (value.empty()) {
        // doesnt exist
        if (pos == additionalJsonHeader.end()) {
            LOG(logINFO) << "Additional json parameter (" << key
                         << ") does not exist anyway";
        } else {
            LOG(logINFO) << "Deleting additional json parameter (" << key
                         << ")";
            additionalJsonHeader.erase(pos);
        }
    }
    // if found, set it
    else if (pos != additionalJsonHeader.end()) {
        additionalJsonHeader[key] = value;
        LOG(logINFO) << "Setting additional json parameter (" << key << ") to "
                     << value;
    }
    // append if not found
    else {
        additionalJsonHeader[key] = value;
        LOG(logINFO) << "Adding additional json parameter (" << key << ") to "
                     << value;
    }
    for (const auto &it : dataStreamer)
        if (IsValidThread(it))
            it->SetAdditionalJsonHeader(additionalJsonHeader);
    LOG(logINFO) << "Additional JSON Header: "
                 << sls::ToString(additionalJsonHeader);
}

/**************************************************
 *                                                 *
 *   Detector Parameters                           *
 *                                                 *
 * ************************************************/
void Implementation::updateTotalNumberOfFrames() {
    int64_t repeats = numberOfTriggers;
    int64_t numFrames = numberOfFrames;
    // gotthard2
    if (detType == GOTTHARD2) {
        // auto
        if (timingMode == AUTO_TIMING) {
            // burst mode, repeats = #bursts
            if (burstMode == BURST_INTERNAL || burstMode == BURST_EXTERNAL) {
                repeats = numberOfBursts;
            }
            // continuous, repeats = 1 (no trigger as well)
            else {
                repeats = 1;
            }
        }
        // trigger
        else {
            // continuous, numFrames is limited
            if (burstMode == CONTINUOUS_INTERNAL ||
                burstMode == CONTINUOUS_EXTERNAL) {
                numFrames = 1;
            }
        }
    }
    numberOfTotalFrames =
        numFrames * repeats * (int64_t)(numberOfAdditionalStorageCells + 1);
    if (numberOfTotalFrames == 0) {
        throw sls::RuntimeError("Invalid total number of frames to receive: 0");
    } else if (numberOfTotalFrames % rrNbRecvs != 0) {
        throw sls::RuntimeError("Total number of frames is not a multiple of "
                                "Round-Robin number of recvs");
    }
    LOG(logINFO) << "Total Number of Frames: " << numberOfTotalFrames;
}

uint64_t Implementation::getNumberOfFrames() const { return numberOfFrames; }

void Implementation::setNumberOfFrames(const uint64_t i) {
    numberOfFrames = i;
    LOG(logINFO) << "Number of Frames: " << numberOfFrames;
    updateTotalNumberOfFrames();
}

uint64_t Implementation::getNumberOfTriggers() const {
    return numberOfTriggers;
}

void Implementation::setNumberOfTriggers(const uint64_t i) {
    numberOfTriggers = i;
    LOG(logINFO) << "Number of Triggers: " << numberOfTriggers;
    updateTotalNumberOfFrames();
}

uint64_t Implementation::getNumberOfBursts() const { return numberOfBursts; }

void Implementation::setNumberOfBursts(const uint64_t i) {
    numberOfBursts = i;
    LOG(logINFO) << "Number of Bursts: " << numberOfBursts;
    updateTotalNumberOfFrames();
}

int Implementation::getNumberOfAdditionalStorageCells() const {
    return numberOfAdditionalStorageCells;
}

void Implementation::setNumberOfAdditionalStorageCells(const int i) {
    numberOfAdditionalStorageCells = i;
    LOG(logINFO) << "Number of Additional Storage Cells: "
                 << numberOfAdditionalStorageCells;
    updateTotalNumberOfFrames();
}

void Implementation::setNumberOfGates(const int i) {
    numberOfGates = i;
    LOG(logINFO) << "Number of Gates: " << numberOfGates;
}

slsDetectorDefs::timingMode Implementation::getTimingMode() const {
    return timingMode;
}

void Implementation::setTimingMode(const slsDetectorDefs::timingMode i) {
    timingMode = i;
    LOG(logINFO) << "Timing Mode: " << timingMode;
    updateTotalNumberOfFrames();
}

slsDetectorDefs::burstMode Implementation::getBurstMode() const {
    return burstMode;
}

void Implementation::setBurstMode(const slsDetectorDefs::burstMode i) {
    burstMode = i;
    LOG(logINFO) << "Burst Mode: " << burstMode;
    updateTotalNumberOfFrames();
}

ns Implementation::getAcquisitionPeriod() const { return acquisitionPeriod; }

void Implementation::setAcquisitionPeriod(const ns i) {
    acquisitionPeriod = i;
    LOG(logINFO) << "Acquisition Period: " << sls::ToString(acquisitionPeriod);
}

ns Implementation::getAcquisitionTime() const { return acquisitionTime; }

void Implementation::updateAcquisitionTime() {
    if (acquisitionTime1 == acquisitionTime2 &&
        acquisitionTime2 == acquisitionTime3) {
        acquisitionTime = acquisitionTime1;
    } else {
        acquisitionTime = std::chrono::nanoseconds(0);
    }
}

void Implementation::setAcquisitionTime(const ns i) {
    acquisitionTime = i;
    LOG(logINFO) << "Acquisition Time: " << sls::ToString(acquisitionTime);
}

void Implementation::setAcquisitionTime1(const ns i) {
    acquisitionTime1 = i;
    LOG(logINFO) << "Acquisition Time1: " << sls::ToString(acquisitionTime1);
    updateAcquisitionTime();
}

void Implementation::setAcquisitionTime2(const ns i) {
    acquisitionTime2 = i;
    LOG(logINFO) << "Acquisition Time2: " << sls::ToString(acquisitionTime2);
    updateAcquisitionTime();
}

void Implementation::setAcquisitionTime3(const ns i) {
    acquisitionTime3 = i;
    LOG(logINFO) << "Acquisition Time3: " << sls::ToString(acquisitionTime3);
    updateAcquisitionTime();
}

void Implementation::setGateDelay1(const ns i) {
    gateDelay1 = i;
    LOG(logINFO) << "Gate Delay1: " << sls::ToString(gateDelay1);
}

void Implementation::setGateDelay2(const ns i) {
    gateDelay2 = i;
    LOG(logINFO) << "Gate Delay2: " << sls::ToString(gateDelay2);
}

void Implementation::setGateDelay3(const ns i) {
    gateDelay3 = i;
    LOG(logINFO) << "Gate Delay3: " << sls::ToString(gateDelay3);
}

ns Implementation::getSubExpTime() const { return subExpTime; }

void Implementation::setSubExpTime(const ns i) {
    subExpTime = i;
    LOG(logINFO) << "Sub Exposure Time: " << sls::ToString(subExpTime);
}

ns Implementation::getSubPeriod() const { return subPeriod; }

void Implementation::setSubPeriod(const ns i) {
    subPeriod = i;
    LOG(logINFO) << "Sub Period: " << sls::ToString(subPeriod);
}

uint32_t Implementation::getNumberofAnalogSamples() const {
    return numberOfAnalogSamples;
}

void Implementation::setNumberofAnalogSamples(const uint32_t i) {
    if (numberOfAnalogSamples != i) {
        numberOfAnalogSamples = i;

        ctbAnalogDataBytes = generalData->setImageSize(
            tengigaEnable ? adcEnableMaskTenGiga : adcEnableMaskOneGiga,
            numberOfAnalogSamples, numberOfDigitalSamples, tengigaEnable,
            readoutType);

        SetupFifoStructure();
    }
    LOG(logINFO) << "Number of Analog Samples: " << numberOfAnalogSamples;
    LOG(logINFO) << "Packets per Frame: " << (generalData->packetsPerFrame);
}

uint32_t Implementation::getNumberofDigitalSamples() const {
    return numberOfDigitalSamples;
}

void Implementation::setNumberofDigitalSamples(const uint32_t i) {
    if (numberOfDigitalSamples != i) {
        numberOfDigitalSamples = i;

        ctbAnalogDataBytes = generalData->setImageSize(
            tengigaEnable ? adcEnableMaskTenGiga : adcEnableMaskOneGiga,
            numberOfAnalogSamples, numberOfDigitalSamples, tengigaEnable,
            readoutType);

        SetupFifoStructure();
    }
    LOG(logINFO) << "Number of Digital Samples: " << numberOfDigitalSamples;
    LOG(logINFO) << "Packets per Frame: " << (generalData->packetsPerFrame);
}

uint32_t Implementation::getCounterMask() const { return counterMask; }

void Implementation::setCounterMask(const uint32_t i) {
    if (counterMask != i) {
        int ncounters = __builtin_popcount(i);
        if (ncounters < 1 || ncounters > 3) {
            throw sls::RuntimeError("Invalid number of counters " +
                                    std::to_string(ncounters) +
                                    ". Expected 1-3.");
        }
        counterMask = i;
        generalData->SetNumberofCounters(ncounters);
        SetupFifoStructure();
    }
    LOG(logINFO) << "Counter mask: " << sls::ToStringHex(counterMask);
    int ncounters = __builtin_popcount(counterMask);
    LOG(logINFO) << "Number of counters: " << ncounters;
}

uint32_t Implementation::getDynamicRange() const { return dynamicRange; }

void Implementation::setDynamicRange(const uint32_t i) {
    if (dynamicRange != i) {
        dynamicRange = i;

        if (detType == EIGER || detType == MYTHEN3) {
            generalData->SetDynamicRange(i);

            fifoDepth = generalData->defaultFifoDepth;
            SetupFifoStructure();
        }
    }
    LOG(logINFO) << "Dynamic Range: " << dynamicRange;
}

slsDetectorDefs::ROI Implementation::getROI() const { return roi; }

void Implementation::setROI(slsDetectorDefs::ROI arg) {
    if (roi.xmin != arg.xmin || roi.xmax != arg.xmax) {
        roi.xmin = arg.xmin;
        roi.xmax = arg.xmax;

        // only for gotthard
        generalData->SetROI(arg);
        framesPerFile = generalData->maxFramesPerFile;
        SetupFifoStructure();
    }

    LOG(logINFO) << "ROI: [" << roi.xmin << ", " << roi.xmax << "]";
    LOG(logINFO) << "Packets per Frame: " << (generalData->packetsPerFrame);
}

bool Implementation::getTenGigaEnable() const { return tengigaEnable; }

void Implementation::setTenGigaEnable(const bool b) {
    if (tengigaEnable != b) {
        tengigaEnable = b;
        // side effects
        switch (detType) {
        case EIGER:
        case MYTHEN3:
            generalData->SetTenGigaEnable(b);
            break;
        case MOENCH:
        case CHIPTESTBOARD:
            ctbAnalogDataBytes = generalData->setImageSize(
                tengigaEnable ? adcEnableMaskTenGiga : adcEnableMaskOneGiga,
                numberOfAnalogSamples, numberOfDigitalSamples, tengigaEnable,
                readoutType);
            break;
        default:
            break;
        }
        SetupFifoStructure();
    }
    LOG(logINFO) << "Ten Giga: " << (tengigaEnable ? "enabled" : "disabled");
    LOG(logINFO) << "Packets per Frame: " << (generalData->packetsPerFrame);
}

bool Implementation::getFlipRows() const { return flipRows; }

void Implementation::setFlipRows(bool enable) {
    flipRows = enable;

    if (!quadEnable) {
        for (const auto &it : dataStreamer)
            if (IsValidThread(it))
                it->SetFlipRows(flipRows);
    }
    // quad
    else {
        if (dataStreamer.size() == 2) {
            if (dataStreamer[0])
                dataStreamer[0]->SetFlipRows(false);
            if (dataStreamer[1])
                dataStreamer[1]->SetFlipRows(true);
        }
    }
    LOG(logINFO) << "Flip Rows: " << flipRows;
}

bool Implementation::getQuad() const { return quadEnable; }

void Implementation::setQuad(const bool b) {
    if (quadEnable != b) {
        quadEnable = b;

        if (!quadEnable) {
            for (const auto &it : dataStreamer)
                if (IsValidThread(it)) {
                    it->SetNumberofModules(numMods);
                    it->SetFlipRows(flipRows);
                }
        } else {
            int size[2] = {1, 2};
            for (const auto &it : dataStreamer)
                if (IsValidThread(it))
                    it->SetNumberofModules(size);
            if (dataStreamer.size() == 2) {
                if (dataStreamer[0])
                    dataStreamer[0]->SetFlipRows(false);
                if (dataStreamer[1])
                    dataStreamer[1]->SetFlipRows(true);
            }
        }
    }
    LOG(logINFO) << "Quad Enable: " << quadEnable;
}

bool Implementation::getActivate() const { return activated; }

void Implementation::setActivate(bool enable) {
    if (activated != enable) {
        DestroyThreads();
        activated = enable;
        CreateThreads();
    }
    LOG(logINFO) << "Activation: " << (activated ? "enabled" : "disabled");
}

bool Implementation::getDetectorDataStream(const portPosition port) const {
    int index = (port == LEFT ? 0 : 1);
    return detectorDataStream[index];
}

void Implementation::setDetectorDataStream(const portPosition port,
                                           const bool enable) {
    int index = (port == LEFT ? 0 : 1);
    if (detectorDataStream[index] != enable) {
        DestroyThreads();
        detectorDataStream[index] = enable;
        CreateThreads();
    }
    LOG(logINFO) << "Detector datastream (" << sls::ToString(port)
                 << " Port): " << sls::ToString(detectorDataStream[index]);
}

int Implementation::getReadNRows() const { return readNRows; }

void Implementation::setReadNRows(const int value) {
    readNRows = value;
    LOG(logINFO) << "Number of rows: " << readNRows;
}

void Implementation::setThresholdEnergy(const int value) {
    thresholdEnergyeV = value;
    LOG(logINFO) << "Threshold Energy: " << thresholdEnergyeV << " eV";
}

void Implementation::setThresholdEnergy(const std::array<int, 3> value) {
    thresholdAllEnergyeV = value;
    LOG(logINFO) << "Threshold Energy (eV): "
                 << sls::ToString(thresholdAllEnergyeV);
}

void Implementation::setRateCorrections(const std::vector<int64_t> &t) {
    rateCorrections = t;
    LOG(logINFO) << "Rate Corrections: " << sls::ToString(rateCorrections);
}

slsDetectorDefs::readoutMode Implementation::getReadoutMode() const {
    return readoutType;
}

void Implementation::setReadoutMode(const readoutMode f) {
    if (readoutType != f) {
        readoutType = f;

        // side effects
        ctbAnalogDataBytes = generalData->setImageSize(
            tengigaEnable ? adcEnableMaskTenGiga : adcEnableMaskOneGiga,
            numberOfAnalogSamples, numberOfDigitalSamples, tengigaEnable,
            readoutType);
        SetupFifoStructure();
    }
    LOG(logINFO) << "Readout Mode: " << sls::ToString(f);
    LOG(logINFO) << "Packets per Frame: " << (generalData->packetsPerFrame);
}

uint32_t Implementation::getADCEnableMask() const {
    return adcEnableMaskOneGiga;
}

void Implementation::setADCEnableMask(uint32_t mask) {
    if (adcEnableMaskOneGiga != mask) {
        adcEnableMaskOneGiga = mask;
        ctbAnalogDataBytes = generalData->setImageSize(
            tengigaEnable ? adcEnableMaskTenGiga : adcEnableMaskOneGiga,
            numberOfAnalogSamples, numberOfDigitalSamples, tengigaEnable,
            readoutType);

        SetupFifoStructure();
    }
    LOG(logINFO) << "ADC Enable Mask for 1Gb mode: 0x" << std::hex
                 << adcEnableMaskOneGiga << std::dec;
    LOG(logINFO) << "Packets per Frame: " << (generalData->packetsPerFrame);
}

uint32_t Implementation::getTenGigaADCEnableMask() const {
    return adcEnableMaskTenGiga;
}

void Implementation::setTenGigaADCEnableMask(uint32_t mask) {
    if (adcEnableMaskTenGiga != mask) {
        adcEnableMaskTenGiga = mask;

        ctbAnalogDataBytes = generalData->setImageSize(
            tengigaEnable ? adcEnableMaskTenGiga : adcEnableMaskOneGiga,
            numberOfAnalogSamples, numberOfDigitalSamples, tengigaEnable,
            readoutType);

        SetupFifoStructure();
    }
    LOG(logINFO) << "ADC Enable Mask for 10Gb mode: 0x" << std::hex
                 << adcEnableMaskTenGiga << std::dec;
    LOG(logINFO) << "Packets per Frame: " << (generalData->packetsPerFrame);
}

std::vector<int> Implementation::getDbitList() const { return ctbDbitList; }

void Implementation::setDbitList(const std::vector<int> &v) { ctbDbitList = v; }

int Implementation::getDbitOffset() const { return ctbDbitOffset; }

void Implementation::setDbitOffset(const int s) { ctbDbitOffset = s; }

/**************************************************
 *                                                *
 *    Callbacks                                   *
 *                                                *
 * ************************************************/
void Implementation::registerCallBackStartAcquisition(
    int (*func)(std::string, std::string, uint64_t, uint32_t, void *),
    void *arg) {
    startAcquisitionCallBack = func;
    pStartAcquisition = arg;
}

void Implementation::registerCallBackAcquisitionFinished(void (*func)(uint64_t,
                                                                      void *),
                                                         void *arg) {
    acquisitionFinishedCallBack = func;
    pAcquisitionFinished = arg;
}

void Implementation::registerCallBackRawDataReady(
    void (*func)(char *, char *, uint32_t, void *), void *arg) {
    rawDataReadyCallBack = func;
    pRawDataReady = arg;
    for (const auto &it : dataProcessor)
        if (IsValidThread(it))
            it->registerCallBackRawDataReady(rawDataReadyCallBack,
                                             pRawDataReady);
}

void Implementation::registerCallBackRawDataModifyReady(
    void (*func)(char *, char *, uint32_t &, void *), void *arg) {
    rawDataModifyReadyCallBack = func;
    pRawDataReady = arg;
    for (const auto &it : dataProcessor)
        if (IsValidThread(it))
            it->registerCallBackRawDataModifyReady(rawDataModifyReadyCallBack,
                                                   pRawDataReady);
}

void Implementation::setListenersCPUAffinity(
    const FixedCPUSetAffinityList &cpu_affinities) {
    if (int(cpu_affinities.size()) != numThreads)
        throw sls::RuntimeError("Invalid cpu_affinities size: " +
                                std::to_string(cpu_affinities.size()));
    else if (!activated)
        throw sls::RuntimeError("Receiver not activated");
    numaMask.clear();
    for (int i = 0; i < numThreads; ++i) {
        if (HasValidThread(listener, i))
            listener[i]->SetThreadCPUAffinity(cpu_affinities[i]);
        auto numa_mask = GetFifoNUMAMask(cpu_affinities[i]);
        numaMask.push_back(std::make_unique<NUMAMask>(numa_mask));
    }
    SetupFifoStructure();
}

Implementation::NUMAMask
Implementation::GetFifoNUMAMask(AnyCPUAffinity cpu_affinity) {
    using FixedCPUSetAffinity = sls::CPUAffinity::FixedCPUSetAffinityMask;
    if (std::holds_alternative<FixedCPUSetAffinity>(cpu_affinity)) {
        auto cpu_mask = std::get<FixedCPUSetAffinity>(cpu_affinity);
        return cpu_mask.get_numa_mask();
    }
    return {};
}

void Implementation::setPacketBlockAllocators(
    const PacketBlockAllocList &packet_allocs) {
    if (int(packet_allocs.size()) != numUDPInterfaces)
        throw sls::RuntimeError("Invalid packet_allocs size: " +
                                std::to_string(packet_allocs.size()));
    else if (!activated)
        throw sls::RuntimeError("Receiver not activated");
    packetAllocPtr.clear();
    std::copy(packet_allocs.begin(), packet_allocs.end(),
              std::back_inserter(packetAllocPtr));
    SetupFifoStructure();
}

sls::AnyPacketBlockList Implementation::GetFramePacketBlocks(uint64_t frame) {
    if (!passiveMode)
        throw sls::RuntimeError("GetFramePacketBlocks: not in passiveMode");

    if (status != RUNNING)
        return {};

    // find the minimum frame number if first available was requested
    auto is_not_valid = [](auto &&f) { return f == uint64_t(-1); };

    if (is_not_valid(frame)) {
        for (auto &f : fifo) {
            uint64_t iface_frame = f->GetNextFrameNumber();
            if (is_not_valid(iface_frame))
                return {};
            else if (is_not_valid(frame))
                frame = iface_frame;
            else if (iface_frame != frame)
                throw sls::RuntimeError("Expected frame " +
                                        std::to_string(frame) + ", got " +
                                        std::to_string(iface_frame));
        }
    }

    sls::AnyPacketBlockList blocks;
    size_t valid_ports = 0;
    for (auto &f : fifo) {
        blocks.emplace_back(f->GetFramePackets(frame));
        std::visit(
            [&](auto &b) {
                if (b && b->getValidPacketMask().any())
                    ++valid_ports;
            },
            blocks.back());
    }

    auto &&fd = frameDiscardMode;
    if (((fd == DISCARD_PARTIAL_FRAMES) && (valid_ports != listener.size())) ||
        ((fd == DISCARD_EMPTY_FRAMES) && !valid_ports))
        for (auto &b : blocks)
            std::visit(
                [](auto &b) {
                    if (b)
                        b->discard();
                },
                b);

    return blocks;
}

void Implementation::clearAllBuffers() {
    for (const auto &f : fifo)
        f->ClearAllBuffers();
}

void Implementation::setRoundRobin(int nb_rr_recvs, int rr_idx) {
    if ((nb_rr_recvs < 1) || (rr_idx < 0) || (rr_idx >= nb_rr_recvs))
        throw sls::RuntimeError("Invalid Round-Robin params: "
                                "nb_recvs=" +
                                std::to_string(nb_rr_recvs) +
                                ", "
                                "recv_idx=" +
                                std::to_string(rr_idx));
    else if (!passiveMode && (nb_rr_recvs > 1))
        throw sls::RuntimeError("Round-Robin supported in passive mode");
    if ((nb_rr_recvs == rrNbRecvs) && (rr_idx == rrRecvIdx))
        return;
    DestroyThreads();
    rrNbRecvs = nb_rr_recvs;
    rrRecvIdx = rr_idx;
    LOG(logINFO) << "Round-Robin: NbRecvs=" << rrNbRecvs << ", "
                 << "RecvIdx=" << rrRecvIdx;
    CreateThreads();
}

/* statistics */
void Implementation::ListenerStatistics::reset() {
    packets_missing = 0;
    packets_caught = 0;
    frames_caught = 0;
    last_frame = 0;
}
