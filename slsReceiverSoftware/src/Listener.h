// SPDX-License-Identifier: LGPL-3.0-or-other
// Copyright (C) 2021 Contributors to the SLS Detector Package
#pragma once
/************************************************
 * @file Listener.h
 * @short creates the listener thread that
 * listens to udp sockets, writes data to memory
 * & puts pointers to their memory addresses into fifos
 ***********************************************/
/**
 *@short creates & manages a listener thread each
 */

#include "ThreadObject.h"
#include "receiver_defs.h"
#include "sls/PacketStream.h"
#include "sls/UdpRxSocket.h"
#include <atomic>
#include <memory>

class GeneralData;
class Fifo;

class Listener : private virtual slsDetectorDefs, public ThreadObject {

  public:
    using AnyCPUAffinity = sls::CPUAffinity::AnyCPUAffinity;

    using Ptr = std::shared_ptr<Listener>;

    /**
     * Constructor
     * Calls Base Class CreateThread(), sets ErrorMask if error and increments
     * NumberofListerners
     * @param ind self index
     * @param dtype detector type
     * @param f address of Fifo pointer
     * @param s pointer to receiver status
     * @param portno pointer to udp port number
     * @param e ethernet interface
     * @param us pointer to udp socket buffer size
     * @param as pointer to actual udp socket buffer size
     * @param fdp frame discard policy
     * @param sm pointer to silent mode
     * @param rrnb number of round-robin recvs
     * @param rridx index of round-robin recv
     */
    Listener(int ind, detectorType dtype, Fifo *f, std::atomic<runStatus> *s,
             uint32_t *portno, std::string *e, int *us, int *as,
             frameDiscardPolicy *fdp, bool *sm, int rrnb, int rridx);

    /**
     * Destructor
     * Calls Base Class DestroyThread() and decrements NumberofListerners
     */
    ~Listener();

    /**
     * Get Packets caught
     * @return Packets caught
     */
    uint64_t GetPacketsCaught() const;

    /**
     * Get First Frame Caught
     * @return first frame
     */
    uint64_t GetFirstFrameCaught() const;

    /**
     * Get Frames Caught for each real time acquisition
     * (eg. for each scan)
     * @return number of frames caught for each scan
     */
    uint64_t GetNumFramesCaught() const;

    /**
     * Get Frames Complete Caught for each real time acquisition
     * (eg. for each scan)
     * @return number of complete frames caught for each scan
     */
    uint64_t GetNumCompleteFramesCaught() const;

    /**
     * Get Last Frame index caught
     * @return last frame index caught
     */
    uint64_t GetLastFrameIndexCaught() const;

    /**
     * Get Last Frame Timestamp
     * @return last frame timestamp
     */
    sls::FrameTimestamp GetLastFrameTimestamp() const;

    /** Get  number of missing packets */
    uint64_t GetNumMissingPacket(bool stoppedFlag, uint64_t numPackets);

    uint64_t GetCurrentFrameIndex();
    /** (-1 if no frames have been caught) */
    uint64_t GetListenedIndex();

    /**
     * Set Fifo pointer to the one given
     * @param f address of Fifo pointer
     */
    void SetFifo(Fifo *f);

    /**
     * Reset parameters for new acquisition
     */
    void ResetParametersforNewAcquisition();

    /**
     * Set GeneralData pointer to the one given
     * @param g address of GeneralData (Detector Data) pointer
     */
    void SetGeneralData(GeneralData *g);

    /**
     * Creates UDP Sockets
     */
    void CreateUDPSockets();

    /**
     * Shuts down and deletes UDP Sockets
     */
    void ShutDownUDPSocket();

    /**
     * Create & closes a dummy UDP socket
     * to set & get actual buffer size
     * @param s UDP socket buffer size to be set
     */
    void CreateDummySocketForUDPSocketBufferSize(int s);

    /**
     * Set receiver threads CPU affinity mask
     */
    void SetThreadCPUAffinity(AnyCPUAffinity cpu_affinity);

  private:
    /**
     * Thread Execution for Listener Class
     * Pop free addresses, listen to udp socket,
     * write to memory & push the address into fifo
     */
    void ThreadExecution() override;

    /** type of thread */
    static const std::string TypeName;

    /** GeneralData (Detector Data) object */
    GeneralData *generalData{nullptr};

    /** Fifo structure */
    Fifo *fifo;

    // individual members
    /** Detector Type */
    detectorType myDetectorType;

    /** Receiver Status */
    std::atomic<runStatus> *status;

    /** UDP Socket - Detector to Receiver */
    std::shared_ptr<sls::UdpRxSocket> udpSocket{nullptr};

    /** UDP Port Number */
    uint32_t *udpPortNumber;

    /** ethernet interface */
    std::string *eth;

    /** UDP Socket Buffer Size */
    int *udpSocketBufferSize;

    /** actual UDP Socket Buffer Size (double due to kernel bookkeeping) */
    int *actualUDPSocketBufferSize;

    /** frame discard policy */
    frameDiscardPolicy *frameDiscardMode;

    /** Silent Mode */
    bool *silentMode;

    /** packet stream **/
    std::shared_ptr<AnyPacketStream> packetStream;

    /** packet stream summary **/
    uint64_t packetsCaught{0};
    uint64_t firstFrameCaught{0};
    uint64_t numFramesCaught{0};
    uint64_t numCompleteFramesCaught{0};
    uint64_t lastFrameIndexCaught{0};

    /** if the udp socket is connected */
    std::atomic<bool> udpSocketAlive{false};

    /** frame assembler CPU affinity **/
    AnyCPUAffinity cpuAffinity;

    /** Round-Robin */
    int rrNbRecvs;
    int rrRecvIdx;
};
