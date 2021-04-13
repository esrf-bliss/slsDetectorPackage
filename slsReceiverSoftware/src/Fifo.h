#pragma once
/************************************************
 * @file Fifo.h
 * @short constructs the fifo structure
 * which is a circular buffer with pointers to
 * parts of allocated memory
 ***********************************************/
/**
 *@short constructs the fifo structure
 */

#include "GeneralData.h"
#include "PacketContainer.h"
#include "receiver_defs.h"
#include "sls/CircularFifo.h"
#include "sls/logger.h"
#include "sls/sls_detector_defs.h"

class Fifo : private virtual slsDetectorDefs {

  public:
    /**
     * Constructor
     * Calls CreateFifos that creates fifos and allocates memory
     * @param ind self index
     * @param gd Pointer to GeneralData
     * @param depth fifo depth
     */
    Fifo(int ind, GeneralDataPtr gd, uint32_t depth, unsigned long node_mask,
         int max_node);

    /**
     * Set fifo node affinity mask
     */
    void SetNodeAffinity(unsigned long fifo_node_mask, int max_node);

    /**
     * Destructor
     */
    ~Fifo();

    /**
     * Get frame packets
     */
    AnyPacketBlockPtr GetFramePackets(uint64_t frame = uint64_t(-1));

    /**
     * Frees the bound frame by pushing into fifoFree
     */
    void FreeFrame(FifoFrame *frame);

    /**
     * Pops free frame from fifoFree
     */
    void GetNewFrame(FifoFrame *&frame);

    /**
     * Pushes bound frame into fifoBound
     */
    void PushFrame(FifoFrame *frame);

    /**
     * Pops bound frame from fifoBound to process data
     */
    void PopFrame(FifoFrame *&frame);

    /**
     * Pushes bound frame into fifoStream
     */
    void PushFrameToStream(FifoFrame *frame);

    /**
     * Pops bound frame from fifoStream to stream data
     */
    void PopFrameToStream(FifoFrame *&frame);

    /**
     * Get Maximum Level filled in Fifo Bound
     * and reset this value for next intake
     */
    int GetMaxLevelForFifoBound();

    /**
     * Get Minimum Level filled in Fifo Free
     * and reset this value to max for next intake
     */
    int GetMinLevelForFifoFree();

    /**
     * Get the Fifo Frame Size
     */
    size_t GetFifoFrameSize();

    /**
     * Get the packet container pointer
     */
    AnyPacketContainerPtr GetPacketContainer();

    /**
     * Clear all buffers
     */
    void ClearAllBuffers();

  private:
    /**
     * Create Fifos, allocate memory & push addresses into fifo
     * @param gd Pointer to GeneralData
     */
    void CreateFifos(GeneralDataPtr gd, unsigned long node_mask, int max_node);

    /**
     * Destroy Fifos and deallocate memory
     */
    void DestroyFifos();

    /** Self Index */
    int index;

    /** Memory allocated, whose addresses are pushed into the fifos */
    char *memory;

    /** packet container **/
    AnyPacketContainerPtr packetContainer;

    /** Circular Fifo pointing to addresses of bound data in memory */
    sls::CircularFifo<FifoFrame *> *fifoBound;

    /** Circular Fifo pointing to addresses of freed data in memory */
    sls::CircularFifo<FifoFrame *> *fifoFree;

    /** Circular Fifo pointing to addresses of to be streamed data in memory */
    sls::CircularFifo<FifoFrame *> *fifoStream;

    /** Fifo depth set */
    int fifoDepth;

    /** Fifo frame size */
    size_t fifoFrameSize{0};

    volatile int status_fifoBound;
    volatile int status_fifoFree;
};
