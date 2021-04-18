#pragma once
#include "sls/FrameAssembler.h"
#include "sls/sls_detector_defs.h"
#include <memory>

class ClientInterface;

namespace sls {

using namespace FrameAssembler;

class Receiver : private virtual slsDetectorDefs {

  public:
    /**
     * Constructor
     * Starts up a Receiver server. Reads configuration file, options, and
     * assembles a Receiver using TCP and UDP detector interfaces
     * throws an exception in case of failure
     * @param argc from command line
     * @param argv from command line
     */
    Receiver(int argc, char *argv[]);

    /**
     * Constructor
     * Starts up a Receiver server. Reads configuration file, options, and
     * assembles a Receiver using TCP and UDP detector interfaces
     * throws an exception in case of failure
     * @param tcpip_port_no TCP/IP port number
     */
    Receiver(int tcpip_port_no = 1954);

    ~Receiver();

    /**
     * get get Receiver Version
     \returns id
     */
    int64_t getReceiverVersion();

    /**
     * Call back for start acquisition
     * callback arguments are
     * filepath
     * filename
     * fileindex
     * datasize
     *
     * return value is undefined at the moment
     * we write depending on file write enable
     * users get data to write depending on call backs registered
     */
    void registerCallBackStartAcquisition(int (*func)(std::string, std::string,
                                                      uint64_t, uint32_t,
                                                      void *),
                                          void *arg);

    /**
     * Call back for acquisition finished
     * callback argument is
     * total frames caught
     */
    void registerCallBackAcquisitionFinished(void (*func)(uint64_t, void *),
                                             void *arg);

    /**
     * Call back for raw data
     * args to raw data ready callback are
     * sls_receiver_header frame metadata,
     * dataPointer is the pointer to the data,
     * dataSize in bytes is the size of the data in bytes.
     */
    void registerCallBackRawDataReady(void (*func)(char *, char *, uint32_t,
                                                   void *),
                                      void *arg);

    /**
     * Call back for raw data (modified)
     * args to raw data ready callback are
     * sls_receiver_header frame metadata,
     * dataPointer is the pointer to the data,
     * revDatasize is the reference of data size in bytes.
     * Can be modified to the new size to be written/streamed. (only smaller
     * value).
     */
    void registerCallBackRawDataModifyReady(void (*func)(char *, char *,
                                                         uint32_t &, void *),
                                            void *arg);

    /**
     * Set the passive mode status. Takes effect in next (re)configuration
     */
    void setPassiveMode(bool passive);

    /**
     * Set thread CPU Affinity: a list of CPU masks affecting the
     * Listener PacketStream writing threads, one per port
     */
    void setThreadCPUAffinity(const CPUMaskList &cpu_masks);

    /**
     * Set the NUMA node affinity for PacketStream buffers
     * max_node is the maximum valid node in the bitmask
     */
    void setBufferNodeAffinity(unsigned long buffer_node_mask, int max_node);

    /**
     * Create the FrameAssembler for the current detector configuration
     */
    MPFrameAssemblerPtr CreateFrameAssembler(AssemblerType asm_type);

    /**
     * Get the next available packet blocks from the active UDP ports
     */
    AnyPacketBlockList GetFramePacketBlocks();

    /**
     * Clear the PacketStream buffers
     */
    void clearAllBuffers();

  private:
    std::unique_ptr<ClientInterface> tcpipInterface;
};

} // namespace sls
