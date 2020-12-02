#pragma once
#include "sls/sls_detector_defs.h"
#include <memory>

class ClientInterface;

namespace sls {

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
     * Enable / disable gap (image reconstruction)
     */
    void enableGap(bool enable);

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
     * Get the image corresponding to the specified frame in image_data
     * Copy the first header of the frame and return the validity of each
     * port data in portsMask
     */
    int getImage(slsDetectorDefs::receiver_image_data &image_data);

    /**
     * Clear the PacketStream buffers
     */
    void clearAllBuffers();

  private:
    std::unique_ptr<ClientInterface> tcpipInterface;
};

} // namespace sls
