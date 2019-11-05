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
#include "FrameAssembler.h"

class GeneralData;
class Fifo;
class genericSocket;

class Listener : private virtual slsReceiverDefs, public ThreadObject {
	
 public:
	/**
	 * Constructor
	 * Calls Base Class CreateThread(), sets ErrorMask if error and increments NumberofListerners
	 * @param ind self index
	 * @param dtype detector type
	 * @param f address of Fifo pointer
	 * @param s pointer to receiver status
	 * @param portno pointer to udp port number
	 * @param e ethernet interface
	 * @param nf pointer to number of images to catch
	 * @param dr pointer to dynamic range
	 * @param us pointer to udp socket buffer size
	 * @param as pointer to actual udp socket buffer size
	 * @param fpf pointer to frames per file
	 * @param fdp frame discard policy
	 * @param act pointer to activated
	 * @param depaden pointer to deactivated padding enable
	 * @param sm pointer to silent mode
	 * @param fl pointer to flipped data
	 */
	Listener(int ind, detectorType dtype, Fifo*& f, volatile runStatus* s,
		 uint32_t* portno, char* e, uint64_t* nf, uint32_t* dr,
		 uint32_t* us, uint32_t* as, uint32_t* fpf,
		 frameDiscardPolicy* fdp, bool* act, bool* depaden, bool* sm, int* fl,
		 bool do_udp_read);

	/**
	 * Destructor
	 * Calls Base Class DestroyThread() and decrements NumberofListerners
	 */
	~Listener();


	//*** getters ***
    /**
     * Returns if the thread is currently running
     * @returns true if thread is running, else false
     */
    bool IsRunning();

	/**
	 * Get acquisition started flag
	 * @return acquisition started flag
	 */
	bool GetAcquisitionStartedFlag();

	/**
	 * Get measurement started flag
	 * @return measurement started flag
	 */
	bool GetMeasurementStartedFlag();

	/**
	 * Get Packets caught in a real time acquisition (start and stop of receiver)
	 * @return Packets caught in a real time acquisition
	 */
	uint64_t GetPacketsCaught();

	/**
	 * Get Frames Complete Caught for each real time acquisition (eg. for each scan)
	 * @return number of frames caught for each scan
	 */
	uint64_t GetNumFramesCaught();

	/**
	 * Get Last Frame index caught
	 * @return last frame index caught
	 */
	uint64_t GetLastFrameIndexCaught();


	//*** setters ***
	/**
	 * Set bit in RunningMask to allow thread to run
	 */
	void StartRunning();

	/**
	 * Reset bit in RunningMask to prevent thread from running
	 */
	void StopRunning();

	/**
	 * Set Fifo pointer to the one given
	 * @param f address of Fifo pointer
	 */
	void SetFifo(Fifo*& f);

	/**
	 * Reset parameters for new acquisition (including all scans)
	 */
	void ResetParametersforNewAcquisition();

	/**
	 * Reset parameters for new measurement (eg. for each scan)
	 */
	void ResetParametersforNewMeasurement();

	/**
	 * Set GeneralData pointer to the one given
	 * @param g address of GeneralData (Detector Data) pointer
	 */
	void SetGeneralData(GeneralData*& g);

	/**
	 * Set thread priority
	 * @priority priority
	 * @returns OK or FAIL
	 */
	int SetThreadPriority(int priority);

	/**
	 * Creates UDP Sockets
	 * @return OK or FAIL
	 */
	int CreateUDPSockets();

	/**
	 * Shuts down and deletes UDP Sockets
	 */
	void ShutDownUDPSocket();

    /**
     * Create & closes a dummy UDP socket
     * to set & get actual buffer size
     * @param s UDP socket buffer size to be set
     * @return OK or FAIL of dummy socket creation
     */
    int CreateDummySocketForUDPSocketBufferSize(uint32_t s);

    /**
     * Set hard coded (calculated but not from detector) row and column
     * r is in row index if detector has not send them yet in firmware,
     * c is in col index for jungfrau and eiger (for missing packets/deactivated eiger)
     * c when used is in 2d
     */
    void SetHardCodedPosition(uint16_t r, uint16_t c);

    /**
     * Set receiver threads CPU affinity mask
     */
    void SetThreadCPUAffinity(const cpu_set_t& cpu_mask);

    /**
     * Set receiver fifo node affinity mask
     */
    void SetFifoNodeAffinity(unsigned long fifo_node_mask, int max_node);

    /**
     * Create a dual-port frame assembler object listening on 2 Listeners' UDP ports
     */
    static FrameAssembler::DualPortFrameAssembler *
      CreateDualPortFrameAssembler(Listener *listener[2]);

    /**
     * Clear all buffers
     */
    void ClearAllBuffers();

 private:

	/**
	 * Get Type
	 * @return type
	 */
	std::string GetType();

	/**
	 * Record First Indices (firstAcquisitionIndex, firstMeasurementIndex)
	 * @param fnum frame index to record
	 */
	void RecordFirstIndices(uint64_t fnum);

	/**
	 * Thread Exeution for Listener Class
	 * Pop free addresses, listen to udp socket,
	 * write to memory & push the address into fifo
	 */
	void ThreadExecution();

	/**
	 * Pushes non empty buffers into fifo/ frees empty buffer,
	 * pushes dummy buffer into fifo
	 * and reset running mask by calling StopRunning()
	 * @param buf address of buffer
	 */
	void StopListening(char* buf);

	/**
	 * Listen to the UDP Socket for an image,
	 * place them in the right order
	 * @param recv_header
	 * @param buf
	 * @returns number of bytes of relevant data (image size or 0, if stop acquisition)
	 * or -1 to discard image
	 */
	int ListenToAnImage(sls_receiver_header* recv_header, char* buf);

	/**
	 * Print Fifo Statistics
	 */
	void PrintFifoStatistics();



	/** type of thread */
	static const std::string TypeName;

	/** Object running status */
	bool runningFlag;

	/** GeneralData (Detector Data) object */
	GeneralData* generalData;

	/** Fifo structure */
	Fifo* fifo;


	// individual members
	/** Detector Type */
	detectorType myDetectorType;

	/** Receiver Status */
	volatile runStatus* status;

	/** UDP Socket - Detector to Receiver */
	genericSocket* udpSocket;

	/** UDP Port Number */
	uint32_t* udpPortNumber;

	/** ethernet interface */
	char* eth;

	/** Number of Images to catch */
	uint64_t* numImages;

	/** Dynamic Range */
	uint32_t* dynamicRange;

	/** UDP Socket Buffer Size */
	uint32_t* udpSocketBufferSize;

	/** actual UDP Socket Buffer Size (double due to kernel bookkeeping) */
	uint32_t* actualUDPSocketBufferSize;

	/** frames per file */
	uint32_t* framesPerFile;

	/** frame discard policy */
	frameDiscardPolicy* frameDiscardMode;

	/** Activated/Deactivated */
	bool* activated;

	/** Deactivated padding enable */
	bool* deactivatedPaddingEnable;

    /** Silent Mode */
    bool* silentMode;

	/** Flipped Data */
	int* flippedData;

	/** row hardcoded as 1D or 2d,
	 * if detector does not send them yet or
	 * missing packets/deactivated (eiger/jungfrau sends 2d pos) **/
	uint16_t row;

	/** column hardcoded as 2D,
	 * deactivated eiger/missing packets (eiger/jungfrau sends 2d pos) **/
	uint16_t column;


	// acquisition start
	/** Aquisition Started flag */
	bool acquisitionStartedFlag;

	/** Measurement Started flag */
	bool measurementStartedFlag;

	/** Frame Number of First Frame of an entire Acquisition (including all scans) */
	uint64_t firstAcquisitionIndex;

	/** Frame Number of First Frame for each real time acquisition (eg. for each scan) */
	uint64_t firstMeasurementIndex;


	// parameters to acquire image
	/** Current Frame Index, default value is 0
	 * ( always check acquisitionStartedFlag for validity first)
	 */
	uint64_t currentFrameIndex;

	/** frame assembler **/
	FrameAssembler::DefaultFrameAssembler *frameAssembler;

	/** if the udp socket is connected */
	volatile bool udpSocketAlive;

    /** Semaphore to synchonize deleting udp socket */
    sem_t semaphore_socket;


	// for print progress during acqusition
	/** number of packets for statistic */
	uint32_t numPacketsStatistic;

	/** number of images for statistic */
	uint32_t numFramesStatistic;

	/**
	 * Do read UDP socket
	 */
	bool doUdpRead;

	/** frame assembler CPU affinity **/
	cpu_set_t cpuMask;

	/** Fifo node affinity **/
	unsigned long fifoNodeMask;
	int maxNode;
};

