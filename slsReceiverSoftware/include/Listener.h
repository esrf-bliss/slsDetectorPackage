/************************************************
 * @file Listener.h
 * @short creates the listener thread that
 * listens to udp sockets, writes data to memory
 * & puts pointers to their memory addresses into fifos
 ***********************************************/
#ifndef LISTENER_H
#define LISTENER_H
/**
 *@short creates & manages a listener thread each
 */

#include "ThreadObject.h"

class GeneralData;
class Fifo;
class genericSocket;

class Listener : private virtual slsReceiverDefs, public ThreadObject {
	
 public:
	/**
	 * Constructor
	 * Calls Base Class CreateThread(), sets ErrorMask if error and increments NumberofListerners
	 * @param f address of Fifo pointer
	 * @param s pointer to receiver status
	 * @param portno pointer to udp port number
	 * @param e ethernet interface
	 */
	Listener(Fifo*& f, runStatus* s, uint32_t* portno, char* e);

	/**
	 * Destructor
	 * Calls Base Class DestroyThread() and decrements NumberofListerners
	 */
	~Listener();


	//*** static functions ***
	/**
	 * Get ErrorMask
	 * @return ErrorMask
	 */
	static uint64_t GetErrorMask();

	/**
	 * Get RunningMask
	 * @return RunningMask
	 */
	static uint64_t GetRunningMask();

	/**
	 * Set GeneralData pointer to the one given
	 * @param g address of GeneralData (Detector Data) pointer
	 */
	static void SetGeneralData(GeneralData*& g);



	//*** non static functions ***
	//*** getters ***
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
	 * Get Total Packets caught in an acquisition
	 * @return Total Packets caught in an acquisition
	 */
	uint64_t GetTotalPacketsCaught();

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
	 * Creates UDP Sockets
	 * @return OK or FAIL
	 */
	int CreateUDPSockets();

	/**
	 * Shuts down and deletes UDP Sockets
	 */
	void ShutDownUDPSocket();




 private:

	/**
	 * Get Type
	 * @return type
	 */
	std::string GetType();

	/**
	 * Returns if the thread is currently running
	 * @returns true if thread is running, else false
	 */
	bool IsRunning();

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
	 * @param buffer
	 * @returns number of bytes of relevant data, can be image size or 0
	 */
	uint32_t ListenToAnImage(char* buf);



	/** type of thread */
	static const std::string TypeName;

	/** Total Number of Listener Objects */
	static int NumberofListeners;

	/** Mask of errors on any object eg.thread creation */
	static uint64_t ErrorMask;

	/** Mask of all listener objects running */
	static uint64_t RunningMask;

	/** Mutex to update static items among objects (threads)*/
	static pthread_mutex_t Mutex;

	/** GeneralData (Detector Data) object */
	static const GeneralData* generalData;

	/** Fifo structure */
	Fifo* fifo;


	// individual members
	/** Aquisition Started flag */
	bool acquisitionStartedFlag;

	/** Measurement Started flag */
	bool measurementStartedFlag;

	/** Receiver Status */
	runStatus* status;

	/** UDP Sockets - Detector to Receiver */
	genericSocket* udpSocket;

	/** UDP Port Number */
	uint32_t* udpPortNumber;

	/** ethernet interface */
	char* eth;

	/**Number of complete Packets caught for an entire acquisition (including all scans) */
	uint64_t numTotalPacketsCaught;

	/** Number of complete Packets caught for each real time acquisition (eg. for each scan) */
	uint64_t numPacketsCaught;

	/** Frame Number of First Frame of an entire Acquisition (including all scans) */
	uint64_t firstAcquisitionIndex;

	/** Frame Number of First Frame for each real time acquisition (eg. for each scan) */
	uint64_t firstMeasurementIndex;

	/** Current Frame Index, default value is 0
	 * ( always check acquisitionStartedFlag for validity first)
	 */
	uint64_t currentFrameIndex;

	/** Last Frame Index caught  from udp network */
	uint64_t lastCaughtFrameIndex;

	/** True if there is a packet carry over from previous Image */
	bool carryOverFlag;

	/** Carry over packet buffer */
	char* carryOverPacket;

};

#endif
