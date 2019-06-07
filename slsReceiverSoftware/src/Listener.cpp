/************************************************
 * @file Listener.cpp
 * @short creates the listener thread that
 * listens to udp sockets, writes data to memory
 * & puts pointers to their memory addresses into fifos
 ***********************************************/


#include "Listener.h"
#include "GeneralData.h"
#include "Fifo.h"
#include "genericSocket.h"

#include <iostream>
#include <errno.h>
#include <cstring>

const std::string Listener::TypeName = "Listener";


Listener::Listener(int ind, detectorType dtype, Fifo*& f, volatile runStatus* s,
        uint32_t* portno, char* e, uint64_t* nf, uint32_t* dr,
        uint32_t* us, uint32_t* as, uint32_t* fpf,
		   frameDiscardPolicy* fdp, bool* act, bool* depaden, bool* sm,
		   bool do_udp_read) :
		ThreadObject(ind),
		runningFlag(0),
		generalData(0),
		fifo(f),
		myDetectorType(dtype),
		status(s),
		udpSocket(0),
		udpPortNumber(portno),
		eth(e),
		numImages(nf),
		dynamicRange(dr),
		udpSocketBufferSize(us),
		actualUDPSocketBufferSize(as),
		framesPerFile(fpf),
		frameDiscardMode(fdp),
		activated(act),
		deactivatedPaddingEnable(depaden),
		silentMode(sm),
		row(0),
		column(0),
		acquisitionStartedFlag(false),
		measurementStartedFlag(false),
		firstAcquisitionIndex(0),
		firstMeasurementIndex(0),
		numPacketsCaught(0),
		lastCaughtFrameIndex(0),
		currentFrameIndex(0),
		udpSocketAlive(0),
		numPacketsStatistic(0),
		numFramesStatistic(0),
		doUdpRead(do_udp_read),
		frameAssembler(0),
		fifoNodeMask(0),
		maxNode(0)
{
	CPU_ZERO(&cpuMask);

	if (doUdpRead && (ThreadObject::CreateThread() == FAIL))
	    throw std::exception();

	FILE_LOG(logDEBUG) << "Listener " << ind << " created";
}


Listener::~Listener() {
	if (udpSocket){
		if (frameAssembler)
			delete frameAssembler;
		sem_post(&semaphore_socket);
		delete udpSocket;
		sem_destroy(&semaphore_socket);
	} 
	if (doUdpRead)
		ThreadObject::DestroyThread();
}

/** getters */
std::string Listener::GetType(){
	return TypeName;
}

bool Listener::IsRunning() {
	return runningFlag;
}

bool Listener::GetAcquisitionStartedFlag(){
	return acquisitionStartedFlag;
}

bool Listener::GetMeasurementStartedFlag(){
	return measurementStartedFlag;
}

uint64_t Listener::GetPacketsCaught() {
	return numPacketsCaught;
}

uint64_t Listener::GetLastFrameIndexCaught() {
	return lastCaughtFrameIndex;
}

/** setters */
void Listener::StartRunning() {
    runningFlag = true;
}


void Listener::StopRunning() {
    runningFlag = false;
}


void Listener::SetFifo(Fifo*& f) {
	fifo = f;
}


void Listener::ResetParametersforNewAcquisition() {
	acquisitionStartedFlag = false;
	firstAcquisitionIndex = 0;
	currentFrameIndex = 0;
	lastCaughtFrameIndex = 0;
}


void Listener::ResetParametersforNewMeasurement() {
    runningFlag = false;
	measurementStartedFlag = false;
	numPacketsCaught = 0;
	firstMeasurementIndex = 0;
	numPacketsStatistic = 0;
	numFramesStatistic = 0;
	//reset fifo statistic
	fifo->GetMaxLevelForFifoBound();
	fifo->GetMinLevelForFifoFree();
}



void Listener::RecordFirstIndices(uint64_t fnum) {
	//listen to this fnum, later +1
	currentFrameIndex = fnum;

	measurementStartedFlag = true;
	firstMeasurementIndex = fnum;

	//start of entire acquisition
	if (!acquisitionStartedFlag) {
		acquisitionStartedFlag = true;
		firstAcquisitionIndex = fnum;
	}

	if(!(*silentMode)) {
		if (!index) cprintf(BLUE,"%d First Acquisition Index:%lu\n"
				"%d First Measurement Index:%lu\n",
				index, firstAcquisitionIndex,
				index, firstMeasurementIndex);
	}
}


void Listener::SetGeneralData(GeneralData*& g) {
	generalData = g;
#ifdef VERY_VERBOSE
	generalData->Print();
#endif
}


int Listener::SetThreadPriority(int priority) {
	struct sched_param param;
	param.sched_priority = priority;
	if (pthread_setschedparam(thread, SCHED_FIFO, &param) == EPERM)
		return FAIL;
	FILE_LOG(logINFO) << "Listener Thread Priority set to " << priority;
	return OK;
}

void Listener::SetThreadCPUAffinity(const cpu_set_t& cpu_mask)
{
	cpuMask = cpu_mask;
}

void Listener::SetFifoNodeAffinity(unsigned long fifo_node_mask, int max_node)
{
	fifoNodeMask = fifo_node_mask;
	maxNode = max_node;
	FILE_LOG(logINFO) << "Node mask: " << std::hex << std::showbase << fifo_node_mask
			  << std::dec << ", max_node: " << max_node;
}

int Listener::CreateUDPSockets() {

    if (!(*activated)) {
    	return OK;
    }

	//if eth is mistaken with ip address
	if (strchr(eth,'.') != NULL){
	    memset(eth, 0, MAX_STR_LENGTH);
	}
	if(!strlen(eth)){
		FILE_LOG(logWARNING) << "eth is empty. Listening to all";
	}

	ShutDownUDPSocket();

	try{
		genericSocket* g = new genericSocket(*udpPortNumber, genericSocket::UDP,
				generalData->packetSize, (strlen(eth)?eth:NULL), generalData->headerPacketSize,
				*udpSocketBufferSize);
		udpSocket = g;
		FILE_LOG(logINFO) << index << ": UDP port opened at port " << *udpPortNumber;
	} catch (...) {
		FILE_LOG(logERROR) << "Could not create UDP socket on port " << *udpPortNumber;
		return FAIL;
	}

	frameAssembler = new DefaultFrameAssembler(udpSocket, generalData, cpuMask, fifoNodeMask, maxNode,
					     *frameDiscardMode, !doUdpRead);

	udpSocketAlive = true;
    sem_init(&semaphore_socket,1,0);

    // doubled due to kernel bookkeeping (could also be less due to permissions)
    *actualUDPSocketBufferSize = udpSocket->getActualUDPSocketBufferSize();

	return OK;
}



void Listener::ShutDownUDPSocket() {
	if(udpSocket){
		udpSocketAlive = false;
		if (!doUdpRead)
			StopRunning();
		if (frameAssembler)
			frameAssembler->stop();
		udpSocket->ShutDownSocket();
		FILE_LOG(logINFO) << "Shut down of UDP port " << *udpPortNumber;
		fflush(stdout);
		// wait only if the threads have started as it is the threads that
		//give a post to semaphore(at stopListening)
		if (runningFlag)
		    sem_wait(&semaphore_socket);
		if (frameAssembler)
			delete frameAssembler;
        delete udpSocket;
        udpSocket = 0;
	    sem_destroy(&semaphore_socket);
	}
}


int Listener::CreateDummySocketForUDPSocketBufferSize(uint32_t s) {
    FILE_LOG(logINFO) << "Testing UDP Socket Buffer size with test port " << *udpPortNumber;

    if (!(*activated)) {
    	*actualUDPSocketBufferSize = (s*2);
    	return OK;
    }

    uint32_t temp = *udpSocketBufferSize;
    *udpSocketBufferSize = s;

    //if eth is mistaken with ip address
    if (strchr(eth,'.') != NULL){
        memset(eth, 0, MAX_STR_LENGTH);
    }

    // shutdown if any open
    if(udpSocket){
        udpSocketAlive = false;
        udpSocket->ShutDownSocket();
	if (frameAssembler)
		delete frameAssembler;
        delete udpSocket;
        udpSocket = 0;
    }

    //create dummy socket
    try {
    	udpSocket = new genericSocket(*udpPortNumber, genericSocket::UDP,
            generalData->packetSize, (strlen(eth)?eth:NULL), generalData->headerPacketSize,
            *udpSocketBufferSize);
    } catch (...) {
        FILE_LOG(logERROR) << "Could not create a test UDP socket on port " << *udpPortNumber;
        return FAIL;
    }


    // doubled due to kernel bookkeeping (could also be less due to permissions)
    *actualUDPSocketBufferSize = udpSocket->getActualUDPSocketBufferSize();
    if (*actualUDPSocketBufferSize != (s*2)) {
        *udpSocketBufferSize = temp;
    }


    // shutdown socket
    if(udpSocket){
        udpSocket->ShutDownSocket();
        delete udpSocket;
        udpSocket = 0;
    }

    return OK;
}

void Listener::SetHardCodedPosition(uint16_t r, uint16_t c) {
	row = r;
	column = c;
}

void Listener::ThreadExecution() {
	char* buffer;
	int rc = 0;

	fifo->GetNewAddress(buffer);

	uint32_t* byte_count = (uint32_t*) buffer;
	sls_receiver_header* recv_header = (sls_receiver_header*) (buffer + FIFO_HEADER_NUMBYTES);
	char* image_data = (buffer + FIFO_HEADER_NUMBYTES + sizeof(sls_receiver_header));

	//udpsocket doesnt exist
	bool carryOverFlag = frameAssembler->hasPendingPacket();
	if (*activated && !udpSocketAlive && !carryOverFlag) {
		*byte_count = 0;
		StopListening(buffer);
		return;
	}

	//get data
	if ((*status != TRANSMITTING && (!(*activated) || udpSocketAlive)) || carryOverFlag) {
		rc = ListenToAnImage(recv_header, image_data);
	}


	//error check, (should not be here) if not transmitting yet (previous if) rc should be > 0
	if (rc == 0) {
		//cprintf(RED,"%d Socket shut down while waiting for future packet. udpsocketalive:%d\n",index, udpSocketAlive );
		if (!udpSocketAlive) {
			(*((uint32_t*)buffer)) = 0;
			StopListening(buffer);
		}else
			fifo->FreeAddress(buffer);
		return;
	}

	// discarding image
	else if (rc < 0) {
		FILE_LOG(logDEBUG) <<  index << " discarding fnum:" << currentFrameIndex;
		fifo->FreeAddress(buffer);
		currentFrameIndex++;
		return;
	}

	*byte_count = rc;
	recv_header->detHeader.frameNumber = currentFrameIndex;		//for those returning earlier
	currentFrameIndex++;

	//push into fifo
	fifo->PushAddress(buffer);

	//Statistics
	if(!(*silentMode)) {
		numFramesStatistic++;
		if (numFramesStatistic >=
				//second condition also for infinite #number of frames
				(((*framesPerFile) == 0) ? STATISTIC_FRAMENUMBER_INFINITE : (*framesPerFile)) )
			PrintFifoStatistics();
	}
}


DualPortFrameAssembler *Listener::CreateDualPortFrameAssembler(Listener *listener[2])
{
	if (listener[0]->myDetectorType != slsReceiverDefs::EIGER) {
		FILE_LOG(logERROR) << "EigerFrameAssembler supported only on Eiger";
		return NULL;
	}

	DefaultFrameAssembler *a[2] = {listener[0]->frameAssembler,
				       listener[1]->frameAssembler};
	return new EigerRawFrameAssembler(a);
}


void Listener::StopListening(char* buf) {
	uint32_t* byte_count = (uint32_t *) buf;
	*byte_count = DUMMY_PACKET_VALUE;
	fifo->PushAddress(buf);
	StopRunning();

	 sem_post(&semaphore_socket);
}


/* buf includes the fifo header and packet header */
int Listener::ListenToAnImage(sls_receiver_header* recv_header, char* buf) {

	int rc = 0;
	uint64_t fnum = 0;
	uint32_t numpackets = 0;
	uint32_t hsize = generalData->headerSizeinPacket; //(includes empty header)
	uint32_t esize = generalData->emptyHeader;
	bool isHeaderEmpty = true;
	sls_detector_header* det_header;
	bool standardheader = generalData->standardheader;

	// deactivated (eiger)
	if (!(*activated)) {
		// no padding
		if (!(*deactivatedPaddingEnable))
			return 0;
		// padding without setting bitmask (all missing packets padded in dataProcessor)
		if (currentFrameIndex >= *numImages)
			return 0;

		//(eiger) first fnum starts at 1
		if (!currentFrameIndex) {
			++currentFrameIndex;
		}
		memset(recv_header, 0, sizeof(sls_receiver_header));
		recv_header->detHeader.frameNumber = currentFrameIndex;
		recv_header->detHeader.row = row;
		recv_header->detHeader.column = column;
		recv_header->detHeader.detType = (uint8_t) generalData->myDetectorType;
		recv_header->detHeader.version = (uint8_t) SLS_DETECTOR_HEADER_VERSION;
		return generalData->imageSize;
	}

	fnum = currentFrameIndex;
	rc = frameAssembler->assembleFrame(fnum, recv_header, buf);
	recv_header->detHeader.row = row;
	recv_header->detHeader.column = column;
	if (rc < 0)
		return -1;

	//update parameters
	numpackets = recv_header->detHeader.packetNumber;
	numPacketsCaught += numpackets;
	numPacketsStatistic += numpackets;

	lastCaughtFrameIndex = fnum;

	if (!measurementStartedFlag)
		RecordFirstIndices(fnum);

	return generalData->imageSize;
}




void Listener::PrintFifoStatistics() {
#ifdef VERBOSE
	cout << "numFramesStatistic:" << numFramesStatistic << " numPacketsStatistic:" << numPacketsStatistic << endl;
#endif
	//calculate packet loss
	int64_t loss = -1;
	loss = (numFramesStatistic*(generalData->packetsPerFrame)) - numPacketsStatistic;
	numPacketsStatistic = 0;
	numFramesStatistic = 0;

	if (loss)
		cprintf(RED,"[%u]:  Packet_Loss:%lu  Used_Fifo_Max_Level:%d \tFree_Slots_Min_Level:%d \tCurrent_Frame#:%lu\n",
				*udpPortNumber,loss, fifo->GetMaxLevelForFifoBound() , fifo->GetMinLevelForFifoFree(), currentFrameIndex);
	else
		cprintf(GREEN,"[%u]:  Packet_Loss:%lu  Used_Fifo_Max_Level:%d  \tFree_Slots_Min_Level:%d \tCurrent_Frame#:%lu\n",
				*udpPortNumber,loss, fifo->GetMaxLevelForFifoBound(), fifo->GetMinLevelForFifoFree(), currentFrameIndex);
}
