#pragma once
/************************************************
 * @file FrameAssembler.h
 * @short helper classes assembling frames
 * from udp packets
 ***********************************************/

#include "logger.h"
#include "sls_receiver_defs.h"
#include "GeneralData.h"
#include "genericSocket.h"

#include <vector>
#include <memory>

#include <sys/mman.h>
#include <numaif.h>
#include <semaphore.h>

namespace FrameAssembler
{

typedef slsReceiverDefs::sls_detector_header DetHeader;
typedef slsReceiverDefs::sls_receiver_header RecvHeader;
typedef slsReceiverDefs::frameDiscardPolicy FramePolicy;


/**
 *@short Utility classes
 */

class bad_mmap_alloc : public std::bad_alloc {
public:
	bad_mmap_alloc(const char *m = "") : msg(m)
	{}
	virtual ~bad_mmap_alloc() throw()
	{}
	virtual const char* what() const throw()
	{ return msg.c_str(); }
private:
	std::string msg;
};

class MmappedRegion
{
public:
	MmappedRegion(size_t size = 0, unsigned long node_mask = 0,
		      int max_node = 0);
	~MmappedRegion();

	void alloc(size_t size, unsigned long node_mask = 0,
		   int max_node = 0);
	void release();
	char *getPtr();

private:
	char *ptr;
	size_t len;
};

class Semaphore
{
public:
	Semaphore(int n);
	~Semaphore();
	void post();
	void wait();

private:
	sem_t sem;
};

class Cond;

/**
 * Mutex
 */

class Mutex
{
 public:
	Mutex()
	{
		if (pthread_mutex_init(&mutex, NULL) != 0)
			throw std::runtime_error("Could not init mutex");
	}

	~Mutex()
	{
		pthread_mutex_destroy(&mutex);
	}

	void lock()
	{
		if (pthread_mutex_lock(&mutex) != 0)
			throw std::runtime_error("Error locking mutex");
	}

	void unlock()
	{
		if (pthread_mutex_unlock(&mutex) != 0)
			throw std::runtime_error("Error unlocking mutex");
	}

 private:
	friend class Cond;
	pthread_mutex_t mutex;
};


/**
 * MutexLock
 */

class MutexLock
{
 public:
	MutexLock(Mutex& m)
	: mutex(m)
	{ mutex.lock(); }

	~MutexLock()
	{ mutex.unlock(); }

 private:
	Mutex& mutex;
};


/**
 * Cond
 */

class Cond
{
 public:
	Cond()
	{
		if (pthread_cond_init(&cond, NULL) != 0)
			throw std::runtime_error("Could not init cond");
	}

	~Cond()
	{
		pthread_cond_destroy(&cond);
	}

	Mutex& getMutex()
	{
		return mutex;
	}

	void wait()
	{
		if (pthread_cond_wait(&cond, &mutex.mutex) != 0)
			throw std::runtime_error("Could not wait on cond");
	}

	void signal()
	{
		if (pthread_cond_signal(&cond) != 0)
			throw std::runtime_error("Could not signal cond");
	}

 private:
	Mutex mutex;
	pthread_cond_t cond;
};


/**
 *@short Packet helper classes
 */

class PacketStream;

const int MaxBufferFrames = 4;
const int MaxBufferPackets = MaxBufferFrames * 512;

struct Packet {
	bool valid;
	DetHeader *header;
	uint64_t frame;
	int number;
	char *buf;

	void setValid(DetHeader *h, char *b, uint64_t f, int n);
	void setInvalid();
};

struct PacketBlock {
	Packet packet[MaxBufferPackets];
	const int len;

	PacketBlock(int l);
	~PacketBlock();

	Packet& operator[](int i);

private:
	friend class PacketStream;
	PacketStream *ps;
	int idx;
};


/**
 *@short manages packet stream with buffer & parallel read functionality
 */

class PacketStream {

 public:
	PacketStream(genericSocket *s, GeneralData *d, FramePolicy fp,
		     cpu_set_t cpu_mask,
		     unsigned long node_mask, int max_node);
	~PacketStream();

	int getPacketBlock(PacketBlock& block, uint64_t frame);

	bool hasPendingPacket();
	void stop();

	int getNumPacketsCaught();
	uint64_t getNumFramesCaught();
	uint64_t getLastFrameIndex();

 private:
	friend class PacketBlock;

	void initMem(unsigned long node_mask, int max_node);
	char *packetPtr(int idx);
	int getIndex(int index);
	void incIndex(int& index);
	int getIndexDiff(int a, int b);

	void initThread();
	static void *threadFunctionStatic(void *data);
	void threadFunction();

	bool canDiscardFrame(int received_packets);
	int getNextPacket(Packet& np, uint64_t frame, int pnum);

	void releasePacketBlock(PacketBlock& block);

	genericSocket *socket;
	GeneralData *general_data;
	FramePolicy frame_policy;
	const int tot_num_packets;
	volatile int packets_caught;
	volatile uint64_t frames_caught;
	volatile uint64_t last_frame;
	int header_pad;
	int packet_len;
	MmappedRegion packet;
	int num_blocked_packets;
	std::bitset<MaxBufferPackets> waiting_mask;
	bool odd_numbering;
	bool first_packet;
	volatile bool stopped;
	int read_idx;
	Semaphore write_sem;
	Semaphore read_sem;
	bool in_get_block;
	Cond block_cond;
	cpu_set_t cpu_aff_mask;
	pthread_t thread;
};


/**
 *@short Default frame assembler in Listener
 */

class EigerStdFrameAssembler;

class DefaultFrameAssembler {

 public:
	DefaultFrameAssembler(genericSocket *s, GeneralData *d,
			      cpu_set_t cpu_mask,
			      unsigned long node_mask, int max_node, 
			      FramePolicy fp, bool e4b);
	~DefaultFrameAssembler();

	int assembleFrame(uint64_t frame, RecvHeader *header, char *buf);

	void stop();

	bool hasPendingPacket();
	int getImageSize();

	int getNumPacketsCaught();
	uint64_t getNumFramesCaught();
	uint64_t getLastFrameIndex();

 protected:
	friend class EigerStdFrameAssembler;

	typedef PacketStream *PacketStreamPtr;

	bool doExpand4Bits();
	void expand4Bits(char *dst, char *src, int src_size);

	PacketStreamPtr packet_stream;
	GeneralData *general_data;
	FramePolicy frame_policy;
	volatile bool stopped;
	bool expand_4bits;
};


/**
 *@short assembles frame data from 2 udp sockets into memory
 */

class DualPortFrameAssembler {

 public:
	typedef std::bitset<2> PortsMask;

	DualPortFrameAssembler(DefaultFrameAssembler *a[2]);
	virtual ~DualPortFrameAssembler();

	virtual void stop();

	virtual PortsMask assembleFrame(uint64_t frame, RecvHeader *header,
					char *buf) = 0;

 protected:
	void stopAssemblers();

	DefaultFrameAssembler *assembler[2];
};


/**
 *@short Eiger frame assembler in raw mode: 2x Default frame assemblers
 */

class EigerRawFrameAssembler : public DualPortFrameAssembler {

public:
	EigerRawFrameAssembler(DefaultFrameAssembler *a[2]);

	virtual PortsMask assembleFrame(uint64_t frame, RecvHeader *recv_header,
					char *buf);
};

/**
 *@short Eiger frame assembler in std mode: port interleaving
 */

class EigerStdFrameAssembler : public DualPortFrameAssembler {

public:
	EigerStdFrameAssembler(DefaultFrameAssembler *a[2], bool flipped);
	~EigerStdFrameAssembler();

	virtual PortsMask assembleFrame(uint64_t frame, RecvHeader *recv_header,
					char *buf);

	class Helper;

 protected:
	Helper *helper;
};

} // namespace FrameAssembler
