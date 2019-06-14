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

/**
 *@short manages packet stream with carry-over functionality
 */

class PacketStream {

 public:
	static const int PSMaxNbPackets = 256;

	typedef slsReceiverDefs::sls_detector_header DetHeader;

	struct Packet {
		bool valid;
		DetHeader *header;
		uint64_t frame;
		int number;
		char *buf;
		PacketStream *pstream;
		int idx;

		Packet();
		~Packet();

		void setValid(DetHeader *h, char *b, uint64_t f, int n,
			      PacketStream *ps, int i);

		void release();
	private:
		void forget();
	};

	PacketStream(genericSocket *s, GeneralData *d, cpu_set_t cpu_mask,
		     unsigned long node_mask, int max_node);
	~PacketStream();

	void getPacket(Packet& ret, uint64_t frame, int num = -1);
	bool hasPendingPacket();
	void stop();

 private:
	friend struct Packet;

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

	class MmapMem
	{
	public:
		MmapMem(size_t size = 0, unsigned long node_mask = 0,
			int max_node = 0);
		~MmapMem();

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

	void initMem(unsigned long node_mask, int max_node);
	char *packetPtr(int idx);
	bool releaseBlockedPacket(int idx);
	int getIndex(int index);
	void incIndex(int& index);

	void initThread();
	static void *threadFunctionStatic(void *data);
	void threadFunction();

	genericSocket *socket;
	GeneralData *general_data;
	const int nb_packets;
	int header_pad;
	int packet_len;
	MmapMem packet;
	int nb_blocked_packets;
	std::bitset<PSMaxNbPackets> waiting_mask;
	bool odd_numbering;
	bool first_packet;
	volatile bool stopped;
	int read_idx;
	Semaphore write_sem;
	Semaphore read_sem;
	cpu_set_t cpu_aff_mask;
	pthread_t thread;
};


/**
 *@short Default frame assembler in Listener
 */

class EigerStdFrameAssembler;

class DefaultFrameAssembler {

 public:
	typedef slsReceiverDefs::sls_receiver_header RecvHeader;
	typedef slsReceiverDefs::frameDiscardPolicy FramePolicy;

	DefaultFrameAssembler(genericSocket *s, GeneralData *d,
			      cpu_set_t cpu_mask,
			      unsigned long node_mask, int max_node, 
			      FramePolicy fp, bool e4b);
	~DefaultFrameAssembler();

	int assembleFrame(uint64_t frame, RecvHeader *header, char *buf);

	void stop();

	bool hasPendingPacket();
	int getImageSize();

 protected:
	friend class EigerStdFrameAssembler;

	typedef PacketStream *PacketStreamPtr;
	typedef PacketStream::Packet Packet;

	bool canDiscardFrame(int num_packets);
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
	typedef slsReceiverDefs::sls_receiver_header RecvHeader;
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
	typedef DefaultFrameAssembler::FramePolicy FramePolicy;
	typedef PacketStream::Packet Packet;

	EigerStdFrameAssembler(DefaultFrameAssembler *a[2], bool flipped);
	~EigerStdFrameAssembler();

	virtual PortsMask assembleFrame(uint64_t frame, RecvHeader *recv_header,
					char *buf);

	class Helper;
 protected:
	typedef Helper *HelperPtr;
	HelperPtr helper;
};

