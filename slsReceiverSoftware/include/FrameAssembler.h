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
#include <queue>
#include <map>
#include <memory>

#include <sys/mman.h>
#include <numaif.h>

#include "ThreadUtils.h"
#include "Stats.h"

namespace FrameAssembler
{

typedef slsReceiverDefs::sls_detector_header DetHeader;
typedef slsReceiverDefs::sls_receiver_header RecvHeader;
typedef slsReceiverDefs::frameDiscardPolicy FramePolicy;


template <typename T>
class AutoPtr
{
public:
	AutoPtr(T *p = NULL) : ptr(p)
	{}

	~AutoPtr()
	{ _delete(); }

	AutoPtr& operator =(T *p)
	{
		_delete();
		ptr = p;
	}

	T *forget()
	{
		T *p = ptr;
		ptr = NULL;
		return p;
	}

	operator T*() const
	{ return ptr; }

	T *operator ->() const
	{ return ptr; }

private:
	void _delete()
	{
		delete ptr;
		ptr = NULL;
	}

	T *ptr;
};

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

	void clear();

private:
	char *ptr;
	size_t len;
};

/**
 *@short Packet helper classes
 */

class PacketStream;

const int MaxBufferFrames = 4;

struct Packet {
	struct StreamInfo {
		GeneralData *gd;
		bool odd_numbering;
		bool first_packet;

		StreamInfo(GeneralData *d)
			: gd(d), odd_numbering(false), first_packet(true)
		{}
	};

	char *buffer;
	StreamInfo *stream_info;
	bool std;
	DetHeader *header;
	char *data;
	uint64_t non_std_frame;
	uint32_t non_std_number;

	Packet();
	Packet(char *b, StreamInfo *si);

	GeneralData *gd()
	{ return stream_info->gd; }

	bool valid()
	{ return buffer; }

	uint64_t frame()
	{ return std ? header->frameNumber : non_std_frame; }

	uint32_t number()
	{ return std ? header->packetNumber : non_std_number; }

	uint32_t size_adjust()
	{
		if (gd()->myDetectorType != slsReceiverDefs::GOTTHARD)
			return 0;
		return ((number() == 0) ? -2 : 2);
	}
};

struct PacketBlock {
	std::vector<Packet> packet;
	int valid_packets;

	int size()
	{ return packet.size(); }

	PacketBlock(int l, PacketStream *s);
	~PacketBlock();

	Packet& operator[](unsigned int i);

	void addPacket(Packet& p);

	bool full_frame()
	{ return valid_packets == size(); }

private:
	friend class PacketStream;
	PacketStream *ps;
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

	PacketBlock *getPacketBlock(uint64_t frame);

	bool hasPendingPacket();
	void stop();

	int getNumPacketsCaught();
	uint64_t getNumFramesCaught();
	uint64_t getLastFrameIndex();

	void clearBuffer();

 private:
	friend class PacketBlock;

	struct WriterData;
	typedef std::map<uint64_t, PacketBlock *> PacketBlockMap;
	typedef PacketBlockMap::iterator MapIterator;
	typedef PacketBlockMap::value_type FramePacketBlock;

	void initMem(unsigned long node_mask, int max_node);

	void initThread();
	static void *threadFunctionStatic(void *data);
	void threadFunction();

	bool canDiscardFrame(int received_packets);

	void releasePacketBlock(PacketBlock *block);

	void addPacketBlock(FramePacketBlock frame_block);
	bool processOnePacket(WriterData *wd);

	genericSocket *socket;
	GeneralData *general_data;
	FramePolicy frame_policy;
	const int tot_num_packets;
	Mutex mutex;
	int packets_caught;
	uint64_t frames_caught;
	uint64_t last_frame;
	Packet::StreamInfo stream_info;
	int header_pad;
	int packet_len;
	MmappedRegion packet;
	Cond free_cond;
	std::queue<char *> free_queue;
	bool stopped;
	Cond block_cond;
	cpu_set_t cpu_aff_mask;
	XYStat packet_delay_stat;
	PacketBlockMap packet_block_map;
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

	void clearBuffers();

 protected:
	friend class EigerStdFrameAssembler;

	typedef PacketStream *PacketStreamPtr;

	bool doExpand4Bits();
	void expand4Bits(char *dst, char *src, int src_size);

	PacketStreamPtr packet_stream;
	GeneralData *general_data;
	FramePolicy frame_policy;
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
