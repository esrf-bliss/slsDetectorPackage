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
 *@short StdPacket
 */

struct StdPacket {
	struct StreamInfo {
		GeneralData *gd;
		StreamInfo(GeneralData *d)
			: gd(d)
		{}
	};

	struct SoftHeader {
		bool valid;
	};

	char *buffer;
	char *start;

	StdPacket(char *b, StreamInfo *si);

	SoftHeader *softHeader()
	{ return reinterpret_cast<SoftHeader *>(buffer); }

	void initSoftHeader()
	{}

	bool valid()
	{ return softHeader()->valid; }

	char *networkBuffer()
	{ return buffer + sizeof(SoftHeader); }

	DetHeader *header()
	{ return reinterpret_cast<DetHeader *>(start); }
	
	uint64_t frame()
	{ return header()->frameNumber; }

	uint32_t number()
	{ return header()->packetNumber; }

	char *data()
	{ return start + sizeof(DetHeader); }

	uint32_t sizeAdjust()
	{ return 0; }

	void initDetHeader(DetHeader *det_header);
};

/**
 *@short LegacyPacket class
 */

struct LegacyPacket {
	struct StreamInfo {
		GeneralData *gd;
		bool odd_numbering;
		StreamInfo(GeneralData *d)
			: gd(d), odd_numbering(true)
		{}
	};

	struct SoftHeader {
		bool valid;
		uint64_t packet_frame;
		uint32_t packet_number;
	};

	char *buffer;
	char *data_ptr;
	StreamInfo *stream_info;

	LegacyPacket(char *b, StreamInfo *si);

	GeneralData *generalData()
	{ return stream_info->gd; }

	SoftHeader *softHeader()
	{ return reinterpret_cast<SoftHeader *>(buffer); }

	void initSoftHeader();

	bool valid()
	{ return softHeader()->valid; }

	uint64_t frame()
	{ return softHeader()->packet_frame; }

	uint32_t number()
	{ return softHeader()->packet_number; }

	char *networkBuffer()
	{ return buffer + sizeof(SoftHeader); }

	char *data()
	{ return data_ptr; }

	uint32_t sizeAdjust()
	{ return 0; }

	void initDetHeader(DetHeader *det_header);
};

/**
 *@short GotthardPacket class
 */

struct GotthardPacket : public LegacyPacket {
	struct StreamInfo : public LegacyPacket::StreamInfo {
		bool first_packet;

		StreamInfo(GeneralData *d)
			: LegacyPacket::StreamInfo(d), first_packet(true)
		{}

		LegacyPacket::StreamInfo *checkInit(char *b);
	};

	GotthardPacket();
	GotthardPacket(char *b, StreamInfo *si);

	uint32_t sizeAdjust()
	{
		// Gotthard data:
		//   1st packet: CACA + CACA, (640 - 1) * 2 bytes data
		//   2nd packet: (1 + 640) * 2 bytes data
		return ((number() == 0) ? -2 : 2);
	}
};


/**
 *@short Packet Block: reference packets in a frame
 */

template <class P>
class PacketStream;

template <class P>
class PacketBlock {
public:
	PacketBlock(PacketStream<P> *s, char *b);
	~PacketBlock();

	int getNbPackets()
	{ return ps->getNbPacketFrames(); }

	P operator[](unsigned int i);

	void setValid(unsigned int i, bool valid);

	void moveToGood(P& p);

	bool hasFullFrame()
	{ return valid_packets == getNbPackets(); }

	int getValidPackets()
	{ return valid_packets; }

private:
	friend class PacketStream<P>;

	PacketStream<P> *ps;
	char *buf;
	int valid_packets;
};


/**
 *@short manages packet stream with buffer & parallel read functionality
 */

template <class P>
class PacketStream {

 public:
	PacketStream(genericSocket *s, GeneralData *d, FramePolicy fp,
		     cpu_set_t cpu_mask,
		     unsigned long node_mask, int max_node);
	~PacketStream();

	PacketBlock<P> *getPacketBlock(uint64_t frame);

	bool hasPendingPacket();
	void stop();

	int getNumPacketsCaught();
	uint64_t getNumFramesCaught();
	uint64_t getLastFrameIndex();

	void clearBuffer();

 private:
	typedef typename P::StreamInfo StreamInfo;
	friend class PacketBlock<P>;

	struct WriterThread;
	typedef std::map<uint64_t, PacketBlock<P> *> PacketBlockMap;
	typedef typename PacketBlockMap::iterator MapIterator;
	typedef typename PacketBlockMap::value_type FramePacketBlock;

	uint32_t getNbPacketFrames();
	
	void initMem(unsigned long node_mask, int max_node);

	bool canDiscardFrame(int received_packets);

	PacketBlock<P> *getEmptyBlock();
	void addPacketBlock(FramePacketBlock frame_block);
	void releasePacketBlock(PacketBlock<P> *block);

	genericSocket *socket;
	GeneralData *general_data;
	FramePolicy frame_policy;
	const int num_frames;
	Mutex mutex;
	int packets_caught;
	uint64_t frames_caught;
	uint64_t last_frame;
	StreamInfo stream_info;
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
	WriterThread *thread;
};


/**
 *@short Default frame assembler in Listener
 */

class EigerStdFrameAssembler;

class DefaultFrameAssemblerBase {

 public:
	DefaultFrameAssemblerBase(GeneralData *d, bool e4b);
	virtual ~DefaultFrameAssemblerBase();

	GeneralData *getGeneralData();
	bool doExpand4Bits();

	virtual int assembleFrame(uint64_t frame, RecvHeader *header,
				  char *buf) = 0;

	virtual void stop() = 0;

	virtual bool hasPendingPacket() = 0;
	virtual int getImageSize() = 0;

	virtual int getNumPacketsCaught() = 0;
	virtual uint64_t getNumFramesCaught() = 0;
	virtual uint64_t getLastFrameIndex() = 0;

	virtual void clearBuffers() = 0;

	static
	DefaultFrameAssemblerBase *create(genericSocket *s, GeneralData *d,
					  cpu_set_t cpu_mask,
					  unsigned long node_mask,
					  int max_node, 
					  FramePolicy fp, bool e4b);
 protected:
	GeneralData *general_data;
	bool expand_4bits;
};


template <class P>
class DefaultFrameAssembler : public DefaultFrameAssemblerBase {

 public:
	DefaultFrameAssembler(genericSocket *s, GeneralData *d,
			      cpu_set_t cpu_mask,
			      unsigned long node_mask, int max_node, 
			      FramePolicy fp, bool e4b);
	~DefaultFrameAssembler();

	virtual int assembleFrame(uint64_t frame, RecvHeader *header,
				  char *buf);

	virtual void stop();

	virtual bool hasPendingPacket();
	virtual int getImageSize();

	virtual int getNumPacketsCaught();
	virtual uint64_t getNumFramesCaught();
	virtual uint64_t getLastFrameIndex();

	virtual void clearBuffers();

 protected:
	friend class EigerStdFrameAssembler;

	typedef PacketStream<P> *PacketStreamPtr;

	void expand4Bits(char *dst, char *src, int src_size);

	PacketStreamPtr packet_stream;
	FramePolicy frame_policy;
};

typedef DefaultFrameAssembler<StdPacket> StdAssembler;
typedef DefaultFrameAssembler<LegacyPacket> LegacyAssembler;
typedef DefaultFrameAssembler<GotthardPacket> GotthardAssembler;


/**
 *@short assembles frame data from 2 udp sockets into memory
 */

class DualPortFrameAssembler {

 public:
	typedef std::bitset<2> PortsMask;

	DualPortFrameAssembler(DefaultFrameAssemblerBase *a[2]);
	virtual ~DualPortFrameAssembler();

	virtual void stop();

	virtual PortsMask assembleFrame(uint64_t frame, RecvHeader *header,
					char *buf) = 0;

 protected:
	void stopAssemblers();

	DefaultFrameAssemblerBase *assembler[2];
};


/**
 *@short Eiger frame assembler in raw mode: 2x Default frame assemblers
 */

class EigerRawFrameAssembler : public DualPortFrameAssembler {

public:
	EigerRawFrameAssembler(DefaultFrameAssemblerBase *a[2]);

	virtual PortsMask assembleFrame(uint64_t frame, RecvHeader *recv_header,
					char *buf);
};

/**
 *@short Eiger frame assembler in std mode: port interleaving
 */

class EigerStdFrameAssembler : public DualPortFrameAssembler {

public:
	EigerStdFrameAssembler(DefaultFrameAssemblerBase *a[2], bool flipped);
	~EigerStdFrameAssembler();

	virtual PortsMask assembleFrame(uint64_t frame, RecvHeader *recv_header,
					char *buf);

	class Helper;

 protected:
	Helper *helper;
};

} // namespace FrameAssembler
