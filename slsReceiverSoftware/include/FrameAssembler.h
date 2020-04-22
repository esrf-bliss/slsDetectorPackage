#pragma once
/************************************************
 * @file FrameAssembler.h
 * @short helper classes assembling frames
 * from udp packets
 ***********************************************/

#include "AutoPtr.h"
#include "logger.h"
#include "sls_receiver_defs.h"
#include "GeneralData.h"
#include "genericSocket.h"

#include <vector>
#include <queue>
#include <map>

#include "MmappedRegion.h"
#include "ThreadUtils.h"
#include "Stats.h"

namespace FrameAssembler
{

typedef slsReceiverDefs::sls_detector_header DetHeader;
typedef slsReceiverDefs::sls_receiver_header RecvHeader;
typedef slsReceiverDefs::frameDiscardPolicy FramePolicy;


/**
 * Memory layout of each element of the packet buffer array
 *
 * < pad > < --------------------- buffer --------------------------- >
 *         < soft_header > < ------------ network_buffer ------------ >
 *                         < empty_header > < network_header > < data >
 *
 * Note: <pad> is calculated in order to have <data> aligned to 128-bits
 */

/**
 *@short The PacketData base struct
 */

struct PacketDataBase {
	// An instance of <derived>::StreamData is included in the PacketStream
	struct StreamData {
		GeneralData *gd;
		StreamData(GeneralData *d) : gd(d)
		{}
	};

	// An instance of <derived>::SoftHeader prepends each network packet
	struct SoftHeader {
		bool valid;
	};
};


/**
 *@short The Packet base struct
 */

template <class PD>
struct Packet {
	typedef typename PD::StreamData StreamData;
	typedef typename PD::SoftHeader SoftHeader;

	char *buffer;
	StreamData *stream_data;

	Packet(char *b, StreamData *d) : buffer(b), stream_data(d)
	{}

	SoftHeader *softHeader()
	{ return reinterpret_cast<SoftHeader *>(buffer); }

	bool valid()
	{ return softHeader()->valid; }

	char *networkBuffer()
	{ return buffer + sizeof(SoftHeader); }

	char *networkHeader()
	{ return networkBuffer() + generalData()->emptyHeader; }

	GeneralData *generalData()
	{ return stream_data->gd; }
};

/**
 *@short StdPacket class
 */

struct StdPacketData : public PacketDataBase {
};

struct StdPacket : public Packet<StdPacketData> {
	typedef Packet<StdPacketData> Base;

	StdPacket(char *b, StreamData *sd) : Base(b, sd)
	{}

	void initSoftHeader()
	{}

	DetHeader *header()
	{ return reinterpret_cast<DetHeader *>(networkHeader()); }
	
	uint64_t frame()
	{ return header()->frameNumber; }

	uint32_t number()
	{ return header()->packetNumber; }

	char *data()
	{ return networkHeader() + sizeof(DetHeader); }

	uint32_t sizeAdjust()
	{ return 0; }

	void fillDetHeader(DetHeader *det_header);
};

/**
 *@short LegacyPacket class
 */

struct LegacyPacketData : public PacketDataBase {
	struct StreamData : public PacketDataBase::StreamData {
		bool odd_numbering;
		StreamData(GeneralData *d)
			: PacketDataBase::StreamData(d), odd_numbering(true)
		{}
	};

	struct SoftHeader : public PacketDataBase::SoftHeader {
		uint64_t packet_frame;
		uint32_t packet_number;
	};
};

template <class PD>
struct LegacyPacketImpl : public Packet<PD> {
	typedef Packet<PD> Base;

	LegacyPacketImpl(char *b, typename Base::StreamData *sd) : Base(b, sd)
	{}
	
	void initSoftHeader();

	uint64_t frame()
	{ return Base::softHeader()->packet_frame; }

	uint32_t number()
	{ return Base::softHeader()->packet_number; }

	char *data()
	{ return (Base::networkHeader()
		  + Base::generalData()->headerSizeinPacket); }

	uint32_t sizeAdjust()
	{ return 0; }

	void fillDetHeader(DetHeader *det_header);
};

typedef LegacyPacketImpl<LegacyPacketData> LegacyPacket;	

/**
 *@short GotthardPacket class
 */

struct GotthardPacketData : public LegacyPacketData {
	struct StreamData : public LegacyPacketData::StreamData {
		bool first_packet;
		StreamData(GeneralData *d)
			: LegacyPacketData::StreamData(d), first_packet(true)
		{}
	};
};

struct GotthardPacket : public LegacyPacketImpl<GotthardPacketData> {
	typedef LegacyPacketImpl<GotthardPacketData> Base;

	GotthardPacket(char *b, StreamData *sd) : Base(b, sd)
	{}

	void initSoftHeader();

	// Gotthard data:
	//   1st packet: CACA + CACA, (640 - 1) * 2 bytes data
	//   2nd packet: (1 + 640) * 2 bytes data

	char *data()
	{ return Base::data() + ((number() == 0) ? 4 : 0); }

	uint32_t sizeAdjust()
	{ return (number() == 0) ? -2 : 2; }
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
	typedef typename P::StreamData StreamData;
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
	void releaseReadyPacketBlocks();
	void waitUsedPacketBlocks();
	void printStat();

	genericSocket *socket;
	GeneralData *general_data;
	FramePolicy frame_policy;
	const unsigned int num_frames;
	Mutex mutex;
	int packets_caught;
	uint64_t frames_caught;
	uint64_t last_frame;
	StreamData stream_data;
	int header_pad;
	int packet_len;
	MmappedRegion packet_buffer_array;
	Cond free_cond;
	std::queue<char *> free_queue;
	bool stopped;
	int waiting_reader_count;
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
