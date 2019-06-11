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

/**
 *@short manages packet stream with carry-over functionality
 */

class PacketStream {

 public:
	typedef slsReceiverDefs::sls_detector_header DetHeader;

	struct Packet {
		bool valid;
		DetHeader *header;
		uint64_t frame;
		int number;
		char *buf;

		Packet(bool v=false, DetHeader *h = nullptr, char *b = nullptr,
		       uint64_t f = 0, int n = 0)
			: valid(v), header(h), buf(b), frame(f), number(n)
		{}
	};

	PacketStream(genericSocket *s, GeneralData *d, cpu_set_t cpu_mask,
		     unsigned long node_mask, int max_node)
		: socket(s),
		  general_data(d),
		  nb_packets(128),
		  carry_over(false),
		  odd_numbering(false),
		  first_packet(true),
		  stopped(false),
		  write_idx(0),
		  read_idx(nb_packets - 1), // will be incremented on first read
		  cpu_aff_mask(cpu_mask)
	{
		initMem(node_mask, max_node);
		initThread();
	}

	~PacketStream()
	{
		stop();
		void *r;
		pthread_join(thread, &r);
		usleep(5000);	// give reader thread time to exit getPacket
	}

	Packet getPacket(uint64_t frame, int num = -1)
	{
		GeneralData& gd = *general_data;

		if (!carry_over) {
			incIndex(read_idx);
			while (!stopped && bufferEmpty())
				usleep(10);
			if (stopped)
				return Packet(false);
		}

		char *p = packetPtr(read_idx);
		p += gd.emptyHeader;
		DetHeader *header = reinterpret_cast<DetHeader *>(p);
		uint64_t packet_frame, packet_bid;
		uint32_t packet_number, packet_subframe;
		
		if (gd.standardheader) {
			packet_frame = header->frameNumber;
			packet_number = header->packetNumber;
		} else {
			int thread_idx;
			if (first_packet && 
			    (gd.myDetectorType == slsReceiverDefs::GOTTHARD)) {
				odd_numbering = gd.SetOddStartingPacket(thread_idx, p);
				first_packet = false;
			}
			gd.GetHeaderInfo(thread_idx, p, odd_numbering,
					 packet_frame, packet_number,
					 packet_subframe, packet_bid);
		}

		bool good_frame = (packet_frame == frame);
		carry_over = ((packet_frame > frame) ||
			      (good_frame && (num >= 0) && (packet_number > num)));
		if (carry_over) {
			return Packet(false);
		} else if (!good_frame || ((num >= 0) && (packet_number < num))) {
			cprintf(RED, "Error: Frame/packet number %lu/%d less than "
				"current number %lu/%d\n",
				packet_frame, packet_number, frame, num);
			return Packet(false);
		}

		return Packet(true, header, p + sizeof(DetHeader),
			      packet_frame, packet_number);
	}

	bool hasPendingPacket()
	{
		return carry_over;
	}

	void stop()
	{
		stopped = true;
	}

 private:
	class bad_mmap_alloc : public std::bad_alloc {
	public:
		bad_mmap_alloc(const char *m = "") : msg(m)
		{}
		virtual const char* what() const throw() override
		{ return msg.c_str(); }
	private:
		std::string msg;
	};

	class MmapMem
	{
	public:
		MmapMem(size_t size = 0, unsigned long node_mask = 0,
			int max_node = 0)
			: ptr(nullptr), len(0)
		{
			alloc(size, node_mask, max_node);
		}

		~MmapMem()
		{
			release();
		}

		void alloc(size_t size, unsigned long node_mask = 0,
			   int max_node = 0)
		{
			release();
			if (size == 0)
				return;

			size_t page_size = sysconf(_SC_PAGESIZE);
			size_t misalign = size % page_size;
			size += misalign ? (page_size - misalign) : 0;

			ptr = (char *) mmap(0, size, PROT_READ | PROT_WRITE,
					    MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
			if (ptr == NULL)
				throw bad_mmap_alloc("Could not allocate packet memory");
			len = size;

			if (node_mask && max_node) {
				int ret = mbind(ptr, len, MPOL_BIND,
						&node_mask, max_node, 0);
				if (ret != 0) {
					release();
					throw bad_mmap_alloc("Could not bind packet memory");
				}
			}
			memset(ptr, 0, len);
		}

		void release()
		{
			if (len) {
				munmap(ptr, len);
				len = 0;
			}
		}

		char *getPtr()
		{
			return ptr;
		}

	private:
		char *ptr;
		size_t len;
	};

	void initMem(unsigned long node_mask, int max_node)
	{
		const int data_align = 128 / 8;
		GeneralData& gd = *general_data;
		packet_len = gd.packetSize;
		size_t header_len = gd.emptyHeader + sizeof(DetHeader);
		size_t misalign = header_len % data_align;
		header_pad = misalign ? (data_align - misalign) : 0;
		packet_len += header_pad;
		misalign = packet_len % data_align;
		packet_len += misalign ? (data_align - misalign) : 0;
		packet.alloc(packet_len * nb_packets, node_mask, max_node);
	}

	int packetsAvailable()
	{
		int diff = write_idx - read_idx;
		if (diff < 0)
			diff += nb_packets;
		return diff;
	}

	char *packetPtr(int idx)
	{
		return packet.getPtr() + idx * packet_len + header_pad;
	}

	void incIndex(volatile int& index)
	{
		int new_idx = index + 1;
		index = new_idx % nb_packets;
	}

	bool bufferEmpty()
	{
		return packetsAvailable() == 0;
	}

	bool bufferFull()
	{
		return packetsAvailable() == (nb_packets - 1);
	}

	void initThread()
	{
		int ret;
		ret = pthread_create(&thread, NULL, threadFunctionStatic, this);
		if (ret != 0)
			throw std::runtime_error("Could not start packet thread");

		struct sched_param param;
		param.sched_priority = 90;
		ret = pthread_setschedparam(thread, SCHED_FIFO, &param);
		if (ret != 0) {
			void *r;
			pthread_join(thread, &r);
			throw std::runtime_error("Could not set packet thread prio");
		}
	}

	static void *threadFunctionStatic(void *data)
	{
		PacketStream *ps = static_cast<PacketStream *>(data);
		ps->threadFunction();
		return NULL;
	}

	void threadFunction()
	{
		if (CPU_COUNT(&cpu_aff_mask) != 0) {
			int size = sizeof(cpu_aff_mask);
			int ret = sched_setaffinity(0, size, &cpu_aff_mask);
			if (ret != 0)
				throw std::runtime_error("Could not set packet "
							 "cpu affinity mask");
		}

		while (true) {
			while (!stopped && bufferFull())
				usleep(10);
			if (stopped)
				break;
			char *p = packetPtr(write_idx);
			int ret = socket->ReceiveDataOnly(p);
			if (ret < 0)
				break;
			incIndex(write_idx);
		}
	}

	genericSocket *socket;
	GeneralData *general_data;
	const int nb_packets;
	int header_pad;
	int packet_len;
	MmapMem packet;
	bool carry_over;
	bool odd_numbering;
	bool first_packet;
	volatile bool stopped;
	volatile int write_idx;
	volatile int read_idx;
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

	DefaultFrameAssembler(genericSocket *s, GeneralData *d, cpu_set_t cpu_mask,
			   unsigned long node_mask, int max_node, 
			   FramePolicy fp, bool e4b)
		: packet_stream(new PacketStream(s, d, cpu_mask,
						 node_mask, max_node)),
		  general_data(d),
		  frame_policy(fp), stopped(false), expand_4bits(e4b)
 	{}

	int assembleFrame(uint64_t frame, RecvHeader *header, char *buf);

	void stop()
	{
		stopped = true;
		packet_stream->stop();
	}

	bool hasPendingPacket()
	{
		return packet_stream->hasPendingPacket();
	}

	int getImageSize()
	{
		return general_data->imageSize * (doExpand4Bits() ? 2 : 1);
	}

 protected:
	friend class EigerStdFrameAssembler;

	typedef std::unique_ptr<PacketStream> PacketStreamPtr;
	typedef PacketStream::Packet Packet;

	bool canDiscardFrame(int num_packets)
	{
		return ((frame_policy ==
			 slsReceiverDefs::DISCARD_PARTIAL_FRAMES) ||
			((frame_policy ==
			  slsReceiverDefs::DISCARD_EMPTY_FRAMES) &&
			 !num_packets));
	}

	bool doExpand4Bits()
	{
		return (general_data->dynamicRange == 4) && expand_4bits;
	}

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

	DualPortFrameAssembler(DefaultFrameAssembler *a[2])
		: assembler({a[0], a[1]})
	{}

	virtual ~DualPortFrameAssembler()
	{}

	virtual void stop()
	{
		stopAssemblers();
	}

	virtual PortsMask assembleFrame(uint64_t frame, RecvHeader *header, char *buf) = 0;

 protected:
	void stopAssemblers()
	{
		assembler[0]->stop(), assembler[1]->stop();
	}

	DefaultFrameAssembler *assembler[2];
};


/**
 *@short Eiger frame assembler in raw mode: 2x Default frame assemblers
 */

class EigerRawFrameAssembler : public DualPortFrameAssembler {

public:
	EigerRawFrameAssembler(DefaultFrameAssembler *a[2])
		: DualPortFrameAssembler(a)
	{}

	virtual PortsMask assembleFrame(uint64_t frame, RecvHeader *recv_header,
					char *buf) override;
};

/**
 *@short Eiger frame assembler in std mode: port interleaving
 */

class EigerStdFrameAssembler : public DualPortFrameAssembler {

public:
	typedef DefaultFrameAssembler::FramePolicy FramePolicy;
	typedef PacketStream::Packet Packet;

	EigerStdFrameAssembler(DefaultFrameAssembler *a[2], bool flipped)
		: DualPortFrameAssembler(a)
	{
		GeneralData *gd = a[0]->general_data;
		if (!gd->tgEnable)
			throw std::runtime_error("10 Giga not enabled!");

		Helper *h;
		if (a[0]->doExpand4Bits())
			h = new Expand4BitsHelper(gd, flipped);
		else
			h = new CopyHelper(gd, flipped);
		helper.reset(h);
	}

	virtual PortsMask assembleFrame(uint64_t frame, RecvHeader *recv_header,
					char *buf) override;

 protected:
	class Helper {
	public:
		Helper(GeneralData *gd, bool f);

		float getSrcPixelBytes()
		{
			return float(general_data->dynamicRange) / 8;
		}

		float getDstPixelBytes()
		{
			int factor = (general_data->dynamicRange == 4) ? 2 : 1;
			return getSrcPixelBytes() * factor;
		}

		void startNewFrame(char *d)
		{
			buf = d;
		}

		virtual void assemblePackets(uint64_t pnum, Packet packet[2]) = 0;

	protected:
		static const int chip_size;
		static const int chip_gap;
		static const int port_chips;
		static const int nb_ports;

		struct Geometry {
			float pixel_size;
			int chip_size;
			int line_size;
			int packet_size;
			int offset;
		};

		GeneralData *general_data;
		bool flipped;
		int packet_lines;
		Geometry src;
		Geometry dst;
		char *buf;
	};

	typedef std::unique_ptr<Helper> HelperPtr;

	class Expand4BitsHelper : public Helper {
	public:
		Expand4BitsHelper(GeneralData *gd, bool flipped);

		virtual void assemblePackets(uint64_t pnum, Packet packet[2])
			override;
	};

	class CopyHelper : public Helper {
	public:
		CopyHelper(GeneralData *gd, bool flipped)
			: Helper(gd, flipped)
		{}

		virtual void assemblePackets(uint64_t pnum, Packet packet[2])
			override;
	};

	HelperPtr helper;
};
