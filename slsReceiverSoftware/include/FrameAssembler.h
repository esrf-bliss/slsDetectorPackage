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

	PacketStream(genericSocket *s, GeneralData *d,
		     unsigned long node_mask, int max_node)
		: socket(s),
		  general_data(d),
		  carry_over(false),
		  odd_numbering(false),
		  first_packet(true)
	{
		initMem(node_mask, max_node);
	}

	Packet getPacket(uint64_t frame)
	{
		GeneralData& gd = *general_data;

		char *p = packet.getPtr() + header_pad;
		if (!carry_over) {
			int ret = socket->ReceiveDataOnly(p);
			if (ret <= 0)
				return Packet(false);
		}

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
			
		carry_over = (packet_frame > frame);
		if (packet_frame < frame)
			cprintf(RED, "Error: Frame number %lu less than "
				"current frame number %lu\n", packet_frame, frame);
		if (packet_frame != frame)
			return Packet(false);

		return Packet(true, header, p + sizeof(DetHeader),
			      packet_frame, packet_number);
	}

	bool hasPendingPacket()
	{
		return carry_over;
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
		size_t packet_len = gd.packetSize;
		size_t header_len = gd.emptyHeader + sizeof(DetHeader);
		size_t misalign = header_len % data_align;
		header_pad = misalign ? (data_align - misalign) : 0;
		packet_len += header_pad;
		packet.alloc(packet_len, node_mask, max_node);
	}

	genericSocket *socket;
	GeneralData *general_data;
	int header_pad;
	MmapMem packet;
	bool carry_over;
	bool odd_numbering;
	bool first_packet;
};


/**
 *@short Default frame assembler in Listener
 */

class DefaultFrameAssembler {

 public:
	typedef slsReceiverDefs::sls_receiver_header RecvHeader;
	typedef slsReceiverDefs::frameDiscardPolicy FramePolicy;

	DefaultFrameAssembler(genericSocket *s, GeneralData *d,
			   unsigned long node_mask, int max_node, 
			   FramePolicy fp, bool e4b)
		: packet_stream(new PacketStream(s, d, node_mask, max_node)),
		  general_data(d),
		  frame_policy(fp), stopped(false), expand_4bits(e4b)
 	{}

	int assembleFrame(uint64_t frame, RecvHeader *header, char *buf);

	void stop()
	{
		stopped = true;
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
