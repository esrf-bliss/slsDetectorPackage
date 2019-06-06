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

	PacketStream(genericSocket *s, GeneralData *d)
		: socket(s),
		  general_data(d),
		  packet(new char[general_data->packetSize]),
		  carry_over(false),
		  odd_numbering(false),
		  first_packet(true)
	{}

	Packet getPacket(uint64_t frame)
	{
		GeneralData& gd = *general_data;

		char *p = packet.get();
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
	typedef std::unique_ptr<char[]> BufferPtr;

	genericSocket *socket;
	GeneralData *general_data;
	BufferPtr packet;
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

	DefaultFrameAssembler(genericSocket *s, GeneralData *d, FramePolicy fp,
			   bool e4b)
		: packet_stream(new PacketStream(s, d)), general_data(d),
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
