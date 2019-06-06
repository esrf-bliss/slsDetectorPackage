/************************************************
 * @file FrameAssembler.cpp
 * @short helper classes assembling frames
 * from udp packets
 ***********************************************/

#include "FrameAssembler.h"

#include <emmintrin.h>

inline void DefaultFrameAssembler::expand4Bits(char *dst, char *src, int src_size)
{
	unsigned long s = (unsigned long) src;
	unsigned long d = (unsigned long) dst;
	if (((s & 15) != 0) || ((d & 15) != 0)) {
		FILE_LOG(logERROR) << "Missaligned src/dst";
		return;
	}

	const int blk = sizeof(__m128i);
	if ((src_size % blk) != 0)
		FILE_LOG(logWARNING) << "len misalignment: "
				     << "src_size=" << src_size << ", blk=" << blk;
	int nb_blocks = src_size / blk;
	const __m128i *src128 = (const __m128i *) src;
	__m128i *dst128 = (__m128i *) dst;
	const __m128i mask = _mm_set1_epi8(0xf);
	for (int i = 0; i < nb_blocks; ++i) {
		__m128i pack4_raw = _mm_load_si128(src128++);
		__m128i pack4_shr = _mm_srli_epi16(pack4_raw, 4);
		__m128i ilace8_0 = _mm_and_si128(pack4_raw, mask);
		__m128i ilace8_1 = _mm_and_si128(pack4_shr, mask);
		__m128i pack8_0 = _mm_unpacklo_epi8(ilace8_0, ilace8_1);
		__m128i pack8_1 = _mm_unpackhi_epi8(ilace8_0, ilace8_1);
		_mm_store_si128(dst128++, pack8_0);
		_mm_store_si128(dst128++, pack8_1);
	}
}

int DefaultFrameAssembler::assembleFrame(uint64_t frame, RecvHeader *recv_header, char *buf)
{
	bool header_empty = true;
	int num_packets = 0;
	int packets_per_frame = general_data->packetsPerFrame;
	PacketStream::DetHeader *det_header = &recv_header->detHeader;
	bool do_expand_4bits = doExpand4Bits();
	uint32_t src_dsize = general_data->dataSize;
	uint32_t last_dsize = general_data->imageSize % src_dsize;
	if (last_dsize == 0)
		last_dsize = src_dsize;
	uint32_t dst_dsize = src_dsize * (do_expand_4bits ? 2 : 1);

	recv_header->packetsMask.reset();

	while (num_packets < packets_per_frame) {
		if (stopped)
			return -1;

		Packet packet = packet_stream->getPacket(frame);
		if (!packet.valid) {
			if (canDiscardFrame(num_packets))
				return -1;
			break;
		}

		int pnum = packet.number;

		//copy packet
		char *dst = buf + pnum * dst_dsize;
		bool last_packet = (pnum == (packets_per_frame - 1));
		uint32_t copy_dsize = last_packet ? last_dsize : src_dsize;
		if (general_data->myDetectorType == slsReceiverDefs::GOTTHARD) {
			// Gotthard data:
			//   1st packet: CACA + CACA, (640 - 1) * 2 bytes data
			//   2nd packet: (1 + 640) * 2 bytes data
			if (pnum == 0) {
				packet.buf += 4;
				copy_dsize -= 2;
			} else {
				dst -= 2;
				copy_dsize += 2;
			}
		}
		if (do_expand_4bits)
			expand4Bits(dst, packet.buf, copy_dsize);
		else
			memcpy(dst, packet.buf, copy_dsize);

		++num_packets;
		recv_header->packetsMask[pnum] = 1;

		//write header
		if (header_empty) {
			const int det_header_len = sizeof(*det_header);
			if (general_data->standardheader) {
				memcpy(det_header, packet.header, det_header_len);
			} else {
				memset(det_header, 0, det_header_len);
				det_header->frameNumber = frame;
				det_header->detType = (uint8_t) general_data->myDetectorType;
				det_header->version = (uint8_t) SLS_DETECTOR_HEADER_VERSION;
			}
			header_empty = false;
		}
	}

	if (header_empty)
		det_header->frameNumber = frame;
	det_header->packetNumber = num_packets;

	return 0;
}

DualPortFrameAssembler::PortsMask
EigerRawFrameAssembler::assembleFrame(uint64_t frame, RecvHeader *recv_header, char *buf)
{
	const int nb_ports = 2;
	const int image_size = assembler[0]->getImageSize();
	PortsMask mask;
	for (int i = 0; i < nb_ports; ++i, buf += image_size) {
		int ret = assembler[i]->assembleFrame(frame, recv_header, buf);
		mask.set(i, (ret == 0));
	}
	return mask;
}
