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


/**
 * EigerStdFrameAssembler::Helper
 */

class EigerStdFrameAssembler::Helper {
public:
	typedef PacketStream::Packet Packet;

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

	virtual void assemblePackets(Packet packet[][2]) = 0;

protected:
	static const int chip_size = 256;
	static const int chip_gap = 2;
	static const int port_chips = 2;
	static const int nb_ports = 2;

	struct Geometry {
		float pixel_size;
		int chip_size;
		int line_size;
		int offset;
	};

	GeneralData *general_data;
	bool flipped;
	int frame_packets;
	int packet_lines;
	Geometry src;
	Geometry dst;
	int first_idx;
	int idx_inc;
	char *buf;
};

EigerStdFrameAssembler::Helper::Helper(GeneralData *gd, bool f)
	: general_data(gd), flipped(f)
{
	frame_packets = gd->packetsPerFrame;
	packet_lines = chip_size / frame_packets;
	src.pixel_size = getSrcPixelBytes();
	src.chip_size = chip_size * src.pixel_size;
	src.line_size = port_chips * src.chip_size;
	dst.pixel_size = getDstPixelBytes();
	dst.chip_size = (chip_size + chip_gap) * dst.pixel_size;
	dst.line_size = nb_ports * gd->nPixelsX * dst.pixel_size;
	if (flipped) {
		src.offset = (packet_lines - 1) * src.line_size;
		src.line_size *= -1;
		dst.offset = 0;
		first_idx = frame_packets - 1;
		idx_inc = -1;
	} else {
		src.offset = 0;
		dst.offset = dst.line_size;
		first_idx = 0;
		idx_inc = 1;
	}
}

/**
 * Expand4BitsHelper
 */

class Expand4BitsHelper : public EigerStdFrameAssembler::Helper {
public:
	Expand4BitsHelper(GeneralData *gd, bool flipped);

	virtual void assemblePackets(Packet packets[][2])
		override;

private:
	static const int half_module_chips = nb_ports * port_chips;
	static const int block_len = sizeof(__m128i);
	static const int block_bits = block_len * 8;
	static const int gap_bits = chip_gap * 8;

	const int chip_blocks;
	const int port_blocks;

	const __m128i m;
	const __m128i m64_0, m64_1;
	const __m128i gap_bits128;
	const __m128i block64_bits128;

	int load_packet(Packet (*&packets)[2]);
	void load_dst128();
	void load_shift_store128();
	void pad_dst128();
	void sync_dst128();

	bool v[nb_ports], valid_data;
	const __m128i *s[nb_ports], *src128;
	__m128i *dst128;
	int dest_misalign;
	int shift_l;
	__m128i shift_l128, shift_r128;
	__m128i prev;

};

Expand4BitsHelper::Expand4BitsHelper(GeneralData *gd, bool flipped)
	: Helper(gd, flipped),
	chip_blocks(src.chip_size / block_len),
	port_blocks(src.line_size / block_len),
	m(_mm_set1_epi8(0xf)),
	m64_0(_mm_set_epi64x(0, -1)),
	m64_1(_mm_set_epi64x(-1, 0)),
	gap_bits128(_mm_set_epi64x(0, gap_bits)),
	block64_bits128(_mm_set_epi64x(0, 64))
{
}

inline int Expand4BitsHelper::load_packet(Packet (*&packets)[2])
{
	s[0] = (const __m128i *) ((*packets)[0].buf + src.offset);
	s[1] = (const __m128i *) ((*packets)[1].buf + src.offset);
	if ((((unsigned long) s[0] | (unsigned long) s[1]) & 15) != 0) {
		FILE_LOG(logERROR) << "Missaligned src";
		return -1;
	}
	v[0] = (*packets)[0].valid;
	v[1] = (*packets)[1].valid;
	return 0;
}

inline void Expand4BitsHelper::load_dst128()
{
	char *d = buf + dst.offset;
	dest_misalign = ((unsigned long) d & 15);
	dst128 = (__m128i *) (d - dest_misalign);
	shift_l = dest_misalign * 8;
	shift_l128 = _mm_set_epi64x(0, shift_l % 64);
	shift_r128 = _mm_sub_epi64(block64_bits128, shift_l128);
	if (shift_l != 0) {
		__m128i m0;
		m0 = _mm_srl_epi64(_mm_set1_epi8(0xff), shift_r128);
		if (shift_l < 64)
			m0 = _mm_and_si128(m0, m64_0);
		prev = _mm_and_si128(_mm_load_si128(dst128), m0);
	} else {
		prev = _mm_setzero_si128();
	}
}

inline void Expand4BitsHelper::load_shift_store128()
{
	__m128i p4_raw;
	if (valid_data)
		p4_raw = _mm_load_si128(src128);
	else
		p4_raw = _mm_set1_epi8(0xff);
	++src128;
	__m128i p4_shr = _mm_srli_epi16(p4_raw, 4);
	__m128i i8_0 = _mm_and_si128(p4_raw, m);
	__m128i i8_1 = _mm_and_si128(p4_shr, m);
	__m128i p8_0 = _mm_unpacklo_epi8(i8_0, i8_1);
	__m128i p8_1 = _mm_unpackhi_epi8(i8_0, i8_1);
	__m128i p8_0l = _mm_sll_epi64(p8_0, shift_l128);
	__m128i p8_0r = _mm_srl_epi64(p8_0, shift_r128);
	__m128i p8_1l = _mm_sll_epi64(p8_1, shift_l128);
	__m128i p8_1r = _mm_srl_epi64(p8_1, shift_r128);
	__m128i d0, d1, d2, d3, d4;
	if (shift_l < 64) {
		d0 = p8_0l;
		d1 = p8_0r;
		d2 = p8_1l;
		d3 = p8_1r;
		d4 = _mm_setzero_si128();
	} else {
		d0 = _mm_setzero_si128();
		d1 = p8_0l;
		d2 = p8_0r;
		d3 = p8_1l;
		d4 = p8_1r;
	}
	__m128i d10 = _mm_slli_si128(d1, 8);
	__m128i d11 = _mm_srli_si128(d1, 8);
	__m128i d30 = _mm_slli_si128(d3, 8);
	__m128i d31 = _mm_srli_si128(d3, 8);
	prev = _mm_or_si128(prev, d0);
	_mm_store_si128(dst128++, _mm_or_si128(prev, d10));
	prev = _mm_or_si128(d11, d2);
	_mm_store_si128(dst128++, _mm_or_si128(prev, d30));
	prev = _mm_or_si128(d31, d4);
}

inline void Expand4BitsHelper::pad_dst128()
{
	shift_l += gap_bits;
	if (shift_l % 64 == 0)
		shift_l128 = _mm_setzero_si128();
	else
		shift_l128 = _mm_add_epi64(shift_l128, gap_bits128);
	shift_r128 = _mm_sub_epi64(block64_bits128, shift_l128);
	if (shift_l == block_bits) {
		_mm_store_si128(dst128++, prev);
		prev = _mm_setzero_si128();
		shift_l = 0;
	}
}

inline void Expand4BitsHelper::sync_dst128()
{
	if (shift_l != 0) {
		__m128i m0;
		m0 = _mm_sll_epi64(_mm_set1_epi8(0xff), shift_l128);
		if (shift_l >= 64)
			m0 = _mm_and_si128(m0, m64_1);
		__m128i a = _mm_and_si128(_mm_load_si128(dst128), m0);
		_mm_store_si128(dst128, _mm_or_si128(prev, a));
	}
}

void Expand4BitsHelper::assemblePackets(Packet packets[][2])
{
	packets += first_idx;
	load_dst128();
	for (int i = 0; i < frame_packets; ++i, packets += idx_inc) {
		if (load_packet(packets) < 0)
			return;
		for (int l = 0; l < packet_lines; ++l) {
			int chip_count = 0;
			for (int p = 0; p < nb_ports; ++p) {
				valid_data = v[p];
				src128 = s[p];
				for (int c = 0; c < port_chips; ++c) {
					for (int b = 0; b < chip_blocks; ++b)
						load_shift_store128();
					if (++chip_count % half_module_chips > 0)
						pad_dst128();
				}
				s[p] += port_blocks;
			}
		}
	}
	sync_dst128();
}

/**
 * CopyHelper
 */

class CopyHelper : public EigerStdFrameAssembler::Helper {
public:
	CopyHelper(GeneralData *gd, bool flipped)
		: Helper(gd, flipped)
	{}
	
	virtual void assemblePackets(Packet packets[][2])
		override;
};

void CopyHelper::assemblePackets(Packet packets[][2])
{
	packets += first_idx;
	char *d = buf + dst.offset;
	for (int i = 0; i < frame_packets; ++i, packets += idx_inc) {
		char *s[nb_ports] = {(*packets)[0].buf + src.offset,
				     (*packets)[1].buf + src.offset};
		for (int l = 0; l < packet_lines; ++l) {
			char *ld = d;
			for (int p = 0; p < nb_ports; ++p) {
				char *ls = s[p];
				for (int c = 0; c < port_chips; ++c) {
					if ((*packets)[p].valid)
						memcpy(ld, ls, src.chip_size);
					else
						memset(ld, 0xff, src.chip_size);
					ls += src.chip_size;
					ld += dst.chip_size;
				}
				s[p] += src.line_size;
			}
			d += dst.line_size;
		}
	}
}


/**
 * EigerStdFrameAssembler
 */

EigerStdFrameAssembler::EigerStdFrameAssembler(DefaultFrameAssembler *a[2], bool flipped)
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


DualPortFrameAssembler::PortsMask
EigerStdFrameAssembler::assembleFrame(uint64_t frame, RecvHeader *recv_header, char *buf)
{
	PortsMask mask;

	const int nb_ports = 2;

	helper->startNewFrame(buf);

	DefaultFrameAssembler **a = assembler;
	bool header_empty = true;
	int num_packets = 0;
	int packets_per_frame = a[0]->general_data->packetsPerFrame;
	PacketStream::DetHeader *det_header = &recv_header->detHeader;
	FramePolicy policy = a[0]->frame_policy;
	bool fp_partial = (policy == slsReceiverDefs::DISCARD_PARTIAL_FRAMES);
	bool fp_empty = (policy == slsReceiverDefs::DISCARD_EMPTY_FRAMES);

	Packet packets[packets_per_frame][nb_ports];
	for (int pnum = 0; pnum < packets_per_frame; ++pnum) {
		if (a[0]->stopped)
			return PortsMask();

		int packet_count = 0;
		for (int i = 0; i < nb_ports; ++i) {
			Packet& p = packets[pnum][i];
			PacketStream *ps = a[i]->packet_stream.get();
			p = ps->getPacket(frame, pnum);
			if (!p.valid && fp_partial)
				return PortsMask();

			mask.set(i, true);
	 		++packet_count;

			//write header
			if (header_empty) {
				const int det_header_len = sizeof(*det_header);
				memcpy(det_header, p.header, det_header_len);
				header_empty = false;
			}
		}
		num_packets += packet_count;
		recv_header->packetsMask[pnum] = (packet_count > 0);
	}

	if (mask.any()) {
		helper->assemblePackets(packets);
	} else {
		if (fp_empty)
			return PortsMask();
		det_header->frameNumber = frame;
	}
	det_header->packetNumber = num_packets;

	return mask;
}