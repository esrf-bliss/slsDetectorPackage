/************************************************
 * @file FrameAssembler.cpp
 * @short helper classes assembling frames
 * from udp packets
 ***********************************************/

#include "FrameAssembler.h"

#include <emmintrin.h>

using namespace FrameAssembler;

/**
 * MmappedRegion
 */

MmappedRegion::MmappedRegion(size_t size, unsigned long node_mask, int max_node)
	: ptr(NULL), len(0)
{
	alloc(size, node_mask, max_node);
}

MmappedRegion::~MmappedRegion()
{
	release();
}

void MmappedRegion::alloc(size_t size, unsigned long node_mask, int max_node)
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

void MmappedRegion::release()
{
	if (len) {
		munmap(ptr, len);
		len = 0;
	}
}

inline char *MmappedRegion::getPtr()
{
	return ptr;
}


/**
 * Semaphore
 */

Semaphore::Semaphore(int n)
{
	if (sem_init(&sem, 0, n) != 0)
		throw std::runtime_error("Could not init sem");
}

Semaphore::~Semaphore()
{
	sem_destroy(&sem);
}

inline void Semaphore::post()
{
	sem_post(&sem);
}

inline void Semaphore::wait()
{
	sem_wait(&sem);
}

/**
 * Packet
 */

inline void Packet::setValid(DetHeader *h, char *b, uint64_t f,
					   int n)
{
	valid = true;
	header = h;
	buf = b;
	frame = f;
	number = n;
}

inline void Packet::setInvalid()
{
	valid = false;
}


/**
 * PacketBlock
 */

inline PacketBlock::PacketBlock(int l)
	: len(l), ps(NULL), idx(-1)
{
}

inline PacketBlock::~PacketBlock()
{
	if (ps && (idx >= 0))
		ps->releasePacketBlock(*this);
}

inline Packet& PacketBlock::operator [](int i)
{
	return packet[i];
}

/**
 * PacketStream
 */

inline char *PacketStream::packetPtr(int idx)
{
	return packet.getPtr() + idx * packet_len + header_pad;
}

inline void PacketStream::releasePacketBlock(PacketBlock& block)
{
	MutexLock l(block_cond.getMutex());

	int n = getIndexDiff(block.idx, read_idx);
	for (int i = 0; (i < block.len) && block[i].valid; ++i, ++n)
		waiting_mask[n] = 0;
	if (block.idx != read_idx)
		return;
	while (num_blocked_packets && !waiting_mask[0]) {
		write_sem.post();
		--num_blocked_packets;
		waiting_mask >>= 1;
		incIndex(read_idx);
	}
}

inline int PacketStream::getIndex(int index)
{
	return index % tot_num_packets;
}

inline int PacketStream::getIndexDiff(int a, int b)
{
	int n = a - b;
	if (n < 0)
		n += tot_num_packets;
	return n;
}

inline void PacketStream::incIndex(int& index)
{
	index = getIndex(index + 1);
}

inline bool PacketStream::canDiscardFrame(int received_packets)
{
	return ((frame_policy ==
		 slsReceiverDefs::DISCARD_PARTIAL_FRAMES) ||
		((frame_policy ==
		  slsReceiverDefs::DISCARD_EMPTY_FRAMES) &&
		 !received_packets));
}

inline int PacketStream::getNextPacket(Packet& np, uint64_t frame, int num)
{
	GeneralData& gd = *general_data;

	int idx = read_idx;
	for (int i = 0; (i < tot_num_packets) && !stopped; ++i, incIndex(idx)) {
		int n = getIndexDiff(idx, read_idx);
		if (n == num_blocked_packets) {
			Mutex& mutex = block_cond.getMutex();
			mutex.unlock();
			read_sem.wait();
			mutex.lock();
			if (stopped)
				break;
			waiting_mask[num_blocked_packets++] = 1;
		} else if (!waiting_mask[n]) {
			continue;
		}

		char *p = packetPtr(idx);
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

		if (packet_frame < frame) {
			continue;
		} else if (packet_frame > frame) {
			break;
		} else if (packet_number == num) {
			np.setValid(header, p + sizeof(DetHeader),
				    packet_frame, packet_number);
			return idx;
		}
	}

	np.setInvalid();
	return -1;
}

inline int PacketStream::getPacketBlock(PacketBlock& block, uint64_t frame)
{
	class Sync
	{
	public:
		Sync(Cond& c, bool& i, volatile bool& s)
			: cond(c), in_get_block(i), stopped(s), l(c.getMutex())
		{
			while (in_get_block && !stopped)
				cond.wait();
			in_get_block = true;
		}

		~Sync()
		{
			in_get_block = false;
			cond.signal();
		}

	private:
		Cond& cond;
		bool& in_get_block;
		volatile bool& stopped;
		MutexLock l;
	} s(block_cond, in_get_block, stopped);

	if (stopped)
		return 0;

	int pnum;
	bool full_frame = true;
	for (pnum = 0; pnum < block.len; ++pnum) {
		Packet& p = block[pnum];
		int idx = getNextPacket(p, frame, pnum);
		if (pnum == 0) {
			block.ps = this;
			block.idx = idx;
		}
		if (p.valid)
			continue;
		full_frame = false;
		while (++pnum < block.len)
			block[pnum].valid = false;
		if (canDiscardFrame(pnum))
			return 0;
		break;
	}

	if (full_frame)
		++frames_caught;
	if (frame > last_frame)
		last_frame = frame;

	return pnum;
}

inline bool PacketStream::hasPendingPacket()
{
	return (num_blocked_packets > 0);
}

PacketStream::PacketStream(genericSocket *s, GeneralData *d, FramePolicy fp,
			   cpu_set_t cpu_mask,
			   unsigned long node_mask, int max_node)
	: socket(s),
	  general_data(d),
	  frame_policy(fp),
	  tot_num_packets(MaxBufferFrames * general_data->packetsPerFrame),
	  packets_caught(0),
	  frames_caught(0),
	  last_frame(0),
	  num_blocked_packets(0),
	  odd_numbering(false),
	  first_packet(true),
	  stopped(false),
	  read_idx(0),
	  write_sem(tot_num_packets),
	  read_sem(0),
	  in_get_block(false),
	  cpu_aff_mask(cpu_mask)
{
	initMem(node_mask, max_node);
	initThread();
}

PacketStream::~PacketStream()
{
	stop();
	void *r;
	pthread_join(thread, &r);
	usleep(5000);	// give reader thread time to exit getPacket
}

void PacketStream::stop()
{
	stopped = true;
	write_sem.post();
	read_sem.post();
	block_cond.signal();
}

inline int PacketStream::getNumPacketsCaught()
{
	return packets_caught;
}

inline uint64_t PacketStream::getNumFramesCaught()
{
	return frames_caught;
}

inline uint64_t PacketStream::getLastFrameIndex()
{
	return last_frame;
}

void PacketStream::initMem(unsigned long node_mask, int max_node)
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
	packet.alloc(packet_len * tot_num_packets, node_mask, max_node);
}

void PacketStream::initThread()
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

void *PacketStream::threadFunctionStatic(void *data)
{
	PacketStream *ps = static_cast<PacketStream *>(data);
	ps->threadFunction();
	return NULL;
}

void PacketStream::threadFunction()
{
	if (CPU_COUNT(&cpu_aff_mask) != 0) {
		int size = sizeof(cpu_aff_mask);
		int ret = sched_setaffinity(0, size, &cpu_aff_mask);
		if (ret != 0)
			throw std::runtime_error("Could not set packet "
						 "cpu affinity mask");
	}

	for (int write_idx = 0; !stopped; incIndex(write_idx)) {
		write_sem.wait();
		if (stopped)
			break;
		char *p = packetPtr(write_idx);
		int ret = socket->ReceiveDataOnly(p);
		if (ret < 0)
			break;
		read_sem.post();
		++packets_caught;
	}
}


/**
 * DefaultFrameAssembler
 */

DefaultFrameAssembler::DefaultFrameAssembler(genericSocket *s, GeneralData *d,
					     cpu_set_t cpu_mask,
					     unsigned long node_mask,
					     int max_node,
					     FramePolicy fp, bool e4b)
	: packet_stream(new PacketStream(s, d, fp, cpu_mask,
					 node_mask, max_node)),
	  general_data(d),
	  frame_policy(fp), stopped(false), expand_4bits(e4b)
{
}

DefaultFrameAssembler::~DefaultFrameAssembler()
{
	delete packet_stream;
}

void DefaultFrameAssembler::stop()
{
	stopped = true;
	packet_stream->stop();
}

bool DefaultFrameAssembler::hasPendingPacket()
{
	return packet_stream->hasPendingPacket();
}

inline int DefaultFrameAssembler::getImageSize()
{
	return general_data->imageSize * (doExpand4Bits() ? 2 : 1);
}

inline bool DefaultFrameAssembler::doExpand4Bits()
{
	return (general_data->dynamicRange == 4) && expand_4bits;
}

inline void DefaultFrameAssembler::expand4Bits(char *dst, char *src,
					       int src_size)
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
				     << "src_size=" << src_size << ", "
				     << "blk=" << blk;
	int num_blocks = src_size / blk;
	const __m128i *src128 = (const __m128i *) src;
	__m128i *dst128 = (__m128i *) dst;
	const __m128i mask = _mm_set1_epi8(0xf);
	for (int i = 0; i < num_blocks; ++i) {
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

int DefaultFrameAssembler::assembleFrame(uint64_t frame,
					 RecvHeader *recv_header, char *buf)
{
	bool header_empty = true;
	int packets_per_frame = general_data->packetsPerFrame;
	DetHeader *det_header = &recv_header->detHeader;
	bool do_expand_4bits = doExpand4Bits();
	uint32_t src_dsize = general_data->dataSize;
	uint32_t last_dsize = general_data->imageSize % src_dsize;
	if (last_dsize == 0)
		last_dsize = src_dsize;
	uint32_t dst_dsize = src_dsize * (do_expand_4bits ? 2 : 1);

	recv_header->packetsMask.reset();

	PacketBlock block(packets_per_frame);
	int num_packets = packet_stream->getPacketBlock(block, frame);
	if (num_packets == 0)
		return -1;

	for (int i = 0; i < num_packets; ++i) {
		Packet& packet = block[i];
		if (!packet.valid)
			break;

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

int DefaultFrameAssembler::getNumPacketsCaught()
{
	return packet_stream->getNumPacketsCaught();
}

uint64_t DefaultFrameAssembler::getNumFramesCaught()
{
	return packet_stream->getNumFramesCaught();
}

uint64_t DefaultFrameAssembler::getLastFrameIndex()
{
	return packet_stream->getLastFrameIndex();
}


/**
 * DualPortFrameAssembler
 */

DualPortFrameAssembler::DualPortFrameAssembler(DefaultFrameAssembler *a[2])
	: assembler({a[0], a[1]})
{}

DualPortFrameAssembler::~DualPortFrameAssembler()
{}

void DualPortFrameAssembler::stop()
{
	stopAssemblers();
}

void DualPortFrameAssembler::stopAssemblers()
{
	assembler[0]->stop(), assembler[1]->stop();
}


/**
 * EigerRawFrameAssembler
 */


EigerRawFrameAssembler::EigerRawFrameAssembler(DefaultFrameAssembler *a[2])
	: DualPortFrameAssembler(a)
{
}

DualPortFrameAssembler::PortsMask
EigerRawFrameAssembler::assembleFrame(uint64_t frame, RecvHeader *recv_header,
				      char *buf)
{
	const int num_ports = 2;
	const int image_size = assembler[0]->getImageSize();
	PortsMask mask;
	for (int i = 0; i < num_ports; ++i, buf += image_size) {
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

	virtual void assemblePackets(PacketBlock block[2]) = 0;

protected:
	static const int chip_size = 256;
	static const int chip_gap = 2;
	static const int port_chips = 2;
	static const int num_ports = 2;

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
	dst.line_size = num_ports * gd->nPixelsX * dst.pixel_size;
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

	virtual void assemblePackets(PacketBlock block[2]);

private:
	static const int half_module_chips = num_ports * port_chips;
	static const int block_len = sizeof(__m128i);
	static const int block_bits = block_len * 8;
	static const int gap_bits = chip_gap * 8;

	const int chip_blocks;
	const int port_blocks;

	const __m128i m;
	const __m128i m64_0, m64_1;
	const __m128i gap_bits128;
	const __m128i block64_bits128;

	struct Worker {
		Expand4BitsHelper& h;

		bool v[num_ports], valid_data;
		const __m128i *s[num_ports], *src128;
		__m128i *dst128;
		int dest_misalign;
		int shift_l;
		__m128i shift_l128, shift_r128;
		__m128i prev;

		Worker(Expand4BitsHelper& helper);

		int load_packet(PacketBlock block[2], int packet);
		void load_dst128();
		void load_shift_store128();
		void pad_dst128();
		void sync_dst128();

		void assemblePackets(PacketBlock block[2]);
	};
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

inline Expand4BitsHelper::Worker::Worker(Expand4BitsHelper& helper)
	: h(helper)
{
}

inline int Expand4BitsHelper::Worker::load_packet(PacketBlock block[2],
						  int packet)
{
	Packet& p0 = block[0][packet];
	Packet& p1 = block[1][packet];
	s[0] = (const __m128i *) (p0.buf + h.src.offset);
	s[1] = (const __m128i *) (p1.buf + h.src.offset);
	if ((((unsigned long) s[0] | (unsigned long) s[1]) & 15) != 0) {
		FILE_LOG(logERROR) << "Missaligned src";
		return -1;
	}
	v[0] = p0.valid;
	v[1] = p1.valid;
	return 0;
}

inline void Expand4BitsHelper::Worker::load_dst128()
{
	char *d = h.buf + h.dst.offset;
	dest_misalign = ((unsigned long) d & 15);
	dst128 = (__m128i *) (d - dest_misalign);
	shift_l = dest_misalign * 8;
	shift_l128 = _mm_set_epi64x(0, shift_l % 64);
	shift_r128 = _mm_sub_epi64(h.block64_bits128, shift_l128);
	if (shift_l != 0) {
		__m128i m0;
		m0 = _mm_srl_epi64(_mm_set1_epi8(0xff), shift_r128);
		if (shift_l < 64)
			m0 = _mm_and_si128(m0, h.m64_0);
		prev = _mm_and_si128(_mm_load_si128(dst128), m0);
	} else {
		prev = _mm_setzero_si128();
	}
}

inline void Expand4BitsHelper::Worker::load_shift_store128()
{
	__m128i p4_raw;
	if (valid_data)
		p4_raw = _mm_load_si128(src128);
	else
		p4_raw = _mm_set1_epi8(0xff);
	++src128;
	__m128i p4_shr = _mm_srli_epi16(p4_raw, 4);
	__m128i i8_0 = _mm_and_si128(p4_raw, h.m);
	__m128i i8_1 = _mm_and_si128(p4_shr, h.m);
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

inline void Expand4BitsHelper::Worker::pad_dst128()
{
	shift_l += h.gap_bits;
	if (shift_l % 64 == 0)
		shift_l128 = _mm_setzero_si128();
	else
		shift_l128 = _mm_add_epi64(shift_l128, h.gap_bits128);
	shift_r128 = _mm_sub_epi64(h.block64_bits128, shift_l128);
	if (shift_l == h.block_bits) {
		_mm_store_si128(dst128++, prev);
		prev = _mm_setzero_si128();
		shift_l = 0;
	}
}

inline void Expand4BitsHelper::Worker::sync_dst128()
{
	if (shift_l != 0) {
		__m128i m0;
		m0 = _mm_sll_epi64(_mm_set1_epi8(0xff), shift_l128);
		if (shift_l >= 64)
			m0 = _mm_and_si128(m0, h.m64_1);
		__m128i a = _mm_and_si128(_mm_load_si128(dst128), m0);
		_mm_store_si128(dst128, _mm_or_si128(prev, a));
	}
}

inline void Expand4BitsHelper::Worker::assemblePackets(PacketBlock block[2])
{
	const int& hm_chips = h.half_module_chips;
	int packet = h.first_idx;
	load_dst128();
	for (int i = 0; i < h.frame_packets; ++i, packet += h.idx_inc) {
		if (load_packet(block, packet) < 0)
			return;
		for (int l = 0; l < h.packet_lines; ++l) {
			int chip_count = 0;
			for (int p = 0; p < h.num_ports; ++p) {
				valid_data = v[p];
				src128 = s[p];
				for (int c = 0; c < h.port_chips; ++c) {
					for (int b = 0; b < h.chip_blocks; ++b)
						load_shift_store128();
					if (++chip_count % hm_chips > 0)
						pad_dst128();
				}
				s[p] += h.port_blocks;
			}
		}
	}
	sync_dst128();
}

void Expand4BitsHelper::assemblePackets(PacketBlock block[2])
{
	Worker w(*this);
	w.assemblePackets(block);
}

/**
 * CopyHelper
 */

class CopyHelper : public EigerStdFrameAssembler::Helper {
public:
	CopyHelper(GeneralData *gd, bool flipped)
		: Helper(gd, flipped)
	{}
	
	virtual void assemblePackets(PacketBlock block[2]);
};

void CopyHelper::assemblePackets(PacketBlock block[2])
{
	int packet = first_idx;
	char *d = buf + dst.offset;
	for (int i = 0; i < frame_packets; ++i, packet += idx_inc) {
		Packet *line_packet[2] = {&block[0][packet],
					  &block[1][packet]};
		char *s[num_ports] = {line_packet[0]->buf + src.offset,
				     line_packet[1]->buf + src.offset};
		for (int l = 0; l < packet_lines; ++l) {
			char *ld = d;
			for (int p = 0; p < num_ports; ++p) {
				char *ls = s[p];
				for (int c = 0; c < port_chips; ++c) {
					if (line_packet[p]->valid)
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

EigerStdFrameAssembler::EigerStdFrameAssembler(DefaultFrameAssembler *a[2],
					       bool flipped)
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
	helper = h;
}

EigerStdFrameAssembler::~EigerStdFrameAssembler()
{
	delete helper;
}

DualPortFrameAssembler::PortsMask
EigerStdFrameAssembler::assembleFrame(uint64_t frame, RecvHeader *recv_header,
				      char *buf)
{
	PortsMask mask;

	const int num_ports = 2;

	helper->startNewFrame(buf);

	DefaultFrameAssembler **a = assembler;
	bool header_empty = true;
	const int packets_per_frame = a[0]->general_data->packetsPerFrame;

	DetHeader *det_header = &recv_header->detHeader;
	det_header->frameNumber = frame;
	det_header->packetNumber = 0;

	PacketBlock block[2] = {packets_per_frame, packets_per_frame};
	for (int i = 0; i < num_ports; ++i) {
		PacketStream *ps = a[i]->packet_stream;
		int packet_count = ps->getPacketBlock(block[i], frame);
		if (packet_count == 0)
			continue;
		mask.set(i, true);
		det_header->packetNumber += packet_count;

		//write header
		if (header_empty) {
			Packet& p = block[i][0];
			const int det_header_len = sizeof(*det_header);
			memcpy(det_header, p.header, det_header_len);
			header_empty = false;
		}
	}

	FramePolicy policy = a[0]->frame_policy;
	bool fp_partial = (policy == slsReceiverDefs::DISCARD_PARTIAL_FRAMES);
	if (fp_partial && (mask.count() != num_ports))
		return PortsMask();

	if (mask.any())
		helper->assemblePackets(block);

	return mask;
}
