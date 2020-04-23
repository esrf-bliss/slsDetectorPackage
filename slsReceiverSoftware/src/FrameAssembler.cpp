/************************************************
 * @file FrameAssembler.cpp
 * @short helper classes assembling frames
 * from udp packets
 ***********************************************/

#include "FrameAssembler.h"
#include "Timestamp.h"

#include <emmintrin.h>

using namespace FrameAssembler;

/**
 * StdPacketImpl
 */

template <class PD>
void Packet<PD>::checkConsistency(GeneralData *gd)
{
	typedef typename PD::EmptyHeader EmptyHeader;
	if (gd->emptyHeader != EmptyHeader::size) {
		std::ostringstream error;
		error << "EmptyHeader size mismatch: "
		      << "got " << gd->emptyHeader << ", "
		      << "expected " << sizeof(EmptyHeader);
		std::cerr << error.str() << std::endl;
		throw std::runtime_error(error.str());
	}
}

/**
 * StdPacketImpl
 */

template <class PD>
void StdPacketImpl<PD>::fillDetHeader(DetHeader *det_header)
{
	memcpy(det_header, header(), sizeof(*det_header));
}

/**
 * LegacyPacket
 */

template <class F>
void LegacyPacketImpl<F>::initSoftHeader()
{
	int thread_idx = 0;
	uint64_t packet_bid;
	uint32_t packet_subframe;
	generalData()->GetHeaderInfo(thread_idx, Base::networkHeader(),
				     stream_data->odd_numbering,
				     Base::softHeader()->packet_frame,
				     Base::softHeader()->packet_number,
				     packet_subframe, packet_bid);
}

template <class F>
void LegacyPacketImpl<F>::fillDetHeader(DetHeader *det_header)
{
	memset(det_header, 0, sizeof(*det_header));
	det_header->frameNumber = frame();
	det_header->detType = (uint8_t) generalData()->myDetectorType;
	det_header->version = (uint8_t) SLS_DETECTOR_HEADER_VERSION;
}

/**
 * GotthardPacket
 */

inline void GotthardPacket::initSoftHeader()
{
	if (stream_data->first_packet) {
		bool& odd_numbering = stream_data->odd_numbering;
		GeneralData *gd = generalData();
		int thread_idx = 0;
		char *start = networkHeader();
		odd_numbering = gd->SetOddStartingPacket(thread_idx, start);
		stream_data->first_packet = false;
	}
	Base::initSoftHeader();
}


/**
 * PacketBlock
 */

template <class P>
PacketBlock<P>::PacketBlock(PacketStream<P> *s, char *b)
	: ps(s), buf(b), valid_packets(0)
{
}

template <class P>
PacketBlock<P>::~PacketBlock()
{
	if (ps)
		ps->releasePacketBlock(this);
}

template <class P>
P PacketBlock<P>::operator [](unsigned int i)
{
	return P(buf + ps->packet_len * i, &ps->stream_data);
}

template <class P>
void PacketBlock<P>::setValid(unsigned int i, bool valid)
{
	(*this)[i].softHeader()->valid = valid;
	if (valid)
		++valid_packets;
}

template <class P>
void PacketBlock<P>::moveToGood(P& p)
{
	P dst = (*this)[p.number()];
	typedef typename P::SoftHeader SoftHeader;
	int len = sizeof(SoftHeader) + ps->general_data->packetSize;
	memcpy(dst.buffer, p.buffer, len);
	p.softHeader()->valid = false;
	dst.softHeader()->valid = true;
}

/**
 * PacketStream
 */

const int MaxBufferFrames = 4;

template <class P>
void PacketStream<P>::releasePacketBlock(PacketBlock<P> *block)
{
	MutexLock l(free_cond.getMutex());
	free_queue.push(block->buf);
	free_cond.signal();
}

template <class P>
bool PacketStream<P>::canDiscardFrame(int received_packets)
{
	return ((frame_policy ==
		 slsReceiverDefs::DISCARD_PARTIAL_FRAMES) ||
		((frame_policy ==
		  slsReceiverDefs::DISCARD_EMPTY_FRAMES) &&
		 !received_packets));
}

template <class P>
PacketBlock<P> *PacketStream<P>::getPacketBlock(uint64_t frame)
{
	class WaitingCountHelper {
	public:
		WaitingCountHelper(PacketStream *s) : ps(s)
		{ ++ps->waiting_reader_count; }
		~WaitingCountHelper()
		{ --ps->waiting_reader_count; }
	private:
		PacketStream *ps;
	};

	PacketBlock<P> *block;
	{
		MutexLock bl(block_cond.getMutex());
		WaitingCountHelper h(this);
		MapIterator it;
		while (!stopped) {
			if (!packet_block_map.empty()) {
				it = packet_block_map.begin();
				bool too_old = (it->first > frame);
				if (too_old)
					return NULL;
			}
			it = packet_block_map.find(frame);
			if (it != packet_block_map.end())
				break;
			block_cond.wait();
		}
		if (stopped)
			return NULL;

		block = it->second;
		packet_block_map.erase(it);
	}

	bool full_frame = block->hasFullFrame();
	{
		MutexLock l(mutex);
		if (full_frame)
			++frames_caught;
		if (frame > last_frame)
			last_frame = frame;
	}
	if (!full_frame && canDiscardFrame(block->getValidPackets())) {
		delete block;
		return NULL;
	}
	return block;
}

template <class P>
bool PacketStream<P>::hasPendingPacket()
{
	MutexLock l(free_cond.getMutex());
	return (free_queue.size() != num_frames);
}

template <class P>
PacketStream<P>::PacketStream(genericSocket *s, GeneralData *d, FramePolicy fp,
			      cpu_set_t cpu_mask,
			      unsigned long node_mask, int max_node)
	: socket(s),
	  general_data(d),
	  frame_policy(fp),
	  num_frames(MaxBufferFrames),
	  packets_caught(0),
	  frames_caught(0),
	  last_frame(0),
	  stream_data(general_data),
	  stopped(false),
	  waiting_reader_count(0),
	  cpu_aff_mask(cpu_mask),
	  packet_delay_stat(1e6)
{
	P::checkConsistency(general_data);
	initMem(node_mask, max_node);
	thread = new WriterThread(this);
}

template <class P>
PacketStream<P>::~PacketStream()
{
	stop();
	delete thread;
	releaseReadyPacketBlocks();
	waitUsedPacketBlocks();
	printStat();
}

template <class P>
void PacketStream<P>::releaseReadyPacketBlocks()
{
	MutexLock l(block_cond.getMutex());
	while (waiting_reader_count > 0)
		usleep(5000);
	while (!packet_block_map.empty()) {
		MapIterator it = packet_block_map.begin();
		PacketBlock<P> *block = it->second;
		packet_block_map.erase(it);
		MutexUnlock u(l);
		delete block;
	}
}

template <class P>
void PacketStream<P>::waitUsedPacketBlocks()
{
	const double wait_reader_timeout = 1;
	Timestamp t0;
	t0.latchNow();
	while (hasPendingPacket()) {
		Timestamp t;
		t.latchNow();
		if (t - t0 > wait_reader_timeout)
			break;
		usleep(5000);
	}
	if (hasPendingPacket()) {
		MutexLock l(free_cond.getMutex());
		std::ostringstream error;
		error << "[" << socket->getPortNumber() << "]: "
		      << "Missing free frames after "
		      << wait_reader_timeout << "sec: "
		      << "expected " << num_frames << ", "
		      << "got " << free_queue.size();
		std::cerr << error.str() << std::endl;
	}
}

template <class P>
void PacketStream<P>::printStat()
{
	MutexLock l(mutex);
	std::ostringstream msg;
	msg << "[" << socket->getPortNumber() << "]: "
	    << "packet_delay_stat=" << packet_delay_stat.calcLinRegress();
	std::cout << msg.str() << std::endl;
}

template <class P>
void PacketStream<P>::stop()
{
	stopped = true;
	{
		MutexLock l(block_cond.getMutex());
		block_cond.broadcast();
	}
	{
		MutexLock l(free_cond.getMutex());
		free_cond.signal();
	}
}

template <class P>
void PacketStream<P>::clearBuffer()
{
	packet_buffer_array.clear();
}

template <class P>
int PacketStream<P>::getNumPacketsCaught()
{
	MutexLock l(mutex);
	return packets_caught;
}

template <class P>
uint64_t PacketStream<P>::getNumFramesCaught()
{
	MutexLock l(mutex);
	return frames_caught;
}

template <class P>
uint64_t PacketStream<P>::getLastFrameIndex()
{
	MutexLock l(mutex);
	return last_frame;
}

template <class P>
uint32_t PacketStream<P>::getNbPacketFrames()
{
	return general_data->packetsPerFrame;
}

template <class P>
void PacketStream<P>::initMem(unsigned long node_mask, int max_node)
{
	const int data_align = 128 / 8;
	GeneralData& gd = *general_data;
	packet_len = gd.packetSize;
	typedef typename P::SoftHeader SoftHeader;
	size_t header_len = sizeof(SoftHeader) + gd.headerSizeinPacket;
	size_t misalign = header_len % data_align;
	header_pad = misalign ? (data_align - misalign) : 0;
	packet_len += header_pad;
	misalign = packet_len % data_align;
	packet_len += misalign ? (data_align - misalign) : 0;
	int frame_len = packet_len * getNbPacketFrames();
	packet_buffer_array.alloc(frame_len * num_frames, node_mask, max_node);

	char *p = packet_buffer_array.getPtr() + header_pad;
	for (unsigned int i = 0; i < num_frames; ++i, p += frame_len)
		free_queue.push(p);
}

template <class P>
PacketBlock<P> *PacketStream<P>::getEmptyBlock()
{
	MutexLock l(free_cond.getMutex());
	while (!stopped && free_queue.empty())
		free_cond.wait();
	if (stopped)
		return NULL;
	char *b = free_queue.front();
	free_queue.pop();
	l.unlock();
	return new PacketBlock<P>(this, b);
}

template <class P>
void PacketStream<P>::addPacketBlock(FramePacketBlock frame_block)
{
	MutexLock l(block_cond.getMutex());
	packet_block_map.insert(frame_block);
	block_cond.broadcast();
}


template <class P>
class PacketStream<P>::WriterThread {
public:
	WriterThread(PacketStream *s)
		: ps(s), frame_packets(ps->getNbPacketFrames()),
		  block(NULL), curr_frame(0), curr_packet(-1)
	{
		int ret;
		ret = pthread_create(&thread, NULL, threadFunctionStatic, this);
		if (ret != 0) {
			const char *error = "Could not start writer thread";
			std::cerr << error << std::endl;
			throw std::runtime_error(error);
		}

		struct sched_param param;
		param.sched_priority = 90;
		ret = pthread_setschedparam(thread, SCHED_FIFO, &param);
		if (ret != 0)
			std::cerr << "Could not set packet thread RT priority!"
				  << std::endl;
	}

	~WriterThread()
	{
		void *r;
		pthread_join(thread, &r);
	}

private:
	typedef AutoPtr<PacketBlock<P> > BlockPtr;

	bool checkBlock()
	{
		if (!block)
			block = ps->getEmptyBlock();
		return block;
	}

	P getNextPacket()
	{
	        return (*block)[++curr_packet];
	}

	FramePacketBlock finishPacketBlock()
	{
		curr_packet = -1;
		return FramePacketBlock(curr_frame, block.forget());
	}

	void setInvalidPacketsUntil(uint32_t good_packet)
	{
		// curr_packet validity was already set
		while (++curr_packet != good_packet)
			block->setValid(curr_packet, false);
	}

	void addPacketDelayStat(P& packet)
	{
		Timestamp t;
		t.latchNow();
		long packet_idx = ((packet.frame() - 1) * frame_packets
				   + packet.number());
		if (packet_idx == 0)
			t0 = t;
		MutexLock l(ps->mutex);
		ps->packet_delay_stat.add(packet_idx, t - t0);
	}

	bool addPacket(P& packet)
	{
		addPacketDelayStat(packet);

		uint64_t packet_frame = packet.frame();
		uint32_t packet_number = packet.number();
		// moveToGood manages both src & dst valid flags
		if (curr_packet == 0)
			curr_frame = packet_frame;
		if (packet_frame != curr_frame) {
			BlockPtr new_block = ps->getEmptyBlock();
			if (new_block)
				new_block->moveToGood(packet);
			else
				block->setValid(curr_packet, false);
			setInvalidPacketsUntil(frame_packets);
			ps->addPacketBlock(finishPacketBlock());
			if (!new_block)
				return false;
			block = new_block.forget();
			curr_frame = packet_frame;
			setInvalidPacketsUntil(packet_number);
		} else if (packet_number != curr_packet) {
			block->moveToGood(packet);
			setInvalidPacketsUntil(packet_number);
		} else {
			block->setValid(curr_packet, true);
		}
		
		if (curr_packet == (frame_packets - 1))
			ps->addPacketBlock(finishPacketBlock());
		return true;
	}

	bool processOnePacket()
	{
		if (!checkBlock())
			return false;

		P packet = getNextPacket();
		char *b = packet.networkBuffer();
		int ret = ps->socket->ReceiveDataOnly(b);
		if (ret < 0)
			return false;

		packet.initSoftHeader();

		return addPacket(packet);
	}

	static void *threadFunctionStatic(void *data)
	{
		WriterThread *wt = static_cast<WriterThread *>(data);
		wt->threadFunction();
		return NULL;
	}

	void threadFunction()
	{
		cpu_set_t& cpu_aff_mask = ps->cpu_aff_mask;
		if (CPU_COUNT(&cpu_aff_mask) != 0) {
			int size = sizeof(cpu_aff_mask);
			int ret = sched_setaffinity(0, size, &cpu_aff_mask);
			if (ret != 0)
				std::cerr << "Could not set writer thread "
					  << "cpu affinity mask" << std::endl;
		}

		while (true) {
			if (!processOnePacket())
				break;

			MutexLock l(ps->mutex);
			++ps->packets_caught;
		}
	}

	PacketStream *ps;
	const uint32_t frame_packets;
	Timestamp t0;
	BlockPtr block;
	uint64_t curr_frame;
	uint32_t curr_packet;
	pthread_t thread;
};


/**
 * DefaultFrameAssemblerBase
 */

DefaultFrameAssemblerBase::DefaultFrameAssemblerBase(GeneralData *d, bool e4b)
	: general_data(d), expand_4bits(e4b)
{
}

DefaultFrameAssemblerBase::~DefaultFrameAssemblerBase()
{
}

inline GeneralData *DefaultFrameAssemblerBase::getGeneralData()
{
	return general_data;
}

inline bool DefaultFrameAssemblerBase::doExpand4Bits()
{
	return (general_data->dynamicRange == 4) && expand_4bits;
}


/**
 * DefaultFrameAssembler
 */

template <class P>
DefaultFrameAssembler<P>::DefaultFrameAssembler(genericSocket *s,
						GeneralData *d,
						cpu_set_t cpu_mask,
						unsigned long node_mask,
						int max_node,
						FramePolicy fp, bool e4b)
	: DefaultFrameAssemblerBase(d, e4b),
	  packet_stream(new PacketStream<P>(s, d, fp, cpu_mask, node_mask,
					    max_node)),
	  frame_policy(fp)
{
}

template <class P>
DefaultFrameAssembler<P>::~DefaultFrameAssembler()
{
	delete packet_stream;
}

template <class P>
void DefaultFrameAssembler<P>::stop()
{
	packet_stream->stop();
}

template <class P>
bool DefaultFrameAssembler<P>::hasPendingPacket()
{
	return packet_stream->hasPendingPacket();
}

template <class P>
int DefaultFrameAssembler<P>::getImageSize()
{
	return general_data->imageSize * (doExpand4Bits() ? 2 : 1);
}

template <class P>
void DefaultFrameAssembler<P>::expand4Bits(char *dst, char *src, int src_size)
{
	unsigned long s = (unsigned long) src;
	unsigned long d = (unsigned long) dst;
	if (((s & 15) != 0) || ((d & 15) != 0)) {
		FILE_LOG(logERROR) << "Missaligned src/dst";
		return;
	}

	const int blk = sizeof(__m128i);
	if ((src_size % blk) != 0) {
		FILE_LOG(logWARNING) << "len misalignment: "
				     << "src_size=" << src_size << ", "
				     << "blk=" << blk;
	}
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

template <class P>
int DefaultFrameAssembler<P>::assembleFrame(uint64_t frame,
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

	AutoPtr<PacketBlock<P> > block = packet_stream->getPacketBlock(frame);
	if (!block || (block->getValidPackets() == 0))
		return -1;

	uint32_t prev_adjust = 0;
	for (int i = 0; i < packets_per_frame; ++i) {
		P packet = (*block)[i];
		if (!packet.valid())
			continue;

		int pnum = packet.number();

		//copy packet
		char *dst = buf + pnum * dst_dsize;
		bool last_packet = (pnum == (packets_per_frame - 1));
		uint32_t copy_dsize = last_packet ? last_dsize : src_dsize;
		uint32_t size_adjust = packet.sizeAdjust();
		copy_dsize += size_adjust;
		dst += prev_adjust;
		prev_adjust = size_adjust;
		if (do_expand_4bits)
			expand4Bits(dst, packet.data(), copy_dsize);
		else
			memcpy(dst, packet.data(), copy_dsize);

		recv_header->packetsMask[pnum] = 1;

		//write header
		if (header_empty) {
			packet.fillDetHeader(det_header);
			header_empty = false;
		}
	}

	if (header_empty)
		det_header->frameNumber = frame;
	det_header->packetNumber = block->getValidPackets();

	return 0;
}

template <class P>
int DefaultFrameAssembler<P>::getNumPacketsCaught()
{
	return packet_stream->getNumPacketsCaught();
}

template <class P>
uint64_t DefaultFrameAssembler<P>::getNumFramesCaught()
{
	return packet_stream->getNumFramesCaught();
}

template <class P>
uint64_t DefaultFrameAssembler<P>::getLastFrameIndex()
{
	return packet_stream->getLastFrameIndex();
}


template <class P>
void DefaultFrameAssembler<P>::clearBuffers()
{
	packet_stream->clearBuffer();
}

DefaultFrameAssemblerBase *
DefaultFrameAssemblerBase::create(genericSocket *s, GeneralData *d,
				  cpu_set_t cpu_mask,
				  unsigned long node_mask, int max_node, 
				  FramePolicy fp, bool e4b)
{
	DefaultFrameAssemblerBase *a;
	switch (d->myDetectorType) {
	case slsReceiverDefs::EIGER:
		a = new EigerAssembler(s, d, cpu_mask, node_mask, max_node,
				       fp, e4b);
		break;
	case slsReceiverDefs::JUNGFRAU:
		a = new JungfrauAssembler(s, d, cpu_mask, node_mask, max_node,
					  fp, e4b);
		break;
	case slsReceiverDefs::GOTTHARD:
		a = new GotthardAssembler(s, d, cpu_mask, node_mask, max_node,
					  fp, e4b);
		break;
	default:
		a = new LegacyAssembler(s, d, cpu_mask, node_mask, max_node,
					fp, e4b);
	}
	return a;
}

/**
 * DualPortFrameAssembler
 */

DualPortFrameAssembler::DualPortFrameAssembler(DefaultFrameAssemblerBase *a[2])
	: assembler{a[0], a[1]}
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


EigerRawFrameAssembler::EigerRawFrameAssembler(DefaultFrameAssemblerBase *a[2])
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
	typedef PacketBlock<EigerPacket> EigerBlock;
	typedef PacketStream<EigerPacket> EigerStream;

	Helper(GeneralData *gd, bool f);
	virtual ~Helper() {}

	float getSrcPixelBytes()
	{
		return float(general_data->dynamicRange) / 8;
	}

	float getDstPixelBytes()
	{
		int factor = (general_data->dynamicRange == 4) ? 2 : 1;
		return getSrcPixelBytes() * factor;
	}

	virtual void assemblePackets(EigerBlock *block[2], char *buf) = 0;

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

	virtual void assemblePackets(EigerBlock *block[2], char *buf) override;

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

		int load_packet(EigerBlock *block[2], int packet);
		void load_dst128(char *buf);
		void load_shift_store128();
		void pad_dst128();
		void sync_dst128();

		void assemblePackets(EigerBlock *block[2], char *buf);
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

inline int Expand4BitsHelper::Worker::load_packet(EigerBlock *block[2],
						  int packet)
{
	EigerPacket p0 = (*block[0])[packet];
	EigerPacket p1 = (*block[1])[packet];
	s[0] = (const __m128i *) (p0.data() + h.src.offset);
	s[1] = (const __m128i *) (p1.data() + h.src.offset);
	if ((((unsigned long) s[0] | (unsigned long) s[1]) & 15) != 0) {
		FILE_LOG(logERROR) << "Missaligned src";
		return -1;
	}
	v[0] = p0.valid();
	v[1] = p1.valid();
	return 0;
}

inline void Expand4BitsHelper::Worker::load_dst128(char *buf)
{
	char *d = buf + h.dst.offset;
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

inline void Expand4BitsHelper::Worker::assemblePackets(EigerBlock *block[2],
						       char *buf)
{
	const int& hm_chips = h.half_module_chips;
	int packet = h.first_idx;
	load_dst128(buf);
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

void Expand4BitsHelper::assemblePackets(EigerBlock *block[2], char *buf)
{
	Worker w(*this);
	w.assemblePackets(block, buf);
}

/**
 * CopyHelper
 */

class CopyHelper : public EigerStdFrameAssembler::Helper {
public:
	CopyHelper(GeneralData *gd, bool flipped)
		: Helper(gd, flipped)
	{}
	
	virtual void assemblePackets(EigerBlock *block[2], char *buf) override;
};

void CopyHelper::assemblePackets(EigerBlock *block[2], char *buf)
{
	int packet = first_idx;
	char *d = buf + dst.offset;
	for (int i = 0; i < frame_packets; ++i, packet += idx_inc) {
		EigerPacket line_packet[2] = {(*block[0])[packet],
					      (*block[1])[packet]};
		char *s[num_ports] = {line_packet[0].data() + src.offset,
				      line_packet[1].data() + src.offset};
		for (int l = 0; l < packet_lines; ++l) {
			char *ld = d;
			for (int p = 0; p < num_ports; ++p) {
				char *ls = s[p];
				for (int c = 0; c < port_chips; ++c) {
					if (line_packet[p].valid())
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

EigerStdFrameAssembler::EigerStdFrameAssembler(DefaultFrameAssemblerBase *a[2],
					       bool flipped)
	: DualPortFrameAssembler(a)
{
	GeneralData *gd = a[0]->getGeneralData();
	if (!gd->tgEnable) {
		const char *error = "10 Giga not enabled!";
		std::cerr << error << std::endl;
		throw std::runtime_error(error);
	}
	
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

	EigerAssembler **a = reinterpret_cast<EigerAssembler **>(assembler);
	bool header_empty = true;

	DetHeader *det_header = &recv_header->detHeader;
	det_header->frameNumber = frame;
	det_header->packetNumber = 0;

	AutoPtr<Helper::EigerBlock> block[2];
	for (int i = 0; i < num_ports; ++i) {
		Helper::EigerStream *ps = a[i]->packet_stream;
		block[i] = ps->getPacketBlock(frame);
		int packet_count = block[i] ? block[i]->getValidPackets() : 0;
		if (packet_count == 0)
			continue;
		mask.set(i, true);
		det_header->packetNumber += packet_count;

		//write header
		if (header_empty) {
			EigerPacket p = (*block[i])[0];
			p.fillDetHeader(det_header);
			header_empty = false;
		}
	}

	FramePolicy policy = a[0]->frame_policy;
	bool fp_partial = (policy == slsReceiverDefs::DISCARD_PARTIAL_FRAMES);
	if (fp_partial && (mask.count() != num_ports))
		return PortsMask();

	if (mask.any()) {
		Helper::EigerBlock *blocks[2] = { block[0], block[1] };
		helper->assemblePackets(blocks, buf);
	}

	return mask;
}
