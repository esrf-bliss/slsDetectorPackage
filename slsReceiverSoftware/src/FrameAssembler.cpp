/************************************************
 * @file FrameAssembler.cpp
 * @short helper classes assembling frames
 * from udp packets
 ***********************************************/


#include "FrameAssembler.h"

int DefaultFrameAssembler::assembleFrame(uint64_t frame, RecvHeader *recv_header, char *buf)
{
	bool header_empty = true;
	int num_packets = 0;
	int packets_per_frame = general_data->packetsPerFrame;
	PacketStream::DetHeader *det_header = &recv_header->detHeader;
	uint32_t dsize = general_data->dataSize;
	uint32_t last_dsize = general_data->imageSize % dsize;
	if (last_dsize == 0)
		last_dsize = dsize;

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
		char *dest = buf + pnum * dsize;
		bool last_packet = (pnum == (packets_per_frame - 1));
		uint32_t copy_dsize = last_packet ? last_dsize : dsize;
		if (general_data->myDetectorType == slsReceiverDefs::GOTTHARD) {
			// Gotthard data:
			//   1st packet: CACA + CACA, (640 - 1) * 2 bytes data
			//   2nd packet: (1 + 640) * 2 bytes data
			if (pnum == 0) {
				packet.buf += 4;
				copy_dsize -= 2;
			} else {
				dest -= 2;
				copy_dsize += 2;
			}
		}
		memcpy(dest, packet.buf, copy_dsize);

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
