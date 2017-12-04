#ifndef RECEIVER_DEFS_H
#define RECEIVER_DEFS_H

#include "sls_receiver_defs.h"

#include <stdint.h> 


/**
 * structure of an eiger packet header
 * subframenum subframe number for 32 bit mode (already written by firmware)
 * missingpacket explicitly put to 0xFF to recognize it in file read (written by software)
 * portnum 0 for the first port and 1 for the second port (written by software to file)
 * dynamicrange dynamic range or bits per pixel (written by software to file)
 */
typedef struct {
	unsigned char subFrameNumber[4];
	unsigned char missingPacket[2];
	unsigned char portIndex[1];
	unsigned char dynamicRange[1];
} eiger_packet_header_t;
/**
 * structure of an eiger packet footer
 * framenum 48 bit frame number (already written by firmware)
 * packetnum packet number (already written by firmware)
 */
typedef struct	{
	unsigned char frameNumber[6];
	unsigned char packetNumber[2];
} eiger_packet_footer_t;

/**
 * structure of an jungfrau packet header
 * empty header
 * framenumber
 * packetnumber
 */
typedef struct {
	unsigned char emptyHeader[6];
	unsigned char reserved[4];
	unsigned char packetNumber[1];
	unsigned char frameNumber[3];
	unsigned char bunchid[8];
} jfrau_packet_header_t;


#define GOODBYE 							-200

#define DO_NOTHING		0
#define DO_EVERYTHING	1

#define BUF_SIZE        		(16*1024*1024) //16mb
#define SAMPLE_TIME_IN_NS		100000000//100ms
#define MAX_JOBS_PER_THREAD		1000
#define HEADER_SIZE_NUM_TOT_PACKETS	4
#define HEADER_SIZE_NUM_FRAMES	2
#define HEADER_SIZE_NUM_PACKETS	1
#define ALL_MASK_32				0xFFFFFFFF

#define SLS_DETECTOR_HEADER_VERSION 		0x1
#define SLS_DETECTOR_JSON_HEADER_VERSION 	0x2
//#define FILE_FRAME_HEADER_LENGTH	 	(8*3)
//#define FILE_HEADER_TIMESTAMP_OFFSET	8	//start of frame/ bunch id
//#define FILE_HEADER_EXPLENGTH_OFFSET	16	//exposure length/ sub frame number



//all max frames defined in sls_receiver_defs.h.  20000 gotthard, 100000 for short gotthard, 1000 for moench, eiger 20000



#define GOTTHARD_FIFO_SIZE					25000 //cannot be less than max jobs per thread = 1000
/*#define GOTTHARD_ALIGNED_FRAME_SIZE		4096*/
#define GOTTHARD_PACKETS_PER_FRAME			2
#define GOTTHARD_ONE_PACKET_SIZE			1286
#define GOTTHARD_ONE_DATA_SIZE				1280
#define GOTTHARD_BUFFER_SIZE 				(GOTTHARD_ONE_PACKET_SIZE*GOTTHARD_PACKETS_PER_FRAME) 	//1286*2
#define GOTTHARD_DATA_BYTES	 				(GOTTHARD_ONE_DATA_SIZE*GOTTHARD_PACKETS_PER_FRAME)		//1280*2

#define GOTTHARD_FRAME_INDEX_MASK			0xFFFFFFFE
#define GOTTHARD_FRAME_INDEX_OFFSET			1
#define GOTTHARD_PACKET_INDEX_MASK			0x1

#define GOTTHARD_PIXELS_IN_ROW				1280
#define GOTTHARD_PIXELS_IN_COL				1


#define GOTTHARD_SHORT_PACKETS_PER_FRAME	1
#define GOTTHARD_SHORT_ONE_PACKET_SIZE		518
#define GOTTHARD_SHORT_BUFFER_SIZE			518
#define GOTTHARD_SHORT_DATABYTES			512
#define GOTTHARD_SHORT_FRAME_INDEX_MASK		0xFFFFFFFF
#define GOTTHARD_SHORT_FRAME_INDEX_OFFSET	0
#define GOTTHARD_SHORT_PACKET_INDEX_MASK	0
#define GOTTHARD_SHORT_PIXELS_IN_ROW		256
#define GOTTHARD_SHORT_PIXELS_IN_COL		1





#define PROPIX_PIXELS_IN_ROW				22
#define PROPIX_PIXELS_IN_COL				22
#define PROPIX_DATABYTES_PER_PIXEL			2

#define PROPIX_FIFO_SIZE					25000 //cannot be less than max jobs per thread = 1000
#define PROPIX_PACKETS_PER_FRAME			2
#define PROPIX_ONE_PACKET_SIZE				1286
#define PROPIX_BUFFER_SIZE 					(PROPIX_ONE_PACKET_SIZE*PROPIX_PACKETS_PER_FRAME) 	//1286*2
//#define PROPIX_DATA_BYTES 					(1280*PROPIX_PACKETS_PER_FRAME) 	//1280*2
#define PROPIX_DATA_BYTES	 				(PROPIX_PIXELS_IN_ROW * PROPIX_PIXELS_IN_COL * PROPIX_DATABYTES_PER_PIXEL) //22 * 22 * 2

#define PROPIX_FRAME_INDEX_MASK				0xFFFFFFFE
#define PROPIX_FRAME_INDEX_OFFSET			1
#define PROPIX_PACKET_INDEX_MASK			0x1





#define MOENCH_FIFO_SIZE					2500 //cannot be less than max jobs per thread = 1000
/*#define MOENCH_ALIGNED_FRAME_SIZE			65536*/
#define MOENCH_PACKETS_PER_FRAME			40
#define MOENCH_ONE_PACKET_SIZE				1286
#define MOENCH_ONE_DATA_SIZE				1280
#define MOENCH_BUFFER_SIZE 					(MOENCH_ONE_PACKET_SIZE*MOENCH_PACKETS_PER_FRAME) 	//1286*40
#define MOENCH_DATA_BYTES	 				(MOENCH_ONE_DATA_SIZE*MOENCH_PACKETS_PER_FRAME)		//1280*40

#define MOENCH_FRAME_INDEX_MASK				0xFFFFFF00
#define MOENCH_FRAME_INDEX_OFFSET			8
#define MOENCH_PACKET_INDEX_MASK			0xFF

#define MOENCH_BYTES_PER_ADC				(40*2)
#define MOENCH_PIXELS_IN_ONE_ROW			160
#define MOENCH_BYTES_IN_ONE_ROW				(MOENCH_PIXELS_IN_ONE_ROW*2)



#define JFRAU_FIFO_SIZE						2500 //cannot be less than max jobs per thread = 1000
#define JFRAU_PACKETS_PER_FRAME				128
#define JFRAU_HEADER_LENGTH					22
#define JFRAU_ONE_DATA_SIZE					8192
#define JFRAU_ONE_PACKET_SIZE				(JFRAU_HEADER_LENGTH+JFRAU_ONE_DATA_SIZE) //8214
#define JFRAU_DATA_BYTES	 				(JFRAU_ONE_DATA_SIZE*JFRAU_PACKETS_PER_FRAME)		//8192*128
#define JFRAU_BUFFER_SIZE 					(JFRAU_ONE_PACKET_SIZE*JFRAU_PACKETS_PER_FRAME) 	//8214*128


#define JFRAU_FRAME_INDEX_MASK				0xffffff //mask after using struct (48 bit)
#define JFRAU_FRAME_INDEX_OFFSET			0x0 //Not Applicable, use struct
#define JFRAU_PACKET_INDEX_MASK				0x0//Not Applicable, use struct

#define JFRAU_PIXELS_IN_ONE_ROW				(256*4)
#define JFRAU_PIXELS_IN_ONE_COL				(256*2)
#define JFRAU_BYTES_IN_ONE_ROW				(JFRAU_PIXELS_IN_ONE_ROW*2)



#define JCTB_FIFO_SIZE						2500 //cannot be less than max jobs per thread = 1000
/*#define MOENCH_ALIGNED_FRAME_SIZE			65536*/
#define JCTB_PACKETS_PER_FRAME			        1
#define JCTB_ONE_PACKET_SIZE				8224
#define JCTB_BUFFER_SIZE 				(JCTB_ONE_PACKET_SIZE*40) 	
#define JCTB_DATA_BYTES	 				(8192*JCTB_PACKETS_PER_FRAME)					

#define JCTB_FRAME_INDEX_MASK			0xFFFFFFFF
#define JCTB_FRAME_INDEX_OFFSET			6+8
#define JCTB_PACKET_INDEX_MASK			0xFFFFFFFF


#define JCTB_BYTES_PER_ADC		        	(2)
#define JCTB_PIXELS_IN_ONE_ROW				32
#define JCTB_BYTES_IN_ONE_ROW		       (JCTB_PIXELS_IN_ONE_ROW*2)




#define EIGER_MAX_PORTS 					2
#define EIGER_HEADER_PACKET_LENGTH			48

#define EIGER_FIFO_SIZE						100
/*#define EIGER_ALIGNED_FRAME_SIZE			65536*/
#define EIGER_ONE_GIGA_CONSTANT				16
#define EIGER_TEN_GIGA_CONSTANT				4
//#define EIGER_PACKETS_PER_FRAME_COSTANT		(16*EIGER_MAX_PORTS)//*bit mode  4*16=64, 8*16=128, 16*16=256, 32*16=512
#define EIGER_ONE_GIGA_ONE_PACKET_SIZE		1040
#define EIGER_ONE_GIGA_ONE_DATA_SIZE		1024
#define EIGER_TEN_GIGA_ONE_PACKET_SIZE		4112
#define EIGER_TEN_GIGA_ONE_DATA_SIZE		4096
#define EIGER_DATA_PACKET_HEADER_SIZE		8
//#define EIGER_BUFFER_SIZE_CONSTANT			(EIGER_ONE_PACKET_SIZE*EIGER_PACKETS_PER_FRAME_COSTANT)//1040*16*2//*bit mode
//#define EIGER_DATA_BYTES_CONSTANT 			(EIGER_ONE_DATA_SIZE*EIGER_PACKETS_PER_FRAME_COSTANT)	//1024*16*2//*bit mode

#define EIGER_FRAME_INDEX_MASK				0xFFFFFFFF //32 bit for now
#define EIGER_FRAME_INDEX_OFFSET			0
#define EIGER_PACKET_INDEX_MASK				0x0

//for each thread
#define EIGER_PIXELS_IN_ONE_ROW				(256*2)
#define EIGER_PIXELS_IN_ONE_COL				(256)


#endif
