/*
 * slsDetectorServer_defs.h
 *
 *  Created on: Jan 24, 2013
 *      Author: l_maliakal_d
 */

#ifndef SLSDETECTORSERVER_DEFS_H_
#define SLSDETECTORSERVER_DEFS_H_

#include "sls_detector_defs.h"
#include <stdint.h>

#define GOODBYE 		-200


/* examples*/
#ifdef JUNGFRAU_DHANYA
#define NCHAN 			(256*256)
#define NCHIP 			8
#define NADC			0
#else
#define NCHAN 			1
#define NCHIP 			1
#define NDAC 			1
#define NADC			1
#endif

#define NMAXMODX  		1
#define NMAXMODY 		1
#define NMAXMOD 		NMAXMODX*NMAXMODY
#define NCHANS 			NCHAN*NCHIP*NMAXMOD
#define NDACS 			NDAC*NMAXMOD



#endif /* SLSDETECTORSERVER_DEFS_H_ */