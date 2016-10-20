/*
 * error_defs.h
 *
 *  Created on: Jan 18, 2013
 *      Author: l_maliakal_d
 */

#ifndef ERROR_DEFS_H_
#define ERROR_DEFS_H_


#include <string>
#include <cstring>
using namespace std;


#include "sls_detector_defs.h"
#include <cstring>
#include <iostream>


/** Error flags */
/*Assumption: Only upto 63 detectors */
#define CRITICAL_ERROR_MASK 0xFFFFFFFF

#define MULTI_DETECTORS_NOT_ADDED			0x8000000000000000ULL


#define CANNOT_CONNECT_TO_DETECTOR  		0x8000000000000000ULL
#define CANNOT_CONNECT_TO_RECEIVER  		0x4000000000000000ULL
#define COULDNOT_SET_CONTROL_PORT			0x2000000000000000ULL
#define COULDNOT_SET_STOP_PORT				0x1000000000000000ULL
#define COULDNOT_SET_DATA_PORT				0x0800000000000000ULL
#define FILE_PATH_DOES_NOT_EXIST			0x0400000000000000ULL
#define COULDNOT_CREATE_UDP_SOCKET			0x0200000000000000ULL
#define COULDNOT_CREATE_FILE				0x0100000000000000ULL
#define COULDNOT_ENABLE_COMPRESSION			0x0080000000000000ULL
#define RECEIVER_DET_HOSTNAME_NOT_SET		0x0040000000000000ULL
#define RECEIVER_DET_HOSTTYPE_NOT_SET		0x0020000000000000ULL
#define DETECTOR_TEN_GIGA					0x0010000000000000ULL
#define DETECTOR_ACTIVATE					0x0008000000000000ULL

//											0xFFFFFFFF00000000ULL

#define COULD_NOT_CONFIGURE_MAC				0x0000000000000001ULL
#define COULDNOT_SET_NETWORK_PARAMETER		0x0000000000000002ULL
#define COULDNOT_SET_ROI					0x0000000000000004ULL
#define RECEIVER_READ_FREQUENCY				0x0000000000000008ULL
#define SETTINGS_NOT_SET					0x0000000000000010ULL
#define SETTINGS_FILE_NOT_OPEN				0x0000000000000020ULL
#define COULDNOT_START_RECEIVER				0x0000000000000040ULL // default error like starting threads
#define COULDNOT_STOP_RECEIVER				0x0000000000000080ULL
#define DETECTOR_TIMER_VALUE_NOT_SET		0x0000000000000100ULL
#define RECEIVER_ACQ_PERIOD_NOT_SET			0x0000000000000200ULL
#define RECEIVER_FRAME_NUM_NOT_SET			0x0000000000000400ULL
#define RECEIVER_DYNAMIC_RANGE				0x0000000000000800ULL
#define RECEIVER_TEN_GIGA					0x0000000000001000ULL
#define ALLTIMBITS_NOT_SET					0x0000000000002000ULL
#define COULD_NOT_SET_SPEED_PARAMETERS		0x0000000000004000ULL
#define COULD_NOT_SET_READOUT_FLAGS			0x0000000000008000ULL
#define COULD_NOT_SET_FIFO_DEPTH			0x0000000000010000ULL
#define COULD_NOT_SET_COUNTER_BIT			0x0000000000020000ULL
#define COULD_NOT_PULSE_PIXEL				0x0000000000040000ULL
#define COULD_NOT_PULSE_PIXEL_NMOVE			0x0000000000080000ULL
#define COULD_NOT_PULSE_CHIP				0x0000000000100000ULL
#define COULD_NOT_SET_RATE_CORRECTION		0x0000000000200000ULL
#define DETECTOR_NETWORK_PARAMETER			0x0000000000400000ULL
#define RATE_CORRECTION_NOT_32or16BIT		0x0000000000800000ULL
#define RATE_CORRECTION_NO_TAU_PROVIDED		0x0000000001000000ULL
#define PROGRAMMING_ERROR					0x0000000002000000ULL
#define RECEIVER_ACTIVATE					0x0000000004000000ULL
#define DATA_STREAMING						0x0000000008000000ULL

//											0x00000000FFFFFFFFULL
/** @short class returning all error messages for error mask */
class errorDefs {


public:

	/** Constructor */
	errorDefs():errorMask(0){
		strcpy(notAddedList,"");
	};

	/** Gets the error message
	 * param errorMask error mask
	 /returns error message from error mask
	*/
	static string getErrorMessage(int64_t slsErrorMask){

		string retval = "";

		if(slsErrorMask&CANNOT_CONNECT_TO_DETECTOR)
			retval.append("Cannot connect to Detector\n");

		if(slsErrorMask&CANNOT_CONNECT_TO_RECEIVER)
			retval.append("Cannot connect to Receiver\n");

		if(slsErrorMask&COULDNOT_SET_CONTROL_PORT)
			retval.append("Could not set control port\n");

		if(slsErrorMask&COULDNOT_SET_STOP_PORT)
			retval.append("Could not set stop port\n");

		if(slsErrorMask&COULDNOT_SET_DATA_PORT)
			retval.append("Could not set receiver port\n");

		if(slsErrorMask&FILE_PATH_DOES_NOT_EXIST)
			retval.append("Path to Output Directory does not exist\n");

		if(slsErrorMask&COULDNOT_CREATE_UDP_SOCKET)
			retval.append("Could not create UDP socket to start receiver\n");

		if(slsErrorMask&COULDNOT_CREATE_FILE)
			retval.append("Could not create file to start receiver.\nCheck permissions of output directory or the overwrite flag\n");

		if(slsErrorMask&COULDNOT_ENABLE_COMPRESSION)
			retval.append("Could not enable/disable data compression in receiver.\nThread creation failed or recompile code with MYROOT1 flag.\n");

		if(slsErrorMask&RECEIVER_DET_HOSTNAME_NOT_SET)
			retval.append("Could not send the detector hostname to the receiver.\n");

		if(slsErrorMask&RECEIVER_DET_HOSTTYPE_NOT_SET)
			retval.append("Could not send the detector type to the receiver.\n");

		if(slsErrorMask&DETECTOR_TEN_GIGA)
			retval.append("Could not enable/disable 10GbE in the detector.\n");

		if(slsErrorMask&DETECTOR_ACTIVATE)
			retval.append("Could not activate/deactivate detector\n");





		if(slsErrorMask&COULD_NOT_CONFIGURE_MAC)
			retval.append("Could not configure mac\n");

		if(slsErrorMask&COULDNOT_SET_NETWORK_PARAMETER)
			retval.append("Could not set network parameter. Should be valid and in proper format\n");

		if(slsErrorMask&COULDNOT_SET_ROI)
			retval.append("Could not set the exact region of interest. Verify ROI set by detector.\n");

		if(slsErrorMask&RECEIVER_READ_FREQUENCY)
			retval.append("Could not set receiver read frequency.\n");

		if(slsErrorMask&SETTINGS_NOT_SET)
			retval.append("Could not set settings.\n");

		if(slsErrorMask&SETTINGS_FILE_NOT_OPEN)
			retval.append("Could not open settings file. Verify if it exists.\n");

		if(slsErrorMask&COULDNOT_START_RECEIVER)
			retval.append("Could not start receiver.\n");

		if(slsErrorMask&COULDNOT_STOP_RECEIVER)
			retval.append("Could not stop receiver.\n");

		if(slsErrorMask&DETECTOR_TIMER_VALUE_NOT_SET)
			retval.append("Could not set one of timer values in detector.\n");

		if(slsErrorMask&RECEIVER_ACQ_PERIOD_NOT_SET)
			retval.append("Could not set acquisition period in receiver.\n");

		if(slsErrorMask&RECEIVER_FRAME_NUM_NOT_SET)
			retval.append("Could not set frame number in receiver.\n");

		if(slsErrorMask&RECEIVER_DYNAMIC_RANGE)
			retval.append("Could not set dynamic range in receiver.\n");

		if(slsErrorMask&RECEIVER_TEN_GIGA)
			retval.append("Could not enable/disable 10GbE in the receiver.\n");

		if(slsErrorMask&ALLTIMBITS_NOT_SET)
			retval.append("Could not set all trimbits to value.\n");

		if(slsErrorMask&COULD_NOT_SET_SPEED_PARAMETERS)
			retval.append("Could not set the speed parameter value\n");

		if(slsErrorMask&COULD_NOT_SET_READOUT_FLAGS)
			retval.append("Could not set the readout flag\n");

		if(slsErrorMask&COULD_NOT_SET_FIFO_DEPTH)
			retval.append("Could not set receiver fifo depth\n");

		if(slsErrorMask&COULD_NOT_SET_COUNTER_BIT)
			retval.append("Could not set/reset counter bit\n");

		if(slsErrorMask&COULD_NOT_PULSE_PIXEL)
			retval.append("Could not pulse pixel\n");

		if(slsErrorMask&COULD_NOT_PULSE_PIXEL_NMOVE)
			retval.append("Could not pulse pixel and move\n");

		if(slsErrorMask&COULD_NOT_PULSE_CHIP)
			retval.append("Could not pulse chip\n");

		if(slsErrorMask&COULD_NOT_SET_RATE_CORRECTION)
			retval.append("Could not set rate correction\n");

		if(slsErrorMask&DETECTOR_NETWORK_PARAMETER)
			retval.append("Could not set/get detector network parameter\n");

		if(slsErrorMask&RATE_CORRECTION_NOT_32or16BIT)
			retval.append("Rate correction Deactivated, must be in 32 or 16 bit mode\n");

		if(slsErrorMask&RATE_CORRECTION_NO_TAU_PROVIDED)
			retval.append("Rate correction Deactivated. No default tau provided in file\n");

		if(slsErrorMask&PROGRAMMING_ERROR)
			retval.append("Could not program FPGA\n");

		if(slsErrorMask&RECEIVER_ACTIVATE)
			retval.append("Could not activate/deactivate receiver\n");

		if(slsErrorMask&DATA_STREAMING)
			retval.append("Could not set/reset Data Streaming\n");



		//------------------------------------------------------ length of message


		return retval;

	}


	  /** Sets multi error mask
	     @param multi error mask to be set to
	     /returns multi error mask
	  */
	  int64_t  setErrorMask(int64_t i){errorMask=i;return getErrorMask();};

	  /**returns multi error mask  */
	   int64_t  getErrorMask(){return errorMask;};

	   /** Clears error mask
	      /returns error mask
	   */
	   int64_t clearErrorMask(){errorMask=0;return errorMask;};

	   /** Gets the not added detector list
	      /returns list
	   */
	   char* getNotAddedList(){return notAddedList;};

	   /** Append the detector to not added detector list
	    * @param name append to the list
	      /returns list
	   */
	   void appendNotAddedList(const char* name){strcat(notAddedList,name);strcat(notAddedList,"+");};

	   /** Clears not added detector list
	      /returns error mask
	   */
	   void clearNotAddedList(){strcpy(notAddedList,"");};


protected:

	  /** Error Mask */
	  int64_t errorMask;

	  /** Detectors Not added List */
	  char notAddedList[MAX_STR_LENGTH];

};

#endif /* ERROR_DEFS_H_ */