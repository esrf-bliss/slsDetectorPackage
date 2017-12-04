/********************************************//**
 * @file slsReceiver.cpp
 * @short creates the UDP and TCP class objects
 ***********************************************/

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include <map>
#include <getopt.h>

#include "slsReceiver.h"
//#include "UDPInterface.h"

using namespace std;



slsReceiver::slsReceiver(int argc, char *argv[], int &success){

	/** 
	 * Constructor method to start up a Receiver server. Reads configuration file, options, and 
	 * assembles a Receiver using TCP and UDP detector interfaces
	 * 
	 * @param iarg 
	 * 
	 * @return 
	 */
	
	udp_interface = NULL;
	tcpipInterface = NULL;

	//creating base receiver
	map<string, string> configuration_map;
	int tcpip_port_no = 1954;
	success=OK;
	string fname = "";
	string udp_interface_type = "standard";
	string rest_hostname = "localhost:8081";
	udp_interface = NULL;

	//parse command line for config
	static struct option long_options[] = {
		/* These options set a flag. */
		//{"verbose", no_argument,       &verbose_flag, 1},
		/* These options don’t set a flag.
		   We distinguish them by their indices. */
		{"type",     required_argument,       0, 't'},
		{"config",     required_argument,       0, 'f'},
		{"rx_tcpport",  required_argument,       0, 'b'},
		{"rest_hostname",  required_argument,       0, 'r'},
		{"help",  no_argument,       0, 'h'},
		{0, 0, 0, 0}
        };
	/* getopt_long stores the option index here. */
	int option_index = 0;
	int c=0;
	optind = 1;

	while ( c != -1 ){
		c = getopt_long (argc, argv, "bfhtr", long_options, &option_index);
		
		/* Detect the end of the options. */
		if (c == -1)
			break;
	
		switch(c){

		case 'f':
			fname = optarg;
			//cout << long_options[option_index].name << " " << optarg << endl;
			break;

		case 'b':
			sscanf(optarg, "%d", &tcpip_port_no);
			break;
			
		case 't':
			udp_interface_type = optarg;
			break;

		case 'r':
			rest_hostname = optarg;
			break;

		case 'h':
			string help_message = """\nSLS Receiver Server\n\n""";
			help_message += """usage: slsReceiver --config config_fname [--rx_tcpport port]\n\n""";
			help_message += """\t--config:\t configuration filename for SLS Detector receiver\n""";
			help_message += """\t--rx_tcpport:\t TCP Communication Port with the client. Default: 1954.\n\n""";
			help_message += """\t--rest_hostname:\t Receiver hostname:port. It applies only to REST receivers, and indicates the hostname of the REST backend. Default: localhost:8081.\n\n""";

			help_message += """\t--type:\t Type of the receiver. Possible arguments are: standard, REST. Default: standard.\n\n""";

			cout << help_message << endl;
			break;
		       
		}
	}

	// if required fname parameter not available, fail
	//if (fname == "")
	//	success = FAIL;

	if( !fname.empty() ){
		try{
			char cstreambuf[MAX_STR_LENGTH]; memset(cstreambuf, 0, MAX_STR_LENGTH);
			sprintf(cstreambuf, "config file name : %s ",fname.c_str());
			FILE_LOG(logDEBUG1, cstreambuf);

			success = read_config_file(fname, &tcpip_port_no, &configuration_map);
			//VERBOSE_PRINT("Read configuration file of " + iline + " lines");
		}
		catch(...){
			char cstreambuf[MAX_STR_LENGTH]; memset(cstreambuf, 0, MAX_STR_LENGTH);
			sprintf(cstreambuf, "Error opening configuration file : %s ",fname.c_str());
			FILE_LOG(logERROR, cstreambuf);

			success = FAIL;
		}
	}


	if(success != OK){
		FILE_LOG(logERROR, "Failed: see output above for more information ");
	}

	if (success==OK){

		char cstreambuf[MAX_STR_LENGTH]; memset(cstreambuf, 0, MAX_STR_LENGTH);
		sprintf(cstreambuf, "SLS Receiver starting %s on port %d ",udp_interface_type.c_str(), tcpip_port_no);
		FILE_LOG(logDEBUG1, cstreambuf);
#ifdef REST
		udp_interface = UDPInterface::create(udp_interface_type);
		udp_interface->configure(configuration_map);
#endif
		tcpipInterface = new slsReceiverTCPIPInterface(success, udp_interface, tcpip_port_no);
	}
}


slsReceiver::~slsReceiver() {
	if(udp_interface) 
		delete udp_interface; 
	if(tcpipInterface) 
		delete tcpipInterface;
}


int slsReceiver::start() {
	return tcpipInterface->start();
}


void slsReceiver::stop() {
	tcpipInterface->stop();
}


void slsReceiver::closeFile(int p) {
	tcpipInterface->closeFile(p);
}


int64_t slsReceiver::getReceiverVersion(){
	return tcpipInterface->getReceiverVersion();
}


void slsReceiver::registerCallBackStartAcquisition(int (*func)(char*, char*, uint64_t, uint32_t, void*),void *arg){
  //tcpipInterface
	if(udp_interface)
		udp_interface->registerCallBackStartAcquisition(func,arg);
	else
		tcpipInterface->registerCallBackStartAcquisition(func,arg);
}



void slsReceiver::registerCallBackAcquisitionFinished(void (*func)(uint64_t, void*),void *arg){
  //tcpipInterface
	if(udp_interface)
		udp_interface->registerCallBackAcquisitionFinished(func,arg);
	else
		tcpipInterface->registerCallBackAcquisitionFinished(func,arg);
}


void slsReceiver::registerCallBackRawDataReady(void (*func)(uint64_t, uint32_t, uint32_t, uint64_t, uint64_t, uint16_t, uint16_t, uint16_t, uint16_t, uint32_t, uint16_t, uint8_t, uint8_t,
		char*, uint32_t, void*),void *arg){
	//tcpipInterface
	if(udp_interface)
		udp_interface->registerCallBackRawDataReady(func,arg);
	else
		tcpipInterface->registerCallBackRawDataReady(func,arg);
}


