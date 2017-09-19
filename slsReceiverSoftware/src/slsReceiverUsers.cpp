#include "slsReceiverUsers.h"
#include "slsReceiver.h"

#include "UDPStandardImplementation.h"

slsReceiverUsers::slsReceiverUsers(int argc, char *argv[], int &success) {
	receiver=new slsReceiver(argc, argv, success);
}

slsReceiverUsers::~slsReceiverUsers() {
  delete receiver;
}

int slsReceiverUsers::start() {
	return receiver->start();
}

void slsReceiverUsers::stop() {
	receiver->stop();
}

int64_t slsReceiverUsers::getReceiverVersion(){
	return receiver->getReceiverVersion();
}

void slsReceiverUsers::registerCallBackStartAcquisition(int (*func)(char*, char*, uint64_t, uint32_t, void*),void *arg){
	receiver->registerCallBackStartAcquisition(func,arg);
}

void slsReceiverUsers::registerCallBackAcquisitionFinished(void (*func)(uint64_t, void*),void *arg){
	receiver->registerCallBackAcquisitionFinished(func,arg);
}
	
void slsReceiverUsers::registerCallBackRawDataReady(void (*func)(uint64_t frameNumber, uint32_t expLength, uint32_t packetNumber, uint64_t bunchId, uint64_t timestamp,
		uint16_t modId, uint16_t xCoord, uint16_t yCoord, uint16_t zCoord, uint32_t debug, uint16_t roundRNumber, uint8_t detType, uint8_t version,
		char* datapointer, uint32_t datasize, void*), void *arg){
	receiver->registerCallBackRawDataReady(func,arg);
}

int slsReceiverUsers::setThreadCPUAffinity(size_t cpusetsize,
					   cpu_set_t *listeners_cpu_mask,
					   cpu_set_t *processors_cpu_mask) {
	UDPInterface *udp_iface = receiver->tcpipInterface->receiverBase;
	if (!udp_iface)
		return 0;
	UDPStandardImplementation *udp_impl;
	udp_impl = static_cast<UDPStandardImplementation *>(udp_iface);
	return udp_impl->setThreadCPUAffinity(cpusetsize, 
					      listeners_cpu_mask,
					      processors_cpu_mask);
}

