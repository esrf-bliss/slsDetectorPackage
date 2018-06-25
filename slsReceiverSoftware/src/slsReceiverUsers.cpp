#include "slsReceiverUsers.h"
#include "slsReceiver.h"

#include "UDPStandardImplementation.h"

slsReceiverUsers::slsReceiverUsers(int argc, char *argv[], int &success) {
	// catch the exception here to limit it to within the library (for current version)
	try {
		slsReceiver* r = new slsReceiver(argc, argv);
		receiver = r;
		success = slsReceiverDefs::OK;
	} catch (...) {
		success = slsReceiverDefs::FAIL;
	}
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

void slsReceiverUsers::registerCallBackRawDataReady(void (*func)(char* header,
		char* datapointer, uint32_t datasize, void*), void *arg){
	receiver->registerCallBackRawDataReady(func,arg);
}

void slsReceiverUsers::registerCallBackRawDataModifyReady(void (*func)(char* header,
		char* datapointer, uint32_t& revDatasize, void*), void *arg){
	receiver->registerCallBackRawDataModifyReady(func,arg);
}

int slsReceiverUsers::setThreadCPUAffinity(CPUMaskList& listeners_cpu_mask,
					   CPUMaskList& processors_cpu_mask) {
	UDPInterface *udp_iface = receiver->tcpipInterface->receiverBase;
	if (!udp_iface)
		return 0;
	UDPStandardImplementation *udp_impl;
	udp_impl = static_cast<UDPStandardImplementation *>(udp_iface);
	return udp_impl->setThreadCPUAffinity(listeners_cpu_mask,
					      processors_cpu_mask);
}

int slsReceiverUsers::setFifoNodeAffinity(NodeMaskList& fifo_node_mask,
					  int max_node) {
	UDPInterface *udp_iface = receiver->tcpipInterface->receiverBase;
	if (!udp_iface)
		return 0;
	UDPStandardImplementation *udp_impl;
	udp_impl = static_cast<UDPStandardImplementation *>(udp_iface);
	return udp_impl->setFifoNodeAffinity(fifo_node_mask, max_node);
}

