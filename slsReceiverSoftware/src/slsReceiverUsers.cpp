#include "slsReceiverUsers.h"
#include "slsReceiver.h"

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

void slsReceiverUsers::setPassiveMode(bool passive) {
	receiver->setPassiveMode(passive);
}

void slsReceiverUsers::setThreadCPUAffinity(const slsReceiverDefs::CPUMaskList& cpu_masks) {
	receiver->setThreadCPUAffinity(cpu_masks);
}

void slsReceiverUsers::setFifoNodeAffinity(unsigned long fifo_node_mask, int max_node) {
	receiver->setFifoNodeAffinity(fifo_node_mask, max_node);
}

int slsReceiverUsers::getImage(slsReceiverDefs::receiver_image_data& image_data) {
	return receiver->getImage(image_data);
}
