/************************************************
 * @file Fifo.cpp
 * @short constructs the fifo structure
 * which is a circular buffer with pointers to
 * parts of allocated memory
 ***********************************************/

#include "Fifo.h"

#include <iostream>
#include <cstdlib>
#include <cstring>
using namespace std;

int Fifo::NumberofFifoClassObjects(0);

Fifo::Fifo(uint32_t fifoItemSize, uint32_t depth, bool &success):
		index(NumberofFifoClassObjects),
		memory(0),
		fifoBound(0),
		fifoFree(0),
		fifoStream(0),
		fifoDepth(depth),
		status_fifoBound(0),
		status_fifoFree(depth){
	FILE_LOG(logDEBUG) << __AT__ << " called";
	NumberofFifoClassObjects++;
	if(CreateFifos(fifoItemSize) == FAIL)
		success = false;
}


Fifo::~Fifo() {
	FILE_LOG(logDEBUG) << __AT__ << " called";
	//cprintf(BLUE,"Fifo Object %d: Goodbye\n", index);
	DestroyFifos();
	NumberofFifoClassObjects--;
}



int Fifo::CreateFifos(uint32_t fifoItemSize) {
	FILE_LOG(logDEBUG) << __AT__ << " called";

	//destroy if not already
	DestroyFifos();

	//create fifos
	fifoBound = new CircularFifo<char>(fifoDepth);
	fifoFree = new CircularFifo<char>(fifoDepth);
	fifoStream = new CircularFifo<char>(fifoDepth);
	//allocate memory
	size_t mem_len = fifoItemSize * fifoDepth * sizeof(char);
	int ret = posix_memalign((void **) &memory, 64, mem_len);
	if (ret != 0){
		FILE_LOG(logERROR) << "Could not allocate memory for fifos";
		memory = 0;
		return FAIL;
	}
	memset(memory, 0, mem_len);
	FILE_LOG(logDEBUG) << "Memory Allocated " << index << ": " << mem_len << " bytes";

	//push free addresses into fifoFree fifo
	char *buffer = memory;
	for (int i = 0; i < fifoDepth; ++i) {
#ifdef FIFODEBUG
		cprintf(MAGENTA,"Fifofree %d: value:%d, pop 0x%p\n", index, fifoFree->getDataValue(), (void*)(buffer));
#endif
		FreeAddress(buffer);
		buffer += fifoItemSize;
	}
	cprintf(GREEN, "Fifofree %d: %d buffers\n", index, fifoFree->getDataValue());

	FILE_LOG(logDEBUG) << "Fifo Reconstructed Depth " << index << ": " << fifoDepth;
	return OK;
}


void Fifo::DestroyFifos(){
	FILE_LOG(logDEBUG) << __AT__ << " called";


	if(memory) {
		free(memory);
		memory = 0;
	}
	if (fifoBound) {
		delete fifoBound;
		fifoBound = 0;
	}
	if (fifoFree) {
		delete fifoFree;
		fifoFree = 0;
	}
	if (fifoStream) {
		delete fifoStream;
		fifoStream = 0;
	}
}


void Fifo::FreeAddress(char*& address) {
	fifoFree->push(address);
}

void Fifo::GetNewAddress(char*& address) {
	int temp = fifoFree->getDataValue();
	if (temp < status_fifoFree)
		status_fifoFree = temp;
	fifoFree->pop(address);
	/*temp = fifoFree->getDataValue();
	if (temp < status_fifoFree)
		status_fifoFree = temp;*/
}

void Fifo::PushAddress(char*& address) {
	int temp = fifoBound->getDataValue();
	if (temp > status_fifoBound)
		status_fifoBound = temp;
	fifoBound->push(address);
	/*temp = fifoBound->getDataValue();
	if (temp > status_fifoBound)
		status_fifoBound = temp;*/
}

void Fifo::PopAddress(char*& address) {
	fifoBound->pop(address);
}

void Fifo::PushAddressToStream(char*& address) {
	fifoStream->push(address);
}

void Fifo::PopAddressToStream(char*& address) {
	fifoStream->pop(address);
}

int Fifo::GetMaxLevelForFifoBound() {
	int temp = status_fifoBound;
	status_fifoBound = 0;
	return temp;
}


int Fifo::GetMinLevelForFifoFree() {
	int temp = status_fifoFree;
	status_fifoFree = fifoDepth;
	return temp;
}

