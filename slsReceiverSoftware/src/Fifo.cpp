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

#include <sys/mman.h>
#include <numaif.h>
#include <errno.h>

Fifo::Fifo(int ind, uint32_t fifoItemSize, uint32_t fifoItemPad, uint32_t depth,
	   unsigned long node_mask, int max_node):
		index(ind),
		memory(0),
		fifoBound(0),
		fifoFree(0),
		fifoStream(0),
		fifoDepth(depth),
		itemPad(fifoItemPad),
		memLen(0),
		nodeMask(node_mask),
		maxNode(max_node),
		status_fifoBound(0),
		status_fifoFree(depth){
	FILE_LOG(logDEBUG) << __AT__ << " called";
	if(CreateFifos(fifoItemSize) == FAIL)
	    throw std::exception();
}


Fifo::~Fifo() {
	FILE_LOG(logDEBUG) << __AT__ << " called";
	//cprintf(BLUE,"Fifo Object %d: Goodbye\n", index);
	DestroyFifos();
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
	memLen = fifoItemSize * fifoDepth * sizeof(char);
	size_t page_size = sysconf(_SC_PAGESIZE);
	size_t misaligned = memLen & (page_size - 1);
	if (misaligned)
		memLen += page_size - misaligned;
	memory = (char *) mmap(0, memLen, PROT_READ | PROT_WRITE,
			       MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (memory == NULL) {
		FILE_LOG(logERROR) << "Could not allocate memory for fifos";
		return FAIL;
	}
	if (nodeMask && maxNode) {
		int ret = mbind(memory, memLen, MPOL_BIND, &nodeMask, maxNode,
				0);
		if (ret != 0)
			cprintf(RED, "mbind failed: %s\n", strerror(errno));
		else
			cprintf(GREEN, "mbind %d: %ld bytes mappted to node 0x%08x\n",
				index, memLen, nodeMask);
	}
	memset(memory, 0, memLen);
	FILE_LOG(logDEBUG) << "Memory Allocated " << index << ": " << memLen << " bytes";

	{ //push free addresses into fifoFree fifo
		char* buffer = memory + itemPad;
		for (int i = 0; i < fifoDepth; ++i) {
			//sprintf(buffer,"memory");
			FreeAddress(buffer);
			buffer += fifoItemSize;
		}
	}
	FILE_LOG(logINFO) << "Fifo " << index << " reconstructed Depth (rx_fifodepth): " << fifoFree->getDataValue();
	return OK;
}


void Fifo::DestroyFifos(){
	FILE_LOG(logDEBUG) << __AT__ << " called";

	if(memory) {
		munmap(memory, memLen);
		memory = 0;
		memLen = 0;
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
}

void Fifo::PushAddress(char*& address) {
	int temp = fifoBound->getDataValue();
	if (temp > status_fifoBound)
		status_fifoBound = temp;
	while(!fifoBound->push(address));
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

