#ifndef FIRMWARE_FUNCS_H
#define FIRMWARE_FUNCS_H


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdarg.h>
#include <unistd.h>
#include <asm/page.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdarg.h>
#include <unistd.h>


int mapCSP0(void);
u_int32_t bus_w(u_int32_t offset, u_int32_t data);
u_int32_t bus_r(u_int32_t offset);


u_int32_t putout(char *s, int modnum);
u_int32_t readin(int modnum);
u_int32_t setClockDivider(int d);
u_int32_t getClockDivider();
u_int32_t setSetLength(int d);
u_int32_t getSetLength();
u_int32_t setWaitStates(int d);
u_int32_t getWaitStates();
u_int32_t setTotClockDivider(int d);
u_int32_t getTotClockDivider();
u_int32_t setTotClockDutyCycle(int d);
u_int32_t getTotClockDutyCycle();

u_int32_t setExtSignal(int d, enum externalSignalFlag  mode);
int  getExtSignal(int d);
int setConfigurationRegister(int d);
int setToT(int d);
int setContinousReadOut(int d);

int setDACRegister(int idac, int val, int imod);


u_int64_t getMcsNumber();
u_int32_t getMcsVersion();
u_int32_t testFifos(void);
u_int32_t testFpga(void);
u_int32_t testRAM(void);
int testBus(void);
int64_t set64BitReg(int64_t value, int aLSB, int aMSB);
int64_t get64BitReg(int aLSB, int aMSB);

int64_t setFrames(int64_t value);
int64_t getFrames();

int64_t setExposureTime(int64_t value);
int64_t getExposureTime();

int64_t setGates(int64_t value);
int64_t getGates();

int64_t setDelay(int64_t value);
int64_t getDelaye();

int64_t setPeriod(int64_t value);
int64_t getPeriod();

int64_t setTrains(int64_t value);
int64_t getTrains();

int64_t setProbes(int64_t value);
int64_t getProbes();

u_int32_t runBusy(void); 
u_int32_t runState(void); 
u_int32_t dataPresent(void); 


u_int32_t startStateMachine();
u_int32_t stopStateMachine();
u_int32_t startReadOut();
u_int32_t fifoReset(void);
u_int32_t fifoReadCounter(int fifonum);
u_int32_t fifoReadStatus();


u_int32_t fifo_full(void);



u_int32_t* fifo_read_event();
u_int32_t* decode_data(int* datain);
//u_int32_t move_data(u_int64_t* datain, u_int64_t* dataout);
int setDynamicRange(int dr);
int getDynamicRange();
int getNModBoard();
int setNMod(int n);
int setStoreInRAM(int b);
int allocateRAM();
int clearRAM();

/*

u_int32_t setNBits(u_int32_t);
u_int32_t getNBits();
*/

/*
//move to mcb_funcs?

int readOutChan(int *val);
u_int32_t getModuleNumber(int modnum);
int testShiftIn(int imod);
int testShiftOut(int imod);
int testShiftStSel(int imod);
int testDataInOut(int num, int imod);
int testExtPulse(int imod);
int testExtPulseMux(int imod, int ow);
int testDataInOutMux(int imod, int ow, int num);
int testOutMux(int imod);
int testFpgaMux(int imod);
int calibration_sensor(int num, int *values, int *dacs) ;
int calibration_chip(int num, int *values, int *dacs);
*/


#endif
