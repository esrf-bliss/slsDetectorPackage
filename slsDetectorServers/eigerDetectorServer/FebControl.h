#pragma once
#include "FebInterface.h"
#include "slsDetectorServer_defs.h"
#include <netinet/in.h>

struct Module {
    unsigned int left_address;
    unsigned int right_address;
    unsigned int idelay[4]; // ll,lr,rl,ll
    float high_voltage;
    int *dac;
};
// module struct
void Module_Module(struct Module *mod);
unsigned int Module_GetBaseAddress(struct Module *mod);
unsigned int Module_GetLeftAddress(struct Module *mod);
unsigned int Module_GetRightAddress(struct Module *mod);
unsigned int Module_SetIDelay(struct Module *mod, unsigned int chip,
                              unsigned int value);
unsigned int Module_GetIDelay(struct Module *mod, unsigned int chip);
float Module_SetHighVoltage(struct Module *mod, float value);
float Module_GetHighVoltage(struct Module *mod);
int Module_SetDACValue(struct Module *mod, unsigned int i, int value);
int Module_GetDACValue(struct Module *mod, unsigned int i);
// setup
void Feb_Control_activate(int activate);
void Feb_Control_FebControl();
int Feb_Control_Init(int master, int normal, int module_num);
int Feb_Control_OpenSerialCommunication();
void Feb_Control_CloseSerialCommunication();
void Feb_Control_AddModule();
int Feb_Control_CheckSetup();
unsigned int Feb_Control_AddressToAll();
int Feb_Control_SetCommandRegister(unsigned int cmd);
int Feb_Control_SetStaticBits();
int Feb_Control_SetStaticBits1(unsigned int the_static_bits);
int Feb_Control_SetInTestModeVariable(int on);
int Feb_Control_GetTestModeVariable();
// idelays
int Feb_Control_SetIDelays(unsigned int ndelay_units);
int Feb_Control_SetIDelays1(unsigned int chip_pos, unsigned int ndelay_units);
int Feb_Control_SendIDelays(unsigned int dst_num, int chip_lr,
                            unsigned int channels, unsigned int ndelay_units);
// high voltage
int Feb_Control_SetHighVoltage(int value);
int Feb_Control_GetHighVoltage(int *value);
int Feb_Control_SendHighVoltage(int dacvalue);
int Feb_Control_ReceiveHighVoltage(unsigned int *value);
// dacs
int Feb_Control_VoltageToDAC(float value, unsigned int *digital,
                             unsigned int nsteps, float vmin, float vmax);
float Feb_Control_DACToVoltage(unsigned int digital, unsigned int nsteps,
                               float vmin, float vmax);
int Feb_Control_DecodeDACString(char *dac_str, unsigned int *dac_ch);
int Feb_Control_SetDAC(char *s, int value, int is_a_voltage_mv);
int Feb_Control_GetDAC(char *s, int *ret_value, int voltage_mv);
int Feb_Control_GetDACName(unsigned int dac_num, char *s);
int Feb_Control_GetDACNumber(char *s, unsigned int *n);
int Feb_Control_SendDACValue(unsigned int dst_num, unsigned int ch,
                             unsigned int *value);

// trimbits
int Feb_Control_SetTrimbits(unsigned int *trimbits, int top);
int Feb_Control_SaveAllTrimbitsTo(int value, int top);
unsigned int *Feb_Control_GetTrimbits();

// acquisition
int Feb_Control_AcquisitionInProgress();
int Feb_Control_AcquisitionStartedBit();
int Feb_Control_WaitForStartedFlag(int sleep_time_us, int prev_flag);
int Feb_Control_WaitForFinishedFlag(int sleep_time_us);
int Feb_Control_GetDAQStatusRegister(unsigned int dst_address,
                                     unsigned int *ret_status);
int Feb_Control_StartDAQOnlyNWaitForFinish(int sleep_time_us);
int Feb_Control_Reset();
int Feb_Control_ResetChipCompletely();
int Feb_Control_ResetChipPartially();
int Feb_Control_SendBitModeToBebServer();
unsigned int Feb_Control_ConvertTimeToRegister(float time_in_sec);
int Feb_Control_PrepareForAcquisition();
void Feb_Control_PrintAcquisitionSetup();
int Feb_Control_StartAcquisition();
int Feb_Control_StopAcquisition();
int Feb_Control_SoftwareTrigger();

// parameters
int Feb_Control_SetDynamicRange(unsigned int four_eight_sixteen_or_thirtytwo);
unsigned int Feb_Control_GetDynamicRange();
int Feb_Control_SetReadoutSpeed(unsigned int readout_speed);
int Feb_Control_SetReadoutMode(unsigned int readout_mode);
int Feb_Control_SetTriggerMode(unsigned int trigger_mode);
int Feb_Control_SetExternalEnableMode(int use_external_enable, int polarity);
int Feb_Control_SetNExposures(unsigned int n_images);
unsigned int Feb_Control_GetNExposures();
int Feb_Control_SetExposureTime(double the_exposure_time_in_sec);
double Feb_Control_GetExposureTime();
int64_t Feb_Control_GetExposureTime_in_nsec();
int Feb_Control_SetSubFrameExposureTime(
    int64_t the_subframe_exposure_time_in_10nsec);
int64_t Feb_Control_GetSubFrameExposureTime();
int Feb_Control_SetSubFramePeriod(int64_t the_subframe_period_in_10nsec);
int64_t Feb_Control_GetSubFramePeriod();
int Feb_Control_SetExposurePeriod(double the_exposure_period_in_sec);
double Feb_Control_GetExposurePeriod();
void Feb_Control_Set_Counter_Bit(int value);
int Feb_Control_Get_Counter_Bit();
int Feb_Control_SetInterruptSubframe(int val);
int Feb_Control_GetInterruptSubframe();
int Feb_Control_SetTop(enum TOPINDEX ind, int left, int right);
void Feb_Control_SetMasterVariable(int val);
int Feb_Control_SetMaster(enum MASTERINDEX ind);
int Feb_Control_SetQuad(int val);
int Feb_Control_SetReadNLines(int value);
int Feb_Control_GetReadNLines();
int Feb_Control_WriteRegister(uint32_t offset, uint32_t data);
int Feb_Control_ReadRegister(uint32_t offset, uint32_t *retval);

// pulsing
int Feb_Control_Pulse_Pixel(int npulses, int x, int y);
int Feb_Control_PulsePixelNMove(int npulses, int inc_x_pos, int inc_y_pos);
int Feb_Control_Shift32InSerialIn(unsigned int value_to_shift_in);
int Feb_Control_SendTokenIn();
int Feb_Control_ClockRowClock(unsigned int ntimes);
int Feb_Control_PulseChip(int npulses);

// rate correction
int64_t Feb_Control_Get_RateTable_Tau_in_nsec();
int64_t Feb_Control_Get_RateTable_Period_in_nsec();
int Feb_Control_SetRateCorrectionTau(int64_t tau_in_Nsec);
int Feb_Control_SetRateCorrectionTable(unsigned int *table);
int Feb_Control_GetRateCorrectionVariable();
void Feb_Control_SetRateCorrectionVariable(int activate_rate_correction);
int Feb_Control_PrintCorrectedValues();

// adcs
int Feb_Control_GetLeftFPGATemp();
int Feb_Control_GetRightFPGATemp();
int64_t Feb_Control_GetMeasuredPeriod();
int64_t Feb_Control_GetSubMeasuredPeriod();
