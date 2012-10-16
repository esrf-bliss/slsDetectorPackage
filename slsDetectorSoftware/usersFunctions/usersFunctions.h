#ifndef USERS_FUNCTIONS_H
#define USERS_FUNCTIONS_H
/******************************************************************

Functions depending on the experimental setup should be defined here

******************************************************************/

//#define PI 3.14159265358979323846


#ifdef EPICS
#include <cadef.h>
#include <epicsEvent.h>
#endif
#include "detectorData.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef EPICS
  int connect_channel(const char *name,  chid *ch_id);
  int disconnect_channel(chid ch_id);
  int caget(chid ch_id,  double *value);
  int caputq(chid ch_id,  double value);
  void put_callback(struct event_handler_args args);
  int caput(chid ch_id,  double value);
#endif


  // double defaultAngleFunction(double ichan, double encoder, double totalOffset, double conv_r, double center, double offset, double tilt, int direction);
  double defaultGetPosition(void *d);
  int defaultGoToPosition(double p,void *d);
  int defaultGoToPositionNoWait(double p,void *d);
  int defaultConnectChannels(void *d);
  int defaultDisconnectChannels(void *d);
  double defaultGetI0(int t,void *d);
  
  int defaultDataReadyFunc(detectorData* d,  void* p);
  int defaultRawDataReadyFunc(double* d, int np,  void* p);


#ifdef __cplusplus
};
#endif


#endif
