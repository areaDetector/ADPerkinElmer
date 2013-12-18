/* PerkinElmer.h
 *
 * This is a driver the PerkinElmer Image Plates
 *        Models:    XRD0820
 *                   XRF0822
 *
 *
 * Author: Brian Tieman
 *
 * Created:  07/24/2008
 *
 * Current author: Mark Rivers
 *
 */

#ifndef PERKINELMER_H
#define PERKINELMER_H

#include <sys/stat.h>

#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsExit.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <cantProceed.h>
#include <iocsh.h>

#include "ADDriver.h"

#include <windows.h>
#include "Acq.h"

#include <epicsExport.h>

//______________________________________________________________________________________________

static const char *driverName = "PerkinElmer";

//______________________________________________________________________________________________


typedef enum
{
  PE_ACQUIRE_ACQUISITION,
  PE_ACQUIRE_OFFSET,
  PE_ACQUIRE_GAIN
} PEAcquisitionMode_t;

// We add an additional mode to ADImageMode = PEImageAverage

typedef enum
{
  PEImageSingle     = ADImageSingle,
  PEImageMultiple   = ADImageMultiple,
  PEImageContinuous = ADImageContinuous,
  PEImageAverage
} PEImageMode_t;

typedef enum
{
  PE_INTERNAL_TRIGGER,
  PE_EXTERNAL_TRIGGER,
  PE_FREE_RUNNING,
  PE_SOFT_TRIGGER
} PETimingMode_t;

#define TIME0        0
#define TIME0_STR    "66.5ms"
#define TIME1        1
#define TIME1_STR    "79.9ms"
#define TIME2        2
#define TIME2_STR    "99.8ms"
#define TIME3        3
#define TIME3_STR    "133.2ms"
#define TIME4        4
#define TIME4_STR    "199.9ms"
#define TIME5        5
#define TIME5_STR    "400.0ms"
#define TIME6        6
#define TIME6_STR    "999.8ms"
#define TIME7        7
#define TIME7_STR    "1999.8ms"


#define GAIN0        0
#define GAIN0_STR    "0.25pF"
#define GAIN1        1
#define GAIN1_STR    "0.5pF"
#define GAIN2        2
#define GAIN2_STR    "1pF"
#define GAIN3        3
#define GAIN3_STR    "2pF"
#define GAIN4        4
#define GAIN4_STR    "4pF"
#define GAIN5        5
#define GAIN5_STR    "8pF"

typedef enum
{
  PE_STATUS_OK,
  PE_STATUS_INITIALIZING,
  PE_STATUS_RUNNING_OFFSET,
  PE_STATUS_RUNNING_GAIN,
  PE_STATUS_ERROR
} PEStatus_t;

typedef enum
{
  NOT_AVAILABLE,
  AVAILABLE
} Avalability_t;

typedef enum
{
  NO,
  YES
} YesNo_t;

//______________________________________________________________________________________________

class PerkinElmer;

/** Driver for the Perkin Elmer flat panel amorphous silicon detectors */

class PerkinElmer : public ADDriver
{
public:
  PerkinElmer(const char *portName, int IDType, const char *IDValue, 
              int maxBuffers, size_t maxMemory,
              int priority, int stackSize);

  /* These are the methods that we override from ADDriver */
  virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  void report(FILE *fp, int details);

  // These should really be private, but they are called from C so must be public
  void endFrameCallback(HACQDESC hAcqDesc);
  void endAcqCallback(HACQDESC hAcqDesc);
  void acquireStopTask(void);
  ~PerkinElmer();

protected:
  int PE_SystemID;
  #define PE_FIRST_PARAM PE_SystemID
  int PE_Initialize;
  int PE_CorrectionsDirectory;
  int PE_AcquireOffset;
  int PE_NumOffsetFrames;
  int PE_CurrentOffsetFrame;
  int PE_UseOffset;
  int PE_OffsetAvailable;
  int PE_AcquireGain;
  int PE_NumGainFrames;
  int PE_CurrentGainFrame;
  int PE_UseGain;
  int PE_GainAvailable;
  int PE_GainFile;
  int PE_LoadGainFile;
  int PE_SaveGainFile;
  int PE_UsePixelCorrection;
  int PE_PixelCorrectionAvailable;
  int PE_PixelCorrectionFile;
  int PE_LoadPixelCorrectionFile;
  int PE_Gain;
  int PE_DwellTime;
  int PE_NumFrameBuffers;
  int PE_Trigger;
  int PE_SyncTime;
  int PE_FrameBufferIndex;
  int PE_ImageNumber;
  int PE_SkipFrames;
  int PE_NumFramesToSkip;
  #define PE_LAST_PARAM PE_NumFramesToSkip


private:
  int           IDType_;
  char          *IDValue_;
  HACQDESC      hAcqDesc_;
  epicsEventId  acquireStopEvent_;
  epicsUInt16   *pAcqBuffer_;
  epicsUInt16   *pOffsetBuffer_;
  DWORD         *pGainBuffer_;
  epicsUInt16   *pBadPixelMap_;
  int           *pPixelCorrectionList_;
  unsigned int  uiNumFrameBuffers_;
  unsigned int  uiNumBuffersInUse_;
  int           iAcqMode_;
  // Keep a copy of parmeters for Acquistion_EnumSensors 
  unsigned int  uiNumSensors_;
  // Keep a copy of parmeters for Acquisition_GetCommChannel
  unsigned int  uiChannelType_;
  int           iChannelNum_;
  // Keep a copy of parmeters from Acquisition_GetConfiguration
  unsigned int  uiDevFrames_;
  unsigned int  uiRows_;
  unsigned int  uiColumns_;
  unsigned int  uiDataType_;
  unsigned int  uiSortFlags_;
  CHwHeaderInfo cHwHeaderInfo_;
  CHwHeaderInfoEx cHwHeaderInfoEx_;
  BOOL          bEnableIRQ_;
  DWORD         dwAcqType_;
  DWORD         dwSystemID_;
  DWORD         dwSyncMode_;
  DWORD         dwHwAccess_;
  DWORD         dwBoardType_;
  double        dAcqTimeReq_;
  double        dAcqTimeAct_;
  int           iTrigModeReq_;
  int           iTrigModeAct_;

  bool initializeDetector (void);
  void setBinning(void);
  void reportSensors(FILE *fp, int details);

  void acquireStart(void);
  void acquireStop(void);
  void acquireOffsetImage(void);
  void acquireGainImage(void);

  asynStatus loadGainFile(void);
  asynStatus saveGainFile(void);
  asynStatus loadPixelCorrectionFile();

  asynStatus PerkinElmer::setTriggerMode(void);
  asynStatus PerkinElmer::setExposureTime(void);

};

//______________________________________________________________________________________________
#define PE_SystemIDString                   "PE_SYSTEMID"
#define PE_InitializeString                 "PE_INITIALIZE"
#define PE_CorrectionsDirectoryString       "PE_CORRECTIONS_DIRECTORY"
#define PE_AcquireOffsetString              "PE_ACQUIRE_OFFSET"
#define PE_NumOffsetFramesString            "PE_NUM_OFFSET_FRAMES"
#define PE_CurrentOffsetFrameString         "PE_CURRENT_OFFSET_FRAME"
#define PE_UseOffsetString                  "PE_USE_OFFSET"
#define PE_OffsetAvailableString            "PE_OFFSET_AVAILABLE"
#define PE_AcquireGainString                "PE_ACQUIRE_GAIN"
#define PE_NumGainFramesString              "PE_NUM_GAIN_FRAMES"
#define PE_CurrentGainFrameString           "PE_CURRENT_GAIN_FRAME"
#define PE_UseGainString                    "PE_USE_GAIN"
#define PE_GainAvailableString              "PE_GAIN_AVAILABLE"
#define PE_GainFileString                   "PE_GAIN_FILE"
#define PE_LoadGainFileString               "PE_LOAD_GAIN_FILE"
#define PE_SaveGainFileString               "PE_SAVE_GAIN_FILE"
#define PE_UsePixelCorrectionString         "PE_USE_PIXEL_CORRECTION"
#define PE_PixelCorrectionAvailableString   "PE_PIXEL_CORRECTION_AVAILABLE"
#define PE_PixelCorrectionFileString        "PE_PIXEL_CORRECTION_FILE"
#define PE_LoadPixelCorrectionFileString    "PE_LOAD_PIXEL_CORRECTION_FILE"
#define PE_GainString                       "PE_GAIN"
#define PE_DwellTimeString                  "PE_DWELL_TIME"
#define PE_NumFrameBuffersString            "PE_NUM_FRAME_BUFFERS"
#define PE_TriggerString                    "PE_TRIGGER"
#define PE_SyncTimeString                   "PE_SYNC_TIME"
#define PE_FrameBufferIndexString           "PE_FRAME_BUFFER_INDEX"
#define PE_ImageNumberString                "PE_IMAGE_NUMBER"
#define PE_SkipFramesString                 "PE_SKIP_FRAMES"
#define PE_NumFramesToSkipString            "PE_NUM_FRAMES_TO_SKIP"

#define NUM_PERKIN_ELMER_PARAMS ((int)(&PE_LAST_PARAM - &PE_FIRST_PARAM + 1))
//______________________________________________________________________________________________

#endif
