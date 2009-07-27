/* PerkinElmer.h
 *
 * This is a driver the PerkinElmer Image Plates
 *		Models:	XRD0820
 *
 *
 * Author: Brian Tieman
 *
 * Created:  07/24/2008
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
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <cantProceed.h>

#include "asynNDArrayDriver.h"
#include "NDArray.h"
#include "ADDriver.h"

#include "drvPerkinElmer.h"

#include <windows.h>
#include "Acq.h"

//______________________________________________________________________________________________

static const char *driverName = "drvPerkinElmer";

//______________________________________________________________________________________________

/* If we have any private driver parameters they begin with ADFirstDriverParam and should end
   with ADLastDriverParam, which is used for setting the size of the parameter library table */
typedef enum {
	PE_SystemID = ADLastStdParam,
    PE_Initialize,
    PE_StatusRBV,
    PE_AcquireOffset,
    PE_NumOffsetFrames,
    PE_UseOffset,
    PE_OffsetAvailable,
    PE_AcquireGain,
    PE_NumGainFrames,
    PE_UseGain,
    PE_GainAvailable,
    PE_PixelCorrectionAvailable,
    PE_Gain,
    PE_GainRBV,
    PE_DwellTime,
    PE_DwellTimeRBV,
    PE_NumFrameBuffers,
    PE_NumFrameBuffersRBV,
    PE_SyncMode,
    PE_SyncModeRBV,
    PE_Trigger,
    PE_SyncTime,
    PE_SyncTimeRBV,
    PE_UsePixelCorrection,
    PE_LoadCorrectionFiles,
    PE_SaveCorrectionFiles,
    PE_PixelCorrectionFile,
    PE_PixelCorrectionFileRBV,
    PE_CorrectionsDirectory,
    ADLastDriverParam
} PerkinElmerParam_t;

//______________________________________________________________________________________________

static asynParamString_t PerkinElmerParamString[] = {
    {PE_SystemID,     				"PE_SYSTEMID"},
    {PE_Initialize,     			"PE_INITIALIZE"},
    {PE_StatusRBV,			     	"PE_STATUS_RBV"},
	{PE_AcquireOffset,				"PE_ACQUIRE_OFFSET"},
    {PE_NumOffsetFrames,	   		"PE_NUM_OFFSET_FRAMES"},
    {PE_UseOffset,	 		  		"PE_USE_OFFSET"},
    {PE_OffsetAvailable,			"PE_OFFSET_AVAILABLE"},
    {PE_AcquireGain,  		   		"PE_ACQUIRE_GAIN"},
    {PE_NumGainFrames,		   		"PE_NUM_GAIN_FRAMES"},
    {PE_UseGain,	 	  			"PE_USE_GAIN"},
    {PE_GainAvailable,				"PE_GAIN_AVAILABLE"},
    {PE_PixelCorrectionAvailable,	"PE_PIXEL_CORRECTION_AVAILABLE"},
    {PE_Gain,   	  				"PE_GAIN"},
    {PE_GainRBV,	    	 		"PE_GAIN_RBV"},
    {PE_DwellTime,     				"PE_DWELL_TIME"},
    {PE_DwellTimeRBV,   	  		"PE_DWELL_TIME_RBV"},
    {PE_NumFrameBuffers,			"PE_NUM_FRAME_BUFFERS"},
    {PE_NumFrameBuffersRBV,			"PE_NUM_FRAME_BUFFERS_RBV"},
    {PE_SyncMode,     				"PE_SYNC_MODE"},
    {PE_SyncModeRBV,     			"PE_SYNC_MODE_RBV"},
    {PE_Trigger,   	  				"PE_TRIGGER"},
    {PE_SyncTime,     				"PE_SYNC_TIME"},
    {PE_SyncTimeRBV,     			"PE_SYNC_TIME_RBV"},
    {PE_UsePixelCorrection,			"PE_USE_PIXEL_CORRECTION"},
    {PE_LoadCorrectionFiles,		"PE_LOAD_CORRECTION_FILES"},
    {PE_SaveCorrectionFiles,		"PE_SAVE_CORRECTION_FILES"},
    {PE_PixelCorrectionFile,	   	"PE_PIXEL_CORRECTION_FILE"},
    {PE_PixelCorrectionFileRBV,		"PE_PIXEL_CORRECTION_FILE_RBV"},
    {PE_CorrectionsDirectory,		"PE_CORRECTIONS_DIRECTORY"},

};

#define NUM_PERKIN_ELMER_PARAMS (sizeof(PerkinElmerParamString)/sizeof(PerkinElmerParamString[0]))

typedef enum
{
    PE_ACQUIRE_ACQUISITION,
    PE_ACQUIRE_OFFSET,
    PE_ACQUIRE_GAIN
} PEAcquisitionMode_t;

typedef enum
{
    PE_INTERNAL_TRIGGER,
    PE_EXTERNAL_TRIGGER,
    PE_FREE_RUNNING,
    PE_SOFT_TRIGGER
} PETimingMode_t;

#define TIME0		0
#define TIME0_STR	"66.5ms"
#define TIME1		1
#define TIME1_STR	"79.9ms"
#define TIME2		2
#define TIME2_STR	"99.8ms"
#define TIME3		3
#define TIME3_STR	"133.2ms"
#define TIME4		4
#define TIME4_STR	"199.9ms"
#define TIME5		5
#define TIME5_STR	"400.0ms"
#define TIME6		6
#define TIME6_STR	"999.8ms"
#define TIME7		7
#define TIME7_STR	"1999.8ms"


#define GAIN0		0
#define GAIN0_STR	"0.25pF"
#define GAIN1		1
#define GAIN1_STR	"0.5pF"
#define GAIN2		2
#define GAIN2_STR	"1pF"
#define GAIN3		3
#define GAIN3_STR	"2pF"
#define GAIN4		4
#define GAIN4_STR	"4pF"
#define GAIN5		5
#define GAIN5_STR	"8pF"

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

//______________________________________________________________________________________________
/** Structure defining the data acquisition parameters for the Perkin Elmer detector */

typedef struct {
	unsigned short 	*pDataBuffer,
					*pOffsetBuffer;
	DWORD			*pGainBuffer;
	unsigned int	uiRows,
					uiColumns,
					numBufferFrames;
	int				iAcqMode,
					iUseOffset,
					iUseGain,
					iUsePixelCorrections,
					*pPixelCorrectionList;
	PerkinElmer		*pPerkinElmer;
} AcqData_t;

//______________________________________________________________________________________________
/** Driver for the Perkin Elmer flat panel amorphous silicon detectors */

class PerkinElmer : public ADDriver
{
public:
int 				imagesRemaining;
epicsEventId 		startAcquisitionEventId;
epicsEventId 		stopAcquisitionEventId;
NDArray 			*pRaw;

    PerkinElmer(const char *portName, int maxSizeX, int maxSizeY, NDDataType_t dataType, int maxBuffers, size_t maxMemory,
                     int priority, int stackSize);

    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus drvUserCreate(asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize);
    void report(FILE *fp, int details);

    void acquireTask ();

	void frameCallback ();
	void offsetCallback ();
	void gainCallback ();

    ~PerkinElmer ();

private:
HACQDESC			hAcqDesc;
unsigned short		*pAcqBuffer,
					*pOffsetBuffer,
					*pBadPixelMap;
DWORD				*pGainBuffer;
DWORD		dwBoardType,
			dwAcqType,
			dwSystemID,
			dwSyncMode,
			dwHwAccess;
int			abortAcq,
			iChannelNum,
			*pPixelCorrectionList;
BOOL		bEnableIRQ,
			bSelfInit,
			bInitAlways,
			bAcquiringOffset,
			bAcquiringGain;
UINT		uiRows,
			uiColumns,
			uiDataType,
			uiSortFlags,
			uiNumSensors,
			uiChannelType,
			uiPEResult,
			uiNumFrameBuffers;
double      acqTimeReq,
            acqTimeAct;

AcqData_t 		dataAcqStruct;

	template <typename epicsType> void computeArray(int maxSizeX, int maxSizeY);

	int allocateBuffer(void);
	int computeImage(void);

	void enumSensors (void);
	BOOL initializeDetector (void);

	void acquireImage (void);
	void acquireOffsetImage (void);
	void acquireGainImage (void);

	void saveCorrectionFiles (void);
	void loadCorrectionFiles (void);

	void readPixelCorrectionFile (char *pixel_correction_file);

};

//______________________________________________________________________________________________

#endif
