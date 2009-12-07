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
	short			iFastCollectMode;
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
//    virtual asynStatus drvUserCreate(asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize);
    void report(FILE *fp, int details);

    void acquireTask ();

	void frameCallback (unsigned int buffFrame);
	void offsetCallback ();
	void gainCallback ();
	int getParamADNumImagesCounter();
	int getParamPE_ImageNumber();
	int getParamPE_FrameBufferIndex();
    ~PerkinElmer ();

protected:
	int PE_SystemID;
	#define PE_FIRST_PARAM PE_SystemID
    int PE_Initialize;
    int PE_StatusRBV;
    int PE_AcquireOffset;
    int PE_NumOffsetFrames;
    int PE_UseOffset;
    int PE_OffsetAvailable;
    int PE_AcquireGain;
    int PE_NumGainFrames;
    int PE_UseGain;
    int PE_GainAvailable;
    int PE_PixelCorrectionAvailable;
    int PE_Gain;
    int PE_GainRBV;
    int PE_DwellTime;
    int PE_DwellTimeRBV;
    int PE_NumFrameBuffers;
    int PE_NumFrameBuffersRBV;
    int PE_SyncMode;
    int PE_SyncModeRBV;
    int PE_Trigger;
    int PE_SyncTime;
    int PE_SyncTimeRBV;
    int PE_UsePixelCorrection;
    int PE_LoadCorrectionFiles;
    int PE_SaveCorrectionFiles;
    int PE_PixelCorrectionFile;
    int PE_PixelCorrectionFileRBV;
    int PE_CorrectionsDirectory;
	int PE_FrameBufferIndex;
	int PE_ImageNumber;
    int PE_FastCollectMode;
    int PE_SkipLeadingPulses;
    int PE_NumPulsesToSkip;
	#define PE_LAST_PARAM PE_NumPulsesToSkip


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
	int			trigModeReq,
				trigModeAct;

	AcqData_t 		dataAcqStruct;

	template <typename epicsType> void computeArray(int maxSizeX, int maxSizeY, unsigned int buffFrame);

	int allocateBuffer(void);
	int computeImage(unsigned int buffFrame);

	void enumSensors (void);
	BOOL initializeDetector (void);

	void acquireImage (void);
	void acquireOffsetImage (void);
	void acquireGainImage (void);

	void saveCorrectionFiles (void);
	void loadCorrectionFiles (void);

	void readPixelCorrectionFile (char *pixel_correction_file);
	asynStatus PerkinElmer::setTriggerMode(void);
	asynStatus PerkinElmer::setExposureTime(void);

};

//______________________________________________________________________________________________
#define PE_SystemIDString     				"PE_SYSTEMID"
#define PE_InitializeString        			"PE_INITIALIZE"
#define PE_StatusRBVString			     	"PE_STATUS_RBV"
#define PE_AcquireOffsetString				"PE_ACQUIRE_OFFSET"
#define PE_NumOffsetFramesString	   		"PE_NUM_OFFSET_FRAMES"
#define PE_UseOffsetString	 		  		"PE_USE_OFFSET"
#define PE_OffsetAvailableString			"PE_OFFSET_AVAILABLE"
#define PE_AcquireGainString  		   		"PE_ACQUIRE_GAIN"
#define PE_NumGainFramesString		   		"PE_NUM_GAIN_FRAMES"
#define PE_UseGainString	 	  			"PE_USE_GAIN"
#define PE_GainAvailableString				"PE_GAIN_AVAILABLE"
#define PE_PixelCorrectionAvailableString	"PE_PIXEL_CORRECTION_AVAILABLE"
#define PE_GainString   	  				"PE_GAIN"
#define PE_GainRBVString	    	 		"PE_GAIN_RBV"
#define PE_DwellTimeString     				"PE_DWELL_TIME"
#define PE_DwellTimeRBVString   	  		"PE_DWELL_TIME_RBV"
#define PE_NumFrameBuffersString			"PE_NUM_FRAME_BUFFERS"
#define PE_NumFrameBuffersRBVString			"PE_NUM_FRAME_BUFFERS_RBV"
#define PE_SyncModeString     				"PE_SYNC_MODE"
#define PE_SyncModeRBVString     			"PE_SYNC_MODE_RBV"
#define PE_TriggerString   	  				"PE_TRIGGER"
#define PE_SyncTimeString     				"PE_SYNC_TIME"
#define PE_SyncTimeRBVString     			"PE_SYNC_TIME_RBV"
#define PE_UsePixelCorrectionString			"PE_USE_PIXEL_CORRECTION"
#define PE_LoadCorrectionFilesString		"PE_LOAD_CORRECTION_FILES"
#define PE_SaveCorrectionFilesString		"PE_SAVE_CORRECTION_FILES"
#define PE_PixelCorrectionFileString	   	"PE_PIXEL_CORRECTION_FILE"
#define PE_PixelCorrectionFileRBVString		"PE_PIXEL_CORRECTION_FILE_RBV"
#define PE_CorrectionsDirectoryString		"PE_CORRECTIONS_DIRECTORY"
#define PE_FrameBufferIndexString			"PE_FRAME_BUFFER_INDEX"
#define PE_ImageNumberString				"PE_IMAGE_NUMBER"
#define PE_FastCollectModeString            "PE_FAST_COLLECT_MODE"
#define PE_SkipLeadingPulsesString          "PE_SKIP_LEADING_PULSES"
#define PE_NumPulsesToSkipString            "PE_NUM_PULSES_TO_SKIP"

#define NUM_PERKIN_ELMER_PARAMS (&PE_LAST_PARAM - &PE_FIRST_PARAM + 1)
//______________________________________________________________________________________________

#endif
