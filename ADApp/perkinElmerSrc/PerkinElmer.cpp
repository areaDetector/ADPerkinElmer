 /* PerkinElmer.cpp
 *
 * This is a driver the PerkinElmer Image Plates
 *		Models:	XRD0820
 *
 *
 * Author: Brian Tieman
 *
 * Created:  07/24/2008
 *
 * Modified by John Hammonds
 * 5/15/2009
 * Fixed problems with frame buffers.  For continuous or large collection switch to use
 * the detectors Continuous collection mode.
 */

#include "PerkinElmer.h"

//_____________________________________________________________________________________________

extern "C" int PerkinElmerConfig(const char *portName, int maxSizeX, int maxSizeY, int dataType, int maxBuffers, size_t maxMemory,
                                 int priority, int stackSize )
{
    new PerkinElmer(portName, maxSizeX, maxSizeY, (NDDataType_t)dataType, maxBuffers, maxMemory, priority, stackSize);
    return(asynSuccess);
}

//_____________________________________________________________________________________________
/** This task is spawned by the constructor.  It in return runs acquireTask which
    monitors and controls acquisition */
static void acquireTaskC(void *drvPvt)
{
    PerkinElmer *pPvt = (PerkinElmer *)drvPvt;

    pPvt->acquireTask();
}

//_____________________________________________________________________________________________

/** callback function that is called by XISL every frame at end of data transfer */
void CALLBACK OnEndFrameCallback(HACQDESC hAcqDesc)
{
DWORD			dwValue;
AcqData_t 		*pUsrArgs;
unsigned int	uiStatus;
unsigned int    SizeX, SizeY;
DWORD ActAcqFrame;
DWORD ActBuffFrame;
unsigned int currBuff;
unsigned int buffOffset;
int imageCounter;
DWORD 				HISError,
					FGError;

	printf ("End Frame callback called...\n");

  	uiStatus = 	Acquisition_GetAcqData(hAcqDesc, (DWORD *) &dwValue);
	if ( uiStatus != 0 ) {
	   printf("Error: %d Acquisition_GetAcqData failed in OnEndFrameCallback!\n", uiStatus);
	   Acquisition_GetErrorCode(hAcqDesc,&HISError,&FGError);
	   printf ("HIS Error: %d, Frame Grabber Error: %d\n", HISError, FGError);
	   }
  	pUsrArgs = ((AcqData_t *) dwValue);

	pUsrArgs->pPerkinElmer->getIntegerParam(0, pUsrArgs->pPerkinElmer->getParamADNumImagesCounter(), &imageCounter);
	imageCounter++;
	pUsrArgs->pPerkinElmer->setIntegerParam(0, pUsrArgs->pPerkinElmer->getParamADNumImagesCounter(), imageCounter);
	pUsrArgs->pPerkinElmer->callParamCallbacks();

	if ((pUsrArgs->iAcqMode == PE_ACQUIRE_ACQUISITION) && !(pUsrArgs->iFastCollectMode) )
	{
        /** find offset into secondary frame buffer */
      	uiStatus =  Acquisition_GetActFrame(hAcqDesc, &ActAcqFrame, &ActBuffFrame);
	    if ( uiStatus != 0 ) {
	       printf("Error: %d Acquisition_GetActFrame failed in OnEndFrameCallback!\n", uiStatus);
	       Acquisition_GetErrorCode(hAcqDesc,&HISError,&FGError);
	       printf ("HIS Error: %d, Frame Grabber Error: %d\n", HISError, FGError);
	       }
		printf( "computeArray: ActAcqFrame = %d, ActBuffFrame = %d\n", ActAcqFrame, ActBuffFrame);
		pUsrArgs->pPerkinElmer->setIntegerParam(0, pUsrArgs->pPerkinElmer->getParamPE_ImageNumber(), ActBuffFrame);
		pUsrArgs->pPerkinElmer->setIntegerParam(0, pUsrArgs->pPerkinElmer->getParamPE_FrameBufferIndex(), ActAcqFrame);

  	    SizeX = pUsrArgs->uiColumns;
	    SizeY = pUsrArgs->uiRows;
		currBuff = (ActBuffFrame - 1)%pUsrArgs->numBufferFrames;
	    buffOffset = SizeX * SizeY * currBuff;

		/** Correct for detector offset and gain as necessary */
		if ((pUsrArgs->iUseOffset) && (pUsrArgs->pOffsetBuffer != NULL))
		{
			if ((pUsrArgs->iUseGain) && (pUsrArgs->pGainBuffer != NULL))
				uiStatus = Acquisition_DoOffsetGainCorrection (&(pUsrArgs->pDataBuffer[buffOffset]), &(pUsrArgs->pDataBuffer[buffOffset]), pUsrArgs->pOffsetBuffer, pUsrArgs->pGainBuffer, pUsrArgs->uiRows * pUsrArgs->uiColumns);
			else
				uiStatus = Acquisition_DoOffsetCorrection (&(pUsrArgs->pDataBuffer[buffOffset]), &(pUsrArgs->pDataBuffer[buffOffset]), pUsrArgs->pOffsetBuffer, pUsrArgs->uiRows * pUsrArgs->uiColumns);
		}

		/** correct for dead pixels as necessary */
		if ((pUsrArgs->iUsePixelCorrections) && (pUsrArgs->pPixelCorrectionList != NULL))
			uiStatus = Acquisition_DoPixelCorrection (&(pUsrArgs->pDataBuffer[buffOffset]), pUsrArgs->pPixelCorrectionList);

		/** Call the routine that actually grabs the data */
		pUsrArgs->pDataBuffer[buffOffset] = ActAcqFrame;
		pUsrArgs->pDataBuffer[buffOffset+1] = ActBuffFrame;
		pUsrArgs->pPerkinElmer->frameCallback (currBuff);
	}

	printf ("End Frame callback done!\n");


}

//_____________________________________________________________________________________________

/** callback function that is called by XISL at end of acquisition */
void CALLBACK OnEndAcqCallback(HACQDESC hAcqDesc)
{
DWORD		dwValue;
AcqData_t 	*pUsrArgs;
unsigned int uiStatus;
unsigned int    SizeX, SizeY;
DWORD ActAcqFrame;
DWORD ActBuffFrame;
unsigned int currBuff;
unsigned int buffOffset;
DWORD 				HISError,
					FGError;

	printf ("End Acquire callback called...\n");

  	uiStatus = 	Acquisition_GetAcqData(hAcqDesc, (DWORD *) &dwValue);
	if ( uiStatus != 0 ) {
	   printf("Error: %d Acquisition_GetAcqData failed in OnEndFrameCallback!\n", uiStatus);
	   Acquisition_GetErrorCode(hAcqDesc,&HISError,&FGError);
	   printf ("HIS Error: %d, Frame Grabber Error: %d\n", HISError, FGError);
	   }
	pUsrArgs = ((AcqData_t *) dwValue);

	/** For normal acquisition mode, send out Arrays all at once once acquisition is over */
	if ((pUsrArgs->iAcqMode == PE_ACQUIRE_ACQUISITION) && (pUsrArgs->iFastCollectMode) )
	{
        /** find offset into secondary frame buffer */
      	uiStatus =  Acquisition_GetActFrame(hAcqDesc, &ActAcqFrame, &ActBuffFrame);
	    if ( uiStatus != 0 ) {
	       printf("Error: %d Acquisition_GetActFrame failed in OnEndFrameCallback!\n", uiStatus);
	       Acquisition_GetErrorCode(hAcqDesc,&HISError,&FGError);
	       printf ("HIS Error: %d, Frame Grabber Error: %d\n", HISError, FGError);
	       }

  	    SizeX = pUsrArgs->uiColumns;
	    SizeY = pUsrArgs->uiRows;
		currBuff = (ActBuffFrame - 1)%pUsrArgs->numBufferFrames;
	    buffOffset = SizeX * SizeY * ((ActBuffFrame - 1)%pUsrArgs->numBufferFrames);



	}

	/* raise a flag to the user if offset data is available */
	if (pUsrArgs->iAcqMode == PE_ACQUIRE_OFFSET)
		pUsrArgs->pPerkinElmer->offsetCallback ();


	/* raise a flag to the user if gain data is available */
	if (pUsrArgs->iAcqMode == PE_ACQUIRE_GAIN)
		pUsrArgs->pPerkinElmer->gainCallback ();


	printf ("End Acquire callback done!\n");
}

//_____________________________________________________________________________________________
//_____________________________________________________________________________________________
//Public methods
//_____________________________________________________________________________________________
//_____________________________________________________________________________________________
/** Constructor for this driver */
PerkinElmer::PerkinElmer(const char *portName, int maxSizeX, int maxSizeY, NDDataType_t dataType, int maxBuffers,
                          size_t maxMemory, int priority, int stackSize)

    : ADDriver(portName, 1, NUM_PERKIN_ELMER_PARAMS, maxBuffers, maxMemory, 0, 0, ASYN_CANBLOCK, 1, priority, stackSize), imagesRemaining(0), pRaw(NULL)
{
    int status = asynSuccess;
    const char *functionName = "PerkinElmer";
    int addr=0;
    int dims[2];

	pAcqBuffer = NULL;
	pOffsetBuffer = NULL;
	pGainBuffer = NULL;
	pBadPixelMap = NULL;
	pPixelCorrectionList = NULL;

	bAcquiringOffset = false;
	bAcquiringGain = false;
	abortAcq = 0;
    /* Create the epicsEvents for signaling to the simulate task when acquisition starts and stops */
    this->startAcquisitionEventId = epicsEventCreate(epicsEventEmpty);
    if (!this->startAcquisitionEventId) {
        printf("%s:%s epicsEventCreate failure for start acquisition event\n", driverName, functionName);
        return;
    }
    this->stopAcquisitionEventId = epicsEventCreate(epicsEventEmpty);
    if (!this->stopAcquisitionEventId) {
        printf("%s:%s epicsEventCreate failure for stop acquisition event\n", driverName, functionName);
        return;
    }

    /* Allocate the raw buffer we use to compute images.  Only do this once */
    dims[0] = maxSizeX;
    dims[1] = maxSizeY;
    this->pRaw = this->pNDArrayPool->alloc(2, dims, dataType, 0, NULL);

  /* Add parameters for this driver */
  createParam(PE_SystemIDString,                    asynParamInt32,   &PE_SystemID);
  createParam(PE_InitializeString,                  asynParamInt32,   &PE_Initialize);
  createParam(PE_StatusRBVString,                   asynParamInt32,   &PE_StatusRBV);
  createParam(PE_AcquireOffsetString,               asynParamInt32,   &PE_AcquireOffset);
  createParam(PE_NumOffsetFramesString,             asynParamInt32,   &PE_NumOffsetFrames);
  createParam(PE_UseOffsetString,                   asynParamInt32,   &PE_UseOffset);
  createParam(PE_OffsetAvailableString,             asynParamInt32,   &PE_OffsetAvailable);
  createParam(PE_AcquireGainString,                 asynParamInt32,   &PE_AcquireGain);
  createParam(PE_NumGainFramesString,               asynParamInt32,   &PE_NumGainFrames);
  createParam(PE_UseGainString,                     asynParamInt32,   &PE_UseGain);
  createParam(PE_GainAvailableString,               asynParamInt32,   &PE_GainAvailable);
  createParam(PE_PixelCorrectionAvailableString,    asynParamInt32,   &PE_PixelCorrectionAvailable);
  createParam(PE_GainString,                        asynParamInt32,   &PE_Gain);
  createParam(PE_GainRBVString,                     asynParamInt32,   &PE_GainRBV);
  createParam(PE_DwellTimeString,                   asynParamInt32,   &PE_DwellTime);
  createParam(PE_DwellTimeRBVString,                asynParamInt32,   &PE_DwellTimeRBV);
  createParam(PE_NumFrameBuffersString,             asynParamInt32,   &PE_NumFrameBuffers);
  createParam(PE_NumFrameBuffersRBVString,          asynParamInt32,   &PE_NumFrameBuffersRBV);
  createParam(PE_SyncModeString,                    asynParamInt32,   &PE_SyncMode);
  createParam(PE_SyncModeRBVString,                 asynParamInt32,   &PE_SyncModeRBV);
  createParam(PE_TriggerString,                     asynParamInt32,   &PE_Trigger);
  createParam(PE_SyncTimeString,                    asynParamInt32,   &PE_SyncTime);
  createParam(PE_SyncTimeRBVString,                 asynParamInt32,   &PE_SyncTimeRBV);
  createParam(PE_UsePixelCorrectionString,          asynParamInt32,   &PE_UsePixelCorrection);
  createParam(PE_LoadCorrectionFilesString,         asynParamInt32,   &PE_LoadCorrectionFiles);
  createParam(PE_SaveCorrectionFilesString,         asynParamInt32,   &PE_SaveCorrectionFiles);
  createParam(PE_PixelCorrectionFileString,         asynParamOctet,   &PE_PixelCorrectionFile);
  createParam(PE_PixelCorrectionFileRBVString,      asynParamOctet,   &PE_PixelCorrectionFileRBV);
  createParam(PE_CorrectionsDirectoryString,        asynParamOctet,   &PE_CorrectionsDirectory);
  createParam(PE_FrameBufferIndexString,            asynParamInt32,   &PE_FrameBufferIndex);
  createParam(PE_ImageNumberString,                 asynParamInt32,   &PE_ImageNumber);
  createParam(PE_FastCollectModeString,             asynParamInt32,   &PE_FastCollectMode);
  createParam(PE_SkipLeadingPulsesString,           asynParamInt32,   &PE_SkipLeadingPulses);
  createParam(PE_NumPulsesToSkipString,             asynParamInt32,   &PE_NumPulsesToSkip);


  /* Set some default values for parameters */
    status =  setStringParam (ADManufacturer, "Perkin Elmer");
    status |= setStringParam (ADModel, "XRD0820");
    status |= setIntegerParam(ADMaxSizeX, maxSizeX);
    status |= setIntegerParam(ADMaxSizeY, maxSizeY);
    status |= setIntegerParam(ADSizeX, maxSizeX);
    status |= setIntegerParam(ADSizeX, maxSizeX);
    status |= setIntegerParam(ADSizeY, maxSizeY);
    status |= setIntegerParam(NDArraySizeX, maxSizeX);
    status |= setIntegerParam(NDArraySizeY, maxSizeY);
    status |= setIntegerParam(NDArraySize, 0);
    status |= setIntegerParam(NDDataType, dataType);
    status |= setIntegerParam(ADImageMode, ADImageContinuous);
    status |= setDoubleParam (ADAcquireTime, 0.0665 );
    status |= setDoubleParam (ADAcquirePeriod, .005);
    status |= setIntegerParam(ADNumImages, 100);
    status |= setIntegerParam(ADTriggerMode, 0);

    //Detector parameter defaults
    status |= setIntegerParam(PE_SystemID, 0);
    status |= setIntegerParam(PE_Initialize, 0);
    status |= setIntegerParam(PE_StatusRBV, PE_STATUS_OK);
    status |= setIntegerParam(PE_AcquireOffset, 0);
    status |= setIntegerParam(PE_NumOffsetFrames, 10);
    status |= setIntegerParam(PE_UseOffset, NO);
    status |= setIntegerParam(PE_OffsetAvailable, NOT_AVAILABLE);
    status |= setIntegerParam(PE_AcquireGain, 0);
    status |= setIntegerParam(PE_NumGainFrames, 10);
    status |= setIntegerParam(PE_UseGain, NO);
    status |= setIntegerParam(PE_GainAvailable, NOT_AVAILABLE);
    status |= setIntegerParam(PE_Gain, 0);
    status |= setIntegerParam(PE_GainRBV, 0);
    status |= setIntegerParam(PE_DwellTime, 0);
    status |= setIntegerParam(PE_DwellTimeRBV, 0);
    status |= setIntegerParam(PE_NumFrameBuffers, 10);
    status |= setIntegerParam(PE_NumFrameBuffersRBV, 10);
    status |= setIntegerParam(PE_SyncMode, PE_INTERNAL_TRIGGER);
    status |= setIntegerParam(PE_SyncModeRBV, PE_INTERNAL_TRIGGER);
    status |= setIntegerParam(PE_Trigger, 0);
    status |= setIntegerParam(PE_LoadCorrectionFiles, 0);
    status |= setIntegerParam(PE_SaveCorrectionFiles, 0);
    status |= setIntegerParam(PE_UsePixelCorrection, 0);
    status |= setStringParam (PE_PixelCorrectionFile, "");
    status |= setStringParam (PE_PixelCorrectionFileRBV, "none");
    status |= setIntegerParam(PE_PixelCorrectionAvailable, NOT_AVAILABLE);
    status |= setStringParam (PE_CorrectionsDirectory, "none");
	status |= setIntegerParam (PE_FastCollectMode, 0);
	status |= setIntegerParam (PE_FrameBufferIndex, 0);
	status |= setIntegerParam (PE_ImageNumber, 0);
	status |= setIntegerParam (PE_SkipLeadingPulses, 0);
	status |= setIntegerParam (PE_NumPulsesToSkip, 0);
    if (status) {
        printf("%s: unable to set camera parameters\n", functionName);
        return;
    }
	/* initialize internal variables uses to hold time delayed information.*/
    status |= getDoubleParam(ADAcquireTime, &(this->acqTimeReq) );
    this->acqTimeAct = this->acqTimeReq;
    status |= getIntegerParam(ADTriggerMode, &(this->trigModeReq) );
    this->trigModeAct = this->trigModeReq;
    initializeDetector ();

    /* Create the thread that updates the images */
    status = (epicsThreadCreate("AcquireTask",
                                epicsThreadPriorityMedium,
                                epicsThreadGetStackSize(epicsThreadStackMedium),
                                (EPICSTHREADFUNC)acquireTaskC,
                                this) == NULL);
    if (status) {
        printf("%s:%s epicsThreadCreate failure for acquire task\n", driverName, functionName);
        return;
    }

}

//_____________________________________________________________________________________________

asynStatus PerkinElmer::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    const char *functionName = "writeInt32";
    int function = pasynUser->reason;
    int adstatus;
    int addr=0;
    int status = asynSuccess;
    int	iDetectorStatus;
	int retstat;

	//If Status is Initializing--ignore user input!
    status = getIntegerParam(addr, PE_StatusRBV, &iDetectorStatus);
         getIntegerParam(addr, ADStatus, &adstatus);
   if (iDetectorStatus != PE_STATUS_OK)
    	return ((asynStatus) status);

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setIntegerParam(addr, function, value);

    /* For a real detector this is where the parameter is sent to the hardware */
    if (function == ADAcquire) {
        getIntegerParam(addr, ADStatus, &adstatus);

        //Start acquisition
        if (value && (adstatus == ADStatusIdle) )
        {
            asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s: Acquire!\n", driverName, functionName);

            /* We need to set the number of images we expect to collect, so the image callback function
               can know when acquisition is complete.  We need to find out what mode we are in and how
               many images have been requested.  If we are in continuous mode then set the number of
               remaining images to -1. */
            int imageMode, numImages;
            status = getIntegerParam(addr, ADImageMode, &imageMode);
            status = getIntegerParam(addr, ADNumImages, &numImages);
            switch(imageMode) {
            case ADImageSingle:
                this->imagesRemaining = 1;
                break;
            case ADImageMultiple:
                this->imagesRemaining = numImages;
                break;
            case ADImageContinuous:
                this->imagesRemaining = -1;
                break;
            }
            setIntegerParam(addr, ADStatus, ADStatusAcquire);

            epicsEventSignal(this->startAcquisitionEventId);
        }

        //Abort acquisition
        if (!value && (adstatus != ADStatusIdle))
        {
            this->imagesRemaining = 0;
			setIntegerParam(addr, ADAcquire, 0);
           	setIntegerParam(addr, ADStatus, ADStatusIdle);
		    callParamCallbacks(addr, addr);
 			this->abortAcq = 1;
          asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s: Idle!\n", driverName, functionName);
        }
	}
	else if ( (function == ADBinX) ||
		(function == ADBinY) ||
		(function == ADMinX) ||
		(function == ADMinY) ||
		(function == ADSizeX) ||
		(function == ADSizeY) ||
		(function == NDDataType) ) {
	}
	else if (function == ADImageMode) {
        /* The image mode may have changed while we are acquiring,
         * set the images remaining appropriately. */
        switch (value) {
        case ADImageSingle:
            this->imagesRemaining = 1;
            break;
        case ADImageMultiple: {
            int numImages;
            getIntegerParam(addr, ADNumImages, &numImages);
            this->imagesRemaining = numImages;
            break;}
        case ADImageContinuous:
            this->imagesRemaining = -1;
            break;
        }
	}
    else if (function == PE_Initialize) {
		if ( adstatus == ADStatusIdle ) {
			initializeDetector ();
		}
	}
    else if (function == PE_AcquireOffset) {
		if ( adstatus == ADStatusIdle ) {
			acquireOffsetImage ();
		}
	}
    else if (function ==  PE_AcquireGain) {
		if ( adstatus == ADStatusIdle ) {
			acquireGainImage ();
		}
	}
    else if (function == PE_Trigger) {
		if ((uiPEResult = Acquisition_SetFrameSync(hAcqDesc))!=HIS_ALL_OK)
			printf("Error: %d  Acquisition_SetFrameSync failed!\n", uiPEResult);
	}

	else if (function == ADTriggerMode) {
        getIntegerParam(addr, ADStatus, &adstatus);
		retstat |= getIntegerParam(addr, ADTriggerMode, &(this->trigModeReq) );
		printf ("Setting Requested Trigger Mode: %d\n", this->trigModeReq);
		retstat |= setIntegerParam(addr, ADTriggerMode, this->trigModeAct );
		//if not running go ahead and set the trigger mode
        if ( adstatus == ADStatusIdle ) {
			this->setTriggerMode();
		}
	}
	else if (function == PE_SaveCorrectionFiles) {
		saveCorrectionFiles ();
	}
	else if (function == PE_LoadCorrectionFiles) {
		loadCorrectionFiles ();
	}


    else {
        /* If this parameter belongs to a base class call its method */
        if (function < PE_FIRST_PARAM) {
			status = ADDriver::writeInt32(pasynUser, value);
		}
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(addr, addr);

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:writeInt32 error, status=%d function=%d, value=%d\n",
              driverName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:writeInt32: function=%d, value=%d\n",
              driverName, function, value);

    return ((asynStatus) status);
}


//_____________________________________________________________________________________________

asynStatus PerkinElmer::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int retstat;
    int addr=0;
    int adstatus;

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(addr, function, value);

    /* Changing any of the following parameters requires recomputing the base image */
    if (function == ADAcquireTime) {
        getIntegerParam(addr, ADStatus, &adstatus);

		retstat |= getDoubleParam(addr, ADAcquireTime, &(this->acqTimeReq) );
		printf ("Setting Requested Acquisition Time: %f\n", this->acqTimeReq);
		retstat |= setDoubleParam(addr, ADAcquireTime, this->acqTimeAct );
		// if the detector is idle then go ahead and set the value
        if ( adstatus == ADStatusIdle ) {
			this->setExposureTime();
		}

	}
    else if (function == ADGain) {
	}
    else {
        /* If this parameter belongs to a base class call its method */
        if (function < PE_FIRST_PARAM) {
			status = ADDriver::writeFloat64(pasynUser, value);
		}
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(addr, addr);

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:writeFloat64 error, status=%d function=%d, value=%f\n",
              driverName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:writeFloat64: function=%d, value=%f\n",
              driverName, function, value);
    return status;
}



//_____________________________________________________________________________________________

/* asynDrvUser routines */
/*asynStatus PerkinElmer::drvUserCreate(asynUser *pasynUser,
/*                                      const char *drvInfo,
/*                                      const char **pptypeName, size_t *psize)
/*{
/*    asynStatus status;
/*    int param;
/*    const char *functionName = "drvUserCreate";
/*
/*    /* See if this is one of our standard parameters */
/*/*    status = findParam(PerkinElmerParamString, NUM_PERKIN_ELMER_PARAMS,
/*                       drvInfo, &param);
/*
/*    if (status == asynSuccess) {
/*        pasynUser->reason = param;
/*        if (pptypeName) {
/*            *pptypeName = epicsStrDup(drvInfo);
/*        }
/*        if (psize) {
/*            *psize = sizeof(param);
/*        }
/*        asynPrint(pasynUser, ASYN_TRACE_FLOW,
/*                  "%s:%s: drvInfo=%s, param=%d\n",
/*                  driverName, functionName, drvInfo, param);
/*        return(asynSuccess);
/*    }
/*
/*    /* If not, then see if it is a base class parameter */
/*    status = ADDriver::drvUserCreate(pasynUser, drvInfo, pptypeName, psize);
/*    return(status);
/*}
*/
//_____________________________________________________________________________________________

void PerkinElmer::report(FILE *fp, int details)
{
    int addr=0;

    fprintf(fp, "Perkin Elmer %s\n", this->portName);
    if (details > 0) {
        int nx, ny, dataType;
        getIntegerParam(addr, ADSizeX, &nx);
        getIntegerParam(addr, ADSizeY, &ny);
        getIntegerParam(addr, NDDataType, &dataType);
        fprintf(fp, "  NX, NY:            %d  %d\n", nx, ny);
        fprintf(fp, "  Data type:         %d\n", dataType);
    }
    /* Invoke the base class method */
    ADDriver::report(fp, details);
}

//_____________________________________________________________________________________________

/** This task is spawned off in a new thread to watch over acquisition */
void PerkinElmer::acquireTask()
{
int status = asynSuccess;
int adstatus;
int eventStatus;
int addr=0;
double pollTime = 0.01;
	while (true)
	{
        status = epicsEventWait(this->startAcquisitionEventId);

        //If in continuous mode, the new image is requested from the callback of the previous image
        //so the new image will fail because the hardware hasn't "finished" the old image
		eventStatus = epicsEventWaitWithTimeout(this->stopAcquisitionEventId, pollTime);
        getIntegerParam(addr, ADStatus, &adstatus);
       if (adstatus == ADStatusAcquire) {
			printf ("acquireTask: Starting Acquisition\n");
			Acquisition_SetReady(hAcqDesc, 1);
        	acquireImage ();
	    }
//		i=0;
		while ( (adstatus == ADStatusAcquire) && (this->abortAcq != 1) ) {
			eventStatus = epicsEventWaitWithTimeout(this->stopAcquisitionEventId, pollTime);

 	       	getIntegerParam(addr, ADStatus, &adstatus);
//			if ( (i%10) == 0 ) {
//				printf ("Acquiring: adstatus = %d, this->abortAcq = %d\n", adstatus, this->abortAcq );
//			}
//			i++;
		}

		if ( this->abortAcq == 1 ) {
			printf ("acquireTask: Aborting Acquisition\n");
			this->abortAcq = 0;

 	       	getIntegerParam(addr, ADStatus, &adstatus);
 	        while (adstatus){
 	           getIntegerParam(addr, ADStatus, &adstatus);
			}
	 	   Acquisition_Abort(this->hAcqDesc);
		}


	}
}

//_____________________________________________________________________________________________

/** called from OnEndFrameCallback to process data from the detector into the pArray */
void PerkinElmer::frameCallback(unsigned int buffFrame)
{
/* This thread computes new image data and does the callbacks to send it to higher layers */
int status = asynSuccess;
int dataType;
int addr=0;
int imageSizeX, imageSizeY, imageSize;
int imageCounter;
int autoSave;
NDArray *pImage;
epicsTimeStamp startTime, endTime;
double elapsedTime;
const char *functionName = "frameCallback";

    this->lock();

    /* Update the image */
    status = computeImage(buffFrame);

    pImage = this->pArrays[addr];

    epicsTimeGetCurrent(&endTime);
    elapsedTime = epicsTimeDiffInSeconds(&endTime, &startTime);

    /* Get the current parameters */
    getIntegerParam(addr, NDArraySizeX, &imageSizeX);
    getIntegerParam(addr, NDArraySizeY, &imageSizeY);
    getIntegerParam(addr, NDArraySize,  &imageSize);
    getIntegerParam(addr, NDDataType,   &dataType);
    getIntegerParam(addr, NDAutoSave,   &autoSave);
    getIntegerParam(addr, NDArrayCounter, &imageCounter);
    imageCounter++;
    setIntegerParam(addr, NDArrayCounter, imageCounter);

    /* Put the frame number and time stamp into the buffer */
    pImage->uniqueId = imageCounter;
    pImage->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;

    /* Get any attributes that have been defined for this driver */
    this->getAttributes(pImage->pAttributeList);

    /* Call the NDArray callback */
    /* Must release the lock here, or we can get into a deadlock, because we can
     * block on the plugin lock, and the plugin can be calling us */
    this->unlock();
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
         "%s:%s: calling imageData callback\n", driverName, functionName);
    doCallbacksGenericPointer(pImage, NDArrayData, addr);
	this->lock();
	/* See if acquisition is done */
    if (this->imagesRemaining > 0)
    {
    	this->imagesRemaining--;
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: %d images remaining.\n", driverName, functionName, imagesRemaining);
	}
    if (this->imagesRemaining == 0)
    {
		printf("Aborting!!!\n");
		status |= setIntegerParam(addr, ADAcquire, 0);
		status |= setIntegerParam(addr, ADStatus, ADStatusIdle);
	    callParamCallbacks(addr, addr);
		this->abortAcq = 1;

	}
    /* Call the callbacks to update any changes */
    callParamCallbacks(addr, addr);

    this->unlock();
	if (abortAcq) {
		epicsEventSignal(this->stopAcquisitionEventId);
	}


}

//_____________________________________________________________________________________________

/** Called from OnEndAcqCallback after offset data has been collected to flag to the user
    that this data is available */
void PerkinElmer::offsetCallback()
{
int 	addr=0,
		status;

	printf ("Offset collection complete!\n");

	if (pOffsetBuffer != NULL)
	{
	    status |= setIntegerParam(addr, PE_OffsetAvailable, AVAILABLE);
		setIntegerParam(addr, PE_StatusRBV, PE_STATUS_OK);
	    callParamCallbacks(addr, addr);
	}


	bAcquiringOffset = false;
}

//_____________________________________________________________________________________________

/** Called from OnEndAcqCallback after gain data has been collected to flag to the user
    that this data is available */
void PerkinElmer::gainCallback()
{
int 	addr=0,
		status;

	printf ("Gain collection complete!\n");

	if (pGainBuffer != NULL)
	{
	    status |= setIntegerParam(addr, PE_GainAvailable, AVAILABLE);
	    setIntegerParam(addr, PE_StatusRBV, PE_STATUS_OK);
	    callParamCallbacks(addr, addr);
	}

	bAcquiringGain = false;
}

//_____________________________________________________________________________________________
	int PerkinElmer::getParamADNumImagesCounter() {
			return this->ADNumImagesCounter;
	}
//_____________________________________________________________________________________________
	int PerkinElmer::getParamPE_ImageNumber() {
		return this->PE_ImageNumber;
	}

//_____________________________________________________________________________________________
	int PerkinElmer::getParamPE_FrameBufferIndex(){
		return this->PE_FrameBufferIndex;
	}

//_____________________________________________________________________________________________

PerkinElmer::~PerkinElmer()
{
	Acquisition_Close (hAcqDesc);

	if (pAcqBuffer != NULL)
		free (pAcqBuffer);

	if (pOffsetBuffer != NULL)
		free (pOffsetBuffer);

	if (pGainBuffer != NULL)
		free (pGainBuffer);

	if (pBadPixelMap != NULL)
		free (pBadPixelMap);

	if (pPixelCorrectionList != NULL)
		free (pPixelCorrectionList);
}

//_____________________________________________________________________________________________
//_____________________________________________________________________________________________
//Private methods
//_____________________________________________________________________________________________
//_____________________________________________________________________________________________
/** Move data from secondary frame buffer to the driver's data space */
template <typename epicsType> void PerkinElmer::computeArray(int maxSizeX, int maxSizeY, unsigned int buffFrame )
{
//DWORD ActAcqFrame;
//DWORD ActBuffFrame;
int bufferOffset;
int yOffset;
//int frameBufferSize;
int addr = 0;

	/* Find which slot in the secondary buffer is active. */
//	Acquisition_GetActFrame(hAcqDesc, &ActAcqFrame, &ActBuffFrame);
//	getIntegerParam(addr, PE_NumFrameBuffersRBV, &frameBufferSize);
	bufferOffset = (maxSizeX*maxSizeY)*buffFrame;
//	printf( "computeArray: ActAcqFrame = %d, ActBuffFrame = %d\n", ActAcqFrame, ActBuffFrame);
//	printf( "computeArray: buffFrame %d\n", buffFrame);
	epicsType *pData = (epicsType *)this->pRaw->pData;


	for (int loopy=0; loopy<maxSizeY; loopy++) {
		yOffset = (loopy*maxSizeX);
		for (int loopx=0; loopx<maxSizeX; loopx++){
			(*pData++) = (epicsType)pAcqBuffer[loopx + yOffset + bufferOffset];
		}
	}
}

//_____________________________________________________________________________________________

int PerkinElmer::allocateBuffer()
{
    int status = asynSuccess;
    NDArrayInfo_t arrayInfo;

    /* Make sure the raw array we have allocated is large enough.
     * We are allowed to change its size because we have exclusive use of it */
    this->pRaw->getInfo(&arrayInfo);
    if (arrayInfo.totalBytes > this->pRaw->dataSize) {
        free(this->pRaw->pData);
        this->pRaw->pData  = malloc(arrayInfo.totalBytes);
        this->pRaw->dataSize = arrayInfo.totalBytes;
        if (!this->pRaw->pData) status = asynError;
    }
    return(status);
}

//_____________________________________________________________________________________________

int PerkinElmer::computeImage(unsigned int buffFrame)
{
    int status = asynSuccess;
    NDDataType_t dataType;
    int addr=0;
    int binX, binY, minX, minY, sizeX, sizeY, reverseX, reverseY;
    int maxSizeX, maxSizeY;
    NDDimension_t dimsOut[2];
    NDArrayInfo_t arrayInfo;
    NDArray *pImage;
    const char* functionName = "computeImage";

    /* NOTE: The caller of this function must have taken the mutex */

    status |= getIntegerParam(addr, ADBinX,         &binX);
    status |= getIntegerParam(addr, ADBinY,         &binY);
    status |= getIntegerParam(addr, ADMinX,         &minX);
    status |= getIntegerParam(addr, ADMinY,         &minY);
    status |= getIntegerParam(addr, ADSizeX,        &sizeX);
    status |= getIntegerParam(addr, ADSizeY,        &sizeY);
    status |= getIntegerParam(addr, ADReverseX,     &reverseX);
    status |= getIntegerParam(addr, ADReverseY,     &reverseY);
    status |= getIntegerParam(addr, ADMaxSizeX,     &maxSizeX);
    status |= getIntegerParam(addr, ADMaxSizeY,     &maxSizeY);
    status |= getIntegerParam(addr, NDDataType,     (int *)&dataType);
    if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error getting parameters\n",
                    driverName, functionName);

    /* Make sure parameters are consistent, fix them if they are not */
    if (binX < 1) {
        binX = 1;
        status |= setIntegerParam(addr, ADBinX, binX);
    }
    if (binY < 1) {
        binY = 1;
        status |= setIntegerParam(addr, ADBinY, binY);
    }
    if (minX < 0) {
        minX = 0;
        status |= setIntegerParam(addr, ADMinX, minX);
    }
    if (minY < 0) {
        minY = 0;
        status |= setIntegerParam(addr, ADMinY, minY);
    }
    if (minX > maxSizeX-1) {
        minX = maxSizeX-1;
        status |= setIntegerParam(addr, ADMinX, minX);
    }
    if (minY > maxSizeY-1) {
        minY = maxSizeY-1;
        status |= setIntegerParam(addr, ADMinY, minY);
    }
    if (minX+sizeX > maxSizeX) {
        sizeX = maxSizeX-minX;
        status |= setIntegerParam(addr, ADSizeX, sizeX);
    }
    if (minY+sizeY > maxSizeY) {
        sizeY = maxSizeY-minY;
        status |= setIntegerParam(addr, ADSizeY, sizeY);
    }

    /* Make sure the buffer we have allocated is large enough. */
    this->pRaw->dataType = dataType;
    status = allocateBuffer();
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: error allocating raw buffer\n",
                  driverName, functionName);
        return(status);
    }
    switch (dataType) {
        case NDInt8:
            computeArray<epicsInt8>(maxSizeX, maxSizeY, buffFrame);
            break;
        case NDUInt8:
            computeArray<epicsUInt8>(maxSizeX, maxSizeY, buffFrame);
            break;
        case NDInt16:
            computeArray<epicsInt16>(maxSizeX, maxSizeY, buffFrame);
            break;
        case NDUInt16:
            computeArray<epicsUInt16>(maxSizeX, maxSizeY, buffFrame);
            break;
        case NDInt32:
            computeArray<epicsInt32>(maxSizeX, maxSizeY, buffFrame);
            break;
        case NDUInt32:
            computeArray<epicsUInt32>(maxSizeX, maxSizeY, buffFrame);
            break;
        case NDFloat32:
            computeArray<epicsFloat32>(maxSizeX, maxSizeY, buffFrame);
            break;
        case NDFloat64:
            computeArray<epicsFloat64>(maxSizeX, maxSizeY, buffFrame);
            break;
    }

    /* Extract the region of interest with binning.
     * If the entire image is being used (no ROI or binning) that's OK because
     * convertImage detects that case and is very efficient */
    this->pRaw->initDimension(&dimsOut[0], sizeX);
    dimsOut[0].binning = binX;
    dimsOut[0].offset = minX;
    dimsOut[0].reverse = reverseX;
    this->pRaw->initDimension(&dimsOut[1], sizeY);
    dimsOut[1].binning = binY;
    dimsOut[1].offset = minY;
    dimsOut[1].reverse = reverseY;
    /* We save the most recent image buffer so it can be used in the read() function.
     * Now release it before getting a new version. */
    if (this->pArrays[addr])
    	this->pArrays[addr]->release();
    status = this->pNDArrayPool->convert(this->pRaw,
                                         &this->pArrays[addr],
                                         dataType,
                                         dimsOut);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error allocating buffer in convert()\n",
                    driverName, functionName);
        return(status);
    }
    pImage = this->pArrays[addr];
    pImage->getInfo(&arrayInfo);

    status = asynSuccess;
    status |= setIntegerParam(addr, NDArraySize,  arrayInfo.totalBytes);
    status |= setIntegerParam(addr, NDArraySizeX, pImage->dims[0].size);
    status |= setIntegerParam(addr, NDArraySizeY, pImage->dims[1].size);
    if (status)
    	asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error setting parameters\n",
                    driverName, functionName);

    return(status);
}

//_____________________________________________________________________________________________

void PerkinElmer::enumSensors (void)
{
ACQDESCPOS Pos = 0;
int iFrames;

	bEnableIRQ =TRUE;
	uiPEResult = Acquisition_EnumSensors(&uiNumSensors, bEnableIRQ, FALSE);
	if (uiPEResult!=HIS_ALL_OK)
		printf("Error: %d  EnumSensors failed!\n", uiPEResult);

	printf("%d sensors recognized!\n", uiNumSensors);

	//now we iterate through all this sensors and display sensor data
	do
	{
		if ((uiPEResult = Acquisition_GetNextSensor(&Pos, &hAcqDesc)) != HIS_ALL_OK)
			printf("Error: %d  GetNextSensor failed!\n", uiPEResult);

		//ask for communication device type and its number
		if ((uiPEResult=Acquisition_GetCommChannel(hAcqDesc, &uiChannelType, &iChannelNum)) != HIS_ALL_OK)
			printf("Error: %d  GetCommChannel failed!\n", uiPEResult);

		//ask for data organization of all sensors
		if ((uiPEResult=Acquisition_GetConfiguration(hAcqDesc, (unsigned int *) &iFrames, &uiRows, &uiColumns, &uiDataType,
				&uiSortFlags, &bEnableIRQ, &dwAcqType, &dwSystemID, &dwSyncMode, &dwHwAccess)) != HIS_ALL_OK)
			printf("Error: %d GetConfiguration failed!\n", uiPEResult);

		if (uiChannelType==HIS_BOARD_TYPE_ELTEC_XRD_FGX)
			printf("Channel Type: HIS_BOARD_TYPE_ELTEC_XRD_FGX\n");
		if (uiChannelType==HIS_BOARD_TYPE_ELTEC)
			printf("Channel Type: HIS_BOARD_TYPE_ELTEC\n");
		printf("Channel Number: %d\n", iChannelNum);
		printf("Frames: %d\n", iFrames);
		printf("Rows: %d, Columns: %d\n\n", uiRows, uiColumns);

	} while (Pos!=0);

}

//_____________________________________________________________________________________________

BOOL PerkinElmer::initializeDetector (void)
{
const char* functionName = "initializeDetector";
ACQDESCPOS Pos = 0;
WORD wBinning=1;
int timings = 8;
double 	m_pTimingsListBinning[8];
/*double acqTime;*/
int status = asynSuccess;
int	iGain,
	iTimeIndex,
	iSyncMode,
	error;
int addr=0;
int iFrames;
DWORD 	dwDwellTime,
		dwSyncTime;
UINT devFrames, devRows, devColumns, devDataType, devSortFlags;
BOOL devIrqEnabled;
DWORD devAcqType, devSystemID, devSyncMode, devHwAccess;

	printf("\n\nAttempting to initialize PE detector...\n");

	//let the user know what's going on...
    status |= setIntegerParam(addr, PE_StatusRBV, PE_STATUS_INITIALIZING);
    callParamCallbacks(addr, addr);

	//get some information
    status |= getIntegerParam(addr, PE_NumFrameBuffers, (int *) &uiNumFrameBuffers);
    status |= getIntegerParam(addr, PE_Gain, &iGain);
    status |= getIntegerParam(addr, PE_DwellTime, &iTimeIndex);
    status |= getIntegerParam(addr, PE_SyncTime, (int *) &dwSyncTime);
    //status |= getIntegerParam(addr, ADTriggerMode, &iSyncMode);
	iSyncMode = this->trigModeReq;
    if (status)
    	asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error getting parameters\n",
                    driverName, functionName);

	enumSensors ();

	printf("***NOTE***  This driver will only control the first enumerated sensor!!!\n\n");

	printf("Select first available sensor...\n");

	Pos = NULL;
	if ((uiPEResult = Acquisition_GetNextSensor(&Pos, &hAcqDesc)) != HIS_ALL_OK)
	{
		printf("Error: %d  GetNextSensor failed!\n", uiPEResult);
		return (false);
	}

	//ask for communication device type and its number
	if ((uiPEResult=Acquisition_GetCommChannel(hAcqDesc, &uiChannelType, &iChannelNum)) != HIS_ALL_OK)
	{
		printf("Error: %d  GetCommChannel failed!\n", uiPEResult);
		return (false);
	}

	// now set callbacks and messages
	if ((uiPEResult=Acquisition_SetCallbacksAndMessages(hAcqDesc, NULL, 0, 0, OnEndFrameCallback, OnEndAcqCallback)) != HIS_ALL_OK)
	{
		printf("Error: %d  SetCallbacksAndMessages failed!\n", uiPEResult);
		return (false);
	}

	//  set detector gain
	if ((uiPEResult = Acquisition_SetCameraGain(hAcqDesc, iGain))!=HIS_ALL_OK)
	{
		printf("Error: %d  SetCameraGain failed!\n", uiPEResult);
		return (false);
	}

	// set detector to default binning mode
	if ((uiPEResult = Acquisition_SetCameraBinningMode(hAcqDesc,wBinning))!=HIS_ALL_OK)
	{
		printf("Error: %d  SetCameraBinningMode failed!\n", uiPEResult);
		return (false);
	}

	// get int times for selected binning mode
	if ((uiPEResult = Acquisition_GetIntTimes(hAcqDesc, m_pTimingsListBinning, &timings))!=HIS_ALL_OK)
	{
		printf("Error: %d  GetIntTimes failed!\n", uiPEResult);
		return (false);
	}

	//  set detector timing mode
	if ((uiPEResult = Acquisition_SetCameraMode(hAcqDesc, 0))!=HIS_ALL_OK)
	{
		printf("Error: %d  SetCameraMode failed!\n", uiPEResult);
		return (false);
	}

	//set dwell time
/**	switch (iSyncMode)
/*	{
/*		case PE_FREE_RUNNING : {
/*			Acquisition_SetFrameSyncMode(hAcqDesc,HIS_SYNCMODE_FREE_RUNNING);
/*
/*			break;
/*		}
/*
/*		case PE_EXTERNAL_TRIGGER : {
/*			Acquisition_SetFrameSyncMode(hAcqDesc,HIS_SYNCMODE_EXTERNAL_TRIGGER);
/*
/*			break;
/*		}
/*
/*		case PE_INTERNAL_TRIGGER : {
/*			printf("Setting AcquireTime %f\n", this->acqTimeReq );
/*			for (int loop=0;loop<timings;loop++)
/*				printf ("m_pTimingsListBinning[%d] = %e\n", loop, m_pTimingsListBinning[loop]);
/*			dwDwellTime = (DWORD) (this->acqTimeReq * 1000000);
/*			printf ("internal timer requested: %d\n", dwDwellTime);
/*
/*			error = Acquisition_SetFrameSyncMode(hAcqDesc,HIS_SYNCMODE_INTERNAL_TIMER);
/*			error = Acquisition_SetTimerSync(hAcqDesc, &dwDwellTime);
/*
/*            this->acqTimeAct = dwDwellTime/1000000.;
/*			printf ("internal timer set: %f\n", this->acqTimeAct);
/*            setDoubleParam(addr, ADAcquireTime, this->acqTimeAct );
/*
/*			printf ("error: %d\n", error);
/*			callParamCallbacks();
/*
/*			break;
/*		}
/*
/*		case PE_SOFT_TRIGGER : {
/*			Acquisition_SetFrameSyncMode(hAcqDesc,HIS_SYNCMODE_SOFT_TRIGGER);
/*
/*			break;
/*		}
/*
/*	}
*/

	this->setTriggerMode();
	this->setExposureTime();
	//ask for data organization of sensor
	if ((uiPEResult=Acquisition_GetConfiguration(hAcqDesc, (unsigned int *) &iFrames, &uiRows, &uiColumns, &uiDataType,
			&uiSortFlags, &bEnableIRQ, &dwAcqType, &dwSystemID, &dwSyncMode, &dwHwAccess)) != HIS_ALL_OK)
		printf("Error: %d GetConfiguration failed!\n", uiPEResult);

	if (uiChannelType==HIS_BOARD_TYPE_ELTEC_XRD_FGX)
		printf("Channel Type: HIS_BOARD_TYPE_ELTEC_XRD_FGX\n");
	if (uiChannelType==HIS_BOARD_TYPE_ELTEC)
		printf("Channel Type: HIS_BOARD_TYPE_ELTEC\n");
	printf ("System ID: %d\n", dwSystemID);

	//allocate frame memory
	if ((uiNumFrameBuffers <= 0) || (uiNumFrameBuffers > 500))
	{
		uiNumFrameBuffers = 500;
		status = setIntegerParam(addr, PE_NumFrameBuffers, uiNumFrameBuffers);
	}
	if (pAcqBuffer != NULL)
		free (pAcqBuffer);
	pAcqBuffer = (unsigned short *) malloc(uiNumFrameBuffers*uiRows*uiColumns*sizeof(short));
	if (pAcqBuffer == NULL)
	{
		printf("Error:  Memory allocation failed for %d frames!\n", uiNumFrameBuffers);
		return (false);
	}
	printf("\nChannel Number: %d\n", iChannelNum);
	printf("Memory for %d frames allocated...\n", uiNumFrameBuffers);
	printf("Rows: %d, Columns: %d\n\n", uiRows, uiColumns);

	printf("Detector Configuration\n");

	Acquisition_GetConfiguration( hAcqDesc, &devFrames, &devRows, &devColumns, &devDataType,
									&devSortFlags, &devIrqEnabled, &devAcqType, &devSystemID, &devSyncMode,
									&devHwAccess);

	printf("     Number of frames: %d\n", devFrames);
	printf("     Number of Rows: %d, Number of Columns: %d\n", devRows, devColumns);
	printf("     Data type: %d\n", devDataType);
	printf("     Sort Flags %d\n", devSortFlags);
	printf("     IRQ Enabled %d\n", devIrqEnabled);
	printf("     Acquisition Type %d\n", devAcqType);
	printf("     SystemID %d\n", devSystemID);
	printf("     Sync Mode %d\n", devSyncMode);
	printf("     HW Access %d\n", devHwAccess);

	//Update readback values
	//let the user know what's going on...
	status = 0;
    status |= setIntegerParam(addr, PE_SystemID, dwSystemID);
    status |= setIntegerParam(addr, PE_StatusRBV, PE_STATUS_OK);
	status |= setIntegerParam(addr, PE_GainRBV, iGain);
	status |= setIntegerParam(addr, PE_DwellTimeRBV, iTimeIndex);
	status |= setIntegerParam(addr, PE_NumFrameBuffersRBV, uiNumFrameBuffers);
//    status |= setIntegerParam(addr, ADTriggerMode, iSyncMode);
//    status |= setIntegerParam(addr, PE_SyncTimeRBV, dwDwellTime);

	return (true);
}

//_____________________________________________________________________________________________

void PerkinElmer::acquireImage (void)
{
const char* 		functionName = "acquireImage";
HANDLE 				hevEndAcq=NULL;
int 				iMode,
					iFrames,
					iUseOffset,
					iUseGain,
					iUsePixelCorrection,
					iFastCollectMode,
					addr=0,
					numImages = 0;
int 				status = asynSuccess;
DWORD 				HISError,
					FGError;
int numLeadingImagesToSkip = 0;
int skipLeadingImages;

	//get some information
   	status |= getIntegerParam(addr, ADImageMode, &iMode);
   	status |= getIntegerParam(addr, PE_UseOffset, &iUseOffset);
   	status |= getIntegerParam(addr, PE_UseGain, &iUseGain);
   	status |= getIntegerParam(addr, PE_UsePixelCorrection, &iUsePixelCorrection);
   	status |= getIntegerParam(addr, PE_FastCollectMode, &iFastCollectMode);
   	status |= getIntegerParam(addr, PE_SkipLeadingPulses, &skipLeadingImages);

   	if (status)
   		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
   	                "%s:%s: error getting parameters\n",
   	                driverName, functionName);


	printf("Frame mode: %d\n", iMode);

	if (skipLeadingImages !=0 ) {
	   	status |= getIntegerParam(addr, PE_NumPulsesToSkip, &numLeadingImagesToSkip);

		printf("Skipping Leading %d images\n", numLeadingImagesToSkip );
	}


    switch(iMode)
    {
    	case ADImageSingle:	iFrames = 1; break;
        case ADImageMultiple: {
		    status = getIntegerParam(addr, PE_NumFrameBuffers, (int *) &uiNumFrameBuffers);
 		   	if ( numImages > (uiNumFrameBuffers - numLeadingImagesToSkip)) {
			    iFrames = uiNumFrameBuffers;
			}
			else {
			    iFrames = numImages;
			}
		}
        case ADImageContinuous: {
		    status = getIntegerParam(addr, PE_NumFrameBuffers, (int *) &uiNumFrameBuffers);
			iFrames = uiNumFrameBuffers;
			break;
		}
    }

	printf("Frames: %d\n", iFrames);
	printf("Rows: %d, Columns: %d\n\n", uiRows, uiColumns);

	dataAcqStruct.pDataBuffer = pAcqBuffer;
	dataAcqStruct.pOffsetBuffer = pOffsetBuffer;
	dataAcqStruct.pGainBuffer = pGainBuffer;
	dataAcqStruct.pPixelCorrectionList = pPixelCorrectionList;
	dataAcqStruct.iAcqMode = PE_ACQUIRE_ACQUISITION;
	dataAcqStruct.uiRows = uiRows;
	dataAcqStruct.uiColumns = uiColumns;
	dataAcqStruct.iUseOffset = iUseOffset;
	dataAcqStruct.iUseGain = iUseGain;
	dataAcqStruct.iUsePixelCorrections = iUsePixelCorrection;
	dataAcqStruct.pPerkinElmer = this;
	dataAcqStruct.numBufferFrames = iFrames;
	dataAcqStruct.iFastCollectMode = iFastCollectMode;
	printf( "FastCollectMode %d\n", iFastCollectMode);

	Acquisition_SetAcqData(hAcqDesc, (DWORD) &dataAcqStruct);
	if ((uiPEResult=Acquisition_DefineDestBuffers(hAcqDesc, pAcqBuffer,	iFrames, uiRows, uiColumns))!=HIS_ALL_OK)
		printf("Error : %d  Acquisition_DefineDestBuffers failed!\n", uiPEResult);

	setIntegerParam(addr, ADNumImagesCounter, 0);
	Acquisition_ResetFrameCnt(hAcqDesc);
	Acquisition_SetReady(hAcqDesc, 1);
	switch (iMode) {
		case ADImageSingle:
	 	if((uiPEResult=Acquisition_Acquire_Image(hAcqDesc,iFrames+numLeadingImagesToSkip,numLeadingImagesToSkip,HIS_SEQ_ONE_BUFFER, NULL, NULL, NULL))!=HIS_ALL_OK)
		{
			printf("Error: %d Acquisition_Acquire_Image failed!\n", uiPEResult);
			Acquisition_GetErrorCode(hAcqDesc,&HISError,&FGError);
			printf ("HIS Error: %d, Frame Grabber Error: %d\n", HISError, FGError);
		}
        break;
    	case ADImageMultiple:
		getIntegerParam(addr, ADNumImages, &numImages);
    	if ( numImages > (uiNumFrameBuffers-1)) {
		 	if((uiPEResult=Acquisition_Acquire_Image(hAcqDesc,iFrames,numLeadingImagesToSkip,HIS_SEQ_CONTINUOUS, NULL, NULL, NULL))!=HIS_ALL_OK)
			{
				printf("Error: %d Acquisition_Acquire_Image failed!\n", uiPEResult);
				Acquisition_GetErrorCode(hAcqDesc,&HISError,&FGError);
				printf ("HIS Error: %d, Frame Grabber Error: %d\n", HISError, FGError);
			}
		}
		else {
		 	if((uiPEResult=Acquisition_Acquire_Image(hAcqDesc,iFrames+numLeadingImagesToSkip,numLeadingImagesToSkip,HIS_SEQ_ONE_BUFFER, NULL, NULL, NULL))!=HIS_ALL_OK)
			{
				printf("Error: %d Acquisition_Acquire_Image failed!\n", uiPEResult);
				Acquisition_GetErrorCode(hAcqDesc,&HISError,&FGError);
				printf ("HIS Error: %d, Frame Grabber Error: %d\n", HISError, FGError);
			}
		}
		break;
		case ADImageContinuous:
		if((uiPEResult=Acquisition_Acquire_Image(hAcqDesc,iFrames,numLeadingImagesToSkip,HIS_SEQ_CONTINUOUS, NULL, NULL, NULL))!=HIS_ALL_OK)
		{
			printf("Error: %d Acquisition_Acquire_Image failed!\n", uiPEResult);
			Acquisition_GetErrorCode(hAcqDesc,&HISError,&FGError);
			printf ("HIS Error: %d, Frame Grabber Error: %d\n", HISError, FGError);
		}
		break;
	}

	printf ("Acquisition started...\n");

}

//_____________________________________________________________________________________________

void PerkinElmer::acquireOffsetImage (void)
{
const char* 		functionName = "acquireOffsetImage";
HANDLE 				hevEndAcq=NULL;
int					iFrames,
					addr=0;
int status = asynSuccess;

    status = getIntegerParam(addr, PE_NumOffsetFrames, &iFrames);
    if (status)
    	asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error getting parameters\n",
                    driverName, functionName);

	printf("acquireOffsetImage...\n");
	printf("Frames: %d\n", iFrames);
	printf("Rows: %d, Columns: %d\n\n", uiRows, uiColumns);

	if (pOffsetBuffer != NULL)
		free (pOffsetBuffer);
	pOffsetBuffer = (unsigned short *) malloc(sizeof(unsigned short)*uiRows*uiColumns);
	if (pOffsetBuffer == NULL)
	{
		printf("Error:  Memory allocation failed for offset buffer!\n");
		return;
	}
	printf("Memory for offset buffer (%dx%d) allocated...\n", uiRows, uiColumns);
	memset (pOffsetBuffer, 0, sizeof(unsigned short)*uiRows*uiColumns);

	dataAcqStruct.pDataBuffer = pAcqBuffer;
	dataAcqStruct.pOffsetBuffer = pOffsetBuffer;
	dataAcqStruct.pGainBuffer = pGainBuffer;
	dataAcqStruct.pPixelCorrectionList = pPixelCorrectionList;
	dataAcqStruct.iAcqMode = PE_ACQUIRE_OFFSET;
	dataAcqStruct.uiRows = uiRows;
	dataAcqStruct.uiColumns = uiColumns;
	dataAcqStruct.iUseOffset = 0;
	dataAcqStruct.iUseGain = 0;
	dataAcqStruct.iUsePixelCorrections = 0;
	dataAcqStruct.pPerkinElmer = this;
	dataAcqStruct.numBufferFrames = 1;
	dataAcqStruct.iFastCollectMode = 0;

	Acquisition_SetAcqData(hAcqDesc, (DWORD) &dataAcqStruct);

	if((uiPEResult=Acquisition_Acquire_OffsetImage(hAcqDesc,pOffsetBuffer,uiRows,uiColumns,iFrames))!=HIS_ALL_OK)
		printf("Error: %d Acquisition_Acquire_OffsetImage failed!\n", uiPEResult);

	bAcquiringOffset = true;

	printf ("Offset acquisition started...\n");

}

//_____________________________________________________________________________________________

void PerkinElmer::acquireGainImage (void)
{
const char* 		functionName = "acquireGainImage";
HANDLE 				hevEndAcq=NULL;
int					iFrames,
					addr=0;
int status = asynSuccess;

    status = getIntegerParam(addr, PE_NumGainFrames, &iFrames);
    if (status)
    	asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error getting parameters\n",
                    driverName, functionName);

	printf("acquireGainImage...\n");
	printf("Frames: %d\n", iFrames);
	printf("Rows: %d, Columns: %d\n\n", uiRows, uiColumns);

	if (pGainBuffer != NULL)
		free (pGainBuffer);
	pGainBuffer = (DWORD *) malloc(uiRows*uiColumns*sizeof(DWORD));
	if (pGainBuffer == NULL)
	{
		printf("Error:  Memory allocation failed for gain buffer!\n");
		return;
	}

	printf("Memory for gain buffer (%dx%d) allocated...\n", uiRows, uiColumns);

	dataAcqStruct.pDataBuffer = pAcqBuffer;
	dataAcqStruct.pOffsetBuffer = pOffsetBuffer;
	dataAcqStruct.pGainBuffer = pGainBuffer;
	dataAcqStruct.pPixelCorrectionList = pPixelCorrectionList;
	dataAcqStruct.iAcqMode = PE_ACQUIRE_GAIN;
	dataAcqStruct.uiRows = uiRows;
	dataAcqStruct.uiColumns = uiColumns;
	dataAcqStruct.iUseOffset = 0;
	dataAcqStruct.iUseGain = 0;
	dataAcqStruct.iUsePixelCorrections = 0;
	dataAcqStruct.pPerkinElmer = this;
	dataAcqStruct.numBufferFrames = 1;
	dataAcqStruct.iFastCollectMode = 0;

	Acquisition_SetAcqData(hAcqDesc, (DWORD) &dataAcqStruct);

	if((uiPEResult=Acquisition_Acquire_GainImage(hAcqDesc,pOffsetBuffer,pGainBuffer,uiRows,uiColumns,iFrames))!=HIS_ALL_OK)
		printf("Error: %d Acquisition_Acquire_GainImage failed!\n", uiPEResult);

	bAcquiringGain = true;

	printf ("Gain acquisition started...\n");

}

//_____________________________________________________________________________________________

void PerkinElmer::saveCorrectionFiles (void)
{
int 				iGainIndex,
					iTimeIndex,
					iSizeX,
					iSizeY,
					iByteDepth,
					addr=0,
					status = asynSuccess;
char				cpCorrectionsDirectory[256],
					cpFileName[256],
					cpGain[10],
					cpTime[10];
FILE				*pOutputFile;

	printf ("Trying to save correction files...\n");

	status |= getStringParam(PE_CorrectionsDirectory, sizeof(cpCorrectionsDirectory), cpCorrectionsDirectory);
	status |= getIntegerParam(addr, PE_GainRBV, &iGainIndex);
	status |= getIntegerParam(addr, PE_DwellTimeRBV, &iTimeIndex);
    status |= getIntegerParam(addr, NDArraySizeX, &iSizeX);
    status |= getIntegerParam(addr, NDArraySizeY, &iSizeY);

	printf ("Saving corrections to path: %s!\n", cpCorrectionsDirectory);

	switch (iTimeIndex)
	{
		case TIME0 : sprintf (cpTime, "%s", TIME0_STR); break;
		case TIME1 : sprintf (cpTime, "%s", TIME1_STR); break;
		case TIME2 : sprintf (cpTime, "%s", TIME2_STR); break;
		case TIME3 : sprintf (cpTime, "%s", TIME3_STR); break;
		case TIME4 : sprintf (cpTime, "%s", TIME4_STR); break;
		case TIME5 : sprintf (cpTime, "%s", TIME5_STR); break;
		case TIME6 : sprintf (cpTime, "%s", TIME6_STR); break;
		case TIME7 : sprintf (cpTime, "%s", TIME7_STR); break;
	}

	switch (iGainIndex)
	{
		case GAIN0 : sprintf (cpGain, "%s", GAIN0_STR); break;
		case GAIN1 : sprintf (cpGain, "%s", GAIN1_STR); break;
		case GAIN2 : sprintf (cpGain, "%s", GAIN2_STR); break;
		case GAIN3 : sprintf (cpGain, "%s", GAIN3_STR); break;
		case GAIN4 : sprintf (cpGain, "%s", GAIN4_STR); break;
		case GAIN5 : sprintf (cpGain, "%s", GAIN5_STR); break;
	}

	//Save offset buffer
	if (pOffsetBuffer != NULL)
	{
		sprintf (cpFileName, "%sOffset_%s_%s.bin", cpCorrectionsDirectory, cpTime, cpGain);
		printf ("Offset file name: %s\n", cpFileName);

		pOutputFile = fopen (cpFileName, "wb");

		if (pOutputFile != NULL)
		{
			iByteDepth = sizeof (unsigned short);

			fwrite ((void *) &iSizeX, sizeof (int), 1, pOutputFile);
			if (ferror (pOutputFile))
			{
				printf ("Failed to write file header for file %s in PerkinElmer::saveCorrectionFiles.\n", cpFileName);
				return;
			}

			fwrite ((void *) &iSizeY, sizeof (int), 1, pOutputFile);
			if (ferror (pOutputFile))
			{
				printf ("Failed to write file header for file %s in PerkinElmer::saveCorrectionFiles.\n", cpFileName);
				return;
			}

			fwrite ((void *) &iByteDepth, sizeof (int), 1, pOutputFile);
			if (ferror (pOutputFile))
			{
				printf ("Failed to write file header for file %s in PerkinElmer::saveCorrectionFiles.\n", cpFileName);
				return;
			}

			fwrite (pOffsetBuffer, iByteDepth, iSizeX*iSizeY, pOutputFile);
			if (ferror (pOutputFile))
			{
				printf ("Failed to write data for file %s in PerkinElmer::saveCorrectionFiles.\n", cpFileName);
				return;
			}

			fclose (pOutputFile);
		}

	}

	//Save gain buffer
	if (pGainBuffer != NULL)
	{
		sprintf (cpFileName, "%sGain_%s_%s.bin", cpCorrectionsDirectory, cpTime, cpGain);
		printf ("Gain file name: %s\n", cpFileName);

		pOutputFile = fopen (cpFileName, "wb");

		if (pOutputFile != NULL)
		{
			iByteDepth = sizeof (DWORD);

			fwrite ((void *) &iSizeX, sizeof (int), 1, pOutputFile);
			if (ferror (pOutputFile))
			{
				printf ("Failed to write file header for file %s in PerkinElmer::saveCorrectionFiles.\n", cpFileName);
				return;
			}

			fwrite ((void *) &iSizeY, sizeof (int), 1, pOutputFile);
			if (ferror (pOutputFile))
			{
				printf ("Failed to write file header for file %s in PerkinElmer::saveCorrectionFiles.\n", cpFileName);
				return;
			}

			fwrite ((void *) &iByteDepth, sizeof (int), 1, pOutputFile);
			if (ferror (pOutputFile))
			{
				printf ("Failed to write file header for file %s in PerkinElmer::saveCorrectionFiles.\n", cpFileName);
				return;
			}

			fwrite (pGainBuffer, iByteDepth, iSizeX*iSizeY, pOutputFile);
			if (ferror (pOutputFile))
			{
				printf ("Failed to write data for file %s in PerkinElmer::saveCorrectionFiles.\n", cpFileName);
				return;
			}

			fclose (pOutputFile);
		}

	}

	printf ("Correction files saved!\n");
}

//_____________________________________________________________________________________________

void PerkinElmer::loadCorrectionFiles (void)
{
int 				iGainIndex,
					iTimeIndex,
					iSizeX,
					iSizeY,
					iByteDepth,
					addr=0,
					status = asynSuccess;
char				cpCorrectionsDirectory[256],
					cpPixelCorrectionFile[256],
					cpFileName[256],
					cpGain[10],
					cpTime[10];
FILE				*pInputFile;

struct stat 		stat_buffer;

	printf ("Trying to load correction files...\n");

	status |= getStringParam(PE_CorrectionsDirectory, sizeof(cpCorrectionsDirectory), cpCorrectionsDirectory);
	status |= getStringParam(PE_PixelCorrectionFile, sizeof(cpPixelCorrectionFile), cpPixelCorrectionFile);
	status |= getIntegerParam(addr, PE_GainRBV, &iGainIndex);
	status |= getIntegerParam(addr, PE_DwellTimeRBV, &iTimeIndex);
    status |= getIntegerParam(addr, NDArraySizeX, &iSizeX);
    status |= getIntegerParam(addr, NDArraySizeY, &iSizeY);

	switch (iTimeIndex)
	{
		case TIME0 : sprintf (cpTime, "%s", TIME0_STR); break;
		case TIME1 : sprintf (cpTime, "%s", TIME1_STR); break;
		case TIME2 : sprintf (cpTime, "%s", TIME2_STR); break;
		case TIME3 : sprintf (cpTime, "%s", TIME3_STR); break;
		case TIME4 : sprintf (cpTime, "%s", TIME4_STR); break;
		case TIME5 : sprintf (cpTime, "%s", TIME5_STR); break;
		case TIME6 : sprintf (cpTime, "%s", TIME6_STR); break;
		case TIME7 : sprintf (cpTime, "%s", TIME7_STR); break;
	}

	switch (iGainIndex)
	{
		case GAIN0 : sprintf (cpGain, "%s", GAIN0_STR); break;
		case GAIN1 : sprintf (cpGain, "%s", GAIN1_STR); break;
		case GAIN2 : sprintf (cpGain, "%s", GAIN2_STR); break;
		case GAIN3 : sprintf (cpGain, "%s", GAIN3_STR); break;
		case GAIN4 : sprintf (cpGain, "%s", GAIN4_STR); break;
		case GAIN5 : sprintf (cpGain, "%s", GAIN5_STR); break;
	}


	//load offset corrections
	sprintf (cpFileName, "%sOffset_%s_%s.bin", cpCorrectionsDirectory, cpTime, cpGain);
	printf ("Offset Correction File: %s\n", cpFileName);
	if ((stat (cpFileName, &stat_buffer) == 0) && (stat_buffer.st_mode & S_IFREG))
	{
		if (pOffsetBuffer != NULL)
			free (pOffsetBuffer);

		pInputFile = fopen (cpFileName, "rb");
		if (pInputFile != NULL)
		{
			fread (&iSizeX, sizeof (int), 1, pInputFile);
			if (ferror (pInputFile))
			{
				printf ("Failed to read file header for file %s in PerkinElmer::loadCorrectionFiles.\n", cpFileName);
				return;
			}

			fread (&iSizeY, sizeof (int), 1, pInputFile);
			if (ferror (pInputFile))
			{
				printf ("Failed to read file header for file %s in PerkinElmer::loadCorrectionFiles.\n", cpFileName);
				return;
			}

			fread (&iByteDepth, sizeof (int), 1, pInputFile);
			if (ferror (pInputFile))
			{
				printf ("Failed to read file header for file %s in PerkinElmer::loadCorrectionFiles.\n", cpFileName);
				return;
			}

			pOffsetBuffer = (unsigned short *) malloc (iSizeX * iSizeY * iByteDepth);
			fread (pOffsetBuffer, iByteDepth, iSizeX * iSizeY, pInputFile);
			if (ferror (pInputFile))
			{
				printf ("Failed to read data for file %s in PerkinElmer::loadCorrectionFiles.\n", cpFileName);
				return;
			}

			fclose (pInputFile);

			status |= setIntegerParam(addr, PE_OffsetAvailable, AVAILABLE);
			callParamCallbacks();
		}
		else
		    printf ("Failed to open file %s in PerkinElmer::loadCorrectionsFile!\n", cpFileName);
	}
	else
	    printf ("Failed to find offset file for time %s and gain %s!\n", cpTime, cpGain);


	//load gain corrections
	sprintf (cpFileName, "%sGain_%s_%s.bin", cpCorrectionsDirectory, cpTime, cpGain);
	printf ("Gain Correction File: %s\n", cpFileName);
	if ((stat (cpFileName, &stat_buffer) == 0) && (stat_buffer.st_mode & S_IFREG))
	{
		if (pGainBuffer != NULL)
			free (pGainBuffer);

		pInputFile = fopen (cpFileName, "rb");
		if (pInputFile != NULL)
		{
			fread (&iSizeX, sizeof (int), 1, pInputFile);
			if (ferror (pInputFile))
			{
				printf ("Failed to read file header for file %s in PerkinElmer::loadCorrectionFiles.\n", cpFileName);
				return;
			}

			fread (&iSizeY, sizeof (int), 1, pInputFile);
			if (ferror (pInputFile))
			{
				printf ("Failed to read file header for file %s in PerkinElmer::loadCorrectionFiles.\n", cpFileName);
				return;
			}

			fread (&iByteDepth, sizeof (int), 1, pInputFile);
			if (ferror (pInputFile))
			{
				printf ("Failed to read file header for file %s in PerkinElmer::loadCorrectionFiles.\n", cpFileName);
				return;
			}

			pGainBuffer = (DWORD *) malloc (iSizeX * iSizeY * iByteDepth);
			fread (pGainBuffer, iByteDepth, iSizeX * iSizeY, pInputFile);
			if (ferror (pInputFile))
			{
				printf ("Failed to read data for file %s in PerkinElmer::loadCorrectionFiles.\n", cpFileName);
				return;
			}

			fclose (pInputFile);

			status |= setIntegerParam(addr, PE_GainAvailable, AVAILABLE);
			callParamCallbacks();
		}
		else
		    printf ("Failed to open file %s in PerkinElmer::loadCorrectionsFile!\n", cpFileName);
	}
	else
	    printf ("Failed to find gain file for time %s and gain %s!\n", cpTime, cpGain);


	//load pixel correction file
	sprintf (cpFileName, "%s%s", cpCorrectionsDirectory, cpPixelCorrectionFile);
	printf ("Pixel Correction File: %s\n", cpFileName);
	if ((stat (cpFileName, &stat_buffer) == 0) && (stat_buffer.st_mode & S_IFREG))
	{
		status |= setStringParam (addr, PE_PixelCorrectionFileRBV, cpPixelCorrectionFile);
		readPixelCorrectionFile (cpFileName);
	}
	else
	    status |= setStringParam (addr, PE_PixelCorrectionFileRBV, "none");


	printf ("Correction files loaded!\n");

}

//_____________________________________________________________________________________________

void PerkinElmer::readPixelCorrectionFile (char *pixel_correction_file)
{
FILE				*pInputFile;
WinHeaderType		file_header;
WinImageHeaderType	image_header;
int 				iBufferSize,
					iCorrectionMapSize,
					addr=0,
					status = asynSuccess;
unsigned int		uiStatus;

	pInputFile = fopen (pixel_correction_file, "r");

	if (pInputFile != NULL)
	{
		//read file header
		fread ((void *) &file_header, 68, 1, pInputFile);
		if (ferror (pInputFile))
		{
			printf ("Failed to read file header from file %s in PerkinElmer::readPixelCorrectionFile.\n", pixel_correction_file);
			return;
		}

		//read image header
		fread ((void *) &image_header, 32, 1, pInputFile);
		if (ferror (pInputFile))
		{
			printf ("Failed to read image header from file %s in PerkinElmer::readPixelCorrectionFile.\n", pixel_correction_file);
			return;
		}

		//read bad pixel map
		if (pBadPixelMap != NULL)
			free (pBadPixelMap);
		iBufferSize = file_header.ULY * file_header.BRX * sizeof (unsigned short);
		pBadPixelMap = (unsigned short *) malloc (iBufferSize);
		printf ("buffer size: %d, pBadPixelMap: %d\n", iBufferSize, pBadPixelMap);
		if (pBadPixelMap == NULL)
		{
			printf ("Failed to allocate bad pixel map buffer in PerkinElmer::readPixelCorrectionFile.\n");
			return;
		}
		fread ((void *) pBadPixelMap, iBufferSize, 1, pInputFile);
		if (ferror (pInputFile))
		{
			printf ("Failed to read bad pixel map file %s in PerkinElmer::readPixelCorrectionFile.\n", pixel_correction_file);
			return;
		}

		fclose (pInputFile);

		int counter = 0;
		for (int loop=0;loop<file_header.ULY * file_header.BRX;loop++)
		{
			if (pBadPixelMap[loop] == 65535)
				counter++;
		}
		printf ("Bad pixel map read in!  %d bad pixels found!\n", counter);

		//first call with correction list = NULL returns size of buffer to allocate
		//second time gets the correction list
		uiStatus = Acquisition_CreatePixelMap (pBadPixelMap, file_header.ULY, file_header.BRX, NULL, &iCorrectionMapSize);
		pPixelCorrectionList = (int *) malloc (iCorrectionMapSize);
		uiStatus = Acquisition_CreatePixelMap (pBadPixelMap, file_header.ULY, file_header.BRX, pPixelCorrectionList, &iCorrectionMapSize);

		free (pBadPixelMap);

		status |= setIntegerParam(addr, PE_PixelCorrectionAvailable, AVAILABLE);
		callParamCallbacks();
	}
	else
		printf ("Failed to open file %s in PerkinElmer::readPixelCorrectionFile.\n", pixel_correction_file);

}

//-------------------------------------------------------------
asynStatus PerkinElmer::setTriggerMode() {
	int error;
	int mode;
	int addr = 0;

	mode = this->trigModeReq;

	switch (mode) {
	   case PE_FREE_RUNNING: {
			error = Acquisition_SetFrameSyncMode(hAcqDesc,HIS_SYNCMODE_FREE_RUNNING);
			break;
	   }
	   case PE_EXTERNAL_TRIGGER: {
			error = Acquisition_SetFrameSyncMode(hAcqDesc,HIS_SYNCMODE_EXTERNAL_TRIGGER);
			break;
	   }
	   case PE_INTERNAL_TRIGGER: {
			error = Acquisition_SetFrameSyncMode(hAcqDesc,HIS_SYNCMODE_INTERNAL_TIMER);
			break;
	   }
	   case PE_SOFT_TRIGGER: {
			error = Acquisition_SetFrameSyncMode(hAcqDesc,HIS_SYNCMODE_SOFT_TRIGGER);
			break;
	   }
	}
	this->trigModeAct = mode;
    setIntegerParam(addr, ADTriggerMode, this->trigModeAct);
    callParamCallbacks();

	return asynSuccess;

}

//-------------------------------------------------------------
asynStatus PerkinElmer::setExposureTime() {
	const char *functionName = "setExposureTime";
	int addr = 0;
	DWORD dwDwellTime;
	int status = asynSuccess;
	int error;

	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Setting AcquireTime %f\n",
				driverName, functionName, this->acqTimeReq );
	dwDwellTime = (DWORD) (this->acqTimeReq * 1000000);
	printf ("internal timer requested: %d\n", dwDwellTime);
	error = Acquisition_SetTimerSync(hAcqDesc, &dwDwellTime);
	this->acqTimeAct = dwDwellTime/1000000.;
	printf ("internal timer set: %f\n", this->acqTimeAct);
	status |= setDoubleParam(addr, ADAcquireTime, this->acqTimeAct );
	callParamCallbacks();

	return asynSuccess;

}
