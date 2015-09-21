 /* PerkinElmer.cpp
 *
 * This is a driver for the Perkin Elmer flat panel detectors
 *
 * It works with panels that are connected with frame grabbers (e.g. 0820, 1621) or
 * Gigabit Ethernet (e.g. 0822).
 *
 *
 * Author: Brian Tieman
 *
 * Created:  07/24/2008
 *
 * Modified by John Hammonds
 *
 * Current author: Mark Rivers
 * 
 * Major re-write done in late 2011 and early 2012 to support many new features
 * and improve performance.
 */

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

#include <epicsExport.h>
#include "PerkinElmer.h"

// Forward function definitions
static void CALLBACK endFrameCallbackC(HACQDESC hAcqDesc);
static void CALLBACK endAcqCallbackC(HACQDESC hAcqDesc);
static void acquireStopTaskC(void *drvPvt);
static void exitCallbackC(void *drvPvt);

//_____________________________________________________________________________________________

/** Configuration command for Perkin Elmer driver; creates a new PerkinElmer object.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] IDType The type of system ID being specifed in IDValue.  Allowed values are:<br/>
  *  IDType = 0 Frame grabber card, IDValue = "" (currently always uses the first frame grabber card)<br/>
  *  IDType = 1 GigE detector, IDValue = IP address (e.g. 164.54.160.21)<br/>
  *  IDType = 2 GigE detector, IDValue = MAC address (e.g. 00005b032e6b, must be lower-case letters)<br/>
  *  IDType = 3 GigE detector, IDValue = detector name (e.g. 8#2608).  Can get network detector names with asynReport(10)
  * \param[in] IDValue The detector ID as explained above (IP name, MAC address, detector name)
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  */
extern "C" int PerkinElmerConfig(const char *portName, int IDType, const char *IDValue,
                                 int maxBuffers, size_t maxMemory, int priority, int stackSize)
{
    new PerkinElmer(portName, IDType, IDValue, maxBuffers, maxMemory, priority, stackSize);
    return(asynSuccess);
}

//_____________________________________________________________________________________________
/** Constructor for Perkin Elmer driver; most parameters are simply passed to ADDriver::ADDriver.
  * After calling the base class constructor this method creates a thread to collect the detector data, 
  * and sets reasonable default values the parameters defined in this class, asynNDArrayDriver, and ADDriver.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] IDType The type of system ID being specifed in IDValue.  Allowed values are:<br/>
  *  IDType = 0 Frame grabber card, IDValue = "" (currently always uses the first frame grabber card)<br/>
  *  IDType = 1 GigE detector, IDValue = IP address (e.g. 164.54.160.21)<br/>
  *  IDType = 2 GigE detector, IDValue = MAC address (e.g. 00005b032e6b, must be lower-case letters)<br/>
  *  IDType = 3 GigE detector, IDValue = detector name (e.g. 8#2608).  Can get network detector names with asynReport(10)
  * \param[in] IDValue The detector ID as explained above (IP name, MAC address, detector name)
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  */

PerkinElmer::PerkinElmer(const char *portName,  int IDType, const char *IDValue,
                         int maxBuffers, size_t maxMemory, int priority, int stackSize)

    : ADDriver(portName, 1, (int)NUM_PERKIN_ELMER_PARAMS, maxBuffers, maxMemory, 0, 0, ASYN_CANBLOCK, 1, priority, stackSize)
{
  int status = asynSuccess;
  static const char *functionName = "PerkinElmer";

  pAcqBuffer_           = NULL;
  pOffsetBuffer_        = NULL;
  pGainBuffer_          = NULL;
  pBadPixelMap_         = NULL;
  pPixelCorrectionList_ = NULL;
  
  IDType_  = IDType;
  IDValue_ = epicsStrDup(IDValue);

  /* Add parameters for this driver */
  createParam(PE_InitializeString,                  asynParamInt32,   &PE_Initialize);
  createParam(PE_AcquireOffsetString,               asynParamInt32,   &PE_AcquireOffset);
  createParam(PE_NumOffsetFramesString,             asynParamInt32,   &PE_NumOffsetFrames);
  createParam(PE_CurrentOffsetFrameString,          asynParamInt32,   &PE_CurrentOffsetFrame);
  createParam(PE_UseOffsetString,                   asynParamInt32,   &PE_UseOffset);
  createParam(PE_OffsetAvailableString,             asynParamInt32,   &PE_OffsetAvailable);
  createParam(PE_OffsetConstantString,              asynParamInt32,   &PE_OffsetConstant);
  createParam(PE_AcquireGainString,                 asynParamInt32,   &PE_AcquireGain);
  createParam(PE_NumGainFramesString,               asynParamInt32,   &PE_NumGainFrames);
  createParam(PE_CurrentGainFrameString,            asynParamInt32,   &PE_CurrentGainFrame);
  createParam(PE_UseGainString,                     asynParamInt32,   &PE_UseGain);
  createParam(PE_GainAvailableString,               asynParamInt32,   &PE_GainAvailable);
  createParam(PE_GainFileString,                    asynParamOctet,   &PE_GainFile);
  createParam(PE_LoadGainFileString,                asynParamInt32,   &PE_LoadGainFile);
  createParam(PE_SaveGainFileString,                asynParamInt32,   &PE_SaveGainFile);
  createParam(PE_UsePixelCorrectionString,          asynParamInt32,   &PE_UsePixelCorrection);
  createParam(PE_PixelCorrectionAvailableString,    asynParamInt32,   &PE_PixelCorrectionAvailable);
  createParam(PE_PixelCorrectionFileString,         asynParamOctet,   &PE_PixelCorrectionFile);
  createParam(PE_LoadPixelCorrectionFileString,     asynParamInt32,   &PE_LoadPixelCorrectionFile);
  createParam(PE_GainString,                        asynParamInt32,   &PE_Gain);
  createParam(PE_NumFrameBuffersString,             asynParamInt32,   &PE_NumFrameBuffers);
  createParam(PE_TriggerString,                     asynParamInt32,   &PE_Trigger);
  createParam(PE_SyncModeString,                    asynParamInt32,   &PE_SyncMode);
  createParam(PE_CorrectionsDirectoryString,        asynParamOctet,   &PE_CorrectionsDirectory);
  createParam(PE_FrameBufferIndexString,            asynParamInt32,   &PE_FrameBufferIndex);
  createParam(PE_ImageNumberString,                 asynParamInt32,   &PE_ImageNumber);
  createParam(PE_SkipFramesString,                  asynParamInt32,   &PE_SkipFrames);
  createParam(PE_NumFramesToSkipString,             asynParamInt32,   &PE_NumFramesToSkip);

  /* Set some default values for parameters */
  status =  setStringParam (ADManufacturer, "Perkin Elmer");
  status |= setIntegerParam(NDArraySize, 0);
  status |= setIntegerParam(NDDataType, NDUInt16);

  //Detector parameter defaults
  status |= setIntegerParam(PE_NumFrameBuffers, 10);
  status |= setIntegerParam(PE_Gain, 0);
  status |= setIntegerParam(PE_Initialize, 0);
  status |= setIntegerParam(PE_AcquireOffset, 0);
  status |= setIntegerParam(PE_OffsetAvailable, 0);
  status |= setIntegerParam(PE_AcquireGain, 0);
  status |= setIntegerParam(PE_GainAvailable, 0);
  status |= setIntegerParam(PE_PixelCorrectionAvailable, 0);
  status |= setStringParam (PE_CorrectionsDirectory, "");
  status |= setStringParam (PE_GainFile, "");
  status |= setStringParam (PE_PixelCorrectionFile, "");
  status |= setIntegerParam(PE_FrameBufferIndex, 0);
  status |= setIntegerParam(PE_ImageNumber, 0);
  if (status) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
    "%s:%s: unable to set camera parameters\n", 
    driverName, functionName);
    return;
  }

  acquireStopEvent_ = epicsEventCreate(epicsEventEmpty);
  /* Create the thread that updates the images */
  status = (epicsThreadCreate("acquireStopTaskC",
                              epicsThreadPriorityMedium,
                              epicsThreadGetStackSize(epicsThreadStackMedium),
                              (EPICSTHREADFUNC)acquireStopTaskC,
                              this) == NULL);
  initializeDetector();

  // Set exit handler to clean up
  epicsAtExit(exitCallbackC, this);

}

//_____________________________________________________________________________________________

/** Callback function that is called by EPICS when the IOC exits */
static void exitCallbackC(void *pPvt)
{
  PerkinElmer *pPerkinElmer = (PerkinElmer*) pPvt;
  delete(pPerkinElmer);
}

//_____________________________________________________________________________________________
/** Destructor for Perkin Elmer driver; most parameters are simply passed to ADDriver::ADDriver.
 * Frees all resources and calls Acquisition_Close() */

PerkinElmer::~PerkinElmer()
{
  unsigned int uiPEResult;
  static const char *functionName="~PerkinElmer";
  
  uiPEResult = Acquisition_Close(hAcqDesc_);
  reportXISStatus(uiPEResult, functionName, "Acquisition_Close(hAcqDesc_=%p)\n", hAcqDesc_);

  if (pAcqBuffer_ != NULL)
    free(pAcqBuffer_);

  if (pOffsetBuffer_ != NULL)
    free(pOffsetBuffer_);

  if (pGainBuffer_ != NULL)
    free(pGainBuffer_);

  if (pBadPixelMap_ != NULL)
    free(pBadPixelMap_);

  if (pPixelCorrectionList_ != NULL)
    free(pPixelCorrectionList_);
}


//_____________________________________________________________________________________________

/** Connects to a detector which is specified in the IDType and IDValue parameters to the constructor */
bool PerkinElmer::initializeDetector(void)
{
  ACQDESCPOS Pos = 0;
  WORD wBinning = 1;
  int timings = 8;
  double m_pTimingsListBinning[8];
  int status = asynSuccess;
  int iGain;
  long lPacketDelay;
  WORD wTiming;
  unsigned int uiPEResult;
  bool bInitAlways = false;
  BOOL bSelfInit = true;
  static const char* functionName = "initializeDetector";

  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s: Attempting to initialize PE detector...\n",
    driverName, functionName);

  status |= setIntegerParam(ADStatus, ADStatusInitializing);
  callParamCallbacks();

  status |= getIntegerParam(PE_NumFrameBuffers, (int *)&uiNumFrameBuffers_);
  status |= getIntegerParam(PE_Gain, &iGain);
  if (status)
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: error getting parameters\n",
              driverName, functionName);

  bEnableIRQ_ = true;
  switch (IDType_) {
    case 0:
      uiPEResult = Acquisition_EnumSensors(&uiNumSensors_, bEnableIRQ_, bInitAlways);
      reportXISStatus(uiPEResult, functionName, 
        "Acquisition_EnumSensors(uiNumSensors_=%d, bEnableIRQ_=%d, bInitAlways=%d)\n",
        uiNumSensors_, bEnableIRQ_, bInitAlways);
      if (uiPEResult != HIS_ALL_OK) {
        return false;
      }
      asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: Total of %d sensors found. This driver will only control the first frame grabber.\n",
        driverName, functionName, uiNumSensors_);

      Pos = NULL;
      uiPEResult = Acquisition_GetNextSensor(&Pos, &hAcqDesc_);
      reportXISStatus(uiPEResult, functionName, "Acquisition_GetNextSensor(&Pos=%p, hAcqDesc_=%p)\n",
        &Pos, &hAcqDesc_);
      if (uiPEResult != HIS_ALL_OK) {
        return false;
      }
      break;
    case 1:
    case 2:
    case 3:
      // Open GigE detector by IP address (IDType=1), MAC address (IDType_=2) or detector name (IDType_=3)
      // Note the XSIZE and YSIZE in this call are not important because we set bSelfInit=true
      uiPEResult = Acquisition_GbIF_Init(&hAcqDesc_, 0, bEnableIRQ_, 1024, 1024, 
                                         bSelfInit, bInitAlways, IDType_, (unsigned char *)IDValue_);
      reportXISStatus(uiPEResult, functionName, 
        "Acquisition_GbIF_Init(&hAcqDesc_=%p, nChannelNr=%d, bEnableIRQ_=%d, uiRows=%d, uiColumns=%d," 
                               "bSelfInit=%d, bInitAlways=%d, IDType_=%d, IDValue_=%p)\n",
         &hAcqDesc_, 0, bEnableIRQ_, 1024, 1024, 
         bSelfInit, bInitAlways, IDType_, (unsigned char *)IDValue_);
      if (uiPEResult != HIS_ALL_OK) {
        return false;
      }
      uiPEResult = Acquisition_GbIF_CheckNetworkSpeed(hAcqDesc_, &wTiming, &lPacketDelay, 100); 
      reportXISStatus(uiPEResult, functionName, 
        "Acquisition_GbIF_CheckNetworkSpeed(hAcqDesc_=%p, wTiming=%d, lPacketDelay=%d, lMaxNetworkPercent=%d)\n",
        hAcqDesc_, wTiming, lPacketDelay, 100);
      uiPEResult = Acquisition_GbIF_GetPacketDelay(hAcqDesc_, &lPacketDelay); 
      reportXISStatus(uiPEResult, functionName, "Acquisition_GbIF_GetPacketDelay(hAcqDesc_=%p, lPacketDelay=%d)\n",
        hAcqDesc_, lPacketDelay);
      break;
    default:
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "%s:%s: Error: unknown ID Type=%d, must be 0-3!\n", 
        driverName, functionName);
      return false;
  }
      
  //ask for communication device type and its number
  uiPEResult = Acquisition_GetCommChannel(hAcqDesc_, &uiChannelType_, &iChannelNum_);
  reportXISStatus(uiPEResult, functionName, 
    "Acquisition_GetCommChannel(hAcqDesc_=%p, uiChannelType_=%d, iChannelNum_=%d)\n",
    hAcqDesc_, uiChannelType_, iChannelNum_);
  if (uiPEResult != HIS_ALL_OK) {
    return false;
  }

  // now set callbacks and messages
  uiPEResult = Acquisition_SetCallbacksAndMessages(hAcqDesc_, NULL, 0, 0, endFrameCallbackC, endAcqCallbackC);
  reportXISStatus(uiPEResult, functionName, 
    "Acquisition_SetCallbacksAndMessages(hAcqDesc_=%p, hWnd=%p,dwErrorMsg=%d, " 
    "dwLoosingFramesMsg=%d, endFrameCallbackC=%p, endAcqCallbackC=%p)\n",
    hAcqDesc_, NULL, 0, 0, endFrameCallbackC, endAcqCallbackC);
  if (uiPEResult != HIS_ALL_OK) {
    return false;
  }

  //  set detector gain
  uiPEResult = Acquisition_SetCameraGain(hAcqDesc_, iGain);
  reportXISStatus(uiPEResult, functionName, "Acquisition_SetCameraGain(hAcqDesc_=%p, iGain=%d)\n",
    hAcqDesc_, iGain);
  if (uiPEResult != HIS_ALL_OK) {
    return false;
  }

  // set detector to default binning mode
  uiPEResult = Acquisition_SetCameraBinningMode(hAcqDesc_, wBinning);
  reportXISStatus(uiPEResult, functionName, "Acquisition_SetCameraBinningMode(hAcqDesc_=%p, wBinning=%d)\n",
    hAcqDesc_, wBinning);
  if (uiPEResult != HIS_ALL_OK) {
    return false;
  }

  // get int times for selected binning mode
  uiPEResult = Acquisition_GetIntTimes(hAcqDesc_, m_pTimingsListBinning, &timings);
  reportXISStatus(uiPEResult, functionName, 
    "Acquisition_GetIntTimes(hAcqDesc_=%p, m_pTimingsListBinning=%d, timings=%d)\n",
    hAcqDesc_, m_pTimingsListBinning, timings);
  if (uiPEResult != HIS_ALL_OK) {
    return false;
  }

  //  set detector timing mode
  uiPEResult = Acquisition_SetCameraMode(hAcqDesc_, 0);
  reportXISStatus(uiPEResult, functionName, "Acquisition_SetCameraMode(hAcqDesc_=%p, dwMode=%d)\n", hAcqDesc_, 0);
  if (uiPEResult != HIS_ALL_OK) {
    return false;
  }

  //ask for data organization of sensor
  uiPEResult = Acquisition_GetConfiguration(hAcqDesc_, (unsigned int *) &uiDevFrames_, 
                                                 &uiRows_, &uiColumns_, &uiDataType_,
                                                 &uiSortFlags_, &bEnableIRQ_, &dwAcqType_, 
                                                 &dwSystemID_, &dwSyncMode_, &dwHwAccess_);
  reportXISStatus(uiPEResult, functionName, 
    "Acquisition_GetConfiguration(hAcqDesc_, uiDevFrames_=%u, uiRows_=%u, uiColumns_=%u, uiDataType_=%u, "
    "uiSortFlags_=%u, bEnableIRQ_=%d, dwAcqType_=%d, dwSystemID_=%d, dwSyncMode_=%d, dwHwAccess_=%d)\n",
    hAcqDesc_, (unsigned int *) uiDevFrames_, uiRows_, uiColumns_, uiDataType_,
    uiSortFlags_, bEnableIRQ_, dwAcqType_, dwSystemID_, dwSyncMode_, dwHwAccess_);
  if (uiPEResult != HIS_ALL_OK) {
    return false;
  }
  
  // Get the hardware header information
  uiPEResult = Acquisition_GetHwHeaderInfoEx(hAcqDesc_, &cHwHeaderInfo_, &cHwHeaderInfoEx_);
  reportXISStatus(uiPEResult, functionName, 
    "Acquisition_GetHwHeaderInfoEx(hAcqDesc_=%p, &cHwHeaderInfo_=%p, &cHwHeaderInfoEx_=%p)\n",
    hAcqDesc_, &cHwHeaderInfo_, &cHwHeaderInfoEx_);
  if (uiPEResult != HIS_ALL_OK) {
    return false;
  }
  
   //allocate frame memory
  if ((uiNumFrameBuffers_ <= 0) || (uiNumFrameBuffers_ > 500))
  {
    uiNumFrameBuffers_ = 500;
    status = setIntegerParam(PE_NumFrameBuffers, uiNumFrameBuffers_);
  }
  if (pAcqBuffer_ != NULL)
    free (pAcqBuffer_);
  pAcqBuffer_ = (epicsUInt16 *) malloc(uiNumFrameBuffers_ * uiRows_ * uiColumns_ * sizeof(epicsUInt16));
  if (pAcqBuffer_ == NULL)
  {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: Error:  Memory allocation failed for %d frames!\n", 
      driverName, functionName, uiNumFrameBuffers_);
      return false;
  }
#ifdef __X64
  uiPEResult = Acquisition_SetAcqData(hAcqDesc_, (void *) this);
#else
  uiPEResult = Acquisition_SetAcqData(hAcqDesc_, (DWORD) this);
#endif
  reportXISStatus(uiPEResult, functionName, "Acquisition_SetAcqData(hAcqDesc_=%p, acqData=%p)\n", hAcqDesc_, this);
  if (uiPEResult != HIS_ALL_OK) {
    return false;
  }

  // Set ADModel based on HeaderID and Cameratype.  Table from XIS manual
  // Default to older cameras with Header ID=13
  status |= setStringParam (ADModel, "XRD[0840/1620/1640]xN");
  if (cHwHeaderInfoEx_.wHeaderID == 14) {
    switch (cHwHeaderInfoEx_.wCameratype) {
      case 1:
        status |= setStringParam (ADModel, "XRD [0820/1621/1641] xN");
        break;
      case 2:
        status |= setStringParam (ADModel, "XRD [0820/1620/1621] xN");
        break;
      case 7:
        status |= setStringParam (ADModel, "XRD 1640 xN ES");
        break;
      case 8:
        status |= setStringParam (ADModel, "XRD 0822 xO");
        break;
      case 9:
        status |= setStringParam (ADModel, "XRD 1622 xO");
        break;
      case 10:
        status |= setStringParam (ADModel, "XRD 0822 xP");
        break;
      case 11:
        status |= setStringParam (ADModel, "XRD 1622/1642 xP");
        break;
      case 12:
        status |= setStringParam (ADModel, "XRD 1642 xP");
        break;
      case 13:
        status |= setStringParam (ADModel, "XRPAD 4336/4343");
        break;
      case 15:
        status |= setStringParam (ADModel, "XRD 1611 xP");
        break;
    }
  }
        

  //Update readback values
  status = 0;
  status |= setIntegerParam(ADMaxSizeX, uiColumns_);
  status |= setIntegerParam(ADMaxSizeY, uiRows_);
  status |= setIntegerParam(ADSizeX, uiColumns_);
  status |= setIntegerParam(ADSizeY, uiRows_);
  status |= setIntegerParam(NDArraySizeX, uiColumns_);
  status |= setIntegerParam(NDArraySizeY, uiRows_);
  status |= setIntegerParam(PE_Gain, iGain);
  status |= setIntegerParam(PE_NumFrameBuffers, uiNumFrameBuffers_);
  status |= setIntegerParam(ADStatus, ADStatusIdle);

  return true;
}


//_____________________________________________________________________________________________

/** Configures the detector binning according to the ADBinX and ADBinY parameters */
void PerkinElmer::setBinning(void)
{
  unsigned int uiPEResult;
  int binX, binY, sizeX, sizeY;
  int status;
  bool validBinning = true;
  WORD PEBinning;
  int averageBinning = 256;
  int accumulateBinning = 512;
  static const char *functionName = "setBinning";

  status = getIntegerParam(ADBinX, &binX);
  status |= getIntegerParam(ADBinY, &binY);
  // If either binning is not defined return because it has not yet
  // been initialized, but we will be called again when it is.
  if (status) return;
  getIntegerParam(ADMaxSizeX, &sizeX);
  getIntegerParam(ADMaxSizeY, &sizeY);

  if ((cHwHeaderInfo_.dwHeaderID >= 14) && 
      (cHwHeaderInfoEx_.wCameratype >= 2)) {
    if      ((binX == 1) && (binY == 1)) PEBinning = 1;
    else if ((binX == 2) && (binY == 2)) PEBinning = 2;
    else if ((binX == 4) && (binY == 4)) PEBinning = 3;
    else if ((binX == 1) && (binY == 2)) PEBinning = 4;
    else if ((binX == 1) && (binY == 4)) PEBinning = 5;
    else validBinning = false;
    // Hardcode average style binning for now
    PEBinning = PEBinning + averageBinning;
  } else {
    if      ((binX == 1) && (binY == 1)) PEBinning = 0;
    else if ((binX == 2) && (binY == 2)) PEBinning = 1;
    else validBinning = false;
  }
  
  if (validBinning) {
    uiPEResult = Acquisition_SetCameraBinningMode(hAcqDesc_, PEBinning);
    reportXISStatus(uiPEResult, functionName, 
      "Acquisition_SetCameraBinningMode(hAcqDesc_=%p, PEBinning=%d)\n",
      hAcqDesc_, PEBinning);
    if (uiPEResult != HIS_ALL_OK) {
      return;
    }
    uiColumns_ = sizeX/binX;
    uiRows_    = sizeY/binY;
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
      "%s:%s:  set binning %d x %d, PEBinning=%d\n", 
      driverName, functionName, binX, binY, PEBinning);
    // If the binning changes then the offset needs to be collected again.
    setIntegerParam(PE_OffsetAvailable, 0);
    if (pOffsetBuffer_ != NULL)
      free(pOffsetBuffer_);
    pOffsetBuffer_ = NULL;
  } else {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s:  invalid binning %d x %d\n", 
      driverName, functionName, binX, binY);
  }
}


//_____________________________________________________________________________________________

/** Report status of the driver.
  * Prints details about the detector in us if details>0.
  * Prints information about all local and network detectors if details>1.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details Controls the level of detail in the report. */
void PerkinElmer::report(FILE *fp, int details)
{
  unsigned int uiPEResult;
  static const char *functionName = "report";

  // Get the hardware header information
  uiPEResult = Acquisition_GetHwHeaderInfoEx(hAcqDesc_, &cHwHeaderInfo_, &cHwHeaderInfoEx_);
  reportXISStatus(uiPEResult, functionName, 
    "Acquisition_GetHwHeaderInfoEx(hAcqDesc_=%p, &cHwHeaderInfo_=%p, &cHwHeaderInfoEx_=%p)\n",
    hAcqDesc_, &cHwHeaderInfo_, &cHwHeaderInfoEx_);
  if (uiPEResult != HIS_ALL_OK) {
    return;
  }
  fprintf(fp, "Perkin Elmer %s\n", this->portName);
  if (details > 0) {
    int nx, ny, dataType;
    getIntegerParam(ADSizeX, &nx);
    getIntegerParam(ADSizeY, &ny);
    getIntegerParam(NDDataType, &dataType);
    fprintf(fp, "  Number of rows:    %d\n", nx);
    fprintf(fp, "  Number of columns: %d\n", ny);
    fprintf(fp, "  Data type:         %d\n", dataType);
    fprintf(fp, "  Channel type:      %d\n", uiChannelType_);
    fprintf(fp, "  System ID:         %d\n", dwSystemID_);
    fprintf(fp, "  Channel number:    %d\n", iChannelNum_);
    fprintf(fp, "  Frames allocated:  %d\n", uiNumFrameBuffers_);
    fprintf(fp, "  Current rows:      %d\n", uiRows_);
    fprintf(fp, "  Current columns:   %d\n", uiColumns_);
    fprintf(fp, "  # device frames:   %d\n", uiDevFrames_);
    fprintf(fp, "  Device data type:  %d\n", uiDataType_);
    fprintf(fp, "  Device sort flags: %d\n", uiSortFlags_);
    fprintf(fp, "  IRQ enabled:       %d\n", bEnableIRQ_);
    fprintf(fp, "  Acquisition type:  %d\n", dwAcqType_);
    fprintf(fp, "  Sync mode:         %d\n", dwSyncMode_);
    if (cHwHeaderInfo_.dwHeaderID >= 14) {
      // This detector supports CHwHeaderInfoEx
      fprintf(fp, "  CHwHeaderInfoEx:\n");
      fprintf(fp, "    HeaderID:     %d\n", cHwHeaderInfoEx_.wHeaderID);
      fprintf(fp, "    PROMID:       %d\n", cHwHeaderInfoEx_.wPROMID);
      fprintf(fp, "    ResolutionX:  %d\n", cHwHeaderInfoEx_.wResolutionX);
      fprintf(fp, "    ResolutionY:  %d\n", cHwHeaderInfoEx_.wResolutionY);
      fprintf(fp, "    NrRows:       %d\n", cHwHeaderInfoEx_.wNrRows);
      fprintf(fp, "    NrColumns:    %d\n", cHwHeaderInfoEx_.wNrColumns);
      fprintf(fp, "    ZoomULRow:    %d\n", cHwHeaderInfoEx_.wZoomULRow);
      fprintf(fp, "    ZoomULColumn: %d\n", cHwHeaderInfoEx_.wZoomULColumn);
      fprintf(fp, "    ZoomBRColumn: %d\n", cHwHeaderInfoEx_.wZoomBRColumn);
      fprintf(fp, "    FrmNrRows:    %d\n", cHwHeaderInfoEx_.wFrmNrRows);
      fprintf(fp, "    FrmRowType:   %d\n", cHwHeaderInfoEx_.wFrmRowType);
      fprintf(fp, "    RowTime:      %d\n", cHwHeaderInfoEx_.wRowTime);
      fprintf(fp, "    Clock:        %d\n", cHwHeaderInfoEx_.wClock);
      fprintf(fp, "    DataSorting:  %d\n", cHwHeaderInfoEx_.wDataSorting);
      fprintf(fp, "    Timing:       %d\n", cHwHeaderInfoEx_.wTiming);
      fprintf(fp, "    Gain:         %d\n", cHwHeaderInfoEx_.wGain);
      fprintf(fp, "    LeakRows:     %d\n", cHwHeaderInfoEx_.wLeakRows);
      fprintf(fp, "    Access:       %d\n", cHwHeaderInfoEx_.wAccess);
      fprintf(fp, "    Bias:         %d\n", cHwHeaderInfoEx_.wBias);
      fprintf(fp, "    UgComp:       %d\n", cHwHeaderInfoEx_.wUgComp);
      fprintf(fp, "    Cameratype:   %d\n", cHwHeaderInfoEx_.wCameratype);
      fprintf(fp, "    FrameCnt:     %d\n", cHwHeaderInfoEx_.wFrameCnt);
      fprintf(fp, "    BinningMode:  %d\n", cHwHeaderInfoEx_.wBinningMode);
      fprintf(fp, "    RealInttime_milliSec: %d\n", cHwHeaderInfoEx_.wRealInttime_milliSec);
      fprintf(fp, "    RealInttime_microSec: %d\n", cHwHeaderInfoEx_.wRealInttime_microSec);
      fprintf(fp, "    Status:       %d\n", cHwHeaderInfoEx_.wStatus);
    } else {
      fprintf(fp, "  CHwHeaderInfo:\n");
      fprintf(fp, "    PROMID:       %d\n", cHwHeaderInfo_.dwPROMID);
      fprintf(fp, "    HeaderID:     %d\n", cHwHeaderInfo_.dwHeaderID);
      fprintf(fp, "    AddRow:       %d\n", cHwHeaderInfo_.bAddRow);
      fprintf(fp, "    PwrSave:      %d\n", cHwHeaderInfo_.bPwrSave);
      fprintf(fp, "    NrRows:       %d\n", cHwHeaderInfo_.dwNrRows);
      fprintf(fp, "    NrColumns:    %d\n", cHwHeaderInfo_.dwNrColumns);
      fprintf(fp, "    ZoomULRow:    %d\n", cHwHeaderInfo_.dwZoomULRow);
      fprintf(fp, "    ZoomULColumn: %d\n", cHwHeaderInfo_.dwZoomULColumn);
      fprintf(fp, "    ZoomBRRow:    %d\n", cHwHeaderInfo_.dwZoomBRRow);
      fprintf(fp, "    ZoomBRColumn: %d\n", cHwHeaderInfo_.dwZoomBRColumn);
      fprintf(fp, "    FrmNrRows:    %d\n", cHwHeaderInfo_.dwFrmNrRows);
      fprintf(fp, "    FrmRowType:   %d\n", cHwHeaderInfo_.dwFrmRowType);
      fprintf(fp, "    FrmFillRowIntervalls: %d\n", cHwHeaderInfo_.dwFrmFillRowIntervalls);
      fprintf(fp, "    NrOfFillingRows:     %d\n", cHwHeaderInfo_.dwNrOfFillingRows);
      fprintf(fp, "    DataType:     %d\n", cHwHeaderInfo_.dwDataType);
      fprintf(fp, "    DataSorting:  %d\n", cHwHeaderInfo_.dwDataSorting);
      fprintf(fp, "    Timing:       %d\n", cHwHeaderInfo_.dwTiming);
      fprintf(fp, "    Gain:         %d\n", cHwHeaderInfo_.dwGain);
      fprintf(fp, "    Offset:       %d\n", cHwHeaderInfo_.dwOffset);
      fprintf(fp, "    SyncMode:     %d\n", cHwHeaderInfo_.bSyncMode);
      fprintf(fp, "    Bias:         %d\n", cHwHeaderInfo_.dwBias);
      fprintf(fp, "    LeakRows:     %d\n", cHwHeaderInfo_.dwLeakRows);
    }
  }
  // Can't call reportSensors when acquiring, it crashes IOC
  int acquiring;
  getIntegerParam(ADAcquire, &acquiring);
  if (!acquiring && (details > 1)) reportSensors(fp, details);
  
  /* Invoke the base class method */
  ADDriver::report(fp, details);
}

//_____________________________________________________________________________________________
/** Report information about all local and network Perkin Elmer detectors */
void PerkinElmer::reportSensors(FILE *fp, int details)
{
  ACQDESCPOS Pos = 0;
  unsigned int uiPEResult;
  unsigned int uiChannelType, uiRows, uiColumns, uiDataType, uiSortFlags;
  int iChannelNum, iFrames;
  long numGbIFSensors=0;
  int i;
  GBIF_DEVICE_PARAM *pGbIFDeviceParam;
  BOOL bEnableIRQ;
  DWORD dwAcqType, dwSystemID, dwSyncMode, dwHwAccess;
  HACQDESC  hAcqDesc;
  
  fprintf(fp, "Total sensors in system: %d\n", uiNumSensors_);
  // Iterate through all this sensors and display sensor data
  do
  {
    if ((uiPEResult = Acquisition_GetNextSensor(&Pos, &hAcqDesc)) != HIS_ALL_OK) {
      fprintf(fp, "  Error: %d  GetNextSensor failed!\n", uiPEResult);
      return;
    }

    fprintf(fp, "  Sensor %d\n", Pos);

    // Ask for communication device type and its number
    if ((uiPEResult = Acquisition_GetCommChannel(hAcqDesc, &uiChannelType, &iChannelNum)) != HIS_ALL_OK) {
      fprintf(fp, "    Error: %d  GetCommChannel failed!\n", uiPEResult);
      return;
    }

    // Ask for data organization
    if ((uiPEResult=Acquisition_GetConfiguration(hAcqDesc, (unsigned int *) &iFrames, &uiRows, &uiColumns, &uiDataType,
        &uiSortFlags, &bEnableIRQ, &dwAcqType, &dwSystemID, &dwSyncMode, &dwHwAccess)) != HIS_ALL_OK) {
      fprintf(fp, "    Error: %d GetConfiguration failed!\n", uiPEResult);
      return;
    }

    fprintf(fp, "    Channel type:     %d\n", uiChannelType);
    fprintf(fp, "    Channel number:   %d\n", iChannelNum);
    fprintf(fp, "    Rows:             %d\n", uiRows);
    fprintf(fp, "    Columns:          %d\n", uiColumns);
    fprintf(fp, "    Frames:           %d\n", iFrames);
    fprintf(fp, "    Data type:        %d\n", uiDataType);
    fprintf(fp, "    Sort flags:       %d\n", uiSortFlags);
    fprintf(fp, "    Enable IRQ:       %d\n", bEnableIRQ);
    fprintf(fp, "    Acquisition type: %d\n", dwAcqType);
    fprintf(fp, "    System ID:        %d\n", dwSystemID);
    fprintf(fp, "    Sync mode:        %d\n", dwSyncMode);
    fprintf(fp, "    Hardware access:  %d\n", dwHwAccess);

  } while (Pos!=0);
  
  if ((uiPEResult = Acquisition_GbIF_GetDeviceCnt(&numGbIFSensors)) != HIS_ALL_OK) {
    fprintf(fp, "  Error: %d  GbIF_GetDeviceCnt failed!\n", uiPEResult);
    return;
  }
  if (numGbIFSensors <= 0) return;
  
  pGbIFDeviceParam = (GBIF_DEVICE_PARAM *)calloc(numGbIFSensors, sizeof(GBIF_DEVICE_PARAM));
  if ((uiPEResult = Acquisition_GbIF_GetDeviceList(pGbIFDeviceParam, numGbIFSensors)) != HIS_ALL_OK) {
    fprintf(fp, "  Error: %d  GbIF_GetDeviceList failed!\n", uiPEResult);
    return;
  }
  
  fprintf(fp, "Total GbIF sensors in system: %d\n", numGbIFSensors);
  for (i=0; i<numGbIFSensors; i++) {
    fprintf(fp, "  GbIF Sensor %d\n", i+1);
    fprintf(fp, "    MAC address:     %s\n", pGbIFDeviceParam[i].ucMacAddress);
    fprintf(fp, "    IP address:      %s\n", pGbIFDeviceParam[i].ucIP);
    fprintf(fp, "    Subnet mask:     %s\n", pGbIFDeviceParam[i].ucSubnetMask);
    fprintf(fp, "    Gateway:         %s\n", pGbIFDeviceParam[i].ucGateway);
    fprintf(fp, "    Adapter IP:      %s\n", pGbIFDeviceParam[i].ucAdapterIP);
    fprintf(fp, "    Adapter mask:    %s\n", pGbIFDeviceParam[i].ucAdapterMask);
    fprintf(fp, "    Boot options:    %d\n", pGbIFDeviceParam[i].dwIPCurrentBootOptions);
    fprintf(fp, "    Manufacturer:    %s\n", pGbIFDeviceParam[i].cManufacturerName);
    fprintf(fp, "    Model:           %s\n", pGbIFDeviceParam[i].cModelName);
    fprintf(fp, "    Firmware:        %s\n", pGbIFDeviceParam[i].cGBIFFirmwareVersion);
    fprintf(fp, "    Device name:     %s\n", pGbIFDeviceParam[i].cDeviceName);  
  }  
}

//_____________________________________________________________________________________________

/** callback function that is called by XISL every frame at end of data transfer */
static void CALLBACK endFrameCallbackC(HACQDESC hAcqDesc)
{
  PerkinElmer   *pPerkinElmer;
  unsigned int  uiStatus;
  DWORD         HISError;
  DWORD         FGError;
  static const char *functionName = "endFrameCallbackC";

#ifdef __X64
  uiStatus =  Acquisition_GetAcqData(hAcqDesc, (void **) &pPerkinElmer);
#else
  uiStatus =  Acquisition_GetAcqData(hAcqDesc, (DWORD *) &pPerkinElmer);
#endif
  if (uiStatus != 0) {
    Acquisition_GetErrorCode(hAcqDesc, &HISError, &FGError);
    printf("%s::%s error calling Acquisition_GetActFrame, return error=%d, HIS Error: %d, Frame Grabber Error: %d\n", 
      driverName, functionName, uiStatus, HISError, FGError);
    return;
  }
  pPerkinElmer->endFrameCallback(hAcqDesc);
}

//_____________________________________________________________________________________________
/** callback function that is called by XISL every frame at end of data transfer */
void PerkinElmer::endFrameCallback(HACQDESC hAcqDesc)
{
  unsigned int  uiStatus;
  DWORD         ActAcqFrame;
  DWORD         ActBuffFrame;
  unsigned int  currBuff;
  epicsUInt16   *pInput;
  NDArrayInfo   arrayInfo;
  int           arrayCounter;
  int           imageCounter;
  int           numImages;
  int           imageMode;
  int           triggerMode;
  int           offsetCounter;
  int           gainCounter;
  int           useOffset;
  int           useGain;
  int           usePixelCorrection;
  size_t        dims[2];
  int           acquiring;
  NDArray       *pImage;
  NDDataType_t  dataType;
  epicsTimeStamp currentTime;
  static const char *functionName = "endFrameCallback";
    
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s: entry ...\n",
    driverName, functionName);

  lock();
  getIntegerParam(PE_UseOffset, &useOffset);
  getIntegerParam(PE_UseGain, &useGain);
  getIntegerParam(PE_UsePixelCorrection, &usePixelCorrection);
  getIntegerParam(ADTriggerMode, &triggerMode);

  switch (iAcqMode_) {
    case PE_ACQUIRE_OFFSET:
      getIntegerParam(PE_CurrentOffsetFrame, &offsetCounter);
      offsetCounter++;
      setIntegerParam(PE_CurrentOffsetFrame, offsetCounter);
      pInput = pOffsetBuffer_;
      dataType = NDUInt16;
      break;

    case PE_ACQUIRE_GAIN:
      getIntegerParam(PE_CurrentGainFrame, &gainCounter);
      gainCounter++;
      setIntegerParam(PE_CurrentGainFrame, gainCounter);
      pInput = (epicsUInt16 *)pGainBuffer_;
      dataType = NDInt32;
      break;

    case PE_ACQUIRE_ACQUISITION:
      /** Find offset into secondary frame buffer */
      uiStatus =  Acquisition_GetActFrame(hAcqDesc, &ActAcqFrame, &ActBuffFrame);
      reportXISStatus(uiStatus, functionName, 
        "Acquisition_GetActFrame(hAcqDesc=%p, ActAcqFrame=%d, ActBuffFrame=%d)\n",
        hAcqDesc, ActAcqFrame, ActBuffFrame);
      getIntegerParam(ADImageMode, &imageMode);
      getIntegerParam(ADNumImages, &numImages);
      getIntegerParam(ADNumImagesCounter, &imageCounter);
      imageCounter++;
      setIntegerParam(ADNumImagesCounter, imageCounter);
      asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: ActAcqFrame = %d, ActBuffFrame = %d\n", 
        driverName, functionName, ActAcqFrame, ActBuffFrame);
      setIntegerParam(PE_ImageNumber, ActBuffFrame);
      setIntegerParam(PE_FrameBufferIndex, ActAcqFrame);
      asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: uiNumBuffersInUse_ = %d\n", 
        driverName, functionName, uiNumBuffersInUse_);
      if (imageMode == PEImageAverage) {
        // We don't want to do anything if we are in average mode except if
        // we have been called from the endAcqCallback, in which case ADAcquire will be 0
        getIntegerParam(ADAcquire, &acquiring);
        if (acquiring) goto done;
        // We force it to use the first buffer, even though Acquisition_GetActFrame 
        // says ActBuffFrame is something else
        ActBuffFrame = 1;
      }
      currBuff = (ActBuffFrame - 1) % uiNumBuffersInUse_;
      pInput = &pAcqBuffer_[uiColumns_ * uiRows_ * currBuff];
      dataType = NDUInt16;
      asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: currBuff = %d, pInput = %p\n", 
        driverName, functionName, currBuff, pInput);

      /** Correct for detector offset and gain as necessary */
      if ((useOffset) && (pOffsetBuffer_ != NULL))
      {
        if ((useGain) && (pGainBuffer_ != NULL)) 
        {
          uiStatus = Acquisition_DoOffsetGainCorrection(pInput, pInput, pOffsetBuffer_, pGainBuffer_, uiRows_ * uiColumns_);
          reportXISStatus(uiStatus, functionName, 
            "Acquisition_DoOffsetGainCorrection(pInput=%p, pInput=%p, pOffsetBuffer_=%p, pGainBuffer_=%p, numPixels=%d)\n",
            pInput, pInput, pOffsetBuffer_, pGainBuffer_, uiRows_ * uiColumns_);
        } else {
          uiStatus = Acquisition_DoOffsetCorrection(pInput, pInput, pOffsetBuffer_, uiRows_ * uiColumns_);
          reportXISStatus(uiStatus, functionName, 
            "Acquisition_DoOffsetCorrection(pInput=%p, pInput=%p, pOffsetBuffer_=%p, numPixels=%d)\n",
            pInput, pInput, pOffsetBuffer_, uiRows_ * uiColumns_);
        }
      }

      /** Correct for dead pixels as necessary */
      if ((usePixelCorrection) && (pPixelCorrectionList_ != NULL))
      {
        uiStatus = Acquisition_DoPixelCorrection(pInput, pPixelCorrectionList_);
        reportXISStatus(uiStatus, functionName, 
          "Acquisition_DoPixelCorrection(pInput=%p, pPixelCorrectionList_=%p)\n",
          pInput, pPixelCorrectionList_);
      }

      if ((imageMode == PEImageMultiple) && (imageCounter >= numImages))
        acquireStop();
      break;
  }

  /* Update the image */
  /* We save the most recent image buffer so it can be used in the read() function.
   * Now release it before getting a new version. */
  if (this->pArrays[0])
      this->pArrays[0]->release();
  /* Allocate the array */
  dims[0] = uiColumns_;
  dims[1] = uiRows_;
  this->pArrays[0] = pNDArrayPool->alloc(2, dims, dataType, 0, NULL);
  if (this->pArrays[0] == NULL) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: error allocating buffer\n",
      driverName, functionName);
    unlock();
    return;
  }
  pImage = this->pArrays[0];
  pImage->getInfo(&arrayInfo);
  // Copy the data from the input to the output
  memcpy(pImage->pData, pInput, arrayInfo.totalBytes);

  setIntegerParam(NDArraySize,  (int)arrayInfo.totalBytes);
  setIntegerParam(NDArraySizeX, (int)pImage->dims[0].size);
  setIntegerParam(NDArraySizeY, (int)pImage->dims[1].size);


  /* Put the frame number and time stamp into the buffer */
  getIntegerParam(NDArrayCounter, &arrayCounter);
  arrayCounter++;
  setIntegerParam(NDArrayCounter, arrayCounter);
  pImage->uniqueId = arrayCounter;
  epicsTimeGetCurrent(&currentTime);
  pImage->timeStamp = currentTime.secPastEpoch + currentTime.nsec / 1.e9;
  updateTimeStamp(&pImage->epicsTS);

  /* Get any attributes that have been defined for this driver */
  getAttributes(pImage->pAttributeList);

  /* Call the NDArray callback */
  /* Must release the lock here, or we can get into a deadlock, because we can
   * block on the plugin lock, and the plugin can be calling us */
  unlock();
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s: calling imageData callback\n", 
    driverName, functionName);
  doCallbacksGenericPointer(pImage, NDArrayData, 0);
  lock();

  done:
  
  // Do software trigger if required
  if (doSoftwareTriggers_) {
    doSoftwareTrigger();
  }

  // Do callbacks on parameters
  callParamCallbacks();

  unlock();
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s: exit\n",
    driverName, functionName);
}

//_____________________________________________________________________________________________

/** callback function that is called by XISL at end of acquisition */
static void CALLBACK endAcqCallbackC(HACQDESC hAcqDesc)
{
  PerkinElmer   *pPerkinElmer;
  unsigned int  uiStatus;
  DWORD         HISError;
  DWORD         FGError;
  static const char *functionName = "endAcqCallbackC";

#ifdef __X64
  uiStatus =  Acquisition_GetAcqData(hAcqDesc, (void **) &pPerkinElmer);
#else
  uiStatus =  Acquisition_GetAcqData(hAcqDesc, (DWORD *) &pPerkinElmer);
#endif
  if (uiStatus != 0) {
    Acquisition_GetErrorCode(hAcqDesc, &HISError, &FGError);
    printf("%s::%s error calling Acquisition_GetActFrame, return error=%d, HIS Error: %d, Frame Grabber Error: %d\n", 
      driverName, functionName, uiStatus, HISError, FGError);
    return;
  }
  pPerkinElmer->endAcqCallback(hAcqDesc);
}

//_____________________________________________________________________________________________

/** callback function that is called by XISL at end of acquisition */
void PerkinElmer::endAcqCallback(HACQDESC hAcqDesc)
{
  int imageMode;
  static const char *functionName = "endAcqCallback";

  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s: entry...\n",
    driverName, functionName);

  lock();
  setIntegerParam(ADStatus, ADStatusIdle);
  setIntegerParam(ADAcquire, 0);
  switch(iAcqMode_) {
    case PE_ACQUIRE_OFFSET: 
    setIntegerParam(PE_AcquireOffset, 0);
      /* raise a flag to the user if offset data is available */
      if (pOffsetBuffer_ != NULL) {
        setIntegerParam(PE_OffsetAvailable, 1);
        /* Subtract the offset constant from the offset
         * Doing this here is much more efficient that doing it when correcting the offset
         * on every frame */
        int offset;
        unsigned int i;
        getIntegerParam(PE_OffsetConstant, &offset);
        if (offset != 0) {
          for (i=0; i<uiRows_*uiColumns_; i++) {
            pOffsetBuffer_[i] -= offset;
          }
        }
      }
      break;

  case PE_ACQUIRE_GAIN: 
    setIntegerParam(PE_AcquireGain, 0);
    /* raise a flag to the user if gain data is available */
    if (pGainBuffer_ != NULL) {
      setIntegerParam(PE_GainAvailable, 1);
    }
    setShutter(ADShutterClosed);
    break;
    
  case PE_ACQUIRE_ACQUISITION:
    getIntegerParam(ADImageMode, &imageMode);
    if (imageMode == PEImageAverage) {
      endFrameCallback(hAcqDesc);
    }
    setShutter(ADShutterClosed);
    break;
  
  }
  
  callParamCallbacks();
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s: exit\n",
    driverName, functionName);
  unlock();
}

//_____________________________________________________________________________________________

/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including ADAcquire, ADBinX, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus PerkinElmer::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  int adstatus;
  int status = asynSuccess;
  unsigned int uiPEResult;
  static const char *functionName = "writeInt32";

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setIntegerParam(function, value);

  getIntegerParam(ADStatus, &adstatus);

  /* For a real detector this is where the parameter is sent to the hardware */
  if (function == ADAcquire) {

    // Start acquisition
    if (value && (adstatus == ADStatusIdle) )
    {
      acquireNormalImage();
    }

    // Stop acquisition
    if (!value && (adstatus != ADStatusIdle))
    {
      acquireStop();
    }
  }
  else if ((function == ADBinX) ||
           (function == ADBinY)) {
    acquireSettingsChanged_ = true;
    if ( adstatus == ADStatusIdle ) {
      setBinning();
    }    
  }
  else if ((function == ADImageMode)   ||
           (function == ADTriggerMode) ||
           (function == ADNumImages)   ||
           (function == PE_SyncMode)) {
    acquireSettingsChanged_ = true;
  }
  else if (function == PE_Initialize) {
    if ( adstatus == ADStatusIdle ) {
      initializeDetector();
    }
  }
  else if (function == PE_AcquireOffset) {
    if ( adstatus == ADStatusIdle ) {
      acquireOffsetImage();
    }
  }
  else if (function ==  PE_AcquireGain) {
    if ( adstatus == ADStatusIdle ) {
      acquireGainImage();
    }
  }
  else if (function == PE_Trigger) {
    doSoftwareTrigger();
  }
  else if (function == PE_Gain) {
    if ( adstatus == ADStatusIdle ) {
      //  set detector gain
      uiPEResult = Acquisition_SetCameraGain(hAcqDesc_, value);
      reportXISStatus(uiPEResult, functionName, "Acquisition_SetCameraGain(hAcqDesc_=%p, value=%d)\n", hAcqDesc_, value);
    }
  }
  else if (function == PE_LoadGainFile) {
    loadGainFile();
  }
  else if (function == PE_SaveGainFile) {
    saveGainFile();
  }
  else if (function == PE_LoadPixelCorrectionFile) {
    loadPixelCorrectionFile();
  }

  else {
    /* If this parameter belongs to a base class call its method */
    if (function < PE_FIRST_PARAM) {
      status = ADDriver::writeInt32(pasynUser, value);
    }
  }

  /* Do callbacks so higher layers see any changes */
  callParamCallbacks();

  if (status)
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
      "%s:%s: error, status=%d function=%d, value=%d\n",
      driverName, functionName, status, function, value);
  else
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
      "%s:%s: function=%d, value=%d\n",
      driverName, functionName, function, value);

  return (asynStatus) status;
}

//_____________________________________________________________________________________________

/** Called when asyn clients call pasynFloat64->write().
  * This function performs actions for some parameters.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks.
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus PerkinElmer::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  static const char *functionName = "writeFloat64";

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setDoubleParam(function, value);

  if (function == ADAcquireTime) {
    acquireSettingsChanged_ = true;
  }
  else {
    /* If this parameter belongs to a base class call its method */
    if (function < PE_FIRST_PARAM) {
      status = ADDriver::writeFloat64(pasynUser, value);
    }
  }

  /* Do callbacks so higher layers see any changes */
  callParamCallbacks();

  if (status)
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
        "%s:%s: error, status=%d function=%d, value=%f\n",
        driverName, functionName, status, function, value);
  else
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
        "%s:%s: function=%d, value=%f\n",
        driverName, functionName, function, value);
  return status;
}


//_____________________________________________________________________________________________

/** Sets up acquisition for normal, offset, or gain acquisition */
void PerkinElmer::acquireSetup(void)
{
  int     imageMode;
  int     triggerMode;
  int     syncMode;
  DWORD   dwTriggerMode;
  double  acquireTime;
  DWORD   dwDwellTime;
  unsigned int delayTime;
  unsigned int uiPEResult;
  static const char *functionName = "acquireStart";

  // If no acquire settings have changed then return immediately.  
  // This is done for performance reasons, since the calls in this function can take over 0.5 seconds
  // so we don't want to do them if it is not necessary.
  if (!acquireSettingsChanged_) return;
  acquireSettingsChanged_ = false;
  
  //get some information
  getIntegerParam(ADTriggerMode, &triggerMode);
  getIntegerParam(ADImageMode,   &imageMode);
  getIntegerParam(PE_SyncMode,   &syncMode);
  getDoubleParam(ADAcquireTime,  &acquireTime);

  // Set the trigger mode
  switch (triggerMode) {
    case PE_FREE_RUNNING: {
      dwTriggerMode = HIS_SYNCMODE_FREE_RUNNING;
      break;
    }
    case PE_EXTERNAL_TRIGGER: {
      dwTriggerMode = HIS_SYNCMODE_EXTERNAL_TRIGGER;
      break;
    }
    case PE_INTERNAL_TRIGGER: {
      dwTriggerMode = HIS_SYNCMODE_INTERNAL_TIMER;
      break;
    }
    case PE_SOFT_TRIGGER: {
      dwTriggerMode = HIS_SYNCMODE_SOFT_TRIGGER;
      break;
    }
  }
  
  uiPEResult = Acquisition_SetFrameSyncMode(hAcqDesc_, dwTriggerMode);
  reportXISStatus(uiPEResult, functionName, "Acquisition_SetFrameSyncMode(hAcqDesc_=%p, dwTriggerMode=%d)\n",
   hAcqDesc_, dwTriggerMode);

  // Set the camera trigger mode
  uiPEResult = Acquisition_SetCameraTriggerMode(hAcqDesc_, syncMode);
  reportXISStatus(uiPEResult, functionName, "Acquisition_SetCameraTriggerMode(hAcqDesc_=%p, mode=%d)\n", hAcqDesc_, syncMode);

  if ((syncMode == PE_SYNC_DDD_CLEAR) ||
      (syncMode == PE_SYNC_DDD_NOCLEAR)) {
    delayTime = (int)(acquireTime*1000. + 0.5);
    uiPEResult = Acquisition_SetFrameSyncTimeMode(hAcqDesc_, 0, delayTime);
    reportXISStatus(uiPEResult, functionName, "Acquisition_SetFrameSyncTimeMode(hAcqDesc_=%p, uiMode=%d, delayTime=%d)\n", 
      hAcqDesc_, 0, delayTime);
  } else {
    dwDwellTime = (DWORD) (acquireTime * 1000000);
    uiPEResult = Acquisition_SetTimerSync(hAcqDesc_, &dwDwellTime);
    reportXISStatus(uiPEResult, functionName, "Acquisition_SetTimerSync(hAcqDesc_=%p, dwDwellTime=%d)\n", hAcqDesc_, dwDwellTime);
    // The actual acquire time could be different from requested
    acquireTime = dwDwellTime/1000000.;
    setDoubleParam(ADAcquireTime, acquireTime);
  }
  // If we are in a DDD mode we need to issue software triggers
  doSoftwareTriggers_ = (((syncMode == PE_SYNC_DDD_CLEAR) || 
                          (syncMode == PE_SYNC_DDD_NOCLEAR)) &&
                         (triggerMode == PE_INTERNAL_TRIGGER));
}

void PerkinElmer::doSoftwareTrigger()
{
  unsigned int uiPEResult;
  static const char *functionName = "doSoftwareTrigger";

  uiPEResult = Acquisition_SetFrameSync(hAcqDesc_);
  reportXISStatus(uiPEResult, functionName, "Acquisition_SetFrameSync(hAcqDesc_=%p)\n", hAcqDesc_);
}

//_____________________________________________________________________________________________

/** Acquires a normal image */
void PerkinElmer::acquireNormalImage (void)
{
  int imageMode;
  int skipFrames;
  int numFramesToSkip;
  UINT sequenceOptions;
  int numFrames;
  unsigned int uiPEResult;
  const char *functionName = "acquireNormalImage";

  //get some information
  getIntegerParam(ADImageMode, &imageMode);

  acquireSetup();

  // Set the number of frames
  switch (imageMode) {
    case PEImageSingle:
      numFrames = 1;
      sequenceOptions = HIS_SEQ_ONE_BUFFER;
      break;
    case PEImageMultiple:
    case PEImageContinuous:
      numFrames = uiNumFrameBuffers_;
      sequenceOptions = HIS_SEQ_CONTINUOUS;
      break;
    case PEImageAverage:
      getIntegerParam(ADNumImages, &numFrames);
      sequenceOptions = HIS_SEQ_AVERAGE;
      break;
  }

  uiNumBuffersInUse_ = numFrames;
  uiPEResult = Acquisition_DefineDestBuffers(hAcqDesc_, pAcqBuffer_, numFrames, uiRows_, uiColumns_);
  reportXISStatus(uiPEResult, functionName, 
    "Acquisition_DefineDestBuffers(hAcqDesc_=%p, pAcqBuffer_=%p, numFrames=%d, uiRows_=%u, uiColumns_=%u)\n",
    hAcqDesc_, pAcqBuffer_, numFrames, uiRows_, uiColumns_);
  
  getIntegerParam(PE_SkipFrames, &skipFrames);
  getIntegerParam(PE_NumFramesToSkip, &numFramesToSkip);
  if (skipFrames == 0) numFramesToSkip = 0;

  iAcqMode_ = PE_ACQUIRE_ACQUISITION;
  setIntegerParam(ADNumImagesCounter, 0);
  setIntegerParam(ADStatus, ADStatusAcquire);

  uiPEResult = Acquisition_ResetFrameCnt(hAcqDesc_);
  reportXISStatus(uiPEResult, functionName,  "Acquisition_ResetFrameCnt(hAcqDesc_=%p)\n", hAcqDesc_);
  Acquisition_SetReady(hAcqDesc_, 1);
  reportXISStatus(uiPEResult, functionName,  "Acquisition_SetReady(hAcqDesc_=%p, ready=%d)\n", hAcqDesc_, 1);
  setShutter(ADShutterOpen);

  uiPEResult = Acquisition_Acquire_Image(hAcqDesc_, numFrames, numFramesToSkip,
                                         sequenceOptions, NULL, NULL, NULL);
  reportXISStatus(uiPEResult, functionName,  
    "Acquisition_Acquire_Image(hAcqDesc_=%p, numFrames=%d, numFramesToSkip=%d, " 
    "sequenceOptions=%d, pOffsetData=%p, pGainData=%p, pPixelData=%p)\n", 
    hAcqDesc_, numFrames, numFramesToSkip, sequenceOptions,  NULL, NULL, NULL);
  
  if (doSoftwareTriggers_) {
    doSoftwareTrigger();
  }
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s: Acquisition started...\n",
    driverName, functionName);
}

//_____________________________________________________________________________________________

/** Acquires an offset image */
void PerkinElmer::acquireOffsetImage (void)
{
  int iFrames;
  int status = asynSuccess;
  unsigned int uiPEResult;
  const char   *functionName = "acquireOffsetImage";

  getIntegerParam(PE_NumOffsetFrames, &iFrames);
  setIntegerParam(ADAcquire, 1);
  setIntegerParam(ADStatus, ADStatusAcquire);

  acquireSetup();

  if (pOffsetBuffer_ != NULL)
    free (pOffsetBuffer_);
  pOffsetBuffer_ = (epicsUInt16 *) malloc(sizeof(epicsUInt16) * uiRows_ * uiColumns_);
  if (pOffsetBuffer_ == NULL)
  {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: Error:  Memory allocation failed for offset buffer!\n",
      driverName, functionName);
    return;
  }

  memset (pOffsetBuffer_, 0, sizeof(epicsUInt16) * uiRows_ * uiColumns_);

  iAcqMode_ = PE_ACQUIRE_OFFSET;
  setIntegerParam(PE_CurrentOffsetFrame, 0);
  setIntegerParam(PE_OffsetAvailable, 0);
  
  // Make sure the shutter is closed
  setShutter(ADShutterClosed);

  uiPEResult = Acquisition_Acquire_OffsetImage(hAcqDesc_, pOffsetBuffer_, uiRows_, uiColumns_, iFrames);
  reportXISStatus(uiPEResult, functionName, 
    "Acquisition_Acquire_OffsetImage(hAcqDesc_=%p, pOffsetBuffer_=%p, uiRows_=%u, uiColumns_=%u, iFrames=%d)\n", 
    hAcqDesc_, pOffsetBuffer_, uiRows_, uiColumns_, iFrames);

  if (doSoftwareTriggers_) {
    doSoftwareTrigger();
  }
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s: Offset acquisition started...\n",
    driverName, functionName);

}

//_____________________________________________________________________________________________

/** Acquires a gain image */
void PerkinElmer::acquireGainImage(void)
{
  int iFrames;
  unsigned int uiPEResult;
  const char  *functionName = "acquireGainImage";

  getIntegerParam(PE_NumGainFrames, &iFrames);
  setIntegerParam(ADAcquire, 1);
  setIntegerParam(ADStatus, ADStatusAcquire);

  acquireSetup();
  
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s: Frames: %d, Rows: %d, Columns: %d\n",
    driverName, functionName, iFrames, uiRows_, uiColumns_);

  if (pGainBuffer_ != NULL)
  free (pGainBuffer_);
  pGainBuffer_ = (DWORD *) malloc(uiRows_ * uiColumns_ * sizeof(DWORD));
  if (pGainBuffer_ == NULL)
  {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: Error:  Memory allocation failed for gain buffer!\n",
      driverName, functionName);
    return;
  }

  iAcqMode_ = PE_ACQUIRE_GAIN;
  setIntegerParam(PE_CurrentGainFrame, 0);
  setIntegerParam(PE_GainAvailable, 0);
  setShutter(ADShutterOpen);

  uiPEResult = Acquisition_Acquire_GainImage(hAcqDesc_, pOffsetBuffer_, pGainBuffer_, uiRows_, uiColumns_, iFrames);
  reportXISStatus(uiPEResult, functionName, 
    "Acquisition_Acquire_GainImage(hAcqDesc_=%p, pOffsetBuffer_=%p, pGainBuffer_=%p, uiRows_=%u, uiColumns_=%u, iFrames=%d)\n", 
    hAcqDesc_, pOffsetBuffer_, pGainBuffer_, uiRows_, uiColumns_, iFrames);

  if (doSoftwareTriggers_) {
    doSoftwareTrigger();
  }
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s: Gain acquisition started...\n",
    driverName, functionName);

}

//_____________________________________________________________________________________________
/** Stops acquisition
  * acquireStop cannot directly call Acquisition_Abort because acquireStop is called from the
  * endFrameCallback function which is running in an XISL thread.  When it calls Acquisition_Abort
  * that appears to result in a deadlock, presumably because there is a second XISL thread that
  * does the abort, and it blocks waiting for the first XISL thread (deadlock). So instead
  * we run a thread whose only job is to call Acquisition_Abort on receipt of an EPICS event. */

void PerkinElmer::acquireStop(void)
{
  static const char *functionName = "acquireStop";

  epicsEventSignal(acquireStopEvent_);
}

static void acquireStopTaskC(void *drvPvt)
{
    PerkinElmer *pPvt = (PerkinElmer *)drvPvt;

    pPvt->acquireStopTask();
}

void PerkinElmer::acquireStopTask(void)
{
  unsigned int uiPEResult;
  static const char *functionName = "acquireStopTask";
  
  lock();
  while (1) {
    unlock();
    epicsEventWait(acquireStopEvent_);
    uiPEResult = Acquisition_Abort(hAcqDesc_);
    reportXISStatus(uiPEResult, functionName, "Acquisition_Abort(hAcqDesc_=%p)\n", hAcqDesc_);
    lock();
    setShutter(ADShutterClosed);
    setIntegerParam(ADStatus, ADStatusIdle);
    setIntegerParam(ADAcquire, 0);
    callParamCallbacks();
  }
}
    
//_____________________________________________________________________________________________

/** Saves a gain file */
asynStatus PerkinElmer::saveGainFile(void)
{
  int iSizeX;
  int iSizeY;
  int iByteDepth;
  int status = asynSuccess;
  char gainPath[256];
  char gainFile[256];
  FILE  *pOutputFile;
  static const char *functionName = "saveGainFile";

  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s:, Saving correction files...\n",
    driverName, functionName);

  status |= getStringParam(PE_CorrectionsDirectory, sizeof(gainPath), gainPath);
  status |= getStringParam(PE_GainFile, sizeof(gainFile), gainFile);
  strcat(gainPath, gainFile);
  status |= getIntegerParam(NDArraySizeX, &iSizeX);
  status |= getIntegerParam(NDArraySizeY, &iSizeY);

  if (pGainBuffer_ == NULL) return asynError;

  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s:, saving gain file: %s\n",
    driverName, functionName, gainPath);

  pOutputFile = fopen (gainPath, "wb");

  if (pOutputFile == NULL) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: Error opening gain file %s\n", 
      driverName, functionName, gainFile);
    return asynError;
  }
  iByteDepth = sizeof (DWORD);

  fwrite ((void *) &iSizeX, sizeof (int), 1, pOutputFile);
  fwrite ((void *) &iSizeY, sizeof (int), 1, pOutputFile);
  fwrite ((void *) &iByteDepth, sizeof (int), 1, pOutputFile);
  if (ferror (pOutputFile)) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: Failed to write file header for file %s\n", 
      driverName, functionName, gainFile);
    fclose (pOutputFile);
    return asynError;
  }

  fwrite (pGainBuffer_, iByteDepth, iSizeX*iSizeY, pOutputFile);
  if (ferror (pOutputFile)) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: Failed to write data for file %s\n", 
      driverName, functionName, gainFile);
    fclose (pOutputFile);
    return asynError;
  }

  fclose (pOutputFile);

  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s:, Gain file %s saved.\n",
    driverName, functionName, gainFile);
  return asynSuccess;
}


//_____________________________________________________________________________________________

/** Loads a gain file */
asynStatus PerkinElmer::loadGainFile (void)
{
  int status = asynSuccess;
  char gainPath[256];
  char gainFile[256];
  int iSizeX, iSizeY, iByteDepth;
  FILE  *pInputFile;
  struct stat stat_buffer;
  static const char *functionName = "loadGainFile";

  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s: Loading gain file...\n",
    driverName, functionName);

  status |= getStringParam(PE_CorrectionsDirectory, sizeof(gainPath), gainPath);
  status |= getStringParam(PE_CorrectionsDirectory, sizeof(gainFile), gainFile);
  strcat(gainPath, gainFile);

  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s:, Gain correction file name: %s\n",
    driverName, functionName, gainPath);
  if ((stat (gainPath, &stat_buffer) != 0)|| (stat_buffer.st_mode & S_IFREG) == 0) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: Failed to find gain correction file %s\n", 
      driverName, functionName, gainPath);
    return asynError;
  }
  if (pGainBuffer_ != NULL)
    free (pGainBuffer_);

  pInputFile = fopen (gainPath, "rb");
  if (pInputFile == NULL) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: Failed to open gain correction file %s\n", 
      driverName, functionName, gainPath);
    return asynError;
  }
  fread (&iSizeX, sizeof (int), 1, pInputFile);
  fread (&iSizeY, sizeof (int), 1, pInputFile);
  fread (&iByteDepth, sizeof (int), 1, pInputFile);
  if (ferror (pInputFile)) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: Failed to read file header for gain correction file %s\n", 
      driverName, functionName, gainPath);
    return asynError;
  }
  pGainBuffer_ = (DWORD *) malloc (iSizeX * iSizeY * iByteDepth);
  fread (pGainBuffer_, iByteDepth, iSizeX * iSizeY, pInputFile);
  if (ferror (pInputFile)) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: Failed to read data for gain correction file %s\n", 
      driverName, functionName, gainPath);
    return asynError;
  }

  fclose (pInputFile);

  status |= setIntegerParam(PE_GainAvailable, 1);
  callParamCallbacks();

  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s:, Gain file %s loaded\n",
    driverName, functionName, gainPath);
  return asynSuccess;

}
//_____________________________________________________________________________________________

/** Loads a pixel correction file */
asynStatus PerkinElmer::loadPixelCorrectionFile()
{
  FILE                *pInputFile;
  WinHeaderType       file_header;
  WinImageHeaderType  image_header;
  int                 iBufferSize;
  int                 iCorrectionMapSize;
  unsigned int        uiStatus;
  char                pixelCorrectionFile[256];
  char                pixelCorrectionPath[256];
  struct              stat stat_buffer;
  static const char   *functionName = "readPixelCorrectionFile";
  
  setIntegerParam(PE_PixelCorrectionAvailable, 0);

  getStringParam(PE_PixelCorrectionFile, sizeof(pixelCorrectionFile), pixelCorrectionFile);
  getStringParam(PE_CorrectionsDirectory, sizeof(pixelCorrectionPath), pixelCorrectionPath);
  strcat(pixelCorrectionPath, pixelCorrectionFile);
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s:, Pixel correction file name: %s\n",
    driverName, functionName, pixelCorrectionPath);
  if ((stat (pixelCorrectionPath, &stat_buffer) != 0) || (stat_buffer.st_mode & S_IFREG) == 0) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: error opening pixel correction file %s\n",
      driverName, functionName, pixelCorrectionPath);
    return asynError;
  }
  
  pInputFile = fopen(pixelCorrectionPath, "r");

  if (pInputFile == NULL) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
    "%s:%s: Failed to open file %s\n", 
    driverName, functionName, pixelCorrectionPath);
    return asynError;
  }

  //read file header
  fread((void *) &file_header, 68, 1, pInputFile);
  if (ferror(pInputFile)) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: Failed to read file header from file %s\n", 
      driverName, functionName, pixelCorrectionPath);
    return asynError;
  }


  //read image header
  fread((void *) &image_header, 32, 1, pInputFile);
  if (ferror (pInputFile)) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: Failed to read image header from file %s\n", 
      driverName, functionName, pixelCorrectionPath);
    return asynError;
  }

  //read bad pixel map
  if (pBadPixelMap_ != NULL)
    free (pBadPixelMap_);
  iBufferSize = file_header.ULY * file_header.BRX * sizeof (epicsUInt16);
  pBadPixelMap_ = (epicsUInt16 *) malloc (iBufferSize);
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s: buffer size: %d, pBadPixelMap: %d\n", 
    driverName, functionName, iBufferSize, pBadPixelMap_);
  if (pBadPixelMap_ == NULL) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: Failed to allocate bad pixel map buffer\n", 
      driverName, functionName);
    return asynError;
  }
  fread ((void *) pBadPixelMap_, iBufferSize, 1, pInputFile);
  if (ferror (pInputFile)) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: Failed to read bad pixel map from file %s\n", 
      driverName, functionName, pixelCorrectionPath);
    free (pBadPixelMap_);
    pBadPixelMap_ = NULL;
    return asynError;
  }

  fclose (pInputFile);

  int counter = 0;
  for (int loop=0;loop<file_header.ULY * file_header.BRX;loop++) {
    if (pBadPixelMap_[loop] == 65535)
      counter++;
  }
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s: Bad pixel map read in: %d bad pixels found\n", 
    driverName, functionName, counter);

  //first call with correction list = NULL returns size of buffer to allocate
  //second time gets the correction list
  uiStatus = Acquisition_CreatePixelMap (pBadPixelMap_, file_header.ULY, file_header.BRX, NULL, &iCorrectionMapSize);
  reportXISStatus(uiStatus, functionName, 
    "Acquisition_CreatePixelMap (pBadPixelMap_=%p, nDataRows=%d, nDataColumns=%d, pCorrList=%p, iCorrectionMapSize=%d)\n", 
    pBadPixelMap_, file_header.ULY, file_header.BRX, NULL, iCorrectionMapSize);
  pPixelCorrectionList_ = (int *) malloc (iCorrectionMapSize);
  uiStatus = Acquisition_CreatePixelMap (pBadPixelMap_, file_header.ULY, file_header.BRX, pPixelCorrectionList_, &iCorrectionMapSize);
  reportXISStatus(uiStatus, functionName, 
    "Acquisition_CreatePixelMap (pBadPixelMap_=%p, nDataRows=%d, nDataColumns=%d, pCorrList=%p, iCorrectionMapSize=%d)\n", 
    pBadPixelMap_, file_header.ULY, file_header.BRX, pPixelCorrectionList_, &iCorrectionMapSize);

  free (pBadPixelMap_);
  pBadPixelMap_ = NULL;

  setIntegerParam(PE_PixelCorrectionAvailable, 1);
  return asynSuccess;
}


void PerkinElmer::reportXISStatus(int returnCode, const char *functionName, const char *formatString, ...)
{
  va_list argp;
  int traceMask;
  char tempString[256];

  traceMask = pasynTrace->getTraceMask(pasynUserSelf);
  va_start(argp, formatString);
  if (returnCode == HIS_ALL_OK) {
    if (traceMask & ASYN_TRACEIO_DRIVER) {
      epicsSnprintf(tempString, sizeof(tempString), "%s::%s, called %s", driverName, functionName, formatString);
      pasynTrace->vprint(pasynUserSelf, ASYN_TRACEIO_DRIVER, tempString, argp);
    }
  } else {
    if (traceMask & ASYN_TRACE_ERROR) {
      epicsSnprintf(tempString, sizeof(tempString), "%s::%s, error=%d calling %s", driverName, functionName, returnCode, formatString);
      pasynTrace->vprint(pasynUserSelf, ASYN_TRACE_ERROR, tempString, argp);
    }
  }
}

/* Code for iocsh registration */

/* PerkinElmerConfig */
static const iocshArg PerkinElmerConfigArg0 = {"Port name", iocshArgString};
static const iocshArg PerkinElmerConfigArg1 = {"ID type", iocshArgInt};
static const iocshArg PerkinElmerConfigArg2 = {"ID value", iocshArgString};
static const iocshArg PerkinElmerConfigArg3 = {"maxBuffers", iocshArgInt};
static const iocshArg PerkinElmerConfigArg4 = {"maxMemory", iocshArgInt};
static const iocshArg PerkinElmerConfigArg5 = {"priority", iocshArgInt};
static const iocshArg PerkinElmerConfigArg6 = {"stackSize", iocshArgInt};
static const iocshArg * const PerkinElmerConfigArgs[] =  {&PerkinElmerConfigArg0,
                                                          &PerkinElmerConfigArg1,
                                                          &PerkinElmerConfigArg2,
                                                          &PerkinElmerConfigArg3,
                                                          &PerkinElmerConfigArg4,
                                                          &PerkinElmerConfigArg5,
                                                          &PerkinElmerConfigArg6};
static const iocshFuncDef configPerkinElmer = {"PerkinElmerConfig", 7, PerkinElmerConfigArgs};
static void configPerkinElmerCallFunc(const iocshArgBuf *args)
{
  PerkinElmerConfig(args[0].sval, args[1].ival, args[2].sval, args[3].ival, 
                    args[4].ival, args[5].ival, args[6].ival);
}


static void PerkinElmerRegister(void)
{
  iocshRegister(&configPerkinElmer, configPerkinElmerCallFunc);
}

extern "C" {
epicsExportRegistrar(PerkinElmerRegister);
}
