#include <stdio.h>
#include <windows.h>
#include "Acq.h"

static unsigned int uiPEResult;
static HACQDESC hAcqDesc;


void checkError(const char *functionName)
{
  DWORD HISError, FGError;
  
  if (uiPEResult == HIS_ALL_OK) {
    printf("%s OK\n", functionName);
  } else {
    Acquisition_GetErrorCode(hAcqDesc, &HISError, &FGError);
    printf("Error %d, HISError=%d, FGError=%d, %s\n", uiPEResult, HISError, FGError, functionName);
    exit(-1);
  }
}

int main(int argc, char **argv)
{
  unsigned short *pOffsetBuffer;
  unsigned int uiRows, uiColumns;
  unsigned int uiNumSensors;
  bool bEnableIRQ = true;
  bool bInitAlways = false;
  ACQDESCPOS pos=NULL;
  int binning, iFrames=10;
  WORD wBinning;
  
  if (argc != 2) {
    printf("Usage: testBinning binning\n");
    exit(-1);
  }
  binning = atoi(argv[1]);
  printf("Binning = %d\n", binning);

  uiPEResult = Acquisition_EnumSensors(&uiNumSensors, bEnableIRQ, bInitAlways);
  checkError("Acquisition_EnumSensors");

  uiPEResult = Acquisition_GetNextSensor(&pos, &hAcqDesc);
  checkError("Acquisition_GetNextSensor");

  // set detector binning mode
  switch(binning) {
    case 1:
      wBinning = 1;
      break;
    case 2:
      wBinning = 2;
      break;
    case 4:
      wBinning = 3;
      break;
    default:
      printf("Error, binning must be 1, 2 or 4\n");
      exit(-1);
      break;
  }
  // Do averaged binning
  wBinning += 256;
  uiRows = 2048/binning;
  uiColumns = 2048/binning;
  uiPEResult = Acquisition_SetCameraBinningMode(hAcqDesc, wBinning);
  checkError("Acquisition_SetCameraBinningMode");

  pOffsetBuffer = (unsigned short *) malloc(sizeof(unsigned short) * uiRows * uiColumns);
  uiPEResult = Acquisition_Acquire_OffsetImage(hAcqDesc, pOffsetBuffer, 
                                               uiRows, uiColumns, iFrames);
  checkError("Acquisition_AcquireOffsetImage");

  uiPEResult = Acquisition_CloseAll();
  checkError("Acquisition_CloseAll");

  return 0;
}
