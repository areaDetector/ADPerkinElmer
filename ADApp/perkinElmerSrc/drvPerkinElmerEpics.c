/* drvPerkinElmerEpics.c
 *
 * This is the EPICS dependent code for the driver for a simulated area detector.
 * By making this separate file for the EPICS dependent code the driver itself
 * only needs libCom from EPICS for OS-independence.
 *
 * Author: Brian Tieman
 *
 * Created:  07/24/2008
 *
 */

#include <iocsh.h>
#include <drvSup.h>
#include <epicsExport.h>

#include "drvPerkinElmer.h"


/* Code for iocsh registration */

/* PerkinElmerConfig */
static const iocshArg PerkinElmerConfigArg0 = {"Port name", iocshArgString};
static const iocshArg PerkinElmerConfigArg1 = {"Max X size", iocshArgInt};
static const iocshArg PerkinElmerConfigArg2 = {"Max Y size", iocshArgInt};
static const iocshArg PerkinElmerConfigArg3 = {"Data type", iocshArgInt};
static const iocshArg PerkinElmerConfigArg4 = {"maxBuffers", iocshArgInt};
static const iocshArg PerkinElmerConfigArg5 = {"maxMemory", iocshArgInt};
static const iocshArg PerkinElmerConfigArg6 = {"priority", iocshArgInt};
static const iocshArg PerkinElmerConfigArg7 = {"stackSize", iocshArgInt};
static const iocshArg * const PerkinElmerConfigArgs[] =  {&PerkinElmerConfigArg0,
                                                          &PerkinElmerConfigArg1,
                                                          &PerkinElmerConfigArg2,
                                                          &PerkinElmerConfigArg3,
                                                          &PerkinElmerConfigArg4,
                                                          &PerkinElmerConfigArg5,
                                                          &PerkinElmerConfigArg6,
                                                          &PerkinElmerConfigArg7};
static const iocshFuncDef configPerkinElmer = {"PerkinElmerConfig", 8, PerkinElmerConfigArgs};
static void configPerkinElmerCallFunc(const iocshArgBuf *args)
{
    PerkinElmerConfig(args[0].sval, args[1].ival, args[2].ival, args[3].ival, args[4].ival, args[5].ival, args[6].ival, args[7].ival);
}


static void PerkinElmerRegister(void)
{

    iocshRegister(&configPerkinElmer, configPerkinElmerCallFunc);
}

epicsExportRegistrar(PerkinElmerRegister);


