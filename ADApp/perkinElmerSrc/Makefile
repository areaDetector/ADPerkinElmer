TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC_WIN32 = PerkinElmer
LIB_SRCS += PerkinElmer.cpp
LIB_SRCS += drvPerkinElmerEpics.c

#Add detector SDK
LIB_LIBS += XISL

# Note, the following files were manually copied from the Perkin Elmer XIS SDK to this directory
BIN_INSTALLS_WIN32    += ../XISL.dll
LIB_INSTALLS_WIN32    += ../XISL.lib

PROD_IOC_WIN32 += PerkinElmerApp

# <name>.dbd will be created from <name>Include.dbd
DBD += PerkinElmerApp.dbd
DBD += PerkinElmerSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PROD_SRCS += PerkinElmerApp_registerRecordDeviceDriver.cpp PerkinElmerAppMain.cpp

# Add locally compiled object code
PROD_LIBS += PerkinElmer
PROD_LIBS += XISL

include $(TOP)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

