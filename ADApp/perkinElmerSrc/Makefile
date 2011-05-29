TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# Note: the SDK does support x64 but we need to get the files
ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
LIBRARY_IOC = PerkinElmer
LIB_SRCS += PerkinElmer.cpp
LIB_SRCS += drvPerkinElmerEpics.c

#Add detector SDK
LIB_LIBS += XISL

# Note, the following files were manually copied from the Perkin Elmer XIS SDK to this directory
BIN_INSTALLS   += ../XISL.dll
LIB_INSTALLS   += ../XISL.lib

PROD_IOC += PerkinElmerApp
endif

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

