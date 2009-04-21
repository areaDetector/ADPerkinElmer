TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#USR_CFLAGS += /DHDF5
#USR_CXXFLAGS += /DHDF5

LIBRARY_IOC = PerkinElmer
LIB_SRCS += PerkinElmer.cpp
LIB_SRCS += drvPerkinElmerEpics.c
LIB_LIBS += ADBase NDPlugin netCDF
#LIB_LIBS += nexusPlugin
#LIB_LIBS += jareadPlugin
LIB_LIBS += sscan autosave
LIB_LIBS += asyn busy
#LIB_LIBS += jvm
#LIB_LIBS += hdf5
LIB_LIBS += $(EPICS_BASE_IOC_LIBS)

#Add detector SDK
LIB_LIBS += XISL

# Note, the following files were manually copied from the Perkin Elmer XIS SDK to this directory
#BIN_INSTALLS_WIN32    += ../XISL.dll
#LIB_INSTALLS_WIN32    += ../XISL.lib

#==================================
PROD_IOC += PerkinElmerApp
# <name>.dbd will be created from <name>Include.dbd
DBD += PerkinElmerApp.dbd
DBD += PerkinElmerSupport.dbd

USR_INCLUDES = -IC:/XIS/SDK

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PerkinElmerApp_SRCS_DEFAULT += PerkinElmerApp_registerRecordDeviceDriver.cpp PerkinElmerAppMain.cpp
PerkinElmerApp_SRCS_vxWorks += PerkinElmerApp_registerRecordDeviceDriver.cpp

# Add locally compiled object code
PerkinElmerApp_LIBS += PerkinElmer
PerkinElmerApp_LIBS += NDPlugin ADBase netCDF
#PerkinElmerApp_LIBS += nexusPlugin
#PerkinElmerApp_LIBS += jareadPlugin
PerkinElmerApp_LIBS += sscan autosave 
PerkinElmerApp_LIBS += asyn busy
#PerkinElmerApp_LIBS += jvm
#PerkinElmerApp_LIBS += hdf5 libc zdll szlib
PerkinElmerApp_LIBS += $(EPICS_BASE_IOC_LIBS)

#Add detector SDK
XISL_DIR = C:/XIS/SDK
PerkinElmerApp_LIBS += XISL

#PerkinElmerCCDApp_SYS_LIBS_cygwin32 += rpc

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

