TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#USR_CFLAGS += /DHDF5
#USR_CXXFLAGS += /DHDF5

LIBRARY_IOC_WIN32 = PerkinElmer
LIB_SRCS += PerkinElmer.cpp
LIB_SRCS += drvPerkinElmerEpics.c
LIB_LIBS += ADBase NDPlugin netCDF
LIB_LIBS_Linux += PvAPI ImageLib
LIB_LIBS_WIN32 += PvAPI ImageLib
LIB_LIBS += calc busy sscan autosave mca
LIB_LIBS += asyn
LIB_LIBS += $(EPICS_BASE_IOC_LIBS)

#Add detector SDK
LIB_LIBS += XISL

# Note, the following files were manually copied from the Perkin Elmer XIS SDK to this directory
BIN_INSTALLS_WIN32    += ../XISL.dll
LIB_INSTALLS_WIN32    += ../XISL.lib

#==================================
PROD_IOC_WIN32 += PerkinElmerApp
# <name>.dbd will be created from <name>Include.dbd
DBD += PerkinElmerApp.dbd
DBD += PerkinElmerSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PerkinElmerApp_SRCS_DEFAULT += PerkinElmerApp_registerRecordDeviceDriver.cpp PerkinElmerAppMain.cpp

# Add locally compiled object code
PerkinElmerApp_LIBS += PerkinElmer
PerkinElmerApp_LIBS += XISL
PerkinElmerApp_LIBS += NDPlugin ADBase netCDF
PerkinElmerApp_LIBS_Linux += PvAPI ImageLib
PerkinElmerApp_LIBS_WIN32 += PvAPI ImageLib
PerkinElmerApp_LIBS += calc busy sscan autosave mca
PerkinElmerApp_LIBS += asyn
PerkinElmerApp_LIBS += $(EPICS_BASE_IOC_LIBS)

PerkinElmerApp_LIBS_WIN32   += libtiff jpeg libNeXus.dll

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

