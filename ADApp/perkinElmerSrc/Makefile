TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
USR_CPPFLAGS += -D__X64
endif

LIBRARY_IOC_WIN32 = PerkinElmer
LIB_SRCS_WIN32 += PerkinElmer.cpp

PROD_NAME = PerkinElmerApp
PROD_IOC_WIN32 += $(PROD_NAME)
PROD_HOST_WIN32 += testBinning

# <name>.dbd will be created from <name>Include.dbd
DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += PerkinElmerSupport.dbd
DBD += PerkinElmerSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PerkinElmerApp_SRCS += PerkinElmerApp_registerRecordDeviceDriver.cpp PerkinElmerAppMain.cpp

# Add locally compiled object code
PerkinElmerApp_LIBS += PerkinElmer
PerkinElmerApp_LIBS += XISL

testBinning_SRCS += testBinning
testBinning_LIBS += XISL

include $(TOP)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

