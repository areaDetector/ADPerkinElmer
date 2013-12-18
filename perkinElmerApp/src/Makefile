TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#======== DETECTOR LIBRARY ==============

ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
USR_CPPFLAGS += -D__X64
endif

LIBRARY_IOC_WIN32 = PerkinElmer
LIB_SRCS_WIN32 += PerkinElmer.cpp
LIB_LIBS += XISL

DBD += PerkinElmerSupport.dbd

#====== DETECTOR APPLICATION ============

ifeq ($(BUILD_APPS), YES)
PROD_NAME = PerkinElmerApp
PROD_IOC_WIN32 += $(PROD_NAME)
PROD_HOST_WIN32 += testBinning

# <name>.dbd will be created from <name>Include.dbd
DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += PerkinElmerSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
$(PROD_NAME)_SRCS += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp

# Add locally compiled object code
$(PROD_NAME)_LIBS += PerkinElmer
$(PROD_NAME)_LIBS += XISL

testBinning_SRCS += testBinning
testBinning_LIBS += XISL

include $(TOP)/ADApp/commonDriverMakefile
endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

