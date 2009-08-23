# Must have loaded envPaths via st.cmd.linux or st.cmd.win32

errlogInit(20000)

< envPaths

dbLoadDatabase("$(AREA_DETECTOR)/dbd/PerkinElmerApp.dbd")
PerkinElmerApp_registerRecordDeviceDriver(pdbbase) 

# Create a PerkinElmer driver
# PerkinElmerConfig(const char *portName, int maxSizeX, int maxSizeY, int dataType, int maxBuffers, size_t maxMemory, int priority, int stackSize)
# DataTypes:
#    0 = NDInt8
#    1 = NDUInt8
#    2 = NDInt16
#    3 = NDUInt16
#    4 = NDInt32
#    5 = NDUInt32
#    6 = NDFloat32
#    7 = NDFloat64
#
PerkinElmerConfig("PEDET1", 1024, 1024, 3, 100, 200000000, 50, 10000)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/ADBase.template",     "P=JPHPE1:,R=cam1:,PORT=PEDET1,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/PerkinElmer.template","P=JPHPE1:,R=cam1:,PORT=PEDET1,ADDR=0,TIMEOUT=1")
#
#
#
#
# Create a standard arrays plugin, set it to get data from first simDetector driver.
NDStdArraysConfigure("PEDET1Image", 3, 0, "PEDET1", 0, 100000000, 50, 10000)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=JPHPE1:,R=image1:,PORT=PEDET1Image,ADDR=0,TIMEOUT=1,NDARRAY_PORT=PEDET1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStdArrays.template", "P=JPHPE1:,R=image1:,PORT=PEDET1Image,ADDR=0,TIMEOUT=1,TYPE=Int16,SIZE=16,FTVL=SHORT,NELEMENTS=10000000")
#
#
#
# Create an ROI plugin
#drvNDROIConfigure(const char *portName, int queueSize, int blockingCallbacks, const char *NDArrayPort, int NDArrayAddr, int maxBuffers, int maxROIs, size_t maxMemory)
NDROIConfigure("PEDET1ROI", 50, 0, "PEDET1", 0, 8, 50, 100000000, 50, 10000)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=JPHPE1:,R=ROI1:,  PORT=PEDET1ROI,ADDR=0,TIMEOUT=1,NDARRAY_PORT=PEDET1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDROI.template",       "P=JPHPE1:,R=ROI1:,  PORT=PEDET1ROI,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDROIN.template",      "P=JPHPE1:,R=ROI1:0:,PORT=PEDET1ROI,ADDR=0,TIMEOUT=1,HIST_SIZE=256")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDROIN.template",      "P=JPHPE1:,R=ROI1:1:,PORT=PEDET1ROI,ADDR=1,TIMEOUT=1,HIST_SIZE=256")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDROIN.template",      "P=JPHPE1:,R=ROI1:2:,PORT=PEDET1ROI,ADDR=2,TIMEOUT=1,HIST_SIZE=256")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDROIN.template",      "P=JPHPE1:,R=ROI1:3:,PORT=PEDET1ROI,ADDR=3,TIMEOUT=1,HIST_SIZE=256")
#
#
#
#
# Create a file saving plugin
# Create a TIFF file saving plugin
NDFileTIFFConfigure("FileTIFF", 20, 0, "SIM1", 0, 50, 10000)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=JPHPE1:,R=TIFF1:,PORT=FileTIFF,ADDR=0,TIMEOUT=1,NDARRAY_PORT=PEDET1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=JPHPE1:,R=TIFF1:,PORT=FileTIFF,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileTIFF.template",  "P=JPHPE1:,R=TIFF1:,PORT=FileTIFF,ADDR=0,TIMEOUT=1")

#
#
#
#
#asynSetTraceIOMask("PEDET1",0,2)
#asynSetTraceMask("PEDET1",0,255)
#
#
#
set_requestfile_path("./")
set_savefile_path("./autosave")
set_requestfile_path("$(AREA_DETECTOR)/ADApp/Db")
set_pass0_restoreFile("auto_settings.sav")
set_pass1_restoreFile("auto_settings.sav")
save_restoreSet_status_prefix("JPHPE1:")
dbLoadRecords("$(AUTOSAVE)/asApp/Db/save_restoreStatus.db", "P=JPHPE1:")
#
#
iocInit()

# save things every thirty seconds
#
#
create_monitor_set("auto_settings.req", 30, "P=JPHPE1:")
