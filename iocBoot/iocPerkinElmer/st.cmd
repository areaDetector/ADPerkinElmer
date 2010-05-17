errlogInit(20000)

< envPaths

dbLoadDatabase("$(AREA_DETECTOR)/dbd/PerkinElmerApp.dbd")
PerkinElmerApp_registerRecordDeviceDriver(pdbbase) 

epicsEnvSet("PREFIX", "13PE1:")
epicsEnvSet("PORT",   "PEDET1")
epicsEnvSet("QSIZE",  "20")
epicsEnvSet("XSIZE",  "1024")
epicsEnvSet("YSIZE",  "1024")
epicsEnvSet("NCHANS", "2048")

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
PerkinElmerConfig("$(PORT)", $(XSIZE), $(YSIZE), 3, 100, 200000000)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/ADBase.template",     "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/PerkinElmer.template","P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Create a standard arrays plugin, set it to get data from Driver.
NDStdArraysConfigure("Image1", 3, 0, "$(PORT)", 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,TYPE=Int16,SIZE=16,FTVL=SHORT,NELEMENTS=10000000")

# Load all other plugins using commonPlugins.cmd
< ../commonPlugins.cmd

#asynSetTraceIOMask("$(PORT)",0,2)
#asynSetTraceMask("$(PORT)",0,255)

iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30, "P=$(PREFIX)")
