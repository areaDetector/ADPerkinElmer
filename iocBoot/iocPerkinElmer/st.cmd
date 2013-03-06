errlogInit(20000)

< envPaths

dbLoadDatabase("$(AREA_DETECTOR)/dbd/PerkinElmerApp.dbd")
PerkinElmerApp_registerRecordDeviceDriver(pdbbase) 

epicsEnvSet("PREFIX", "13PE1:")
epicsEnvSet("PORT",   "PEDET1")
epicsEnvSet("QSIZE",  "20")
epicsEnvSet("XSIZE",  "2048")
epicsEnvSet("YSIZE",  "2048")
epicsEnvSet("NCHANS", "2048")

# Create a PerkinElmer driver
# PerkinElmerConfig(const char *portName, IDType, IDValue, maxBuffers, size_t maxMemory, int priority, int stackSize)
# IDType = 0 Frame grabber card, IDValue="", use first frame grabber or directly connected GigE detector
#        = 1 GigE detector by IP address (e.g. 164.54.160.21)
#        = 2 GigE detector by MAC address (e.g. 00005b032e6b, must be lower-case letters)
#        = 3 GigE detector by detector name (e.g. 8#2608).  Can get network detector names with asynReport(10)

# This is for the first PCI/PCIExpress frame grabber detector in the system
#PerkinElmerConfig("$(PORT)", 0, "", 100, 200000000, 0, 0)

# This is for a GigE detector at IP address 164.54.160.204
#PerkinElmerConfig("$(PORT)", 1, 164.54.160.204, 100, 200000000, 0, 0)

# This is for a GigE detector at MAC address 00005b032e6b
#PerkinElmerConfig("$(PORT)", 2, 00005b032e6b, 100, 200000000, 0, 0)

# This is for a GigE detector with name 8#2608
PerkinElmerConfig("$(PORT)", 3, 8#2608, 100, 200000000, 0, 0)

asynSetTraceIOMask($(PORT), 0, 2)
#asynSetTraceMask($(PORT),0,0xff)

dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/ADBase.template",     "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/PerkinElmer.template","P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Create a standard arrays plugin, set it to get data from Driver.
NDStdArraysConfigure("Image1", 3, 0, "$(PORT)", 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,TYPE=Int16,SIZE=16,FTVL=SHORT,NELEMENTS=10000000")

# Load all other plugins using commonPlugins.cmd
< ../commonPlugins.cmd

iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30, "P=$(PREFIX)")

#asynSetTraceMask($(PORT),0,0xff)
