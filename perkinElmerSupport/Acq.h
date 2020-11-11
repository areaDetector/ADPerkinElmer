/** @file Acq.h*/


#ifndef _ACQUISITION_H
#define _ACQUISITION_H

//////////////////////////////////////////////////////
// IMPORTANT:
// When implementing the XISL dll into a .net environment some datatypes might have to be changed to to size definition:
//
// e.g. 
// XISL dll: long has 4 byte
// .net: int has 4 byte, long has 8 byte
// so long has to be important as int
// 
// XISL dll: char has 1 byte
// .net: char has 2 bytes
//
//////////////////////////////////////////////////////

#define DATASHORT               2 //2 byte integer
#define DATALONG                4 //4 byte integer
#define DATAFLOAT               8 //8 byte double
#define DATASIGNED              16 // signed

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

#define __NOASM

//#define __X64 // Define this for the 64bit Driver Version
#ifdef _WIN64
#   ifndef __X64
#       define __X64
#   endif
#endif

#if defined(__X64)
#	ifndef XIS_OS_64
#		define XIS_OS_64
#	endif
#endif

#ifdef _WIN32
#include <Windows.h>
#endif

#ifdef __linux__   
	#include "windefines.h"
	#include "DataType.h"
#endif


#ifdef XIS_OS_64	// MERGE: think about these macros
    /**
    * @ingroup enum
    */
	typedef void* ACQDESCPOS;
#else
    /**
    * @ingroup enum
    */
	typedef UINT ACQDESCPOS;
#endif


/** 
* AcquisitionDesc defines a data structure that is 
* used by all functions of XISL. It contains all 
* required parameters for the acquisition.
* Access to the data fields is only possible via 
* the XISL API functions. HACQDESC defines a HANLDE to the acquisition descriptor.
* @see AcquisitionDesc
* @ingroup enum
**/
typedef HANDLE HACQDESC;
#ifdef __linux__
    /**
    * @ingroup enum
    */
    typedef unsigned int DEX_RETURN;
#else
	// Windows
	#ifndef USE_XISL_AS_STATIC_LIB
    #ifdef _DLL_EXPORT
		#define _DLL_API __declspec(dllexport) 
    #else
		#define _DLL_API __declspec(dllimport) 
    #endif //_ACQDEFS_H
	#else
		#define _DLL_API 
	#endif // USE_XISL_AS_STATIC_LIB

    #define HIS_RETURN _DLL_API UINT WINAPI
	#define DEX_RETURN _DLL_API UINT WINAPI
    
#endif


#define _GBIF_1313


#define WINRESTSIZE					34
#define WINHARDWAREHEADERSIZE		32
#define WINRESTSIZE101				32
#define WINHARDWAREHEADERSIZEID15	2048
#define DETEKTOR_DATATYPE_18BIT 16
#define MAX_GREY_VALUE_18BIT  262144

#ifndef __GBIF_Types
#define __GBIF_Types
	#define GBIF_IP_MAC_NAME_CHAR_ARRAY_LENGTH		16
	#define GBIF_STRING_DATATYPE unsigned char
	#define GBIF_STRING_DATATYPE_ELTEC char
#endif


typedef struct
{	
	GBIF_STRING_DATATYPE	ucMacAddress[GBIF_IP_MAC_NAME_CHAR_ARRAY_LENGTH];			// unsigned since the adress components can be higher than 128
	GBIF_STRING_DATATYPE	ucIP[GBIF_IP_MAC_NAME_CHAR_ARRAY_LENGTH];
	GBIF_STRING_DATATYPE	ucSubnetMask[GBIF_IP_MAC_NAME_CHAR_ARRAY_LENGTH];
	GBIF_STRING_DATATYPE	ucGateway[GBIF_IP_MAC_NAME_CHAR_ARRAY_LENGTH];
	GBIF_STRING_DATATYPE	ucAdapterIP[GBIF_IP_MAC_NAME_CHAR_ARRAY_LENGTH];
	GBIF_STRING_DATATYPE	ucAdapterMask[GBIF_IP_MAC_NAME_CHAR_ARRAY_LENGTH];

	DWORD					dwIPCurrentBootOptions;
	CHAR 					cManufacturerName[32];
	CHAR					cModelName[32];												// GBIF
	CHAR					cGBIFFirmwareVersion[32];
	CHAR					cDeviceName[16];

} GBIF_DEVICE_PARAM;

// 128Byte in total
typedef struct
{	
	char	cDetectorType[32];															// e.g. XRD 0822 AO 14
	char	cManufacturingDate[8];														// e.g. 201012
	char	cPlaceOfManufacture[8];														// e.g. DE
    char    cUniqueDeviceIdentifier[16];
    char    cDeviceIdentifier[16];
	char	cDummy[48];
} GBIF_Detector_Properties;

#define HIS_GbIF_FIRST_CAM 0
#define HIS_GbIF_IP 1
#define HIS_GbIF_MAC 2
#define HIS_GbIF_NAME 3

#define HIS_GbIF_IP_STATIC	1
#define HIS_GbIF_IP_DHCP	2
#define HIS_GbIF_IP_LLA		4

//*/

typedef struct
{
	DWORD	dwPROMID;
	DWORD	dwHeaderID;
	BOOL	bAddRow;
	BOOL	bPwrSave;
	DWORD	dwNrRows;
	DWORD	dwNrColumns;
	DWORD	dwZoomULRow;
	DWORD	dwZoomULColumn;
	DWORD	dwZoomBRRow;
	DWORD	dwZoomBRColumn;
	DWORD	dwFrmNrRows;
	DWORD	dwFrmRowType;
	DWORD	dwFrmFillRowIntervalls;
	DWORD	dwNrOfFillingRows;
	DWORD	dwDataType;
	DWORD	dwDataSorting;
	DWORD	dwTiming;
	DWORD	dwAcqMode;
	DWORD	dwGain;
	DWORD	dwOffset;
	DWORD	dwAccess;
	BOOL	bSyncMode;
	DWORD	dwBias;
	DWORD	dwLeakRows;
} CHwHeaderInfo;

typedef struct
{
	WORD	wHeaderID;		// 0
	WORD	wPROMID;		// 1
	WORD	wResolutionX;	// 2
	WORD	wResolutionY;	// 3
	WORD	wNrRows;		// 4
	WORD	wNrColumns;		// 5
	WORD	wZoomULRow;		// 6
	WORD	wZoomULColumn;	// 7
	WORD	wZoomBRRow;		// 8
	WORD	wZoomBRColumn;	// 9
	WORD	wFrmNrRows;		// A
	WORD	wFrmRowType;	// B
	WORD	wRowTime;		// C << 6
	WORD	wClock;			// D << 6
	WORD	wDataSorting;	// E
	WORD	wTiming;		// F
	WORD	wGain;			// 10
	WORD	wLeakRows;		// 11
	WORD	wAccess;		// 12
	WORD	wBias;			// 13
	WORD	wUgComp;		// 14
	WORD	wCameratype;	// 15
	WORD	wFrameCnt;		// 16
	WORD	wBinningMode;	// 17
	WORD	wRealInttime_milliSec;	// 18 
	WORD	wRealInttime_microSec;	// 19
	WORD	wStatus;		// 1A
	WORD	wCommand1;		// 1B
	WORD	wCommand2;		// 1C
	WORD	wCommand3;		// 1D
	WORD	wCommand4;		// 1E
	WORD	wDummy;			// 1F
}	CHwHeaderInfoEx;

#pragma pack(push, 1)
typedef struct
{
	WORD FileType;			//!<  File ID (0x7000)
	WORD HeaderSize;		//!<  Size of this file header in Bytes
	WORD HeaderVersion;		//!<  Must be 100, default XIS file header not to be used for onboard 
    UINT FileSize;			//!<  Size of the whole file in Bytes ( HeaderSize+ImageHeaderSize+Frames*rows*columns*datatypesize )
	WORD ImageHeaderSize;	//!<  Size of the Image header in Bytes 32 Bytes, values can be all zero
	WORD ULX, ULY, BRX, BRY;//!<  Bounding rectangle of the image
	WORD NrOfFrames;		//!<  Nr of Frames in seq
	WORD Correction;		//!<  0 = none, 1 = offset, 2 = gain, 4 = bad pixel, (ored) can be 0
	double IntegrationTime;	//!<  Frame time in microseconds can by 0
	WORD TypeOfNumbers;		//!<  Refer to enum XIS_FileType
	BYTE x[WINRESTSIZE];	//!<  Fill up this struct to have a size of 68 bytes (FileHeaderSize)
} WinHeaderType;




typedef struct 
{
	WORD FileType;			//!<  File ID (0x7000)
	WORD HeaderSize;		//!<  Size of this file header in Bytes
	WORD HeaderVersion;		//!<  Must be 101 for Onboard file Header used by XRpad[2] using this file type for onboard corrections
	UINT FileSize;			//!<  Size of the whole file in Bytes ( HeaderSize+ImageHeaderSize+Frames*rows*columns*datatypesize )
	WORD ImageHeaderSize;	//!<  Size of the Image header in Bytes 2048 Bytes, values can be all zero
	WORD ULX, ULY, BRX, BRY;//!<  Bounding rectangle of the image
	WORD NrOfFrames;		//!<  Nr of Frames in seq
	WORD Correction;		//!<  0 = none, 1 = offset, 2 = gain, 4 = bad pixel, (ored) can be 0
	double IntegrationTime;	//!<  Frame time in microseconds
	WORD TypeOfNumbers;		//!<  Refer to enum XIS_FileType must be PKI_ERRORMAPONBOARD ( 1bit per pixel images filled up to full byte) or PKI_SHORT (2Bytes per Pixel) for onboard usage
	WORD wMedianValue;		//!<  Median of the image / shall be 0. use 0 for onboard corrections. median for gain corr will be automatically calculated
	BYTE x[WINRESTSIZE101];	//!<  Fill up this struct to have a size of 68 bytes (FileHeaderSize)
} WinHeaderType101;

#pragma pack(pop)

typedef struct 
{
	DWORD	dwPROMID;
	char	strProject[6];		// project / cam-nr
	char	strSystemused[3];	//
	char	strPrefilter[9];	// used filter
	float	fKVolt;				// 
	float	fAmpere;			//
	WORD	n_avframes;			// average count
		
} WinImageHeaderType;

typedef struct
{
	unsigned char	wTiming;		// Timing und Triggermode
	unsigned char	wValue0;		// value 3 HIGHBYTE
	unsigned char	wValue1;        // value 3 LOBYTE
	unsigned char	wValue2;        // value 2 HIGHBYTE
	unsigned char	wValue3;        // value 2 LOBYTE
	unsigned char	wValue4;        // value 1 HIGHBYTE
	unsigned char	wValue5;        // value 1 LOBYTE
	unsigned char	wValue6;        // command to send
} FPGAType;					// 8 Byte werden bertragen


#define EPC_REGISTER_LENGTH      1024

typedef struct RTC_STRUCT {    
  DWORD year;    
  DWORD month;								// e.g.: 5 for may
  DWORD day;    
  DWORD hour;    
  DWORD minute;    
  DWORD second;    
   
}RTC_STRUCT; 

typedef struct DETECTOR_BATTERY{    
  DWORD status; 							// D0: present
											// D1: charging
  DWORD serial_no;    
  DWORD cycle_count;    
  DWORD temperature;   						// e.g.: 2510 for 25.1 C 
  DWORD voltage;    						// in mV
  DWORD current;							// in mA (positive or negative)
  DWORD capacity;    						// in %
  DWORD energy;    							// in mWh
  DWORD charge;    							// in mAh
      
}DETECTOR_BATTERY; 


typedef struct EPC_REGISTER {
  DWORD version;
  DWORD temperature_value[8];				// in 1/1000 C (e.g. 43000 for 43 C)
  DWORD temperature_warning_level[8];
  DWORD temperature_error_level[8];
  RTC_STRUCT rtc_value;
  DETECTOR_BATTERY battery;
  DWORD power_state;
  DWORD sdcard_state;						// D0: is mounted flag
  DWORD sdcard_usage;						// in %
  DWORD active_network_config;
  DWORD lan_status_register;				// D0: LAN enabled
											// D1: LAN up
											// D2: LAN used for image transfer
  DWORD wlan_status_register;				// D0: WLAN enabled
											// D1: WLAN up
											// D2: WLAN used for image transfer
											// D3: is accesspoint (0 for station)
											// D4: HT20 mode
											// D5: HT40+ mode
											// D6: HT40- mode
  DWORD signal_strength;					// in dBm
  DWORD channel;
  DWORD exam_flag;
  DWORD spartan_id;
  CHwHeaderInfoEx spartan_register;
  
}EPC_REGISTER;

/* internal voltages and currents of Gen2 detector */
typedef struct DETECTOR_CURRENT_VOLTAGE{
  int iV1;
  int imA1;
  int iV2;
  int imA2;
  int iV3;
  int imA3;
} DETECTOR_CURRENT_VOLTAGE;


/** Possible system control actions for XRpad 
* @ingroup enum
*/
typedef enum {
	XRpad_SYSTEM_CONTROL_REBOOT = 0,           //!<  restart XRpad 
	XRpad_SYSTEM_CONTROL_RESTART_NETWORK = 1,  //!<  restart XRpad Network 
	XRpad_SYSTEM_CONTROL_SHUTDOWN = 2,         //!<  shutdown XRpad
	XRpad_SYSTEM_CONTROL_SET_DEEP_SLEEP = 3,   //!<  power down analog circuitry and sensor FPGA
	XRpad_SYSTEM_CONTROL_SET_IDLE = 4,         //!<  power up analog circuitry and sensor FPGA
	XRpad_SYSTEM_CONTROL_RESTART_WLAN=9        //!<  restart XRpad2 WLAN only
} XRpad_SystemControlEnum;


/**
* @ingroup enum
*/
typedef struct XRpad_TempSensor
{
    char            index;
    char            name[32];
    BOOL            is_virtual;
    unsigned char   warn_level;
    double          temperature;
} XRpad_TempSensor;


/**
* @ingroup enum
*/
typedef struct XRpad_TempSensorReport
{
    unsigned char   system_warn_level;
    unsigned char   sensor_count;
    unsigned int    shutdown_time;
    XRpad_TempSensor sensors[16];
} XRpad_TempSensorReport;


/**
* @ingroup enum
*/
typedef enum XRpad_ChargeMode
{
    XRpad_NOT_CHARGING      = 0,    //!< Charging disabled
    XRpad_CHARGING_SLOW     = 1,    //!< Charging slowly
    XRpad_CHARGING_NORMAL   = 2,    //!< Charging normal
    XRpad_CHARGING_FAST     = 3,    //!< Charging fast
    XRpad_FULLY_CHARGED     = 4,    //!< Battery is fully charged
    XRpad_DISCHARGING       = 5     //!< Battery is discharging
} XRpad_ChargeMode;


/**
* @ingroup enum
*/
typedef enum XRpad_BatteryPresence
{
    XRpad_NO_BATTERY        = 0,    //!< No battery detecte
    XRpad_BATTERY_INSERTED  = 1,    //!< Battery detected
    XRpad_DUMMY_INSERTED    = 2,    //!< Dummy battery detected (unused)
    XRpad_BATTERY_COM_ERR   = 3     //!< Communication error on SMBUS
} XRpad_BatteryPresence;


/**
* @ingroup enum
*/
typedef enum XRpad_BatteryHealth
{
    XRpad_BATTERY_OK                    = 0x0000,   //!< All OK
    XRpad_COMMUNICATION_ERROR           = 0x0001,   //!< Communication error on SMBUS
    XRpad_TERMINATE_DISCHARGE_ALARM     = 0x0002,   //!< Battery is nearly empty but still discharging
    XRpad_UNDERVOLTAGE_ALARM            = 0x0004,   //!< Battery voltage is to low will be stopped
    XRpad_OVERVOLTAGE_ALARM             = 0x0008,   //!< Battery voltage is to high charging will be stopped
    XRpad_OVERTEMPERATURE_ALARM         = 0x0010,   //!< Battery temperature is to high chargin will be stopped
    XRpad_BATTERY_UNKNOWN_ERROR         = 0x0080,   //!< Unknown error reported by battery controller

} XRpad_BatteryHealth;


/**
* @ingroup enum
*/
typedef struct XRpad_BatteryStatus
{
    XRpad_BatteryPresence   presence;              //!< Provides the information if a battery is present (XRpad_BatteryPresence).
    int                     design_capacity;       //!< Displays the information of the original capacity of this battery.
    int                     remaining_capacity;    //!< Retrieves the information about the remaining charge in mA.
    int                     charge_state;          //!< Retrieves the information about the charge status in percent.
    XRpad_ChargeMode        charge_mode;           //!< Retrieves the current charge mode (XRpad_ChargeMode). 
    int                     cycle_count;           //!< Retrieves number of charging cycles.
    int                     temperature;           //!< Retrieves the temperature of the battery.
    int                     authenticated;         //!< Retrieves the information if an OEM battery is used.
    int                     health;                //!< Displays the overall status of the battery which is an ored combination of the XRpad_BatteryHealth enum values provided by the battery controller
} XRpad_BatteryStatus;



/**
 * @ingroup enum
 */
typedef struct XRpad_ShockEvent
{
    unsigned int    timestamp;
    unsigned int    critical_sensor1;
    unsigned int    critical_sensor2;
    unsigned int    critical_sensor3;
    unsigned int    warning_sensor1;
    unsigned int    warning_sensor2;
    unsigned int    warning_sensor3;
} XRpad_ShockEvent;



/**
 * @ingroup enum
 */
typedef struct XRpad_ShockSensorReport
{
    XRpad_ShockEvent largest;
    XRpad_ShockEvent latest;
} XRpad_ShockSensorReport;



/**
 * @ingroup enum
 */
typedef struct XRpad_VersionInfo
{
    char subversion[256];
    char linux_kernel[32];
    char software[32];
    char hwdriver[32];
    char zynq_firmware[32];
    char spartan_firmware[32];
    char msp_firmware[32];
    char pld_firmware[32];
    char xrpd[32];
    char wlan[32];
} XRpad_VersionInfo;



/** Possible image transfer channels for XRpad
* @ingroup enum
*/
typedef enum XRpad_DataInterfaceControlEnum{
    XRpad_DATA_VIA_LAN = 0,             //!<  use LAN
    XRpad_DATA_VIA_WLAN = 1             //!<  use WLAN
} XRpad_DataInterfaceControlEnum;



/**
* @ingroup enum
*/
typedef enum
{
	LEVEL_TRACE = 0,
	LEVEL_DEBUG,
	LEVEL_INFO,
	LEVEL_WARN,
	LEVEL_ERROR,
	LEVEL_FATAL,
	LEVEL_ALL,
	LEVEL_NONE
} XislLoggingLevels;

/**
* @ingroup enum
*/
typedef void *XislFtpSession;

/**
* @ingroup enum
*/
typedef void *XislFileHandle;

/**
* @ingroup enum
*/
typedef enum XislFileEntryType
{
    XFT_File = 1,
    XFT_Directory = 2,
    XFT_Link = 4,
    // ...
    XFT_Other = 0x80000000,
    XFT_Any = 0xFFFFFFFF
} XislFileEntryType;

/**
* @ingroup enum
*/
typedef enum XislFileStorageLocation
{
    XFSL_Local = 0,
    XFSL_FTP = 1
} XislFileStorageLocation;

typedef struct XislFileInfo
{
    const char *filename;
    const char *directory;
    const char *address;
    size_t filesize;
    XislFileEntryType type;
    const char *timestamp;
    // XislFileStorageLocation location;
} XislFileInfo;

/**
* @ingroup enum
*
* @brief
* enum used for addressing XRpad2 on board correction stages.
*/
typedef enum ProcScriptOperation
{
    PREBINNING,         ///< reserved. don not use!
    PREMEAN,            ///< reserved. don not use!
    PRESTOREBUFFER,     ///< reserved. don not use!
    OFFSET,             ///< offset correction
    GAIN,               ///< gain correction (do not use in dual enmergy mode!)
    MEAN,               ///< pixel correction
    PREVIEW,            ///< reserved. don not use!
    BINNING,            ///< reserved. don not use!
    STOREBUFFER,        ///< reserved. don not use!
    STORESD,            ///< reserved. don not use!
    SEND,               ///< reserved. don not use!
    OFFSET_2,           ///< offset correction of second bright image in dual energy mode
    GAIN_2              ///< reserved. don not use!
} ProcScriptOperation;

/**
* @ingroup enum
*/
typedef enum OnboardBinningMode
{
	ONBOARDBINNING2x1 = 0,
	ONBOARDBINNING2x2 = 1,
	ONBOARDBINNING4x1 = 2,
	ONBOARDBINNING4x4 = 3,
	ONBOARDBINNING3x3 = 4,
	ONBOARDBINNING9to4 = 5
} OnboardBinningMode;

/**
* @ingroup enum
*/
typedef enum XIS_DetectorTriggerMode
{
	TRIGGERMODE_DDD,
	TRIGGERMODE_DDD_WO_CLEARANCE,
	TRIGGERMODE_STARTSTOP,
	TRIGGERMODE_FRAMEWISE,
	TRIGGERMODE_AED,
	TRIGGERMODE_ROWTAG,
	TRIGGERMODE_DDD_POST_OFFSET,
	TRIGGERMODE_DDD_DUAL_POST_OFFSET
}XIS_DetectorTriggerMode;

/**
* @ingroup enum
*/
typedef enum XIS_Detector_TRIGOUT_SignalMode
{
	TRIGOUT_SIGNAL_FRM_EN_PWM,
	TRIGOUT_SIGNAL_FRM_EN_PWM_INV,
	TRIGOUT_SIGNAL_EP,
	TRIGOUT_SIGNAL_EP_INV,
	TRIGOUT_SIGNAL_DDD_Pulse,
	TRIGOUT_SIGNAL_DDD_Pulse_INV,
	TRIGOUT_SIGNAL_GND,
	TRIGOUT_SIGNAL_VCC
}XIS_Detector_TRIGOUT_SignalMode;

/**
* @ingroup enum
*/
typedef enum XIS_FileType
{
	PKI_RESERVED = 1,
	PKI_DOUBLE = 2,
	PKI_SHORT = 4,
	PKI_SIGNED = 8,
	PKI_ERRORMAPONBOARD = 16,
	PKI_LONG = 32,
	PKI_SIGNEDSHORT = PKI_SHORT | PKI_SIGNED,
	PKI_SIGNEDLONG = PKI_LONG | PKI_SIGNED,
	PKI_FAULTMASK = PKI_LONG | PKI_RESERVED
} XIS_FileType;

/**
* @ingroup enum
*/
typedef enum XIS_Event
{
   XE_ACQUISITION_EVENT = 0x00000001,
   XE_SENSOR_EVENT      = 0x00000002,
   XE_SDCARD_EVENT      = 0x00000004,
   XE_BATTERY_EVENT     = 0x00000005,
   XE_LOCATION_EVENT    = 0x00000006,
   XE_NETWORK_EVENT     = 0x00000007,
   XE_DETECTOR_EVENT    = 0x00000008,
   XE_LIBRARY_EVENT     = 0x00000009,
   XE_SDCARD_FSCK_EVENT = 0x0000000A,
   XE_XRPD_EVENT        = 0x0000000B,
} XIS_Event;

/**
* @ingroup enum
*/
typedef enum XIS_Acquisition_Event
{
    XAE_TRIGOUT          = 0x00000002,
    XAE_READOUT          = 0x00000004,
    XAE_TRIGGERED        = 0x00000008,
    XAE_AED_READY        = 0x00000010,
} XIS_Acquisition_Event;

/**
* @ingroup enum
*/
typedef enum XIS_Sensor_Event
{
    XSE_HALL                            = 0x00000001,
    XSE_SHOCK                           = 0x00000010,
    XSE_TEMPERATURE                     = 0x00000020,
    XSE_TEMPERATURE_BACK_TO_NORMAL      = 0x00000021,
    XSE_THERMAL_SHUTDOWN                = 0x00000022,
} XIS_Sensor_Event;

/**
* @ingroup enum
*/
typedef enum XIS_Battery_Event
{
    XBE_BATTERY_REPORT                  = 0x00000001,
    XBE_BATTERY_WARNING                 = 0x00000002,
} XIS_Battery_Event;

/**
* @ingroup enum
*/
typedef enum XIS_Detector_Event
{
    XDE_BUFFERS_IN_USE                  = 0x00000001,
    XDE_STORED_IMAGE                    = 0x00000002,
    XDE_DROPPED_IMAGE                   = 0x00000003,
    XDE_POWER_STATE_CHANGED             = 0x00000004, //!<value 2 success: new power state
    XDE_POWER_STATE_CHANGE_FAILED       = 0x00000005  //!<value 2 error: code XRPD_ERROR_SWITCH_PWR_STATE_BLOCKED_BY_EXAM 0x0052, any other: internal error
} XIS_Detector_Event;

/**
* @ingroup enum
*/
typedef enum XIS_Library_Event
{
	XLE_HIS_ERROR_PACKET_LOSS           = 0x00000001, //!< Frame is lost. not all network packages could be received
} XIS_Library_Event;

/**
* @ingroup enum
*/
typedef enum XIS_Xrpd_Event
{
    XXE_CONNECTION_LOST                 = 0x00000001, //!< Connection to XRpad Daemon unexpectedly closed
    XXE_RECONNECTED                     = 0x00000002, //!< Connection re-established automatically
} XIS_Xrpd_Event;


typedef void (*XIS_EventCallback)(XIS_Event, UINT, UINT, void *, void *);


/**
 * @ingroup enum
 */
typedef enum XIS_Init_Flags
{
    XIF_XISL                            = 0x00000001, //!< Initialize libXISL
    XIF_GBIF                            = 0x00000002, //!< Initialize libgbif
    XIF_WPE200                          = 0x00000004, //!< Initialize libwpe200 (currently not used)
    XIF_CURL                            = 0x00000008, //!< Initialize libcurl
    // ... to be continued ...
    XIF_ALL                             = 0xFFFFFFFF  //!< Initialize everything
} XIS_Init_Flags;



/**
 * @ingroup enum
 * @brief   Transmission modes.
 * @note    Not all devices support each mode.
 *          Only XTM_LIVE is supported by all devices.
 */
typedef enum XIS_Transmission_Mode {
    XTM_LIVE                            = 0x00000001, //!< All images are sent out directly as they are acquired
    XTM_SAFE                            = 0x00000002  //!< Images are sent out when the previous was completely received
} XIS_Transmission_Mode;



// wpe library includes
/* Error codes */



#ifdef _DLL_EXPORT
#include "wpe200def.h"
#else
#define WPE_ERR_OK                   0  //!< No error
#define WPE_ILLEGAL_BUFFER      -10000  //!< A buffer supplied is 0, or a buffer length to small
#define WPE_ERR_JSON_PARSE      -10001  //!< Json parse error
#define WPE_ERR_JSON_UNPACK     -10002  //!< Json unpack error
#define WPE_ERR_SERVER_ERROR    -10003  //!< Web server error
#define WPE_ERR_CURL_ERROR      -10004  //!< Error returned by the curl library
#define WPE_ERR_NO_NET_ADAPTER  -10005  //!< No network adapters found
#define WPE_ERR_ILLEGAL_PARAM   -10006  //!< Illegal parameter
#define WPE_ERR_BASE64_ENCODE   -10007  //!< Error during base64 encoding
#define WPE_ERR_FORCE_IP        -10008  //!< Error force IP
#define WPE_ERR_NET_ADAPTER     -10009  //!< Error getting network adapters
#define WPE_ERR_JSON_CREATE     -10010  //!< Json creation error
#define WPE_ERR_PROPSTORE       -10011  //!< Error while using the PropertyStore

/**
 * The device info
 *
 */
struct deviceInfo
{
    char device_version[16];             //!< The device version
    char spec_version[16];               //!< GigE vision spec version
    char manufacturer_name[32];          //!< The manufactures name
    char model_name[32];                 //!< The model name
    char serial_number[16];              //!< The serial number
    char manufacturer_specific[48];      //!< Some manufacture info
    char user_name[16];                  //!< Device specific
};


/**
 * The network information
 *
 */
struct networkInfo
{
    char ip[16];        //!< The IP (v4) address as string
    char mask[16];      //!< The IP mask as string
    char broadcast[16]; //!< The IP broadcast as string
    char mac[18];       //!< The MAC address as string    
};


/**
* Information about a single discovery Reply network packet
*
*/
struct discoveryReplyMsg
{
	char remote_ip[16];
	char local_ip[16];
};


/**
* The original discovery reply
*
*/
struct discoveryReply
{
	struct deviceInfo deviceInfo; //!< The device information
	struct networkInfo lanInfo;   //!< The LAN network setup
	struct networkInfo wlanInfo;  //!< The WLAN network setup

	char gvcp_ip[16];             //!< Which IP address is used for image transfer
};


/**
* The extended discovery reply
* Can carry additional information
*/
struct discoveryReplyEx
{
	struct deviceInfo deviceInfo;   //!< The device information
	struct networkInfo lanInfo;     //!< The LAN network setup
	struct networkInfo wlanInfo;    //!< The WLAN network setup

	char gvcp_ip[16];               //!< Which IP address is used for image transfer

	struct discoveryReplyMsg messages[32]; //!< Info about the received reply packets
	unsigned messageCount;          //!< How many messages this reply carries
};

/**
* Structure for holding the adapter
* part of a configuration
*
*
*/
struct networkAdapterConfiguration
{
	int enabled;        //!< Enabled flag, this is not changeable for LAN by API, but changeable for WLAN
	int hw_accel;       //!< Used image transfer, this is not changeable by API
	int bridged;        //!< Is the device in a bridge, this is not changeable by API

	char ifname[16];    //!< Interface name (eth0), this is not changeable by API
	char ipaddr[16];    //!< IP address
	char netmask[16];   //!< Netmask
	char proto[16];     //!< "static" or "dhcp", only these two options are available
	char dns[16];       //!< DNS server
	char gateway[16];   //!< Gateway
	char macaddr[18];   //!< MAC address, this is not changeable by API

	char not_used[110]; //!< To fill up the struct to 320 byte
};


/**
* Wifi configurations
*/
struct wifiConfiguration
{
	char mode[32];      //!< Accesspoint or client
	char agmode[32];    //!< agmode
	int channel;        //!< Channel, only valid options will be accepted, use 0 for automatically select channel, otherwise will return with error

	char ssid[64];          //!< Own SSID if mode == "ap" or the accesspoints ssid
	char description[64];   //!< Contains the description in case of a station
};


/**
* Wifi configurations, extended version
*/
struct wifiConfigurationEx
{
	char mode[32];      //!< Accesspoint or client
	char agmode[32];    //!< agmode
	int channel;        //!< Channel, only valid options will be accepted, use 0 for automatically select channel, otherwise will return with error

	char ssid[64];          //!< Own SSID if mode == "ap" or the accesspoints ssid
	char description[64];   //!< Contains the description in case of a station

	char passphrase[68];    //!< new Passphrase (station or accesspoint) (may be 64 byte)
	int scan_ssid;          //!< only of station mode: scan the SSID
};




/**
* Structure for holding the complete
* network configuration.
*
*/
struct networkConfiguration
{
	char path[128];         //!< The configurations path, this is not changeable by API
	char name[80];          //!< The configuration name
	char hostname[80];      //!< The hostname
	int readonly;           //!< Is the configuration readonly, this is not changeable by API
	int sshd_enabled;       //!< SSH daemon enabled

	int gbif_enabled;       //!< Is the GBif enabled

	struct networkAdapterConfiguration lan;     //!< LAN
	struct networkAdapterConfiguration wlan;    //!< WLAN

	struct wifiConfigurationEx wifi; //!< Wifi configuration, extended version

	char notUsed[184];               //!< For later extensions
};
#endif // _DLL_EXPORT
// wpe library includes end

HIS_RETURN Acquisition_Global_Init(XIS_Init_Flags flags);
HIS_RETURN Acquisition_Global_Cleanup(void);

HIS_RETURN Acquisition_Init(HACQDESC *phAcqDesc,
					  DWORD dwChannelType, int nChannelNr,
					  BOOL bEnableIRQ, 
					  UINT Rows, UINT Columns, 
					  UINT dwSortFlags,
					  BOOL bSelfInit, BOOL bAlwaysOpen
					  );


HIS_RETURN Acquisition_SetCallbacksAndMessages(HACQDESC pAcqDesc,
					  HWND hWnd,
					  UINT dwErrorMsg, UINT dwLoosingFramesMsg,
					  void (CALLBACK *lpfnEndFrameCallback)(HACQDESC), 
					  void (CALLBACK *lpfnEndAcqCallback)(HACQDESC)
					  );
HIS_RETURN Acquisition_EnumSensors(UINT *pdwNumSensors, BOOL bEnableIRQ, BOOL bAlwaysOpen);
HIS_RETURN Acquisition_GetNextSensor(ACQDESCPOS *Pos, 
					  HACQDESC *phAcqDesc);
HIS_RETURN Acquisition_GetCommChannel(HACQDESC pAcqDesc, UINT *pdwChannelType, int *pnChannelNr);
HIS_RETURN Acquisition_DefineDestBuffers(HACQDESC pAcqDesc, unsigned short *pProcessedData, UINT nFrames, UINT nRows, UINT nColumns);
HIS_RETURN Acquisition_Acquire_Image(HACQDESC pAcqDesc, UINT dwFrames, UINT dwSkipFrms, UINT dwOpt, unsigned short *pwOffsetData, DWORD *pdwGainData, DWORD *pdwPxlCorrList);
HIS_RETURN Acquisition_Acquire_Image_Ex(HACQDESC hAcqDesc, UINT dwFrames, UINT dwSkipFrms, UINT dwOpt, 
									  unsigned short *pwOffsetData, UINT dwGainFrames, unsigned short *pwGainData,
									  unsigned short *pwGainAvgData, DWORD *pdwGainData, DWORD *pdwPxlCorrList);
HIS_RETURN Acquisition_Abort(HACQDESC hAcqDesc);
HIS_RETURN Acquisition_SetFPGACameraMode(HACQDESC hAcqDesc, FPGAType FPGACommand, BOOL bInverse);
HIS_RETURN Acquisition_SetCameraMode(HACQDESC hAcqDesc, UINT dwMode);
HIS_RETURN Acquisition_Acquire_OffsetImage(HACQDESC hAcqDesc, unsigned short *pOffsetData, UINT nRows, UINT nCols, UINT nFrames);
HIS_RETURN Acquisition_Acquire_OffsetImage_Ex(HACQDESC hAcqDesc, unsigned short *pOffsetData, UINT nRows, UINT nCols, UINT nFrames, UINT dwOpt);
HIS_RETURN Acquisition_Acquire_GainImage(HACQDESC hAcqDesc, WORD *pOffsetData, DWORD *pGainData, UINT nRows, UINT nCols, UINT nFrames);
HIS_RETURN Acquisition_Acquire_GainImage_Ex(HACQDESC hAcqDesc, WORD *pOffsetData, DWORD *pGainData, UINT nRows, UINT nCols, UINT nFrames, UINT dwOpt);
HIS_RETURN Acquisition_CreateGainMap(WORD *pGainData, WORD *pGainAVG, int nCount, int nFrame );
HIS_RETURN Acquisition_CreatePixelMap(WORD *pData, int nDataRows, int nDataColumns, int *pCorrList, int *nCorrListSize);
HIS_RETURN Acquisition_DoOffsetCorrection(WORD *pSource, WORD *pDest, WORD *pOffsetData, int nCount);
HIS_RETURN Acquisition_DoOffsetGainCorrection(WORD *pSource, WORD *pDest, WORD *pOffsetData, DWORD *pGainData, int nCount);
HIS_RETURN Acquisition_DoOffsetGainCorrection_Ex(WORD *pSource, WORD *pDest, WORD *pOffsetData, WORD *pGainData, 
															WORD *pGainAVG, int nCount, int nFrame);	//	19.09.02

HIS_RETURN Acquisition_DoOffsetCorrection32(unsigned long *pSource, unsigned long *pDest, unsigned long *pOffsetData, int nCount); // val 20070124
HIS_RETURN Acquisition_DoOffsetGainCorrection32(unsigned long *pSource, unsigned long *pDest, unsigned long *pOffsetData, unsigned long *pGainData, int nCount);// val 20070124
HIS_RETURN Acquisition_DoOffsetGainCorrection_Ex32(unsigned long *pSource, unsigned long *pDest, unsigned long *pOffsetData, unsigned long *pGainData,
							unsigned long *pGainAVG, int nCount, int nFrame); // val 20070124
HIS_RETURN Acquisition_CreateGainMap32(unsigned long *pGainData, unsigned long *pGainAVG, int nCount, int nFrame );


HIS_RETURN Acquisition_DoPixelCorrection(WORD *pData, int *pCorrList);
HIS_RETURN Acquisition_IsAcquiringData(HACQDESC hAcqDesc);
HIS_RETURN Acquisition_Close(HACQDESC hAcqDesc);
HIS_RETURN Acquisition_CloseAll();
HIS_RETURN Acquisition_SetReady(HACQDESC hAcqDesc, BOOL bFlag);
HIS_RETURN Acquisition_GetReady(HACQDESC hAcqDesc);
HIS_RETURN Acquisition_GetErrorCode(HACQDESC hAcqDesc, DWORD *dwHISError, DWORD *dwBoardError);
HIS_RETURN Acquisition_GetConfiguration(HACQDESC hAcqDesc, 
					UINT *dwFrames, UINT *dwRows, UINT *dwColumns, UINT *dwDataType,
					UINT *dwSortFlags, BOOL *bIRQEnabled, DWORD *dwAcqType, DWORD *dwSystemID,
					DWORD *dwSyncMode, DWORD *dwHwAccess);
HIS_RETURN Acquisition_GetIntTimes(HACQDESC hAcqDesc, double *dblIntTime, int *nIntTimes);
HIS_RETURN Acquisition_GetWinHandle(HACQDESC hAcqDesc, HWND *hWnd);
HIS_RETURN Acquisition_GetActFrame(HACQDESC hAcqDesc, DWORD *dwActAcqFrame, DWORD *dwActSecBuffFrame);
#ifdef XIS_OS_64
HIS_RETURN Acquisition_SetAcqData(HACQDESC hAcqDesc, void *AcqData);
HIS_RETURN Acquisition_GetAcqData(HACQDESC hAcqDesc, void **VoidAcqData);
#else
HIS_RETURN Acquisition_SetAcqData(HACQDESC hAcqDesc, DWORD dwAcqData);
HIS_RETURN Acquisition_GetAcqData(HACQDESC hAcqDesc, DWORD *dwAcqData);
#endif // XIS_OS_64
HIS_RETURN Acquisition_GetHwHeaderInfo(HACQDESC hAcqDesc, CHwHeaderInfo *pInfo);
HIS_RETURN Acquisition_SetFrameSync(HACQDESC hAcqDesc);
HIS_RETURN Acquisition_SetFrameSyncMode(HACQDESC hAcqDesc, DWORD dwMode);
HIS_RETURN Acquisition_SetTimerSync(HACQDESC hAcqDesc, DWORD *dwCycleTime);
HIS_RETURN Acquisition_AbortCurrentFrame(HACQDESC hAcqDesc);
HIS_RETURN Acquisition_SetCorrData(HACQDESC hAcqDesc, unsigned short *pwOffsetData, DWORD *pdwGainData, DWORD *pdwPxlCorrList);
HIS_RETURN Acquisition_SetCorrData_Ex(HACQDESC hAcqDesc, unsigned short *pwOffsetData, unsigned short *pwGainData, 
									unsigned short *pwGainAvgData, UINT nGainFrames,
									DWORD *pdwGainData, DWORD *pdwPxlCorrList);
HIS_RETURN Acquisition_GetCorrData(HACQDESC hAcqDesc, unsigned short **ppwOffsetData, DWORD **ppdwGainData, DWORD **ppdwPxlCorrList);
HIS_RETURN Acquisition_GetCorrData_Ex(HACQDESC hAcqDesc, unsigned short **ppwOffsetData, unsigned short **ppwGainData, 
									unsigned short **ppwGainAvgData, UINT **nGainFrames, DWORD **pdwGainData, DWORD **pdwPxlCorrList);
HIS_RETURN Acquisition_SetCameraGain(HACQDESC hAcqDesc, WORD wMode);
HIS_RETURN Acquisition_SetFrameSyncTimeMode(HACQDESC hAcqDesc, unsigned int uiMode, unsigned int dwDelayTime);

HIS_RETURN Acquisition_Acquire_GainImage_Ex_ROI(HACQDESC hAcqDesc, WORD *pOffsetData, DWORD *pGainData, UINT nRows, UINT nCols, UINT nFrames, UINT dwOpt,UINT uiULX,UINT uiULY,UINT uiBRX,UINT uiBRY,UINT uiMode);
HIS_RETURN Acquisition_Acquire_Image_PreloadCorr(HACQDESC hAcqDesc, UINT dwFrames, UINT dwSkipFrms, UINT dwOpt);
HIS_RETURN Acquisition_Acquire_OffsetImage_PreloadCorr(HACQDESC hAcqDesc, WORD *pwOffsetData, UINT nRows, UINT nColumns, UINT nFrames, UINT dwOpt);
HIS_RETURN Acquisition_GetHwHeader(HACQDESC hAcqDesc,unsigned char* pData,unsigned int uiSize);
HIS_RETURN Acquisition_Acquire_GainImage_Ex_ROI_PreloadCorr(HACQDESC hAcqDesc, DWORD *pGainData, UINT nRows, UINT nCols, UINT nFrames, UINT dwOpt,UINT uiULX,UINT uiULY,UINT uiBRX,UINT uiBRY,UINT uiMode);
HIS_RETURN Acquisition_Acquire_GainImage_PreloadCorr(HACQDESC hAcqDesc, DWORD *pGainData, UINT nRows, UINT nCols, UINT nFrames);

HIS_RETURN Acquisition_SetCameraBinningMode(HACQDESC hAcqDesc, WORD wMode);
HIS_RETURN Acquisition_GetCameraBinningMode(HACQDESC hAcqDesc, WORD* wMode);
HIS_RETURN Acquisition_ResetFrameCnt(HACQDESC hAcqDesc);
HIS_RETURN Acquisition_GetLatestFrameHeader(HACQDESC hAcqDesc, CHwHeaderInfo *pInfo, CHwHeaderInfoEx *pInfoEx);
HIS_RETURN Acquisition_GetHwHeaderInfoEx(HACQDESC hAcqDesc, CHwHeaderInfo *pInfo ,CHwHeaderInfoEx *pInfoEx);
HIS_RETURN Acquisition_SetCameraTriggerMode(HACQDESC hAcqDesc, WORD wMode);
HIS_RETURN Acquisition_GetCameraTriggerMode(HACQDESC hAcqDesc, WORD *wMode);

HIS_RETURN Acquisition_SetRotationAngle(HACQDESC hAcqDesc, long lRotAngle); // FG-E only can be -90 | 0 | 90
HIS_RETURN Acquisition_GetRotationAngle(HACQDESC hAcqDesc, long* lRotAngle);


HIS_RETURN Acquisition_GbIF_Init(					HACQDESC *phAcqDesc,
													int nChannelNr,
													BOOL bEnableIRQ, 
													UINT uiRows, UINT uiColumns, 
													BOOL bSelfInit, BOOL bAlwaysOpen,
													long lInitType, 
													GBIF_STRING_DATATYPE* ucAddress
												  );

HIS_RETURN Acquisition_GbIF_GetDeviceList(GBIF_DEVICE_PARAM* pGBIF_DEVICE_PARAM, int nDeviceCnt);
HIS_RETURN Acquisition_GbIF_GetDevice(GBIF_STRING_DATATYPE* ucAddress, DWORD dwAddressType, GBIF_DEVICE_PARAM* pDevice);
HIS_RETURN Acquisition_GbIF_GetDeviceCnt(long* plNrOfboards);

HIS_RETURN Acquisition_GbIF_SetConnectionSettings(	GBIF_STRING_DATATYPE* cMAC,
													unsigned long ulBootOptions,
													GBIF_STRING_DATATYPE* cDefIP,
													GBIF_STRING_DATATYPE* cDefSubNetMask,
													GBIF_STRING_DATATYPE* cStdGateway
												);

HIS_RETURN Acquisition_GbIF_GetConnectionSettings(	GBIF_STRING_DATATYPE* ucMAC,
													unsigned long* ulBootOptions,
													GBIF_STRING_DATATYPE* ucDefIP,
													GBIF_STRING_DATATYPE* ucDefSubNetMask,
													GBIF_STRING_DATATYPE* ucStdGateway
												);

HIS_RETURN Acquisition_GbIF_GetPacketDelay(			HACQDESC hAcqDesc,
													long* lPacketdelay
												);

HIS_RETURN Acquisition_GbIF_SetPacketDelay(			HACQDESC hAcqDesc,
													long lPacketdelay
											);

HIS_RETURN Acquisition_GbIF_ForceIP(				GBIF_STRING_DATATYPE* cMAC,
													GBIF_STRING_DATATYPE* cDefIP,
													GBIF_STRING_DATATYPE* cDefSubNetMask,
													GBIF_STRING_DATATYPE* cStdGateway
									);

HIS_RETURN Acquisition_GbIF_GetFilterDrvState(		HACQDESC hAcqDesc
											  );

HIS_RETURN Acquisition_GbIF_CheckNetworkSpeed(	HACQDESC hAcqDesc, WORD* wTiming, long* lPacketDelay, long lMaxNetworkLoadPercent);
HIS_RETURN Acquisition_GbIF_GetDetectorProperties(HACQDESC hAcqDesc, GBIF_Detector_Properties* pDetectorProperties);
HIS_RETURN Acquisition_GbIF_GetDeviceParams(HACQDESC hAcqDesc, GBIF_DEVICE_PARAM* pDevice);

HIS_RETURN Acquisition_GbIF_GetVersion(int *pMajor, int* pMinor, int *pRelease, char *pStrVersion, int iStrLength);
HIS_RETURN Acquisition_GbIF_DiscoverDetectors();
HIS_RETURN Acquisition_GbIF_DiscoveredDetectorCount(long *pDeviceCount);
HIS_RETURN Acquisition_GbIF_DiscoveredDetectorByIndex(long lIndex, GBIF_DEVICE_PARAM *pDevice);
HIS_RETURN Acquisition_GbIF_SetDiscoveryTimeout(long timeout);

HIS_RETURN Acquisition_wpe_GetVersionNEW(int * major, int * minor, int * release, int * build); // deprecated!
HIS_RETURN Acquisition_wpe_GetVersionEx(int *pMajor, int* pMinor, int *pRelease, char *pStrVersion, int iStrLength);
HIS_RETURN Acq_WPE_Init();

HIS_RETURN Acquisition_wpe_ForceIP(const char* macAddress, struct networkConfiguration* config, int port, int *isAnswered);
HIS_RETURN Acquisition_wpe_ChangeNetworkConfig(const char * ipAddress, int configIndex, struct networkConfiguration *config);
HIS_RETURN Acquisition_wpe_FillDefaultNetworkConfiguration(struct networkConfiguration *config);

// FTP specific
HIS_RETURN Acquisition_FTP_InitSession(HACQDESC hAcqDesc, XislFtpSession *session);
HIS_RETURN Acquisition_FTP_CloseSession(XislFtpSession session);
HIS_RETURN Acquisition_GetMissedImageCount(XislFtpSession session, UINT *count);
HIS_RETURN Acquisition_OpenMissedImage(XislFtpSession session, UINT index, XislFileHandle *fileHandle);

// Common (multi-purpose) file handling
HIS_RETURN Acquisition_GetFileInfo(XislFileHandle fileHandle, XislFileInfo *fileInfo);
HIS_RETURN Acquisition_LoadFile(XislFileHandle fileHandle, unsigned char **buffer);
HIS_RETURN Acquisition_DeleteFile(XislFileHandle fileHandle);
HIS_RETURN Acquisition_CloseFile(XislFileHandle fileHandle);

HIS_RETURN Acq_wpe_LoadCorrectionImageToBuffer(HACQDESC hAcqDesc, const char* pccCorrectionFilePath, ProcScriptOperation Operation);
HIS_RETURN Acq_wpe_DualEnergy_LoadCorrectionImageToBuffer(HACQDESC hAcqDesc, const char* pccCorrectionFilePath, const ProcScriptOperation Operation);


HIS_RETURN Acquisition_AcknowledgeImage(HACQDESC hAcqDesc, const char *tag);

HIS_RETURN Acquisition_wpe_SetUniqueImageTag(HACQDESC hAcqDesc, const char * imageTag);

HIS_RETURN Acquisition_Reset_OnboardOptions(HACQDESC hAcqDesc);
HIS_RETURN Acquisition_Set_OnboardOptionsPostOffset(HACQDESC hAcqDesc, BOOL bNoOnboardCorr, BOOL bSendPreviewFrist, BOOL bSendFULLFirst, BOOL bEnableAckFirst, BOOL bEnableAckSecond, BOOL bEnableOffsetFirst, BOOL bEnablePostOffsetCorr, BOOL bGain, BOOL bPixel);
HIS_RETURN Acquisition_Set_OnboardOptionsPostOffsetEx(HACQDESC hAcqDesc, BOOL bNoOnboardCorr, BOOL bSendPreviewFrist, BOOL bSendFULLFirst, BOOL bEnableAckFirst, BOOL bEnableAckSecond, BOOL bEnableOffsetFirst, BOOL bEnablePostOffsetCorr, BOOL bGain, BOOL bPixel, BOOL bStoreOffsetToSD);
HIS_RETURN Acquisition_Set_OnboardOptionsDualEnergy(HACQDESC hAcqDesc, BOOL bEnablePreviewFirst, BOOL bEnablePreviewSecond, BOOL bEnablePreviewOffset, BOOL bEnableOffset, BOOL bOnboardPxlCorr, BOOL bEnableAckFirst, BOOL bEnableFirst, BOOL bEnableAckSecond, BOOL bEnableSecond, BOOL bEnableAckThird, BOOL bEnableAckFourth, OnboardBinningMode ePreviewBinningMode);


HIS_RETURN Acquisition_wpe_SetMaxOnboardCorrValue(HACQDESC hAcqDesc, unsigned short usMax, unsigned short usReplace);

HIS_RETURN Acquisition_Set_OnboardOffsetImageAcquisition(HACQDESC hAcqDesc, BOOL bEnable, BOOL bSend, BOOL bStoreSD);
HIS_RETURN Acquisition_Set_OnboardOptions(HACQDESC hAcqDesc, BOOL bStoreSD, BOOL bOffset, BOOL bGain, BOOL bPixel);

HIS_RETURN Acquisition_ActivateServiceMode(HACQDESC hAcqDesc, BOOL bActivate);

HIS_RETURN Acquisition_SetCameraROI(HACQDESC hAcqDesc, unsigned short usActivateGrp); // val 2010-05-12
HIS_RETURN Acquisition_GetCameraROI(HACQDESC hAcqDesc, unsigned short *usActivateGrp); // val 2010-05-17

HIS_RETURN Acquisition_SetTriggerOutSignalOptions(HACQDESC hAcqDesc,unsigned short usTiggerOutSignalMode,
																	unsigned short usEP_SeqLength, 
																	unsigned short usEP_FirstBrightFrm,
																	unsigned short usEP_LastBrightFrm,
																	unsigned short usEP_Delay1,
																	unsigned short usEP_Delay2,
																	unsigned short usDDD_Delay,
																	int iTriggerOnRisingEdgeEnable,
																	int iSaveAsDefault
																	); // val 2010-05-12

// mk 2013-04-19:
HIS_RETURN Acquisition_wpe_GetVersion(int * major, int * minor, int * release, int * build);
HIS_RETURN Acquisition_wpe_getAvailableSystems(struct discoveryReply * reply, int * numDevices, int timeout,int port);

HIS_RETURN Acquisition_wpe_GetNetworkConfigs(const char * ipAddress, struct networkConfiguration * configs, int * arrayLength, int * activeConfig);
HIS_RETURN Acquisition_wpe_ActivateNetworkConfig(const char * ipAddress, int configIndex);

HIS_RETURN Acquisition_wpe_GetErrorCode(void);
HIS_RETURN Acquisition_wpe_GetErrorCodeEx(char *pBuffer, long len);

// mk 2013-04-19 end
HIS_RETURN Acquisition_GetTriggerOutStatus(HACQDESC hAcqDesc, int* iTriggerStatus); // 2013-04-22 Val GetTriggerStatus GbIF 

HIS_RETURN Acquisition_SetCameraFOVMode(HACQDESC hAcqDesc, WORD wMode); // 2013-07-03 val R&F Field Of View 
HIS_RETURN Acquisition_GetCameraFOVMode(HACQDESC hAcqDesc, WORD* wMode); // 2013-07-03 val R&F Field Of View 

HIS_RETURN Acquisition_wpe_ReadCameraRegisters(const char * ipAddress, unsigned long * buffer);
HIS_RETURN Acquisition_xrpd_ReadCameraRegisters(HACQDESC hAcqDesc, unsigned long * buffer);
HIS_RETURN Acquisition_wpe_GetExamFlag(const char * ipAddress, unsigned long *pExamFlag);
HIS_RETURN Acquisition_xrpd_GetExamFlag(HACQDESC hAcqDesc, unsigned long *pExamFlag);
HIS_RETURN Acq_wpe_SystemControl(const char * ipAddress, XRpad_SystemControlEnum eAction);
HIS_RETURN Acq_wpe_SetImageTransferInterface(const char *ipAddress, XRpad_DataInterfaceControlEnum eInterface);
HIS_RETURN Acq_wpe_GetSystemInformation(const char * ipAddress, char *buffer, int bufferLen);

HIS_RETURN Acquisition_GetFTPFile(const char * ipAddress, const char * filename, void ** databuffer, long * filesize); //2013-07-17 mk
HIS_RETURN Acquisition_FreeFTPFileBuffer(void * databuffer); //2013-09-19 mv
HIS_RETURN Acquisition_SetFTPFile(const char * ipAddress, const char * filename, void * databuffer, long filesize); //2013-07-17 mk

HIS_RETURN Acquisition_GetVersion(int * major, int * minor, int * release, int * build);

HIS_RETURN Acquisition_GetProvidedEnhancedFeatures(HACQDESC hAcqDesc, unsigned int* uiEnhancesFeatures);
HIS_RETURN Acquisition_Set_OnboardOptionPreview(HACQDESC hAcqDesc, BOOL bEnablePreview, BOOL bPreviewOptionSendFull, OnboardBinningMode eMode, unsigned int uiSelectedScript);
HIS_RETURN Acquisition_IsPreviewImage(HACQDESC hAcqDesc, unsigned int* uiIsPreview);
HIS_RETURN Acquisition_SetPhototimedParams(HACQDESC hAcqDesc, unsigned short usNrOfScrubs, unsigned short usMaxDelay);
HIS_RETURN Acquisition_SetDualEnergyParams(HACQDESC hAcqDesc, unsigned short usNrOfScrubs, unsigned short usMaxDelay);

HIS_RETURN Acquisition_EnableLogging(BOOL onOff);
HIS_RETURN Acquisition_SetLogLevel(XislLoggingLevels xislLogLvl);
HIS_RETURN Acquisition_GetLogLevel(XislLoggingLevels *xislLogLvl);
HIS_RETURN Acquisition_TogglePerformanceLogging(BOOL onOff);
HIS_RETURN Acquisition_SetLogOutput(const char* filePath, BOOL consoleOnOff);
HIS_RETURN Acquisition_SetFileLogging(const char* filename, BOOL enableLogging);
HIS_RETURN Acquisition_SetConsoleLogging(BOOL enableConsole);

HIS_RETURN Acquisition_GetXISFileBufferSize(size_t* pFileSize, UINT dwRows, UINT dwColumns, UINT dwFrames, BOOL uiOnboardFileHeader,XIS_FileType filetype);
HIS_RETURN Acquisition_CreateOnboardPixelMaskFrom16BitPixelMask(unsigned short* uspPixelMaskSrc, DWORD dwRows, DWORD dwColumns, unsigned char* bpOnboardPixelMask);
HIS_RETURN Acquisition_CreateXISFileInMemory(void* pMemoryFileBuffer, void*pDataBuffer, UINT dwRows, UINT dwColumns, UINT dwFrames, BOOL uiOnboardFileHeader, XIS_FileType filetype);
HIS_RETURN Acquisition_SaveFile(const char *filename, void* pImageBuffer, UINT dwRows, UINT dwColumns, UINT dwFrames , BOOL uiOnboardFileHeader, XIS_FileType usTypeOfNumbers);
HIS_RETURN Acquisition_SaveRawData(const char *filename, const unsigned char *buffer, size_t bufferSize);
HIS_RETURN Acquisition_LoadXISFileToMemory(const char *filename, void* pMemoryFileBuffer, size_t bufferSize);

HIS_RETURN Acquisition_SetDACOffsetFloorValueByMode(HACQDESC hAcqDesc, unsigned int uiMode);
HIS_RETURN Acquisition_SetDACOffsetFloorValueInFlash(HACQDESC hAcqDesc, unsigned int uiMode, WORD wValue);
HIS_RETURN Acquisition_GetDACOffsetFloorValueFromFlash(HACQDESC hAcqDesc, unsigned int uiMode, WORD* pwValue);
HIS_RETURN Acquisition_GetDetectorProperties(HACQDESC hAcqDesc, GBIF_Detector_Properties* pDetectorProperties);
HIS_RETURN Acquisition_SetDACoffset(HACQDESC hAcqDesc, WORD wDACoffsetValue);
HIS_RETURN Acquisition_SetDACoffsetBinningFPS(HACQDESC hAcqDesc, WORD wBinningMode, double dblFps, WORD *pwValueToFPGA);
HIS_RETURN Acquisition_Enable_EMI_Data_Readout(HACQDESC hAcqDesc, unsigned int uiOnOff);
HIS_RETURN Acquisition_GetGridSensorStatus(HACQDESC hAcqDesc, unsigned int *uiStatus);
HIS_RETURN Acquisition_GetConnectionStatus(HACQDESC hAcqDesc);
HIS_RETURN Acquisition_Set_FPGA_Power_Mode(HACQDESC hAcqDesc, unsigned int uiMode);
HIS_RETURN Acquisition_SetTailTimeforTriggerMode(HACQDESC hAcqDesc, unsigned short usTailTime, XIS_DetectorTriggerMode eTriggerMode);

HIS_RETURN Acquisition_SetEventCallback(HACQDESC hAcqDesc, XIS_EventCallback EventCallback, void *userData);
HIS_RETURN Acquisition_DisableEventCallback(HACQDESC hAcqDesc);
HIS_RETURN Acquisition_ResetOnboardShockEvent(HACQDESC hAcqDesc, unsigned int latestShock_Timestamp);

HIS_RETURN Acquisition_SetSDCardForceFsck(HACQDESC hAcqDesc);
HIS_RETURN Acquisition_AckSDCardForceFsck(HACQDESC hAcqDesc);
HIS_RETURN Acquisition_AckSDCardForceFsckError(HACQDESC hAcqDesc);

HIS_RETURN Acquisition_GetSDCardInfo(HACQDESC hAcqDesc, unsigned int *total, unsigned int *avail);
HIS_RETURN Acquisition_SetFakeTemperature(HACQDESC hAcqDesc, BOOL bEnableFakeMode, int iFakeTemperature);
HIS_RETURN Acquisition_IdentifyDevice(HACQDESC hAcqDesc);
HIS_RETURN Acquisition_Resend_All_Messages(HACQDESC hAcqDesc);
HIS_RETURN Acquisition_GetLocation(HACQDESC hAcqDesc, unsigned int *location);
HIS_RETURN Acquisition_GetNetwork(HACQDESC hAcqDesc, unsigned int *network);
HIS_RETURN Acquisition_GetNetworkSpeed(HACQDESC hAcqDesc, unsigned int *network);
HIS_RETURN Acquisition_SetNetworkSpeed(HACQDESC hAcqDesc, unsigned int network);
HIS_RETURN Acquisition_SetIdleTimeout(HACQDESC hAcqDesc, unsigned short timeout);
HIS_RETURN Acquisition_SetChargeMode(HACQDESC hAcqDesc, unsigned char charge_mode);
HIS_RETURN Acquisition_VerifyGenuineness(HACQDESC hAcqDesc, char (*msg)[128], size_t *msg_len, unsigned char (*md)[20]);
HIS_RETURN Acquisition_SetPrivateKey(HACQDESC hAcqDesc, unsigned char (*key_old)[64], unsigned char (*key_new)[64]);
HIS_RETURN Acquisition_SetTemperatureTimeout(HACQDESC hAcqDesc, unsigned short timeout);
HIS_RETURN Acquisition_ResetTemperatureTimeout(HACQDESC hAcqDesc);
HIS_RETURN Acquisition_SetTemperatureThresholds(HACQDESC hAcqDesc, unsigned int threshold_warning, unsigned int threshold_critical);
HIS_RETURN Acquisition_GetTemperatureThresholds(HACQDESC hAcqDesc, unsigned int *threshold_warning, unsigned int *threshold_critical);
HIS_RETURN Acquisition_GetTemperature(HACQDESC hAcqDesc, unsigned int (*current_value)[8]);
HIS_RETURN Acquisition_GetBatteryStatus(HACQDESC hAcqDesc, XRpad_BatteryStatus *batteryStatus);
HIS_RETURN Acquisition_Get_Current_Voltage(HACQDESC hAcqDesc, DETECTOR_CURRENT_VOLTAGE *pstructCurrentVoltage);
HIS_RETURN Acquisition_CreateFakeShockWarningLevel(HACQDESC hAcqDesc);
HIS_RETURN Acquisition_CreateFakeShockCriticalLevel(HACQDESC hAcqDesc);
HIS_RETURN Acquisition_FactoryResetShock(HACQDESC hAcqDesc);
HIS_RETURN Acquisition_SetSystemTime(HACQDESC hAcqDesc, const char *cDateTime);
HIS_RETURN Acquisition_GetPowerstate(HACQDESC hAcqDesc, unsigned int *powerstate);
HIS_RETURN Acquisition_GetAutoPowerOnLocations(HACQDESC hAcqDesc, unsigned int *autopoweronlocations);
HIS_RETURN Acquisition_SetAutoPowerOnLocations(HACQDESC hAcqDesc, unsigned int autopoweronlocations);
HIS_RETURN Acquisition_GetChargeMode(HACQDESC hAcqDesc, unsigned char* charge_mode_req, unsigned char* charge_mode_charger);
HIS_RETURN Acquisition_SetSDCardTimeout(HACQDESC hAcqDesc, unsigned short sdcard_timeout);
HIS_RETURN Acquisition_GetSDCardTimeout(HACQDESC hAcqDesc, unsigned short *sdcard_timeout);
HIS_RETURN Acquisition_GetVersionInfo(HACQDESC hAcqDesc, XRpad_VersionInfo *versionInfo);
HIS_RETURN Acquisition_GetIpAdress(HACQDESC hAcqDesc, const char **ipAddress);
HIS_RETURN Acquisition_Test_SDCardPerformance(HACQDESC hAcqDesc, unsigned int buffersize,
                                              double *wbitrate, unsigned int *wmicroseconds,
                                              double *rbitrate, unsigned int *rmicroseconds);
HIS_RETURN Acquisition_Enable_TestPattern(HACQDESC hAcqDesc, unsigned int uiOnOff);
HIS_RETURN Acquisition_GetWLAN_CountryCode(HACQDESC hAcqDesc, char *wlan_cc_str);
HIS_RETURN Acquisition_SetWLAN_CountryCode(HACQDESC hAcqDesc, const char * wlan_cc_value);
HIS_RETURN Acquisition_GetWLAN_ChannelList(HACQDESC hAcqDesc, char * wlan_channel_list_str, size_t wlan_channel_list_len);
HIS_RETURN Acquisition_DisableSyslogSaving(HACQDESC hAcqDesc);
HIS_RETURN Acquisition_SetAEDOptions(HACQDESC hAcqDesc, unsigned short usMode, unsigned short usOffsetImageDelay, unsigned short usSelectReadoutSection);
HIS_RETURN Acquisition_GetAutoDeepSleepIdleLocations(HACQDESC hAcqDesc, unsigned int *autoDeepSleepLocations, unsigned int *autoIdleLocations);
HIS_RETURN Acquisition_SetAutoDeepSleepIdleLocations(HACQDESC hAcqDesc, unsigned int autoDeepSleepLocations, unsigned int autoIdleLocations);
HIS_RETURN Acquisition_GbIF_SetTransmissionMode(HACQDESC hAcqDesc, XIS_Transmission_Mode transmissionMode);
HIS_RETURN Acquisition_GbIF_GetTransmissionMode(HACQDESC hAcqDesc, XIS_Transmission_Mode *pTransmissionMode);

HIS_RETURN Acquisition_SetDefaultBootConfiguration(HACQDESC hAcqDesc, int default_boot_cfg);
HIS_RETURN Acquisition_GetDefaultBootConfiguration(HACQDESC hAcqDesc, int* default_boot_cfg);

HIS_RETURN Acquisition_xrpd_GetDetectorType(HACQDESC hAcqDesc, char* response_buffer, size_t response_buffer_len);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

//error codes


/** No error @ingroup enum **/
#define HIS_ALL_OK							0
/** Memory couldn't be allocated. @ingroup enum **/
#define HIS_ERROR_MEMORY					1
/** Unable to initialize board. @ingroup enum **/
#define HIS_ERROR_BOARDINIT					2
/** Got a time out. May be no detector present. @ingroup enum **/
#define HIS_ERROR_NOCAMERA					3
/** Your correction files do not have a proper size. @ingroup enum **/
#define HIS_ERROR_CORRBUFFER_INCOMPATIBLE	4
/** Acquisition is already running. @ingroup enum **/
#define HIS_ERROR_ACQ_ALREADY_RUNNING		5
/** Got a time out from hardware. @ingroup enum **/
#define HIS_ERROR_TIMEOUT					6
/** Acquisition descriptor invalid. @ingroup enum **/
#define HIS_ERROR_INVALIDACQDESC			7
/** Unable to find VxD. @ingroup enum **/
#define HIS_ERROR_VXDNOTFOUND				8
/** Unable to open VxD. @ingroup enum **/
#define HIS_ERROR_VXDNOTOPEN				9
/** Unknown error during VxD loading. @ingroup enum **/
#define HIS_ERROR_VXDUNKNOWNERROR			10
/** VxD Error: GetDmaAddr failed. @ingroup enum **/
#define HIS_ERROR_VXDGETDMAADR				11
/** An unexpected acquisition abort occurred. @ingroup enum **/
#define HIS_ERROR_ACQABORT					12
/** error occurred during data acquisition. @ingroup enum **/
#define HIS_ERROR_ACQUISITION				13
/** Unable to register interrupt. @ingroup enum **/
#define HIS_ERROR_VXD_REGISTER_IRQ			14
/** Register status address failed. @ingroup enum **/
#define HIS_ERROR_VXD_REGISTER_STATADR		15
/** Getting version of operating system failed. @ingroup enum **/
#define HIS_ERROR_GETOSVERSION				16
/** Can not set frame sync. @ingroup enum **/
#define HIS_ERROR_SETFRMSYNC				17
/** Can not set frame sync mode. @ingroup enum **/
#define HIS_ERROR_SETFRMSYNCMODE			18
/** Can not set timer sync. @ingroup enum **/
#define HIS_ERROR_SETTIMERSYNC				19
/** Invalid function call. @ingroup enum **/
#define HIS_ERROR_INVALID_FUNC_CALL			20
/** Aborting current frame failed. @ingroup enum **/
#define HIS_ERROR_ABORTCURRFRAME			21
/** Getting hardware header failed. @ingroup enum **/
#define HIS_ERROR_GETHWHEADERINFO			22
/** Hardware header is invalid. @ingroup enum **/
#define HIS_ERROR_HWHEADER_INV				23
/** Setting line trigger mode failed. @ingroup enum **/
#define HIS_ERROR_SETLINETRIG_MODE			24
/** Writing data failed. @ingroup enum **/
#define HIS_ERROR_WRITE_DATA				25
/** Reading data failed. @ingroup enum **/
#define HIS_ERROR_READ_DATA					26
/** Setting baud rate failed. @ingroup enum **/
#define HIS_ERROR_SETBAUDRATE				27
/** No acquisition descriptor available. @ingroup enum **/
#define HIS_ERROR_NODESC_AVAILABLE			28
/** Buffer space not sufficient. @ingroup enum **/
#define HIS_ERROR_BUFFERSPACE_NOT_SUFF		29
/** Setting detector mode failed. @ingroup enum **/
#define HIS_ERROR_SETCAMERAMODE				30
/** Frame invalid. @ingroup enum **/
#define HIS_ERROR_FRAME_INV					31
/** System to slow. @ingroup enum **/
#define HIS_ERROR_SLOW_SYSTEM				32
/** Error during getting number of boards. @ingroup enum **/
#define HIS_ERROR_GET_NUM_BOARDS			33
/** Communication channel already opened by another process. @ingroup enum **/
#define HIS_ERROR_HW_ALREADY_OPEN_BY_ANOTHER_PROCESS	34
/** Error creating memory mapped file. @ingroup enum **/
#define HIS_ERROR_CREATE_MEMORYMAPPING				35
/** Error registering DMA address. @ingroup enum **/
#define HIS_ERROR_VXD_REGISTER_DMA_ADDRESS			36
/** Error registering static address. @ingroup enum **/
#define HIS_ERROR_VXD_REGISTER_STAT_ADDR			37
/** Unable to unmask interrupt. @ingroup enum **/
#define HIS_ERROR_VXD_UNMASK_IRQ					38
/** Unable to load driver. @ingroup enum **/
#define HIS_ERROR_LOADDRIVER						39
/** Function is not implemented. @ingroup enum **/
#define HIS_ERROR_FUNC_NOTIMPL						40
/** Unable to create memory mapping. @ingroup enum **/
#define HIS_ERROR_MEMORY_MAPPING					41
/** Could not create Mutex. @ingroup enum **/
#define HIS_ERROR_CREATE_MUTEX						42
/** Error starting the acquisition. @ingroup enum **/
#define HIS_ERROR_ACQ								43
/** Acquisition descriptor is not local. @ingroup enum **/
#define HIS_ERROR_DESC_NOT_LOCAL					44
/** Invalid Parameter. @ingroup enum **/
#define HIS_ERROR_INVALID_PARAM						45
/** Error during abort acquisition function. @ingroup enum **/
#define HIS_ERROR_ABORT								46
/** The wrong board is selected. @ingroup enum **/
#define HIS_ERROR_WRONGBOARDSELECT					47
/** Change of Detector Mode during Acquisition. @ingroup enum **/
#define HIS_ERROR_WRONG_CAMERA_MODE					48	 
/** The number of images for frame grabber onboard averaging must be 2 to the power of n. @ingroup enum **/
#define HIS_ERROR_AVERAGED_LOST						49	 
/** Parameter for (onboard) sorting not valid. @ingroup enum **/
#define HIS_ERROR_BAD_SORTING_PARAM					50	
/** Connection to Network Detector cannot be opened due to invalid IP address / MAC / Detector name. @ingroup enum **/
#define HIS_ERROR_UNKNOWN_IP_MAC_NAME				51	
/** Detector could not be found in the Subnet. @ingroup enum **/
#define HIS_ERROR_NO_BOARD_IN_SUBNET				52
/** Unable to open connection to Network Detector. @ingroup enum **/
#define HIS_ERROR_UNABLE_TO_OPEN_BOARD				53
/** Unable to close connection to Network Detector. @ingroup enum **/
#define HIS_ERROR_UNABLE_TO_CLOSE_BOARD				54
/** Unable to access the flash memory of Detector. @ingroup enum **/
#define HIS_ERROR_UNABLE_TO_ACCESS_DETECTOR_FLASH	55
/** No frame header received from Detector. @ingroup enum **/
#define HIS_ERROR_HEADER_TIMEOUT					56
/** Command not acknowledged. @ingroup enum **/
#define HIS_ERROR_NO_FPGA_ACK						57
/** Number of boards within network changed during broadcast. @ingroup enum **/
#define HIS_ERROR_NR_OF_BOARDS_CHANGED				58
/** Unable to set the exam flag. @ingroup enum **/
#define HIS_ERROR_SETEXAMFLAG						59
/** Error Function called with an illegal index number. @ingroup enum **/
#define HIS_ERROR_ILLEGAL_INDEX                     60
/** Error Function or function environment not correctly initialised. @ingroup enum **/
#define HIS_ERROR_NOT_INITIALIZED                   61
/** Error No detectors discovered yet. @ingroup enum **/
#define HIS_ERROR_NOT_DISCOVERED                    62
/** Error onbaord averaging failed. @ingroup enum **/
#define HIS_ERROR_ONBOARDAVGFAILED                  63
/** Error getting onboard offset. @ingroup enum **/
#define HIS_ERROR_GET_ONBOARD_OFFSET				64
/** Error CURL. @ingroup enum **/
#define HIS_ERROR_CURL                              65
/** Error setting onboard offset corr mode. @ingroup enum **/
#define HIS_ERROR_ENABLE_ONBOARD_OFFSET				66
/** Error setting onboard mean corr mode. @ingroup enum **/
#define HIS_ERROR_ENABLE_ONBOARD_MEAN				67
/** Error setting onboard gain corr mode. @ingroup enum **/
#define HIS_ERROR_ENABLE_ONBOARD_GAINOFFSET			68
/** Error setting onboard preview mode. @ingroup enum **/
#define HIS_ERROR_ENABLE_ONBOARD_PREVIEW			69
/** Error setting onboard binning mode. @ingroup enum **/
#define HIS_ERROR_SET_ONBOARD_BINNING				70
/** Error Loading image from SD to onboard buffer. @ingroup enum **/
#define HIS_ERROR_LOAD_COORECTIONIMAGETOBUFFER		71
/** Error Invalid pointer/buffer passed as parameter. @ingroup enum **/
#define HIS_ERROR_INVALIDBUFFERNR					72
/** Error Invalid SHOCKID. @ingroup enum **/
#define HIS_ERROR_INVALID_HANDLE                    73
/** Error Invalid filename, file already exists. @ingroup enum **/
#define HIS_ERROR_ALREADY_EXISTS                    74
/** Error Invalid filename type does not exist. @ingroup enum **/
#define HIS_ERROR_DOES_NOT_EXIST                    75
/** Error Invalid filename for image tag or log file. @ingroup enum **/
#define HIS_ERROR_OPEN_FILE                         76
/** Error Invalid filename for image tag or log file. @ingroup enum **/
#define HIS_ERROR_INVALID_FILENAME                  77
/** Error setting gbif discovery timeout. @ingroup enum **/
#define HIS_ERROR_SETDISCOVERYTIMEOUT               78
// DEXELA
#define HIS_ERROR_SERIALREAD						100
// DEXELA
#define HIS_ERROR_SERIALWRITE						101
// 1313
#define HIS_ERROR_SETDAC							102
// 1313
#define HIS_ERROR_SETADC							103
/** Error setting the onboard image tag. @ingroup enum **/
#define HIS_ERROR_SET_IMAGE_TAG						104
/** Error setting the onboard process script. @ingroup enum **/
#define HIS_ERROR_SET_PROC_SCRIPT					105
/** Error Image tag length exceeded 128char (including path: autosave/). @ingroup enum **/
#define HIS_ERROR_SET_IMAGE_TAG_LENGTH				106
/** Error retrieving the enhanced header. @ingroup enum **/
#define HIS_ERROR_RETRIEVE_ENHANCED_HEADER			107
/** Error enabling XRPD interrupts. @ingroup enum **/
#define HIS_ERROR_ENABLE_INTERRUPTS                 108
/** Error XRPD session Error. @ingroup enum **/
#define HIS_ERROR_XRPD_SESSION_ERROR                109
/** Error No interface to communicate event messages active. @ingroup enum **/
#define HIS_ERROR_XRPD_SET_EVENT                    110
/** Error No interface to communicate event messages active. @ingroup enum **/
#define HIS_ERROR_XRPD_NO_EVENT_INTERFACE           111
/** Error creating fake shock events. @ingroup enum **/
#define HIS_ERROR_XRPD_CREATE_FAKE_SHOCK_EVENT      112
/** Error retrieving the sd card info. @ingroup enum **/
#define HIS_ERROR_XRPD_GET_SDCARD_INFO              113
/** Error activating the fake temperatur mode on the detector. @ingroup enum **/
#define HIS_ERROR_XRPD_SET_TEMP_FAKE_MODE           114
/** Error the requested EMI readout mode was not reported by the detector. @ingroup enum **/
#define HIS_ERROR_EMI_NOT_SET                       115
/** Error retrieving the location info from the detector. @ingroup enum **/
#define HIS_ERROR_XRPD_NO_LOCATION                  116
/** Error setting the on detector idle timeout. @ingroup enum **/
#define HIS_ERROR_SET_IDLE_TIMEOUT                  117
/** Error setting the software requested charge mode. @ingroup enum **/
#define HIS_ERROR_SET_CHARGE_MODE                   118
/** Error creating critical level fake shock events. @ingroup enum **/
#define HIS_ERROR_XRPD_CREATE_FAKE_SHOCK_EVENT_CRIT 119
/** Error creating warning level fake shock events. @ingroup enum **/
#define HIS_ERROR_XRPD_CREATE_FAKE_SHOCK_EVENT_WARN 120
/** Error resetting the shock events to factory values. @ingroup enum **/
#define HIS_ERROR_XRPD_FACTORY_RESET_SHOCK_EVENT    121
/** Error getting the on detector LAN network speed. @ingroup enum **/
#define HIS_ERROR_XRPD_NO_NETWORK                   122
/** Error setting the on detector LAN network speed. @ingroup enum **/
#define HIS_ERROR_XRPD_SET_NETWORK                  123
/** Error verifying the private key for genuiness. @ingroup enum **/
#define HIS_ERROR_XRPD_VERIFY_GENUINENESS           124
/** Error setting the private key for genuiness. @ingroup enum **/
#define HIS_ERROR_XRPD_SET_PRIVATE_KEY              125
/** Error setting the temperature timeout. @ingroup enum **/
#define HIS_ERROR_XRPD_SET_TEMPERATURE_TIMEOUT      126
/** Error resetting the temperature timeout counter. @ingroup enum **/
#define HIS_ERROR_XRPD_RESET_TEMPERATURE_TIMEOUT    127
/** Error setting the temperature thresholds on the detector. @ingroup enum **/
#define HIS_ERROR_XRPD_SET_TEMPERATURE_THRESHOLDS   128
/** Error getting the temperature thresholds from the detector. @ingroup enum **/
#define HIS_ERROR_XRPD_GET_TEMPERATURE_THRESHOLDS   129
/** Error no eventcallback defined for irq messages. @ingroup enum **/
#define HIS_ERROR_XRPD_NO_EVENTCALLBACK_DEFINED     130
/** Error setting on detectors date and time. @ingroup enum **/
#define HIS_ERROR_XRPD_SET_DATE_TIME                131
/** Error triggering the resend of all current messages by the XRPD. @ingroup enum **/
#define HIS_ERROR_XRPD_RESEND_ALL_MSG               132
/** Error acknowledging the image. @ingroup enum **/
#define HIS_ERROR_ACKNOWLEDGE_IMAGE                 133
/** Error connecting to the on detector XRPD process. @ingroup enum **/
#define HIS_ERROR_XRPD_CONNECT                      134
/** Error resetting the shock event. @ingroup enum **/
#define HIS_ERROR_XRPD_RESET_SHOCK                  135
/** Error setting the power state on the detector. @ingroup enum **/
#define HIS_ERROR_XRPD_REQUEST_POWERSTATE           136
/** Error retrieving the auto power on locations from the detector. @ingroup enum **/
#define HIS_ERROR_XRPD_GET_AUTOPOWERONLOCATIONS     137
/** Error setting the auto power on locations on the detector. @ingroup enum **/
#define HIS_ERROR_XRPD_SET_AUTOPOWERONLOCATIONS     138
/** Error retrieving the requested charge mode from the detector. @ingroup enum **/
#define HIS_ERROR_GET_CHARGE_MODE                   139
/** Error requesting an fscheck on next boot. @ingroup enum **/
#define HIS_ERROR_XRPD_SET_FORCE_FSCK               140
/** Error setting the sd card timeout on the detector. @ingroup enum **/
#define HIS_ERROR_XRPD_SET_SDCARD_TIMEOUT           141
/** Error getting the sd card timeout from the detector. @ingroup enum **/
#define HIS_ERROR_XRPD_GET_SDCARD_TIMEOUT           142
/** Error not connected to on detector XRPD process. @ingroup enum **/
#define HIS_ERROR_MISSING_VERSION_INFORMATION       143
/** Error not connected to on detector XRPD process. @ingroup enum **/
#define HIS_ERROR_XRPD_NOT_CONNECTED                144
/** Error retrieving the SD card performance. @ingroup enum **/
#define HIS_ERROR_XRPD_SDCARDPERFORMANCE            145
/** Requested channel is already openend. @ingroup enum **/
#define HIS_ERROR_HW_BOARD_CHANNEL_ALREADY_USED		146
/** Error retrieving the Voltage or Current from detector. @ingroup enum **/
#define HIS_ERROR_XRPD_GET_CURRENT_VOLTAGE			147
/** Unable to set on detector CPU govenor. @ingroup enum **/
#define HIS_ERROR_XRPD_SET_CPUFREQ_GOVERNOR         148
/** Unable to get on detector WLAN country code info. @ingroup enum **/
#define HIS_ERROR_XRPD_GET_WLAN_CC                  149
/** Unable to get on detector WLAN country code. @ingroup enum **/
#define HIS_ERROR_XRPD_SET_WLAN_CC                  150
/** Unable to get on detector WLAN channel list info. @ingroup enum **/
#define HIS_ERROR_XRPD_GET_WLAN_ChannelList         151
/** Unable to disable the saving og the syslog. @ingroup enum **/
#define HIS_ERROR_XRPD_DISABLE_SYSLOG_SAVING        152
/** Could not set on detector packet delay. @ingroup enum **/
#define HIS_ERROR_SET_PACKET_DELAY                  153
/** Could not retrieve available systems. @ingroup enum **/
#define HIS_ERROR_GET_AVAILABLE_SYSTEMS              154
/** Error retrieving the deep-sleep idle change locations from the detector. @ingroup enum **/
#define HIS_ERROR_XRPD_GET_DEEPSLEEPIDLELOCATIONS    155
/** Error setting the deep-sleep idle change locations from the detector. @ingroup enum **/
#define HIS_ERROR_XRPD_SET_DEEPSLEEPIDLELOCATIONS    156
/** Function returned error since detector is in deep sleep and does not support the function in that state. @ingroup enum **/
#define HIS_ERROR_XRPD_DETECTOR_IN_DEEP_SLEEP        157
/** unable to get temp values from detector. @ingroup enum **/
#define HIS_ERROR_XRPD_GET_TEMP_VALUES               158
/** unable to get epc register. @ingroup enum **/
#define HIS_ERROR_XRPD_GET_EPC_REGISTER              159
/** unable to set header size. @ingroup enum **/
#define HIS_ERROR_SETHEADERSIZE                      160
/** unable to set register Timeout size. @ingroup enum **/
#define HIS_ERROR_SETREGISTERTIMEOUT                 161
/** unable to set epc register. @ingroup enum **/
#define HIS_ERROR_XRPD_SET_EPC_REGISTER              162
/** unable to communicate with battery. @ingroup enum **/
#define HIS_ERROR_XRPD_BATTERY_COM                   163
/** WSAStartup or WSACleanup failed. @ingroup enum **/
#define HIS_ERROR_WSA                                164
/** Function is not supported by your device or setup. @ingroup enum **/
#define HIS_ERROR_NOT_SUPPORTED                      165
/** Failed to enable live transmission mode. @ingroup enum **/
#define HIS_ERROR_TRANSMISSION_MODE                  166
/** The configuration/function call is conflicting with another option. @ingroup enum **/
#define HIS_ERROR_CONFLICT                           167
/** WLAN Restart failed. @ingroup enum **/
#define HIS_ERROR_WLAN_RESTART                       168
/** Unable to set event callback. Could be caused by multiple threads running the function in parallel. @ingroup enum **/
#define HIS_ERROR_SET_EVENT_CALLBACK                 169
/** Error getting default boot configuration @ingroup enum **/
#define HIS_ERROR_XRPD_GET_DEF_BOOT_CFG              170
/** Error setting default boot configuration @ingroup enum **/
#define HIS_ERROR_XRPD_SET_DEF_BOOT_CFG              171
/** Resetting the Zynq FPGA failed @ingroup enum **/
#define HIS_ERROR_RESET_ZYNQ                         172
/** Error getting the detector type @ingroup enum **/
#define HIS_ERROR_XRPD_GET_DET_TYPE                  174
/** Error getting the detector options @ingroup enum **/
#define HIS_ERROR_XRPD_GET_DET_OPTIONS               175
/** Error initializing the detector options data @ingroup enum **/
#define HIS_ERROR_INIT_DET_OPTIONS                   176

//sort definitions

#define HIS_SORT_NOSORT						0
#define HIS_SORT_QUAD						1
#define HIS_SORT_COLUMN						2
#define HIS_SORT_COLUMNQUAD					3
#define HIS_SORT_QUAD_INVERSE				4
#define HIS_SORT_QUAD_TILE					5
#define HIS_SORT_QUAD_TILE_INVERSE			6
#define HIS_SORT_QUAD_TILE_INVERSE_SCRAMBLE	7
#define HIS_SORT_OCT_TILE_INVERSE			8		//	1640 and 1620
#define HIS_SORT_OCT_TILE_INVERSE_BINDING	9		//	1680
#define HIS_SORT_OCT_TILE_INVERSE_DOUBLE	10		//	1620 reverse
#define HIS_SORT_HEX_TILE_INVERSE			11		//	1621 ADIC
#define HIS_SORT_HEX_CS						12		//	1620/1640 continous scan
#define HIS_SORT_12x1						13		//	12X1 Combo
#define HIS_SORT_14							14		//	
#define HIS_SORT_TOP_BOTTOM					15		//	2013-07-01 val 1717 RNF  full lines top row bottom row


//sequence acquisition options
#define HIS_SEQ_TWO_BUFFERS				0x1
#define HIS_SEQ_ONE_BUFFER				0x2
#define HIS_SEQ_AVERAGE					0x4
#define HIS_SEQ_DEST_ONE_FRAME			0x8
#define HIS_SEQ_COLLATE					0x10
#define HIS_SEQ_CONTINUOUS				0x100		
#define HIS_SEQ_LEAKAGE					0x1000
#define HIS_SEQ_NONLINEAR				0x2000
#define HIS_SEQ_AVERAGESEQ				0x4000	// sequence of averaged frames
#define HIS_SEQ_PREVIEW					0x8000


#define HIS_SYNCMODE_SOFT_TRIGGER			1
#define HIS_SYNCMODE_INTERNAL_TIMER			2
#define HIS_SYNCMODE_EXTERNAL_TRIGGER		3
#define HIS_SYNCMODE_FREE_RUNNING			4
#define HIS_SYNCMODE_AUTO_TRIGGER			8
#define HIS_SYNCMODE_EXTERNAL_TRIGGER_FG	16

#define HIS_CAMMODE_SETSYNC		0x8
#define HIS_CAMMODE_TIMEMASK	0x7
#define HIS_CAMMODE_FPGA		0x7F

#define HIS_BOARD_TYPE_NOONE					0x0
#define HIS_BOARD_TYPE_ELTEC					0x1
#define HIS_BOARD_TYPE_DIPIX					0x2
#define HIS_BOARD_TYPE_RS232					0x3
#define HIS_BOARD_TYPE_USB						0x4
//#define HIS_BOARD_TYPE_ELTEC_SCIURIUS			0x8
#define HIS_BOARD_TYPE_ELTEC_XRD_FGX			0x8
#define HIS_BOARD_TYPE_ELTEC_XRD_FGE_Opto		0x10
#define HIS_BOARD_TYPE_ELTEC_GbIF				0x20
#define HIS_BOARD_TYPE_ELTEC_WPE				0x40	// mk 2013-04-16 additional functions for wpe lib
#define HIS_BOARD_TYPE_ELTEC_EMBEDDED			0x60	// mk 2013-04-16 embedded is gbif and wpe

#define HIS_BOARD_TYPE_CMOS						0x100								// msi 2013-06-20 CMOS are all devices with CMOS FW (until now 1512, 13x13) / => HIS_BOARD_TYPE_DEXELA_1512 | define HIS_BOARD_TYPE_ELTEC_13x13
//#define HIS_BOARD_TYPE_ELTEC_13x13			0x200
#define HIS_BOARD_TYPE_ELTEC_13x13				0x320	// msi 2013-06-21 13x13 includes gbif and CMOS / => 0x200 | HIS_BOARD_TYPE_ELTEC_GbIF | HIS_BOARD_TYPE_CMOS
//#define HIS_BOARD_TYPE_DEXELA_1512CL			0x400
#define HIS_BOARD_TYPE_DEXELA_1512CL			0x500	// msi 2013-06-21 13x13 includes and CMOS / => 0x400 | HIS_BOARD_TYPE_CMOS



#define HIS_MAX_TIMINGS							0x8


#endif	//_ACQUISITION_H
