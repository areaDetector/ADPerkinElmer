#ifndef _ACQUISITION_H
#define _ACQUISITION_H

#define DATASHORT               2 //2 byte integer
#define DATALONG                4 //4 byte integer
#define DATAFLOAT               8 //8 byte double
#define DATASIGNED              16 // signed

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

typedef UINT ACQDESCPOS;

#ifdef __linux__
	typedef int HACQDESC;
#else
	// Windows
	typedef HANDLE HACQDESC;
    #ifdef _DLL_EXPORT
		#define _DLL_API __declspec(dllexport) 
    #else
		#define _DLL_API __declspec(dllimport) 
    #endif //_ACQDEFS_H

    #define HIS_RETURN _DLL_API UINT WINAPI
    
#endif


#define WINRESTSIZE					34
#define WINHARDWAREHEADERSIZE		32

#define DETEKTOR_DATATYPE_18BIT 16
#define MAX_GREY_VALUE_18BIT  262144

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


typedef struct
{
	WORD FileType;			// File ID (0x7000)
	WORD HeaderSize;		// Size of this file header in Bytes
	WORD HeaderVersion;		// yy.y
	ULONG FileSize;			// Size of the whole file in Bytes
	WORD ImageHeaderSize;	// Size of the image header in Bytes
	WORD ULX, ULY, BRX, BRY;// bounding rectangle of the image
	WORD NrOfFrames;		// self explanatory
	WORD Correction;		// 0 = none, 1 = offset, 2 = gain, 4 = bad pixel, (ored)
	double IntegrationTime;	// frame time in microseconds
	WORD TypeOfNumbers;		// short, long integer, float, signed/unsigned, inverted, 
							// fault map, offset/gain correction data, badpixel correction data
	BYTE x[WINRESTSIZE];		// fill up to 68 byte
} WinHeaderType;

typedef struct
{
	DWORD	dwPROMID;
	char	strProject[6];		// project / cam-nr
	char	strSystemused[3];	//
	char	strPrefilter[9];	// used filter
	float	fKVolt;				// 
	float	fAmpere;			//
	WORD	n_avframes;		// average count
		
} WinImageHeaderType;

typedef struct
{
	unsigned char	wTiming;		//	Timing und Triggermode
	unsigned char	wValue0;		// 
	unsigned char	wValue1;
	unsigned char	wValue2;
	unsigned char	wValue3;
	unsigned char	wValue4;
	unsigned char	wValue5;
	unsigned char	wValue6;
} FPGAType;					// 8 Byte werden bertragen


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
HIS_RETURN Acquisition_SetAcqData(HACQDESC hAcqDesc, DWORD dwAcqData);
HIS_RETURN Acquisition_GetAcqData(HACQDESC hAcqDesc, DWORD *dwAcqData);
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

// noch in doku einpflegen
HIS_RETURN Acquisition_DefineDestBuffersEx_32(HACQDESC hAcqDesc, void* pProcessedData, UINT nFrames, UINT nRows, UINT nColumns,UINT m_dwDataType);
HIS_RETURN Acquisition_Init_Ex(HACQDESC *phAcqDesc,
					  DWORD dwChannelType, int nChannelNr,
					  BOOL bEnableIRQ, 
					  UINT Rows, UINT Columns, 
					  UINT dwSortFlags,
					  BOOL bSelfInit, BOOL bAlwaysOpen,
					  BOOL bEnable32Bit);
HIS_RETURN Acquisition_SetCorrData_Ex_32(HACQDESC hAcqDesc, void *pwOffsetData, void *pwGainData, 
									void *pwGainAvgData,UINT nGainFrames, DWORD *pdwGainData, DWORD *pdwPxlCorrList, DWORD dwDataType);
HIS_RETURN Acquisition_Acquire_OffsetImage_32(HACQDESC hAcqDesc, void *pOffsetData, UINT nRows, UINT nCols, UINT nFrames, DWORD dwDataType);
HIS_RETURN Acquisition_Acquire_OffsetImage_Ex_32(HACQDESC hAcqDesc, void *pwOffsetData, UINT nRows, UINT nColumns, UINT nFrames, UINT dwOpt, DWORD dwDataType);
HIS_RETURN Acquisition_Acquire_OffsetImage_PreloadCorr_32(HACQDESC hAcqDesc, void *pOffsetData, UINT nRows, UINT nColumns, UINT nFrames, UINT dwOpt, DWORD dwDataType);

HIS_RETURN Acquisition_SetRotationAngle(HACQDESC hAcqDesc, long lRotAngle); // FG-E only can be -90 | 0 | 90
HIS_RETURN Acquisition_GetRotationAngle(HACQDESC hAcqDesc, long* lRotAngle);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

//error codes

#define HIS_ALL_OK							0
#define HIS_ERROR_MEMORY					1
#define HIS_ERROR_BOARDINIT					2
#define HIS_ERROR_NOCAMERA					3
#define HIS_ERROR_CORRBUFFER_INCOMPATIBLE	4
#define HIS_ERROR_ACQ_ALREADY_RUNNING		5
#define HIS_ERROR_TIMEOUT					6
#define HIS_ERROR_INVALIDACQDESC			7
#define HIS_ERROR_VXDNOTFOUND				8
#define HIS_ERROR_VXDNOTOPEN				9
#define HIS_ERROR_VXDUNKNOWNERROR			10
#define HIS_ERROR_VXDGETDMAADR				11
#define HIS_ERROR_ACQABORT					12
#define HIS_ERROR_ACQUISITION				13
#define HIS_ERROR_VXD_REGISTER_IRQ			14
#define HIS_ERROR_VXD_REGISTER_STATADR		15
#define HIS_ERROR_GETOSVERSION				16
#define HIS_ERROR_SETFRMSYNC				17
#define HIS_ERROR_SETFRMSYNCMODE			18
#define HIS_ERROR_SETTIMERSYNC				19
#define HIS_ERROR_INVALID_FUNC_CALL			20
#define HIS_ERROR_ABORTCURRFRAME			21
#define HIS_ERROR_GETHWHEADERINFO			22
#define HIS_ERROR_HWHEADER_INV				23
#define HIS_ERROR_SETLINETRIG_MODE			24
#define HIS_ERROR_WRITE_DATA				25
#define HIS_ERROR_READ_DATA					26
#define HIS_ERROR_SETBAUDRATE				27
#define HIS_ERROR_NODESC_AVAILABLE			28
#define HIS_ERROR_BUFFERSPACE_NOT_SUFF		29
#define HIS_ERROR_SETCAMERAMODE				30
#define HIS_ERROR_FRAME_INV					31
#define HIS_ERROR_SLOW_SYSTEM				32
#define HIS_ERROR_GET_NUM_BOARDS			33
#define HIS_ERROR_HW_ALREADY_OPEN_BY_ANOTHER_PROCESS	34
#define HIS_ERROR_CREATE_MEMORYMAPPING				35
#define HIS_ERROR_VXD_REGISTER_DMA_ADDRESS			36
#define HIS_ERROR_VXD_REGISTER_STAT_ADDR			37
#define HIS_ERROR_VXD_UNMASK_IRQ					38
#define HIS_ERROR_LOADDRIVER						39
#define HIS_ERROR_FUNC_NOTIMPL						40
#define HIS_ERROR_MEMORY_MAPPING					41
#define HIS_ERROR_CREATE_MUTEX						42
#define HIS_ERROR_ACQ								43
#define HIS_ERROR_DESC_NOT_LOCAL					44
#define HIS_ERROR_INVALID_PARAM						45
#define HIS_ERROR_ABORT								46
#define HIS_ERROR_WRONGBOARDSELECT					47
#define HIS_ERROR_WRONG_CAMERA_MODE					48	 
#define HIS_ERROR_AVERAGED_LOST						49	 

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


#define HIS_SYNCMODE_SOFT_TRIGGER		1
#define HIS_SYNCMODE_INTERNAL_TIMER		2
#define HIS_SYNCMODE_EXTERNAL_TRIGGER	3
#define HIS_SYNCMODE_FREE_RUNNING		4

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

#define HIS_MAX_TIMINGS							0x8

#endif	//_ACQUISITION_H
