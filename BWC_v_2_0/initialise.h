/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include "configuration.h"
#include "stm32f0xx_conf.h"
#include "stm32f0xx.h"
#include "system_stm32f0xx.h"
#include "core_cm0.h"
#include "core_cmFunc.h"
#include "core_cmInstr.h"
#include "stm32f0xx_flash.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_misc.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_spi.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_wwdg.h"
#include "stm32f0xx_dma.h"
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define Device_ICP
//#define Device_BWC

//#define BWC_v_2_0
#ifndef BWC_v_2_0
#define BWC_v_3_0
#endif

#define app_TIME_MS(t)  		((t))
#define app_TIME_S(t)  			((t)*1000)
#define app_TIME_Min(t)  		((t)*60000)
#define app_TIME_Hour(t)  		((t)*3600000)

#define MSG_UART_TX_RX_SIZE 	256
#define MSG_UART_TX_STACK 		5
#define MAGIC_NUMBER      		0x5A
#define TASK_STACK_SIZE			3

#define FLASH_SAVE				((uint16 *)(FLASH_BASE + 0x7C00))	/*!< FLASH base address in the Save Data region */
#define FLASH_SAVE_Res			((uint16 *)(FLASH_BASE + 0x7800))	/*!< FLASH base address in the Save Data region */

#if !defined FALSE && !defined TRUE
#define TRUE            (1)   /* page 207 K+R 2nd Edition */
#define FALSE           (0)
#endif /* !defined FALSE && #if !defined TRUE */
/****************************************************************************/
/***        Base Type Definitions                                         ***/
/****************************************************************************/
typedef unsigned char           BOOL_T;     /* boolean type nothing to do with C++ */
typedef unsigned char           bool_t;     /* boolean type nothing to do with C++ */
typedef signed char             int8;
typedef short                   int16;
typedef long                    int32;
typedef long long               int64;
typedef unsigned char           uint8;
typedef unsigned short          uint16;
typedef unsigned long           uint32;
typedef unsigned long long      uint64;
typedef char *                  string;
typedef volatile uint8          u8Register;
typedef volatile uint16         u16Register;
typedef volatile uint32         u32Register;
#define NULL ((void *)0)

typedef union{
	uint16 	UI16;
	uint8 	UI8[2];
}tsuUint16;

typedef union{
	uint8  	UI8[4];
	uint16 	UI16[2];
	uint32 	UI32;
}tsuUint32;

typedef union{
	uint64 	UI64;
	uint32 	UI32[2];
	uint16 	UI16[4];
	uint8 	UI8[8];
}tsuUint64;

/****************************************************************************/
/***        Base Type Definitions End                                     ***/
/****************************************************************************/

/****************************************************************************/
/***        USART: Type Definitions                                       ***/
/****************************************************************************/
/** \brief ������������ ��������� ��� ���������� ������� USART */
typedef enum{
	FIND_MAGIC_WORD,
	FIND_END,
} teHwRxState;

/** \brief ��������� �������� ������� �� USART */
typedef struct{
    uint8 			Data[MSG_UART_TX_RX_SIZE];
    uint8 			FirstElement;
    uint8 			NextElement;
    uint32 			SilenceElement;
    teHwRxState 	State;
    bool_t 			bDataOnRxLine;
    bool_t 			bExe;
} tsHwRx;

/** \brief ��������� ������� ������������/������������� ������*/
typedef struct{
	uint8			Data[64];
	uint8			Length;
}tsUartOut;

/** \brief ��������� ����������� ������� �� USART */
typedef struct{
    uint8 			Data[MSG_UART_TX_RX_SIZE];
    uint8 			CurrentElement;
    uint8 			NextElement;
    bool_t 			bTx;
} tsHwTx;

/** \brief ��������� ������� �������� ������������� ������ �� USART */
typedef struct{
	tsUartOut 		Msg[MSG_UART_TX_STACK];
    uint8 			FirstElement;
    uint8 			NextElement;
    bool_t			bExe;
}tsUartOutStack;

typedef enum{
    DA_EBS	=	1,	/** @arg  �������������� ���� ���������� */
    DA_CUV	=	2,	/** @arg  ���� ���������� ��������� */
    DA_BWC	=	3	/** @arg  ���� ������������ ������������ */
} teDeviceAddress;

/** \brief ��������� ������� ����� ������ - �������� */
typedef struct{
	unsigned char HeadLength	: 3;	//	���������� ���� � ��������� ������
	unsigned char Reserve		: 2;	//	������, �� ������������
	unsigned char bSA			: 1;	//	������� ������� ����� ������ ����������� � ��������� ������
	unsigned char bDA			: 1;	//	������� ������� ����� ������ ���������� � ��������� ������
	unsigned char bEncript		: 1;	//	������� ��������������� ������
}tsAttribute;
/** \brief ��������� ������� ����� ������ - �������� */
typedef union{
	tsAttribute 	Field;
	uint8			Byte;
}tuAttribute;

typedef struct{
	tuAttribute		Attribute;		// ������ ����� ������ - ���� ��������� ���������
	uint8			Length;			// ����� ���������� ���� � ������ �� ����������� ����������
	uint8			DateLength;		// ���������� �������� ����
	uint8			DestAddress;	// 8  ������ ��������� ����� ����������
	uint8			SourceAddress;	// 8  ������ ��������� ����� �����������
	uint8*			pPacket;		// ��������� �� ������ ���� ������ ��������� (�� ���� ���������)
	uint8*			pData;			// ��������� �� ������ ���� ������ � ������ ���������
	bool_t			bEncript;		// ������� ����������� ������
	bool_t			bDA;
	bool_t			bSA;
} tsHeadMessage;
/****************************************************************************/
/***        USART: Type Definitions End                                   ***/
/****************************************************************************/

/****************************************************************************/
/***        External protocol: Type Definitions                           ***/
/****************************************************************************/
/** \brief �������� ������ ��������� � ������� ����� ����� ������������ */
typedef enum{
	MEPE_EBS,
	MEPE_ICP,
	MEPE_ACC,
	MEPE_SetNetBath,
	MEPE_GetNetBath,
	MEPE_DS18S20temperature,
	MEPE_SetAllWorkData,
	MEPE_AckSetAllWorkData,
	MEPE_BeginHeating,
	MEPE_AckBeginHeating,
	MEPE_WaitPerson,
	MEPE_StartCount,
	MEPE_AckStartCount,
	MEPE_StopBathProcedure,
	MEPE_AckStopBathProcedure,
	MEPE_AckOccureErr1,
	MEPE_AckOccureErr2,
	MEPE_AckOccureErr3,
	MEPE_SendACCtoICP,
	MEPE_SendICPtoACC,
	MEPE_SwitchCompressor,
	MEPE_AckSwitchOnCompressor,
	MEPE_AckSwitchOffCompressor,
	MEPE_SwitchOff,
	MEPE_LostACC,
	MEPE_tsDutyCycleData,
	MEPE_ConfigureIPU,
	MEPE_PresenceAerosolTherapy,		/** \arg ������� ������� � ������������� ������� ��������������� ��������� ���������������*/
	MEPE_LocalExchange,
	MEPE_AckDutyCycleData,
	MEPE_SwitchesStates,
	MEPE_AskCalibrateSenseKey
}teMyEndPointExe;

typedef enum{
	SM_CC110L,
	SM_ST3485
}teSourceMessage;
/****************************************************************************/
/***        External protocol: Type Definitions End                       ***/
/****************************************************************************/

/****************************************************************************/
/***        Deferred actions: Type Definitions                            ***/
/****************************************************************************/
/** \brief ��������� ������������ ��������-������ � ������� */
typedef struct{
	bool_t			bQueryExe;
	uint8			u8Task;
	uint32			u32Time;
	tsuUint32		Data;
}tsTaskTime;

/** \brief ��������� ������� ����������� ��������-����� */
typedef struct{
	tsTaskTime		sQueue		[TASK_STACK_SIZE];
	uint8			iFreeQueue	[TASK_STACK_SIZE];
	uint8			iBusyQueue	[TASK_STACK_SIZE];
	uint8			pFreeTask;				/* �������������� ������ ��������� �� ��������� ������ � ������� ���������� pFreeQueue[TASK_STACK_SIZE] */
	uint8			pNearestTask;			/* ������ ��������� �� ��������� ��������������� ��������� ����������� �������� � ������� ���������� pBusyQueue[TASK_STACK_SIZE] */
	uint8			pFurthestTask;			/* ������ ��������� �� ����� ������� ��������������� ��������� ����������� �������� � ������� ���������� pBusyQueue[TASK_STACK_SIZE] */
	uint32			u32RealTime;			/* ��������� ������� ��������� ������� */
	bool_t			bBusy;					/* �������� ���� � ���������� ������������ ��������������� ����������� ��������*/
}tsTimerExecute;

/** \brief �������� ����� ����������� ��������� bIsThereDeleteFutureTask � bIsThereDeleteFutureTaskData */
typedef enum{
	FTM_IsThere,
	FTM_Delete,
	FTM_Clean,
	FTM_GetPointerStruct,
	FTM_GetTime,
	FTM_EstimateLeftTime,
	FTM_GetData,
	FTM_GetQueuePosition,
}teFutureTaskModify;

/** \brief ������ ����������� ��������-����� � �������*/
typedef enum{
	Task_NoTask,
	Task_MainWork,
	Task_CC110L,
	Task_tsDutyCycleData,
	Task_Save,
	Task_StopProcedure,
	Task_WWGD,
	Task_CleanStackTxRF
}teTaskTime;
/****************************************************************************/
/***        Deferred actions: Type Definitions End                        ***/
/****************************************************************************/

/****************************************************************************/
/***        Radio: Type Definitions                                       ***/
/****************************************************************************/
/****************************************************************************
 * 							Own Protocol of Communication
 ****************************************************************************/
#define		ResolveBWCAbsence			3

#define		OneByteCommunication		4

#define		PauseAfterTransmitPkt		100
#define		PauseAfterReceivedAck		100

#define		PauseAfterReceivedPkt		30
#define		PauseAfterTransmitAck		30

#define		PeriodLifeBeamToBeam		1485

#define 	MSG_CC11_TX_RX_SIZE 		17

/** \brief �������� ������� �����-����������� */
typedef enum{
    TE_NONE,						/* ������� ��� - �����-���������� �� ������� �� ��������� */
    TE_Transmit,					/* ����������� �������� ������ - ������� ��� ��� */
    TE_Receive,						/* ��������� ���� ������ */
}teTransceiverEventRF;

/** \brief �������� ��������� ���������� (����) ������� � ��������� ������� �� ����� */
typedef enum{
	WP_WaitOfPacket,				/* ������ �������� ���������� ������� ����� (EBS ����� ��������� �������) */
    WP_TransmitPacket,				/* �������� ������ */
    WP_PauseAfterTransmitPkt,		/* ����� ����� �������� ������ */
    WP_ReceiveAcknowledge,			/* ��������� ������������� ��������� ������ */
    WP_PauseAfterReceiveAck,		/* ����� ����� ��������� ������������� */

    WP_ReceivePacket,				/* ��������� ������ */
    WP_PauseAfterReceivePkt,		/* ����� ����� ��������� ������ */
    WP_TransmitAcknowledge,			/* �������� ������������� ��������� ������*/
    WP_PauseAfterTransmitAck,		/* ����� ����� �������� ������������� ��������� ������ */
    WP_BeAbleBothRandomPacket,		/* ������� ���������� �������� �������� ������, ����� ����� �� ������ ���������� ������� Beam-Beam */
}teWindowsRF;

/** \brief ������ �������� ������� */
typedef enum{
	MP_NONE,						/* ��������� �����, ��� �� ������� � ������� teWindowsRF */
    MP_FirstPacket,					/* ������ ����� 	*/
    MP_SecondPacket,				/* ������ ����� 	*/
    MP_ThirdPacket,					/* ������ �����		*/
    MP_Acknowledge,					/* �������������	*/
}teMarkerPacketRF;

/** \brief ���� ��������� ���� �������� teWindowsRF */
typedef struct{
	unsigned char bFirstPacket		: 1;	/*	���� ��������/���������� ������� ������, ����������� ������� teWindowsRF, ����������� - Owner Ether */
	unsigned char bSecondPacket		: 1;	/*	���� ��������/���������� ������� ������ ������ */
	unsigned char bThirdPacket		: 1;	/*	���� ��������/���������� �������� ������ ������ */
	unsigned char bAcknowledge		: 1;	/*	���� ����������/�������� ��������� ������ ������ ��� ������������� ����������� ����� */
	unsigned char Reserve			: 4;
}tsTransmitionWork;

/** \brief ������ ����� �������� teWindowsRF */
typedef struct{
	tsTransmitionWork	bHasBeenPacket;
	uint8				u8Marker;			/*	������ ������ �������������/��������� � ��������� ������ �� teMarkerPacketRF */
}tsWorkWindows;

/** \brief ������ ����� �������� teWindowsRF */
typedef union{
	tsWorkWindows		s8Field;
	uint16				u16Clean;	/*	������ ���� ��� �������� ������� ����� ������� */
}tuWorkWindows;

/** \brief ��������� ������� ���������� ����� ���� */
typedef struct{
	bool_t							bOrphan;					/* ������� ���������� ������� ���������� ICP ��� BWC �������������� */
	uint8							u8CountAbsenceBeam;			/* ������� ������������� ������ ��� ������� �� ��� */
	uint8							eDataOnEther;				/* ������� ����������� ������ � ����� � ����������� teTransceiverEventRF */
	uint8							eCurrentWindowRF;			/* ������� ����������� ��������� �� ������������ �������� teWindowsRF */
	tuWorkWindows					uAllBeenWindows;			/* ������� ����� tsWorkWindows ��������� ����������� ���� �������� teWindowsRF ��� ������������� ������ */
	uint32							u32EndLifeWindowRF;			/* ��������������� ������� ����� ����� � �������� � ���������� ���� �� �������� teWindowsRF */
	uint32							u32StartBeamToBeamTime;		/* �������� ������� ������� ������ ������� ����-����, ����� ����������� � ������ ����� ��������� teWindowsRF ���������� �� ��������� */
	uint32							u32LastReceivedPacketTime;
	uint32							u32LastTransmitedPacketTime;
}tsNwkState;

/** \brief ��������� ������� ������ ������ � ������ ������������� ������ � �������� teWindowsRF */
typedef struct{
	uint8 u8NumberPacketRF;		/*	����� ������ (8 ���) - ����������, ��������� ������, ����������� ������� ������� ICP �� ����� ������ BWC */
	uint8 u8Marker;				/*	������ ������ �������� ��������� teMarkerPacketRF */
}tsdIndexMarker;

/** \brief ������ ����� ������� ������������� ������ � �������� teWindowsRF */
typedef union{
	tsdIndexMarker	d8Field;
	tsuUint16		ShortInt;
}tuIndexMarker;

/** \brief �������� ��������� ���������� ��������� */
typedef enum{
    RF_NONE	=		0x00,
	RF_BWC	=		0x11,			/* Block of Wireless Communication */
    RF_ICP	=		0x22,			/* Indicator Control Panel */
    RF_ACC	=		0x33			/* Additional Control Console */
} teRFAddress;

/** \brief ����� ��� ��������/����� �� ����� */
typedef struct{
	uint8			Date[35];
	uint8			Length;
}tsPacketRF;

/** \brief ��������� � ������� ������� ��� ��������/����� �� ����� */
typedef struct{
	tsPacketRF		pHeap[MSG_CC11_TX_RX_SIZE];
    uint8 			FirstPacket;
    uint8 			FreePacket;
    tuIndexMarker	u13Index_u3Marker;		// ������� ������������� ��� ���������� ������ ���������� ��������� ������
    uint8			EmptyPackets;
    bool_t			QuantityPackets;
}tsCC110lOutStack;

typedef struct{
	uint8			Length;					// ����� ���������� ���� � ������
	uint8			DestinationAddress;		// 8 ������ ��������� ����� ��������
	uint8			SourceAddress;			// 8 ������ ��������� ����� �����������
	tuIndexMarker	u13Index_u3Marker;		// ���������� ����� ������������� ������ � ������ ������ � �������� teWindowsRF
	uint8			DateLength;				// ���������� �������� ����
	uint8*			pPacket;				// ��������� �� ������ ���� ������ ���������
	uint8*			pDate;					// ��������� �� ������ ���� ������ � ������ ���������
}tsHeadRFMessage;

typedef struct{
	uint8			IndexPacket;
	uint8			NextData;
	tuIndexMarker	Index;
}tsDataTask;

typedef union{
	tsuUint32		su32Byte;
	tsDataTask		Field;
}tuDataTask;

typedef struct{
	uint8			Length;					// ����� ���������� ���� � ������
	uint8			ReceivedData;			// ���������� �������� ���� ������
	uint8			FormerRestOfData;		// ������� ������� ������ � ������ �������� CC110l
}tsHeadInRFMessage;
/****************************************************************************/
/***        Radio: Type Definitions End                                   ***/
/****************************************************************************/

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
