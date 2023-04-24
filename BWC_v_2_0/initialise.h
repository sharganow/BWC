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
/** \brief Перечисление состояний при разделении пакетов USART */
typedef enum{
	FIND_MAGIC_WORD,
	FIND_END,
} teHwRxState;

/** \brief Структура приёмника пакетов по USART */
typedef struct{
    uint8 			Data[MSG_UART_TX_RX_SIZE];
    uint8 			FirstElement;
    uint8 			NextElement;
    uint32 			SilenceElement;
    teHwRxState 	State;
    bool_t 			bDataOnRxLine;
    bool_t 			bExe;
} tsHwRx;

/** \brief Структура массива принимаемого/передаваемого пакета*/
typedef struct{
	uint8			Data[64];
	uint8			Length;
}tsUartOut;

/** \brief Структура передатчика пакетов по USART */
typedef struct{
    uint8 			Data[MSG_UART_TX_RX_SIZE];
    uint8 			CurrentElement;
    uint8 			NextElement;
    bool_t 			bTx;
} tsHwTx;

/** \brief Структура очереди массивов передаваемого пакета по USART */
typedef struct{
	tsUartOut 		Msg[MSG_UART_TX_STACK];
    uint8 			FirstElement;
    uint8 			NextElement;
    bool_t			bExe;
}tsUartOutStack;

typedef enum{
    DA_EBS	=	1,	/** @arg  Исполнительный Блок Коммутации */
    DA_CUV	=	2,	/** @arg  Блок Управления Клапанами */
    DA_BWC	=	3	/** @arg  Блок Беспроводной Коммуникации */
} teDeviceAddress;

/** \brief Структура первого байта пакета - Атрибуты */
typedef struct{
	unsigned char HeadLength	: 3;	//	Количество байт в заголовке пакета
	unsigned char Reserve		: 2;	//	резерв, не используются
	unsigned char bSA			: 1;	//	Признак наличия байта адреса ОТПРАВИТЕЛЯ в заголовке пакета
	unsigned char bDA			: 1;	//	Признак наличия байта адреса ПОЛУЧАТЕЛЯ в заголовке пакета
	unsigned char bEncript		: 1;	//	Признак зашифрованности данных
}tsAttribute;
/** \brief Структура первого байта пакета - Атрибуты */
typedef union{
	tsAttribute 	Field;
	uint8			Byte;
}tuAttribute;

typedef struct{
	tuAttribute		Attribute;		// первое слово пакета - байт признаков атрибутов
	uint8			Length;			// общее количество байт в пакете за исключением стартового
	uint8			DateLength;		// количество полезной даты
	uint8			DestAddress;	// 8  битный локальный адрес получателя
	uint8			SourceAddress;	// 8  битный локальный адрес отправителя
	uint8*			pPacket;		// указатель на первый байт пакета сообщения (на байт атрибутов)
	uint8*			pData;			// указатель на первый байт данных в пакете сообщения
	bool_t			bEncript;		// признак кодирования данных
	bool_t			bDA;
	bool_t			bSA;
} tsHeadMessage;
/****************************************************************************/
/***        USART: Type Definitions End                                   ***/
/****************************************************************************/

/****************************************************************************/
/***        External protocol: Type Definitions                           ***/
/****************************************************************************/
/** \brief Протокол обмена командами и данными между всеми устройствами */
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
	MEPE_PresenceAerosolTherapy,		/** \arg Команда запроса и подтверждения Наличия автоматического Смесителя Аэрозольтерапии*/
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
/** \brief Структура отсроченного действия-задачи в системе */
typedef struct{
	bool_t			bQueryExe;
	uint8			u8Task;
	uint32			u32Time;
	tsuUint32		Data;
}tsTaskTime;

/** \brief Структура системы отсроченных действий-задач */
typedef struct{
	tsTaskTime		sQueue		[TASK_STACK_SIZE];
	uint8			iFreeQueue	[TASK_STACK_SIZE];
	uint8			iBusyQueue	[TASK_STACK_SIZE];
	uint8			pFreeTask;				/* Подготовленный индекс указателя на свободную ячейку в массиве указателей pFreeQueue[TASK_STACK_SIZE] */
	uint8			pNearestTask;			/* Индекс указателя на ближайшее запланированное очередное отсроченное действие в массиве указателей pBusyQueue[TASK_STACK_SIZE] */
	uint8			pFurthestTask;			/* Индекс указателя на самое позднее запланированное очередное отсроченное действие в массиве указателей pBusyQueue[TASK_STACK_SIZE] */
	uint32			u32RealTime;			/* Системный счётчик реального времени */
	bool_t			bBusy;					/* Отражает факт и количество существующих запланированных отсроченных действий*/
}tsTimerExecute;

/** \brief Перечень задач исполняемых функциями bIsThereDeleteFutureTask и bIsThereDeleteFutureTaskData */
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

/** \brief Список отсроченных действий-задач в системе*/
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

/** \brief Перечень событий приёмо-передатчика */
typedef enum{
    TE_NONE,						/* Событий нет - приёмо-передатчик ни передаёт ни принимает */
    TE_Transmit,					/* Произведена передача пакета - успешно или нет */
    TE_Receive,						/* Произведён приём пакета */
}teTransceiverEventRF;

/** \brief Перечень Временных интервалов (окон) отпраки и получения пакетов по радио */
typedef enum{
	WP_WaitOfPacket,				/* Период Ожидания поискового сигнала Маяка (EBS после включения питания) */
    WP_TransmitPacket,				/* Передача пакета */
    WP_PauseAfterTransmitPkt,		/* Пауза После передачи пакета */
    WP_ReceiveAcknowledge,			/* Получение подтверждения получения пакета */
    WP_PauseAfterReceiveAck,		/* Пауза после получения подтверждения */

    WP_ReceivePacket,				/* Получение Пакета */
    WP_PauseAfterReceivePkt,		/* Пауза после получения Пакета */
    WP_TransmitAcknowledge,			/* Передача подтверждения получения пакета*/
    WP_PauseAfterTransmitAck,		/* Пауза после передачи подтверждения получения пакета */
    WP_BeAbleBothRandomPacket,		/* Обоюдно правомерно Случайно Передача Пакета, время жизни до начала следующего периода Beam-Beam */
}teWindowsRF;

/** \brief Список маркеров пакетов */
typedef enum{
	MP_NONE,						/* Первичный пакет, ещё не встроен в подцикл teWindowsRF */
    MP_FirstPacket,					/* Первый пакет 	*/
    MP_SecondPacket,				/* Второй пакет 	*/
    MP_ThirdPacket,					/* Третий пакет		*/
    MP_Acknowledge,					/* Подтверждение	*/
}teMarkerPacketRF;

/** \brief Поля отработки окон подцикла teWindowsRF */
typedef struct{
	unsigned char bFirstPacket		: 1;	/*	Факт отправки/наблюдения первого пакета, Запускающий подцикл teWindowsRF, отправитель - Owner Ether */
	unsigned char bSecondPacket		: 1;	/*	Факт отправки/наблюдения второго Пакета данных */
	unsigned char bThirdPacket		: 1;	/*	Факт отправки/наблюдения третьего Пакета данных */
	unsigned char bAcknowledge		: 1;	/*	Факт наблюдения/отправки ответного Пакета данных или подтверждения обнаружения МАЯКА */
	unsigned char Reserve			: 4;
}tsTransmitionWork;

/** \brief Регист полей подцикла teWindowsRF */
typedef struct{
	tsTransmitionWork	bHasBeenPacket;
	uint8				u8Marker;			/*	Маркер пакета передаваемого/принятого в настоящий момент из teMarkerPacketRF */
}tsWorkWindows;

/** \brief Регист полей подцикла teWindowsRF */
typedef union{
	tsWorkWindows		s8Field;
	uint16				u16Clean;	/*	Единое поле для удобства очистки после анализа */
}tuWorkWindows;

/** \brief Структура рабочих параметров радио сети */
typedef struct{
	bool_t							bOrphan;					/* Признак отсутствия парного устройства ICP или BWC соответственно */
	uint8							u8CountAbsenceBeam;			/* Счётчик отсутствующих МАЯКов или ответов на них */
	uint8							eDataOnEther;				/* Признак прохождения данных в эфире и направление teTransceiverEventRF */
	uint8							eCurrentWindowRF;			/* Регистр актуального фрагмента из действующего подцикла teWindowsRF */
	tuWorkWindows					uAllBeenWindows;			/* Регистр полей tsWorkWindows признаков случившихся окон подцикла teWindowsRF для передаваемого пакета */
	uint32							u32EndLifeWindowRF;			/* Предопределение времени конца жизни и перехода к следующему окну из подцикла teWindowsRF */
	uint32							u32StartBeamToBeamTime;		/* Фиксация момента времени начала периода МАЯК-МАЯК, ЕДИНО обновляется с каждым новым подциклом teWindowsRF независимо от авторства */
	uint32							u32LastReceivedPacketTime;
	uint32							u32LastTransmitedPacketTime;
}tsNwkState;

/** \brief Составной регистр номера пакета и маркер отправляемого пакета в подцикле teWindowsRF */
typedef struct{
	uint8 u8NumberPacketRF;		/*	Номер пакета (8 бит) - транзакции, нумерация единая, запускается счётчик пультом ICP во время поиска BWC */
	uint8 u8Marker;				/*	маркер пакета согласно структуре teMarkerPacketRF */
}tsdIndexMarker;

/** \brief Регист полей маркера отправляемого пакета в подцикле teWindowsRF */
typedef union{
	tsdIndexMarker	d8Field;
	tsuUint16		ShortInt;
}tuIndexMarker;

/** \brief Перечень устройств занимающих радиоэфир */
typedef enum{
    RF_NONE	=		0x00,
	RF_BWC	=		0x11,			/* Block of Wireless Communication */
    RF_ICP	=		0x22,			/* Indicator Control Panel */
    RF_ACC	=		0x33			/* Additional Control Console */
} teRFAddress;

/** \brief Пакет для передачи/приёма по радио */
typedef struct{
	uint8			Date[35];
	uint8			Length;
}tsPacketRF;

/** \brief Структура и очередь пакетов для передачи/приёма по радио */
typedef struct{
	tsPacketRF		pHeap[MSG_CC11_TX_RX_SIZE];
    uint8 			FirstPacket;
    uint8 			FreePacket;
    tuIndexMarker	u13Index_u3Marker;		// счётчик отправленного или уникальный индекс последнего принятого пакета
    uint8			EmptyPackets;
    bool_t			QuantityPackets;
}tsCC110lOutStack;

typedef struct{
	uint8			Length;					// общее количество байт в пакете
	uint8			DestinationAddress;		// 8 битный локальный адрес приёмника
	uint8			SourceAddress;			// 8 битный локальный адрес отправителя
	tuIndexMarker	u13Index_u3Marker;		// Порядковый номер отправляемого пакета и Маркер пакета в подцикле teWindowsRF
	uint8			DateLength;				// количество полезной даты
	uint8*			pPacket;				// указатель на первый байт пакета сообщения
	uint8*			pDate;					// указатель на первый байт данных в пакете сообщения
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
	uint8			Length;					// общее количество байт в пакете
	uint8			ReceivedData;			// количество принятых байт данных
	uint8			FormerRestOfData;		// прежний остаток данных в буфере приёмника CC110l
}tsHeadInRFMessage;
/****************************************************************************/
/***        Radio: Type Definitions End                                   ***/
/****************************************************************************/

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
