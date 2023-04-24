/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include "initialise.h"
#include "spi.h"
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern	tsCC110L 				RadioRxTx;
extern	tsuUint16 				sRxCC110l[0x32];
extern	tsuUint16 				sTxCC110l[0x32];
extern	tsHwTx 					sTx;
extern	tsHwRx 					sRx;
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
tsTimerExecute			sTaskStack;
uint64  const uic64DecHex[] = {
	1,
	10,
	100,
	1000,
	10000,
	100000,
	1000000,
	10000000,
	100000000,
	1000000000,
	10000000000,
	100000000000,
	1000000000000,
	10000000000000,
	100000000000000,
	1000000000000000,
	10000000000000000,
	100000000000000000,
	1000000000000000000,
};
/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
void 	vGPIO_Initialization(void);
void	vTIM17_Initialization(void);
void	vDMA1_Initialization(void);
void	vWWGD_Initialization(void);
uint16	u16GetCRC(uint8* pvMsg, uint8 Displace,uint8 u8MsgLen);
void	vResetTaskExecuteStack(void);
bool_t	bAddTaskExeInTime(uint8 TASK, uint32 TIME, uint32 DATA);
uint32	bIsThereDeleteFutureTask(uint8 TASK, bool_t bDELETE);
uint32	bIsThereDeleteFutureTaskData(uint8 TASK, uint32 DATA, bool_t bDELETE);
void	vSetRealTimeModifyTasks(uint32 u32NewRealTime);
void	MemCpy(uint8* pDest, uint8* pSour, uint8 Number);
void	MemSet8(uint8* pDest, uint8 Set, uint8 Number);
void	MemSet16(uint16* pDest, uint16 Set, uint8 Number);
void	vDelay(uint32 TIME);
void	vTransHexToDec(uint8* Dec, uint8* Hex, uint8 LengthHex);
uint8	u8DecCountValue(uint64* SoS, uint64 Const);
void	vClearString(uint8* String, uint8 Length);
void	vTransDecToHex(uint8* Hex, uint8* Dec, uint8 LengthDec);
/****************************************************************************/
/***        Local Functions			                                      ***/
/****************************************************************************/
/****************************************************************************
 * NAME:		vGPIO_Initialization
 * DESCRIPTION: Конфигурирует задействованные линии портов контроллера в схеме
 ****************************************************************************/
void vGPIO_Initialization(void){
	/** КОНФИГУРИРОВАНИЕ ПОРТА А */
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
		GPIO_InitTypeDef Port;
		Port.GPIO_Mode 	= GPIO_Mode_IN;
		Port.GPIO_OType = GPIO_OType_PP;
		Port.GPIO_Speed	= GPIO_Speed_Level_3;
		Port.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
#ifdef 	BWC_v_2_0
		Port.GPIO_Pin	= GPIO_Pin_2;
		GPIO_Init(GPIOA, &Port);
		Port.GPIO_PuPd 	= GPIO_PuPd_UP;
		Port.GPIO_Pin	= GPIO_Pin_5 | GPIO_Pin_6  | GPIO_Pin_7 |
						  GPIO_Pin_9 |  GPIO_Pin_10;
		GPIO_Init(GPIOA, &Port);
		Port.GPIO_Mode 	= GPIO_Mode_OUT;
		Port.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
		Port.GPIO_Pin	= GPIO_Pin_1 | 	GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12;
		GPIO_Write(GPIOA, GPIO_Pin_1 | 	GPIO_Pin_11 | GPIO_Pin_12);
		GPIO_Init(GPIOA, &Port);
		/* Конфигурирование альтернативных функций портов */
		Port.GPIO_Mode 	= GPIO_Mode_AF;
		Port.GPIO_PuPd 	= GPIO_PuPd_UP;
		Port.GPIO_Pin	= /* GPIO_Pin_4 |*/ GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_9 | GPIO_Pin_10 ;
		GPIO_Init(GPIOA, &Port);
		/** КОНФИГУРИРОВАНИЕ ПОРТА F */
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
		Port.GPIO_Mode 	= GPIO_Mode_IN;
		Port.GPIO_Pin	= GPIO_Pin_0;
		GPIO_Init(GPIOF, &Port);
#else
#ifdef BWC_v_3_0
		Port.GPIO_Pin	= GPIO_Pin_9 | GPIO_Pin_10;
		GPIO_Init(GPIOA, &Port);

		Port.GPIO_Mode 	= GPIO_Mode_OUT;
		Port.GPIO_Pin	= GPIO_Pin_1;
		GPIO_Write(GPIOA, GPIO_Pin_1);
		GPIO_Init(GPIOA, &Port);

		/* Конфигурирование альтернативных функций портов */
		Port.GPIO_Mode 	= GPIO_Mode_AF;
		Port.GPIO_PuPd 	= GPIO_PuPd_UP;		// нужно попробовать без подтягивания
		Port.GPIO_Pin	= GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_Init(GPIOA, &Port);

#endif 	/* BWC_v_3_0 */
#endif	/* BWC_v_2_0 */
}
/****************************************************************************
 * NAME:		vTIM_Initialization
 * DESCRIPTION: Настраивает работу таймера 17
 ****************************************************************************/
void vTIM17_Initialization(void){
	sTaskStack.u32RealTime = 0;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM17, ENABLE );
	TIM_TimeBaseInitTypeDef   sTIM_TimeBaseInitStruct;
	TIM_TimeBaseStructInit ( &sTIM_TimeBaseInitStruct );
	sTIM_TimeBaseInitStruct.TIM_Period = 48000;
	TIM_TimeBaseInit  ( TIM17, &sTIM_TimeBaseInitStruct );
	TIM_Cmd  ( TIM17, ENABLE );

	NVIC_InitTypeDef sNVIC_InitStruct;
	sNVIC_InitStruct.NVIC_IRQChannel = TIM17_IRQn;
	sNVIC_InitStruct.NVIC_IRQChannelPriority = 2;
	sNVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&sNVIC_InitStruct);/**/

	TIM_ITConfig(TIM17, TIM_IT_Update, ENABLE);
}
/****************************************************************************
 * NAME:		TIM17_IRQHandler
 * DESCRIPTION: Вектор прерывания от TIM17
 ****************************************************************************/
void TIM17_IRQHandler(void){
		TIM_ClearFlag  (TIM17, TIM_FLAG_Update);
		sTaskStack.u32RealTime++;
}
/****************************************************************************
 * NAME:		vDMA1_Initialization
 * DESCRIPTION: Конфигурирует DMA1 для работы с модулем USART
 ****************************************************************************/
void vDMA1_Initialization(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG->CFGR1		   |= SYSCFG_CFGR1_USART1TX_DMA_RMP | SYSCFG_CFGR1_USART1RX_DMA_RMP;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
/*	Конфигурация DMA Channel2 на работу с SPI1_Rx	*/
    DMA1_Channel2->CPAR 	= (uint32)&SPI1->DR;
    DMA1_Channel2->CMAR 	= (uint32)&sRxCC110l[0].UI16;
    DMA1_Channel2->CNDTR 	= 2;
    DMA1_Channel2->CCR		= DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PL_1;
/*	Конфигурация DMA Channel3 на работу с SPI1_Tx	*/
	DMA1_Channel3->CPAR 	= (uint32)&SPI1->DR;
	DMA1_Channel3->CMAR 	= (uint32)&sTxCC110l[0].UI16;
	DMA1_Channel3->CNDTR 	= 2;
	DMA1_Channel3->CCR		= DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0;
/*	Конфигурация DMA Channel4 на работу с USART1_Tx	*/
    DMA1_Channel4->CPAR 	= (uint32)&USART1->TDR;
    DMA1_Channel4->CMAR 	= (uint32)&sTx.Data[0];
    DMA1_Channel4->CCR		= DMA_CCR_MINC |  DMA_CCR_PL_1 | DMA_CCR_DIR;
/*	Конфигурация DMA Channel5 на работу с USART1_Rx	*/
	DMA1_Channel5->CPAR 	= (uint32)&USART1->RDR;
	DMA1_Channel5->CMAR 	= (uint32)&sRx.Data[0];
	DMA1_Channel5->CCR		= DMA_CCR_MINC |  DMA_CCR_CIRC;
	DMA1_Channel5->CNDTR 	= MSG_UART_TX_RX_SIZE;
	DMA1_Channel5->CCR	   |= DMA_CCR_EN;
}
/****************************************************************************
 * NAME:		vWWGD_Initialization
 * DESCRIPTION: Конфигурирует сторожевой оконный таймер Период: 43,690(6) мс
 ****************************************************************************/
void vWWGD_Initialization(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
	WWDG_SetPrescaler  ( WWDG_Prescaler_8);
	WWDG_SetWindowValue  ( 0x7F );
	WWDG_Enable  		 ( 0x7F );
	bAddTaskExeInTime(Task_WWGD, app_TIME_MS(15), 0);
	WWDG_EnableIT();
}
/****************************************************************************
 * NAME:		u16GetCRC
 * DESCRIPTION: Вычисляет код контрольной суммы u16CRC
 * RETURN: 		контрольная сумма u16CRC
 ****************************************************************************/
uint16 u16GetCRC(uint8* pvMsg, uint8 Displace,uint8 u8MsgLen){
	 bool_t bCRChl;
	 bool_t bCRCll;
	 tsuUint16 ByteTemp;
	 tsuUint16 u16CRC;
	 ByteTemp.UI16=0;
	 u16CRC.UI16=0xFFFF;
	 uint8 i,n;
	 for(i=0;i<u8MsgLen;i++){
		  ByteTemp.UI8[0]=pvMsg[i+Displace];
		  ByteTemp.UI8[1]=0;
		  u16CRC.UI16^=ByteTemp.UI16;
		  for(n=0;n<8;n++){
			   bCRChl=u16CRC.UI8[1]&1;
			   bCRCll=u16CRC.UI8[0]&1;
			   u16CRC.UI8[1]/=2;
			   u16CRC.UI8[0]/=2;
			   if(bCRChl){u16CRC.UI8[0]+=128;}
			   if(bCRCll){u16CRC.UI8[0]^=0x01; u16CRC.UI8[1]^=0xA0;}
		  }
	 }
	 return u16CRC.UI16;
}
/****************************************************************************
 * NAME:		vResetTaskExecuteStack
 * DESCRIPTION: подготавливает стек с его рабочими регистрами и массивами к
 * 				работе
 ****************************************************************************/
void vResetTaskExecuteStack(void){
	sTaskStack.bBusy 						= FALSE;
	sTaskStack.pFreeTask 					= 0;
	sTaskStack.pNearestTask 				= 0;
	sTaskStack.pFurthestTask 				= 0;
	uint8	u8i;
	for(u8i = 0; u8i < TASK_STACK_SIZE; u8i++){
		sTaskStack.iBusyQueue[u8i] 			= TASK_STACK_SIZE;
		sTaskStack.iFreeQueue[u8i] 			= u8i;
		sTaskStack.sQueue[u8i].u8Task 		= Task_NoTask;
		sTaskStack.sQueue[u8i].Data.UI32	= 0;
		sTaskStack.sQueue[u8i].u32Time		= 0;
		sTaskStack.sQueue[u8i].bQueryExe	= FALSE;
	}
}
/****************************************************************************
 * NAME:		p8GetFreeTask
 * DESCRIPTION: находит свободную ячейку с стеке очереди отсроченных действий
 * RETURN: 		указатель на указатель из массива iFreeQueue
 ****************************************************************************/
uint8 p8GetFreeTask(void){
	uint8	i;
	for(i=0; i < TASK_STACK_SIZE; i++){
		if(sTaskStack.iFreeQueue[i] != TASK_STACK_SIZE){
			return i;
		}
	}
	return TASK_STACK_SIZE;
}
/****************************************************************************
 * NAME:		bAddTaskExeInTime
 * DESCRIPTION: Добавляет задание в очередь
 * RETURN: 		число отсроченных действий в статусе ожидания
 ****************************************************************************/
bool_t	bAddTaskExeInTime(uint8 TASK, uint32 TIME, uint32 DATA){
	if(sTaskStack.pFreeTask >= TASK_STACK_SIZE){
		sTaskStack.pFreeTask = p8GetFreeTask();
		if(sTaskStack.pFreeTask >= TASK_STACK_SIZE){
			return FALSE;
		}
	}
	else{
		if(sTaskStack.iFreeQueue[sTaskStack.pFreeTask] >= TASK_STACK_SIZE){
			sTaskStack.pFreeTask = p8GetFreeTask();
			if(sTaskStack.pFreeTask >= TASK_STACK_SIZE){
				return FALSE;
			}
		}
	}
	uint32	ExeTime = sTaskStack.u32RealTime + TIME;
	uint8	AddPosition = sTaskStack.pFurthestTask + 1; if(AddPosition >= TASK_STACK_SIZE){	AddPosition = 0;}
	sTaskStack.pFurthestTask = AddPosition;
	if(sTaskStack.bBusy){
		// Поскольку уже имеются задания - то нужно определить позицию нового задания в очереди исполнений, и
		// при необходимости произвести сдвиг очереди для подготовки места новому заданию
		uint8	SearchAddPosition = AddPosition;
		do{
			if(SearchAddPosition > 0){	SearchAddPosition--;} 	else{	SearchAddPosition 	= TASK_STACK_SIZE - 1;}
			if((ExeTime >= sTaskStack.sQueue[sTaskStack.iBusyQueue[SearchAddPosition]].u32Time)||(AddPosition == sTaskStack.pNearestTask)){
				// Позиция в очереди отсроченных действий найдена для добавления нового
				SearchAddPosition 										= TASK_STACK_SIZE;
			}
			else{
				sTaskStack.iBusyQueue[AddPosition] 						= sTaskStack.iBusyQueue[SearchAddPosition];
				sTaskStack.iBusyQueue[SearchAddPosition]				= TASK_STACK_SIZE;
				if(AddPosition > 0){ 	AddPosition--; } 		else{	AddPosition 		= TASK_STACK_SIZE - 1; }
			}
		}while(SearchAddPosition < TASK_STACK_SIZE);
	}
	else{
		vResetTaskExecuteStack();
		// Позиция в очереди отсроченных действий найдена для добавления нового
		AddPosition = 0;
	}

	sTaskStack.iBusyQueue[AddPosition] 				= sTaskStack.iFreeQueue[sTaskStack.pFreeTask];
	sTaskStack.iFreeQueue[sTaskStack.pFreeTask] 	= TASK_STACK_SIZE;
	sTaskStack.pFreeTask 							= p8GetFreeTask();
	tsTaskTime*	pTask 								= &sTaskStack.sQueue[sTaskStack.iBusyQueue[AddPosition]];
	pTask->u32Time 									= ExeTime;
	pTask->u8Task									= TASK;
	pTask->Data.UI32								= DATA;
	pTask->bQueryExe								= TRUE;

	return ++sTaskStack.bBusy;
}
/****************************************************************************
 * NAME:		bIsThereFutureTask
 * DESCRIPTION: Поиск отсроченного действия в очереди ожидания, для последующего УДАЛЕНИЯ
 * 				если ставится такая задача:
 * 				bDELETE = 0; - не удалять, просто проинформировать факт наличия искомого отсроченного действия
 * 				bDELETE = 1; - удалить первое найденное искомое отсроченное действие и выйти
 * 				bDELETE > 1; - удалить все найденые искомые отсроченные действия и выйти
 * RETURN:		Возвращает собственные данные отсроченного действия в случае успеха поиска,
 * 				если данные нулевые, то ЕДИНИЦУ, в противном случае возвращает НОЛЬ
 ****************************************************************************/
uint32 u32IsThereDeleteFutureTask(uint8 TASK, teFutureTaskModify Modify){
	uint8	QuantityOfTasks 				= sTaskStack.bBusy;
	uint8	TaskPosition 					= sTaskStack.pNearestTask;
	while(QuantityOfTasks){
		if(sTaskStack.sQueue[sTaskStack.iBusyQueue[TaskPosition]].u8Task == TASK){
			switch(Modify){
				case FTM_IsThere:{
					return 			(uint32)  TRUE;
				}break;
				case FTM_Delete:
					QuantityOfTasks 		= 1;
				case FTM_Clean:{
					sTaskStack.sQueue[sTaskStack.iBusyQueue[TaskPosition]].bQueryExe 	= FALSE;
					sTaskStack.iFreeQueue[sTaskStack.iBusyQueue[TaskPosition]]			= sTaskStack.iBusyQueue[TaskPosition];
					sTaskStack.pFreeTask												= sTaskStack.iBusyQueue[TaskPosition];
					uint8	DelPosition 	= TaskPosition;
					uint8	MoveDelPosition = TaskPosition;
					while(MoveDelPosition != sTaskStack.pNearestTask){
						if(MoveDelPosition > 0){	MoveDelPosition--;} 	else{		  MoveDelPosition 	= TASK_STACK_SIZE - 1; }
						sTaskStack.iBusyQueue[DelPosition] 								= sTaskStack.iBusyQueue[MoveDelPosition];
						if(DelPosition > 0){ 		DelPosition--; } 		else{		  DelPosition 		= TASK_STACK_SIZE - 1; }
					}
					sTaskStack.iBusyQueue[sTaskStack.pNearestTask]						= TASK_STACK_SIZE;
					if(++sTaskStack.pNearestTask	>= TASK_STACK_SIZE)					{ sTaskStack.pNearestTask = 0;			   }
					if(--sTaskStack.bBusy			>= TASK_STACK_SIZE)					{ vResetTaskExecuteStack();				   }
				}break;
				case FTM_GetPointerStruct:{
					return			(uint32)    &sTaskStack.sQueue[sTaskStack.iBusyQueue[TaskPosition]];
				}break;
				case FTM_GetTime:{
					return						 sTaskStack.sQueue[sTaskStack.iBusyQueue[TaskPosition]].u32Time;
				}break;
				case FTM_EstimateLeftTime:{
					return					   ( sTaskStack.sQueue[sTaskStack.iBusyQueue[TaskPosition]].u32Time - sTaskStack.u32RealTime );
				}break;
				case FTM_GetData:{
					return						 sTaskStack.sQueue[sTaskStack.iBusyQueue[TaskPosition]].Data.UI32;
				}break;
				case FTM_GetQueuePosition:{
					return			(uint32)   ( sTaskStack.bBusy - QuantityOfTasks );
				}break;
				default:{
					return 			(uint32)  FALSE;
				}break;
			}
		}
		if(++TaskPosition >= TASK_STACK_SIZE){	TaskPosition = 0;}
		QuantityOfTasks--;
	}
	return 							(uint32)  FALSE;
}
/****************************************************************************
 * NAME:		bIsThereFutureTaskData
 * DESCRIPTION: Поиск отсроченного действия в соответстии данным в очереди ожидания,
 * 				для последующего УДАЛЕНИЯ если ставится такая задача
 * 				bDELETE = 0; - не удалять, просто проинформировать факт наличия искомого отсроченного действия
 * 				bDELETE = 1; - удалить первое найденное искомое отсроченное действие и выйти
 * 				bDELETE > 1; - удалить все найденые искомые отсроченные действия и выйти
 * RETURN:		Возвращает собственные данные отсроченного действия в случае успеха поиска,
 * 				если данные нулевые, то ЕДИНИЦУ, в противном случае возвращает НОЛЬ
 ****************************************************************************/
uint32 u32IsThereDeleteFutureTaskData(uint8 TASK, uint32 DATA, teFutureTaskModify Modify){
	uint8	QuantityOfTasks 				= sTaskStack.bBusy;
	uint8	TaskPosition 					= sTaskStack.pNearestTask;
	while(QuantityOfTasks){
		if(		(sTaskStack.sQueue[sTaskStack.iBusyQueue[TaskPosition]].u8Task    == TASK) &&
				(sTaskStack.sQueue[sTaskStack.iBusyQueue[TaskPosition]].Data.UI32 == DATA)){
			switch(Modify){
				case FTM_IsThere:{
					return 			(uint32)  TRUE;
				}break;
				case FTM_Delete:
					QuantityOfTasks 		= 1;
				case FTM_Clean:{
					sTaskStack.sQueue[sTaskStack.iBusyQueue[TaskPosition]].bQueryExe 	= FALSE;
					sTaskStack.iFreeQueue[sTaskStack.iBusyQueue[TaskPosition]]			= sTaskStack.iBusyQueue[TaskPosition];
					sTaskStack.pFreeTask												= sTaskStack.iBusyQueue[TaskPosition];
					uint8	DelPosition 	= TaskPosition;
					uint8	MoveDelPosition = TaskPosition;
					while(MoveDelPosition != sTaskStack.pNearestTask){
						if(MoveDelPosition > 0){	MoveDelPosition--;} 	else{		  MoveDelPosition 	= TASK_STACK_SIZE - 1; }
						sTaskStack.iBusyQueue[DelPosition] 								= sTaskStack.iBusyQueue[MoveDelPosition];
						if(DelPosition > 0){ 		DelPosition--; } 		else{		  DelPosition 		= TASK_STACK_SIZE - 1; }
					}
					sTaskStack.iBusyQueue[sTaskStack.pNearestTask]						= TASK_STACK_SIZE;
					if(++sTaskStack.pNearestTask	>= TASK_STACK_SIZE)					{ sTaskStack.pNearestTask = 0;			   }
					if(--sTaskStack.bBusy			>= TASK_STACK_SIZE)					{ vResetTaskExecuteStack();				   }
				}break;
				case FTM_GetPointerStruct:{
					return			(uint32)    &sTaskStack.sQueue[sTaskStack.iBusyQueue[TaskPosition]];
				}break;
				case FTM_GetTime:{
					return						 sTaskStack.sQueue[sTaskStack.iBusyQueue[TaskPosition]].u32Time;
				}break;
				case FTM_EstimateLeftTime:{
					return					   ( sTaskStack.sQueue[sTaskStack.iBusyQueue[TaskPosition]].u32Time - sTaskStack.u32RealTime );
				}break;
				case FTM_GetData:{
					return						 sTaskStack.sQueue[sTaskStack.iBusyQueue[TaskPosition]].Data.UI32;
				}break;
				case FTM_GetQueuePosition:{
					return			(uint32)   ( sTaskStack.bBusy - QuantityOfTasks );
				}break;
				default:{
					return 			(uint32)  FALSE;
				}break;
			}
		}
		if(++TaskPosition >= TASK_STACK_SIZE){	TaskPosition = 0;}
		QuantityOfTasks--;
	}
	return 							(uint32)  FALSE;
}
/****************************************************************************
 * NAME:		vSetRealTimeModifyTasks
 * DESCRIPTION: Задаёт новое значение системного времени, в соответствии с
 * 				изменением значения системного времени корректирует момент
 * 				отсроченного действия имеющихся заданий в очереди событий
 ****************************************************************************/
void vSetRealTimeModifyTasks(uint32 u32NewRealTime){
	uint32	DifferenceRealTime							= u32NewRealTime - sTaskStack.u32RealTime;
	uint8	QuantityOfTasks 							= sTaskStack.bBusy;
	uint8	TaskPosition 								= sTaskStack.pNearestTask;
	while(QuantityOfTasks){
		sTaskStack.sQueue[sTaskStack.iBusyQueue[TaskPosition]].u32Time += DifferenceRealTime;
		if(++TaskPosition >= TASK_STACK_SIZE){	TaskPosition = 0;}
		QuantityOfTasks--;
	}
	sTaskStack.u32RealTime								= u32NewRealTime;
	RadioRxTx.sNwkState.u32EndLifeWindowRF			   += DifferenceRealTime;
	RadioRxTx.sNwkState.u32StartBeamToBeamTime		   += DifferenceRealTime;
	RadioRxTx.sNwkState.u32LastReceivedPacketTime	   += DifferenceRealTime;
	RadioRxTx.sNwkState.u32LastTransmitedPacketTime	   += DifferenceRealTime;
}
/****************************************************************************
 * NAME:		MemCpy
 * DESCRIPTION: копирует масив данных
 ****************************************************************************/
void MemCpy(uint8* pDest, uint8* pSour, uint8 Number){
	uint8 var;
	for (var = 0; var < Number; ++var) {
		pDest[var] = pSour[var];
	}
}
/****************************************************************************
 * NAME:		MemSet8
 * DESCRIPTION: копирует масив данных
 ****************************************************************************/
void MemSet8(uint8* pDest, uint8 Set, uint8 Number){
	uint8 var;
	for (var = 0; var < Number; ++var) {
		pDest[var] = Set;
	}
}
/****************************************************************************
 * NAME:		MemSet16
 * DESCRIPTION: копирует масив данных
 ****************************************************************************/
void MemSet16(uint16* pDest, uint16 Set, uint8 Number){
	uint8 var;
	for (var = 0; var < Number; ++var) {
		pDest[var] = Set;
	}
}
/****************************************************************************
 * NAME:		vDelay
 * DESCRIPTION: Обеспечивает временную задержку ориентировочно в микросекундах
 ****************************************************************************/
void vDelay(uint32 TIME){
	for(;TIME!=0; TIME--){}
}
/****************************************************************************
 * NAME:		vTransHexToDec
 * DESCRIPTION: Преобразует двоичный код массива в десятичный по длине массива
 * 				Корректировано под LitleEndian
 ****************************************************************************/
void vTransHexToDec(uint8* Dec, uint8* Hex, uint8 LengthHex){
	//18 44 67 44 07 37 09 55 16 16
	tsuUint64 uHex;
	uint8 i,j;
	for (i=0; i<8; i++){
		if (i<LengthHex){ uHex.UI8[i] = Hex[i];}
		else			{ uHex.UI8[i] = 0;     }
	}
	switch (LengthHex){
		// i - количество декад в ответе, j - количество байт в ответе (необходимая ёмкость приёмника)
	case 1:{i = 3;  j = 2;}	break;
	case 2:{i = 5;  j = 3;}	break;
	case 3:{i = 8;  j = 4;}	break;
	case 4:{i = 10; j = 5;}	break;
	case 5:{i = 13; j = 7;}	break;
	case 6:{i = 15; j = 8;}	break;
	case 7:{i = 17; j = 9;}	break;
	case 8:{i = 20; j = 10;}break;
	default:{ i = 0; j = 0;}
	}
	vClearString(Dec, j);
	for(;i>0;){
		i--; j=i/2;
		if((i&1)==1){Dec[j] = 16 *	u8DecCountValue(&uHex.UI64, uic64DecHex[i]);}
		else{		 Dec[j] +=		u8DecCountValue(&uHex.UI64, uic64DecHex[i]);}
	}
}
uint8 u8DecCountValue(uint64* SoS, uint64 Const){
	uint8 rest = 0;
	uint8 i=1;
	while(i>0){
		if (*SoS<Const)	{ i=0;}
		else{ rest++; *SoS -= Const;}
	}
	return rest;
}
void vClearString(uint8* String, uint8 Length){
	uint8 i=0;
	for(;i<Length;i++){
		String[i] = 0;
	}
}
/****************************************************************************
 * NAME:		vTransDecToHex
 * DESCRIPTION: Преобразует десятичный код массива в двоичный по длине массива
 ****************************************************************************/
void vTransDecToHex(uint8* Hex, uint8* Dec, uint8 LengthDec){
	tsuUint64 uHex;
	uint8 i,j,k;
	uHex.UI64 = 0;
	switch (LengthDec){
		// i - ёмкость приёмника в байтах
		case 1:{i = 1;}	break;
		case 2:{i = 2;}	break;
		case 3:{i = 3;}	break;
		case 4:{i = 4;}	break;
		case 5:{i = 5;}	break;
		case 6:{i = 5;}	break;
		case 7:{i = 6;}	break;
		case 8:{i = 7;}break;
		case 9:{i = 8;}	break;
		case 10:{i = 8;}break;
		default:{ i = 0;}
	}
	j = LengthDec*2;
	for(k=LengthDec; k>0; ){
		k--;
		uHex.UI64 += (Dec[k]/16) *	uic64DecHex[--j];
		uHex.UI64 += (Dec[k]&15) *	uic64DecHex[--j];
	}
	for (j=0; j<i; j++){ Hex[j] = uHex.UI8[j];}
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
