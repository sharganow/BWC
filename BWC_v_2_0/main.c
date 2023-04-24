/**
 * 			БЛОК БЕСПРОВОДНОЙ КОММУНИКАЦИИ
 * 			Block of Wireless Communication
 * */
/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include "initialise.h"
#include "spi.h"
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern		tsHwTx 					sTx;
extern		tsHwRx 					sRx;
extern		tsUartOutStack			sHeapTxUart;
extern		tsTimerExecute			sTaskStack;
extern		tsCC110lOutStack		sStackRxRF;
extern		tsCC110lOutStack		sStackTxRF;
extern		tsCC110L 				RadioRxTx;
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
tsuUint16 su16Temperature;
/****************************************************************************/
/***        Exported Function Prototypes                                  ***/
/****************************************************************************/
void 	vGPIO_Initialization(void);
void	vTIM17_Initialization(void);
void	vDMA1_Initialization(void);
void	vWWGD_Initialization(void);
void	vSPI_Initialization(void);
void	vUSART_Initialization(void);
bool_t	bAddTaskExeInTime(uint8 TASK, uint32 TIME, uint32 DATA);
void	vCheckUartEndOut(void);
void	vPacket485SeparatorExe(void);
void	vUartOut(void);
bool_t	bRfPacketSeparatorExe(void);
void	vRfTxRxPacketCleaner(tsCC110lOutStack*	pStackTxRxRF);
bool_t	bIsThereMsgUart(void);
bool_t	bMsgTransitTo485(uint8* pMsg, uint8 DestAddress, uint8 AddDateLength);
bool_t	bSendPacketToICP(uint8* pDate, uint8 DateLength);
uint32	vCC110lWork(void);
void	vWritePktToFIFO(void);
void	vResetTaskExecuteStack(void);
uint32	u32IsThereDeleteFutureTask(uint8 TASK, teFutureTaskModify Modify);
void	vSetRealTimeModifyTasks(uint32 u32NewRealTime);
/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
void	vExePacket(uint8*	pMsg, uint8 sourceMessage);
void	vExePacket485(tsHeadMessage* pHead);
void	vExecuteNextTask(void);
/****************************************************************************/
/***        Local Functions			                                      ***/
/****************************************************************************/
/****************************************************************************
 * NAME:		main
 * DESCRIPTION: Main user routine.
 ****************************************************************************/
int main(void){
	su16Temperature.UI16 = 0;
	vResetTaskExecuteStack();
	vDMA1_Initialization();
	vGPIO_Initialization();
	vTIM17_Initialization();
	vUSART_Initialization();
	vSPI_Initialization();
	vWWGD_Initialization();
    while(1){
    	if(sRx.bDataOnRxLine){
    		if(sRx.SilenceElement	!= DMA1_Channel5->CNDTR){
    			sRx.bExe = TRUE;	// vPacketSeparatorExe();
    			sRx.SilenceElement = DMA1_Channel5->CNDTR;
    		}
    	}
    	else{
    		if(sRx.SilenceElement	!= DMA1_Channel5->CNDTR){
    			sRx.bDataOnRxLine = TRUE;
    		}
    	}
    	vCC110lWork();
    	if(sRx.bExe)							{ vPacket485SeparatorExe();	}
    	if(sTx.bTx)								{ vCheckUartEndOut();		}
    	if(sHeapTxUart.bExe)					{ vUartOut();		}
    	if(sTaskStack.bBusy)					{ vExecuteNextTask();		}
    	if(sStackRxRF.QuantityPackets)			{ bRfPacketSeparatorExe();  }
    	/*if(TIM_GetFlagStatus  ( TIM17, TIM_FLAG_Update)==SET){
												  TIM_ClearFlag  (TIM17, TIM_FLAG_Update);
												  sTaskStack.u32RealTime++;
    	}*/
    }
}
/*****************************************************************************
 * NAME:		vExePacket
 * DESCRIPTION: Исполнитель входящих сообщений
 ****************************************************************************/
void vExePacket(uint8*	pMsg, uint8 sourceMessage){
	switch(pMsg[1]){
		case MEPE_ICP:{}break;
		case MEPE_SetAllWorkData:
		{
			bMsgTransitTo485(pMsg, DA_EBS, 2);
		}break;
		case MEPE_BeginHeating:
		case MEPE_StartCount:
		case MEPE_StopBathProcedure:
		case MEPE_SwitchCompressor:
		case MEPE_PresenceAerosolTherapy:
		case MEPE_AckDutyCycleData:
		{
			bMsgTransitTo485(pMsg, DA_EBS, 0);
		}break;
		case MEPE_AckSetAllWorkData:
		case MEPE_AckBeginHeating:
		case MEPE_AckSwitchOnCompressor:
		case MEPE_AckSwitchOffCompressor:
		case MEPE_AckStartCount:
		case MEPE_AckStopBathProcedure:
		case MEPE_tsDutyCycleData:
		case MEPE_AckOccureErr1:
		case MEPE_AckOccureErr2:
		case MEPE_AckOccureErr3:
		{
			bSendPacketToICP(pMsg, *pMsg);
		}break;
		case MEPE_DS18S20temperature:{
			if(sourceMessage == SM_CC110L){
				if(pMsg[0] == 3){
					bMsgTransitTo485(pMsg, DA_EBS, 0);
				}
			}
			else{
				if(sourceMessage == SM_ST3485){
					bSendPacketToICP(pMsg, *pMsg);
				}
			}
		}break;
		case MEPE_SwitchesStates:{
			if(pMsg[0] == 2){
				bMsgTransitTo485(pMsg, DA_EBS, 0);
			}
			else{
				bSendPacketToICP(pMsg, *pMsg);
			}
		}break;
	}
}
/****************************************************************************
 * NAME:		vExePacket485
 * DESCRIPTION: Исполнитель локальных входящих сообщений от устройств RS485
 ****************************************************************************/
void vExePacket485(tsHeadMessage* pHead){
	if(pHead->DestAddress == DA_BWC){
		switch(pHead->pData[1]){
			case MEPE_LocalExchange:{
				switch(*pHead->pData){
					case 2:{
						if(!bIsThereMsgUart()){
							tsuUint32 Time;
							Time.UI32 = sTaskStack.u32RealTime / 10;
							uint8 SendTimetoEBS[] = {6, MEPE_LocalExchange, Time.UI8[0], Time.UI8[1], Time.UI8[2], Time.UI8[3] };
							bMsgTransitTo485(SendTimetoEBS, DA_EBS, 0);
						}
						sHeapTxUart.bExe = TRUE;
					}break;
					case 6:{
						tsuUint32 Time;
						Time.UI8[3] = pHead->pData[2]; Time.UI8[2] = pHead->pData[3];
						Time.UI8[1] = pHead->pData[4]; Time.UI8[0] = pHead->pData[5];
						vSetRealTimeModifyTasks(Time.UI32 * 10);
					}break;
				}/**/
			}break;
			default:{
				vExePacket(pHead->pData,  SM_ST3485);
			}break;
		}
	}
}
/*****************************************************************************
 * NAME:		vExecuteNextTask
 * DESCRIPTION: Выполняет следующее задание по наступлению момента
 ****************************************************************************/
void vExecuteNextTask(void){
	tsTaskTime*	pTask = &sTaskStack.sQueue[sTaskStack.iBusyQueue[sTaskStack.pNearestTask]];
	if(sTaskStack.u32RealTime >= pTask->u32Time){
		if(pTask->bQueryExe){
			switch(pTask->u8Task){
				case Task_CC110L:{}break;
				case Task_WWGD:{
					bAddTaskExeInTime(Task_WWGD, app_TIME_MS(12), 0);
					WWDG_SetCounter  ( 0x7F );
				}break;
				case Task_CleanStackTxRF:{
					while(sStackTxRF.QuantityPackets){
						vRfTxRxPacketCleaner(&sStackTxRF);
					}
				}break;
			}
		}
		u32IsThereDeleteFutureTask(pTask->u8Task, FTM_Delete);
	}
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
