/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include "initialise.h"
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern			tsuUint16 		sRxCC110l[0x32];
extern			tsuUint16 		sTxCC110l[0x32];
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
tsHwRx 			sRx;
tsHwTx 			sTx;
tsUartOutStack	sHeapTxUart;
bool_t			bDirectLineTxNotRx;
/****************************************************************************/
/***        Exported Function Prototypes                                  ***/
/****************************************************************************/
void 	MemCpy(uint8* pDest, uint8* pSour, uint8 Number);
uint16	u16GetCRC(uint8* pvMsg, uint8 Displace,uint8 u8MsgLen);
void	vExePacket(uint8*	pMsg, uint8 sourceMessage);
void	vExePacket485(tsHeadMessage* pHead);
/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
void 	vUSART_Initialization(void);
void	vPacket485SeparatorExe(void);
bool_t	bMsgUartPutToHeap(tsUartOut* pMsg);
bool_t	GetMsgUartFromHeap(tsUartOut* pMsg);
bool_t	bIsThereMsgUart(void);
void	vUartOut(void);
void	vCheckUartEndOut(void);
void	vFillHeadMessage(tsHeadMessage* pHead, uint8* pMsg);
uint8	u8CollectMessage(tsHeadMessage* pHead);
bool_t	bMsgTransitTo485(uint8* pMsg, uint8 DestAddress, uint8 AddDateLength);
/****************************************************************************/
/***        Local Functions			                                      ***/
/****************************************************************************/

/****************************************************************************
 * NAME: vUSART_Initialization
 * DESCRIPTION: Конфигурирует USART
 ****************************************************************************/
void vUSART_Initialization(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#ifdef 	BWC_v_2_0
    /* Connect PA9 to USART1_Tx */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
    /* Connect PA10 to USART1_Rx */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
#else
#ifdef BWC_v_3_0
    /* Connect PA2 to USART1_Tx */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
    /* Connect PA3 to USART1_Rx */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
#endif 	/* BWC_v_3_0 */
#endif	/* BWC_v_2_0 */

	USART_InitTypeDef sUSART_InitStruct;
	USART_StructInit ( &sUSART_InitStruct  );
	sUSART_InitStruct.USART_BaudRate = 38400;
	USART_Init ( USART1 ,&sUSART_InitStruct );
	USART_Cmd(USART1, ENABLE);

	USART_SetReceiverTimeOut(USART1, 35);
	USART_ReceiverTimeOutCmd(USART1, ENABLE);

	USART_ITConfig(USART1, USART_IT_RTO, ENABLE);
	USART1->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;

	NVIC_InitTypeDef sNVIC_InitStruct;
	sNVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	sNVIC_InitStruct.NVIC_IRQChannelPriority = 3;
	sNVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&sNVIC_InitStruct);

	sTx.NextElement 			= 0;
	sTx.bTx 					= FALSE;
	sRx.NextElement 			= sRx.FirstElement   = 0;
	sRx.State 					= FIND_MAGIC_WORD;
	sRx.bDataOnRxLine 			= FALSE;
	sRx.SilenceElement			= MSG_UART_TX_RX_SIZE;
	sHeapTxUart.FirstElement 	= sHeapTxUart.NextElement = 0;
	sHeapTxUart.bExe 			= FALSE;
	bDirectLineTxNotRx 			= FALSE;
#ifdef 	BWC_v_2_0
	GPIO_ResetBits  		  	  (GPIOA, GPIO_Pin_8);
#else
#ifdef BWC_v_3_0
	GPIOA->BRR 					= GPIO_Pin_1;
#endif 	/* BWC_v_3_0 */
#endif	/* BWC_v_2_0 */
}
/****************************************************************************
 * NAME:		USART1_IRQHandler
 * DESCRIPTION: Вектор прерывания от USART1
 ****************************************************************************/
void USART1_IRQHandler(void){
	if(SET == USART_GetFlagStatus(USART1, USART_FLAG_RTO)){
		USART_ClearFlag(USART1, USART_FLAG_RTO);
		sRx.SilenceElement	= DMA1_Channel5->CNDTR;
		sRx.bDataOnRxLine 	= FALSE;
		sRx.bExe = TRUE;	//vPacketSeparatorExe();
	}
}
/****************************************************************************
 * NAME:		vPacketSeparator
 * DESCRIPTION: Разделяет пакеты команд, принятые приёмником USART_Rx,
 * 				Выпоняет инструкцию выделенного пакета
 ****************************************************************************/
void vPacket485SeparatorExe(void){
	sRx.bExe = FALSE;
	tsuUint16 u16CRC;
	uint8 u8MsgLen,i;
	sRx.NextElement = MSG_UART_TX_RX_SIZE - DMA1_Channel5->CNDTR;
	switch(sRx.State){
		default:
		case FIND_MAGIC_WORD:
			for(i=sRx.NextElement-sRx.FirstElement;i>0;i--){
			  if(sRx.Data[sRx.FirstElement++] == MAGIC_NUMBER){
				 sRx.State = FIND_END; i=1;		/* закончить цикл */
			  }
			}
		case FIND_END:
			if(sRx.State != FIND_END){
				if(!sRx.bDataOnRxLine){		sRx.FirstElement = sRx.NextElement;  sRx.State = FIND_MAGIC_WORD; }
			}
			else{
				i = sRx.NextElement-sRx.FirstElement;
				u8MsgLen = sRx.Data[sRx.FirstElement+1];
				if((i >= 5)&&(i >= (u8MsgLen+2))){
					tsHeadMessage Head;
					if(sRx.FirstElement > sRx.NextElement){
						uint8 TempRxData[128];
						uint8 TempFirst, TempNext, TempI;
						for(TempI = 0, TempFirst = sRx.FirstElement, TempNext = u8MsgLen+2; TempNext > 0; TempFirst++, TempI++, TempNext--){
							TempRxData[TempI] = sRx.Data[TempFirst];
						}
						u16CRC.UI16 = u16GetCRC(TempRxData, 0 , u8MsgLen);
						if(u16CRC.UI8[0] == TempRxData[u8MsgLen] && u16CRC.UI8[1] == TempRxData[u8MsgLen+1]){
							vFillHeadMessage(&Head, TempRxData);
							vExePacket485(&Head);
						}
						else{
							tsUartOut sUO;
							sUO.Length = 2;
							MemCpy(sUO.Data, u16CRC.UI8, sUO.Length);
							bMsgUartPutToHeap(&sUO);
						}
					}
					else{
						u16CRC.UI16 = u16GetCRC(sRx.Data, sRx.FirstElement , u8MsgLen);
						if(u16CRC.UI8[0] == sRx.Data[sRx.FirstElement+u8MsgLen] && u16CRC.UI8[1] == sRx.Data[sRx.FirstElement+u8MsgLen+1]){
							vFillHeadMessage(&Head, &sRx.Data[sRx.FirstElement]);
							vExePacket485(&Head);
						}
						else{
							tsUartOut sUO;
							sUO.Length = 2;
							MemCpy(sUO.Data, u16CRC.UI8, sUO.Length);
							bMsgUartPutToHeap(&sUO);
						}
					}
					sRx.State = FIND_MAGIC_WORD;
					sRx.FirstElement = sRx.FirstElement + u8MsgLen + 2;
					if(!sRx.bDataOnRxLine){	sRx.bExe = TRUE;}
				}
				else{
					if(!sRx.bDataOnRxLine){ sRx.FirstElement=sRx.NextElement; sRx.State = FIND_MAGIC_WORD;}
				}
				break;
			}
	}
}
/****************************************************************************
 * NAME:		bMsgUartPutToHeap
 * DESCRIPTION: Функция складирования сообщения в очередь
 ****************************************************************************/
bool_t bMsgUartPutToHeap(tsUartOut* pMsg){
	uint8 next 											= sHeapTxUart.NextElement + 1;
	if( next >= MSG_UART_TX_STACK )						{ next = 0; }
	if( next == sHeapTxUart.FirstElement )				{ return FALSE;}
	else{
		MemCpy											( sHeapTxUart.Msg[sHeapTxUart.NextElement].Data, pMsg->Data, pMsg->Length );
		sHeapTxUart.Msg[sHeapTxUart.NextElement].Length = pMsg->Length;
		sHeapTxUart.NextElement							= next;
		sHeapTxUart.bExe								= TRUE;
		return											  TRUE;
	}
}
/****************************************************************************
 * NAME:		GetMsgUartFromHeap
 * DESCRIPTION: Функция извлечения сообщений из очереди
 ****************************************************************************/
bool_t GetMsgUartFromHeap(tsUartOut* pMsg){
	if( sHeapTxUart.FirstElement == sHeapTxUart.NextElement)	{ return FALSE; }
	else{
		uint8 next										= sHeapTxUart.FirstElement + 1;
		if( next >= MSG_UART_TX_STACK )					{ next = 0; }
		pMsg->Length									= sHeapTxUart.Msg[sHeapTxUart.FirstElement].Length;
		MemCpy											( pMsg->Data, sHeapTxUart.Msg[sHeapTxUart.FirstElement].Data, pMsg->Length );
		sHeapTxUart.FirstElement						= next;
		return											  TRUE;
	}
}
/****************************************************************************
 * NAME:		bIsThereMsgUart
 * DESCRIPTION: Функция проверки наличия сообщений в очереди
 ****************************************************************************/
bool_t bIsThereMsgUart(void){
	if(sHeapTxUart.FirstElement == sHeapTxUart.NextElement)		{	return FALSE;	}
	else														{	return TRUE;	}
}
/****************************************************************************
 * NAME:		vUartOut
 * DESCRIPTION: инициирует передачу данных
 ****************************************************************************/
void vUartOut(void){
	if(!sRx.bDataOnRxLine){
		if(!sTx.bTx){
			if(bIsThereMsgUart()){
				tsUartOut sUO;
				uint8 p8Preamble[] = {255, 254, 253, 252};
				if(!bDirectLineTxNotRx){
					bDirectLineTxNotRx 	= TRUE;
#ifdef 	BWC_v_2_0
					GPIO_SetBits  		(GPIOA, GPIO_Pin_8);
#else
#ifdef BWC_v_3_0
					GPIOA->BSRR			= GPIO_Pin_1;
#endif 	/* BWC_v_3_0 */
#endif	/* BWC_v_2_0 */
				}
				GetMsgUartFromHeap(&sUO);
				MemCpy(&sTx.Data[0], 				  p8Preamble,  sizeof(p8Preamble));
				MemCpy(&sTx.Data[sizeof(p8Preamble)], sUO.Data,    sUO.Length);
				DMA1_Channel4->CNDTR 	= sUO.Length + sizeof(p8Preamble);
				DMA1_Channel4->CCR	   |= DMA_CCR_EN;
				sTx.bTx 				= TRUE;
			}
			else{
				bDirectLineTxNotRx 		= FALSE;
#ifdef 	BWC_v_2_0
				GPIO_ResetBits  		(GPIOA, GPIO_Pin_8);
#else
#ifdef BWC_v_3_0
				GPIOA->BRR				= GPIO_Pin_1;
#endif 	/* BWC_v_3_0 */
#endif	/* BWC_v_2_0 */
			}
			sHeapTxUart.bExe 		= FALSE;
		}
	}
}
/****************************************************************************
 * NAME:		vCheckUartEndOut
 * DESCRIPTION: Проверяет окончание передачи данных по RS485 интерфейсу
 ****************************************************************************/
void vCheckUartEndOut(void){
	if(USART1->ISR & USART_FLAG_TC){
		USART1->ICR 			= USART_FLAG_TC;
		DMA1_Channel4->CCR	   &= (uint16)(~DMA_CCR_EN);
		sTx.bTx 				= FALSE;
		sHeapTxUart.bExe 		= TRUE;
	}
}
/****************************************************************************
 * NAME:		vFillHeadMessage
 * DESCRIPTION: выделяет заголовок сообщения в структуру
 ****************************************************************************/
void vFillHeadMessage(tsHeadMessage* pHead, uint8* pMsg){
	tuAttribute 						  Attribute;
	pHead->pPacket 						= pMsg;
	Attribute.Byte 						= *pMsg;
	pHead->Attribute.Byte 				= *pMsg; pMsg += 1;
	pHead->Length 						= *pMsg; pMsg += 1;
	if(Attribute.Field.bEncript)		{ pHead->bEncript = TRUE;	}
	else								{ pHead->bEncript = FALSE;	}
	if(Attribute.Field.bDA){
		pHead->bDA 						= TRUE;
		pHead->DestAddress 				= *pMsg; pMsg += 1;
	}
	else{
		pHead->bDA 						= FALSE;
	}
	if(Attribute.Field.bSA){
		pHead->bSA 						= TRUE;
		pHead->SourceAddress  			= *pMsg; pMsg += 1;
	}
	else{
		pHead->bSA 						= FALSE;
	}
	pHead->pData 						= pMsg;
	pHead->DateLength 					= (uint8)(pMsg - pHead->pPacket);
	pHead->DateLength 					= pHead->Length - pHead->DateLength;
}
/****************************************************************************
 * NAME:		u8CollectMessage
 * DESCRIPTION: Собирает новое сообщение
 * RETURNS:		длину сообщения
 ****************************************************************************/
uint8 u8CollectMessage(tsHeadMessage* pHead){
	uint8 L = 1;
	tsuUint16 	u16CRC;
	uint8* pMsg; pMsg = pHead->pPacket;

	*pMsg = MAGIC_NUMBER;

	if(pHead->bEncript){pMsg[1] = 0x81;}else{pMsg[1] = 1;} L += 2;

	if(pHead->bDA){
		pMsg[L] = pHead->DestAddress;
		L += 1;		pMsg[1] += 0x41;
	}
	if(pHead->bSA){
		pMsg[L] = pHead->SourceAddress;
		L += 1;		pMsg[1] += 0x21;
	}

	MemCpy(&pMsg[L], pHead->pData, pHead->DateLength);
	L += pHead->DateLength;
	pMsg[2] = L - 1;
	u16CRC.UI16 = u16GetCRC(pMsg, 1, L - 1);
	pMsg[L] = u16CRC.UI8[0];
	pMsg[L+1] = u16CRC.UI8[1];
	L += 2;
	return L;
}
/****************************************************************************
 * NAME:		bMsgTransitTo485
 * DESCRIPTION: Производит пересылку сообщения в RS485 порт и дублирует в UART
 * RETURNS: 	SUCCESS - TRUE; FAIL - FALSE
 ****************************************************************************/
bool_t bMsgTransitTo485(uint8* pMsg, uint8 DestAddress, uint8 AddDateLength){
	tsUartOut 				  sUO;
	tsHeadMessage 			  sHead;
	sHead.DestAddress 		= DestAddress;
	sHead.SourceAddress 	= DA_BWC;
	sHead.bDA = sHead.bSA 	= TRUE; ///	sHead.bEncript = FALSE; // намёк на перспективу
	sHead.DateLength 		= *pMsg + AddDateLength;
	sHead.pData 			= pMsg;
	sHead.pPacket 			= &sUO.Data[0];
	sUO.Length 				= u8CollectMessage(&sHead);
	return 					  bMsgUartPutToHeap(&sUO);
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
