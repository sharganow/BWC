/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include "initialise.h"
#include "spi.h"
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern	tsTimerExecute			sTaskStack;
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
tsuUint16 				sRxCC110l[0x32];
tsuUint16 				sTxCC110l[0x32];
tuCNFGRGSTR				uRegCC110l;
tsCC110L 				RadioRxTx;
tsCC110lOutStack		sStackTxRF;
tsCC110lOutStack		sStackRxRF;
/****************************************************************************/
/***        Exported Function Prototypes                                  ***/
/****************************************************************************/
void	vExePacket						(uint8*	pMsg,	uint8 sourceMessage);
uint32 	u32IsThereDeleteFutureTask		(uint8 TASK, 					teFutureTaskModify Modify);
uint32	u32IsThereDeleteFutureTaskData	(uint8 TASK, 	uint32 DATA, 	teFutureTaskModify Modify);
bool_t	bAddTaskExeInTime				(uint8 TASK, 	uint32 TIME, 	uint32 DATA);
void	vDeleteFutureTask				(uint8 TASK);
void	MemCpy							(uint8* pDest, uint8* pSour, uint8 Number);
void	MemSet8							(uint8* pDest, uint8 Set, uint8 Number);
uint16	u16GetCRC						(uint8* pvMsg, uint8 Displace,uint8 u8MsgLen);
void	vDMA1_Initialization			(void);
bool_t	bMsgTransitTo485				(uint8* pMsg, uint8 DestAddress, uint8 AddDateLength);
/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
void	vSPI_Initialization(void);
void	vCC110lWork(void);
void	vDMA1ExecuteSPI(uint8	DoubleNum);
uint8	u8GetOneRegCC110l(uint8	Address);
void	vGetConfRegCC110l(uint8	StartAddress, uint8	Num);
void 	vConfOneRegCC110l(uint8	Address, uint8	Value);
void	vSetConfRegCC110l(uint8	StartAddress, uint8	Num);
uint8	u8GetStatusRegister(uint8	StatusReg);
void	vCommandStrobe(uint8	COMMAND);
void	vReadTwoBytePA_TABLE(void);
void	vWriteTwoBytePA_TABLE(void);
void 	u8ReadFIFO(void);
void	vWritePktToFIFO(void);
void	vWriteMarkerToFIFO(void);
bool_t	bPutMsgRFToHeap(tsCC110lOutStack*	pStackTxRxRF, tsHeadRFMessage* pHead);
bool_t	bSendPacketToICP(uint8* pDate, uint8 DateLength);
bool_t	bGetMsgRFFromHeap(tsHeadRFMessage* pHead, tsCC110lOutStack*	pStackTxRxRF);
bool_t	bRfPacketSeparatorExe(void);
void	vRfTxRxPacketCleaner(tsCC110lOutStack*	pStackTxRxRF);
/****************************************************************************/
/***        Local Functions			                                      ***/
/****************************************************************************/
/****************************************************************************
 * NAME:		vSPI_Initialization
 * DESCRIPTION: Конфигурирует SPI
 ****************************************************************************/
void vSPI_Initialization(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    /* Connect PA4 to SPI1_NSS */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_0);
    /* Connect PA5 to SPI1_SCK */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0);
    /* Connect PA6 to SPI1_MISO */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_0);
    /* Connect PA7 to SPI1_MOSI */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0);

    SPI1->CR1 = SPI_CR1_MSTR	| SPI_CR1_BR_0 		| SPI_CR1_BR_1	| SPI_CR1_BR_2 | SPI_CR1_SSI;
    SPI1->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN 	| SPI_CR2_SSOE	| SPI_CR2_DS_0 |
    			SPI_CR2_DS_1 	| SPI_CR2_DS_2 		/*| SPI_CR2_FRXTH*/;

	sStackRxRF.FirstPacket 	  	= 0;
	sStackRxRF.FreePacket 		= 0;
	sStackRxRF.EmptyPackets	  	= MSG_CC11_TX_RX_SIZE;
	sStackRxRF.QuantityPackets 	= 0;

	sStackTxRF.FirstPacket 	  	= 0;
	sStackTxRF.FreePacket 		= 0;
	sStackTxRF.EmptyPackets	  	= MSG_CC11_TX_RX_SIZE;
	sStackTxRF.QuantityPackets 	= 0;

    RadioRxTx.teState								= CC110l_Reset;

	RadioRxTx.sNwkState.bOrphan						= TRUE;
	RadioRxTx.sNwkState.u8CountAbsenceBeam			= FALSE;
	RadioRxTx.sNwkState.eDataOnEther				= TE_NONE;
	RadioRxTx.sNwkState.eCurrentWindowRF			= WP_WaitOfPacket;
	RadioRxTx.sNwkState.uAllBeenWindows.u16Clean	= FALSE;

	RadioRxTx.sNwkState.u32EndLifeWindowRF			= sTaskStack.u32RealTime;
	RadioRxTx.sNwkState.u32StartBeamToBeamTime		= sTaskStack.u32RealTime;
	RadioRxTx.sNwkState.u32LastReceivedPacketTime	= sTaskStack.u32RealTime;
	RadioRxTx.sNwkState.u32LastTransmitedPacketTime	= sTaskStack.u32RealTime;
}
/****************************************************************************
 * NAME:		vCC110lWork
 * DESCRIPTION: Read Number (Num) Configuration Registers of CC110L
 * 				from Start Address inclusive
 * RETURNS: 	Value TIME for bAddTaskExecuteTime(uint8, uint32 TIME, uint32)
 ****************************************************************************/
void vCC110lWork(void){
	switch(RadioRxTx.teState){
		case CC110l_Reset:{
			vCommandStrobe(SNOP);
			if(RadioRxTx.bCHIP_RDYn){
				RadioRxTx.teState = CC110l_Reset;
			}
			else{
				vCommandStrobe(SRES);
				RadioRxTx.u32WatchDogTime = sTaskStack.u32RealTime + app_TIME_MS(3);
				RadioRxTx.teState 								= CC110l_Start;
			}
		}break;
		case CC110l_Start:{			// - С этого места CC110l восстанавливается в работе
			vCommandStrobe(SNOP);
			if(!RadioRxTx.bCHIP_RDYn){
				vCommandStrobe(SFRX);
				vCommandStrobe(SFTX);
				RadioRxTx.teState 								= CC110l_Initialise;
			}
			else{
				if( RadioRxTx.u32WatchDogTime <= sTaskStack.u32RealTime ){
					RadioRxTx.teState = CC110l_Reset;
				}
			}
		}break;
		case CC110l_Initialise:{
			tsuUint16									  s16WirelessNumber;
			s16WirelessNumber.UI16						= WIRELESS_NUMBER;
			vGetConfRegCC110l							( IOCFG2, 	0x30	);

			uRegCC110l.Byte[IOCFG2]						= IOCFG2_Value;				/*PF0*/	/* 	Associated to the RX FIFO: Asserts when RX FIFO is filled at or above the RX FIFO threshold. Deasserts when RX FIFO is
																								drained below the same threshold */
			uRegCC110l.Byte[IOCFG1]						= IOCFG1_Value;				/*PA6*/	/* 	Asserts when sync word has been sent / received, and de-asserts at the end of the packet. In RX, the pin will also de-assert
																								when a packet is discarded due to address or maximum length filtering or when the radio enters RXFIFO_OVERFLOW state.
																								In TX the pin will de-assert if the TX FIFO underflows.*/
			uRegCC110l.Byte[IOCFG0]						= IOCFG0_Value;				/*PA2*/ /* Clear channel assessment. High when RSSI level is below threshold (dependent on the current CCA_MODE setting). */
			uRegCC110l.Byte[FIFOTHR]					= FIFOTHR_Value;
			uRegCC110l.Byte[SYNC1]						= s16WirelessNumber.UI8[1];
			uRegCC110l.Byte[SYNC0]						= s16WirelessNumber.UI8[0];
			uRegCC110l.Byte[PKTLEN]						= PKTLEN_Value;
			uRegCC110l.Byte[PKTCTRL1]					= PKTCTRL1_Value;
			uRegCC110l.Byte[PKTCTRL0]					= PKTCTRL0_Value;
			uRegCC110l.Byte[ADDR]						= RF_BWC;
			uRegCC110l.Byte[CHANNR]						= WIRELESS_CHANNEL;
			uRegCC110l.Byte[FSCTRL1]					= FSCTRL1_Value;
			uRegCC110l.Byte[FSCTRL0]					= FSCTRL0_Value;
			uRegCC110l.Byte[FREQ2]						= FREQ2_Value;
			uRegCC110l.Byte[FREQ1]						= FREQ1_Value;
			uRegCC110l.Byte[FREQ0]						= FREQ0_Value;
			uRegCC110l.Byte[MDMCFG4]					= MDMCFG4_Value;
			uRegCC110l.Byte[MDMCFG3]					= MDMCFG3_Value;
			uRegCC110l.Byte[MDMCFG2]					= MDMCFG2_Value;
			uRegCC110l.Byte[MDMCFG1]					= MDMCFG1_Value;
			uRegCC110l.Byte[MDMCFG0]					= MDMCFG0_Value;
			uRegCC110l.Byte[DEVIATN]					= DEVIATN_Value;
			uRegCC110l.Byte[MCSM2]						= MCSM2_Value;
			uRegCC110l.Byte[MCSM1]						= MCSM1_Value;
			uRegCC110l.Byte[MCSM0]						= MCSM0_Value;
			uRegCC110l.Byte[FOCCFG]						= FOCCFG_Value;
			uRegCC110l.Byte[BSCFG]						= BSCFG_Value;
			uRegCC110l.Byte[AGCCTRL2]					= AGCCTRL2_Value;
			uRegCC110l.Byte[AGCCTRL1]					= AGCCTRL1_Value;
			uRegCC110l.Byte[AGCCTRL0]					= AGCCTRL0_Value;


			uRegCC110l.Byte[RESERVED20]					= RESERVED20_Value;
			uRegCC110l.Byte[FREND1]						= FREND1_Value;
			uRegCC110l.Byte[FREND0]						= FREND0_Value;
			uRegCC110l.Byte[FSCAL3]						= FSCAL3_Value;
			uRegCC110l.Byte[FSCAL2]						= FSCAL2_Value;
			uRegCC110l.Byte[FSCAL1]						= FSCAL1_Value;
			uRegCC110l.Byte[FSCAL0]						= FSCAL0_Value;


			uRegCC110l.Byte[RESERVED29]					= RESERVED29_Value;
			uRegCC110l.Byte[RESERVED2A]					= RESERVED2A_Value;
			uRegCC110l.Byte[RESERVED2B]					= RESERVED2B_Value;
			uRegCC110l.Byte[TEST2]						= TEST2_Value;
			uRegCC110l.Byte[TEST1]						= TEST1_Value;
			uRegCC110l.Byte[TEST0]						= TEST0_Value;

			vSetConfRegCC110l							( IOCFG2, 	0x30 	 );
			vConfOneRegCC110l							( PATABLE, 	TX_POWER_BWC );
			RadioRxTx.teState 							= CC110l_Calibrate;
		}break;
		case CC110l_Calibrate:{
			vCommandStrobe(SCAL);
			RadioRxTx.teState = CC110l_IDLE;
			RadioRxTx.u32WatchDogTime = sTaskStack.u32RealTime + app_TIME_MS(3);
		}break;
		case CC110l_IDLE:{
			RadioRxTx.uMARCSTATE.Byte 	= u8GetStatusRegister(MARCSTATE);
			switch(RadioRxTx.uMARCSTATE.Field.MARC_STATE){
				case MRCSMS_SLEEP:
				case MRCSMS_XOFF:{
						RadioRxTx.teState = CC110l_Reset;
				}break;
				case MRCSMS_ENDCAL:{
					vCommandStrobe(SIDLE);
					RadioRxTx.u32WatchDogTime = sTaskStack.u32RealTime + app_TIME_MS(1);
				}break;
				case MRCSMS_RXFIFO_OVERFLOW:{
					vCommandStrobe(SFRX);
				}
				case MRCSMS_RX:
				case MRCSMS_RX_END:
				case MRCSMS_RX_RST:
				case MRCSMS_TXRX_SWITCH:{
					RadioRxTx.teState = CC110l_Calibrate;
				}break;
				case MRCSMS_TXFIFO_UNDERFLOW:{
					vCommandStrobe(SFTX);
				}
				case MRCSMS_FSTXON:
				case MRCSMS_TX:
				case MRCSMS_TX_END:
				case MRCSMS_RXTX_SWITCH:{
					RadioRxTx.teState = CC110l_Calibrate;
				}break;
				case MRCSMS_VCOON_MC:
				case MRCSMS_REGON_MC:
				case MRCSMS_MANCAL:
				case MRCSMS_VCOON:
				case MRCSMS_REGON:
				case MRCSMS_STARTCAL:
				case MRCSMS_BWBOOST:
				case MRCSMS_FS_LOCK:
				case MRCSMS_IFADCON:
				default:{
					if( RadioRxTx.u32WatchDogTime <= sTaskStack.u32RealTime ){
						RadioRxTx.teState = CC110l_Reset;
					}
				}break;
				case MRCSMS_IDLE:{
					vCommandStrobe				 (SRX);
					RadioRxTx.teState 			= CC110l_RX_Wait;
					RadioRxTx.u32WatchDogTime	= sTaskStack.u32RealTime + app_TIME_MS(3);
					switch(RadioRxTx.sNwkState.eDataOnEther){
						default:
						case TE_NONE:{
							if(RadioRxTx.sNwkState.bOrphan)	{}
							else							{}
						}break;
						case TE_Transmit:{
							switch(RadioRxTx.sNwkState.eCurrentWindowRF){
								case WP_TransmitPacket:{
									switch(RadioRxTx.sNwkState.uAllBeenWindows.s8Field.u8Marker){
										case MP_FirstPacket:{
											RadioRxTx.sNwkState.uAllBeenWindows.s8Field.bHasBeenPacket.bFirstPacket 	= TRUE;
										}break;
										case MP_SecondPacket:{
											RadioRxTx.sNwkState.uAllBeenWindows.s8Field.bHasBeenPacket.bSecondPacket	= TRUE;
										}break;
										case MP_ThirdPacket:{
											RadioRxTx.sNwkState.uAllBeenWindows.s8Field.bHasBeenPacket.bThirdPacket 	= TRUE;
										}break;
									}
									RadioRxTx.sNwkState.eCurrentWindowRF			= WP_PauseAfterTransmitPkt;
									RadioRxTx.sNwkState.u32StartBeamToBeamTime 		= RadioRxTx.sNwkState.u32LastTransmitedPacketTime;
									RadioRxTx.sNwkState.u32EndLifeWindowRF			= RadioRxTx.sNwkState.u32LastTransmitedPacketTime + app_TIME_MS(PauseAfterTransmitPkt);
								}break;
								case WP_TransmitAcknowledge:{
									RadioRxTx.sNwkState.eCurrentWindowRF			= WP_PauseAfterTransmitAck;
									RadioRxTx.sNwkState.u32EndLifeWindowRF			= RadioRxTx.sNwkState.u32LastTransmitedPacketTime + app_TIME_MS(PauseAfterTransmitAck);
								}break;
								default:{
									if(RadioRxTx.sNwkState.bOrphan){
										RadioRxTx.sNwkState.eCurrentWindowRF		= WP_WaitOfPacket;
									}
									else{
										RadioRxTx.sNwkState.eCurrentWindowRF		= WP_BeAbleBothRandomPacket;
									}
									RadioRxTx.sNwkState.u32StartBeamToBeamTime 		= sTaskStack.u32RealTime;
									RadioRxTx.sNwkState.u32EndLifeWindowRF			= sTaskStack.u32RealTime + app_TIME_MS(PeriodLifeBeamToBeam);
								}break;
							}
						}break;
						case TE_Receive:{															/* Сложная и ёмкая часть */
							switch(sStackRxRF.u13Index_u3Marker.d8Field.u8Marker){
								case MP_FirstPacket:
								case MP_SecondPacket:
								case MP_ThirdPacket:{
									RadioRxTx.sNwkState.eCurrentWindowRF			= WP_PauseAfterReceivePkt;
									RadioRxTx.sNwkState.u32StartBeamToBeamTime	 	= RadioRxTx.sNwkState.u32LastReceivedPacketTime;
									RadioRxTx.sNwkState.u32EndLifeWindowRF			= RadioRxTx.sNwkState.u32LastReceivedPacketTime + app_TIME_MS(PauseAfterReceivedPkt);
								}break;
								case MP_Acknowledge:{
									RadioRxTx.sNwkState.eCurrentWindowRF			= WP_ReceiveAcknowledge;
									tuIndexMarker					  		  		  s13Index_u3Marker;
									uint8*								  			  pSource;
									pSource											= sStackTxRF.pHeap[sStackTxRF.FirstPacket].Date;
									s13Index_u3Marker.ShortInt.UI8[0]				= pSource[3];
									s13Index_u3Marker.ShortInt.UI8[1]				= pSource[4];
									if(s13Index_u3Marker.d8Field.u8NumberPacketRF  == sStackRxRF.u13Index_u3Marker.d8Field.u8NumberPacketRF){
										RadioRxTx.sNwkState.uAllBeenWindows.s8Field.bHasBeenPacket.bAcknowledge = TRUE;
									}
									RadioRxTx.sNwkState.u32EndLifeWindowRF			= RadioRxTx.sNwkState.u32LastReceivedPacketTime;
								}break;
								default:{
									if(RadioRxTx.sNwkState.bOrphan){
										RadioRxTx.sNwkState.eCurrentWindowRF		= WP_WaitOfPacket;
									}
									else{
										RadioRxTx.sNwkState.eCurrentWindowRF		= WP_BeAbleBothRandomPacket;
									}
									RadioRxTx.sNwkState.u32StartBeamToBeamTime 		= sTaskStack.u32RealTime;
									RadioRxTx.sNwkState.u32EndLifeWindowRF			= sTaskStack.u32RealTime + app_TIME_MS(PeriodLifeBeamToBeam);
								}break;
							}
						}break;
					}
					RadioRxTx.sNwkState.eDataOnEther = TE_NONE;
				}break;
			}
		}break;

		case CC110l_RX_Wait:{
			RadioRxTx.uMARCSTATE.Byte 	= u8GetStatusRegister(MARCSTATE);
			switch(RadioRxTx.uMARCSTATE.Field.MARC_STATE){
				case MRCSMS_RX:{
#ifdef 	BWC_v_2_0
					if((GPIOA->IDR & GPIO_Pin_6) || (GPIOF->IDR & GPIO_Pin_0)){
#else
#ifdef BWC_v_3_0
					if((GPIOA->IDR & GPIO_Pin_6) || (GPIOA->IDR & GPIO_Pin_9)){
#endif 	/* BWC_v_3_0 */
#endif	/* BWC_v_2_0 */
						RadioRxTx.teState 		  			= CC110l_RX_Receive;
						RadioRxTx.sNwkState.eDataOnEther	= TE_Receive;
						RadioRxTx.u32WatchDogTime 			= sTaskStack.u32RealTime + app_TIME_MS(OneByteCommunication * 39);
					}
					else{
						switch(RadioRxTx.sNwkState.eCurrentWindowRF){
							case WP_WaitOfPacket:{
								if(RadioRxTx.sNwkState.u32EndLifeWindowRF 	   <= sTaskStack.u32RealTime){
									RadioRxTx.sNwkState.u32EndLifeWindowRF 	   += app_TIME_MS(PeriodLifeBeamToBeam);
									if(++RadioRxTx.sNwkState.u8CountAbsenceBeam > ResolveBWCAbsence){
										 RadioRxTx.sNwkState.u8CountAbsenceBeam	= FALSE;
										 RadioRxTx.teState 						= CC110l_Reset;
									}
								}
							}break;
							case WP_PauseAfterTransmitPkt:{
								/* Здесь ожидается получение подтверждения от получателя переданного пакета */
								if(RadioRxTx.sNwkState.u32EndLifeWindowRF 	   <= sTaskStack.u32RealTime){
									RadioRxTx.sNwkState.eCurrentWindowRF 		= WP_ReceiveAcknowledge;
									/* подтверждения получения пакета не было, необходимо произвести реинициализацию */
									RadioRxTx.sNwkState.uAllBeenWindows.s8Field.bHasBeenPacket.bAcknowledge = FALSE;
								}
							}break;
							case WP_ReceiveAcknowledge:{
								if(RadioRxTx.sNwkState.u32EndLifeWindowRF 	   <= sTaskStack.u32RealTime){
									RadioRxTx.sNwkState.eCurrentWindowRF 						= WP_PauseAfterReceiveAck;
									/*	Здесь должен быть анализ произведённой передачи пакета и дальнейшее
									 * действие с ним ( удалить/модифицировать/ретранслировать ) */
									if(RadioRxTx.sNwkState.bOrphan){
										if(	RadioRxTx.sNwkState.uAllBeenWindows.s8Field.bHasBeenPacket.bAcknowledge	){
											RadioRxTx.sNwkState.bOrphan							= FALSE;
											RadioRxTx.sNwkState.u8CountAbsenceBeam				= FALSE;
								        	uint8	u8SendToEBS[] 								= { 3, MEPE_ICP, TRUE };
								        	bMsgTransitTo485									( u8SendToEBS, DA_EBS, 0 );
								        	RadioRxTx.sNwkState.u32EndLifeWindowRF			   += app_TIME_MS(PauseAfterReceivedAck);
								        	u32IsThereDeleteFutureTask							( Task_CleanStackTxRF, FTM_Delete );
										}
										else{
											RadioRxTx.sNwkState.u32EndLifeWindowRF			   += app_TIME_MS(PauseAfterTransmitAck);
										}
										RadioRxTx.sNwkState.uAllBeenWindows.u16Clean			= FALSE;
										vRfTxRxPacketCleaner(&sStackTxRF);
									}
									else{
										if(	RadioRxTx.sNwkState.uAllBeenWindows.s8Field.bHasBeenPacket.bAcknowledge	){
											RadioRxTx.sNwkState.uAllBeenWindows.u16Clean		= FALSE;
											vRfTxRxPacketCleaner(&sStackTxRF);
											RadioRxTx.sNwkState.u32EndLifeWindowRF			   += app_TIME_MS(PauseAfterReceivedAck);
										}
										else{
											RadioRxTx.sNwkState.u32EndLifeWindowRF			   += app_TIME_MS(PauseAfterTransmitAck);
											if(RadioRxTx.sNwkState.uAllBeenWindows.s8Field.bHasBeenPacket.bThirdPacket){
												RadioRxTx.sNwkState.uAllBeenWindows.u16Clean	= FALSE;
												vRfTxRxPacketCleaner(&sStackTxRF);
												RadioRxTx.sNwkState.u32EndLifeWindowRF		   += app_TIME_MS(PauseAfterReceivedAck);

												if(++RadioRxTx.sNwkState.u8CountAbsenceBeam 	> ResolveBWCAbsence  ){	/* Здесь можно обозначить, что связь пропала */
													 RadioRxTx.sNwkState.u8CountAbsenceBeam		= FALSE;
													 RadioRxTx.sNwkState.bOrphan				= TRUE;
													 uint8	u8SendToEBS[] 						= {3, MEPE_ICP, FALSE};
													 bMsgTransitTo485							( u8SendToEBS, DA_EBS, 0 );
													 bAddTaskExeInTime							( Task_CleanStackTxRF, app_TIME_S(5), 0);
												}
											}
										}
									}
								}
							}break;
							case WP_PauseAfterReceiveAck:{
								if(RadioRxTx.sNwkState.u32EndLifeWindowRF 	   <= sTaskStack.u32RealTime){
									if(RadioRxTx.sNwkState.bOrphan){
										RadioRxTx.sNwkState.eCurrentWindowRF 	= WP_WaitOfPacket;
									}
									else{
										RadioRxTx.sNwkState.eCurrentWindowRF 	= WP_BeAbleBothRandomPacket;
									}
									RadioRxTx.sNwkState.u32EndLifeWindowRF		= RadioRxTx.sNwkState.u32StartBeamToBeamTime + app_TIME_MS(PeriodLifeBeamToBeam);
								}
							}break;
							case WP_PauseAfterReceivePkt:{
								if(RadioRxTx.sNwkState.u32EndLifeWindowRF 	   <= sTaskStack.u32RealTime){
#ifdef 	BWC_v_2_0
									if( GPIOA->IDR & GPIO_Pin_2 ){
#else
#ifdef BWC_v_3_0
									if( GPIOA->IDR & GPIO_Pin_10 ){
#endif 	/* BWC_v_3_0 */
#endif	/* BWC_v_2_0 */
										vWriteMarkerToFIFO();
									}
									else{
										if((RadioRxTx.sNwkState.u32EndLifeWindowRF  + PauseAfterTransmitPkt - PauseAfterReceivedPkt)  <= sTaskStack.u32RealTime){
											RadioRxTx.sNwkState.u32EndLifeWindowRF 	= sTaskStack.u32RealTime + PauseAfterTransmitAck;
											RadioRxTx.sNwkState.eCurrentWindowRF 	= WP_PauseAfterTransmitAck;
										}
									}
								}
							}break;
							case WP_PauseAfterTransmitAck:{
								if(RadioRxTx.sNwkState.u32EndLifeWindowRF 	   <= sTaskStack.u32RealTime){
									RadioRxTx.sNwkState.eCurrentWindowRF 		= WP_BeAbleBothRandomPacket;
									RadioRxTx.sNwkState.u32EndLifeWindowRF		= RadioRxTx.sNwkState.u32StartBeamToBeamTime + app_TIME_MS(PeriodLifeBeamToBeam);
								}
							}break;
							case WP_BeAbleBothRandomPacket:{
								if(RadioRxTx.sNwkState.u32EndLifeWindowRF 	   <= sTaskStack.u32RealTime){
#ifdef 	BWC_v_2_0
									if( GPIOA->IDR & GPIO_Pin_2 ){
#else
#ifdef BWC_v_3_0
									if( GPIOA->IDR & GPIO_Pin_10 ){
#endif 	/* BWC_v_3_0 */
#endif	/* BWC_v_2_0 */
										vWritePktToFIFO();
									}
									else{
										if((RadioRxTx.sNwkState.u32EndLifeWindowRF  + PeriodLifeBeamToBeam)  <= sTaskStack.u32RealTime){
											RadioRxTx.sNwkState.u32EndLifeWindowRF += PeriodLifeBeamToBeam;
											if(++RadioRxTx.sNwkState.u8CountAbsenceBeam 	> ResolveBWCAbsence){
												 RadioRxTx.sNwkState.u8CountAbsenceBeam		= FALSE;
												 RadioRxTx.sNwkState.bOrphan				= TRUE;
												 RadioRxTx.teState 		  					= CC110l_Reset;
											}
										}
									}
								}
								else{
									if(sStackTxRF.QuantityPackets || RadioRxTx.sNwkState.bOrphan){
#ifdef 	BWC_v_2_0
										if( GPIOA->IDR & GPIO_Pin_2 ){
#else
#ifdef BWC_v_3_0
										if( GPIOA->IDR & GPIO_Pin_10 ){
#endif 	/* BWC_v_3_0 */
#endif	/* BWC_v_2_0 */
											vWritePktToFIFO();
										}
									}
								}
							}break;
						}
					}
				}break;
				default:{
					if(RadioRxTx.u32WatchDogTime <= sTaskStack.u32RealTime){
						RadioRxTx.teState = CC110l_IDLE;
					}
				}break;
			}
		}break;

		case CC110l_RX_Receive:{
			if((GPIOA->IDR & GPIO_Pin_6)){
				if(RadioRxTx.u32WatchDogTime <= sTaskStack.u32RealTime){
					RadioRxTx.teState 									= CC110l_Reset;
				}
			}
			else{
				RadioRxTx.uMARCSTATE.Byte 	= u8GetStatusRegister(MARCSTATE);
				switch(RadioRxTx.uMARCSTATE.Field.MARC_STATE){
					case MRCSMS_RX_RST:
					case MRCSMS_RX_END:
					case MRCSMS_RXFIFO_OVERFLOW:
						vCommandStrobe(SIDLE);
					case MRCSMS_IDLE:{
						RadioRxTx.teState								= CC110l_IDLE;
						RadioRxTx.sNwkState.u32LastReceivedPacketTime	= sTaskStack.u32RealTime;
						u8ReadFIFO();
					}break;
					case MRCSMS_RX:{}
					default:{
						if(RadioRxTx.u32WatchDogTime <= sTaskStack.u32RealTime){
							RadioRxTx.teState 							= CC110l_Reset;
						}
					}break;
				}
			}
		}break;

		case CC110l_TX_Transmit:{
			if((GPIOA->IDR & GPIO_Pin_6)){
				if(RadioRxTx.u32WatchDogTime <= sTaskStack.u32RealTime){
					RadioRxTx.teState 									= CC110l_Reset;
				}
			}
			else{
				RadioRxTx.uMARCSTATE.Byte 	= u8GetStatusRegister(MARCSTATE);
				switch(RadioRxTx.uMARCSTATE.Field.MARC_STATE){
					case MRCSMS_TXFIFO_UNDERFLOW:
					case MRCSMS_TX_END:
						vCommandStrobe(SIDLE);
					case MRCSMS_IDLE:{
						RadioRxTx.teState 								= CC110l_IDLE;
						RadioRxTx.sNwkState.u32LastTransmitedPacketTime = sTaskStack.u32RealTime;
						vCommandStrobe(SFTX);
					}break;
					case MRCSMS_TX:{}
					default:{
						if(RadioRxTx.u32WatchDogTime <= sTaskStack.u32RealTime){
							RadioRxTx.teState 							= CC110l_Reset;
						}
					}break;
				}
			}
		}break;
	}
}
/****************************************************************************
 * NAME:		vDMA1ExecuteSPI
 * DESCRIPTION: Read/Write Double Number Byte by SPI interface from/to CC110l
 ****************************************************************************/
void vDMA1ExecuteSPI(uint8	DoubleNum){
#ifdef 	BWC_v_2_0
	GPIO_ResetBits  		  (GPIOA, GPIO_Pin_1);
#endif	/* BWC_v_2_0 */
	SPI1->CR1|= SPI_CR1_SPE;						//  SPI_Cmd  (SPI1, ENABLE);
	/*while					  (GPIOA->IDR & GPIO_Pin_6);vDMA1_Initialization();*/
	DMA1_Channel2->CCR	   &= (uint16)(~DMA_CCR_EN);
	DMA1_Channel3->CCR	   &= (uint16)(~DMA_CCR_EN);

	DMA1_Channel2->CNDTR 	= DMA1_Channel3->CNDTR 	=	DoubleNum;
	DMA1->IFCR				= DMA_IFCR_CTCIF2;

	DMA1_Channel2->CCR	   |= DMA_CCR_EN;
	DMA1_Channel3->CCR	   |= DMA_CCR_EN;

	while					  ((DMA1->ISR & DMA_ISR_TCIF2)!= DMA_ISR_TCIF2);

	DMA1_Channel2->CCR	   &= (uint16)(~DMA_CCR_EN);
	DMA1_Channel3->CCR	   &= (uint16)(~DMA_CCR_EN);
	SPI1->CR1 			   &= (uint16_t)~((uint16_t)SPI_CR1_SPE);
#ifdef 	BWC_v_2_0
	GPIO_SetBits  		  	  (GPIOA, GPIO_Pin_1);
#endif	/* BWC_v_2_0 */
}
/****************************************************************************
 * NAME:		u8ReadOneConfRegCC110l
 * DESCRIPTION: Read One Configuration Registers of CC110L by Address
 * RETURNS: 	Value of Configuration Register
 ****************************************************************************/
uint8 u8GetOneRegCC110l(uint8	Address){
	tuHeaderByte					  sHeaderByte;
	sHeaderByte.Field.BURST			= HB_Not_Burst;
	sHeaderByte.Field.DIRECTION		= HB_Read;
	sHeaderByte.Field.ADDRESS		= Address;
	sTxCC110l[0].UI8[0] 			= sHeaderByte.Byte;
	vDMA1ExecuteSPI(1);
	return sRxCC110l[0].UI8[1];
}
/****************************************************************************
 * NAME:		vGetConfRegCC110l
 * DESCRIPTION: Read Number (Num) Configuration Registers of CC110L
 * 				from Start Address inclusive
 ****************************************************************************/
void vGetConfRegCC110l(uint8	StartAddress, uint8	Num){
	uint8*							  pSource;
	uint8							  i;
	bool_t							  Odd;
	tuHeaderByte					  sHeaderByte;
	tuChipStatusByte				  sStatus;
	sHeaderByte.Field.BURST			= HB_Burst;
	sHeaderByte.Field.DIRECTION		= HB_Read;
	sHeaderByte.Field.ADDRESS		= StartAddress;

	Odd 							= Num & 0b00000001;
	if(Odd){
		sTxCC110l[0].UI8[0] 		= sHeaderByte.Byte;
		pSource						= (uint8*)&sRxCC110l[0].UI8[1];
	}
	else{
		sTxCC110l[0].UI8[0] 		= SNOP;
		sTxCC110l[0].UI8[1] 		= sHeaderByte.Byte;
		pSource						= (uint8*)&sRxCC110l[1].UI8[0];
	}

	vDMA1ExecuteSPI					  ((Num / 2) + 1);

	for(i = 0; i < Num; i++){
		uRegCC110l.Byte[StartAddress++] = pSource[i];
	}

	sStatus.Byte					= sRxCC110l[Num / 2].UI8[1];
	RadioRxTx.bCHIP_RDYn			= sStatus.Field.CHIP_RDYn;
	RadioRxTx.u8MainStateMachine	= sStatus.Field.STATE;
	RadioRxTx.u8AvailableRxFifo		= sStatus.Field.FIFO_BYTES_AVAILABLE;
}
/****************************************************************************
 * NAME:		vConfOneRegCC110l
 * DESCRIPTION: Write One Configuration Register of CC110L
 * 				with Value
 ****************************************************************************/
void vConfOneRegCC110l(uint8	Address, uint8	Value){
	tuHeaderByte					  sHeaderByte;
	sHeaderByte.Field.BURST			= HB_Not_Burst;
	sHeaderByte.Field.DIRECTION		= HB_Write;
	sHeaderByte.Field.ADDRESS		= Address;

	sTxCC110l[0].UI8[0] 			= sHeaderByte.Byte;
	sTxCC110l[0].UI8[1] 			= Value;

	vDMA1ExecuteSPI(1);
}
/****************************************************************************
 * NAME:		vSetConfRegCC110l
 * DESCRIPTION: Write Number (Num) Configuration Registers of CC110L
 * 				from Start Address inclusive
 ****************************************************************************/
void vSetConfRegCC110l(uint8	StartAddress, uint8	Num){
	uint8*							  pRecipient;
	uint8							  i;
	bool_t							  Odd;
	tuHeaderByte					  sHeaderByte;
	tuChipStatusByte				  sStatus;
	sHeaderByte.Field.BURST			= HB_Burst;
	sHeaderByte.Field.DIRECTION		= HB_Write;
	sHeaderByte.Field.ADDRESS		= StartAddress;

	Odd 							= Num & 0b00000001;
	if(Odd){
		sTxCC110l[0].UI8[0] 		= sHeaderByte.Byte;
		pRecipient					= (uint8*)&sTxCC110l[0].UI8[1];
	}
	else{
		sTxCC110l[0].UI8[0] 		= SNOP;
		sTxCC110l[0].UI8[1] 		= sHeaderByte.Byte;
		pRecipient					= (uint8*)&sTxCC110l[1].UI8[0];
	}

	for(i = 0; i < Num; i++){
		pRecipient[i]				= uRegCC110l.Byte[StartAddress++];
	}

	vDMA1ExecuteSPI					  ((Num / 2) + 1);

	sStatus.Byte					= sRxCC110l[Num / 2].UI8[1];
	RadioRxTx.bCHIP_RDYn			= sStatus.Field.CHIP_RDYn;
	RadioRxTx.u8MainStateMachine	= sStatus.Field.STATE;
	RadioRxTx.u8AvailableRxFifo		= sStatus.Field.FIFO_BYTES_AVAILABLE;
}
/****************************************************************************
 * NAME:		u8GetStatusRegister
 * DESCRIPTION: Read Status Register with passed address StatusReg from CC110L
 * RETURNS: 	value Status Register
 ****************************************************************************/
uint8 u8GetStatusRegister(uint8	StatusReg){
	if((PARTNUM > StatusReg)||(StatusReg > RXBYTES)){	return 0xFF;}

	tuHeaderByte					  sHeaderByte;
	tuChipStatusByte				  sStatus;
	sHeaderByte.Field.BURST			= HB_Burst;
	sHeaderByte.Field.DIRECTION		= HB_Read;
	sHeaderByte.Field.ADDRESS		= StatusReg;
	sTxCC110l[0].UI8[0] 			= sHeaderByte.Byte;
	sTxCC110l[0].UI8[1] 			= SNOP;

	vDMA1ExecuteSPI(1);

	sStatus.Byte					= sRxCC110l[0].UI8[0];
	RadioRxTx.bCHIP_RDYn			= sStatus.Field.CHIP_RDYn;
	RadioRxTx.u8MainStateMachine	= sStatus.Field.STATE;
	RadioRxTx.u8AvailableRxFifo		= sStatus.Field.FIFO_BYTES_AVAILABLE;
	return	sRxCC110l[0].UI8[1];
}
/****************************************************************************
 * NAME:		vCommandStrobe
 * DESCRIPTION: Transfer a single strobe COMMAND to CC110L
 ****************************************************************************/
void vCommandStrobe(uint8	COMMAND){
	if((SRES > COMMAND)||(COMMAND > SNOP)){	return;}

	tuHeaderByte					  sHeaderByte;
	tuChipStatusByte				  sStatus;
	sHeaderByte.Field.BURST			= HB_Not_Burst;
	sHeaderByte.Field.DIRECTION		= HB_Write;
	sHeaderByte.Field.ADDRESS		= COMMAND;
	sTxCC110l[0].UI8[0] 			= SNOP;
	sTxCC110l[0].UI8[1] 			= sHeaderByte.Byte;

	vDMA1ExecuteSPI(1);

	sStatus.Byte					= sRxCC110l[0].UI8[0];
	RadioRxTx.bCHIP_RDYn			= sStatus.Field.CHIP_RDYn;
	RadioRxTx.u8MainStateMachine	= sStatus.Field.STATE;
	RadioRxTx.u8FreeTxFifo			= sStatus.Field.FIFO_BYTES_AVAILABLE;
}
/****************************************************************************
 * NAME:		vReadTwoBytePA_TABLE
 * DESCRIPTION: Read Two Bytes of PA_TABLE just PA_TABLE0 and PA_TABLE1
 ****************************************************************************/
void vReadTwoBytePA_TABLE(void){
	tuHeaderByte					  sHeaderByte;
	tuChipStatusByte				  sStatus;
	sHeaderByte.Field.BURST			= HB_Burst;
	sHeaderByte.Field.DIRECTION		= HB_Read;
	sHeaderByte.Field.ADDRESS		= PATABLE;
	sTxCC110l[0].UI8[0] 			= SNOP;
	sTxCC110l[0].UI8[1] 			= sHeaderByte.Byte;
	sTxCC110l[1].UI8[0] 			= SNOP;
	sTxCC110l[1].UI8[1] 			= SNOP;

	vDMA1ExecuteSPI(2);

	RadioRxTx.su16PA_TABLE.UI8[0]	= sRxCC110l[1].UI8[0];
	RadioRxTx.su16PA_TABLE.UI8[1]	= sRxCC110l[1].UI8[1];
	sStatus.Byte					= sRxCC110l[0].UI8[0];
	RadioRxTx.bCHIP_RDYn			= sStatus.Field.CHIP_RDYn;
	RadioRxTx.u8MainStateMachine	= sStatus.Field.STATE;
	RadioRxTx.u8FreeTxFifo			= sStatus.Field.FIFO_BYTES_AVAILABLE;
}
/****************************************************************************
 * NAME:		vWriteTwoBytePA_TABLE
 * DESCRIPTION: Read Two Bytes of PA_TABLE just PA_TABLE0 and PA_TABLE1
 ****************************************************************************/
void vWriteTwoBytePA_TABLE(void){
	tuHeaderByte					  sHeaderByte;
	tuChipStatusByte				  sStatus;
	sHeaderByte.Field.BURST			= HB_Burst;
	sHeaderByte.Field.DIRECTION		= HB_Write;
	sHeaderByte.Field.ADDRESS		= PATABLE;
	sTxCC110l[0].UI8[0] 			= SNOP;
	sTxCC110l[0].UI8[1] 			= sHeaderByte.Byte;
	sTxCC110l[1].UI8[0] 			= RadioRxTx.su16PA_TABLE.UI8[0];
	sTxCC110l[1].UI8[1] 			= RadioRxTx.su16PA_TABLE.UI8[1];

	vDMA1ExecuteSPI(2);

	sStatus.Byte					= sRxCC110l[0].UI8[0];
	RadioRxTx.bCHIP_RDYn			= sStatus.Field.CHIP_RDYn;
	RadioRxTx.u8MainStateMachine	= sStatus.Field.STATE;
	RadioRxTx.u8FreeTxFifo			= sStatus.Field.FIFO_BYTES_AVAILABLE;
}
/****************************************************************************
 * NAME:		vReadFIFO
 * DESCRIPTION: Read Num byte from RX FIFO bufer
 * RETURN:		teMarkerPacketRF
 ****************************************************************************/
void u8ReadFIFO(void){
	uint8							  	  Num;
	bool_t							  	  Odd;
	uint8*								  pRecipient;
	uint8*							  	  pSource;
	tuHeaderByte					  	  sHeaderByte;
	tuIndexMarker					  	  s13Index_u3Marker;
	tsuUint16 							  u16CRC;

	RadioRxTx.uRXBYTES.Byte				= u8GetStatusRegister(RXBYTES);
	Num									= RadioRxTx.uRXBYTES.Field.NUM_RXBYTES;
	RadioRxTx.u8RSSI 					= u8GetStatusRegister(RSSI);

	if(Num > 6){
		sHeaderByte.Field.BURST			= HB_Burst;
		sHeaderByte.Field.DIRECTION		= HB_Read;
		sHeaderByte.Field.ADDRESS		= FIFO;

		Odd 							= Num & 0b00000001;
		if(Odd){
			sTxCC110l[0].UI8[0] 		= sHeaderByte.Byte;
			pSource						= (uint8*)&sRxCC110l[0].UI8[1];
		}
		else{
			sTxCC110l[0].UI8[0] 		= SNOP;
			sTxCC110l[0].UI8[1] 		= sHeaderByte.Byte;
			pSource						= (uint8*)&sRxCC110l[1].UI8[0];
		}

		vDMA1ExecuteSPI					  ((Num / 2) + 1);

										  s13Index_u3Marker.ShortInt.UI8[0]
										= pSource[3];
										  s13Index_u3Marker.ShortInt.UI8[1]
										= pSource[4];

		if(sStackRxRF.u13Index_u3Marker.d8Field.u8NumberPacketRF != s13Index_u3Marker.d8Field.u8NumberPacketRF){
			if((s13Index_u3Marker.d8Field.u8Marker != MP_Acknowledge) && (sStackRxRF.EmptyPackets)){
				pSource[0]			   -= 1;
				u16CRC.UI16				= u16GetCRC(pSource, 0, pSource[0]);
				if((u16CRC.UI8[0] == pSource[pSource[0]]) && (u16CRC.UI8[1] == pSource[pSource[0] + 1])){
					pRecipient				= sStackRxRF.pHeap[sStackRxRF.FreePacket].Date;
					MemCpy					( pRecipient, pSource, Num );
											  sStackRxRF.pHeap[sStackRxRF.FreePacket].Length
											= Num;
					pRecipient[0] 		   += 1;
					if( ++sStackRxRF.FreePacket	   >= MSG_CC11_TX_RX_SIZE){
						  sStackRxRF.FreePacket 	= 0;
					}
					sStackRxRF.EmptyPackets--;
					sStackRxRF.QuantityPackets++;
				}
			}
		}
		sStackRxRF.u13Index_u3Marker.ShortInt.UI16 					= s13Index_u3Marker.ShortInt.UI16;
		if(RadioRxTx.sNwkState.u8CountAbsenceBeam){
			RadioRxTx.sNwkState.u8CountAbsenceBeam--;
		}
	}
	else{
		vCommandStrobe												( SFRX );
	}
}
/****************************************************************************
 * NAME:		vWritePktToFIFO
 * DESCRIPTION: Write Current Packet to TX FIFO bufer
 ****************************************************************************/
void vWritePktToFIFO(void){
	uint8								  Num;
	bool_t								  Odd;
	uint8*								  pRecipient;
	uint8*								  pSource;
	tuHeaderByte						  sHeaderByte;
	tuIndexMarker					  	  s13Index_u3Marker;
	tsuUint16 							  u16CRC;

	sHeaderByte.Field.BURST				= HB_Burst;
	sHeaderByte.Field.DIRECTION			= HB_Write;
	sHeaderByte.Field.ADDRESS			= FIFO;

	if(sStackTxRF.QuantityPackets	   == FALSE){
		uint8 SendEmptyMsgToICP[] 		= {2, MEPE_EBS};			// поскольку сообщения нет, но отправить нужно - обозначим отправителя
		bSendPacketToICP				( SendEmptyMsgToICP, SendEmptyMsgToICP[0] );
	}

	pSource								= sStackTxRF.pHeap[sStackTxRF.FirstPacket].Date;
	Num 								= pSource[0];
	Odd 								= Num & 0b00000001;
	if(Odd){
		sTxCC110l[0].UI8[0] 			= sHeaderByte.Byte;
		pRecipient						=(uint8*)&sTxCC110l[0].UI8[1];
	}
	else{
		sTxCC110l[0].UI8[0] 			= SNOP;
		sTxCC110l[0].UI8[1] 			= sHeaderByte.Byte;
		pRecipient						=(uint8*)&sTxCC110l[1].UI8[0];
	}

	u16CRC.UI16							= u16GetCRC(pSource, 0, Num);
	MemCpy								 (pRecipient, pSource, Num);
	pRecipient[0]					   += 1;
	pRecipient[Num++]					= u16CRC.UI8[0];
	pRecipient[Num++]					= u16CRC.UI8[1];

	vDMA1ExecuteSPI						  ((Num / 2) + 1);
	vCommandStrobe(STX);

	s13Index_u3Marker.ShortInt.UI8[0]	= pSource[3];
	s13Index_u3Marker.ShortInt.UI8[1]	= pSource[4];
										  RadioRxTx.sNwkState.uAllBeenWindows.s8Field.u8Marker
										= s13Index_u3Marker.d8Field.u8Marker;
	if(s13Index_u3Marker.d8Field.u8Marker   < MP_ThirdPacket){
		s13Index_u3Marker.d8Field.u8Marker += 1;
	}
	pSource[3]							= s13Index_u3Marker.ShortInt.UI8[0];
	pSource[4]							= s13Index_u3Marker.ShortInt.UI8[1];

	RadioRxTx.teState 					= CC110l_TX_Transmit;
	RadioRxTx.sNwkState.eDataOnEther	= TE_Transmit;
	RadioRxTx.sNwkState.eCurrentWindowRF= WP_TransmitPacket;

	RadioRxTx.u32WatchDogTime 			= sTaskStack.u32RealTime + app_TIME_MS(OneByteCommunication * (Num + 10));
}
/****************************************************************************
 * NAME:		vWriteMarkerToFIFO
 * DESCRIPTION: Write Acknowledge of Received Packet to TX FIFO bufer
 ****************************************************************************/
void vWriteMarkerToFIFO(void){
	uint8								  Num;
	bool_t								  Odd;
	uint8*								  pRecipient;
	uint8*								  pSource;
	tuHeaderByte						  sHeaderByte;
	tuIndexMarker					  	  s13Index_u3Marker;
	uint8							  	  sMarkerToEther[8];
	tsuUint16 							  u16CRC;

	sHeaderByte.Field.BURST				= HB_Burst;
	sHeaderByte.Field.DIRECTION			= HB_Write;
	sHeaderByte.Field.ADDRESS			= FIFO;
	s13Index_u3Marker.ShortInt.UI16		= sStackRxRF.u13Index_u3Marker.ShortInt.UI16;
	s13Index_u3Marker.d8Field.u8Marker	= MP_Acknowledge;

	sMarkerToEther[0]					= 2 + 5;
	sMarkerToEther[1]					= RF_ICP;
	sMarkerToEther[2]					= RF_BWC;
	sMarkerToEther[3]					= s13Index_u3Marker.ShortInt.UI8[0];
	sMarkerToEther[4]					= s13Index_u3Marker.ShortInt.UI8[1];
	sMarkerToEther[5]					= 2;
	sMarkerToEther[6]					= MEPE_ICP;

	pSource								= sMarkerToEther;
	Num 								= pSource[0];
	Odd 								= Num & 0b00000001;
	if(Odd){
		sTxCC110l[0].UI8[0] 			= sHeaderByte.Byte;
		pRecipient						=(uint8*)&sTxCC110l[0].UI8[1];
	}
	else{
		sTxCC110l[0].UI8[0] 			= SNOP;
		sTxCC110l[0].UI8[1] 			= sHeaderByte.Byte;
		pRecipient						=(uint8*)&sTxCC110l[1].UI8[0];
	}

	u16CRC.UI16							= u16GetCRC(pSource, 0, Num);
	MemCpy								 (pRecipient, pSource, Num);
	pRecipient[0]					   += 1;
	pRecipient[Num++]					= u16CRC.UI8[0];
	pRecipient[Num++]					= u16CRC.UI8[1];

	vDMA1ExecuteSPI						  ((Num / 2) + 1);
	vCommandStrobe(STX);

	RadioRxTx.teState 					= CC110l_TX_Transmit;
	RadioRxTx.sNwkState.eDataOnEther	= TE_Transmit;
	RadioRxTx.sNwkState.eCurrentWindowRF= WP_TransmitAcknowledge;

	RadioRxTx.u32WatchDogTime 			= sTaskStack.u32RealTime + app_TIME_MS(OneByteCommunication * (Num + 10));
}
/****************************************************************************
 * NAME:		bPutMsgRFToHeap
 * DESCRIPTION: Функция помещения сообщения в общий поток очереди ожидания
 ****************************************************************************/
bool_t bPutMsgRFToHeap(tsCC110lOutStack*	pStackTxRxRF, tsHeadRFMessage* pHead){
	if( pStackTxRxRF->QuantityPackets  == 0){
		pStackTxRxRF->FirstPacket 	  	= 0;
		pStackTxRxRF->FreePacket 		= 0;
		pStackTxRxRF->EmptyPackets	  	= MSG_CC11_TX_RX_SIZE;
		pStackTxRxRF->QuantityPackets 	= 0;
	}

	if( pStackTxRxRF->EmptyPackets 	   == FALSE ){
		return FALSE;										// для контроля безопасности соблюдения последовательности исходящих сообщений по Ether
	}
	else{
										  pStackTxRxRF->pHeap[pStackTxRxRF->FreePacket].Length
										= pHead->Length;
		uint8*	pRecipient				= pStackTxRxRF->pHeap[pStackTxRxRF->FreePacket].Date;
		pRecipient[0] 					= pHead->Length;
		pRecipient[1] 					= pHead->DestinationAddress;
		pRecipient[2] 					= pHead->SourceAddress;
		pRecipient[3] 					= pHead->u13Index_u3Marker.ShortInt.UI8[0];
		pRecipient[4] 					= pHead->u13Index_u3Marker.ShortInt.UI8[1];

		MemCpy							(&pRecipient[5], pHead->pDate, pHead->DateLength);

		if( ++pStackTxRxRF->FreePacket >= MSG_CC11_TX_RX_SIZE){
			pStackTxRxRF->FreePacket 	= 0;
		}
		pStackTxRxRF->QuantityPackets++;
		pStackTxRxRF->EmptyPackets--;
		return TRUE;
	}
}
/****************************************************************************
 * NAME:		bGetMsgRFFromHeap
 * DESCRIPTION: Функция извлечения сообщения из общего потока очереди ожидания
 * 				для последующей загрузки в радиоресивер
 ****************************************************************************/
bool_t bGetMsgRFFromHeap(tsHeadRFMessage* pHead, tsCC110lOutStack*	pStackTxRxRF){
	if(pStackTxRxRF->QuantityPackets == 0){
		return FALSE;
	}
	else{
		pHead->pPacket								= pStackTxRxRF->pHeap[pStackTxRxRF->FirstPacket].Date;
		pHead->Length 								= pHead->pPacket[0];
		pHead->DestinationAddress					= pHead->pPacket[1];
		pHead->SourceAddress						= pHead->pPacket[2];
		pHead->u13Index_u3Marker.ShortInt.UI8[0]	= pHead->pPacket[3];
		pHead->u13Index_u3Marker.ShortInt.UI8[1]	= pHead->pPacket[4];
		pHead->pDate								= pHead->pPacket + 5;
		pHead->DateLength							= pHead->Length  - 5;
		return TRUE;
	}
}
/****************************************************************************
 * NAME:		bSendPacketToBWC
 * DESCRIPTION: Отправляет пакет данных по радио каналу CC110L
 ****************************************************************************/
bool_t bSendPacketToICP(uint8* pDate, uint8 DateLength){
	if((RadioRxTx.sNwkState.bOrphan) && (sStackTxRF.QuantityPackets > 1)){
		vRfTxRxPacketCleaner(&sStackTxRF);
		return FALSE;
	}
	else{
		tsHeadRFMessage											  sTxHead;
		if( (DateLength > PACKETLENGTH)  )						{ return FALSE; }
		else{
			sTxHead.Length										= DateLength + 5;
			sTxHead.DestinationAddress							= RF_ICP;
			sTxHead.SourceAddress								= RF_BWC;
			sTxHead.u13Index_u3Marker.d8Field.u8NumberPacketRF	= ++sStackTxRF.u13Index_u3Marker.d8Field.u8NumberPacketRF;
			sTxHead.u13Index_u3Marker.d8Field.u8Marker			= MP_FirstPacket;
			sTxHead.DateLength 									= DateLength;
			sTxHead.pDate										= pDate;
			return bPutMsgRFToHeap								( &sStackTxRF, &sTxHead );
		}
	}
}
/****************************************************************************
 * NAME:		vRfPacketSeparator
 * DESCRIPTION: Разделяет пакеты команд, принятые CC110l
 * 				Выпоняет инструкцию выделенного пакета
 ****************************************************************************/
bool_t bRfPacketSeparatorExe(void){
	tsHeadRFMessage 				  RxHead;
	if(bGetMsgRFFromHeap(&RxHead, &sStackRxRF)){
		vExePacket(RxHead.pDate, SM_CC110L);
		vRfTxRxPacketCleaner(&sStackRxRF);
		return						  TRUE;
	}
	else{
		sStackRxRF.FirstPacket 	  	= 0;
		sStackRxRF.FreePacket 		= 0;
		sStackRxRF.EmptyPackets	  	= MSG_CC11_TX_RX_SIZE;
		sStackRxRF.QuantityPackets 	= 0;
		return 						  FALSE;
	}
}
/****************************************************************************
 * NAME:		vRfTxRxPacketCleaner
 * DESCRIPTION: Удаляет из хранилищ отслуживший пакет
 ****************************************************************************/
void vRfTxRxPacketCleaner(tsCC110lOutStack*	pStackTxRxRF){
	if( pStackTxRxRF->QuantityPackets ){
		pStackTxRxRF->EmptyPackets++;
		pStackTxRxRF->QuantityPackets--;
		MemSet8	(	pStackTxRxRF->pHeap[pStackTxRxRF->FirstPacket].Date,
					0,
					pStackTxRxRF->pHeap[pStackTxRxRF->FirstPacket].Length	);
		pStackTxRxRF->pHeap[pStackTxRxRF->FirstPacket].Length	= 0;
		if(++pStackTxRxRF->FirstPacket >= MSG_CC11_TX_RX_SIZE){
			 pStackTxRxRF->FirstPacket  = 0;
		}
	}
	else{
		pStackTxRxRF->FirstPacket 	  	= 0;
		pStackTxRxRF->FreePacket 		= 0;
		pStackTxRxRF->EmptyPackets	  	= MSG_CC11_TX_RX_SIZE;
		pStackTxRxRF->QuantityPackets 	= 0;
	}
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
