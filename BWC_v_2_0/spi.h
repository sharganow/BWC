/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
// 			Command Strobes
#define 	SRES		0x30		/*	Reset chip.	*/
#define 	SFSTXON		0x31		/*	Enable and calibrate frequency synthesizer
										(if MCSM0.FS_AUTOCAL=1). If in RX (with	CCA):
										Go to a wait state where only the synthesizer
										is running (for quick RX / TX turnaround). 		*/
#define 	SXOFF		0x32		/*	Turn off crystal oscillator */
#define 	SCAL		0x33		/*	Calibrate frequency synthesizer and turn it off.
										SCAL can be strobed from IDLE mode without setting
										manual calibration mode (MCSM0.FS_AUTOCAL=0)	*/
#define 	SRX			0x34		/*	In IDLE state: Enable RX.
										Perform calibration first if MCSM0.FS_AUTOCAL=1.*/
#define 	STX			0x35		/*	In IDLE state: Enable TX.
										Perform calibration first if MCSM0.FS_AUTOCAL=1.
										If in RX state and CCA is enabled:
										Only go to TX if channel is clear.				*/
#define 	SIDLE		0x36		/*	Enter IDLE state	*/
#define 	SPWD		0x39		/*	Enter power down mode when CSn goes high.		*/
#define 	SFRX		0x3A		/*	Flush the RX FIFO buffer. Only issue SFRX in
										IDLE or RXFIFO_OVERFLOW states.					*/
#define 	SFTX		0x3B		/*	Flush the TX FIFO buffer. Only issue SFTX in
										IDLE or TXFIFO_UNDERFLOW states.				*/
#define 	SNOP		0x3D		/*	No operation. May be used to get access
										to the chip status byte.						*/
//			Configuration Registers
#define 	IOCFG2		0x00		/*	GDO2 output pin configuration	*/
#define		IOCFG1		0x01		/*	GDO1 output pin configuration	*/
#define		IOCFG0		0x02		/*	GDO0 output pin configuration	*/
#define 	FIFOTHR		0x03		/*	RX FIFO and TX FIFO thresholds	*/
#define		SYNC1		0x04		/*	Sync word, high byte	*/
#define		SYNC0		0x05		/*	Sync word, low byte	*/
#define		PKTLEN		0x06		/*	Packet length	*/
#define 	PKTCTRL1	0x07		/*	Packet automation control	*/
#define		PKTCTRL0	0x08		/*	Packet automation control	*/
#define		ADDR		0x09		/*	Device address	*/
#define 	CHANNR		0x0A		/*	Channel number	*/
#define		FSCTRL1		0x0B		/*	Frequency synthesizer control	*/
#define		FSCTRL0		0x0C		/*	Frequency synthesizer control	*/
#define		FREQ2		0x0D		/*	Frequency control word, high byte	*/
#define		FREQ1		0x0E		/*	Frequency control word, middle byte	*/
#define		FREQ0		0x0F		/*	Frequency control word, low byte	*/
#define		MDMCFG4		0x10		/*	Modem configuration	*/
#define		MDMCFG3		0x11		/*	Modem configuration	*/
#define		MDMCFG2		0x12		/*	Modem configuration	*/
#define		MDMCFG1		0x13		/*	Modem configuration	*/
#define		MDMCFG0		0x14		/*	Modem configuration	*/
#define		DEVIATN		0x15		/*	Modem deviation setting	*/
#define 	MCSM2		0x16		/*	Main Radio Control State Machine configuration	*/
#define 	MCSM1		0x17		/*	Main Radio Control State Machine configuration	*/
#define 	MCSM0		0x18		/*	Main Radio Control State Machine configuration	*/
#define		FOCCFG		0x19		/*	Frequency Offset Compensation configuration	*/
#define		BSCFG		0x1A		/*	Bit Synchronization configuration	*/
#define		AGCCTRL2	0x1B		/*	AGC control	*/
#define		AGCCTRL1	0x1C		/*	AGC control	*/
#define		AGCCTRL0	0x1D		/*	AGC control	*/
#define		RESERVED20	0x20		/*	Reserved	*/
#define		FREND1		0x21		/*	Front end RX configuration	*/
#define		FREND0		0x22		/*	Front end TX configuration	*/
#define		FSCAL3		0x23		/*	Frequency synthesizer calibration	*/
#define		FSCAL2		0x24		/*	Frequency synthesizer calibration	*/
#define		FSCAL1		0x25		/*	Frequency synthesizer calibration	*/
#define		FSCAL0		0x26		/*	Frequency synthesizer calibration	*/
#define		RESERVED29	0x29
#define		RESERVED2A	0x2A
#define		RESERVED2B	0x2B
#define		TEST2		0x2C		/*	Various test settings	*/
#define		TEST1		0x2D		/*	Various test settings	*/
#define		TEST0		0x2E		/*	Various test settings	*/
//			Status Registers
#define		PARTNUM		0x30/*(0xF0)*/		/*	Part number for CC110L	*/
#define		VERSION		0x31/*(0xF1)*/		/*	Current version number	*/
#define		FREQEST		0x32/*(0xF2)*/		/*	Frequency Offset Estimate	*/
#define		CRC_REG		0x33/*(0xF3)*/		/*	CRC OK	*/
#define		RSSI		0x34/*(0xF4)*/		/*	Received signal strength indication	*/
#define		MARCSTATE	0x35/*(0xF5)*/		/*	Control state machine state	*/
#define		PKTSTATUS	0x38/*(0xF8)*/		/*	Current GDOx status and packet status	*/
#define		TXBYTES		0x3A/*(0xFA)*/		/*	Underflow and number of bytes in the TX FIFO	*/
#define		RXBYTES		0x3B/*(0xFB)*/		/*	Overflow and number of bytes in the RX FIFO	*/
//			FIFO Access
#define		FIFO		0x3F				/*	64-byte TX FIFO and the 64-byte RX FIFO	*/

//			Work restrictions
#define 	PACKETLENGTH		0x3B		/*	Maximal length of payload to send over the radio	*/

// 			PATABLE Access
#define		PATABLE		0x3E
/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
/** \brief Header Byte Read/Write atribute */
typedef enum{
	HB_Write 				= 0,
	HB_Read 				= 1,
}teHeaderByteReadNotWrite;

/** \brief Header Byte Burst atribute */
typedef enum{
	HB_Not_Burst			= 0,
	HB_Burst 				= 1,
}teHeaderByteBurst;

/** \brief Configuration Registers Write and Read Operations with Header Byte*/
typedef struct{
	unsigned char ADDRESS	: 6;	//	6-bit address (A5ЦA0)
	unsigned char BURST		: 1;	//	burst access bit (B)
	unsigned char DIRECTION	: 1;	//	R/W bit
}tsHeaderByte;

/** \brief Header Byte*/
typedef union{
	tsHeaderByte 	Field;
	uint8			Byte;
}tuHeaderByte;

/** \brief List of main state machine modes */
typedef enum{
	MSM_IDLE			 = 0,	/*	IDLE state
									(Also reported for some transitional states instead of SETTLING or CALIBRATE) */
	MSM_RX				 = 1,	//	Receive mode
	MSM_TX				 = 2,	//	Transmit mode
	MSM_FSTXON			 = 3,	//	Fast TX ready
	MSM_CALIBRATE		 = 4,	//	Frequency synthesizer calibration is running
	MSM_SETTLING		 = 5,	//	PLL is settling
	MSM_RXFIFO_OVERFLOW	 = 6,	/*	RX FIFO has overflowed. Read out any
									useful data, then flush the FIFO with SFRX	*/
	MSM_TXFIFO_UNDERFLOW = 7	//	TX FIFO has underflowed. Acknowledge with SFTX
}teMainStateMachine;

/** \brief Description Filds of the Chip Status Byte (Status Byte Summary)*/
typedef struct{
	unsigned char FIFO_BYTES_AVAILABLE: 4;	//	The number of bytes available in the RX FIFO or free bytes in the TX FIFO
	unsigned char STATE		: 3;	//	Indicates the current main state machine mode (teMainStateMachine)
	unsigned char CHIP_RDYn	: 1;	//	Stays high until power and crystal have stabilized. Should always be low when using the SPI interface.
}tsChipStatusByte;

/** \brief Header Byte*/
typedef union{
	tsChipStatusByte 	Field;
	uint8				Byte;
}tuChipStatusByte;

/*
When the header byte, data byte, or command strobe is sent on the SPI interface, the chip status byte is
sent by the CC110L on the SO pin. The status byte contains key status signals, useful for the MCU. The
first bit, s7, is the CHIP_RDYn signal and this signal must go low before the first positive edge of SCLK.
The CHIP_RDYn signal indicates that the crystal is running.
Bits 6, 5, and 4 comprise the STATE value. This value reflects the state of the chip. The XOSC and power
to the digital core are on in the IDLE state, but all other modules are in power down. The frequency and
channel configuration should only be updated when the chip is in this state. The RX state will be active
when the chip is in receive mode. Likewise, TX is active when the chip is transmitting.
The last four bits (3:0) in the status byte contains FIFO_BYTES_AVAILABLE. For read operations (the
R/W bit in the header byte is set to 1), the FIFO_BYTES_AVAILABLE field contains the number of bytes
available for reading from the RX FIFO. For write operations (the R/W bit in the header byte is set to 0),
the FIFO_BYTES_AVAILABLE field contains the number of bytes that can be written to the TX FIFO.
When FIFO_BYTES_AVAILABLE=15, 15 or more bytes are available/free.*/
/** \brief Status Byte Summary */
typedef struct{						//																		Reset
	unsigned char FIFO_BYTES_AVAILABLE: 4;	/*	The number of bytes available 								0xX			R
	 	 	 	 	 	 	 	 	 	in the RX FIFO or free bytes in the TX FIFO 	 	 	 */
	unsigned char STATE		: 3;	/*	Indicates the current main state machine mode									R
	 	 	 	 	 	 	 	 	 	Value 		State 			Description
	 	 	 	 	 	 	 	 	 	000 		IDLE			IDLE state (Also reported for some transitional
	 	 	 	 	 	 	 	 	 								states instead of SETTLING or CALIBRATE) */
	unsigned char CHIP_RDYn	: 1;	/*	Stays high until power and crystal have stabilized.								R
										Should always be low when using the SPI interface.	*/
}tsSTATUS;
/* 		General Purpose and Test Output Control Pins
		The three digital output pins GDO0, GDO1, and GDO2 are general control pins configured with
		IOCFG0.GDO0_CFG, IOCFG1.GDO1_CFG, and IOCFG2.GDO2_CFG respectively. Table 5-18 shows
		the different signals that can be monitored on the GDO pins. These signals can be used as inputs to the
		MCU.
		GDO1 is the same pin as the SO pin on the SPI interface, thus the output programmed on this pin will
		only be valid when CSn is high. The default value for GDO1 is 3-stated which is useful when the SPI
		interface is shared with other devices.
		The default value for GDO0 is a 135-141 kHz clock output (XOSC frequency divided by 192). Since the
		XOSC is turned on at power-on-reset, this can be used to clock the MCU in systems with only one crystal.
		When the MCU is up and running, it can change the clock frequency by writing to IOCFG0.GDO0_CFG.
		If the IOCFGx.GDOx_CFG setting is less than 0x20 and IOCFGx_GDOx_INV is 0 (1), the GDO0 and
		GDO2 pins will be hardwired to 0 (1), and the GDO1 pin will be hardwired to 1 (0) in the SLEEP state.
		These signals will be hardwired until the CHIP_RDYn signal goes low.
		If the IOCFGx.GDOx_CFG setting is 0x20 or higher, the GDO pins will work as programmed also in
		SLEEP state. As an example, GDO1 is high impedance in all states if IOCFG1.GDO1_CFG=0x2E.
*/
/** \brief 0x00: IOCFG2 - GDO2 Output Pin Configuration */
typedef struct{						//																		Reset
	unsigned char GDO2_CFG	: 6;	//	Default is CHP_RDYn (see Table 5-18).								41(0x29)	R/W
	unsigned char GDO2_INV	: 1;	//	Invert output, that is, select active low (1) / high (0)			0			R/W
	unsigned char 			: 1;	//	Not used																		R0
}tsIOCFG2;

/** \brief 0x01: IOCFG1 - GDO1 Output Pin Configuration */
typedef struct{						//																		Reset
	unsigned char GDO1_CFG	: 6;	//	Default is 3-state (see Table 5-18).								46(0x2E)	R/W
	unsigned char GDO1_INV	: 1;	//	Invert output, that is, select active low (1) / high (0)			0			R/W
	unsigned char GDO_DS	: 1;	//	Set high (1) or low (0) output drive strength on the GDO pins.		0			R/W
}tsIOCFG1;

/** \brief 0x02: IOCFG0 - GDO1 Output Pin Configuration */
typedef struct{						//																		Reset
	unsigned char GDO0_CFG	: 6;	/*	Default is CLK_XOSC/192 (see Table 5-18).							63(0x3F)	R/W
										It is recommended to disable the clock output in
										initialization, in order to optimize RF performance. */
	unsigned char GDO0_INV	: 1;	//	Invert output, that is, select active low (1) / high (0)			0			R/W
	unsigned char 			: 1;	//	Use setting from SmartRF Studio										0			R/W
}tsIOCFG0;

/*		The CC110L contains two 64-byte FIFOs, one for received data and one for data to be transmitted. The
		SPI interface is used to read from the RX FIFO and write to the TX FIFO. Section 5.10 contains details on
		the SPI FIFO access. The FIFO controller will detect overflow in the RX FIFO and underflow in the TX FIFO.
*/
/** \brief 0x03: FIFOTHR - RX FIFO and TX FIFO Thresholds */
typedef struct{						//																		Reset
	unsigned char FIFO_THR		: 4;	/*	Set the threshold for the RX FIFO and TX FIFO. The				7 (0111)	R/W
											threshold is exceeded when the number of bytes in the
											FIFO is equal to or higher than the threshold value.
											Setting 		Bytes in RX FIFO 		Bytes in TX FIFO
											0  (0000) 		4 						61
											1  (0001) 		8 						57
											2  (0010) 		12 						53
											3  (0011) 		16 						49
											4  (0100) 		20 						45
											5  (0101) 		24 						41
											6  (0110) 		28 						37
											7  (0111) 		32 						33
											8  (1000) 		36 						29
											9  (1001) 		40 						25
											10 (1010) 		44 						21
											11 (1011) 		48 						17
											12 (1100) 		52 						13
											13 (1101) 		56 						9
											14 (1110) 		60 						5
											15 (1111) 		64 						1	*/
	unsigned char CLOSE_IN_RX	: 2;	/*	For more details, see DN010 SWRA147								0 (00) 		R/W
											Setting 	RX Attenuation, Typical Values
											0 (00) 		0 dB
											1 (01) 		6 dB
											2 (10) 		12 dB
											3 (11) 		18 dB	*/
	unsigned char ADC_RETENTION	: 1;	/*	0: TEST1 = 0x31 and TEST2= 0x88 when waking up from				0 			R/W
											SLEEP
											1: TEST1 = 0x35 and TEST2 = 0x81 when waking up
											from SLEEP

											Note that the changes in the TEST registers due to the
											ADC_RETENTION bit setting are only seen INTERNALLY
											in the analog part. The values read from the TEST
											registers when waking up from SLEEP mode will always
											be the reset value.

											The ADC_RETENTION bit should be set to 1before going
											into SLEEP mode if settings with an RX filter bandwidth
											below 325 kHz are wanted at time of wake-up.*/
	unsigned char Reserve		: 1;	//	Use setting from SmartRF Studio									0			R/W
}tsFIFOTHR;

/*
 *
 * */
/** \brief 0x07: PKTCTRL1 - Packet Automation Control */
typedef struct{						//																		Reset
	unsigned char ADR_CHK		: 2;	/*	Controls address check configuration of received packages.		0 (00)		R/W
											Setting 				Address check configuration
											0 (00) 					No address check
											1 (01) 					Address check, no broadcast
											2 (10) 					Address check and 0 (0x00) broadcast
											3 (11)					Address check and 0	(0x00) and 255 (0xFF) broadcast */
	unsigned char APPEND_STATUS	: 1;	/*	When enabled, two status bytes will be appended to the			1 			R/W
											payload of the packet. The status bytes contain the RSSI
											value, as well as CRC OK.*/
	unsigned char CRC_AUTOFLUSH	: 1;	/*	Enable automatic flush of RX FIFO when CRC is not OK.			0 			R/W
											This requires that only one packet is in the RX FIFO and
											that packet length is limited to the RX FIFO size.	*/
	unsigned char NotUsed		: 1;	//	Not used														0			R0
	unsigned char Reserve		: 3;	//	Use setting from SmartRF Studio									0 (000)		R/W
}tsPKTCTRL1;

/** \brief 0x08: PKTCTRL0 - Packet Automation Control */
typedef struct{						//																		Reset
	unsigned char LENGTH_CONFIG	: 2;	/* 	Configure the packet length										1 (01)		R/W
											Setting 		Packet length configuration
											0 (00) 			Fixed packet length mode. Length
															configured in PKTLEN register
											1 (01) 			Variable packet length mode. Packet length
															configured by the first byte after sync word
											2 (10) 			Infinite packet length mode
											3 (11) 			Reserved */
	unsigned char CRC_EN		: 1;	/*	1: CRC calculation enabled										1 			R/W
											0: CRC calculation disabled */
	unsigned char 				: 1;	//	Not used														0			R0
	unsigned char PKT_FORMAT	: 2;	/*	Format of RX data												0 (00) 		R/W
											Setting 	Packet format
											0 (00) 		Normal mode, use FIFOs for RX and TX
											1 (01)		Synchronous serial mode. Data in on
											 			GDO0 and data out on either of the GDOx	pins
											2 (10) 		Random TX mode; sends random data
														using PN9 generator. Used for test. Works
														as normal mode, setting 0 (00), in RX
											3 (11) 		Asynchronous serial mode. Data in on
														GDO0 and data out on either of the GDOx	pins	*/


	unsigned char Reserve		: 1;	//	Use setting from SmartRF Studio									1			R/W
	unsigned char 				: 1;	//	Not used																	R0
}tsPKTCTRL0;

/** \brief 0x0B: FSCTRL1 - Frequency Synthesizer Control */
typedef struct{						//																		Reset
	unsigned char FREQ_IF		: 5;	/*  The desired IF frequency to employ in RX. 						15 (01111)	R/W
											Subtracted from FS base frequency in RX and controls
											the digital complex mixer in the demodulator. (See datasheet for equation).
											The default value gives an IF frequency of 381kHz, assuming a 26.0 MHz crystal.*/
	unsigned char Reserve		: 1;	//	Use setting from SmartRF Studio									1			R/W
	unsigned char 				: 2;	//	Not used																	R0
}tsFSCTRL1;

/** \brief 0x0C: FSCTRL0 - Frequency Synthesizer Control */
typedef struct{						//																		Reset
	unsigned char FREQOFF		: 8;	/*  Frequency offset added to the base frequency before being 		0 (0x00) 	R/W
											used by the frequency synthesizer. (2s-complement).
											Resolution is FXTAL/214 (1.59kHz-1.65kHz); range is ±202
											kHz to ±210 kHz, dependent of XTAL frequency.*/
}tsFSCTRL0;

/** \brief 0x0D: FREQ2 - Frequency Control Word, High Byte */
typedef struct{						//																		Reset
	unsigned char FREQ21_16		: 6;	/*  FREQ[23:0] is the base frequency for 							30(0x1E)	R/W
											the frequency synthesiser in increments of fXOSC/2^16. (See datasheet for equation). */
	unsigned char FREQ23_22		: 2;	/*	FREQ[23:22] is always 0 (the FREQ2 register is 					0 (00)		R
											less than 36 with 26 - 27 MHz crystal)	*/
}tsFREQ2;

/** \brief 0x0E: FREQ1 - Frequency Control Word, High Byte */
typedef struct{						//																		Reset
	unsigned char FREQ15_8		: 8;	/*  FREQ[23:0] is the base frequency for 						   196(0xC4)	R/W
											the frequency synthesiser in increments of fXOSC/2^16. (See datasheet for equation). */
}tsFREQ1;

/** \brief 0x0F: FREQ0 - Frequency Control Word, High Byte */
typedef struct{						//																		Reset
	unsigned char FREQ7_0		: 8;	/*  FREQ[23:0] is the base frequency for 						   236(0xEC)	R/W
											the frequency synthesiser in increments of fXOSC/2^16. (See datasheet for equation). */
}tsFREQ0;



/** \brief 0x10: MDMCFG4 - Modem Configuration */
typedef struct{						//																		Reset
	unsigned char DRATE_E		: 4;	// 	The exponent of the user specified symbol rate 					12(1100)	R/W
	unsigned char CHANBW_M		: 2;	/*  Sets the decimation ratio for the delta-sigma ADC input stream 	0 (00) 		R/W
											and thus the channel bandwidth. (See datasheet for equation).
											The default values give 203 kHz channel filter bandwidth, assuming a 26.0 MHz crystal. */
	unsigned char CHANBW_E		: 2;	//																	2 (10) 		R/W
}tsMDMCFG4;

/** \brief 0x11: MDMCFG3 - Modem Configuration */
typedef struct{						//																		Reset
	unsigned char DRATE_M		: 8;	// 	The exponent of the user specified symbol rate 					12(1100)	R/W
}tsMDMCFG3;

/** \brief 0x12: MDMCFG2 - Modem Configuration */
typedef struct{						//																		Reset
	unsigned char SYNC_MODE		: 3;	/*	Combined sync-word qualifier mode.								2 (010) 	R/W
											The values 0 and 4 disables preamble and sync word
											detection
											The values 1, 2, 5, and 6 enables 16-bit sync word
											detection. Only 15 of 16 bits need to match when using
											setting 1 or 5. The values 3 and 7 enables 32-bits sync word
											detection (only 30 of 32 bits need to match).
											Setting 	Sync-word qualifier mode
											0 (000) 	No preamble/sync
											1 (001) 	15/16 sync word bits detected
											2 (010) 	16/16 sync word bits detected
											3 (011) 	30/32 sync word bits detected
											4 (100) 	No preamble/sync, carrier-sense above threshold
											5 (101) 	15/16 + carrier-sense above threshold
											6 (110) 	16/16 + carrier-sense above threshold
											7 (111) 	30/32 + carrier-sense above threshold	*/
	unsigned char MANCHESTER_EN	: 1;	/*	Enables Manchester encoding/decoding.							0  			R/W
											0 = Disable	1 = Enable
											Manchester encoding cannot be used when using
											asynchronous serial mode or 4-FSK modulation	*/
	unsigned char MOD_FORMAT	: 3;	/*  The modulation format of the radio signal						0 (000)		R/W
											Setting 	Modulation format
											0 (000) 	2-FSK
											1 (001) 	GFSK
											2 (010) 	Reserved
											3 (011) 	OOK
											4 (100) 	4-FSK
											5 (101) 	Reserved
											6 (110) 	Reserved
											7 (111) 	Reserved
											4-FSK modulation cannot be used together with Manchester encoding */
	unsigned char DEM_DCFILT_OFF: 1;	/* 	Disable digital DC blocking filter before demodulator.			0  			R/W
											0 = Enable (better sensitivity)
											1 = Disable (current optimized). Only for data rates < 250 kBaud
											The recommended IF frequency changes when the DC
											blocking is disabled. Use SmartRF Studio to calculate
											correct register setting. */
}tsMDMCFG2;

/** \brief 0x13: MDMCFG1 - Modem Configuration */
typedef struct{						//																		Reset
	unsigned char CHANSPC_E		: 2;	// 	2 bit exponent of channel spacing			 					2 (10)		R/W
	unsigned char 				: 2;	//	Not used																	R0
	unsigned char NUM_PREAMBLE	: 3;	/*  Sets the minimum number of preamble bytes to be transmitted		2 (010) 	R/W
											Setting 	Number of preamble bytes
											0 (000) 	2
											1 (001) 	3
											2 (010) 	4
											3 (011) 	6
											4 (100) 	8
											5 (101) 	12
											6 (110) 	16
											7 (111) 	24 */
	unsigned char Reserve		: 1;	//	Use setting from SmartRF Studio									0			R/W
}tsMDMCFG1;

/** \brief 0x15: DEVIATN - Modem Deviation Setting */
typedef struct{						//																		Reset
	unsigned char DEVIATION_M	: 3;	/* 	RX			 													7 (111)		R/W
											2-FSK/GFSK/4-FSK		Specifies the expected frequency deviation
																	of incoming signal, must be approximately
																	right for demodulation to be performed
																	reliably and robustly.
											OOK						This setting has no effect.
											TX
											2-FSK/GFSK/4-FSK		Specifies the nominal frequency deviation
																	from the carrier for a '0' (-DEVIATN) and
																	'1' (+DEVIATN) in a mantissa- exponent
																	format, interpreted as a 4-bit value with
																	MSB implicit 1. The default values give ±47.607 kHz
																	deviation assuming 26.0 MHz crystal	frequency.
											OOK						This setting has no effect. */
	unsigned char 				: 1;	//	Not used																	R0
	unsigned char DEVIATION_E	: 3;	//	Deviation exponent.												4 (100) 	R/W
	unsigned char 				: 1;	//	Not used																	R0
}tsDEVIATN;

/** \brief 0x16: MCSM2 - Main Radio Control State Machine Configuration */
typedef struct{						//																		Reset
	unsigned char Reserve		: 4;	//	Use setting from SmartRF Studio									7 (0111)	R/W
	unsigned char RX_TIME_RSSI	: 1;	/*  Direct RX termination based on RSSI measurement (carrier		0			R/W
											sense). For OOK modulation, RX times out if there is no
											carrier sense in the first 8 symbol periods. */
	unsigned char 				: 3;	//	Not used																	R0
}tsMCSM2;

/** \brief 0x17: MCSM1 - Main Radio Control State Machine Configuration */
typedef struct{						//																		Reset
	unsigned char TXOFF_MODE	: 2;	/*  Select what should happen when a packet has been sent			0 (00)		R/W
											Setting 	Next state after finishing packet transmission
											0 (00) 		IDLE
											1 (01) 		FSTXON
											2 (10) 		Stay in TX (start sending preamble)
											3 (11) 		RX		*/
	unsigned char RXOFF_MODE	: 2;	/*  Select what should happen when a packet has been received.		0 (00)		R/W
											Setting 	Next state after finishing packet reception
											0 (00) 		IDLE
											1 (01) 		FSTXON
											2 (10) 		TX
											3 (11) 		Stay in RX		*/
	unsigned char CCA_MODE		: 2;	/*  Selects CCA_MODE; Reflected in CCA signal						3 (11)		R/W
											Setting 	Clear channel indication
											0 (00) 		Always
											1 (01) 		If RSSI below threshold
											2 (10) 		Unless currently receiving a packet
											3 (11) 		If RSSI below threshold unless currently
														receiving a packet		*/
	unsigned char 				: 2;	//	Not used																	R0
}tsMCSM1;

/** \brief 0x18: MCSM0 - Main Radio Control State Machine Configuration */
typedef struct{						//																		Reset
	unsigned char XOSC_FORCE_ON	: 1;	//	Force the XOSC to stay on in the SLEEP state.					0			R/W
	unsigned char Reserve		: 1;	//	Use setting from SmartRF Studio									0			R/W
	unsigned char PO_TIMEOUT	: 2;	/*	Programs the number of times the six-bit ripple counter must	1 (01)		R/W
											expire after the XOSC has settled before CHP_RDYn goes
											low.(1)
											If XOSC is on (stable) during power-down, PO_TIMEOUT
											shall be set so that the regulated digital supply voltage has
											time to stabilize before CHP_RDYn goes low
											(PO_TIMEOUT=2 recommended). Typical start-up time for the
											voltage regulator is 50 us.
											For robust operation it is recommended to use PO_TIMEOUT
											= 2 or 3 when XOSC is off during power-down.
											Setting 	Expire count 	Timeout after XOSC start
											0 (00) 		1 				Approximately 2.3 - 2.4 us
											1 (01) 		16 				Approximately 37 - 39 us
											2 (10) 		64 				Approximately 149 - 155 us
											3 (11) 		256 			Approximately 597 - 620 us
											Exact timeout depends on crystal frequency.	*/
	unsigned char FS_AUTOCAL	: 2;	/*  Automatically calibrate when going to RX or TX, 				0 (00)		R/W
											or back to IDLE
											Setting 	When to perform automatic calibration
											0 (00) 		Never (manually calibrate using SCAL strobe)
											1 (01) 		When going from IDLE to RX or TX (or FSTXON)
											2 (10) 		When going from RX or TX back to IDLE automatically
											3 (11) 		Every 4th time when going from RX or TX
														to IDLE automatically */
	unsigned char 				: 2;	//	Not used																	R0
}tsMCSM0;
/*	Note that the XOSC_STABLE signal will be asserted at the same time as the CHIP_RDYn signal; that is, the PO_TIMEOUT delays both
signals and does not insert a delay between the signals. */

/** \brief 0x19: FOCCFG - Frequency Offset Compensation Configuration */
typedef struct{						//																		Reset
	unsigned char FOC_LIMIT		: 2;	/*	The saturation point for the frequency offset compensation		2 (10)		R/W
											algorithm:
											Setting 	Saturation point (max compensated offset)
											0 (00) 		+/-0 (no frequency offset compensation)
											1 (01) 		+/-BW(CHAN)/8
											2 (10) 		+/-BW(CHAN)/4
											3 (11) 		+/-BW(CHAN)/2
											Frequency offset compensation is not supported for OOK.
											Always use FOC_LIMIT=0 with this modulation format.	*/
	unsigned char FOC_POST_K	: 1;	/*	The frequency compensation loop gain to be used after			1			R/W
											a sync word is detected.
											Setting 	Freq. compensation loop gain after sync word
											0 			Same as FOC_PRE_K
											1 			K/2		*/
	unsigned char FOC_PRE_K		: 2;	/*	The frequency compensation loop gain to be used before a		2 (10)		R/W
											sync word is detected.
											Setting 	Freq. compensation loop gain before sync word
											0 (00) 		 K
											1 (01) 		2K
											2 (10) 		3K
											3 (11) 		4K		*/
	unsigned char FOC_BS_CS_GATE: 1;	/*  If set, the demodulator freezes the frequency offset			1 			R/W
											compensation and clock recovery feedback loops until the CS
											signal goes high. */
	unsigned char 				: 2;	//	Not used																	R0
}tsFOCCFG;

/** \brief 0x1A: BSCFG - Bit Synchronization Configuration */
typedef struct{						//																		Reset
	unsigned char BS_LIMIT		: 2;	/*	The saturation point for the data rate offset					0 (00)		R/W
											compensation algorithm:
														Data rate offset saturation
											Setting		(max data rate difference)
											0 (00) 		±0 (No data rate offset compensation performed)
											1 (01) 		±3.125% data rate offset
											2 (10) 		±6.25%  data rate offset
											3 (11) 		±12.5%  data rate offset		*/
	unsigned char BS_POST_KP	: 1;	/*	The clock recovery feedback loop proportional gain				1			R/W
											to be used after a sync word is detected.
														Clock recovery loop proportional gain
											Setting		after sync word
											0 			Same as BS_PRE_KP
											1 			KP			*/
	unsigned char BS_POST_KI	: 1;	/*	The clock recovery feedback loop integral gain to be used		1			R/W
											after a sync word is detected.
														Clock recovery loop integral gain
											Setting		after sync word
											0 			Same as BS_PRE_KI
											1 			KI /2		*/
	unsigned char BS_PRE_KP		: 2;	/*  The clock recovery feedback loop proportional gain 				2 (10)		R/W
											to be used before a sync word is detected.
														Clock recovery loop proportional gain
											Setting		before sync word
											0 (00) 		KP
											1 (01) 		2KP
											2 (10) 		3KP
											3 (11) 		4KP			*/
	unsigned char BS_PRE_KI		: 2;	/*	The clock recovery feedback loop integral gain to be used		1 (01)		R/W
											before a sync word is detected (used to correct offsets
											in data	rate):
											 			Clock recovery loop integral gain
											Setting		before sync word
											0 (00) 		KI
											1 (01) 		2KI
											2 (10) 		3KI
											3 (11) 		4KI			*/
}tsBSCFG;

/** \brief 0x1B: AGCCTRL2 - AGC Control */
typedef struct{						//																		Reset
	unsigned char MAGN_TARGET	: 3;	/*	These bits set the target value for the averaged amplitude		3 (011)		R/W
											from the digital channel filter (1 LSB = 0 dB):
											Setting 	Target amplitude from channel filter
											0 (000) 	24 dB
											1 (001) 	27 dB
											2 (010) 	30 dB
											3 (011) 	33 dB
											4 (100) 	36 dB
											5 (101) 	38 dB
											6 (110) 	40 dB
											7 (111) 	42 dB										*/
	unsigned char MAX_LNA_GAIN	: 3;	/*  Sets the maximum allowable LNA + LNA 2 gain relative 			0 (000)		R/W
											to the maximum possible gain:
											Setting 	Maximum allowable LNA + LNA 2 gain
											0 (000) 	Maximum possible LNA + LNA 2 gain
											1 (001) 	Approximately 2.6 dB below maximum possible gain
											2 (010) 	Approximately 6.1 dB below maximum possible gain
											3 (011) 	Approximately 7.4 dB below maximum possible gain
											4 (100) 	Approximately 9.2 dB below maximum possible gain
											5 (101) 	Approximately 11.5 dB below maximum possible gain
											6 (110) 	Approximately 14.6 dB below maximum possible gain
											7 (111) 	Approximately 17.1 dB below maximum possible gain */
	unsigned char MAX_DVGA_GAIN	: 2;	/*	Reduces the maximum allowable DVGA gain.						0 (00)		R/W
											Setting 	Allowable DVGA settings
											0 (00) 		All gain settings can be used
											1 (01) 		The highest gain setting cannot be used
											2 (10) 		The 2 highest gain settings cannot be used
											3 (11) 		The 3 highest gain settings cannot be used	*/
}tsAGCCTRL2;

/** \brief 0x1C: AGCCTRL1 - AGC Control */
typedef struct{						//																		Reset
	unsigned char CARRIER_SENSE_ABS_THR: 4; /*	Sets the absolute RSSI threshold for asserting
											carrier sense. The 2-complement signed threshold is programmed
											in steps of 1 dB and is relative to the MAGN_TARGET setting.
														Carrier sense absolute threshold (Equal to
											Setting 	channel filter amplitude when AGC has not
														decreased gain)
											-8 (1000) 	Absolute carrier sense threshold disabled
											-7 (1001) 	7 dB below MAGN_TARGET setting
											Е 			Е
											-1 (1111) 	1 dB below MAGN_TARGET setting
											0 (0000) 	At MAGN_TARGET setting
											1 (0001) 	1 dB above MAGN_TARGET setting
											Е 			Е
											7 (0111) 	7 dB above MAGN_TARGET setting		*/
	unsigned char CARRIER_SENSE_REL_THR: 2;	/*	Sets the relative change threshold							0 (00)		R/W
												for asserting carrier sense
											Setting 	Carrier sense relative threshold
											0 (00) 		Relative carrier sense threshold disabled
											1 (01) 		6 dB increase in RSSI value
											2 (10) 		10 dB increase in RSSI value
											3 (11) 		14 dB increase in RSSI value	*/
	unsigned char AGC_LNA_PRIORITY: 1;	/*  Selects between two different strategies						1 			R/W
											for LNA and LNA 2 gain adjustment. When 1, the LNA gain
											is decreased first. When 0, the LNA 2 gain is decreased
											to minimum before decreasing LNA gain. */
	unsigned char 				: 1;	//	Not used																	R0
}tsAGCCTRL1;

/** \brief 0x1D: AGCCTRL0 - AGC Control */
typedef struct{						//																		Reset
	unsigned char FILTER_LENGTH	: 2;	/*	2-FSK and 4-FSK: Sets the averaging length for the amplitude	1 (01)		R/W
											from the channel filter. OOK: Sets the OOK decision boundary
											for OOK reception.
											Setting 	Channel filter samples 		OOK decision boundary
											0 (00) 		8 							4 dB
											1 (01) 		16 							8 dB
											2 (10) 		32 							12 dB
											3 (11) 		64 							16 dB		*/
	unsigned char AGC_FREEZE	: 2;	/*	Control when the AGC gain should be frozen.						0 (00)		R/W
											Setting 	Function
											0 (00) 		Normal operation. Always adjust gain when required.
											1 (01)		The gain setting is frozen when a sync word has
														been found.
											2 (10) 		Manually freeze the analogue gain setting
														and continue to adjust the digital gain.
											3 (11)		Manually freezes both the analogue and the
														digital gain setting. Used for manually overriding the gain. */
	unsigned char WAIT_TIME		: 2;	/*  Sets the number of channel filter samples from a gain			1 (01)		R/W
											adjustment has been made until the AGC algorithm starts
											accumulating new samples.
											Setting 	Channel filter samples
											0 (00) 		8
											1 (01) 		16
											2 (10) 		24
											3 (11) 		32			*/
	unsigned char HYST_LEVEL	: 2;	/*	Sets the level of hysteresis on the magnitude deviation			2 (10)		R/W
											(internal AGC signal that determine gain changes).
											Setting 	Description
											0 (00) 		No hysteresis, small symmetric dead zone, high gain
											1 (01)		Low hysteresis, small asymmetric dead zone, medium gain
											2 (10) 		Medium hysteresis, medium asymmetric dead zone, medium gain
											3 (11) 		Large hysteresis, large asymmetric dead zone, low gain */
}tsAGCCTRL0;

/** \brief 0x20: RESERVED */
typedef struct{						//																		Reset
	unsigned char reserve		: 2;	//	Use setting from SmartRF Studio									0 (00)		R/W
	unsigned char 				: 1;	//	Not used																	R0
	unsigned char Reserve		: 5;	//	Use setting from SmartRF Studio									0 (11111)	R/W
}tsRESERVED;

/** \brief 0x21: FREND1 - FrontEnd RX Configuration */
typedef struct{						//																		Reset
	unsigned char MIX_CURRENT	: 2;	//	Adjusts current in mixer										2 (10)		R/W
	unsigned char LODIV_BUF_CURRENT_RX: 2;	//	Adjusts current in RX LO buffer (LO input to mixer)			1 (01)		R/W
	unsigned char LNA2MIX_CURRENT: 2;	//	Adjusts front-end PTAT outputs									1 (01)		R/W
	unsigned char LNA_CURRENT	: 2;	//	Adjusts front-end LNA PTAT current output						1 (01)		R/W
}tsFREND1;

/** \brief 0x22: FREND0 - FrontEnd TX Configuration */
typedef struct{						//																		Reset
	unsigned char PA_POWER		: 3;	/*	Selects PA power setting. This value is an index to the			0 (000)		R/W
											PATABLE, which can be programmed with up to 2 different PA settings.
											When using OOK, PA_POWER should be 001, and for all other modulation
											formats it should be 000, see Section 5.11.	*/
	unsigned char 				: 1;	//	Not used																	R0
	unsigned char LODIV_BUF_CURRENT_TX: 2;	/*	Adjusts current TX LO buffer (input to PA). 				1 (01)		R/W
											The value to use in this field is given by the SmartRF Studio software. */
	unsigned char 				: 2;	//	Not used																	R0
}tsFREND0;

/** \brief 0x23: FSCAL3 - Frequency Synthesizer Calibration */
typedef struct{						//																		Reset
	unsigned char FSCAL3_3_0	: 4;	/*	Frequency synthesizer calibration result register. 				9 (1001)	R/W
											Digital bit vector defining the charge pump output current,
											on an exponential scale:  I_OUT = I0Ј2^FSCAL3[3:0]/4
											See Section 5.27.2 for more details.	 */
	unsigned char CHP_CURR_CAL_EN: 2;	//	Disable charge pump calibration stage when 0.					2 (10) 		R/W
	unsigned char FSCAL3_7_6	: 2;	/*	Frequency synthesizer calibration configuration.				2 (10) 		R/W
											The value to write in this field before calibration
											is given by the SmartRF Studio software. */
}tsFSCAL3;

/** \brief 0x24: FSCAL2 - Frequency Synthesizer Calibration */
typedef struct{						//																		Reset
	unsigned char Fscal2		: 5;	/*	Frequency synthesizer calibration result register. 				10 (01010)	R/W
											VCO current calibration result and override value.
											See Section 5.27.2 for more	details. */
	unsigned char VCO_CORE_H_EN	: 1;	//	Choose high (1) / low (0) VCO									0 			R/W
	unsigned char 				: 2;	//	Not used																	R0
}tsFSCAL2;

/** \brief 0x25: FSCAL1 - Frequency Synthesizer Calibration */
typedef struct{						//																		Reset
	unsigned char Fscal1		: 6;	/*	Frequency synthesizer calibration result register. 				32(0x20)	R/W
											Capacitor array setting for VCO coarse tuning.
											See Section 5.27.2 for more details. */
	unsigned char 				: 2;	//	Not used																	R0
}tsFSCAL1;

/** \brief 0x26: FSCAL0 - Frequency Synthesizer Calibration */
typedef struct{						//																		Reset
	unsigned char Fscal0		: 7;	/*	Frequency synthesizer calibration control. 						13(0x0D)	R/W
											The value to use in this register is given
											by the SmartRF Studio software */
	unsigned char 				: 1;	//	Not used																	R0
}tsFSCAL0;

/** \brief 0x2E: TEST0 - Various Test Settings */
typedef struct{						//																		Reset
	unsigned char TEST0_0		: 1;	//	Use setting from SmartRF Studio SWRC176							1 			R/W
	unsigned char VCO_SEL_CAL_EN: 1;	//	Enable VCO selection calibration stage when 1					1 			R/W
	unsigned char TEST0_7_2		: 6;	//	Use setting from SmartRF Studio SWRC176							2 (0x02)	R/W
}tsTEST0;

/** \brief 0x33 (0xF3): CRC_REG - CRC OK */
typedef struct{						//																		Reset
	unsigned char 				: 7;	//	Reserved														X			R
	unsigned char CRC_OK		: 1;	/*	The last CRC comparison matched. 								X 			R
											Cleared when entering/restarting RX mode.	*/
}tsCRC_REG;

/** \brief CRC_REG - CRC OK*/
typedef union{
	tsCRC_REG 		Field;
	uint8			Byte;
}tuCRC_REG;

/** \brief —писок переходов состо€ни€ Main Radio Control State Machine State*/
typedef enum{
	MRCSMS_SLEEP			= 0,	//	SLEEP
	MRCSMS_IDLE				= 1,	//	IDLE
	MRCSMS_XOFF				= 2,	//	XOFF
	MRCSMS_VCOON_MC			= 3,	//	MANCAL
	MRCSMS_REGON_MC			= 4,	//	MANCAL
	MRCSMS_MANCAL			= 5,	//	MANCAL
	MRCSMS_VCOON			= 6,	//	FS_WAKEUP
	MRCSMS_REGON			= 7,	//	FS_WAKEUP
	MRCSMS_STARTCAL			= 8,	//	CALIBRATE
	MRCSMS_BWBOOST			= 9,	//	SETTLING
	MRCSMS_FS_LOCK			= 10,	//	SETTLING
	MRCSMS_IFADCON			= 11,	//	SETTLING
	MRCSMS_ENDCAL			= 12,	//	CALIBRATE
	MRCSMS_RX				= 13,	//	RX
	MRCSMS_RX_END			= 14,	//	RX
	MRCSMS_RX_RST			= 15,	//	RX
	MRCSMS_TXRX_SWITCH		= 16,	//	TXRX_SETTLING
	MRCSMS_RXFIFO_OVERFLOW	= 17,	//	RXFIFO_OVERFLOW
	MRCSMS_FSTXON			= 18,	//	FSTXON
	MRCSMS_TX				= 19,	//	TX
	MRCSMS_TX_END			= 20,	//	TX
	MRCSMS_RXTX_SWITCH		= 21,	//	RXTX_SETTLING
	MRCSMS_TXFIFO_UNDERFLOW	= 22,	//	TXFIFO_UNDERFLOW
}teMainRadioControlStateMachineState;

/** \brief 0x35 (0xF5): MARCSTATE - Main Radio Control State Machine State */
typedef struct{						//																		Reset
	unsigned char MARC_STATE	: 5;	/*	Main Radio Control FSM State									X			R
											(teMainRadioControlStateMachineState)
											Value 		State name 			State (see Figure 5-11)
											0 (0x00) 	SLEEP 				SLEEP
											1 (0x01) 	IDLE 				IDLE
											2 (0x02) 	XOFF 				XOFF
											3 (0x03) 	VCOON_MC 			MANCAL
											4 (0x04) 	REGON_MC 			MANCAL
											5 (0x05) 	MANCAL 				MANCAL
											6 (0x06) 	VCOON 				FS_WAKEUP
											7 (0x07) 	REGON 				FS_WAKEUP
											8 (0x08) 	STARTCAL 			CALIBRATE
											9 (0x09) 	BWBOOST 			SETTLING
											10 (0x0A) 	FS_LOCK 			SETTLING
											11 (0x0B) 	IFADCON 			SETTLING
											12 (0x0C) 	ENDCAL 				CALIBRATE
											13 (0x0D) 	RX 					RX
											14 (0x0E) 	RX_END 				RX
											15 (0x0F) 	RX_RST 				RX
											16 (0x10) 	TXRX_SWITCH 		TXRX_SETTLING
											17 (0x11) 	RXFIFO_OVERFLOW 	RXFIFO_OVERFLOW
											18 (0x12) 	FSTXON 				FSTXON
											19 (0x13) 	TX 					TX
											20 (0x14) 	TX_END 				TX
											21 (0x15) 	RXTX_SWITCH 		RXTX_SETTLING
											22 (0x16) 	TXFIFO_UNDERFLOW 	TXFIFO_UNDERFLOW
											Note: 	it is not possible to read back the SLEEP or XOFF state
													numbers because setting CSn low will make the chip enter the IDLE
													mode from the SLEEP or XOFF states.	*/
	unsigned char 				: 3;	//	Not used																	R0

}tsMARCSTATE;

/** \brief MARCSTATE - Main Radio Control State Machine State */
typedef union{
	tsMARCSTATE		Field;
	uint8			Byte;
}tuMARCSTATE;

/** \brief 0x38 (0xF8): PKTSTATUS - Current GDOx Status and Packet Status */
typedef struct{						//																		Reset
	unsigned char GDO0			: 1;	/*	Current GDO0 value.												X			R
											Note: the reading gives the non-inverted value irrespective
											of what IOCFG0.GDO0_INV is programmed to. It is not
											recommended to check for PLL lock by reading
											PKTSTATUS[0] with GDO0_CFG=0x0A.*/
	unsigned char 				: 1;	//	Not used																	R
	unsigned char GDO2			: 1;	/*	Current GDO2 value.												X			R
											Note: the reading gives the non-inverted value irrespective
											of what IOCFG2.GDO2_INV is programmed to. It is not
											recommended to check for PLL lock by reading
											PKTSTATUS[2] with GDO2_CFG=0x0A.*/
	unsigned char SFD			: 1;	/*	Start of Frame Delimiter. 										X			R
											This bit is asserted when sync word has been received and
											deasserted at the end of the packet. It will also de-assert
											when a packet is discarded due to address or maximum
											length filtering or the radio enters RXFIFO_OVERFLOW state. */
	unsigned char CCA			: 1;	//	Channel is clear												X			R
	unsigned char 				: 1;	//	Not used																	R
	unsigned char CS			: 1;	//	Carrier sense. Cleared when entering IDLE mode.					X			R
	unsigned char CRC_OK		: 1;	/*	The last CRC comparison matched. 								X			R
											Cleared when entering/restarting RX mode.		*/
}tsPKTSTATUS;

/** \brief PKTSTATUS - Current GDOx Status and Packet Status */
typedef union{
	tsPKTSTATUS		Field;
	uint8			Byte;
}tuPKTSTATUS;

/** \brief 0x3A (0xFA): TXBYTES - Underflow and Number of Bytes */
typedef struct{						//																		Reset
	unsigned char NUM_TXBYTES	: 7;	//	Number of bytes in TX FIFO										0x00		R
	unsigned char TXFIFO_UNDERFLOW: 1;	//																	X			R
}tsTXBYTES;

/** \brief TXBYTES - Underflow and Number of Bytes */
typedef union{
	tsTXBYTES		Field;
	uint8			Byte;
}tuTXBYTES;

/** \brief 0x3B (0xFB): RXBYTES - Overflow and Number of Bytes */
typedef struct{						//																		Reset
	unsigned char NUM_RXBYTES	: 7;	//	Number of bytes in RX FIFO										0x00		R
	unsigned char RXFIFO_OVERFLOW: 1;	//																	X			R
}tsRXBYTES;

/** \brief RXBYTES - Overflow and Number of Bytes */
typedef union{
	tsRXBYTES		Field;
	uint8			Byte;
}tuRXBYTES;

/** \brief Configuration Registers - Address space 0x00 - 0x2E */
typedef struct{						//																		Reset
	tsIOCFG2	s8IOCFG2;			//																		0x29
	tsIOCFG1	s8IOCFG1;			//																		0x2E
	tsIOCFG0	s8IOCFG0;			//																		0x3F
	tsFIFOTHR	s8FIFOTHR;			//																		0x07
	uint8		u8SYNC1;			//	0x04: SYNC1 - Sync Word, High Byte: 8 MSB of 16-bit sync word		0xD3
	uint8		u8SYNC0;			//	0x05: SYNC0 - Sync Word, Low Byte: 8 LSB of 16-bit sync word		0x91
	uint8		u8PKTLEN;			/*	Indicates the packet length when fixed packet length mode is		0xFF
										enabled. If variable packet length mode is used, this value
										indicates the maximum packet length allowed. This value must
										be different from 0.	*/
	tsPKTCTRL1	s8PKTCTRL1;			//																		0x04
	tsPKTCTRL0	s8PKTCTRL0;			//																		0x45
	uint8		u8ADDR;				/*	Address used for packet filtration. Optional broadcast 				0x00
										addresses are 0 (0x00) and 255 (0xFF). */
	uint8		u8CHANNR;			/*	The 8-bit unsigned channel number, which is multiplied by			0x00
										the channel spacing setting and added to the base frequency.	*/
	tsFSCTRL1	s8FSCTRL1;			//																		0x0F
	tsFSCTRL0	s8FSCTRL0;			/*	Frequency offset added to the base frequency before being			0x00
										used by the frequency synthesizer. (2s-complement). Resolution
										is FXTAL/214 (1.59kHz-1.65kHz); range is ±202kHz to ±210 kHz,
										dependent of XTAL frequency.	*/
	tsFREQ2		s8FREQ2;			//																		0x1E
	tsFREQ1		s8FREQ1;			//	0x0E: FREQ1 - FREQ[15:8]											0xC4
	tsFREQ0		s8FREQ0;			//	0x0F: FREQ0 - FREQ[7:0]												0xEC
	tsMDMCFG4	s8MDMCFG4;			//																		0x8C
	tsMDMCFG3	s8MDMCFG3;			/*	DRATE_M[7:0]	The mantissa of the user specified symbol rate. 	0x22
										The symbol rate is configured using an unsigned, floating-point
										number with 9-bit mantissa and 4-bit exponent. The 9th bit is
										a hidden "1". The resulting data rate is: (See datasheet for equation).
										The default values give a data rate of 115.051 kBaud (closest setting to 115.2 kBaud),
										assuming a 26.0 MHz crystal.		*/
	tsMDMCFG2	s8MDMCFG2;			//																		0x02
	tsMDMCFG1	s8MDMCFG1;			//																		0x22
	uint8		u8MDMCFG0;			/*	CHANSPC_M[7:0]	8-bit mantissa of channel spacing. 					0xF8
										The channel spacing is multiplied by the channel number CHAN
										and added to the base frequency. It is unsigned and has the format:
										(See datasheet for equation). The default values give 199.951 kHz
										channel spacing (the closest setting to 200 kHz), assuming 26.0 MHz
										crystal frequency.	*/
	tsDEVIATN	s8DEVIATN;			//																		0x47
	tsMCSM2		s8MCSM2;			//																		0x07
	tsMCSM1		s8MCSM1;			//																		0x30
	tsMCSM0		s8MCSM0;			//																		0x00
	tsFOCCFG	s8FOCCFG;			//																		0x36
	tsBSCFG		s8BSCFG;			//																		0x6C
	tsAGCCTRL2	s8AGCCTRL2;			//																		0x03
	tsAGCCTRL1	s8AGCCTRL1;			//																		0x40
	tsAGCCTRL0	s8AGCCTRL0;			//																		0x91
	uint8		u8NotUsed0;			//	Not Used															0xXX
	uint8		u8NotUsed1;			//	Not Used															0xXX
	tsRESERVED	s8RESERVED;			//	Reserved															0xF8
	tsFREND1	s8FREND1;			//																		0x56
	tsFREND0	s8FREND0;			//																		0x10
	tsFSCAL3	s8FSCAL3;			//																		0xA9
	tsFSCAL2	s8FSCAL2;			//																		0x0A
	tsFSCAL1	s8FSCAL1;			//																		0x20
	tsFSCAL0	s8FSCAL0;			//																		0x0D
	uint8		u8NotUsed2;			//	Not Used															0xXX
	uint8		u8NotUsed3;			//	Not Used															0xXX
	uint8		u8RESERVED2;		//	0x29:	Reserved													0x59
	uint8		u8RESERVED1;		//	0x2A:	Reserved													0x7F
	uint8		u8RESERVED0;		//	0x2B:	Reserved													0x3F
	uint8		u8TEST2;			/*	0x2C: TEST2 - Various Test Settings									0x88
	 	 	 	 	 	 	 	 	 	Use setting from SmartRF Studio
										This register will be forced to 0x88 or 0x81 when it wakes up from
										SLEEP mode, depending on the configuration of
										FIFOTHR.ADC_RETENTION.
										The value read from this register when waking up from SLEEP
										always is the reset value (0x88) regardless of the
										ADC_RETENTION setting. The inverting of some of the bits due to
										the ADC_RETENTION setting is only seen INTERNALLY in the analog part.	*/
	uint8		u8TEST1;			/*	0x2D: TEST1 - Various Test Settings									0x31
										Use setting from SmartRF Studio. This register will be forced
										to 0x31 or 0x35 when it wakes up from SLEEP mode, depending on
										the configuration of FIFOTHR.ADC_RETENTION. Note that the value
										read from this register when waking up from SLEEP always is the
										reset value (0x31) regardless of the ADC_RETENTION setting.
										The inverting of some of the bits due to the ADC_RETENTION setting
										is only seen INTERNALLY in the analog part.	*/
	tsTEST0		s8TEST0;			//																		0x0B
	uint8		u8NotUsed4;			//	Not Used															0xXX
}tsCNFGRGSTR;

/** \brief Configuration Registers - Address space 0x00 - 0x2E 				BiAccess*/
typedef union{
	tsCNFGRGSTR		Field;
	uint8			Byte[0x30];
}tuCNFGRGSTR;

/** \brief —писок переходов и состо€ний CC110L и его инициализаци€ */
typedef enum{
	CC110l_Reset,
	CC110l_Start,
	CC110l_Initialise,
	CC110l_Calibrate,
	CC110l_IDLE,
	CC110l_RX_Wait,
	CC110l_RX_Receive,
	CC110l_TX_Transmit
}teCC110LState;

/** \brief Ѕазова€ структура переменных дл€ работы с радио приЄмо передатчиком*/
typedef struct{
	uint8				teState;			//	teCC110LState
	tsNwkState			sNwkState;
	uint32				u32WatchDogTime;
	bool_t				bCHIP_RDYn;
	uint8				u8MainStateMachine;	//	teMainStateMachine
	uint8				u8AvailableRxFifo;
	uint8				u8FreeTxFifo;
	uint8				u8PARTNUMchipID;
	uint8				u8VERSIONchipID;
	uint8				u8FREQEST;
	uint8				u8RSSI;
	tuMARCSTATE			uMARCSTATE;
	tuPKTSTATUS			uPKTSTATUS;
	tuTXBYTES			uTXBYTES;
	tuRXBYTES			uRXBYTES;
	tsuUint16			su16PA_TABLE;
}tsCC110L;
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
