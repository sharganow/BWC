/**
 * 			 ќЌ‘»√”–ј÷»я ЁЋ≈ “–ќ ќћѕЋ≈ “ј  ≈ƒ–.485
 * */
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
  #define MT240128A
//#define TG240128A_09C
//#define WG240128B_TFH

  #define Device_ICP
//#define Device_BWC

//#define CHANGEWORKTEMPERATURE

#define WIRELESS_NUMBER		15867

#define WIRELESS_CHANNEL	9

//#define CONFIG_RF_434_MHz_2_4_kBaude
//#define CONFIG_RF_434_MHz_250_kBaude
#define CONFIG_RF_434_MHz_38k3835_Baude

#ifdef	CONFIG_RF_434_MHz_2_4_kBaude
#define IOCFG2_Value		0x00
#define IOCFG1_Value		0x06
#define IOCFG0_Value		0x09
#define FIFOTHR_Value		0x40
#define SYNC1_Value
#define SYNC0_Value
#define PKTLEN_Value		0xFF
#define PKTCTRL1_Value		0x0C
#define PKTCTRL0_Value		0x05
#define ADDR_Value
#define	CHANNR_Value		WIRELESS_CHANNEL
#define FSCTRL1_Value		0x06
#define FSCTRL0_Value		0x00
#define FREQ2_Value			0x10
#define FREQ1_Value			0xB1
#define FREQ0_Value			0x3B
#define MDMCFG4_Value		0xF6
#define MDMCFG3_Value		0x83
#define MDMCFG2_Value		0x13
#define MDMCFG1_Value		0x32
#define MDMCFG0_Value		0xF8
#define DEVIATN_Value		0x15
#define MCSM2_Value			0x07
#define MCSM1_Value			0x30
#define MCSM0_Value			0x18
#define FOCCFG_Value		0x16
#define BSCFG_Value			0x6C
#define AGCCTRL2_Value		0x03
#define AGCCTRL1_Value		0x40
#define AGCCTRL0_Value		0x91
#define NOTUSED4_Value
#define NOTUSED3_Value
#define RESERVED20_Value	0xFB
#define FREND1_Value		0x56
#define FREND0_Value		0x10
#define FSCAL3_Value		0xE9
#define FSCAL2_Value		0x2A
#define FSCAL1_Value		0x00
#define FSCAL0_Value		0x1F
#define NOTUSED2_Value
#define NOTUSED1_Value
#define RESERVED29_Value
#define RESERVED2A_Value
#define RESERVED2B_Value
#define TEST2_Value			0x81
#define TEST1_Value			0x35
#define TEST0_Value			0x09
#define NOTUSED0_Value

#define TX_POWER			0x84	/* 5.1 dBm */
#endif /* CONFIG_RF_434_MHz_2_4_kBaude */

#ifdef	CONFIG_RF_434_MHz_250_kBaude
#define IOCFG2_Value		0x00
#define IOCFG1_Value		0x06
#define IOCFG0_Value		0x09
#define FIFOTHR_Value		0x00
#define SYNC1_Value
#define SYNC0_Value
#define PKTLEN_Value		0xFF
#define PKTCTRL1_Value		0x0D
#define PKTCTRL0_Value		0x05
#define ADDR_Value
#define	CHANNR_Value		WIRELESS_CHANNEL
#define FSCTRL1_Value		0x0C
#define FSCTRL0_Value		0x00
#define FREQ2_Value			0x10
#define FREQ1_Value			0xB1
#define FREQ0_Value			0x3B
#define MDMCFG4_Value		0x2D
#define MDMCFG3_Value		0x3B
#define MDMCFG2_Value		0x13
#define MDMCFG1_Value		0x32
#define MDMCFG0_Value		0xF8
#define DEVIATN_Value		0x62
#define MCSM2_Value			0x07
#define MCSM1_Value			0x30
#define MCSM0_Value			0x18
#define FOCCFG_Value		0x1D
#define BSCFG_Value			0x1C
#define AGCCTRL2_Value		0xC7
#define AGCCTRL1_Value		0x00
#define AGCCTRL0_Value		0xB0
#define NOTUSED4_Value
#define NOTUSED3_Value
#define RESERVED20_Value	0xFB
#define FREND1_Value		0xB6
#define FREND0_Value		0x10
#define FSCAL3_Value		0xEA
#define FSCAL2_Value		0x2A
#define FSCAL1_Value		0x00
#define FSCAL0_Value		0x1F
#define NOTUSED2_Value
#define NOTUSED1_Value
#define RESERVED29_Value	0x59
#define RESERVED2A_Value	0x7F
#define RESERVED2B_Value	0x3F
#define TEST2_Value			0x88
#define TEST1_Value			0x31
#define TEST0_Value			0x09
#define NOTUSED0_Value

#define TX_POWER_ICP		0x84			/* 8.5 dBm - пока наиболее приоритетно */
#define TX_POWER_BWC		0x84			/* 8.5 dBm - пока наиболее приоритетно */
#endif /* CONFIG_RF_434_MHz_250_kBaude */

#ifdef	CONFIG_RF_434_MHz_38k3835_Baude
#define IOCFG2_Value		0x00
#define IOCFG1_Value		0x06
#define IOCFG0_Value		0x09
#define FIFOTHR_Value		0x00
#define SYNC1_Value
#define SYNC0_Value
#define PKTLEN_Value		0xFF
#define PKTCTRL1_Value		0x0D
#define PKTCTRL0_Value		0x05
#define ADDR_Value
#define	CHANNR_Value		WIRELESS_CHANNEL
#define FSCTRL1_Value		0x0C
#define FSCTRL0_Value		0x00
#define FREQ2_Value			0x10
#define FREQ1_Value			0xB1
#define FREQ0_Value			0x3B
#define MDMCFG4_Value		0x2A
#define MDMCFG3_Value		0x83
#define MDMCFG2_Value		0x03
#define MDMCFG1_Value		0x32
#define MDMCFG0_Value		0xF8
#define DEVIATN_Value		0x54
#define MCSM2_Value			0x07
#define MCSM1_Value			0x30
#define MCSM0_Value			0x18
#define FOCCFG_Value		0x1D
#define BSCFG_Value			0x1C
#define AGCCTRL2_Value		0xC7
#define AGCCTRL1_Value		0x00
#define AGCCTRL0_Value		0xB0
#define NOTUSED4_Value
#define NOTUSED3_Value
#define RESERVED20_Value	0xFB
#define FREND1_Value		0xB6
#define FREND0_Value		0x10
#define FSCAL3_Value		0xE9
#define FSCAL2_Value		0x2A
#define FSCAL1_Value		0x00
#define FSCAL0_Value		0x1F
#define NOTUSED2_Value
#define NOTUSED1_Value
#define RESERVED29_Value	0x59
#define RESERVED2A_Value	0x7F
#define RESERVED2B_Value	0x3F
#define TEST2_Value			0x88
#define TEST1_Value			0x31
#define TEST0_Value			0x09
#define NOTUSED0_Value

#define TX_POWER_ICP		0xC0			/* 10 dBm - пока наиболее приоритетно */
#define TX_POWER_BWC		0xC0			/* 10 dBm - пока наиболее приоритетно */
#endif /* CONFIG_RF_434_MHz_38k3835_Baude */






//#define ITALIAN_ITa
#ifdef 	ITALIAN_ITa
#define LNG_ITa				1
#else
#define LNG_ITa				0
#endif /* 	ITALIAN_ITa */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
