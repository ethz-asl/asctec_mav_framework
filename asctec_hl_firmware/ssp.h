/*****************************************************************************
 *   ssp.h:  Header file for Philips LPC214x Family Microprocessors
 *
 *   Copyright(C) 2006, Philips Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2005.10.01  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/
#ifndef __SSP_H__
#define __SSP_H__

volatile unsigned int SSP_trans_cnt;

/* SPI read and write buffer size */
#define FIFOSIZE	8
	
/* SPI Status register */
#define SSPSR_TFE	1 << 0
#define SSPSR_TNF	1 << 1 
#define SSPSR_RNE	1 << 2
#define SSPSR_RFF	1 << 3 
#define SSPSR_BSY	1 << 4

/* SPI 1 CR0 register */
#define SSPCR0_DSS	1 << 0
#define SSPCR0_FRF	1 << 4
#define SSPCR0_CPOL	1 << 6
#define SSPCR0_CPHA	1 << 7
#define SSPCR0_SCR	1 << 8

/* SPI 1 CR1 register */
#define SSPCR1_LBM	1 << 0
#define SSPCR1_SSE	1 << 1
#define SSPCR1_MS	1 << 2
#define SSPCR1_SOD	1 << 3

/* SPI 1 Interrupt Mask Set/Clear register */
#define SSPIMSC_RORIM	1 << 0
#define SSPIMSC_RTIM	1 << 1
#define SSPIMSC_RXIM	1 << 2
#define SSPIMSC_TXIM	1 << 3

/* SPI 1 Interrupt Status register */
#define SSPRIS_RORRIS	1 << 0
#define SSPRIS_RTRIS	1 << 1
#define SSPRIS_RXRIS	1 << 2
#define SSPRIS_TXRIS	1 << 3

/* SPI 1 Masked Interrupt register */
#define SSPMIS_RORMIS	1 << 0
#define SSPMIS_RTMIS	1 << 1
#define SSPMIS_RXMIS	1 << 2
#define SSPMIS_TXMIS	1 << 3

/* SPI 1 Interrupt clear register */
#define SSPICR_RORIC	1 << 0
#define SSPICR_RTIC	1 << 1

extern void SSPHandler (void) __irq;
int LL_write(unsigned char *, unsigned short, unsigned char);
void LL_write_init(void);

extern unsigned char IMU_CalcData_updated;

#endif  /* __SSP_H__ */
/*****************************************************************************
**                            End Of File
******************************************************************************/

