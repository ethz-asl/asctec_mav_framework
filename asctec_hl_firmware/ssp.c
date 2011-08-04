#include "LPC214x.h"			/* LPC21XX Peripheral Registers	*/
#include "type.h"
#include "irq.h"
#include "ssp.h"
#include "main.h"
#include "system.h"
#include "LL_HL_comm.h"

char SPIWRData[128];
char SPIRDData[128];
int CurrentTxIndex;
int CurrentRxIndex;
unsigned int SPIWR_num_bytes;

volatile unsigned int SSP_trans_cnt=0;

unsigned char data_sent_to_LL=1;

unsigned char SSP_receiption_complete=1;
unsigned char IMU_CalcData_updated=0;


char data_sent_to_HL=1;

inline void SSPReceive(unsigned char);

void SSPHandler (void) __irq
{
    int regValue;
    unsigned short input_data;
//    unsigned char timeout=0;

    IENABLE;				/* handles nested interrupt */

    regValue = SSPMIS;
    if ( regValue & SSPMIS_RORMIS )	/* Receive overrun interrupt */
    {
		SSPICR = SSPICR_RORIC;		/* clear interrupt */
    }
    if ( regValue & SSPMIS_RTMIS )	/* Receive timeout interrupt */
    {
		SSPICR = SSPICR_RTIC;		/* clear interrupt */
    }

    if ( regValue & SSPMIS_RXMIS )	/* Rx at least half full */
    {
    			/* receive until it's empty */
	while ( SSPSR & SSPSR_RNE )
	{
		input_data=SSPDR;
	    //SSPReceive(input_data&0xFF);
	    //SSPReceive(input_data>>8);

		SSP_rx_handler_HL(input_data&0xFF);
		SSP_rx_handler_HL(input_data>>8);

		//SSP_trans_cnt+=2;
	    /* Wait until the Busy bit is cleared */
	//    while ( (!(SSPSR & SSPSR_BSY) )&&(timeout++<50) );
	}				/* interrupt will be cleared when */
					/* data register is read or written */
    }

    if ( regValue & SSPMIS_TXMIS )	/* Tx at least half empty */
    {
	/* transmit until it's full */
	while ( (SSPSR & SSPSR_TNF) )
	{
	    if(CurrentTxIndex<SPIWR_num_bytes)
	    {
	    	SSPDR = SPIWRData[CurrentTxIndex]|(SPIWRData[CurrentTxIndex+1]<<8);
	    	CurrentTxIndex+=2;
	    }
	    else
	    {
	    	CurrentTxIndex=0;
	    	SPIWR_num_bytes=0;
	    	data_sent_to_LL=1;
			SSPDR=0;
	    }

	    /* Wait until the Busy bit is cleared */
	//    while ( !(SSPSR & SSPSR_BSY) );
	}				/* interrupt will be cleared when */
					/* data register is read or written */
    }

    IDISABLE;
    VICVectAddr = 0;		/* Acknowledge Interrupt */
}


inline void SSPReceive(unsigned char SPI_rxdata)
{
	static unsigned char SPI_syncstate=0;
	static unsigned char SPI_rxcount=0;
	static unsigned char *SPI_rxptr;
	static volatile unsigned char incoming_data;

        //receive handler
        if (SPI_syncstate==0)
		{
			if (SPI_rxdata=='>') SPI_syncstate++; else SPI_syncstate=0;
		}
		else if (SPI_syncstate==1)
		{
			if (SPI_rxdata=='*') SPI_syncstate++; else SPI_syncstate=0;
		}
		else if (SPI_syncstate==2)
		{
			if (SPI_rxdata=='>') SPI_syncstate++; else SPI_syncstate=0;
		}
		else if (SPI_syncstate==3)
		{
			if (SPI_rxdata==PD_IMUCALCDATA) //IMU CalcData
			{
				SPI_rxcount=sizeof(IMU_CalcData);
				SPI_rxptr=(unsigned char *)&IMU_CalcData_tmp;
				SPI_syncstate=4;
				incoming_data=PD_IMUCALCDATA;
			}
			else if (SPI_rxdata==PD_IMURAWDATA) //IMU CalcData
			{
				SPI_rxcount=sizeof(IMU_RawData);
				SPI_rxptr=(unsigned char *)&IMU_RawData;
				SPI_syncstate=4;
				incoming_data=PD_IMURAWDATA;
			}
            else SPI_syncstate=0;
        }
        else if (SPI_syncstate==4)
		{
			SPI_rxcount--;
			*SPI_rxptr=SPI_rxdata;
			SPI_rxptr++;
			if (SPI_rxcount==0)
        	{
             	SPI_syncstate=5;
             	if(incoming_data==PD_IMUCALCDATA)
             	{
             		IMU_CalcData_updated=1;
             	}
             	incoming_data=0;
        	}
		}
		else if(SPI_syncstate==5) //check if another packet is pending
		{
			if(SPI_rxdata==0)
			{
				SPI_syncstate=0;
			}
			else SPI_syncstate=1;
		}
		else SPI_syncstate=0;

		if(!SPI_syncstate) SSP_receiption_complete=1;
		else SSP_receiption_complete=0;
}

void LL_write_init(void)
{
		SPIWRData[0]='>';
		SPIWRData[1]='*';
		SPIWRData[2]='>';
}

int LL_write(unsigned char *data, unsigned short cnt, unsigned char PD )	//write data to high-level processor
{
	unsigned int i;
/*
	if(data_sent_to_LL)
	{
		//SSP_trans_cnt++;
		if(!SPIWR_num_bytes)
		{
			SPIWRData[3]=PD;
			for(i=0; i<cnt; i++)
			{
				SPIWRData[i+4]=data[i];
			}
			SPIWRData[cnt+4]=0;
			SPIWR_num_bytes=cnt+5;

		}
	}
*/
	if(data_sent_to_LL)
	{
		SPIWRData[3]=PD;
		for(i=0; i<cnt; i++)
		{
			SPIWRData[i+4]=data[i];
		}
		SPIWRData[cnt+4]=0;
		SPIWR_num_bytes=cnt+5;
	}
	else if(SPIWR_num_bytes+cnt<127)
	{
		SPIWRData[SPIWR_num_bytes-1]='>';
		SPIWRData[0+SPIWR_num_bytes]='*';
		SPIWRData[1+SPIWR_num_bytes]='>';
		SPIWRData[2+SPIWR_num_bytes]=PD;
		for(i=SPIWR_num_bytes; i<cnt+SPIWR_num_bytes; i++)
		{
			SPIWRData[i+3]=data[i-SPIWR_num_bytes];
		}
		SPIWR_num_bytes+=cnt+5;
		SPIWRData[SPIWR_num_bytes-1]=0;
	}
	else return(0);
	data_sent_to_LL=0;

	return(1);
}


