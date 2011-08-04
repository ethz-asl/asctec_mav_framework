#include "main.h"
#include "LL_HL_comm.h"
#include "system.h"
#include "gpsmath.h"
#include "sdk.h"

unsigned short SSP_ack=0;
extern char SPIWRData[128];
extern char data_sent_to_HL;
extern char data_sent_to_LL;
extern unsigned int SPIWR_num_bytes;

struct LL_ATTITUDE_DATA LL_1khz_attitude_data;
struct LL_CONTROL_INPUT LL_1khz_control_input;

void SSP_data_distribution_HL(void)
{
	unsigned char i;
	unsigned char current_page=LL_1khz_attitude_data.system_flags&0x03;

	if(LL_1khz_attitude_data.system_flags&SF_GPS_NEW) gpsDataOkTrigger=0;

	IMU_CalcData.angle_nick=LL_1khz_attitude_data.angle_pitch*10;
	IMU_CalcData.angle_roll=LL_1khz_attitude_data.angle_roll*10;
	IMU_CalcData.angle_yaw=LL_1khz_attitude_data.angle_yaw*10;

	IMU_CalcData.angvel_nick=LL_1khz_attitude_data.angvel_pitch;
	IMU_CalcData.angvel_roll=LL_1khz_attitude_data.angvel_roll;
	IMU_CalcData.angvel_yaw=LL_1khz_attitude_data.angvel_yaw;

	if(!current_page)	//page 0
	{
		for(i=0;i<8;i++)
		{
			RO_RC_Data.channel[i]=LL_1khz_attitude_data.RC_data[i]*16;
		}
		IMU_CalcData.acc_x_calib=LL_1khz_attitude_data.acc_x*10;
		IMU_CalcData.acc_y_calib=LL_1khz_attitude_data.acc_y*10;
		IMU_CalcData.acc_z_calib=LL_1khz_attitude_data.acc_z*10;
	}
	else if(current_page==1)	//page 1
	{
		IMU_CalcData.height=LL_1khz_attitude_data.height;
		IMU_CalcData.dheight=LL_1khz_attitude_data.dheight;
	}
	else if(current_page==2)
	{
		IMU_CalcData.Hx=LL_1khz_attitude_data.mag_x;
		IMU_CalcData.Hy=LL_1khz_attitude_data.mag_y;
		IMU_CalcData.Hz=LL_1khz_attitude_data.mag_z;
	}
}

int HL2LL_write_cycle(void)	//write data to low-level processor
{
	static char pageselect=0;

	if(!data_sent_to_LL) return(0);

	//update 1kHz data
	LL_1khz_control_input.system_flags=0|pageselect;
	//SSP_ack=0;	//reset ack

	if(gpsDataOkTrigger) LL_1khz_control_input.system_flags|=SF_GPS_NEW;

#ifndef FALCON
	if(WO_SDK.ctrl_enabled)  LL_1khz_control_input.system_flags|=SF_HL_CONTROL_ENABLED;
	else LL_1khz_control_input.system_flags&=~SF_HL_CONTROL_ENABLED;

	if(WO_SDK.ctrl_mode==0x01) LL_1khz_control_input.system_flags|=SF_DIRECT_MOTOR_CONTROL;
	else LL_1khz_control_input.system_flags&=~SF_DIRECT_MOTOR_CONTROL;
#else //Disable Control Input if system is a FALCON
	LL_1khz_control_input.system_flags&=~SF_HL_CONTROL_ENABLED;
	LL_1khz_control_input.system_flags&=~SF_DIRECT_MOTOR_CONTROL;
#endif



	LL_1khz_control_input.ctrl_flags=WO_CTRL_Input.ctrl;
	LL_1khz_control_input.pitch=WO_CTRL_Input.pitch;
	LL_1khz_control_input.roll=WO_CTRL_Input.roll;
	LL_1khz_control_input.yaw=WO_CTRL_Input.yaw;
	LL_1khz_control_input.thrust=WO_CTRL_Input.thrust;

	if(WO_SDK.ctrl_mode==0x01)
	{
		LL_1khz_control_input.direct_motor_control[0]=WO_Direct_Motor_Control.pitch;
		LL_1khz_control_input.direct_motor_control[1]=WO_Direct_Motor_Control.roll;
		LL_1khz_control_input.direct_motor_control[2]=WO_Direct_Motor_Control.yaw;
		LL_1khz_control_input.direct_motor_control[3]=WO_Direct_Motor_Control.thrust;
	}

/*	for(i=0;i<8;i++)
	{
		LL_1khz_control_input.direct_motor_control[i]=0;
	}
*/
	if(pageselect==0)
	{
		//fill struct with 500Hz data
		LL_1khz_control_input.latitude=GPS_Data.latitude;
		LL_1khz_control_input.longitude=GPS_Data.longitude;
		LL_1khz_control_input.height=GPS_Data.height;
		LL_1khz_control_input.speed_x=GPS_Data.speed_x;
		LL_1khz_control_input.speed_y=GPS_Data.speed_y;
		LL_1khz_control_input.heading=GPS_Data.heading;
		LL_1khz_control_input.status=GPS_Data.status;

		//write data
		LL_write_ctrl_data(pageselect);
		//set pageselect to other page for next cycle
		pageselect=1;
	}
	else //pageselect=1
	{
		//fill struct with 500Hz data
		LL_1khz_control_input.hor_accuracy=GPS_Data.horizontal_accuracy;
		LL_1khz_control_input.vert_accuracy=GPS_Data.vertical_accuracy;
		LL_1khz_control_input.speed_accuracy=GPS_Data.speed_accuracy;
		LL_1khz_control_input.numSV=GPS_Data.numSV;
		LL_1khz_control_input.battery_voltage_1=HL_Status.battery_voltage_1;
		LL_1khz_control_input.battery_voltage_2=HL_Status.battery_voltage_2;
		LL_1khz_control_input.dummy_500Hz_2=0;
		LL_1khz_control_input.dummy_500Hz_3=0;

		//write data
		LL_write_ctrl_data(pageselect);
		//set pageselect to other page for next cycle
		pageselect=0;
	}
	return(1);
}

void LL_write_ctrl_data(char page)
{
	unsigned int i;
	unsigned char *dataptr;
	static volatile short spi_chksum;

	dataptr=(unsigned char *)&LL_1khz_control_input;

	//initialize syncbytes
	SPIWRData[0]='>';
	SPIWRData[1]='*';

	spi_chksum=0xAAAA;

	if(!page)
	{
		for(i=2;i<40;i++)
		{
			SPIWRData[i]=*dataptr++;
			spi_chksum+=SPIWRData[i];
		}
	}
	else
	{
		for(i=2;i<22;i++)
		{
			SPIWRData[i]=*dataptr++;
			spi_chksum+=SPIWRData[i];
		}
		dataptr+=18;
		for(i=22;i<40;i++)
		{
			SPIWRData[i]=*dataptr++;
			spi_chksum+=SPIWRData[i];
		}
	}

	SPIWRData[40]=spi_chksum;		//chksum LSB
	SPIWRData[41]=(spi_chksum>>8);	//chksum MSB

	SPIWR_num_bytes=42;
	data_sent_to_LL=0;
}


inline void SSP_rx_handler_HL(unsigned char SPI_rxdata)	//rx_handler @ high-level processor
{
	static volatile unsigned char SPI_syncstate=0;
	static volatile unsigned char SPI_rxcount=0;
	static volatile unsigned char *SPI_rxptr;
	static volatile unsigned char incoming_page;

        //receive handler
        if (SPI_syncstate==0)
		{
			if (SPI_rxdata=='>') SPI_syncstate++; else SPI_syncstate=0;
		}
		else if (SPI_syncstate==1)
		{
			if (SPI_rxdata=='*')
			{
				SPI_syncstate++;
				SPI_rxptr=(unsigned char *)&LL_1khz_attitude_data;
				SPI_rxcount=40;
			}
			else SPI_syncstate=0;
		}
		else if (SPI_syncstate==2)
		{
			if(SPI_rxcount==26) //14 bytes transmitted => select 500Hz page
			{
				incoming_page=LL_1khz_attitude_data.system_flags&0x03;	//system flags were already received
				if(incoming_page==1) SPI_rxptr+=26;
				else if(incoming_page==2) SPI_rxptr+=52;
			}
			SPI_rxcount--;
			*SPI_rxptr=SPI_rxdata;
			SPI_rxptr++;
			if (SPI_rxcount==0)
        	{
             	SPI_syncstate++;
        	}
		}
		else if (SPI_syncstate==3)
		{
			if(SPI_rxdata=='<')	//last byte ok => data should be valid
			{
				SSP_data_distribution_HL();	//only distribute data to other structs, if it was received correctly
											//ack data receiption
			}
			SPI_syncstate=0;
		}
		else SPI_syncstate=0;
}

