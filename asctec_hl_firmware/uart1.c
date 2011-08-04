#include "LPC214x.h"
#include "interrupt_utils.h"
#include "system.h"
#include "main.h"
#include "uart1.h"
#include "irq.h"
#include "hardware.h"
#include "gpsmath.h"
#include "ssp.h"

unsigned char packets;
unsigned char DataOutputsPerSecond;
unsigned int uart_cnt;

unsigned char data_requested=0;
extern int ZeroDepth;

unsigned short current_chksum;
unsigned char chksum_to_check=0;
unsigned char chksum_trigger=1;

unsigned char transmission1_running=0;
unsigned char trigger_transmission=0;

volatile unsigned char baudrate1_change=0;

unsigned char send_buffer[16];
unsigned char *tx_buff;
unsigned char UART1_syncstate=0;
unsigned int UART1_rxcount=0;
unsigned char *UART1_rxptr;

unsigned char UART_CalibDoneFlag = 0;

static volatile unsigned char rb_busy=0;

static volatile unsigned char GPS_ACK_received=0;

/*
//configuration commands for GPS
const unsigned char GPS_CFG_PRT[26] =
		{	0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xd0, 0x08, 0x08, 0x00, 0x00,
			0xe1, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe4, 0x2d };
const unsigned char GPS_CFG_ANT[10] =
		{	0x06, 0x13, 0x04, 0x00, 0x0b, 0x00, 0x0f, 0x38, 0x6f, 0x4f };
const unsigned char GPS_CFG_MSG[11][12] =
		{	{0x06, 0x01, 0x06, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x80},
			{0x06, 0x01, 0x06, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x11, 0x88},
			{0x06, 0x01, 0x06, 0x00, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x12, 0x8d},
			{0x06, 0x01, 0x06, 0x00, 0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x12, 0x8f},
			{0x06, 0x01, 0x06, 0x00, 0x01, 0x06, 0x00, 0x01, 0x00, 0x00, 0x15, 0x9c},
			{0x06, 0x01, 0x06, 0x00, 0x01, 0x11, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xd0},
			{0x06, 0x01, 0x06, 0x00, 0x01, 0x12, 0x00, 0x01, 0x00, 0x00, 0x21, 0xd8},
			{0x06, 0x01, 0x06, 0x00, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x2e, 0x1b},
			{0x06, 0x01, 0x06, 0x00, 0x01, 0x21, 0x00, 0x00, 0x00, 0x00, 0x2f, 0x20},
			{0x06, 0x01, 0x06, 0x00, 0x01, 0x22, 0x00, 0x00, 0x00, 0x00, 0x30, 0x25},
			{0x06, 0x01, 0x06, 0x00, 0x01, 0x30, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x6b} };
const unsigned char GPS_CFG_NAV2[46] =
		{	0x06, 0x1a, 0x28, 0x00, 0x05, 0x00, 0x00, 0x00, 0x04, 0x03, 0x10, 0x02,
			0x50, 0xc3, 0x00, 0x00, 0x0f, 0x0a, 0x0a, 0x3c, 0x00, 0x01, 0x00, 0x00,
			0xfa, 0x00, 0xfa, 0x00, 0x64, 0x00, 0x2c, 0x01, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5e, 0x30 };
const unsigned char GPS_CFG_RATE[12] =
		{	0x06, 0x08, 0x06, 0x00, 0xc8, 0x00, 0x01, 0x00, 0x00, 0x00, 0xdd, 0x68 };
const unsigned char GPS_CFG_SBAS[14] =
		{	0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x2f, 0xd9 };
const unsigned char GPS_CFG_CFG[19] =
		{	0x06, 0x09, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x07, 0x21, 0xaf };
*/

// new commands for gps, sbas disabled
const unsigned char GPS_CFG_PRT[26] =
                {       0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xd0, 0x08, 0x08, 0x00, 0x00,
                        0xe1, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe4, 0x2d };
const unsigned char GPS_CFG_ANT[10] =
                {       0x06, 0x13, 0x04, 0x00, 0x0b, 0x00, 0x0f, 0x38, 0x6f, 0x4f };
const unsigned char GPS_CFG_MSG[11][12] =
                {       {0x06, 0x01, 0x06, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x80},
                        {0x06, 0x01, 0x06, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x11, 0x88},
                        {0x06, 0x01, 0x06, 0x00, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x12, 0x8d},
                        {0x06, 0x01, 0x06, 0x00, 0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x12, 0x8f},
                        {0x06, 0x01, 0x06, 0x00, 0x01, 0x06, 0x00, 0x01, 0x00, 0x00, 0x15, 0x9c},
                        {0x06, 0x01, 0x06, 0x00, 0x01, 0x11, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xd0},
                        {0x06, 0x01, 0x06, 0x00, 0x01, 0x12, 0x00, 0x01, 0x00, 0x00, 0x21, 0xd8},
                        {0x06, 0x01, 0x06, 0x00, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x2e, 0x1b},
                        {0x06, 0x01, 0x06, 0x00, 0x01, 0x21, 0x00, 0x00, 0x00, 0x00, 0x2f, 0x20},
                        {0x06, 0x01, 0x06, 0x00, 0x01, 0x22, 0x00, 0x00, 0x00, 0x00, 0x30, 0x25},
                        {0x06, 0x01, 0x06, 0x00, 0x01, 0x30, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x6b} };
const unsigned char GPS_CFG_SBAS[14] =
            {     0x06, 0x16, 0x08, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
                  0x25, 0x90 }; //SBAS OFF
const unsigned char GPS_CFG_NAV2[46] =
            {     0x06, 0x1a, 0x28, 0x00, 0x05, 0x00, 0x00, 0x00, 0x04, 0x03, 0x0A, 0x02,
                  0x50, 0xc3, 0x00, 0x00, 0x0f, 0x0a, 0x0a, 0x3c, 0x00, 0x01, 0x00, 0x00,
                  0xfa, 0x00, 0xfa, 0x00, 0x64, 0x00, 0x2c, 0x01, 0x00, 0x00, 0x00, 0x00,
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x64 }; //max SV=10
const unsigned char GPS_CFG_RATE[12] =
            {     0x06, 0x08, 0x06, 0x00, 0xc8, 0x00, 0x01, 0x00, 0x00, 0x00, 0xdd, 0x68 };      //5Hz
const unsigned char GPS_CFG_CFG[19] =
                {       0x06, 0x09, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x07, 0x21, 0xaf };

/*

//globals for NMEA parser
	double latitudeDeg;
	double longitudeDeg;
	char gprmc_string[5]="GPRMC";
	static unsigned char gpsState=GPS_IDLE;
	static unsigned char gpsCnt=0;
	static unsigned char gpsFieldStart=1;
	static unsigned char gpsFieldCnt=0;
	static unsigned char gpsInitString[5];
	static unsigned char gpsValue[20];
//<- globals
*/
unsigned char startstring[]={'>','*','>'};
unsigned char stopstring[]={'<','#','<'};

void parse_POSLLH(unsigned char, unsigned char);
void parse_POSUTM(unsigned char, unsigned char);
void parse_VELNED(unsigned char, unsigned char);
void parse_STATUS(unsigned char, unsigned char);
void parse_NAVSOL(unsigned char, unsigned char);

inline void parse_VELNED(unsigned char c, unsigned char reset)
{
	static unsigned char cnt=0;
	static int VE, VN, heading;
	static unsigned int sacc;

	if(reset) cnt=0;
	else
	{
		if(cnt==0) VN=c;
		else if(cnt==1) VN+=c<<8;
		else if(cnt==2) VN+=c<<16;
		else if(cnt==3) VN+=c<<24;
		else if(cnt==4) VE=c;
		else if(cnt==5) VE+=c<<8;
		else if(cnt==6) VE+=c<<16;
		else if(cnt==7) VE+=c<<24;
		else if(cnt==20) heading=c;
		else if(cnt==21) heading+=c<<8;
		else if(cnt==22) heading+=c<<16;
		else if(cnt==23) heading+=c<<24;
		else if(cnt==24) sacc=c;
		else if(cnt==25) sacc+=c<<8;
		else if(cnt==26) sacc+=c<<16;
		else if(cnt==27)
		{
			sacc+=c<<24;
			GPS_Data.speed_x=VE*10;	//convert to mm/s
			GPS_Data.speed_y=VN*10; //convert to mm/s
			GPS_Data.heading=heading/100;	//convert to deg * 1000
			GPS_Data.speed_accuracy=sacc*10;	//convert to mm/s
			gpsDataOkTrigger=1;
		}
		cnt++;
	}
}
inline void parse_POSLLH(unsigned char c, unsigned char reset)
{
	static unsigned char cnt=0;
	static int lat, lon, height;
	static unsigned int hacc, vacc;

	if(reset) cnt=0;
	else
	{
		if(cnt==0) lon=c;
		else if(cnt==1) lon+=c<<8;
		else if(cnt==2) lon+=c<<16;
		else if(cnt==3) lon+=c<<24;
		else if(cnt==4) lat=c;
		else if(cnt==5) lat+=c<<8;
		else if(cnt==6) lat+=c<<16;
		else if(cnt==7) lat+=c<<24;
		else if(cnt==12) height=c;
		else if(cnt==13) height+=c<<8;
		else if(cnt==14) height+=c<<16;
		else if(cnt==15) height+=c<<24;
		else if(cnt==16) hacc=c;
		else if(cnt==17) hacc+=c<<8;
		else if(cnt==18) hacc+=c<<16;
		else if(cnt==19) hacc+=c<<24;
		else if(cnt==20) vacc=c;
		else if(cnt==21) vacc+=c<<8;
		else if(cnt==22) vacc+=c<<16;
		else if(cnt==23)
		{
			vacc+=c<<24;
			GPS_Data.latitude=lat;
			GPS_Data.longitude=lon;
			GPS_Data.height=height;
			GPS_Data.horizontal_accuracy=hacc;
			GPS_Data.vertical_accuracy=vacc;
		}
		cnt++;
	}
}
inline void parse_POSUTM(unsigned char c, unsigned char reset)
{
	static unsigned char cnt=0;
	static int E, N;

	if(reset) cnt=0;
	else
	{
		if(cnt==0) E=c;
		else if(cnt==1) E+=c<<8;
		else if(cnt==2) E+=c<<16;
		else if(cnt==3) E+=c<<24;
		else if(cnt==4) N=c;
		else if(cnt==5) N+=c<<8;
		else if(cnt==6) N+=c<<16;
		else if(cnt==7)
		{
			N+=c<<24;
//			GPS_Data.x=E;
//			GPS_Data.y=N;
		}
		cnt++;
	}
}

//NAVSOL is the only packet where the first 4 bytes need to be parsed. Any other packet discardes the first 4 bytes!!!
inline void parse_NAVSOL(unsigned char c, unsigned char reset)
{
	static unsigned char cnt=0;
	static unsigned int tow;
	static unsigned short week;

	if(reset) cnt=0;
	else
	{
		if(cnt==0) tow=c;
		else if(cnt==1)	tow+=c<<8;
		else if(cnt==2)	tow+=c<<16;
		else if(cnt==3) tow+=c<<24;
		else if(cnt==8) week=c;
		else if(cnt==9)
		{
			week+=c<<8;
			GPS_Time.time_of_week=tow;
			GPS_Time.week=week;
		}
		else if(cnt==47)
		{
			GPS_Data.numSV=c;
		}
		cnt++;
	}
}

inline void parse_STATUS(unsigned char c, unsigned char reset)
{
	static unsigned char cnt=0;
	static unsigned char GPSfix, flags, diffs;

	if(reset) cnt=0;
	else
	{
		if(cnt==0) GPSfix=c;
		else if(cnt==1) flags=c;
		else if(cnt==2)
		{
			diffs=c;
			GPS_Data.status=GPSfix|(flags<<8)|(diffs<<16);
		}
		cnt++;
	}
}

void uart1ISR(void) __irq
{
  static unsigned char state;
  static unsigned char current_packet;
  static unsigned short cnt, length;
  unsigned char t;
  unsigned char c;
  IENABLE;
  unsigned iir = U1IIR;
  // Handle UART interrupt
  switch ((iir >> 1) & 0x7)
    {
      case 1:
		  // THRE interrupt

		 if (ringbuffer1(RBREAD, &t, 1))
		 {
		   transmission1_running=1;
		   UART1WriteChar(t);
		 }
		 else
		 {
		   transmission1_running=0;
		   if (baudrate1_change)		//baudrate change after first GPS config command
		   {
			   UART1Initialize(57600);
			   baudrate1_change=0;
		   }
		 }
        break;
      case 2:
		c=U1RBR;

		//UARTWriteChar(c);

#ifndef INDOOR_GPS	//run GPS statemachine

        //parse UBX (U0RBR);

	//SSP_trans_cnt++;
		switch (state)
		{
			case 0:
				if(c==0xB5)
				{
					state=1;
				}
			break;
			case 1:
				if(c==0x62)
				{
					state=2;
				}
				else state=0;
			break;
			case 2:
				if(c==0x01)	//NAV message
				{
				 	state=3;
				}
				else if (c==0x05)	//ACK message
					{
						state=10;
					}
				else state=0;
			break;
			case 3:
				current_packet=c;
				cnt=0;
				state=4;
			break;
			case 4:
				if(!cnt) length=c;
				if(current_packet==0x06) parse_NAVSOL(0,1);
				if(++cnt==2)
				{
					cnt=0;
					state=5;
				}
			break;
			case 5:	//Four bytes ITOW
				//NAVSOL is the only packets where the first 4 bytes need to be parsed. Any other packet discardes the first 4 bytes!!!
				if(current_packet==0x06) parse_NAVSOL(c,0);
				if(++cnt==4)
				{
					cnt=0;
					state=6;
					if(current_packet==0x02) parse_POSLLH(0,1);
					//else if(current_packet==0x08) parse_POSUTM(0,1);
					else if(current_packet==0x03) parse_STATUS(0,1);
					else if(current_packet==0x12) parse_VELNED(0,1);
				}
			break;
			case 6:
				if(current_packet==0x02)
				{
					parse_POSLLH(c,0);
				}
		/*		else if(current_packet==0x08	//POSUTM currently not used
				{
					parse_POSUTM(c,0);
				}
			*/	else if(current_packet==0x03)
				{
					parse_STATUS(c,0);
				}
				else if(current_packet==0x12)
				{
					parse_VELNED(c,0);
				}
				else if(current_packet==0x06)
				{
					parse_NAVSOL(c,0);
				}
				else state=0;

				if(++cnt>=length-4)
				{
					state=0;
				}
			break;
			case 10:
				if (c==0x01)
				{
					cnt=0;
					state=11;
				} else
					state=0;
			break;
			case 11:
				if (!cnt) length=c;
				if (cnt++==1)
				{
					cnt=0;
					state=12;
				}
			break;
			case 12:
				if (c==0x06)		//ACK of a CFG-message
				{
					state=13;
				} else
					state=0;
			break;
			case 13:
				state=14;
			break;
			case 14:
				if (!GPS_ACK_received)
				{
					GPS_ACK_received=1;
					state=0;
				}
			break;
			default:
				state=0;
			break;
		}

#else	//run optical tracking statemachine
		switch (state)
		{
			case 0:
				if(c=='>') state=1;
			break;
			case 1:
				if(c=='*') state=2;
				else state=0;
			break;
			case 2:
				if(c=='>')	//Startstring received
				{
					UART1_rxcount=sizeof(OF_Data);
					UART1_rxptr=(unsigned char *)&OF_Data_e;
				 	state=3;
				}
				else state=0;
			break;
			case 3:
				UART1_rxcount--;
				*UART1_rxptr=c;
				UART1_rxptr++;
				if (UART1_rxcount==0)
	        	{
	             	state=0;
	             	OF_data_updated=0;
	        	}
			break;
			default:
			state=0;
			break;
		}
#endif

        break;
      case 3:
        // RLS interrupt
        break;
      case 6:
        // CTI interrupt
        break;
   }
  IDISABLE;
  VICVectAddr = 0;		/* Acknowledge Interrupt */
}

void UART1Initialize(unsigned int baud)
{
  unsigned int divisor = peripheralClockFrequency() / (16 * baud);
//UART1
  U1LCR = 0x83; /* 8 bit, 1 stop bit, no parity, enable DLAB */
  U1DLL = divisor & 0xFF;
  U1DLM = (divisor >> 8) & 0xFF;
  U1LCR &= ~0x80; /* Disable DLAB */
  U1FCR = 1;
}

//Write to UART1
void UART1WriteChar(unsigned char ch)
{
  while ((U1LSR & 0x20) == 0);
  U1THR = ch;
}

unsigned char UART1ReadChar(void)
{
  while ((U1LSR & 0x01) == 0);
  return U1RBR;
}

void UART1_send(unsigned char *buffer, unsigned char length)
{
	unsigned char cnt=0;
	while(length--)
	{
		while (!(U0LSR & 0x20)); //wait until U0THR is empty
		U1THR = buffer[cnt++];
	}
}

void UART1_send_ringbuffer(void)
{
  unsigned char t;
  if(!transmission1_running)
  {
    if(ringbuffer1(RBREAD, &t, 1))
    {
      transmission1_running=1;
      UART1WriteChar(t);
    }
  }
}

int ringbuffer1(unsigned char rw, unsigned char *data, unsigned int count)	//returns 1 when write/read was successful, 0 elsewise
{
    static volatile unsigned char buffer[RINGBUFFERSIZE];
//	static volatile unsigned int pfirst=0, plast=0;	//Pointers to first and last to read byte
	static volatile unsigned int read_pointer, write_pointer;
	static volatile unsigned int content=0;
	unsigned int p=0;
    unsigned int p2=0;

	if(rw==RBWRITE)
	{
		if(count<RINGBUFFERSIZE-content)	//enough space in buffer?
		{
			while(p<count)
			{
				buffer[write_pointer++]=data[p++];
			}
            content+=count;
            return(1);
		}
	}
	else if(rw==RBREAD)
	{
		if(content>=count)
		{
			while(p2<count)
			{
				data[p2++]=buffer[read_pointer++];
			}
            content-=count;
            if(!content) //buffer empty
            {
            	write_pointer=0;
            	read_pointer=0;
            }
			return(1);
		}
	}
        else if(rw==RBFREE)
        {
          if(content) return 0;
          else return(RINGBUFFERSIZE-11);
        }

	return(0);
}

void GPS_configure(void)
{
	const unsigned char gps_startstring[2]={ 0xb5, 0x62 };		//sync chars

	static unsigned char gpsconf_state, gps_cfg_msg_counter;
	int written;
	static unsigned char gpsconfig_timeout;
	static unsigned char StartWithHighBaudrate = 0;

	switch (gpsconf_state)
	{
		case 0:
			if (StartWithHighBaudrate)
			{
				UART1Initialize(57600);
				StartWithHighBaudrate = 0;
			}
			else
				UART1Initialize(9600);
			if ((ringbuffer1(RBFREE, 0, 0))>28)
			{
				written=ringbuffer1(RBWRITE, (unsigned char*)gps_startstring, 2);
				written=ringbuffer1(RBWRITE, (unsigned char*)GPS_CFG_PRT, 26);
				UART1_send_ringbuffer();
				baudrate1_change=1;
				gpsconf_state++;
			}
			gpsconfig_timeout=0;
		break;
		case 1:
			if (GPS_ACK_received)
			{
				GPS_ACK_received=0;
				if (ringbuffer1(RBFREE, 0, 0)>12)
				{
					ringbuffer1(RBWRITE, (unsigned char*)gps_startstring, 2);
					ringbuffer1(RBWRITE, (unsigned char*)GPS_CFG_ANT, 10);
					UART1_send_ringbuffer();
					gps_cfg_msg_counter=0;
					gpsconf_state++;
				}
				gpsconfig_timeout=0;
				StartWithHighBaudrate = 0;
			} else
			{
				gpsconfig_timeout++;
				StartWithHighBaudrate = 1;
			}
		break;
		case 2:
			if (GPS_ACK_received)
			{
				GPS_ACK_received=0;
				if (ringbuffer1(RBFREE, 0, 0)>14)
				{
					ringbuffer1(RBWRITE, (unsigned char*)gps_startstring, 2);
					ringbuffer1(RBWRITE, (unsigned char*)GPS_CFG_MSG[gps_cfg_msg_counter], 12);
					UART1_send_ringbuffer();
					if (++gps_cfg_msg_counter==11)
					{
						gpsconf_state++;
					}
				}
				gpsconfig_timeout=0;
			} else
				gpsconfig_timeout++;
		break;
		case 3:
			if (GPS_ACK_received)
			{
				GPS_ACK_received=0;
				if (ringbuffer1(RBFREE, 0, 0)>48)
				{
					ringbuffer1(RBWRITE, (unsigned char*)gps_startstring, 2);
					ringbuffer1(RBWRITE, (unsigned char*)GPS_CFG_NAV2, 46);
					UART1_send_ringbuffer();
					gpsconf_state++;
				}
				gpsconfig_timeout=0;
			} else
				gpsconfig_timeout++;
		break;
		case 4:
			if (GPS_ACK_received)
			{
				GPS_ACK_received=0;
				if (ringbuffer1(RBFREE, 0, 0)>14)
				{
					ringbuffer1(RBWRITE, (unsigned char*)gps_startstring, 2);
					ringbuffer1(RBWRITE, (unsigned char*)GPS_CFG_RATE, 12);
					UART1_send_ringbuffer();
					gpsconf_state++;
				}
				gpsconfig_timeout=0;
			} else
				gpsconfig_timeout++;
		break;
		case 5:
			if (GPS_ACK_received)
			{
				GPS_ACK_received=0;
				if (ringbuffer1(RBFREE, 0, 0)>16)
				{
					ringbuffer1(RBWRITE, (unsigned char*)gps_startstring, 2);
					ringbuffer1(RBWRITE, (unsigned char*)GPS_CFG_SBAS, 14);
					UART1_send_ringbuffer();
					gpsconf_state++;
				}
				gpsconfig_timeout=0;
			} else
				gpsconfig_timeout++;
		break;
		case 6:
			if (GPS_ACK_received)
			{
				GPS_ACK_received=0;
				if (ringbuffer1(RBFREE, 0, 0)>21)
				{
					ringbuffer1(RBWRITE, (unsigned char*)gps_startstring, 2);
					ringbuffer1(RBWRITE, (unsigned char*)GPS_CFG_CFG, 19);
					UART1_send_ringbuffer();
					gpsconf_state++;
				}
				gpsconfig_timeout=0;
			} else
				gpsconfig_timeout++;
		break;
		case 7:
			if (GPS_ACK_received)
			{
				GPS_ACK_received=0;
				GPS_init_status=GPS_IS_CONFIGURED;
				gpsconf_state=0;
				gpsconfig_timeout=0;
			} else
				gpsconfig_timeout++;
		break;
		default:
			gpsconf_state=0;
		break;
	}
	if (gpsconfig_timeout>GPSCONF_TIMEOUT)				//timeout for ACK receiving
	{
		if (StartWithHighBaudrate)
		{
			gpsconf_state=0;
			gpsconfig_timeout=0;
		} else
		{
			gpsconf_state=0;
			gpsconfig_timeout=0;
			GPS_init_status=GPS_CONFIG_ERROR;
		}
	}
}
