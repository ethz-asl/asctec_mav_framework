
#ifndef __UART1_H
#define __UART1_H

extern void UART1Initialize(unsigned int baud);

extern void UART1WriteChar(unsigned char);
extern unsigned char UART1ReadChar(void);
extern void UART1_send(unsigned char *, unsigned char);
extern void UART1_send_ringbuffer(void);
extern int ringbuffer1(unsigned char, unsigned char*, unsigned int);
extern void uart1ISR(void);
extern void GPS_configure(void);

extern unsigned char send_buffer[16];
extern unsigned char chksum_trigger;
extern unsigned char UART_CalibDoneFlag;
extern unsigned char trigger_transmission;
extern unsigned char transmission1_running;

#define RBREAD 0
#define RBWRITE 1
#define RBFREE  2 
#define RINGBUFFERSIZE	384

#define RX_IDLE 0
#define RX_ACTSYNC1 1
#define RX_ACTSYNC2 2
#define RX_ACTDATA 3
#define RX_ACTCHKSUM 4

#define GPSCONF_TIMEOUT 200

#endif //__UART_H

