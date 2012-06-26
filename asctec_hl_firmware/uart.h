/*
Copyright (c) 2011, Ascending Technologies GmbH
Copyright (c) 2011, Markus Achtelik, ASL, ETH Zurich, Switzerland
You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef __UART_H
#define __UART_H

#include <inttypes.h>

extern void UARTInitialize(unsigned int);
extern void UARTWriteChar(unsigned char);
extern unsigned char UARTReadChar(void);
extern void __putchar(int);
extern void UART_send(char *, unsigned char);
extern void UART_send_ringbuffer(void);
extern int ringbuffer(unsigned char, unsigned char*, unsigned int);

void startAutoBaud(void);
volatile extern char autobaud_in_progress;

int writePacket2Ringbuffer(uint8_t descriptor, void * data, uint8_t length);
extern void uart0ISR(void);

inline uint16_t crc16(void *, uint16_t count, uint16_t prev_crc);
inline uint16_t crc_update(uint16_t, uint8_t);

#define RBREAD 0
#define RBWRITE 1
#define RBFREE  2 
#define RINGBUFFERSIZE	384

int UART0_txEmpty(void);
void UART0_rxFlush(void);
void UART0_txFlush(void);
uint8_t UART0_writeFifo(void * data, uint32_t length);

// this has to be 2^n !!!
#define UART0_RX_BUFFERSIZE 512
#define UART0_TX_BUFFERSIZE 512

// not more than 64 different packettypes
#define PACKET_INFO_SIZE 64

typedef struct
{
  uint8_t descriptor;
  void * data;
  uint8_t updated;
} PacketInfo;

extern volatile unsigned int UART_rxPacketCount;
extern volatile unsigned int UART_rxGoodPacketCount;

typedef struct
{
  volatile uint8_t *buffer;
  uint8_t inUse;
  uint32_t bufferSize;
  uint32_t readIdx;
  uint32_t writeIdx;
  uint32_t tmp;
  uint32_t mask;
}volatile Fifo;

extern volatile short uart0_min_tx_buffer;
extern volatile short uart0_min_rx_buffer;

void Fifo_initialize(Fifo * fifo, volatile uint8_t * buffer, uint32_t bufferSize);
inline uint8_t Fifo_writeByte(Fifo * fifo, uint8_t byte);
inline uint8_t Fifo_writeBlock(Fifo * fifo, void *data, uint32_t length);
inline uint8_t Fifo_readByte(Fifo * fifo, uint8_t * byte);
inline uint16_t Fifo_availableMemory(Fifo * fifo);
inline void Fifo_reset(Fifo * fifo);

void parseRxFifo(void);

PacketInfo* registerPacket(uint8_t descriptor, void * data);

#endif //__UART_H
