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

#include <string.h>

#include "LPC214x.h"
#include "interrupt_utils.h"
#include "system.h"
#include "main.h"
#include "uart.h"
#include "irq.h"
#include "hardware.h"
#include "gpsmath.h"
#include "ssp.h"
#include "lpcUART.h"

#include "HL_interface.h"

volatile unsigned char transmission_running = 0;

unsigned uart0_rx_cpsr;
unsigned uart0_tx_cpsr;

#define UART0_DISABLE_TX_INT uart0_tx_cpsr=disableIRQ();U0IER &= ~UIER_ETBEI;restoreIRQ(uart0_tx_cpsr);
#define UART0_ENABLE_TX_INT  uart0_tx_cpsr=disableIRQ();U0IER |= UIER_ETBEI;restoreIRQ(uart0_tx_cpsr);
#define UART0_DISABLE_RX_INT uart0_rx_cpsr=disableIRQ();U0IER &= ~UIER_ERBFI;restoreIRQ(uart0_rx_cpsr);
#define UART0_ENABLE_RX_INT  uart0_rx_cpsr=disableIRQ();U0IER |= UIER_ERBFI;restoreIRQ(uart0_rx_cpsr);

volatile short uart0_min_tx_buffer = UART0_TX_BUFFERSIZE;
volatile short uart0_min_rx_buffer = UART0_RX_BUFFERSIZE;

volatile uint8_t rxBuffer[UART0_RX_BUFFERSIZE];
uint8_t rxParseBuffer[UART0_RX_BUFFERSIZE];
Fifo rxFifo;

volatile uint8_t txBuffer[UART0_TX_BUFFERSIZE];
Fifo txFifo;

volatile unsigned int UART_rxPacketCount = 0;
volatile unsigned int UART_rxGoodPacketCount = 0;

PacketInfo packetInfo[PACKET_INFO_SIZE];
uint32_t registeredPacketCnt = 0;

volatile char autobaud_in_progress = 0;

void uart0ISR(void) __irq
{
  uint8_t t;
  uint16_t iid;
  short freemem = 0;

  // Read IIR to clear interrupt and find out the cause
  while (((iid = U0IIR) & UIIR_NO_INT) == 0)
  {
    if (iid & 0x100)
    {
      autobaud_in_progress = 1;
      U0ACR |= 0x100; //clear ABEO interrupt
      U0ACR &= ~0x01; // disable autobaud
      U0IER &= ~((1 << 8) | (1 << 9)); //disable ABEO and ABTO interrupts

      autobaud_in_progress = 0;
    }

    if (iid & 0x200)
    {
      autobaud_in_progress = 1;
      U0ACR |= 0x200; //clear ABTO int

      autobaud_in_progress = 0;
    }

    switch (iid & UIIR_ID_MASK)
    {
      case UIIR_RLS_INT: // Receive Line Status
        U0LSR; // read LSR to clear
        break;

      case UIIR_CTI_INT: // <-- keep this together, in order to read remaining bytes below fifo threshold
      case UIIR_RDA_INT:
        // RDA interrupt
        //receive handler
        rxFifo.inUse = 1;
        do
        { // read from fifo as long as there is data available
          t = U0RBR;
          freemem = Fifo_availableMemory(&rxFifo);
          if (freemem < uart0_min_rx_buffer)
            uart0_min_rx_buffer = freemem;
          if (!Fifo_writeByte(&rxFifo, t))
            break;
        } while (U0LSR & ULSR_RDR);
        rxFifo.inUse = 0;

        break;

      case UIIR_THRE_INT:
        // THRE interrupt
        if(!(IOPIN0&(1<<CTS_RADIO))){
          txFifo.inUse = 1;
          while (U0LSR & ULSR_THRE)
          {
            if (Fifo_readByte(&txFifo, &t))
            {
              U0THR = t;
            }
            else
            {
              transmission_running = 0;
              break;
            }
          }
          txFifo.inUse = 0;
        }
        break;

      default: // Unknown
        U0LSR;
        U0RBR;
        break;
    }
  }
  VICVectAddr = 0; // Acknowledge Interrupt
}

void Fifo_initialize(Fifo * fifo, volatile uint8_t * buffer, uint32_t bufferSize)
{
  fifo->buffer = buffer;
  fifo->bufferSize = bufferSize;
  fifo->readIdx = 0;
  fifo->writeIdx = 0;
  fifo->tmp = 0;
  fifo->mask = bufferSize - 1;
  fifo->inUse = 0;
}

uint8_t Fifo_writeByte(Fifo * fifo, uint8_t byte)
{
  fifo->tmp = ((fifo->writeIdx + 1) & fifo->mask);
  if (fifo->readIdx == fifo->tmp)
    return 0;
  fifo->buffer[fifo->writeIdx] = byte;
  fifo->writeIdx = fifo->tmp;
  return 1;
}

uint8_t Fifo_writeBlock(Fifo * fifo, void *data, uint32_t length)
{
  if (Fifo_availableMemory(fifo) <= length)
    return 0;
  uint8_t *ptr = (uint8_t *)data;
  while (length--)
  {
    fifo->buffer[fifo->writeIdx] = *ptr++;
    fifo->writeIdx = (fifo->writeIdx + 1) & fifo->mask;
  }

  //	//safe method
  //	int i=0;
  //	uint8_t *ptr = (uint8_t *)data;
  //	for(i=0; i<length; i++){
  //		if(!Fifo_writeByte(fifo, ptr[i]))
  //			return 0;
  //	}

  return 1;
}

uint8_t Fifo_readByte(Fifo * fifo, uint8_t * byte)
{
  if (fifo->readIdx == fifo->writeIdx)
    return 0;
  *byte = fifo->buffer[fifo->readIdx];
  fifo->readIdx = (fifo->readIdx + 1) & fifo->mask;
  return 1;
}

uint16_t Fifo_availableMemory(Fifo * fifo)
{
  return (fifo->readIdx - fifo->writeIdx - 1) & fifo->mask;
}

void Fifo_reset(Fifo * fifo)
{
  fifo->writeIdx = 0;
  fifo->readIdx = 0;
}

PacketInfo* registerPacket(uint8_t descriptor, void * data)
{
  //	if(registeredPacketCnt < PACKET_INFO_SIZE){
  packetInfo[registeredPacketCnt].data = data;
  packetInfo[registeredPacketCnt].descriptor = descriptor;
  packetInfo[registeredPacketCnt].updated = 0;
  registeredPacketCnt++;
  return &packetInfo[registeredPacketCnt - 1];
  //	}
  // TODO: what if space for packets is exceeded??
  //	return NULL;
}

void parseRxFifo(void)
{
  static uint8_t packetType;
  static uint8_t flag;
  static int packetSize = 0;
  static int rxCount = 0;
  static uint16_t checksum_computed = 0;
  static uint16_t checksum_received = 0;
  static uint32_t syncstate = 0;
  static HLI_ACK packet_ack;
  uint32_t i = 0;
  uint8_t rxdata = 0;

  if (rxFifo.inUse == 1)
    return;

  //	UART0_DISABLE_RX_INT;

  while (Fifo_readByte(&rxFifo, &rxdata))
  {

    if (syncstate == 0)
    {
      if (rxdata == 'a')
        syncstate++;
      else
        syncstate = 0;

      rxCount = 0;
      checksum_received = 0;
      packetSize = 0;
      flag = 0;
    }
    else if (syncstate == 1)
    {
      if (rxdata == '*')
        syncstate++;
      else
        syncstate = 0;
    }
    else if (syncstate == 2)
    {
      if (rxdata == '>')
        syncstate++;
      else
        syncstate = 0;
    }
    else if (syncstate == 3)
    {
      packetSize = rxdata; // get size of packet
      syncstate++;
    }
    else if (syncstate == 4)
    {
      packetType = rxdata; // get packet type
      if (packetSize < 1)
        syncstate = 0;
      else
      {
        rxCount = packetSize;
        syncstate++;
      }
    }
    else if (syncstate == 5)
    {
      flag = rxdata;
      syncstate++;
    }
    else if (syncstate == 6) // read data
    {
      rxParseBuffer[packetSize - rxCount] = rxdata;
      rxCount--;

      if (rxCount == 0)
      {
        syncstate++;
      }
    }
    else if (syncstate == 7) // first byte of checksum
    {
      checksum_received = rxdata & 0xff;
      syncstate++;
    }
    else if (syncstate == 8) // second byte of checksum + check (and dispatch?)
    {
      checksum_received |= ((unsigned short)rxdata << 8);
      UART_rxPacketCount++;

      checksum_computed = crc16(&packetType, 1, 0xff);
      checksum_computed = crc16(&flag, 1, checksum_computed);
      checksum_computed = crc16(rxParseBuffer, packetSize, checksum_computed);

      if (checksum_received == checksum_computed)
      {
        UART_rxGoodPacketCount++;
        for (i = 0; i < registeredPacketCnt; i++)
        {
          if (packetType == packetInfo[i].descriptor)
          {
            memcpy((packetInfo[i].data), rxParseBuffer, packetSize);
            packetInfo[i].updated = 1;
            if (flag & HLI_COMM_ACK)
            {
              packet_ack.ack_packet = flag;
              writePacket2Ringbuffer(HLI_PACKET_ID_ACK, &packet_ack, sizeof(packet_ack));
            }
            break;
          }
        }
      }
      syncstate = 0;
    }
    else
      syncstate = 0;
  }
  //	UART0_ENABLE_RX_INT;
}

int writePacket2Ringbuffer(uint8_t descriptor, void * data, uint8_t length)
{
  static uint8_t header[] = {0xFF, 0x09, 0, 0};
  uint16_t checksum = 0;
  int state = 0;

  header[2] = length;
  header[3] = descriptor;
  checksum = crc16(&descriptor, 1, 0xff);
  checksum = crc16(data, length, checksum);

  state = 1;
  state &= UART0_writeFifo(header, sizeof(header));
  state &= UART0_writeFifo(data, length);
  state &= UART0_writeFifo(&checksum, sizeof(checksum));

  return state;
}

uint8_t UART0_writeFifo(void * data, uint32_t length)
{
  uint8_t ret = 0;
  short freemem;
  //	while(txFifo.inUse);
  //	UART0_DISABLE_TX_INT;
  ret = Fifo_writeBlock(&txFifo, data, length);
  freemem = Fifo_availableMemory(&txFifo);
  if (freemem < uart0_min_tx_buffer)
    uart0_min_tx_buffer = freemem;
  //	UART0_ENABLE_TX_INT;
  return ret;
}

void UARTInitialize(unsigned int baud)
{
  UART0_DISABLE_RX_INT;
  UART0_DISABLE_TX_INT;

  unsigned int divisor = peripheralClockFrequency() / (16 * baud);

  //UART0
  U0LCR = 0x83; /* 8 bit, 1 stop bit, no parity, enable DLAB */
  U0DLL = divisor & 0xFF;
  U0DLM = (divisor >> 8) & 0xFF;
  U0LCR &= ~0x80; /* Disable DLAB */
  U0FCR = UFCR_FIFO_ENABLE | UFCR_FIFO_TRIG8 | UFCR_RX_FIFO_RESET | UFCR_TX_FIFO_RESET;//1; fifo enable, trigger interrupt after 8 bytes in the fifo
  Fifo_initialize(&rxFifo, rxBuffer, UART0_RX_BUFFERSIZE);
  Fifo_initialize(&txFifo, txBuffer, UART0_TX_BUFFERSIZE);

  UART0_ENABLE_RX_INT;
  UART0_ENABLE_TX_INT;
}

void startAutoBaud(void)
{
  if (U0ACR & 0x01)
    return;

  U0ACR = 0x01 | 0x04;// start, mode 0, autorestart
  U0IER |= ((1 << 8) | (1 << 9)); //enable ABEO and ABTO interrupts
}

void UART0_rxFlush(void)
{
  U0FCR |= UFCR_RX_FIFO_RESET;
  Fifo_reset(&rxFifo);
}
void UART0_txFlush(void)
{
  U0FCR |= UFCR_TX_FIFO_RESET;
}

int UART0_txEmpty(void)
{
  return (U0LSR & (ULSR_THRE | ULSR_TEMT)) == (ULSR_THRE | ULSR_TEMT);
}

//Write to UART0
void UARTWriteChar(unsigned char ch)
{
  while ((U0LSR & 0x20) == 0)
    ;
  U0THR = ch;
}

unsigned char UARTReadChar(void)
{
  while ((U0LSR & 0x01) == 0)
    ;
  return U0RBR;
}

void __putchar(int ch)
{
  if (ch == '\n')
    UARTWriteChar('\r');
  UARTWriteChar(ch);
}

void UART_send(char *buffer, unsigned char length)
{
  unsigned char cnt = 0;
  while (!(U0LSR & 0x20))
    ; //wait until U0THR and U0TSR are both empty
  while (length--)
  {
    U0THR = buffer[cnt++];
    if (cnt > 15)
    {
      while (!(U0LSR & 0x20))
        ; //wait until U0THR is empty
    }
  }
}

void UART_send_ringbuffer(void)
{
  uint8_t t;
  if (!transmission_running)
  {
    if (Fifo_readByte(&txFifo, &t))
    {
      transmission_running = 1;
      UARTWriteChar(t);
    }
  }
}

inline uint16_t crc_update(uint16_t crc, uint8_t data)
{
  data ^= (crc & 0xff);
  data ^= data << 4;

  return ((((uint16_t)data << 8) | ((crc >> 8) & 0xff)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}

inline uint16_t crc16(void* data, uint16_t cnt, uint16_t crc)
{
  uint8_t * ptr = (uint8_t *)data;
  int i;

  for (i = 0; i < cnt; i++)
  {
    crc = crc_update(crc, *ptr);
    ptr++;
  }
  return crc;
}

