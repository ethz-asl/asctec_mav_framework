/*
 * ublox.c
 *
 *  Created on: 16.11.2011
 *      Author: jan
 */
#include "LPC214x.h"
#include "system.h"
#include "main.h"
#include "uart.h"
#include "irq.h"
#include "hardware.h"
#include "gpsmath.h"
#include "ssp.h"
#include "LL_HL_comm.h"
#include "ublox.h"

unsigned int urTimeCnt = 0;

volatile unsigned char urAckReceived = 0;
volatile unsigned char urAckClass = 0;
volatile unsigned char urAckId = 0;

unsigned char urConfigCnt;

#define MAX_COMMON_CFG_PACKETS_ANT4 5
#define MAX_COMMON_CFG_PACKETS_UB6 6
const unsigned char urCfgIdListAnt4[MAX_COMMON_CFG_PACKETS_ANT4] = {0x00, 0x08, 0x13, 0x16, 0x1A};
const unsigned char urCfgIdListUB6[MAX_COMMON_CFG_PACKETS_UB6] = {0x00, 0x08, 0x13, 0x16, 0x24, 0x23};

unsigned char * urCfgIdList;

volatile unsigned char urConfigMessageReceived = 0;
volatile unsigned char urVersionCheck = 0;

volatile unsigned char urDesiredRate;

volatile unsigned char GPS_RawBuffer[UR_MAX_RAWDATA_LENGTH];
volatile unsigned int GPS_raw_data_size;
volatile unsigned char GPS_trigger_raw_logging;

//CFG-PRT
const unsigned char urCfg00Ant4[] = {0x01, 0x00, 0x00, 0x00, 0xd0, 0x08, 0x00, 0x00, 0x00, 0xe1, 0x00, 0x00, 0x01,
                                     0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char urCfg00UB6[] = {0x01, 0x12, 0x00, 0x00, 0xc0, 0x08, 0x00, 0x00, 0x00, 0xe1, 0x00, 0x00, 0x01, 0x00,
                                    0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
//CFG-RATE
const unsigned char urCfg08[] = {0xc8, 0x00, 0x01, 0x00, 0x00, 0x00};
//CFG-ANT
const unsigned char urCfg13[] = {0x0b, 0x00, 0x00, 0x00};
//CFG-SBAS
const unsigned char urCfg16[] = {0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};

#define UBX_NO_OF_MSG_CONFIGS 8
//don't change the last two entries! MON-SCHED is not working with the UB6 chipset and the lest entry activates the raw output on T chipsets. If packages are added, add them in the beginning!
const unsigned char urCfgMsgList[UBX_NO_OF_MSG_CONFIGS][3] = { {0x01, 0x02, 1}, {0x01, 0x03, 1}, {0x01, 0x06, 1},
                                                               {0x01, 0x12, 1}, {0x01, 0x30, 1}, {0x0A, 0x09, 1},
                                                               {0x0A, 0x01, 1}, {0x02, 0x10, 1}};

const unsigned char urCfgNav2[] = {0x05, 0x00, 0x00, 0x00, 0x04, 0x03, 0x0A, 0x02, 0x50, 0xc3, 0x00, 0x00, 0x0f, 0x0a,
                                   0x0a, 0x3c, 0x00, 0x01, 0x00, 0x00, 0xfa, 0x00, 0xfa, 0x00, 0x64, 0x00, 0x2c, 0x01,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const unsigned char urCfgNav5[] = {0xFF, 0xFF, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x0A, 0x00,
                                   0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*const unsigned char urCfgNav5X[]={0x00, 0x00, 0x4C, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x03, 0x0A, 0x0A, 0x00, 0x01, 0x00, 0x00, 0x00, 0xF8, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};*///Firmware 7.00

const unsigned char urCfgNav5X[] = {0x00, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x03, 0x02, 0x03, 0x0A, 0x0A, 0x00,
                                    0x01, 0x01, 0x00, 0x00, 0xF8, 0x05, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,
                                    0x00, 0x64, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //Firmware 7.03


unsigned short urMsgCnt = 0;

#define UBX_UNKNOWN  0
#define UBX_VER_ANT4 1
#define UBX_VER_UB6  2
unsigned char ubloxVersion = UBX_UNKNOWN;

// prototype functions
void uBloxSendMessage(unsigned char urClass, unsigned char urId, unsigned char * urData, unsigned char urLength);
void pollMessageRate(unsigned char urClass, unsigned char urId);
void setMessageRate(unsigned char urClass, unsigned char urId, unsigned char rate);
void uBloxResetConfiguration(void);
void uBloxHandleMessage(unsigned char urClass, unsigned char urId, unsigned char * urData, unsigned short urLength);

void uBloxSendMessage(unsigned char urClass, unsigned char urId, unsigned char * urData, unsigned char urLength)
{
  unsigned char ckA, ckB;
  unsigned char header[2];

  header[0] = 0xB5;
  header[1] = 0x62;

  ckA = 0;
  ckB = 0;

  ckA += urClass;
  ckB += ckA;

  ckA += urId;
  ckB += ckA;

  ckA += urLength & 0xff;
  ckB += ckA;

  ckA += urLength >> 8;
  ckB += ckA;

  for (int i = 0; i < urLength; i++)
  {
    ckA += urData[i];
    ckB += ckA;
  }

#ifdef FALCON
  UART_send((unsigned char *)&header,2);
  UART_send((unsigned char *)&urClass,1);
  UART_send((unsigned char *)&urId,1);

  unsigned char c=urLength;
  UART_send(&c,1);

  c=urLength>>8;
  UART_send(&c,1);

  UART_send(urData,urLength);
  UART_send(&ckA,1);
  UART_send(&ckB,1);

  //wait until all data is sent out
  while ((U0LSR&(1<<6))==0);
#else
  UART1_send((unsigned char *)&header, 2);
  UART1_send((unsigned char *)&urClass, 1);
  UART1_send((unsigned char *)&urId, 1);

  unsigned char c = urLength;
  UART1_send(&c, 1);

  c = urLength >> 8;
  UART1_send(&c, 1);

  UART1_send(urData, urLength);
  UART1_send(&ckA, 1);
  UART1_send(&ckB, 1);

  //wait until all data is sent out
  while ((U1LSR & (1 << 6)) == 0)
    ;

#endif

}

void pollMessageRate(unsigned char urClass, unsigned char urId)
{
  unsigned char data[2];

  data[0] = urClass;
  data[1] = urId;
  uBloxSendMessage(0x06, 0x01, (unsigned char *)&data, 2);

}

void setMessageRate(unsigned char urClass, unsigned char urId, unsigned char rate)
{
  unsigned char data[3];

  data[0] = urClass;
  data[1] = urId;
  data[2] = rate;
  uBloxSendMessage(0x06, 0x01, (unsigned char *)&data, 3);
}

void uBloxResetConfiguration(void)
{
  struct UBX_CFG_CFG ubxCfg;

  ubxCfg.clearMask = 0xffff; //reset complete config except port config
  ubxCfg.loadMask = 0xffff;
  ubxCfg.saveMask = 0;
  ubxCfg.deviceMask = 0x07;

  uBloxSendMessage(0x06, 0x09, (unsigned char *)&ubxCfg, sizeof(struct UBX_CFG_CFG));

}

void uBloxHandleMessage(unsigned char urClass, unsigned char urId, unsigned char * urData, unsigned short urLength)
{
  struct UBX_NAV_POSLLH * navPosLLH;
  struct UBX_NAV_STATUS * navStatus;
  struct UBX_NAV_SOL * navSol;
  struct UBX_NAV_VELNED * navVelNed;
//  struct UBX_NAV_SVINFO_HEADER * navSVInfoHeader;
//  struct UBX_MON_HW * monHw;
//  struct UBX_MON_SCHED * monSched;

  static int sacc_filter = 0;

  urMsgCnt++;

  switch (urClass)
  {
    case 0x01:
      switch (urId)
      {
        case 0x02: //NAV-POSLLH

          navPosLLH = (struct UBX_NAV_POSLLH *)urData;
          GPS_Data.latitude = navPosLLH->lat;
          GPS_Data.longitude = navPosLLH->lon;
          GPS_Data.height = navPosLLH->height;
          GPS_Data.horizontal_accuracy = navPosLLH->hAcc;
          GPS_Data.vertical_accuracy = navPosLLH->vAcc;
          break;

        case 0x03: //NAV-STATUS
          navStatus = (struct UBX_NAV_STATUS *)urData;
          GPS_Data.status = (navStatus->flags << 8) | (navStatus->fixStat << 16) | navStatus->gpsFix;
          break;

        case 0x06: //NAV-SOL
          navSol = (struct UBX_NAV_SOL *)urData;
          GPS_Time.week = navSol->week;
          GPS_Time.time_of_week = navSol->iTow;
          GPS_Data.numSV = navSol->numSV;
          break;

        case 0x12: //NAV-VELNED
          navVelNed = (struct UBX_NAV_VELNED *)urData;
          GPS_Data.speed_x = navVelNed->velE * 10;
          GPS_Data.speed_y = navVelNed->velN * 10;
          GPS_Data.speed_accuracy = navVelNed->sAcc * 10;
          GPS_Data.heading = navVelNed->heading / 100;
          sacc_filter = (2 * sacc_filter + navVelNed->sAcc) / 3;
          if ((GPS_Data.horizontal_accuracy > 15000) || (sacc_filter > 230))
            GPS_Data.status &= ~0x03;
          gpsDataOkTrigger = 1;

          break;

//        case 0x30:
//          navSVInfoHeader = (struct UBX_NAV_SVINFO_HEADER *)urData;
//          if (navSVInfoHeader->numCh > 16)
//            break;
//          memcpy(&satInfo, &urData[sizeof(struct UBX_NAV_SVINFO_HEADER)], navSVInfoHeader->numCh
//              * sizeof(struct UBX_NAV_SVINFO_SAT));
//
//          break;
      }
      break;

    case 0x05: //ACK
      urAckClass = urData[0];
      urAckId = urData[1];

      if ((urVersionCheck) && (urAckClass == 0x06) && (urAckId == 0x24))
      {

        urVersionCheck = 0;
        if (urId == 0x01)
        {
          ubloxVersion = UBX_VER_UB6;
          urCfgIdList = (unsigned char *)&urCfgIdListUB6;
        }
        else
        {
          ubloxVersion = UBX_VER_ANT4;
          urCfgIdList = (unsigned char *)&urCfgIdListAnt4;
        }
      }
      else
      {
        switch (urId)
        {
          case 0x01:
            urAckReceived = 1;
            break;
        }
      }

      break;

//    case 0x0A: //MON
//      switch (urId)
//      {
//        case 0x01: //MON-SCHED
//          monSched = (struct UBX_MON_SCHED *)urData;
//          gpsHardwareStatus.stackUsed = monSched->stackUsed;
//          gpsHardwareStatus.stackAv = monSched->stackAv;
//          gpsHardwareStatus.cpuLoad = monSched->cpuLoad;
//          gpsHardwareStatus.fullSlots = monSched->fullSlots;
//          gpsHardwareStatus.partSlots = monSched->partSlots;
//          break;
//        case 0x09: //MON-HW
//          monHw = (struct UBX_MON_HW *)urData;
//          gpsHardwareStatus.antennaStatus = monHw->aStatus;
//          gpsHardwareStatus.antennaPower = monHw->aPower;
//          gpsHardwareStatus.agcMonitor = monHw->agcCnt;
//          gpsHardwareStatus.noiseLevel = monHw->noisePerMs;
//          break;
//      }
      break;

  }
}

void uBloxReceiveHandler(unsigned char recByte)
{
#define URS_SYNC1 0
#define URS_SYNC2 1
#define URS_CLASS 2
#define URS_ID    3
#define URS_LENGTH1 4
#define URS_LENGTH2 5
#define URS_DATA    6
#define URS_CKA     7
#define URS_CKB     8
#define URS_RAWDATA 9

#define UR_MAX_DATA_LENGTH 256

  static unsigned char urState = URS_SYNC1;
  static unsigned char urClass;
  static unsigned char urId;
  static unsigned short urLength;
  static unsigned short urCnt = 0;
  static unsigned char urData[UR_MAX_DATA_LENGTH];
  static unsigned char urCkARec, urCkBRec;
  static unsigned char urCkA, urCkB;

  switch (urState)
  {
    case URS_SYNC1:
      if (recByte == 0xB5)
        urState = URS_SYNC2;
      break;
    case URS_SYNC2:
      if (recByte == 0x62)
      {
        urState = URS_CLASS;
        urCkA = 0;
        urCkB = 0;
      }
      else
        urState = URS_SYNC1;
      break;
    case URS_CLASS:
      urClass = recByte;
      urState = URS_ID;

      //update chkSum
      urCkA += recByte;
      urCkB += urCkA;
      break;
    case URS_ID:
      urId = recByte;
      urState = URS_LENGTH1;
      //update chkSum
      urCkA += recByte;
      urCkB += urCkA;
      break;
    case URS_LENGTH1:
      urLength = recByte;
      urState = URS_LENGTH2;
      //update chkSum
      urCkA += recByte;
      urCkB += urCkA;
      break;
    case URS_LENGTH2:
      urLength |= recByte << 8;
      urCnt = 0;
      //update chkSum
      urCkA += recByte;
      urCkB += urCkA;
      if ((urClass == 0x02) && (urId == 0x10))
      {
        if (urLength + 8 > UR_MAX_RAWDATA_LENGTH)
          urState = URS_SYNC1;
        else
          urState = URS_RAWDATA;
      }
      else
      {
        if (urLength > UR_MAX_DATA_LENGTH)
          urState = URS_SYNC1;
        else
          urState = URS_DATA;
      }
      break;
    case URS_DATA:
      //update chkSum
      urCkA += recByte;
      urCkB += urCkA;

      if (urCnt >= UR_MAX_DATA_LENGTH)
      {
        urState = URS_SYNC1;
        return;
      }

      urData[urCnt++] = recByte;

      if (urCnt == urLength)
        urState = URS_CKA;
      break;
    case URS_CKA:
      urCkARec = recByte;
      urState = URS_CKB;
      break;
    case URS_CKB:
      urCkBRec = recByte;
      if ((urCkA == urCkARec) && (urCkB == urCkBRec))
      {
        uBloxHandleMessage(urClass, urId, &urData[0], urLength);
      }
      urState = URS_SYNC1;
      break;

    case URS_RAWDATA:
      if (6 + urCnt >= UR_MAX_RAWDATA_LENGTH)
      {
        urCnt = 0;
        urState = URS_SYNC1;
        return;
      }
      GPS_RawBuffer[6 + urCnt] = recByte;
      urCnt++;
      if (urCnt == urLength + 2)
      {
        GPS_RawBuffer[0] = 0xB5;
        GPS_RawBuffer[1] = 0x62;
        GPS_RawBuffer[2] = 0x02;
        GPS_RawBuffer[3] = 0x10;
        GPS_RawBuffer[4] = urLength & 0xff;
        GPS_RawBuffer[5] = urLength >> 8;

        GPS_raw_data_size = 8 + urLength;
        GPS_trigger_raw_logging = 1;
        urState = URS_SYNC1;
      }

      break;
    default:
      urState = URS_SYNC1;

      break;
  }
}

void uBloxReceiveEngine(void)
{
#define URES_IDLE 0
#define URES_RESET_UBLOX_CONFIG 1
#define URES_WAIT_FOR_DATA 2
#define URES_RESETCONFIG 3
#define URES_ACK_CFG 4
#define URES_ACK_MSG 5
#define URES_CONFIG_DONE 6

#define UR_MAX_RETRYS 80
#define MAX_BR_CNT 	  2

  static unsigned char urEngineState = URES_IDLE;
  static unsigned char urTimeOut;
  static unsigned char urReconfigurationRetries = UR_MAX_RETRYS;
  static unsigned short currentBaudrate = 57600;
  const unsigned short baudrateList[MAX_BR_CNT] = {9600, 57600};
  static unsigned char baudrateCnt = 0;
  int i;

  urTimeCnt++;

#ifndef FALCON
  if (urTimeCnt % 10)
    return;
#endif

  if (!urReconfigurationRetries)
  {
    //gps configuration failed completly. Dont' do anything again
    // 	 	 	 LED(1,OFF);
    return;
  }

  switch (urEngineState)
  {
    case URES_IDLE:

      //init all vars
      baudrateCnt = 0;
      currentBaudrate = baudrateList[baudrateCnt];

      urMsgCnt = 0;
      urCfgIdList = (unsigned char *)&urCfgIdListAnt4;

      urEngineState = URES_RESET_UBLOX_CONFIG;
      //          LED(1,OFF);

      break;
    case URES_RESET_UBLOX_CONFIG:
      //cycle through baudrates and reset config

#ifdef FALCON
      //wait until all data is sent out
      while ((U0LSR&(1<<6))==0);
      UARTInitialize(currentBaudrate);
      //clear RX and TX fifos
      U0FCR=0x03;
#else
      //wait until all data is sent out
      while ((U1LSR & (1 << 6)) == 0)
        ;
      UART1Initialize(currentBaudrate);
      //clear RX and TX fifos
      U1FCR = 0x03;
#endif

      //send reset command three times
      uBloxResetConfiguration();
      uBloxResetConfiguration();
      uBloxResetConfiguration();

      //get next baudrate
      baudrateCnt++;
      if (baudrateCnt < MAX_BR_CNT)
        currentBaudrate = baudrateList[baudrateCnt];
      else
      {
        currentBaudrate = 9600;
#ifdef FALCON
        //wait until all data is sent out
        while ((U0LSR&(1<<6))==0);
        UARTInitialize(currentBaudrate);
        //clear RX and TX fifos
        U0FCR=0x03;
#else
        //wait until all data is sent out
        while ((U1LSR & (1 << 6)) == 0)
          ;
        UART1Initialize(currentBaudrate);
        //clear RX and TX fifos
        U1FCR = 0x03;
#endif

        urEngineState = URES_WAIT_FOR_DATA;
        //poll NAV5 to check if the GPS is a newer or older uBlox
        urVersionCheck = 1;
        uBloxSendMessage(0x6, 0x24, (unsigned char *)0, 0);
        urTimeOut = 10;
      }

      break;
    case URES_WAIT_FOR_DATA:
      if (urVersionCheck == 0)
      {
        //see if baudrate is correct. Otherwise transmit port setting and start over
        if (currentBaudrate == 9600)
        {
          for (i = 0; i < 3; i++)
          {
            if (ubloxVersion == UBX_VER_ANT4)
              uBloxSendMessage(0x6, urCfgIdList[urConfigCnt], (unsigned char *)&urCfg00Ant4[0], sizeof(urCfg00Ant4));
            else
              uBloxSendMessage(0x6, urCfgIdList[urConfigCnt], (unsigned char *)&urCfg00UB6[0], sizeof(urCfg00UB6));

          }
          currentBaudrate = 57600;
#ifdef FALCON
          //wait until all data is sent out
          while ((U0LSR&(1<<6))==0);
          UARTInitialize(currentBaudrate);
          //clear RX and TX fifos
          U0FCR=0x03;
#else
          //wait until all data is sent out
          while ((U1LSR & (1 << 6)) == 0)
            ;
          UART1Initialize(currentBaudrate);
          //clear RX and TX fifos
          U1FCR = 0x03;
#endif
        }

        urEngineState = URES_ACK_CFG;
        urAckReceived = 0;
        urConfigCnt = 1;
        urConfigMessageReceived = 0;
        urTimeOut = 10;
        //set first packet
        uBloxSendMessage(0x6, urCfgIdList[urConfigCnt], (unsigned char *)&urCfg08, sizeof(urCfg08));
      }
      else
      {
        if (urTimeOut)
          urTimeOut--;
        else
        {
          urReconfigurationRetries--;

          urMsgCnt = 0;
          urEngineState = URES_WAIT_FOR_DATA;
          urVersionCheck = 1;
          uBloxSendMessage(0x6, 0x24, (unsigned char *)0, 0);
          urTimeOut = 10;
        }
      }
      break;
    case URES_ACK_CFG:
      if ((urAckReceived) && (urAckClass == 0x06) && (urAckId == urCfgIdList[urConfigCnt]))
      {
        urAckReceived = 0;
        urReconfigurationRetries = UR_MAX_RETRYS;
        urConfigCnt++;
        if (((ubloxVersion == UBX_VER_UB6) && (urConfigCnt == MAX_COMMON_CFG_PACKETS_UB6)) || ((ubloxVersion
            == UBX_VER_ANT4) && (urConfigCnt == MAX_COMMON_CFG_PACKETS_ANT4)))
        {
          urEngineState = URES_ACK_MSG;
          urConfigCnt = 0;
          urTimeOut = 10;
          setMessageRate(urCfgMsgList[urConfigCnt][0], urCfgMsgList[urConfigCnt][1], urCfgMsgList[urConfigCnt][2]);
        }
        else
        {
          urTimeOut = 10;
          if (urConfigCnt == 1)
            uBloxSendMessage(0x6, urCfgIdList[urConfigCnt], (unsigned char *)&urCfg08, sizeof(urCfg08));
          else if (urConfigCnt == 2)
            uBloxSendMessage(0x6, urCfgIdList[urConfigCnt], (unsigned char *)&urCfg13, sizeof(urCfg13));
          else if (urConfigCnt == 3)
            uBloxSendMessage(0x6, urCfgIdList[urConfigCnt], (unsigned char *)&urCfg16, sizeof(urCfg16));
          else if (urCfgIdList[urConfigCnt] == 0x23)
            uBloxSendMessage(0x6, 0x23, (unsigned char *)&urCfgNav5X, sizeof(urCfgNav5X));
          else if (urCfgIdList[urConfigCnt] == 0x24)
            uBloxSendMessage(0x6, 0x24, (unsigned char *)&urCfgNav5, sizeof(urCfgNav5));
          else if (urCfgIdList[urConfigCnt] == 0x1A)
            uBloxSendMessage(0x6, 0x1A, (unsigned char *)&urCfgNav2, sizeof(urCfgNav2));
        }

      }
      else
      {
        if (urTimeOut)
          urTimeOut--;
        else
        {
          urReconfigurationRetries--;
          urAckReceived = 0;
          urTimeOut = 10;
          if (urConfigCnt == 1)
            uBloxSendMessage(0x6, urCfgIdList[urConfigCnt], (unsigned char *)&urCfg08, sizeof(urCfg08));
          else if (urConfigCnt == 2)
            uBloxSendMessage(0x6, urCfgIdList[urConfigCnt], (unsigned char *)&urCfg13, sizeof(urCfg13));
          else if (urConfigCnt == 3)
            uBloxSendMessage(0x6, urCfgIdList[urConfigCnt], (unsigned char *)&urCfg16, sizeof(urCfg16));
          else if (urCfgIdList[urConfigCnt] == 0x23)
            uBloxSendMessage(0x6, 0x23, (unsigned char *)&urCfgNav5X, sizeof(urCfgNav5X));
          else if (urCfgIdList[urConfigCnt] == 0x24)
            uBloxSendMessage(0x6, 0x24, (unsigned char *)&urCfgNav5, sizeof(urCfgNav5));
          else if (urCfgIdList[urConfigCnt] == 0x1A)
            uBloxSendMessage(0x6, 0x1A, (unsigned char *)&urCfgNav2, sizeof(urCfgNav2));

        }
      }

      break;

    case URES_ACK_MSG:
      if (urAckReceived)
      {
        urAckReceived = 0;
        urConfigCnt++;
        urReconfigurationRetries = UR_MAX_RETRYS;
        if ((ubloxVersion == UBX_VER_UB6) && (urConfigCnt == UBX_NO_OF_MSG_CONFIGS - 2))
          urConfigCnt++;
#ifndef GPS_RAWDATA_PROCESSING
        if ((urConfigCnt == UBX_NO_OF_MSG_CONFIGS - 1))
          urConfigCnt++;
#endif

        if (urConfigCnt == UBX_NO_OF_MSG_CONFIGS)
        {
          urEngineState = URES_CONFIG_DONE;
        }
        else
        {
          urEngineState = URES_ACK_MSG;
          urConfigMessageReceived = 0;
          urTimeOut = 10;
          setMessageRate(urCfgMsgList[urConfigCnt][0], urCfgMsgList[urConfigCnt][1], urCfgMsgList[urConfigCnt][2]);
        }
      }
      else
      {

        if (urTimeOut)
          urTimeOut--;
        else
        {
          urReconfigurationRetries--;
          setMessageRate(urCfgMsgList[urConfigCnt][0], urCfgMsgList[urConfigCnt][1], urCfgMsgList[urConfigCnt][2]);
          urTimeOut = 10;
          urAckReceived = 0;
          urEngineState = URES_ACK_MSG;
        }
      }

      break;

    case URES_CONFIG_DONE:
      //  	 LED(1,ON);

      //
      break;
  }

}
