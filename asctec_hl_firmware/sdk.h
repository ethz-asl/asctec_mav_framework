/*

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


#ifndef SDK_
#define SDK_

extern void SDK_mainloop(void);

#include <HL_interface.h>

#define CMD_MAX_PERIOD 100 // in [ms] ==> 10Hz

/// initializes the sdk
void sdkInit(void);

/// Writes command to the LLP
inline void writeCommand(short pitch, short roll, short yaw, short thrust, short ctrl, short enable);

/// assembles IMU packet and sends it
inline void sendImuData(void);

/// assembles GPS packet and sends it
inline void sendGpsData(void);

/// assembles status packet and sends it
inline void sendStatus(void);

/// assembles rc packet and sends it
inline void sendRcData(void);

/// adjusts HLP time to host PC time
/***
 * Timestamped packets get send around every 2 s to average the transmission delay.
 * Corrects at max 500 us per second. If the time offset is large, the server (host PC)
 * time is taken directly and synchronization starts from that time.
 */
inline void synchronizeTime(void);

/// gets called every sdk loops. Currently, only checks for packets from the PC and starts autobaud in case there wwas no communication in the last 10 s
inline void watchdog(void);

/// checks if a packet has to be sent
inline int checkTxPeriod(uint16_t period);

/// counts ssdk loops
extern unsigned int sdkLoops;

extern HLI_EXT_POSITION extPosition;
extern HLI_CMD_HL extPositionCmd;
extern HLI_STATUS statusData;

/// current time. Gets incremented by timer0 and gets corrected by synchronizeTime().
extern volatile int64_t timestamp;

// ----------------------------------


struct WO_SDK_STRUCT
{

  unsigned char ctrl_mode;
  //0x00: "standard scientific interface" => send R/C stick commands to LL
  //0x01:	direct motor control
  //0x02: waypoint control (not yet implemented)

  unsigned char ctrl_enabled; //0x00: Control commands are ignored by LL processor
  //0x01: Control commands are accepted by LL processor

};
extern struct WO_SDK_STRUCT WO_SDK;

struct RO_RC_DATA
{

  unsigned short channel[8];
  /*
   * channel[0]: Pitch
   * channel[1]: Roll
   * channel[2]: Thrust
   * channel[3]: Yaw
   * channel[4]: Serial interface enable/disable
   * channel[5]: manual / height control / GPS + height control
   *
   * range of each channel: 0..4095
   */
};
extern struct RO_RC_DATA RO_RC_Data;

struct WO_DIRECT_MOTOR_CONTROL
{
  unsigned char pitch;
  unsigned char roll;
  unsigned char yaw;
  unsigned char thrust;

  /*
   * commands will be directly interpreted by the mixer
   * running on each of the motor controllers
   *
   * range (pitch, roll, yaw commands): 0..200 = - 100..+100 %
   * range of thrust command: 0..200 = 0..100 %
   */

};
extern struct WO_DIRECT_MOTOR_CONTROL WO_Direct_Motor_Control;

struct WO_CTRL_INPUT
{ //serial commands (= Scientific Interface)
  short pitch; //Pitch input: -2047..+2047 (0=neutral)
  short roll; //Roll input: -2047..+2047	(0=neutral)
  short yaw; //(=R/C Stick input) -2047..+2047 (0=neutral)
  short thrust; //Collective: 0..4095 = 0..100%
  short ctrl; /*control byte:
   bit 0: pitch control enabled
   bit 1: roll control enabled
   bit 2: yaw control enabled
   bit 3: thrust control enabled
   bit 4: Height control enabled
   bit 5: GPS position control enabled
   */
};
extern struct WO_CTRL_INPUT WO_CTRL_Input;

#endif /*SDK_*/
