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

#ifndef HL_MATLABCTRL_H_
#define HL_MATLABCTRL_H_

#include <inttypes.h>

typedef struct __attribute__((packed))
{
	uint64_t timestamp;
	short debug01;
	short debug02;
	short debug03;
	short debug04;
	short debug05;
	short debug06;
	short debug07;
	short debug08;
	short debug09;
	short debug10;
	short debug11;
	short debug12;
	short debug13;
	short debug14;
	short debug15;
	short debug16;
	short debug17;
	short debug18;
	short debug19;
	short debug20;
	short debug21;
	short debug22;
	short debug23;
	short debug24;
	short debug25;
	short debug26;
	short debug27;
	short debug28;
	short debug29;
	short debug30;
	short debug31;
	short debug32;
	short debug33;
	short debug34;
	short debug35;
	short debug36;
	short debug37;
	short debug38;
	short debug39;
}HLI_SSDK_DEBUG;

//
typedef struct __attribute__((packed))
{
	unsigned int p01;
	unsigned int p02;
	unsigned int p03;
	unsigned int p04;
	unsigned int p05;
	unsigned int p06;
	unsigned int p07;
	unsigned int p08;
	unsigned int p09;
	unsigned int p10;
	unsigned int p11;
	unsigned int p12;
	unsigned int p13;
	unsigned int p14;
	unsigned int p15;
	unsigned int p16;
	unsigned int p17;
	unsigned int p18;
	unsigned int p19;
	unsigned int p20;
	unsigned int p21;
	unsigned int p22;
	unsigned int p23;
	unsigned int p24;
	unsigned int p25;
	unsigned int p26;
	unsigned int p27;
	unsigned int p28;
	unsigned int p29;
	unsigned int p30;
	unsigned int p31;
	unsigned int p32;
	unsigned int p33;
	unsigned int p34;
	unsigned int p35;
	unsigned int p36;
	unsigned int p37;
	unsigned int p38;
	unsigned int p39;
	unsigned int p40;
	unsigned int p41;
	unsigned int p42;
	unsigned int p43;
	unsigned int p44;
	unsigned int p45;
	unsigned int p46;
	unsigned int p47;
	unsigned int p48;
	unsigned int p49;
	unsigned int p50;
	unsigned int crc;
}HLI_SSDK_PARAMS;

typedef struct __attribute__((packed))
{
	short ctrl01;
	short ctrl02;
	short ctrl03;
	short ctrl04;
	short ctrl05;
	short ctrl06;
	short ctrl07;
	short ctrl08;
	short ctrl09;
	short ctrl10;
	unsigned short crc;
}HLI_SSDK_UART;

#endif /* HL_MATLABCTRL_H_ */
