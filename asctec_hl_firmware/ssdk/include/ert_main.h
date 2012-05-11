/*
 * ert_main.h
 *
 *  Created on: Jun 18, 2010
 *      Author: acmarkus
 */

#ifndef ERT_MAIN_H_
#define ERT_MAIN_H_

#include "HL_SSDK.h"

void rt_OneStep(void);

extern void onboard_matlab_initialize(void);

extern short matlab_switch_debug_packet;
extern HLI_SSDK_DEBUG ssdk_debug;
extern HLI_SSDK_PARAMS ssdk_params, ssdk_params_tmp;
extern HLI_SSDK_UART ssdk_uart, ssdk_uart_tmp;

#endif /* ERT_MAIN_H_ */
