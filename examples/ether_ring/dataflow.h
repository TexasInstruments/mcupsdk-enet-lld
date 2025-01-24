/*
 *  Copyright (c) Texas Instruments Incorporated 2024
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _DATAFLOW_H_
#define _DATAFLOW_H_

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include <enet.h>
#include <kernel/dpl/ClockP.h>
#include "FreeRTOS.h"
#include <kernel/dpl/TaskP.h>
#include <task.h>
#include <enet_apputils.h>
#include <enet_appmemutils.h>
#include <tsn_combase/tilld/cb_lld_ethernet.h>
#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"
#include "ti_enet_config.h"
#include "ti_enet_open_close.h"
#include "tsnapp_porting.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Task stack size */
#define ENETAPP_TASK_STACK_SZ                     (10U * 1024U)

#define ETH_P_IPV4 (0x0800U)

#define PTP_ENABLED

/* ========================================================================== */
/*                                Function Declarations                       */
/* ========================================================================== */
int EnetApp_lldCfgUpdateCb(cb_socket_lldcfg_update_t *update_cfg);
void rxDefaultDataCb(void *data, int size, int port, void *cbArg);

int32_t EnetApp_open();
int32_t EnetApp_createTxRetrievePollTask();
void EnetApp_destroyRxTask();
void EnetApp_createRxTask();
void EnetApp_createEtherRingClearTask();
void EnetApp_createStreamTask();
void EnetApp_startHwTimer();

#endif //_DATAFLOW_H_
