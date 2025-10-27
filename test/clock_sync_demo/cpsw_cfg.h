/*
 *  Copyright (c) Texas Instruments Incorporated 2025
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

/*!
 * \file
 *
 * \brief
 */

#ifndef CLOCK_SYNC_CPSW_H
#define CLOCK_SYNC_CPSW_H

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <assert.h>
#include <enet_apputils.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define ENETAPP_DEFAULT_CFG_NAME    "sitara-cpsw"

// timer 0 base addresses are same for am243, am62d and am275
#define TIMER_BASEADDR          (0x2400000u)
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct EnetApp_Cfg_s
{
    /* Peripheral type */
      Enet_Type enetType;

      /* Peripheral instance */
      uint32_t instId;

      char *name;

      /* This core's id */
      uint32_t coreId;

      /* Core key returned by Enet RM after attaching this core */
      uint32_t coreKey;

      /* Enet driver handle for this peripheral type/instance */
      Enet_Handle hEnet;

      Enet_MacPort macPort;

} EnetApp_Cfg;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

uint64_t EnetApp_getHwPushEventTimestamp(void);

uint64_t EnetApp_getCurrentTimestamp(void);

void EnetApp_updateCpswInitCfg(Enet_Type enetType,  uint32_t instId, Cpsw_Cfg *cpswCfg);

void EnetApp_setCPTStime(int32_t nudge);

void EnetApp_registerCallback(void * cb, void* ClockSync_Handle ,Enet_Handle hEnet, uint32_t coreId);

int32_t EnetApp_setTimeSyncRouter(void);


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

#endif