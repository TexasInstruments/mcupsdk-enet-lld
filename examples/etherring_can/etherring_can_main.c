/*
 *  Copyright (C) Texas Instruments Incorporated 2025
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
 * \file  etherring_can_main.c
 *
 * \brief This file contains the implementation of the Ethernet CAN Application
 *        entry point
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include "ethernet_config/ethernet_config.h"
#include "etherring_can_cfg.h"
#include "etherring_can_app.h"
/* ========================================================================== */
/*                              Global Variables                              */
/* ========================================================================== */
EnetApp_Cfg gEnetAppCfg;

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
void EnetApp_mainTask(void *args)
{
    EnetPer_AttachCoreOutArgs attachCoreOutArgs;
    EnetApp_HandleInfo handleInfo;

    Drivers_open();
    Board_driversOpen();
    DebugP_log("==========================\r\n");
    DebugP_log("     EtherRing CAN App    \r\n");
    DebugP_log("==========================\r\n");

    EnetApp_configureNodeId(&gEnetAppCfg.nodeId);

    EnetApp_initAppCfg(&attachCoreOutArgs, &handleInfo, &gEnetAppCfg);

    EnetApp_initApp(&gEnetAppCfg);

    EnetApp_startTraffic(&gEnetAppCfg);

    while (true)
    {
        /* Print CPU load periodically is disabled as performance is degraded */
        /* EnetApp_printCpuLoad(); */
        ClockP_usleep(30000);
        TaskP_yield();
    }
}


