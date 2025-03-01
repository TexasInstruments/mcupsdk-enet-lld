/*
 *  Copyright (C) Texas Instruments Incorporated 2024
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
 * \file  tsnapp_cpsw_main.c
 *
 * \brief This file contains the implementation of the Enet TSN example entry
 *        point
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "debug_log.h"
#include "enetapp_cpsw.h"
#include "dataflow.h"
#include "tsninit.h"

/* ========================================================================== */
/*                              Global Variables                              */
/* ========================================================================== */
extern EnetApp_Cfg gEnetAppCfg;
EnetDma_Handle ghEnetDma;

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
void configureNodeId();
void EnetApp_mainTask(void *args);
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
    DebugP_log("       EtherRing App      \r\n");
    DebugP_log("==========================\r\n");

    configureNodeId();

    EnetApp_initAppCfg(&attachCoreOutArgs, &handleInfo);

    EnetAppUtils_print("%s: Create RX task for regular traffic \r\n", ENETAPP_DEFAULT_CFG_NAME);

    EnetApp_open();

    ghEnetDma = Enet_getDmaHandle(gEnetAppCfg.hEnet);

    /* Creates Rx Task for Redundancy packets received on DMA channel 0 */
    EnetApp_createRxTask();

    /* Task to clear Ether-Ring Look-up table entries periodically */
    EnetApp_createEtherRingClearTask();

    /* Starts Hardware Timer for periodic callback for every 125us */
    EnetApp_startHwTimer();

    /* Initialize TSN and EST Configuration */
    if (EnetApp_initTsn())
    {
        DebugP_log("EnetApp_initTsn failed\r\n");
    }
    else
    {
        /* Update PortMask for Ptp Mcast ALE Entry */
        EnetApp_updatePtpMcastAddress(gEnetAppCfg.hEnet, gEnetAppCfg.coreId);

        /* Create Real-time task for traffic generation */
        EnetApp_createStreamTask();

        while (true)
        {
            /* Print CPU load periodically is disabled as performance is degraded */
            /* EnetApp_printCpuLoad(); */
            ClockP_usleep(30000);
            TaskP_yield();
        }
        EnetApp_stopTsn();
        EnetApp_deInitTsn();
    }
}

void configureNodeId()
{
    while (true)
    {
        EnetAppUtils_print("0 - Central Compute Node\r\n");
        EnetAppUtils_print("1 - Zone Left Node\r\n");
        EnetAppUtils_print("2 - Zone Right Node\r\n");
        EnetAppUtils_print("3 - Zone Tail Node\r\n");
        EnetAppUtils_print("Enter the nodeId : \r\n");

        DebugP_scanf("%d", &gEnetAppCfg.nodeId);
        if ((gEnetAppCfg.nodeId < 0) || (gEnetAppCfg.nodeId > 3))
        {
            EnetAppUtils_print("Enter a valid number for nodeId\r\n");
            continue;
        }
        break;
    }
}
