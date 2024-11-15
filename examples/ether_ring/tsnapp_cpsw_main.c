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
extern EnetApp_Cfg gEnetAppCfg;
EnetDma_Handle ghEnetDma;

//volatile static uint32_t gTIMERaRRAY[100];
/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
void configureNodeTxRx();
void configureTrafficGenerator();
void configureNodeId();
void configureIntervlan();
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
#if 0
    gEnetAppCfg.isTxEnabled = false;
    gEnetAppCfg.isIntervlanEnabled = false;
    configureNodeTxRx();
#else
    gEnetAppCfg.isTxEnabled = true;
#endif
    if (gEnetAppCfg.isTxEnabled)
    {
        gEnetAppCfg.numClassAStreams = 1;
        gEnetAppCfg.numClassDStreams = 1;
        gEnetAppCfg.packetCount = 10;
        // configureTrafficGenerator();
    }

    EnetApp_initAppCfg(&attachCoreOutArgs, &handleInfo);

    EnetAppUtils_print("%s: Create RX task for regular traffic \r\n", ENETAPP_DEFAULT_CFG_NAME);

    EnetApp_open();

    ghEnetDma = Enet_getDmaHandle(gEnetAppCfg.hEnet);

    EnetApp_createRxTask();
    EnetApp_createEtherRingClearTask();

    EnetApp_startHwTimer();
//     EnetApp_createTxRetrievePollTask();

    if (EnetApp_initTsn())
    {
        DebugP_log("EnetApp_initTsn failed\r\n");
    }
    else
    {
        if (gEnetAppCfg.isTxEnabled)
        {
           EnetApp_createStreamTask();
        }

        while (true)
        {
            // Print CPU load
            ClockP_usleep(30000);
//            ClockP_usleep(5000);
//            EnetApp_printCpuLoad();
            TaskP_yield();
        }
        EnetApp_stopTsn();
        EnetApp_deInitTsn();
    }
}

void configureNodeTxRx()
{
    char option = ' ';
    while(true)
    {
        EnetAppUtils_print("\r\nCPSW ETHERRING Test Menu:\r\n");
        EnetAppUtils_print(" 't' - Configure node as TX \r\n");
        EnetAppUtils_print(" 'r' - Configure node as RX \r\n");
        DebugP_scanf("%c", &option);
        if (option == 't')
        {
            gEnetAppCfg.isTxEnabled = true;
            EnetAppUtils_print("Configured as TX\r\n");
        }
        else if (option == 'r')
        {
            EnetAppUtils_print("Configured as RX\r\n");
        }
        else
        {
            EnetAppUtils_print("Enter valid data\r\n");
            EnetAppUtils_print(" 't' for TX and 'r' for RX \r\n");
            continue;
        }
        break;
    }
}

void configureTrafficGenerator()
{
    while (true)
    {
        EnetAppUtils_print("Max ClassA Streams Supported : %d\r\n", MAX_CLASSA_STREAMS);
        EnetAppUtils_print("Max ClassD Streams Supported : %d\r\n", MAX_CLASSD_STREAMS);
        EnetAppUtils_print("Enter the number of class A Streams : \r\n");

        DebugP_scanf("%d", &gEnetAppCfg.numClassAStreams);
        if ((gEnetAppCfg.numClassAStreams < 0) || (gEnetAppCfg.numClassAStreams > MAX_CLASSA_STREAMS))
        {
            EnetAppUtils_print("Enter a valid number for ClassA stream\r\n");
            continue;
        }

        EnetAppUtils_print("Enter the number of class D Streams : \r\n");
        DebugP_scanf("%d", &gEnetAppCfg.numClassDStreams);
        if ((gEnetAppCfg.numClassDStreams >= 0) || (gEnetAppCfg.numClassDStreams <= MAX_CLASSD_STREAMS))
        {
            EnetAppUtils_print("%d\r\n", gEnetAppCfg.numClassDStreams);
            break;
        }
    }

    gEnetAppCfg.payLoadLength = 100;
    while(true)
    {

        EnetAppUtils_print("Enter number of packets to send\r\n");
        DebugP_scanf("%d", &gEnetAppCfg.packetCount);
        break;
    }
}

void configureNodeId()
{
    while (true)
    {
#if 0
        EnetAppUtils_print("\r\n");
        EnetAppUtils_print("\r\n");
        EnetAppUtils_print("\r\n");
        EnetAppUtils_print("      EtherRing Demonstartion Completed\r\n");
        EnetAppUtils_print("\r\n");
        EnetAppUtils_print("\r\n");
        EnetAppUtils_print("\r\n");
#endif
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
