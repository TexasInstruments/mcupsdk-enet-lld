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
 * \file  enet_app_main.c
 *
 * \brief This file contains the implementation of the Ether-Ring LWIP example entry
 *        point
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include "cpsw_config/etherring_cpsw_config.h"
#include "etherring_trafficgen_config.h"
#include "back_ground_tcp_app/tcp_config.h"
/* ========================================================================== */
/*                         Global Variables                                   */
/* ========================================================================== */
extern EnetApp_Cfg gEnetAppCfg;

/* Variable to select the traffic Profile Type for traffic generation */
EnetApp_TrafficProfile gTrafficProfile = TRAFFIC_PROFILE_A;

EnetApp_TrafficGen_Obj TrafficGen_ClassAList[ENETAPP_MAX_CLASSA_STREAMS];
/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
void EnetApp_configureNodeId();
void EnetApp_mainTask(void *args);
void EnetApp_initAllTrafficObj();
void EnetApp_configureTrafficProfileA();
void EnetApp_configureTrafficProfileB();
void EnetApp_configureTrafficProfileC();
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
void EnetApp_mainTask(void *args)
{
    Drivers_open();
    Board_driversOpen();
    DebugP_log("===============================\r\n");
    DebugP_log("    EtherRing TrafficGen App    \r\n");
    DebugP_log("===============================\r\n");

    EnetApp_configureNodeId();

    EnetAppUtils_print("Create RX task for redundancy Traffic\r\n");

    EnetApp_open();

    /* Creates FreeRTOS Rx Task to receive Redundancy Packets */
    EnetApp_createRxTrafficTask();

    /* Creates FreeRTOS Task to clear Look-up table */
    EnetApp_createEtherRingClearTask();

    /* Initialize all the streams(Class-A) */
    EnetApp_initAllTrafficObj();

#ifdef ENETAPP_ENABLE_TCP_BG_TRAFFIC
    SemaphoreP_constructBinary(&gEnetAppCfg.lwipSemObj, 0);
    EnetApp_setupNetworkStack();

    while (false == EnetApp_isNetworkUp(netif_default))
    {
        EnetAppUtils_print("Waiting for network UP ...\r\n");
        ClockP_sleep(3U);
    }

    EnetAppUtils_print("Network is UP ...\r\n");

    /* Devices acts as either TCP Server or Client based on NodeId */
    if (gEnetAppCfg.nodeId % 2 == 0)
    {
        EnetAppUtils_print("Acts as TCP Server\r\n");
        EnetApp_startTcpServer();
    }
    else
    {
        EnetAppUtils_print("Acts as TCP Client\r\n");
        EnetApp_startTcpClient();
    }
#endif

    /* Configures the streams based on TrafficProfile(TxHeavy, RxHeavy) */
    if (gTrafficProfile == TRAFFIC_PROFILE_A)
    {
        EnetApp_configureTrafficProfileA();
    }
    else if (gTrafficProfile == TRAFFIC_PROFILE_B)
    {
        EnetApp_configureTrafficProfileB();
    }
    else if (gTrafficProfile == TRAFFIC_PROFILE_C)
    {
        EnetApp_configureTrafficProfileC();
    }

    /* Creates RTOS Task to send periodic Redundancy Traffic */
    EnetApp_createPeriodicTrafficTask();

    /* Starts the hardware(RTI) timer */
    EnetApp_startHwTimer();


    while (true)
    {
        /* Print CPU Load */
        /* EnetApp_printCpuLoad() */
        ClockP_usleep(30000);

        TaskP_yield();
    }
}

void EnetApp_configureNodeId()
{
    while (true)
    {
        EnetAppUtils_print("0 - Central Compute Node\r\n");
        EnetAppUtils_print("1 - Zone Left Node\r\n");
        EnetAppUtils_print("2 - Zone Right Node\r\n");
        EnetAppUtils_print("3 - Zone Tail Node\r\n");
        EnetAppUtils_print("Enter the nodeId : \r\n");

        DebugP_scanf("%d", &gEnetAppCfg.nodeId);
        if ((gEnetAppCfg.nodeId < 0) || (gEnetAppCfg.nodeId > ENETAPP_MAX_NODES_IN_RING-1))
        {
            EnetAppUtils_print("Enter a valid number for nodeId\r\n");
            continue;
        }
        break;
        TaskP_yield();
    }
}

void EnetApp_initAllTrafficObj()
{
    uint32_t objIndex;
    for (objIndex = 0; objIndex < ENETAPP_MAX_CLASSA_STREAMS ; objIndex++)
    {
        TrafficGen_ClassAList[objIndex].hEtherRing = gEnetAppCfg.hEtherRing;
        TrafficGen_ClassAList[objIndex].isEnabled = false;
        TrafficGen_ClassAList[objIndex].isProfilingEnabled = false;
        TrafficGen_ClassAList[objIndex].trafficType = CLASS_A;
    }
}

void EnetApp_configureTrafficProfileA()
{
    /* Tx Heavy Traffic
     * node 0 -> 3tx streams(each tx to each node) + 1rx stream
     * other nodes -> 3tx streams + 1rx stream */
    if (gEnetAppCfg.nodeId == 0)
    {
        /* Profiling is Enabled only for Class-A stream-0 */
        TrafficGen_ClassAList[0].isEnabled = true;
        TrafficGen_ClassAList[0].isProfilingEnabled = true;
        TrafficGen_ClassAList[0].destinationNodeId = ENETAPP_MAX_NODES_IN_RING/2;
        for (uint32_t objIndex = 1; objIndex < ENETAPP_NUM_CLASSA_STREAMS ;objIndex++)
        {
            TrafficGen_ClassAList[objIndex].isEnabled = true;
            TrafficGen_ClassAList[objIndex].destinationNodeId = ENETAPP_MAX_NODES_IN_RING/2 + 2*(objIndex%2) - 1;
        }
    }
    else
    {
        for (uint32_t objIndex = 0; objIndex < ENETAPP_NUM_CLASSA_STREAMS ;objIndex++)
        {
            if (gEnetAppCfg.nodeId == ENETAPP_MAX_NODES_IN_RING/2 &&
                objIndex == ENETAPP_NUM_CLASSA_STREAMS - 1)
            {
                continue;
            }
            TrafficGen_ClassAList[objIndex].isEnabled = true;
            TrafficGen_ClassAList[objIndex].destinationNodeId = ENETAPP_MAX_NODES_IN_RING + 1;
        }
    }
}

void EnetApp_configureTrafficProfileB()
{
    /* Rx Heavy Traffic
     * node 0 -> 1tx stream + 3rx streams
     * other nodes -> 1tx stream + 3rx streams */
    if (gEnetAppCfg.nodeId == 0)
    {
        /* Profiling is Enabled only for Class-A stream-0 */
        TrafficGen_ClassAList[0].isEnabled = true;
        TrafficGen_ClassAList[0].isProfilingEnabled = true;
        TrafficGen_ClassAList[0].destinationNodeId = ENETAPP_MAX_NODES_IN_RING/2;
    }
    else
    {
        for (uint32_t objIndex = 0; objIndex < ENETAPP_NUM_CLASSA_STREAMS ;objIndex++)
        {
            if (gEnetAppCfg.nodeId == objIndex || gEnetAppCfg.nodeId == ENETAPP_MAX_NODES_IN_RING/2)
            {
                continue;
            }
            TrafficGen_ClassAList[objIndex].isEnabled = true;
            TrafficGen_ClassAList[objIndex].destinationNodeId = objIndex;
        }
    }
}

void EnetApp_configureTrafficProfileC()
{
    /* Symmetric traffic
     * node 0 -> 1tx stream + 1rx stream
     * node others -> 1tx stream + 1rx stream */
    if (gEnetAppCfg.nodeId == 0)
    {
        /* Profiling is Enabled only for Class-A stream-0 */
        TrafficGen_ClassAList[0].isEnabled = true;
        TrafficGen_ClassAList[0].isProfilingEnabled = true;
        TrafficGen_ClassAList[0].destinationNodeId = ENETAPP_MAX_NODES_IN_RING/2;
    }
    else if (gEnetAppCfg.nodeId != ENETAPP_MAX_NODES_IN_RING/2)
    {
        TrafficGen_ClassAList[0].isEnabled = true;
        TrafficGen_ClassAList[0].destinationNodeId = ENETAPP_MAX_NODES_IN_RING + 1;
    }
}

void EnetApp_printCpuLoad(void)
{
    static uint32_t startTime_ms = 0;
    const  uint32_t currTime_ms  = ClockP_getTimeUsec()/1000;
    const  uint32_t printInterval_ms = 5000;

    if (startTime_ms == 0)
    {
        startTime_ms = currTime_ms;
    }
    else if ((currTime_ms - startTime_ms) > printInterval_ms)
    {
        const uint32_t cpuLoad = TaskP_loadGetTotalCpuLoad();

        DebugP_log(" %6d.%3ds : CPU load = %3d.%02d %%\r\n",
                  currTime_ms/1000, currTime_ms%1000,
                  cpuLoad/100, cpuLoad%100 );

        startTime_ms = currTime_ms;
        TaskP_loadResetAll();
    }
    return;
}
