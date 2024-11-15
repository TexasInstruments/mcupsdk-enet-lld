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
#ifndef _ENETAPP_H_
#define _ENETAPP_H_

#include <ether_ring/inc/ether_ring.h>
#include "dataflow.h"

#define NUM_CLASSA_STREAMS 0 // 8000pkts per sec per stream
#define NUM_CLASSD_STREAMS 1 // 1000pkts per sec per stream

#define MAX_CLASSA_STREAMS 3
#define MAX_CLASSD_STREAMS 3

#define MAX_NODES_IN_LOOP 4
/*============================================================================*/
/*                           Macros and Constants                             */
/*============================================================================*/

#define MAX_NUM_MAC_PORTS           (3U)
#define ENETAPP_DEFAULT_CFG_NAME    "sitara-cpsw"
#define EnetAppAbort(message)       \
    EnetAppUtils_print(message);    \
    EnetAppUtils_assert(false);

/* ========================================================================== */
/*                          Structure Declarations                            */
/* ========================================================================== */

/* Test parameters for each port in the multi-channel test */
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

    /* MAC address. It's port's MAC address in Dual-MAC or
     * host port's MAC addres in Switch */
    uint8_t macAddr[ENET_MAC_ADDR_LEN];

    /* TX channel number */
    uint32_t txChNum;

    /* TX channel handle */
    EnetDma_TxChHandle hTxCh;

    /* Queue of free TX packets */
    EnetDma_PktQ txFreePktInfoQ;

    /* Start flow index */
    uint32_t rxStartFlowIdx;

    /* Regular traffic RX flow index */
    uint32_t rxFlowIdx;

    /* RX channel handle for regular traffic */
    EnetDma_RxChHandle hRxCh;

    /* Number of Ethernet ports to be enabled */
    uint8_t numMacPorts;

    /* RX task handle - receives Regular packets, changes source/dest MAC addresses
     * and transmits the packets back */
    TaskP_Object rxTaskObj;

    /* AVB talker task */
    TaskP_Object talkerTaskObj;

    /* AVB listener task */
    TaskP_Object listenerTaskObj;

    TaskP_Object etherringTaskObj;

    /* Semaphore posted from IPC Notify to start the listener */
    SemaphoreP_Object startListnerSemObj;

    /* Semaphore posted from IPC Notify to start the talker */
    SemaphoreP_Object startTalkerSemObj;

    /* Semaphore posted from RX callback when Regular packets have arrived */
    SemaphoreP_Object rxSemObj;

    /* Semaphore posted every 32ms to clear the task for lookup table */
    SemaphoreP_Object etherringSemObj;

    /* Semaphore used to synchronize all Regular RX tasks exits */
    SemaphoreP_Object rxDoneSemObj;

    /* Peripheral's MAC ports to use */
    Enet_MacPort macPorts[MAX_NUM_MAC_PORTS];

    /* Semaphore posted from RX callback when Regular packets have arrived */
    SemaphoreP_Object streamSemObj[MAX_CLASSA_STREAMS + MAX_CLASSD_STREAMS];

    TaskP_Object streamTaskObj[MAX_CLASSA_STREAMS + MAX_CLASSD_STREAMS];

    uint64_t txSubmittedCount;

    uint64_t totalRxCnt;

    EtherRing_Handle hEtherRing;

    int numClassAStreams;

    int numClassDStreams;

    bool isTxEnabled;

    int payLoadLength;

    int packetCount;

    bool isIntervlanEnabled;

    int nodeId;
} EnetApp_Cfg;

#if 0
typedef struct EtherRingApp_TxRtrvPollTaskInfo_s
{
    TaskP_Object      task;
    uint8_t gEnetAppTaskStackPolling[10U * 1024U] __attribute__ ((aligned(32)));
    SemaphoreP_Object sem;

    /*
     * Handle to counting shutdown semaphore, which all subtasks created in the
     * open function must post before the close operation can complete.
     */
    SemaphoreP_Object shutDownSemObj;
    /** Boolean to indicate shutDownFlag status of translation layer.*/
    volatile bool shutDownFlag;

    /*
     * Clock handle for triggering the packet Rx notify
     */
    ClockP_Object pollLinkClkObj;
} EtherRingApp_TxRtrvPollTaskInfo;

EtherRingApp_TxRtrvPollTaskInfo pollTxRtrvTaskInfo;
#endif
/* ========================================================================== */
/*                          External Functions                                */
/* ========================================================================== */

int EnetApp_initTsn(void);
void EnetApp_initAppCfg(EnetPer_AttachCoreOutArgs *attachArgs, EnetApp_HandleInfo *handleInfo);
void EnetApp_setMacAddr(uint8_t hwaddr[]);
void EnetApp_addBroadcastEntry(void);
void EnetApp_printCpuLoad(void);

#endif //_ENETAPP_H_
