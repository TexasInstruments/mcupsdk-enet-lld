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
 * \file  etherring_can_cfg.h
 *
 * \brief This file contains the Etherring App Objects
 */
#ifndef ETHERRING_CAN_CFG_H_
#define ETHERRING_CAN_CFG_H_

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include "ether_ring/inc/ether_ring.h"
#include "kernel/dpl/TaskP.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                  Macros                                    */
/* ========================================================================== */
/* Timestamp pool size for profiling */
#define ETHERRING_CAN_TIMESTAMP_POOL_COUNT                            (32U)

/* Macro to enable/disable the profiling. */
#define ETHERRING_MCAN_ENABLE_PROFILING
/* ========================================================================== */
/*                                 Structures                                 */
/* ========================================================================== */
/* Structure to store 64bit timestamp */
typedef struct
{
    /* Pointer to next buffer in queue
     * Note: Keep EnetQ_Node as first member always as driver uses generic
     * queue functions and dereferences to this member */
    EnetQ_Node node;
    /* CAN element containing the actual message */
    uint64_t timeStamp;
}EnetApp_timestampQ;

/* Test parameters for each port in the multi-channel test */
typedef struct EnetApp_Cfg_s
{
    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

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

    /* Queue of free RX packets */
    EnetDma_PktQ rxFreePktInfoQ;

    /* Pool of Timestamps */
    EnetApp_timestampQ timestampPool[ETHERRING_CAN_TIMESTAMP_POOL_COUNT];

    /* Queue of free Timestamps */
    EnetQ timestampFreeQ;

    /* Queue of ready Timestamps */
    EnetQ timestampReadyQ;

    /* Regular traffic RX flow index */
    uint32_t rxChNum;

    /* RX channel handle for regular traffic */
    EnetDma_RxChHandle hRxCh;

    /* Number of Ethernet ports to be enabled */
    uint8_t numMacPorts;

    /* RX task handle - receives Regular packets, changes source/dest MAC addresses
     * and transmits the packets back */
    TaskP_Object rxTaskObj;

    /* TX task handle */
    TaskP_Object txTaskObj;

    /* Semaphore posted from RX callback when Regular packets have arrived */
    SemaphoreP_Object rxSemObj;

    /* Semaphore posted every 32ms to clear the task for lookup table */
    SemaphoreP_Object etherringSemObj;

    /* Peripheral's MAC ports to use */
    Enet_MacPort macPorts[2];

    /* Ether-Ring Driver Handle */
    EtherRing_Handle hEtherRing;

    /* Ether-Ring Node ID*/
    uint32_t nodeId;

    /* Test Pass boolean */
    bool isTestPassPrinted;

} EnetApp_Cfg;

#ifdef __cplusplus
}
#endif
#endif /* ETHERRING_CAN_CFG_H_ */
