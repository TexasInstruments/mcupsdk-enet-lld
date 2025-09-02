/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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
 * \file  etherring_mcan.h
 *
 * \brief This file contains the MCAN Driver config APIs and configuration params
 */
#ifndef ETHERRING_MCAN_APP_H_
#define ETHERRING_MCAN_APP_H_
/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/mcan.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include "enet_apputils.h"
#include "etherring_can_cfg.h"

/* ========================================================================== */
/*                                  Macros                                    */
/* ========================================================================== */
/* total CAN Packet count for test */
#define ETHERRING_MCAN_TEST_PKT_COUNT                                 (1000U)

/* ========================================================================== */
/*                                 Structures                                 */
/* ========================================================================== */
/**
* \brief  Structure to store CAN packet stats .
*/
typedef struct
{
    /* Tx CAN packet count */
    uint32_t txCanPacketCount;
    /* Rx CAN packet count */
    uint32_t rxCanPacketCount;
    /* Tx CAN error packet count */
    uint32_t txErrorCount;
    /* Rx CAN error packet count */
    uint32_t rxErrorCount;
    /* Rx ISR count */
    uint32_t rxIsrCount;
}EtherringMcanStats;

/**
* \brief  EtherRing MCAN Object.
*/
typedef struct
{
    /* Semaphore to indicate CAN packet transfer completion */
    SemaphoreP_Object mcanTxDoneSem;
    /* Semaphore to indicate rx CAN packet */
    SemaphoreP_Object mcanRxDoneSem;
    /* Hwip obj for MCAN interrupts */
    HwiP_Object       mcanHwiObject;
    /* MCAN0 base address */
    uint32_t          mcanBaseAddr;
    /* RX MCAN task handle - receives MCAN packets */
    TaskP_Object rxTaskObj;
    /* Structure object to store MCAN APP stats */
    EtherringMcanStats mcanStats;
#ifdef ETHERRING_MCAN_ENABLE_PROFILING
    /* Maximum Round trip time(CAN -> ETH -> ETH -> CAN -> CAN) */
    uint64_t maxTimeStamp;
    /* Average Round trip time(CAN -> ETH -> ETH -> CAN -> CAN) */
    uint64_t avgTimeStamp;
    /* Profile Index for CAN packets */
    uint64_t profileIndex;
    /* Pointer to the Free Timestamp Queue */
    EnetQ *timestampFreeQPtr;
    /* Pointer to the Ready Timestamp Queue */
    EnetQ *timestampReadyQPtr;
    /* Core key returned by Enet RM after attaching this core */
    uint32_t coreId;
    /* Enet driver handle for this peripheral type/instance */
    Enet_Handle hEnet;
#endif
}EtherringMcanObj;

/* ========================================================================== */
/*                          Function Declarations                              */
/* ========================================================================== */
void EtherringCAN_mcanConfig(EtherringMcanObj *etherringMcanObj);
int32_t EtherringCAN_createRxCANTask(EtherringMcanObj *etherringMcanObj);
void EtherringCAN_sendCANPacket(MCAN_TxBufElement *      txMsg, EtherringMcanObj *etherringMcanObj);

#ifdef __cplusplus
}
#endif
#endif /* ETHERRING_MCAN_APP_H_ */
