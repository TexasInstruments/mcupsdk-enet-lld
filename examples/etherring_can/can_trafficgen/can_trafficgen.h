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
 * \file  can_trafficgen.h
 *
 * \brief This file contains the CAN traffic generator API declarations
 */
#ifndef ETHERRING_CAN_TRAFFIC_GENERATOR_H_
#define ETHERRING_CAN_TRAFFIC_GENERATOR_H_

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <include/core/enet_osal.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <enet.h>
#include <enet_cfg.h>
#include <include/core/enet_dma.h>
#include <include/per/cpsw.h>

#include <enet_apputils.h>
#include <enet_appmemutils.h>
#include <enet_appmemutils_cfg.h>

#include "ti_board_config.h"

#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "etherring_can_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
/* Task stack size in bytes */
#define ENETAPP_TASK_STACK_SZ                                           (4096U)
/* Maximum number of packet configurations */
#define ENETAPP_MAX_PKT_CONFIG                                          (16U)
/* Count of memory blocks for CAN packets */
#define ENETAPP_MAX_CANPKT_MEMBLOCKS                                    (32U)

/* ========================================================================== */
/*                                 Structures                                 */
/* ========================================================================== */
/**
* \brief  Structure for CAN packet wrapper with queue node.
*/
typedef struct
{
    /* Pointer to next buffer in queue
     * Note: Keep EnetQ_Node as first member always as driver uses generic
     * queue functions and dereferences to this member */
    EnetQ_Node node;
    /* CAN element containing the actual message */
    MCAN_RxBufElement canElement;
}CAN_pkt;

/**
* \brief  Structure for CAN packet generation parameters.
*/
typedef struct
{
    /* CAN message identifier to be used */
    uint32_t CAN_msgId;
    /* Size of the payload in bytes */
    uint32_t payloadSize;
    /* Period in microseconds between packet generation */
    uint32_t periodicity;
    /* Number of packets to generate per tick */
    uint32_t packetCountPerTick;
    /* Running counter of generated packets */
    uint32_t packetCounter;
    /* Maximum number of packets to send */
    uint32_t maxPacketSend;
    /* Flag to check if maxPacketSend pkts are sent*/
    bool hasAllCanPktsSent;
}CanFdPktPararms;

/**
* \brief  Main Traffic Generator configuration structure.
*/
typedef struct
{
    /* Timer tick period in microseconds */
    uint32_t timerTickPeriodicityInus;
    /* Number of valid packet configurations */
    uint32_t validPktParamsCount;
    /* Array of CAN packet buffers */
    CAN_pkt rxCANPktList[ENETAPP_MAX_CANPKT_MEMBLOCKS];
    /* Queue for ready-to-process CAN elements */
    EnetQ rxReadyElementQ;
    /* Array of packet configurations */
    CanFdPktPararms pktParams[ENETAPP_MAX_PKT_CONFIG];
    /* Semaphore for synchronizing TG task */
    SemaphoreP_Object tgSemObj;
    /* Number of valid packet configurations */
    uint32_t canGeneratedPktCount;
    /* Pointer to the Free Timestamp Queue */
    EnetQ *timestampFreeQPtr;
    /* Pointer to the Ready Timestamp Queue */
    EnetQ *timestampReadyQPtr;
    /* Core key returned by Enet RM after attaching this core */
    uint32_t coreId;
    /* Enet driver handle for this peripheral type/instance */
    Enet_Handle hEnet;
}CanTrafficGen_Object;


/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

/**
 * @brief Sets up the CAN traffic generator
 *
 * Initializes the traffic generator system, creates task, and prepares
 * memory for CAN packet generation.
 *
 * @param cfg Pointer to the traffic generator configuration
 * @return int32_t ENET_SOK on success, ENET_EFAIL on failure
 */
int32_t CanTrafficGen_setup(CanTrafficGen_Object *cfg);

/**
 * @brief Releases a CAN packet back to the free pool
 *
 * Returns a used CAN packet to the free packet queue for reuse
 *
 * @param rxPkt Pointer to the CAN packet to be freed
 */
void CanTrafficGen_memFree(CAN_pkt *rxPkt);

#ifdef __cplusplus
}
#endif
#endif /* ETHERRING_CAN_TRAFFIC_GENERATOR_H_ */
