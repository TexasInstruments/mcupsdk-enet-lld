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
 * \brief  Structure for MCAN Tx Buffer element.
 */
typedef struct
{

    uint32_t id;
    /**< Identifier */

    uint32_t rtr;
    /**< Remote Transmission Request
     *   0 = Transmit data frame
     *   1 = Transmit remote frame
     */
    uint32_t xtd;
    /**< Extended Identifier
     *   0 = 11-bit standard identifier
     *   1 = 29-bit extended identifier
     */
    uint32_t esi;
    /**< Error State Indicator
     *   0 = ESI bit in CAN FD format depends only on error passive flag
     *   1 = ESI bit in CAN FD format transmitted recessive
     */
    uint32_t dlc;
    /**< Data Length Code
     *   0-8  = CAN + CAN FD: transmit frame has 0-8 data bytes
     *   9-15 = CAN: transmit frame has 8 data bytes
     *   9-15 = CAN FD: transmit frame has 12/16/20/24/32/48/64 data bytes
     */
    uint32_t brs;
    /**< Bit Rat Switching
     *   0 = CAN FD frames transmitted without bit rate switching
     *   1 = CAN FD frames transmitted with bit rate switching
     */
    uint32_t fdf;
    /**< FD Format
     *   0 = Frame transmitted in Classic CAN format
     *   1 = Frame transmitted in CAN FD format
     */
    uint32_t efc;
    /**< Event FIFO Control
     *   0 = Don't store Tx events
     *   1 = Store Tx events
     */
    uint32_t mm;
    /**< Message Marker */

    uint8_t  data[64];
    /**< Data bytes.
     *   Only first dlc number of bytes are valid.
     */
}MCAN_TxBufElement;

/**
 * \brief  Structure for MCAN Rx Buffer element.
 */
typedef struct
{
    uint32_t id;
    /**< Identifier */

    uint32_t rtr;
    /**< Remote Transmission Request
     *   0 = Received frame is a data frame
     *   1 = Received frame is a remote frame
     */
    uint32_t xtd;
    /**< Extended Identifier
     *   0 = 11-bit standard identifier
     *   1 = 29-bit extended identifier
     */
    uint32_t esi;
    /**< Error State Indicator
     *   0 = Transmitting node is error active
     *   1 = Transmitting node is error passive
     */
    uint32_t rxts;
    /**< Rx Timestamp */

    uint32_t dlc;
    /**< Data Length Code
     *   0-8  = CAN + CAN FD: received frame has 0-8 data bytes
     *   9-15 = CAN: received frame has 8 data bytes
     *   9-15 = CAN FD: received frame has 12/16/20/24/32/48/64 data bytes
     */
    uint32_t brs;
    /**< Bit Rat Switching
     *   0 = Frame received without bit rate switching
     *   1 = Frame received with bit rate switching
     */
    uint32_t fdf;
    /**< FD Format
     *   0 = Standard frame format
     *   1 = CAN FD frame format (new DLC-coding and CRC)
     */
    uint32_t fidx;
    /**< Filter Index */

    uint32_t anmf;
    /**< Accepted Non-matching Frame
     *   0 = Received frame matching filter index FIDX
     *   1 = Received frame did not match any Rx filter element
     */
    uint8_t  data[64];
    /**< Data bytes.
     *   Only first dlc number of bytes are valid.
     */
}MCAN_RxBufElement;
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
