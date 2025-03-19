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


#ifndef ETHER_RING_LWIP_APP_CPSW_CONFIG_HANDLER_H_
#define ETHER_RING_LWIP_APP_CPSW_CONFIG_HANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include <enet_apputils.h>
#include <enet_board.h>

#include <stdarg.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/* SDK includes */
#include <enet_apputils.h>
#include <enet_board.h>
#include <enet.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/QueueP.h>
#include "ti_board_config.h"
#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"
#include "ti_enet_config.h"
#include "ti_enet_open_close.h"
#include <ether_ring/inc/ether_ring.h>
#include "../etherring_trafficgen_config.h"
/*============================================================================*/
/*                           Macros and Constants                             */
/*============================================================================*/
/*! \brief Maximum Number of MAC Ports */
#define MAX_NUM_MAC_PORTS           (3U)

/*! \brief Vlan TPID */
#define ENETAPP_VLAN_TPID                            (0x8100U)

/*! \brief Vlan PCP MASK */
#define ENETAPP_VLAN_PCP_MASK                        (0x7U)

/*! \brief Vlan DEI MASK */
#define ENETAPP_VLAN_DEI_MASK                        (0x1U)

/*! \brief Vlan VID MASK */
#define ENETAPP_VLAN_VID_MASK                        (0xFFFU)

/*! \brief Vlan TCI */
#define ENETAPP_VLAN_TCI(pcp, dei, vid)              ((((pcp) & ENETAPP_VLAN_PCP_MASK) << 13U) | \
                                                      (((dei) & ENETAPP_VLAN_DEI_MASK) << 12U) | \
                                                      (((vid) & ENETAPP_VLAN_VID_MASK)))

/* Total packet count sent per stream */
#define ENETAPP_PACKETS_SENT_PER_STREAM               1000
/* ========================================================================== */
/*                          Structure Declarations                            */
/* ========================================================================== */

/*!
 * \brief This structure configuration required for the application
 *
 * The parameter structure for AppCfg
 */
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

    /* Regular traffic RX flow index */
    uint32_t rxChNum;

    /* RX channel handle for regular traffic */
    EnetDma_RxChHandle hRxCh;

    /* Number of Ethernet ports to be enabled */
    uint8_t numMacPorts;

    /* RX task handle - receives Regular packets, changes source/dest MAC addresses
     * and transmits the packets back */
    TaskP_Object rxTaskObj;

    /* Ether-Ring Task Object */
    TaskP_Object etherringTaskObj;

    /* Semaphore posted from RX callback when Regular packets have arrived */
    SemaphoreP_Object rxSemObj;

    /* Semaphore posted every 32ms to clear the task for lookup table */
    SemaphoreP_Object etherringSemObj;

    /* Semaphore used to synchronize all Regular RX tasks exits */
    SemaphoreP_Object rxDoneSemObj;

    /* Peripheral's MAC ports to use */
    Enet_MacPort macPorts[MAX_NUM_MAC_PORTS];

    /* Semaphore posted from RX callback when Regular packets have arrived */
    SemaphoreP_Object streamSemObj[ENETAPP_MAX_CLASSA_STREAMS];

    /* Semaphore posted from RX callback when Regular packets have arrived */
    SemaphoreP_Object lwipSemObj;

    /* Semaphore posted from RX callback when Regular packets have arrived */
    SemaphoreP_Object latencyEchoSemObj;

    TaskP_Object streamTaskObj[ENETAPP_MAX_CLASSA_STREAMS];

    /* Ether-Ring Task Object */
    TaskP_Object latencyEchoTaskObj;

    /* Ether-Ring Rx Packet Count */
    uint64_t totalRxCnt;

    /* Ether-Ring Tx Packet Count */
    uint64_t txCount;

    /* Ether-Ring Driver Handle */
    EtherRing_Handle hEtherRing;

    /* Node ID of Device */
    int nodeId;
} EnetApp_Cfg;

/** @brief
 *
 *  Defines TrafficType enums for Traffic Priority
 */
typedef enum {
    CLASS_A = 0U, /** Real-time Traffic */
    CLASS_D,      /** Medium Priority Traffic */
}TrafficType;

/*!
 * \brief This structure stores Traffic Generator Stream Params
 *
 * The parameter structure for Traffic Generator Stream Params
 */
typedef struct TrafficGen_s
{
    /* Ether-Ring Driver Handle */
    EtherRing_Handle hEtherRing;

    /* Flag to check whether TrafficGenerator is Enabled */
    bool isEnabled;

    /* Flag to check whether profiling is Enabled */
    bool isProfilingEnabled;

    /* Traffic Generator uses destinationNodeId while creating packet */
    uint32_t destinationNodeId;

    /* High Priority or Low Priority Traffic(Class-A or D) */
    TrafficType trafficType;
}EnetApp_TrafficGen_Obj, *TrafficGen_Handle;

/** @brief
 *
 *  Defines EnetApp_TrafficProfile enums for various Traffic Types
 */
typedef enum {
    TRAFFIC_PROFILE_A = 0U, /** Tx Heavy on allNodes */
    TRAFFIC_PROFILE_B,      /** Rx Heavy on allNodes */
    TRAFFIC_PROFILE_C,      /** 1Tx + 1Rx Stream on allNodes */
}EnetApp_TrafficProfile;

/*!
 * \brief This structure stores Latency Profile params
 *
 * The parameter structure for Latency Profile params
 */
typedef struct
{
    /* Timestamp Array when packet is created */
    uint32_t sentTimeArray[ENETAPP_PACKETS_SENT_PER_STREAM];

    /* Timestamp Array when packet is received back to the source */
    uint32_t rxTsArray[ENETAPP_PACKETS_SENT_PER_STREAM];

    /* Maximum round-trip latency = max(rxTsArray[i] - sentTimeArray[i]) */
    uint32_t maxLatencyRT;

    /* Maximum latency from source to destination = max((rxTsArray[i] - sentTimeArray[i])/2) */
    uint32_t maxLatency;

    /* Average latency from source to destination = avg((rxTsArray[i] - sentTimeArray[i])/2) */
    uint32_t avgLatency;

    /* Sum of Round-trip latencies  = sum((rxTsArray[i] - sentTimeArray[i])) */
    uint32_t sumLatency;

    /* Sequence number of Tx Packet in the sentTimeArray */
    uint32_t tsSeqNumber;
}EnetApp_LatencyProfile;
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/*!
 * \brief Opens Enets and DMA Channel for Redundancy Packet Generation.
 *
 * \param hEtherRing  [IN] Void
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetApp_open();

/*!
 * \brief Creates FreeRTOS Rx Task to receive Redundancy Packets.
 *
 * \param Void  [IN] Void
 *
 * \return \ref Void
 */
void EnetApp_createRxTrafficTask();

/*!
 * \brief Creates FreeRTOS Task to periodically clear the Ether-Ring Driver Look-up table.
 *
 * \param Void  [IN] Void
 *
 * \return \ref Void
 */
void EnetApp_createEtherRingClearTask();

/*!
 * \brief Creates FreeRTOS Task to send periodic redundancy traffic.
 *
 * \param Void  [IN] Void
 *
 * \return \ref Void
 */
void EnetApp_createPeriodicTrafficTask();

/*!
 * \brief Starts the hardware(RTI) timer to generate interrupts periodically.
 *
 * \param Void  [IN] Void
 *
 * \return \ref Void
 */
void EnetApp_startHwTimer();

#ifdef __cplusplus
}
#endif
#endif /* ETHER_RING_LWIP_APP_CPSW_CONFIG_HANDLER_H_ */
