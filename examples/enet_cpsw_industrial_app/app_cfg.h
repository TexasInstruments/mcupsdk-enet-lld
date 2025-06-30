/*
 * Copyright (C) 2025 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _APP_CFG_H_
#define _APP_CFG_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include "FreeRTOS.h"
#include "task.h"

#include <include/core/enet_osal.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/QueueP.h>

#include <enet.h>
#include <enet_cfg.h>
#include <include/core/enet_dma.h>
#include <include/per/cpsw.h>

#include <enet_board.h>
#include <enet_apputils.h>
#include <enet_appboardutils.h>
#include <enet_appmemutils.h>
#include <enet_appmemutils_cfg.h>

#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include "ti_enet_config.h"
#include "ti_enet_open_close.h"

#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "ti_enet_lwipif.h"

#ifdef __cplusplus
extern "C" {
#endif


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/* Number of nodes in the network */
#define ENETAPP_NODE_COUNT                        (10U)

/* Task stack size */
#define ENETAPP_TASK_STACK_SZ                     (10U * 1024U)

/*Counting Semaphore count*/
#define ENETAPP_COUNTING_SEM_COUNT                (10U)

/* Number of MAC ports */
#define ENETAPP_NUM_MAC_PORTS                     (2U)

/* Header size */
#define ENETAPP_TOTAL_HEADER_SIZE                 (42U)

/* Ethernet header size */
#define ENETAPP_ETH_HDR_SIZE                      (14U)

/* IPv4 header size */
#define ENETAPP_IPV4_HDR_SIZE                     (20U)

/* UDP header size */
#define ENETAPP_UDP_HDR_SIZE                      (8U)

/*IPv4 protocol number for UDP*/
#define ENETAPP_IPV4_HDR_UDP                      (0x11)

/*IPv4 ethertype value*/
#define ENETAPP_IPV4_ETHERTYPE                    (0x0800)

/* (Maximum) Packet payload size */
#define ENETAPP_PKT_PAYLOAD_SIZE                  (1470U)

/*Number of packets to profile */
#define ENETAPP_PKT_PROFILE                       (10000U)

/*IP address pool count for static IP configuration */
#define IP_ADDR_POOL_COUNT                        (1U)

/*Port number for TCP server */
#define APP_SERVER_PORT                           (8888)

/* Static IP addresses for TCP server */
/** Static IP address */
#define ENETAPP_STATIC_IP_ADDR(node_id) (ip_addr_t)IPADDR4_INIT_BYTES(192, 168, 1, node_id)

/** Static IP gateway */
#define ENETAPP_STATIC_IP_GATEWAY (ip_addr_t)IPADDR4_INIT_BYTES(192, 168, 1, 1)

/** Static IP netmask */
#define ENETAPP_STATIC_IP_NETMASK (ip_addr_t)IPADDR4_INIT_BYTES(255, 255, 255, 0)

/* Static IP gateway for TCP server */
#define ENETAPP_STATIC_IP_GATEWAY (ip_addr_t)IPADDR4_INIT_BYTES(192, 168, 1, 1)

/* Static IP netmask for TCP server */
#define ENETAPP_STATIC_IP_NETMASK (ip_addr_t)IPADDR4_INIT_BYTES(255, 255, 255, 0)

/*Broadcast MAC address*/
#define BROADCAST_MAC_ADDRESS (const uint8_t[]) { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }

/*Macro for enabling profiling */
#define ENETAPP_PROFILE_EN

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/* Context of a channel */
typedef struct EnetApp_ChInfo_s
{
 /* TX channel number */
    uint32_t txChNum;

    /* TX channel handle */
    EnetDma_TxChHandle hTxCh;

    /* Regular traffic RX channel number */
    uint32_t rxChNum;

    /* RX channel handle for regular traffic */
    EnetDma_RxChHandle hRxCh;

    /* RX task handle - receives Regular packets, changes source/dest MAC addresses
     * and transmits the packets back */
    TaskP_Object rxTaskObj;

    /* Semaphore posted from RX callback when Regular packets have arrived */
    SemaphoreP_Object rxSemObj;

    /* Semaphore used to synchronize all REgular RX tasks exits */
    SemaphoreP_Object rxDoneSemObj;

    /* Queue of free TX packets */
    EnetDma_PktQ txFreePktInfoQ;

    /* Queue of free RX packets */
    EnetDma_PktQ rxFreePktInfoQ;

    /* Real-time channel IP address*/
    uint8_t IpAddr[ENET_IPv4_ADDR_LEN];

} EnetApp_ChInfo;


typedef struct EnetApp_TcpObj_s{

/* dhcp struct for the ethernet netif */
    struct dhcp sDhcp[ENET_SYSCFG_NETIF_COUNT];

    struct netif *sNetif[ENET_SYSCFG_NETIF_COUNT];

    LwipifEnetApp_Handle hlwipIfApp ;

    ip_addr_t staticIP[IP_ADDR_POOL_COUNT];

    ip_addr_t staticIPGateway[IP_ADDR_POOL_COUNT];

    ip_addr_t staticIPNetmask[IP_ADDR_POOL_COUNT];

}EnetApp_TcpObj ;

/* Context of a peripheral/port */
typedef struct EnetApp_Obj_s
{
    /* Enet driver handle for this peripheral type/instance */
    Enet_Handle hEnet;

    /* Channel handle for Realtime traffic */
    EnetApp_ChInfo hRawUdp;

    /* TCP context handle*/
    EnetApp_TcpObj hTcp;

      /* MAC address. It's port's MAC address in Dual-MAC or
     * host port's MAC addres in Switch */
    uint8_t macAddr[ENET_MAC_ADDR_LEN];

    /* Core key returned by Enet RM after attaching this core */
    uint32_t coreKey;

    /* Flag which indicates if test shall run */
    volatile bool run;

    /* This core's id */
    uint32_t coreId;

    /* Node number of this device */
    uint8_t nodeId;

    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

    /* MAC ports List to use for the above EnetType & InstId*/
    uint8_t numMacPort;

    /* Num MAC ports to use for the above EnetType & InstId*/
    Enet_MacPort macPortList[ENETAPP_NUM_MAC_PORTS];

} EnetApp_Obj;

/* IPv4 header structure */
typedef struct
{
    /* Version info */
    uint8_t verIHL;

    /* Type of service */
    uint8_t tos;

    /* Packet length */
    uint16_t totalPktLen;

    /* IP identifier */
    uint16_t ipId;

    /* Fragment info */
    uint16_t flagFragOffset;

    /* Time to live */
    uint8_t ttl;

    /* Protocol type */
    uint8_t protocol;

    /* Header checksum */
    uint16_t hdrChksum;

    /* Source IP address */
    uint8_t srcIP[ENET_IPv4_ADDR_LEN];

    /* Destination IP address */
    uint8_t dstIP[ENET_IPv4_ADDR_LEN];

} __attribute__ ((packed)) EthIPv4Header;

/* UDP-Lite header structure */
typedef struct
{
    /* Source Port Address */
    uint16_t srcPort;

    /* Destination Port Address */
    uint16_t dstPort;

    /* UDP header checksum coverage */
    uint16_t csumCoverage;

    /* UDP header checksum */
    uint16_t csum;

} __attribute__ ((packed)) EthUdpLiteHeader;

/* Complete UDP frame structure */
typedef struct
{
    /* Ethernet frame header */
    EthFrameHeader hdr;

    /* IPv4 header */
    EthIPv4Header ipv4Header;

    /* UDP-Lite header */
    EthUdpLiteHeader udpHeader;

    /* Payload data */
    uint8_t payload[ENETAPP_PKT_PAYLOAD_SIZE];

} __attribute__ ((packed)) EthUdpFrame;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif/*_APP_CFG_H_*/
