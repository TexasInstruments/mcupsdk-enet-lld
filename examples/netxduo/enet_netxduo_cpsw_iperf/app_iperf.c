/*
 *  Copyright (C) 2018-2024 Texas Instruments Incorporated
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


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* C runtime includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* NetX includes */
#include <netxduo_enet.h>
#include <tx_port.h>
#include <nx_api.h>
#include <nxd_dhcp_client.h>

/* App includes */
#include "app_cpswconfighandler.h"

/* SDK includes */
#include <enet_apputils.h>
#include "ti_board_config.h"
#include "ti_enet_netxduo.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>


#if (NETXDUO_COUNT > 1u)
#error "This example does not support more than one Netx instance."
#endif

#if (NETXDUO_IF_COUNT > 1u)
#error "This example does not support more than one interface."
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define PACKET_SIZE                    (1536u)
#define IPERF_POOL_SIZE                ((sizeof(NX_PACKET) + PACKET_SIZE) * (ENET_SYSCFG_TOTAL_NUM_TX_PKT / 2))
#define INTERNAL_POOL_SIZE             ((sizeof(NX_PACKET) + PACKET_SIZE) * (ENET_SYSCFG_TOTAL_NUM_TX_PKT / 2 + ENET_SYSCFG_TOTAL_NUM_RX_PKT))

#define IP_THREAD_STACK_SIZE           (8192u)
#define IP_ARP_THREAD_STACK_SIZE       (8192u)
#define HTTP_STACK_SIZE                (8192u)
#define IPERF_STACK_SIZE               (8192u)

#define APP_NUM_ITERATIONS             (2u)
#define APP_SEND_DATA_NUM_ITERATIONS   (5u)

#define SERVER_PORT                    (8888u)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static const uint8_t BROADCAST_MAC_ADDRESS[ENET_MAC_ADDR_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static uint8_t gIpThreadStack[IP_THREAD_STACK_SIZE]__attribute__((aligned(ENET_UTILS_CACHELINE_SIZE)));
static uint8_t gIpArpThreadStack[IP_ARP_THREAD_STACK_SIZE]__attribute__((aligned(ENET_UTILS_CACHELINE_SIZE)));
static uint8_t gHttpThreadStack[HTTP_STACK_SIZE]__attribute__((aligned(ENET_UTILS_CACHELINE_SIZE)));
static uint8_t gIperfThreadStack[IPERF_STACK_SIZE]__attribute__((aligned(ENET_UTILS_CACHELINE_SIZE)));
static uint8_t gInternalPoolMem[INTERNAL_POOL_SIZE]__attribute__ ((aligned(ENETDMA_CACHELINE_ALIGNMENT), section(".bss:ENET_DMA_PKT_MEMPOOL")));
static uint8_t gIperfPoolMem[IPERF_POOL_SIZE]__attribute__ ((aligned(ENETDMA_CACHELINE_ALIGNMENT), section(".bss:ENET_DMA_PKT_MEMPOOL")));

static NX_PACKET_POOL gInternalPacketPool;
static NX_PACKET_POOL gIperfPacketPool;
static NX_IP gIp;
static NX_DHCP gDhcpClient;


/* ========================================================================== */
/*                            Function Prototypes                             */
/* ========================================================================== */

extern  VOID nx_iperf_entry(NX_PACKET_POOL *pool_ptr, NX_IP *ip_ptr, UCHAR* http_stack, ULONG http_stack_size, UCHAR *iperf_stack, ULONG iperf_stack_size);


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int netxduo_cpsw_main(ULONG arg)
{
    Enet_Type enetType;
    uint32_t instId;
    Enet_Handle hEnet;
    ULONG ipAddr;
    ULONG netMask;
    Enet_MacPort macPort;
    uint32_t rxChCnt;
    uint32_t txChCnt;
    ULONG actual_status;
    const uint32_t *rxChIds;
    const uint32_t *txChIds;
    nx_enet_drv_rx_ch_hndl_t rxChs[ENET_NETX_MAX_RX_CHANNELS_PER_PHERIPHERAL];
    nx_enet_drv_tx_ch_hndl_t txChs[ENET_NETX_MAX_RX_CHANNELS_PER_PHERIPHERAL];
    EnetApp_GetMacAddrOutArgs outArgs;
    bool isLinked;
    int32_t status = ENET_SOK;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("========================\r\n");
    DebugP_log("   CPSW NETXDUO IPERF   \r\n");
    DebugP_log("========================\r\n");

    EnetApp_driverInit();

    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &enetType, &instId);

    EnetAppUtils_enableClocks(enetType, instId);
    EnetApp_driverInit();

    status = EnetApp_driverOpen(enetType, instId);
    DebugP_assert(status == ENET_SOK);


    EnetApp_addMCastEntry(enetType, instId, EnetSoc_getCoreId(), BROADCAST_MAC_ADDRESS, CPSW_ALE_ALL_PORTS_MASK);


    /* Initialize the NetX system.  */
    nx_system_initialize();

    /* Create NetX internal packet pool.  */
    status = nx_packet_pool_create(&gInternalPacketPool, "NetX internal packet pool", PACKET_SIZE, &gInternalPoolMem[0], INTERNAL_POOL_SIZE);
    EnetAppUtils_assert(status == NX_SUCCESS);

    /* Create Iperf packet pool.  */
    status = nx_packet_pool_create(&gIperfPacketPool, "Iperf packet pool", PACKET_SIZE, &gIperfPoolMem[0], IPERF_POOL_SIZE);
    EnetAppUtils_assert(status == NX_SUCCESS);



    /* Allocate NetX Rx channel and corresponding buffers. */
    NetxEnetApp_getAllRxChIDs(&rxChIds, &rxChCnt);
    for(size_t k = 0u; k < rxChCnt; k++) {

        EnetApp_GetDmaHandleInArgs inArgs = {0};
        EnetApp_GetRxDmaHandleOutArgs outArgs;

        EnetApp_getRxDmaHandle(rxChIds[k], &inArgs, &outArgs);

        EnetAppUtils_assert(outArgs.hRxCh != NULL);
        NetxEnetDriver_allocRxCh(outArgs.hRxCh, outArgs.maxNumRxPkts, &gInternalPacketPool, &rxChs[k]);
    }

    /* Allocate NetX Tx channel and corresponding buffers. */
    NetxEnetApp_getAllTxChIDs(&txChIds, &txChCnt);
    for (size_t k = 0u; k < txChCnt; k++) {

        EnetApp_GetDmaHandleInArgs inArgs = {0};
        EnetApp_GetTxDmaHandleOutArgs outArgs;

        EnetApp_getTxDmaHandle(txChIds[k], &inArgs, &outArgs);

        EnetAppUtils_assert(outArgs.hTxCh != NULL);
        NetxEnetDriver_allocTxCh(outArgs.hTxCh, outArgs.maxNumTxPkts, &txChs[k]);
    }

    /* Allocate NetX interface and bind to DMA channels. */
    macPort = NetxEnetApp_getMacPort(0, 0);
    EnetApp_getMacAddress(rxChIds[0], &outArgs);
    NetxEnetDriver_allocIf("PRI", macPort, &outArgs.macAddr[0][0], &rxChs[0], rxChCnt, txChs, txChCnt);


    /* Wait for the link on default interface to come up. */
    hEnet = Enet_getHandle(enetType, instId);

    isLinked = false;
    while (!isLinked) {

        EnetAppUtils_print("Waiting for link up...\n");
        isLinked = EnetApp_isPortLinked(hEnet);

        tx_thread_sleep(2u * TX_TIMER_TICKS_PER_SECOND);
    }


    /* Create an IP instance.  */
    status = nx_ip_create(&gIp, "NetX IP Instance 0", IP_ADDRESS(0, 0, 0, 0), 0xFFFFFF00UL, &gInternalPacketPool, _nx_enet_driver, (void *)&gIpThreadStack[0], IP_THREAD_STACK_SIZE, 1);
    EnetAppUtils_assert(status == NX_SUCCESS);


    /* Enable ARP */
    status = nx_arp_enable(&gIp, (void *)&gIpArpThreadStack[0], IP_ARP_THREAD_STACK_SIZE);
    EnetAppUtils_assert(status == NX_SUCCESS);

    /* Enable ICMP */
    status = nxd_icmp_enable(&gIp);
    EnetAppUtils_assert(status == NX_SUCCESS);

    /* Enable UDP */
    status = nx_udp_enable(&gIp);
    EnetAppUtils_assert(status == NX_SUCCESS);

    /* Enable TCP */
    status = nx_tcp_enable(&gIp);
    EnetAppUtils_assert(status == NX_SUCCESS);


    /* Create the DHCP instance.  */
    status = nx_dhcp_create(&gDhcpClient, &gIp, "DHCP-CLIENT");
    EnetAppUtils_assert(status == NX_SUCCESS);

    nx_dhcp_interface_enable(&gDhcpClient, 0u);

    /* Start the DHCP Client.  */
    status = nx_dhcp_interface_start(&gDhcpClient, 0u);
    EnetAppUtils_assert(status == NX_SUCCESS);


    /* Wait for DHCP to assign the IP address.  */
    EnetAppUtils_print("Waiting for address from DHCP server on primary interface...\n");
    do {

        /* Check for address resolution.  */
        status = nx_ip_interface_status_check(&gIp, 0u, NX_IP_ADDRESS_RESOLVED, (ULONG *) &actual_status, NX_IP_PERIODIC_RATE);

        tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);

    } while ((actual_status & NX_IP_ADDRESS_RESOLVED) != NX_IP_ADDRESS_RESOLVED);


    status = nx_ip_interface_address_get(&gIp, 0u, &ipAddr, &netMask);
    EnetAppUtils_assert(status == NX_SUCCESS);

    DebugP_log("Local Interface IP is: %lu.%lu.%lu.%lu\n", ((ipAddr >> 24u) & 0xFF), ((ipAddr >> 16u) & 0xFF), ((ipAddr >> 8u) & 0xFF), (ipAddr & 0xFF));


    /* Call entry function to start iperf test.  */
    nx_iperf_entry(&gIperfPacketPool, &gIp, gHttpThreadStack, HTTP_STACK_SIZE, gIperfThreadStack, IPERF_STACK_SIZE);

    return 0;
}

