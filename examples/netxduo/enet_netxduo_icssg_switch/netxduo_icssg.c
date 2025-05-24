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

/* C runtime includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <netxduo_enet.h>
#include <tx_port.h>
#include <nx_api.h>
#include <nxd_dhcp_client.h>

/* SDK includes */
#include <enet_apputils.h>
#include "ti_enet_netxduo.h"
#include "ti_board_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>

#include <include/per/icssg.h>
#include <priv/per/icssg_priv.h>


#if (NETXDUO_COUNT > 1u)
#error "This example does not support more than one Netx instance."
#endif

#if (NETXDUO_IF_COUNT > 1u)
#error "This example does not support more than one interface."
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define PACKET_SIZE     1536
#define POOL_SIZE      ((sizeof(NX_PACKET) + PACKET_SIZE) * (ENET_SYSCFG_TOTAL_NUM_RX_PKT + ENET_SYSCFG_TOTAL_NUM_TX_PKT))

#define IP_THREAD_STACK_SIZE        8192u
#define IP_ARP_THREAD_STACK_SIZE    8192u


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static uint8_t gIpThreadStack[IP_THREAD_STACK_SIZE]__attribute__((aligned(ENET_UTILS_CACHELINE_SIZE)));
static uint8_t gIpArpThreadStack[IP_ARP_THREAD_STACK_SIZE]__attribute__((aligned(ENET_UTILS_CACHELINE_SIZE)));

static uint8_t gPoolMem[POOL_SIZE]__attribute__ ((aligned(ENETDMA_CACHELINE_ALIGNMENT), section(".bss:ENET_DMA_PKT_MEMPOOL")));

static NX_PACKET_POOL gPacketPool;
static NX_IP gIp;
static NX_DHCP gDhcpClient;


/* ========================================================================== */
/*                          Function Prototypes                               */
/* ========================================================================== */

static int32_t EnetApp_setMacAddress(const Enet_Type enetType, uint32_t instId, Enet_MacPort macPort, uint8_t macAddr[ENET_MAC_ADDR_LEN]);


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int netxduo_icssg_main(ULONG arg)
{
    Enet_Type enetType;
    uint32_t instId;
    Enet_MacPort macPort;
    ULONG actual_status;
    ULONG netMask;
    ULONG ipAddr;
    EnetApp_GetMacAddrOutArgs outArgs;
    uint32_t rxChCnt;
    uint32_t txChCnt;
    const uint32_t *rxChIds;
    const uint32_t *txChIds;
    nx_enet_drv_rx_ch_hndl_t rxChs[ENET_SYSCFG_RX_FLOWS_NUM];
    nx_enet_drv_tx_ch_hndl_t txChs[ENET_SYSCFG_TX_CHANNELS_NUM];
    int32_t status = ENET_SOK;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("==============================\r\n");
    DebugP_log("   NETXDUO ICSSG SWITCH MODE  \r\n");
    DebugP_log("==============================\r\n");


    EnetApp_getEnetInstInfo(CONFIG_ENET_ICSS0, &enetType, &instId);

    EnetAppUtils_enableClocks(enetType, instId);

    EnetApp_driverInit();
    status = EnetApp_driverOpen(enetType, instId);
    DebugP_assert(status == ENET_SOK);


    /* Initialize the NetX system.  */
    nx_system_initialize();

    /* Create a packet pool.  */
    status = nx_packet_pool_create(&gPacketPool, "NetX Main Packet Pool", PACKET_SIZE, &gPoolMem[0], POOL_SIZE);
    EnetAppUtils_assert(status == NX_SUCCESS);


    /* Allocate NetX Rx channel and corresponding buffers. */
    NetxEnetApp_getAllRxChIDs(&rxChIds, &rxChCnt);
    for(size_t k = 0u; k < rxChCnt; k++) {

        EnetApp_GetDmaHandleInArgs inArgs = {0};
        EnetApp_GetRxDmaHandleOutArgs outArgs;

        EnetApp_getRxDmaHandle(rxChIds[k], &inArgs, &outArgs);

        EnetAppUtils_assert(outArgs.hRxCh != NULL);
        NetxEnetDriver_allocRxCh(outArgs.hRxCh, outArgs.maxNumRxPkts, &gPacketPool, &rxChs[k]);
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

    macPort = NetxEnetApp_getMacPort(0, 0);
    EnetApp_getMacAddress(rxChIds[0], &outArgs);

    EnetApp_setMacAddress(enetType, instId, macPort, &outArgs.macAddr[0][0]);

    NetxEnetDriver_allocIf("PRI", macPort, &outArgs.macAddr[0][0], &rxChs[0], rxChCnt, &txChs[0], txChCnt);


    /* Create an IP instance.  */
    status = nx_ip_create(&gIp, "NetX IP Instance 0", IP_ADDRESS(0, 0, 0, 0), 0xFFFFFF00UL, &gPacketPool, _nx_enet_driver, (void *)&gIpThreadStack[0], IP_THREAD_STACK_SIZE, 1);
    EnetAppUtils_assert(status == NX_SUCCESS);


    /* Enable ARP */
    status = nx_arp_enable(&gIp, (void *)&gIpArpThreadStack[0], IP_ARP_THREAD_STACK_SIZE);
    EnetAppUtils_assert(status == NX_SUCCESS);

    /* Enable ICMP */
    status = nxd_icmp_enable(&gIp);
    EnetAppUtils_assert(status == NX_SUCCESS);

    /* Enable UDP. */
    status = nx_udp_enable(&gIp);
    EnetAppUtils_assert(status == NX_SUCCESS);


    /* Create the DHCP instance.  */
    status = nx_dhcp_create(&gDhcpClient, &gIp, "DHCP client");
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


    /* Get primary interface address. */
    status = nx_ip_interface_address_get(&gIp, 0u, &ipAddr, &netMask);
    EnetAppUtils_assert(status == NX_SUCCESS);

    DebugP_log("Local Interface IP is: %lu.%lu.%lu.%lu\n", ((ipAddr >> 24u) & 0xFF), ((ipAddr >> 16u) & 0xFF), ((ipAddr >> 8u) & 0xFF), (ipAddr & 0xFF));

    while (1) {

        tx_thread_sleep(100);
    }

    return 0;
}


static int32_t EnetApp_setMacAddress(const Enet_Type enetType, uint32_t instId, Enet_MacPort macPort, uint8_t macAddr[ENET_MAC_ADDR_LEN])
{
    Enet_Handle hEnet;
    int32_t  status = ENET_SOK;
    uint32_t coreId = EnetSoc_getCoreId();

    hEnet = Enet_getHandle(enetType, instId);

    /* Add port MAC entry in case of ICSSG dual MAC */
    if (ENET_ICSSG_DUALMAC == enetType)
    {
        Enet_IoctlPrms prms;
        IcssgMacPort_SetMacAddressInArgs inArgs;

        EnetUtils_copyMacAddr(&inArgs.macAddr[0U], &macAddr[0U]);
        inArgs.macPort = macPort;

        ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
        ENET_IOCTL(hEnet, coreId, ICSSG_MACPORT_IOCTL_SET_MACADDR, &prms, status);

        if (status != ENET_SOK)
        {
            EnetAppUtils_print(
                    "Lwip2Enet_setMacAddress() failed ICSSG_MACPORT_IOCTL_ADD_INTERFACE_MACADDR: %d\r\n",
                    status);
        }
        EnetAppUtils_assert(status == ENET_SOK);
    }
    else if (ENET_ICSSG_SWITCH == enetType)
    {
        Enet_IoctlPrms prms;
        Icssg_MacAddr addr;

        /* Set host port's MAC address */
        EnetUtils_copyMacAddr(&addr.macAddr[0U], &macAddr[0U]);
        ENET_IOCTL_SET_IN_ARGS(&prms, &addr);
        {
            ENET_IOCTL(hEnet, coreId, ICSSG_HOSTPORT_IOCTL_SET_MACADDR, &prms,
                    status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print(
                        "EnetAppUtils_addHostPortEntry() failed ICSSG_HOSTPORT_IOCTL_SET_MACADDR: %d\r\n",
                        status);
            }
            EnetAppUtils_assert(status == ENET_SOK);
        }
    }
    else
    {
        EnetAppUtils_assert(false);
    }
    return status;
}

