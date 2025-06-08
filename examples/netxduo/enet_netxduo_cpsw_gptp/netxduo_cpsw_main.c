/*
 *  Copyright (C) Texas Instruments Incorporated 2022-2024
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
 * \file  tsnapp_lwip_cpsw_main.c
 *
 * \brief This file contains the implementation of the Enet CPSW gPTP stack along with LwIP stack.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <nx_api.h>
#include <tx_api.h>

#include <nxd_dhcp_client.h>

#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/QueueP.h>

#include <enet_apputils.h>
#include <enet_board.h>
#include <netxduo_enet.h>
#include "ti_board_config.h"
#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"
#include "ti_enet_config.h"
#include "ti_enet_open_close.h"
#include "ti_enet_netxduo.h"
#include <tsn_combase/combase.h>
#include <tsn_combase/combase_link.h>
#include "debug_log.h"
#include "dataflow.h"
#include "tsninit.h"


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

static const uint8_t BROADCAST_MAC_ADDRESS[ENET_MAC_ADDR_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };


#if (NETXDUO_COUNT > 1u)
#error "This example does not support more than one Netx instance."
#endif

#if (NETXDUO_IF_COUNT > 1)
#error "This example only supports a single NetX interface."
#endif

#define PACKET_SIZE                    (1536)
#define INTERNAL_POOL_SIZE             ((sizeof(NX_PACKET) + PACKET_SIZE) * (ENET_DMA_TX_CH0_NUM_PKTS + ENET_DMA_RX_CH0_NUM_PKTS))

#define IP_THREAD_STACK_SIZE           (4096u)
#define IP_ARP_THREAD_STACK_SIZE       (4096u)

#define SERVER_PORT                    (8888u)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */




/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */


static void EnetApp_addMCastEntry(Enet_Type enetType, uint32_t instId, uint32_t coreId, const uint8_t *testMCastAddr, uint32_t portMask);

static void EnetApp_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info, void *appArg);

static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort, bool isLinkUp, void *appArg);

static int EnetApp_initTsn(Enet_Type enetType, uint32_t instId, Enet_MacPort macPortList[], uint8_t numMacPort);

static void EnetApp_enableTsSync(Enet_Type enetType, uint32_t instId);

static bool IsMacAddrSet(uint8_t *mac);

static void ConsolePrint(const char *pcString, ...);


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


static uint8_t gIpThreadStack[IP_THREAD_STACK_SIZE]__attribute__((aligned(ENET_UTILS_CACHELINE_SIZE)));
static uint8_t gIpArpThreadStack[IP_ARP_THREAD_STACK_SIZE]__attribute__((aligned(ENET_UTILS_CACHELINE_SIZE)));
static uint8_t gInternalPoolMem[INTERNAL_POOL_SIZE]__attribute__ ((aligned(ENETDMA_CACHELINE_ALIGNMENT), section(".bss:ENET_DMA_PKT_MEMPOOL")));

static NX_PACKET_POOL gInternalPacketPool;
static NX_IP gIp;
static NX_DHCP gDhcpClient;
static NX_TCP_SOCKET gServerSocket;



#define EnetAppAbort(message) \
    EnetAppUtils_print(message);                \
    EnetAppUtils_assert(false);

/* these vars are shared with gptp task to configure gptp, put it in the global mem */
#define ENET_MAX_NUM_MAC_PORTS 3U
static char g_netdevices[ENET_MAX_NUM_MAC_PORTS][CB_MAX_NETDEVNAME] = {0};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

#define LOG_BUFFER_SIZE (1024)


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int netxduo_cpsw_main(ULONG arg)
{
    Enet_Type enetType;
    uint32_t instId;
    Enet_Handle hEnet;
    NX_PACKET *pPacket;
    ULONG ipAddr;
    ULONG netMask;
    Enet_MacPort macPort;
    uint32_t rxChCnt;
    uint32_t txChCnt;
    const uint32_t *rxChIds;
    const uint32_t *txChIds;
    nx_enet_drv_rx_ch_hndl_t rxChs[ENET_NETX_MAX_RX_CHANNELS_PER_PHERIPHERAL];
    nx_enet_drv_tx_ch_hndl_t txChs[ENET_NETX_MAX_TX_CHANNELS_PER_PHERIPHERAL];
    ULONG actual_status;
    Enet_MacPort macPortList[ENET_SYSCFG_MAX_MAC_PORTS];
    uint8_t numMacPort;
    EnetApp_GetMacAddrOutArgs outArgs;
    bool isLinked;
    UINT status;
    int32_t res;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("===============================\r\n");
    DebugP_log("     CPSW NETXDUO TSN GPTP     \r\n");
    DebugP_log("===============================\r\n");


    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &enetType, &instId);

    EnetApp_getEnetInstMacInfo(enetType, instId, &macPortList[0], &numMacPort);

    EnetAppUtils_enableClocks(enetType, instId);

    EnetApp_driverInit();
    res = EnetApp_driverOpen(enetType, instId);
    DebugP_assert(res == ENET_SOK);

    EnetApp_addMCastEntry(enetType, instId, EnetSoc_getCoreId(), BROADCAST_MAC_ADDRESS, CPSW_ALE_ALL_PORTS_MASK);

    EnetApp_enableTsSync(enetType, instId);
    res = EnetApp_initTsn(enetType, instId, &macPortList[0], numMacPort);
    EnetAppUtils_assert(res == ENET_SOK);


    /* Initialize the NetX system.  */
    nx_system_initialize();


    status = nx_packet_pool_create(&gInternalPacketPool, "NetX internal packet pool", PACKET_SIZE, &gInternalPoolMem[0], INTERNAL_POOL_SIZE);
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

    /* Allocate network interfaces and bind to corresponding DMA channels. */
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


    /* Create a socket.  */
    status =  nx_tcp_socket_create(&gIp, &gServerSocket, "Server Socket", NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, 200, NX_NULL, NX_NULL);
    EnetAppUtils_assert(status == NX_SUCCESS);

    DebugP_log("Socket created\r\n");


    /* Listen on the socket.  */
    status =  nx_tcp_server_socket_listen(&gIp, SERVER_PORT, &gServerSocket, 1u, NULL);
    EnetAppUtils_assert(status == NX_SUCCESS);

    DebugP_log("Listening on port %lu\r\n", (ULONG)SERVER_PORT);

    /* Accept the connection. */
    status = nx_tcp_server_socket_accept(&gServerSocket, NX_WAIT_FOREVER);
    EnetAppUtils_assert(status == NX_SUCCESS);

    DebugP_log("Connection accepted.\r\n");

    /* Loop to repeat things over and over again!  */
    while (1)
    {
        status = nx_tcp_socket_receive(&gServerSocket, &pPacket, NX_WAIT_FOREVER);
        EnetAppUtils_assert((status == NX_SUCCESS) || (status == NX_NOT_CONNECTED));

        if (status == NX_NOT_CONNECTED) {
            break;
        }

        /* Send the packet out!  */
        status = nx_tcp_socket_send(&gServerSocket, pPacket, NX_IP_PERIODIC_RATE);
        EnetAppUtils_assert((status == NX_SUCCESS) || (status == NX_NOT_CONNECTED));

        if (status == NX_NOT_CONNECTED) {
            break;
        }
    }
    DebugP_log("Done\r\n");


    /* Disconnect this socket.  */
    status =  nx_tcp_server_socket_unaccept(&gServerSocket);
    EnetAppUtils_assert(status == NX_SUCCESS);

    /* Unbind the socket.  */
    status =  nx_tcp_server_socket_unlisten(&gIp, SERVER_PORT);
    EnetAppUtils_assert(status == NX_SUCCESS);

    DebugP_log("Connection closed\r\n");

    /* Delete the socket.  */
    status =  nx_tcp_socket_delete(&gServerSocket);
    EnetAppUtils_assert(status == NX_SUCCESS);

    DebugP_log("Socket deleted\r\n");

    tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);

    return (0);
}



static void EnetApp_addMCastEntry(Enet_Type enetType, uint32_t instId, uint32_t coreId, const uint8_t *testMCastAddr, uint32_t portMask)
{
    Enet_IoctlPrms prms;
    int32_t status;
    CpswAle_SetMcastEntryInArgs setMcastInArgs;
    uint32_t setMcastOutArgs;

    if (Enet_isCpswFamily(enetType))
    {
        Enet_Handle hEnet = Enet_getHandle(enetType, instId);

        EnetAppUtils_assert(hEnet != NULL);
        memset(&setMcastInArgs, 0, sizeof(setMcastInArgs));
        memcpy(&setMcastInArgs.addr.addr[0U], testMCastAddr,
               sizeof(setMcastInArgs.addr.addr));
        setMcastInArgs.addr.vlanId  = 0;
        setMcastInArgs.info.super = false;
        setMcastInArgs.info.numIgnBits = 0;
        setMcastInArgs.info.fwdState = CPSW_ALE_FWDSTLVL_FWD;
        setMcastInArgs.info.portMask = portMask;
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &setMcastInArgs, &setMcastOutArgs);
        ENET_IOCTL(hEnet, coreId, CPSW_ALE_IOCTL_ADD_MCAST, &prms, status);
        if (status != ENET_SOK)
        {
           EnetAppUtils_print("EnetTestBcastMcastLimit_AddAleEntry() failed CPSW_ALE_IOCTL_ADD_MCAST: %d\n",
                               status);
        }
    }
}

static void EnetApp_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                         void *appArg)
{
    if (info->linkChanged)
    {
        EnetAppUtils_print("Link Status Changed. PHY: 0x%x, state: %s\r\n",
                info->phyAddr,
                info->isLinked? "up" : "down");
    }
}

static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp,
                                           void *appArg)
{
    EnetAppUtils_print("MAC Port %u: link %s\r\n",
                       ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
    notify_linkchange();
}

static int EnetApp_initTsn(Enet_Type enetType, uint32_t instId, Enet_MacPort macPortList[], uint8_t numMacPort)
{
    lld_ethdev_t ethdevs[MAX_NUMBER_ENET_DEVS] = {0};
    EnetApp_GetMacAddrOutArgs outArgs;
    int i;
    int res = 0;
    AppTsnCfg_t appCfg =
    {
        .consoleOutCb = ConsolePrint,
    };

    for (i = 0; i < numMacPort; i++)
    {
        snprintf(&g_netdevices[i][0], CB_MAX_NETDEVNAME, "tilld%d", i);
        appCfg.netdevs[i] = &g_netdevices[i][0];
        ethdevs[i].netdev = g_netdevices[i];
        ethdevs[i].macport = macPortList[i];
        if (i == 0)
        {
            /* tilld0 reuses the allocated source mac, other interfaces will allocate
             * the mac by themself */
            EnetApp_getMacAddress(ENET_DMA_RX_CH0, &outArgs);
            EnetAppUtils_assert(outArgs.macAddressCnt == 1);
            memcpy(ethdevs[i].srcmac, &outArgs.macAddr[0][0], ENET_MAC_ADDR_LEN);
        }
    }
    appCfg.netdevs[i] = NULL;
    if (EnetApp_initTsnByCfg(&appCfg) < 0)
    {
        EnetAppAbort("Failed to int tsn!\r\n");
    }
    if (cb_lld_init_devs_table(ethdevs, i, enetType, instId) < 0)
    {
        EnetAppAbort("Failed to int devs table!\r\n");
    }
    cb_socket_set_lldcfg_update_cb(EnetApp_lldCfgUpdateCb);

    if (EnetApp_startTsn() < 0)
    {
        EnetAppAbort("Failed to start TSN App!\r\n");
    }
    EnetAppUtils_print("%s:TSN app start done!\r\n", __func__);

    return res;
}


static void EnetApp_enableTsSync(Enet_Type enetType, uint32_t instId)
{
    Enet_IoctlPrms prms;
    CpswCpts_OutputBitSel bitSelect;
    int32_t status;

    Enet_Handle hEnet = Enet_getHandle(enetType, instId);
    bitSelect = CPSW_CPTS_TS_OUTPUT_BIT_24;
    ENET_IOCTL_SET_IN_ARGS(&prms, &bitSelect);
    ENET_IOCTL(hEnet, EnetSoc_getCoreId(), CPSW_CPTS_IOCTL_SELECT_TS_OUTPUT_BIT, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to set TS SYNC OUT BIT : %d\r\n", status);
    }
    return;
}


static bool IsMacAddrSet(uint8_t *mac)
{
    return ((mac[0]|mac[1]|mac[2]|mac[3]|mac[4]|mac[5]) != 0);
}


static void ConsolePrint(const char *pcString, ...)
{
    /* Use DebugP_log() because EnetAppUtils_print() has limit bufsize */
    va_list args;
    char buffer[LOG_BUFFER_SIZE];

    va_start(args, pcString);
    vsnprintf(buffer, sizeof(buffer), pcString, args);
    va_end(args);

    DebugP_log("%s", buffer);
}


uint32_t EnetApp_applyClassifier(Enet_Handle hEnet, uint32_t coreId, uint8_t *dstMacAddr, uint32_t vlanId,
                                    uint32_t ethType, uint32_t rxFlowIdx)
{
    Enet_IoctlPrms prms;
    CpswAle_SetPolicerEntryOutArgs setPolicerEntryOutArgs;
    CpswAle_SetPolicerEntryInArgs setPolicerEntryInArgs;
    int32_t status;

    if (IsMacAddrSet(dstMacAddr) == true)
    {
        status = EnetAppUtils_addAllPortMcastMembership(hEnet, dstMacAddr);
        if (status != ENET_SOK) {
            EnetAppUtils_print("EnetAppUtils_addAllPortMcastMembership failed: %d\r\n", status);
        }
    }
    memset(&setPolicerEntryInArgs, 0, sizeof (setPolicerEntryInArgs));

    if (ethType > 0) {
        setPolicerEntryInArgs.policerMatch.policerMatchEnMask |=
            CPSW_ALE_POLICER_MATCH_ETHERTYPE;
        setPolicerEntryInArgs.policerMatch.etherType = ethType;
    }
    setPolicerEntryInArgs.policerMatch.portIsTrunk = false;
    setPolicerEntryInArgs.threadIdEn = true;
    setPolicerEntryInArgs.threadId = rxFlowIdx;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setPolicerEntryInArgs, &setPolicerEntryOutArgs);
    ENET_IOCTL(hEnet, coreId,
            CPSW_ALE_IOCTL_SET_POLICER, &prms, status);
    return status;
}


int32_t EnetApp_filterPriorityPacketsCfg(Enet_Handle hEnet, uint32_t coreId)
{
    EnetMacPort_SetPriorityRegenMapInArgs params;
    Enet_IoctlPrms prms;
    int32_t retVal = ENET_SOK;

    params.macPort = ENET_MAC_PORT_1;

    params.priorityRegenMap.priorityMap[0] =0U;
    for (int i = 1; i < 8U; i++)
    {
        params.priorityRegenMap.priorityMap[i] =1U;  // Map all priorities from (1 to 7) to priority 1, these packets will be received on DMA channel 1.
    }

    ENET_IOCTL_SET_IN_ARGS(&prms, &params);

    ENET_IOCTL(hEnet, coreId, ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP, &prms, retVal);

    return retVal;
}


void EnetApp_updateCpswInitCfg(Enet_Type enetType, uint32_t instId, Cpsw_Cfg *cpswCfg)
{
#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_CPDMA)
    EnetCpdma_Cfg * dmaCfg = (EnetCpdma_Cfg *)cpswCfg->dmaCfg;

    EnetAppUtils_assert(dmaCfg != NULL);
    EnetAppUtils_assert(EnetAppUtils_isDescCached() == false);
    dmaCfg->rxInterruptPerMSec = 8;
    dmaCfg->txInterruptPerMSec = 2;
    dmaCfg->enChOverrideFlag = true;
#endif


#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
    cpswCfg->mdioLinkStateChangeCb    = NULL;
    cpswCfg->mdioLinkStateChangeCbArg = NULL;
#else
    cpswCfg->mdioLinkStateChangeCb    = &EnetApp_mdioLinkStatusChange;
    cpswCfg->mdioLinkStateChangeCbArg = NULL;
#endif
    cpswCfg->portLinkStatusChangeCb = &EnetApp_portLinkStatusChangeCb;
    cpswCfg->portLinkStatusChangeCbArg = NULL;
}

