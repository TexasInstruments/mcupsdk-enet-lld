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
 * \file  tcp_config.c
 *
 * \brief This file configures NetIf and TCP Lwip-stack
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
/* lwIP core includes */
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"

#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/ClockP.h>
#include <enet_apputils.h>
#include <enet_board.h>
#include "ti_board_config.h"
#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"
#include "ti_enet_config.h"
#include "ti_enet_open_close.h"

#include "ti_enet_lwipif.h"
#include "../cpsw_config/etherring_cpsw_config.h"
#include "tcp_config.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define IP_ADDR_POOL_COUNT  (2U)

#define ENETAPP_MAX_IPV4_STRING_LEN (20U)

ip_addr_t gStaticIP[IP_ADDR_POOL_COUNT]        =  { IPADDR4_INIT_BYTES(192, 168, 1, 100), /* For NetifIdx = 0 */
                                                                                                       IPADDR4_INIT_BYTES(  10,  64,  1, 100),}; /* For NetifIdx = 1 */

const ip_addr_t gStaticIPGateway[IP_ADDR_POOL_COUNT] =  { IPADDR4_INIT_BYTES(192, 168, 1, 1), /* For NetifIdx = 0 */
                                                                                                       IPADDR4_INIT_BYTES(  10,   64, 1, 1),}; /* For NetifIdx = 1 */

const ip_addr_t gStaticIPNetmask[IP_ADDR_POOL_COUNT] =  { IPADDR4_INIT_BYTES(255,255,255,0), /* For NetifIdx = 0 */
                                                                                                       IPADDR4_INIT_BYTES(255,255,252,0),}; /* For NetifIdx = 1 */
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void EnetApp_tcpipInitCompleteCb(void *pArg);

static void EnetApp_setupNetif();

static void EnetApp_allocateIPAddress();

static void EnetApp_shutdownNetworkStack();

static void EnetApp_netifStatusChangeCb(struct netif *state_netif);

static void EnetApp_netifLinkChangeCb(struct netif *state_netif);

void EnetApp_setupNetworkStack();

int32_t EnetApp_isNetworkUp(struct netif* pnetif);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* dhcp struct for the ethernet netif */
static struct dhcp gNetifDhcp[ENET_SYSCFG_NETIF_COUNT];

/* netif array for the syscfg enabled netif */
struct netif *gNetif[ENET_SYSCFG_NETIF_COUNT];

/* Handle to the Application interface for the LwIPIf Layer */
LwipifEnetApp_Handle hLwipIfApp = NULL;

extern EnetApp_Cfg gEnetAppCfg;

char gHostServerIp4[ENETAPP_MAX_IPV4_STRING_LEN] = "192.168.1.";

uint16_t gServerPort = 8888;
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
void EnetApp_fillHostSocketInfo(struct EnetApp_hostInfo* pHostInfo)
{
    char buffer[5];
    int32_t addr_ok;

    sprintf(buffer, "%d", 100 + gEnetAppCfg.nodeId - 1);
    strcat(gHostServerIp4, buffer);

    EnetAppUtils_print(" Connection to serverIp: %s \r\n", gHostServerIp4);
    gServerPort = gServerPort - (gEnetAppCfg.nodeId / 2);
    pHostInfo->port = gServerPort;
    memset(&pHostInfo->ipAddr, 0, sizeof(pHostInfo->ipAddr));
    ip_addr_t*  pAddr = &pHostInfo->ipAddr;
    IP_SET_TYPE_VAL(*pAddr, IPADDR_TYPE_V4);
    addr_ok = ip4addr_aton(gHostServerIp4, ip_2_ip4(pAddr));
    EnetAppUtils_assert(addr_ok);
    return;
}

void EnetApp_setupNetworkStack()
{
    sys_sem_t pInitSem;
    const err_t status = sys_sem_new(&pInitSem, 0);
    EnetAppUtils_assert(status == ERR_OK);

    tcpip_init(EnetApp_tcpipInitCompleteCb, &pInitSem);

    /* wait for TCP/IP initialization to complete */
    sys_sem_wait(&pInitSem);
    sys_sem_free(&pInitSem);

    return;
}

static void EnetApp_shutdownNetworkStack()
{
    LwipifEnetApp_netifClose(hLwipIfApp, NETIF_INST_ID0);
    return;
}

static void EnetApp_tcpipInitCompleteCb(void *pArg)
{
    sys_sem_t *pSem = (sys_sem_t*)pArg;
    EnetAppUtils_assert(pArg != NULL);

    /* init randomizer again (seed per thread) */
    srand((unsigned int)sys_now()/1000);

    EnetApp_setupNetif();

    EnetApp_allocateIPAddress();

    sys_sem_signal(pSem);
}

static void EnetApp_setupNetif()
{
    ip4_addr_t ipaddr, netmask, gw;

    ip4_addr_set_zero(&gw);
    ip4_addr_set_zero(&ipaddr);
    ip4_addr_set_zero(&netmask);

    DebugP_log("Starting lwIP, local interface IP is dhcp-enabled\r\n");
    hLwipIfApp = LwipifEnetApp_getHandle();

    for (uint32_t netifIndex = 0U; netifIndex < ENET_SYSCFG_NETIF_COUNT; netifIndex++)
    {
        gStaticIP[netifIndex].addr += (uint32_t)((uint8_t)gEnetAppCfg.nodeId << 24);

        /* Assigning StaticIp to the Interface */
        ip4_addr_set(&ipaddr, & gStaticIP[netifIndex]);
        ip4_addr_set(&gw, & gStaticIPGateway[netifIndex]);
        ip4_addr_set(&netmask, & gStaticIPNetmask[netifIndex]);

        /* Open the netif and get it populated*/
        gNetif[netifIndex] = LwipifEnetApp_netifOpen(hLwipIfApp, NETIF_INST_ID0 + netifIndex, &ipaddr, &netmask, &gw);
        netif_set_status_callback(gNetif[netifIndex], EnetApp_netifStatusChangeCb);
        netif_set_link_callback(gNetif[netifIndex], EnetApp_netifLinkChangeCb);
        netif_set_up(gNetif[NETIF_INST_ID0 + netifIndex]);
    }
    LwipifEnetApp_startSchedule(hLwipIfApp, gNetif[ENET_SYSCFG_DEFAULT_NETIF_IDX]);
}

static void EnetApp_allocateIPAddress()
{
    sys_lock_tcpip_core();
    for (uint32_t  netifIndex = 0U; netifIndex < ENET_SYSCFG_NETIF_COUNT; netifIndex++)
    {
        dhcp_set_struct(gNetif[NETIF_INST_ID0 + netifIndex], &gNetifDhcp[NETIF_INST_ID0 + netifIndex]);

        const err_t err = dhcp_start(gNetif[NETIF_INST_ID0 + netifIndex]);
        EnetAppUtils_assert(err == ERR_OK);
    }
    sys_unlock_tcpip_core();
    return;
}

static void EnetApp_netifStatusChangeCb(struct netif *pNetif)
{
    if (netif_is_up(pNetif))
    {
        DebugP_log("Enet IF UP Event. Local interface IP:%s\r\n",
                    ip4addr_ntoa(netif_ip4_addr(pNetif)));
    }
    else
    {
        DebugP_log("Enet IF DOWN Event\r\n");
    }
    return;
}

static void EnetApp_netifLinkChangeCb(struct netif *pNetif)
{
    if (netif_is_link_up(pNetif))
    {
        DebugP_log("Network Link UP Event\r\n");
    }
    else
    {
        DebugP_log("Network Link DOWN Event\r\n");
    }
    return;
}

int32_t EnetApp_isNetworkUp(struct netif* pNetif)
{
    int32_t status = ENET_SOK;

    status = (netif_is_up(pNetif) &&
             netif_is_link_up(pNetif) &&
             !ip4_addr_isany_val(*netif_ip4_addr(pNetif)));
    return status;
}

