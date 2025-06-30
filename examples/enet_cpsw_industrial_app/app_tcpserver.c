/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "app_tcpserver.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
typedef struct {
    sys_sem_t pSem;
    EnetApp_TcpObj* hTcp;
} TcpInitCb_Arg;
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void TcpApp_tcpipInitCompleteCb(void *pArg);

static void TcpApp_setupNetif(EnetApp_TcpObj* hTcp);

static void TcpApp_allocateIPAddress(EnetApp_TcpObj* hTcp);

static void TcpApp_setupNetworkStack(EnetApp_TcpObj* hTcp);

static void TcpApp_netifStatusChangeCb(struct netif *state_netif);

static void TcpApp_netifLinkChangeCb(struct netif *state_netif);

static inline int32_t TcpApp_isNetworkUp(struct netif* netif_);

static void TcpApp_echoPckt(struct netconn *pClientConn);

static void TcpApp_ServerTask(void *arg);

static void TcpApp_startServer();

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static const uint8_t APP_CLIENT_TX_MSG1[] = "Greetings from Texas Instruments!";

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
void TcpApp_setup(EnetApp_TcpObj* hTcp){

    TcpApp_setupNetworkStack(hTcp);
    uint32_t netupMask = 0;
    /* wait for atleast one Network Interface to get IP */
    while (netupMask == 0)
    {
        for(uint32_t netifIdx = 0; netifIdx < ENET_SYSCFG_NETIF_COUNT; netifIdx++)
        {
            if (TcpApp_isNetworkUp(hTcp->sNetif[netifIdx]))
            {
                netupMask |= (1 << netifIdx);
            }
            else
            {
                DebugP_log("[%d]Waiting for network UP ...\r\n",hTcp->sNetif[netifIdx]->num);
            }
            ClockP_sleep(2);
        }
    }

    DebugP_log("Network is UP ...\r\n");
    ClockP_sleep(2);

    /*creates tasks for TCP/IP*/ 
    TcpApp_startServer();
}

static void TcpApp_setupNetworkStack(EnetApp_TcpObj* hTcp)
{
    TcpInitCb_Arg hTcp_initCbArg;
    hTcp_initCbArg.hTcp = hTcp;
    sys_sem_t pInitSem;
    const err_t err = sys_sem_new(&pInitSem, 0);
    EnetAppUtils_assert(err == ERR_OK);
    hTcp_initCbArg.pSem = pInitSem;

    tcpip_init(TcpApp_tcpipInitCompleteCb, &hTcp_initCbArg);

    /* wait for TCP/IP initialization to complete */
    sys_sem_wait(&pInitSem);
    sys_sem_free(&pInitSem);

    return;
}

static void TcpApp_tcpipInitCompleteCb(void *pArg)
{
    TcpInitCb_Arg* hTcp_initCbArg = (TcpInitCb_Arg*)(pArg);
    sys_sem_t *pSem = (sys_sem_t*)(&hTcp_initCbArg->pSem);

    EnetAppUtils_assert(hTcp_initCbArg != NULL);
    EnetAppUtils_assert(pSem != NULL);

    /* init randomizer again (seed per thread) */
    srand((unsigned int)sys_now()/1000);

    TcpApp_setupNetif((EnetApp_TcpObj*)(hTcp_initCbArg->hTcp));

    TcpApp_allocateIPAddress(hTcp_initCbArg->hTcp);

    sys_sem_signal(pSem);
}

static void TcpApp_setupNetif(EnetApp_TcpObj* hTcp)
{
    ip4_addr_t ipaddr, netmask, gw;

    ip4_addr_set_zero(&gw);
    ip4_addr_set_zero(&ipaddr);
    ip4_addr_set_zero(&netmask);

    DebugP_log("Starting lwIP, local interface IP is dhcp-enabled\r\n");
    hTcp->hlwipIfApp = LwipifEnetApp_getHandle();

    /*todo : Setting the static IP only for one netif. Need to updated based on usecase */
    for (uint32_t netifIdx = 0U; netifIdx < ENET_SYSCFG_NETIF_COUNT; netifIdx++)
    {
        ip4_addr_set(&ipaddr, &hTcp->staticIP[netifIdx]);
        ip4_addr_set(&gw, &hTcp->staticIPGateway[netifIdx]);
        ip4_addr_set(&netmask, &hTcp->staticIPNetmask[netifIdx]);
        DebugP_log("[%d]: Static IP with LWIP-Stack is %s\r\n", netifIdx, ip4addr_ntoa(&ipaddr));

        /* Open the netif and get it populated*/
        hTcp->sNetif[netifIdx] = LwipifEnetApp_netifOpen(hTcp->hlwipIfApp, NETIF_INST_ID0 + netifIdx, &ipaddr, &netmask, &gw);
        netif_set_status_callback(hTcp->sNetif[netifIdx], TcpApp_netifStatusChangeCb);
        netif_set_link_callback(hTcp->sNetif[netifIdx], TcpApp_netifLinkChangeCb);
        netif_set_up(hTcp->sNetif[NETIF_INST_ID0 + netifIdx]);
    }
    LwipifEnetApp_startSchedule(hTcp->hlwipIfApp, hTcp->sNetif[ENET_SYSCFG_DEFAULT_NETIF_IDX]);
}

static void TcpApp_allocateIPAddress(EnetApp_TcpObj* hTcp)
{
    sys_lock_tcpip_core();
    for (uint32_t  netifIdx = 0U; netifIdx < ENET_SYSCFG_NETIF_COUNT; netifIdx++)
    {
        dhcp_set_struct(hTcp->sNetif[NETIF_INST_ID0 + netifIdx], &hTcp->sDhcp[NETIF_INST_ID0 + netifIdx]);

        const err_t err = dhcp_start(hTcp->sNetif[NETIF_INST_ID0 + netifIdx]);
        EnetAppUtils_assert(err == ERR_OK);
    }
    sys_unlock_tcpip_core();
    return;
}

static void TcpApp_netifStatusChangeCb(struct netif *pNetif)
{
    if (netif_is_up(pNetif))
    {
       DebugP_log("[%d]Enet IF UP Event. Local interface IP:%s\r\n",
                   pNetif->num, ip4addr_ntoa(netif_ip4_addr(pNetif)));
    }
    else
    {
        DebugP_log("[%d]Enet IF DOWN Event\r\n", pNetif->num);
    }
    return;
}

static void TcpApp_netifLinkChangeCb(struct netif *pNetif)/*fine*/
{
    if (netif_is_link_up(pNetif))
    {
        DebugP_log("[%d]Network Link UP Event\r\n", pNetif->num);
    }
    else
    {
        DebugP_log("[%d]Network Link DOWN Event\r\n", pNetif->num);
    }
    return;
}

static int32_t TcpApp_isNetworkUp(struct netif* netif_)/*fine*/
{
    return (netif_is_up(netif_) && netif_is_link_up(netif_) && !ip4_addr_isany_val(*netif_ip4_addr(netif_)));
}

static void TcpApp_echoPckt(struct netconn *pClientConn)/*fine*/
{
    struct netbuf *buf;
    void *data;
    u16_t len;
    err_t err;

    while ((err = netconn_recv(pClientConn, &buf)) == ERR_OK)
    {
        err = netconn_write(pClientConn, APP_CLIENT_TX_MSG1, sizeof(APP_CLIENT_TX_MSG1), NETCONN_COPY);
        if (err != ERR_OK)
        {
            printf("tcpecho: netconn_write: error \"%s\"\r\n", lwip_strerr(err));
            break;
        }
        do
        {
            netbuf_data(buf, &data, &len);
            err = netconn_write(pClientConn, data, len, NETCONN_COPY);
            if (err != ERR_OK)
            {
                printf("tcpecho: netconn_write: error \"%s\"\r\n", lwip_strerr(err));
            }

        } while (netbuf_next(buf) >= 0);
        netbuf_delete(buf);
    }
}

static void TcpApp_ServerTask(void *arg)
{
    struct netconn *pConn = NULL, *pClientConn = NULL;
    err_t err;
    LWIP_UNUSED_ARG(arg);

    pConn = netconn_new(NETCONN_TCP);
    netconn_bind(pConn, IP_ADDR_ANY, APP_SERVER_PORT);
    LWIP_ERROR("tcpecho: invalid conn\r\n", (pConn != NULL), return;);

    /* Tell connection to go into listening mode. */
    netconn_listen(pConn);

    while (1)
    {

        /* Grab new connection. */
        err = netconn_accept(pConn, &pClientConn);
        printf("accepted new connection %p\r\n", pClientConn);

        /* Process the new connection. */
        if (err < ERR_OK)
        {
            DebugP_log("Unable to accept connection: errno %d\r\n", err);
            break;
        }

        TcpApp_echoPckt(pClientConn);

        /* Close connection and discard connection identifier. */
        netconn_close(pClientConn);
        netconn_delete(pClientConn);
    }
}

void TcpApp_startServer()
{
    sys_thread_new("TcpApp_ServerTask", TcpApp_ServerTask, NULL, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
}
/*-----------------------------------------------------------------------------------*/