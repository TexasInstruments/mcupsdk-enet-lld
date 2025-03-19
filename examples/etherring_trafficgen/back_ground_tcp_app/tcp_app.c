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
#include <string.h>
#include <stdio.h>
#include <string.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "enet_apputils.h"
#include "lwip/priv/tcp_priv.h"
#include "lwip/ip4_addr.h"
#include "../cpsw_config/etherring_cpsw_config.h"
#include "tcp_config.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ENETAPP_MAX_RX_DATA_LEN     (1020U)

#define ENETAPP_IP_ADDR_POOL_COUNT  (2U)

#define ENETAPP_LWIP_TCP_DEBUG

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static struct EnetApp_hostInfo gHostInfo;

static char gSndBuf[ENETAPP_MAX_RX_DATA_LEN];
static uint32_t gTxClientSentCount = 0;
static uint32_t gTxClientSentFailCount = 0;
static uint32_t gTcpServerRxCount = 0;
extern EnetApp_Cfg gEnetAppCfg;
extern ip_addr_t gStaticIP[ENETAPP_IP_ADDR_POOL_COUNT];
extern char gHostServerIp4[20U];
extern uint16_t gServerPort;
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void EnetApp_tcpClient(void *pArg)
{
    struct netconn *pConn = NULL;
    err_t status = ERR_OK, connectError = ERR_OK;
    struct EnetApp_hostInfo* pHostInfo = (struct EnetApp_hostInfo*) pArg;

    const enum netconn_type connType = NETCONN_TCP;

    /* Create a new connection identifier. */
    pConn = netconn_new(connType);
    if (pConn != NULL)
    {
        ClockP_sleep(5);
        /* Connect to the TCP Server */
        EnetAppUtils_print(" Connecting to: %s:%d \r\n",gHostServerIp4 , gServerPort);
        connectError = netconn_connect(pConn, &pHostInfo->ipAddr, pHostInfo->port);
        if (connectError != ERR_OK)
        {
            netconn_close(pConn);
            DebugP_log("Connection with the server isn't established\r\n");

        }
        else
        {
#ifdef ENETAPP_LWIP_TCP_DEBUG
            DebugP_log("Connection with the server is established\r\n");
#endif
        }

        /* fills the buffer */
        memset(&gSndBuf, 0, ENETAPP_MAX_RX_DATA_LEN);

        while(1)
        {
            SemaphoreP_pend(&gEnetAppCfg.lwipSemObj, SystemP_WAIT_FOREVER);

            status = netconn_write(pConn, gSndBuf, ENETAPP_MAX_RX_DATA_LEN, NETCONN_COPY);

            if (status == ERR_OK)
            {
                gTxClientSentCount++;
            }
            else
            {
                gTxClientSentFailCount++;
            }
        }
    }
}


void EnetApp_startTcpClient(void)
{
    EnetApp_fillHostSocketInfo(&gHostInfo);
    sys_thread_new("tcpinit_thread", EnetApp_tcpClient, &gHostInfo, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
}

static void EnetApp_receivePkt(struct netconn *pClientConn)
{
    struct netbuf *buffer;
    void *data;
    uint16_t length;
    err_t status = ERR_OK;

    while ((status = netconn_recv(pClientConn, &buffer)) == ERR_OK)
    {
        do
        {
            netbuf_data(buffer, &data, &length);
            gTcpServerRxCount++;
        } while (netbuf_next(buffer) >= 0);
        netbuf_delete(buffer);
    }
}

static void EnetApp_tcpServerTask(void *arg)
{
    struct netconn *pConn = NULL, *pClientConn = NULL;
    err_t status = ERR_OK;
    LWIP_UNUSED_ARG(arg);

    gServerPort = gServerPort - (gEnetAppCfg.nodeId/2);
    pConn = netconn_new(NETCONN_TCP);
    netconn_bind(pConn, IP_ADDR_ANY, gServerPort);
    LWIP_ERROR("tcpecho: invalid conn\r\n", (pConn != NULL), return;);

    /* Tell connection to go into listening mode. */
    netconn_listen(pConn);
    printf("Listening on %s:%d\r\n", ip4addr_ntoa((const ip4_addr_t *)&gStaticIP[0]), gServerPort);
    while (1)
    {
        /* Grab new connection. */
        status = netconn_accept(pConn, &pClientConn);
#ifdef ENETAPP_LWIP_TCP_DEBUG
         printf("accepted new connection %p\r\n", pClientConn);
#endif

        /* Process the new connection. */
        if (status < ERR_OK)
        {
            DebugP_log("Unable to accept connection: errno %d\r\n", status);
            break;
        }

        EnetApp_receivePkt(pClientConn);

        /* Close connection and discard connection identifier. */
        netconn_close(pClientConn);
        netconn_delete(pClientConn);
    }
}

void EnetApp_startTcpServer()
{
    sys_thread_new("EnetApp_tcpServerTask", EnetApp_tcpServerTask, NULL, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
}
