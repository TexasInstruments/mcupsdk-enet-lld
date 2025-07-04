//###########################################################################
//
// FILE:   lwip2lwipif_ic.c
//
// TITLE:  lwIP Interface port file.
//
//###########################################################################
// $TI Release: $
// $Release Date: $
// $Copyright: $
//###########################################################################

/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
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

/**
 *  Copyright (c) Texas Instruments Incorporated 2024
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
 *
 * This file is dervied from the ``ethernetif.c'' skeleton Ethernet network
 * interface driver for lwIP.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* Standard language headers */
#include <stdio.h>
#include <stdbool.h>

#include <networking/enet/core/include/core/enet_osal.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/DebugP.h>


/* lwIP specific header files */
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include <lwip/stats.h>
#include <lwip/snmp.h>
#include <lwip/netifapi.h>
#include "netif/etharp.h"
#include "netif/ppp/pppoe.h"

/* This module's header */
#include <../../intercore/intercore.h>
#include <lwip_ic.h>
#include <lwip2lwipif_ic.h>
#include <../../intercore/include/ShdMemCircularBufferP_nortos.h>
#include "lwipopts.h"

/* ========================================================================== */
/*                             Local Macros                                   */
/* ========================================================================== */

/* Define those to better describe your network interface. */
#define IFNAME0 'i'
#define IFNAME1 'c'

#define OS_TASKPRIHIGH               (7)

#define LWIP_RX_PACKET_TASK_PRI      (TCPIP_THREAD_PRIO + 1)

#if defined (SAFERTOS)
#define LWIP_RX_PACKET_TASK_STACK_SIZE     (16U * 1024)
#define LWIP_RX_PACKET_TASK_STACK_ALIGN    LWIP_RX_PACKET_TASK_STACK_SIZE
#else
#define LWIP_RX_PACKET_TASK_STACK_SIZE     (4096)
#define LWIP_RX_PACKET_TASK_STACK_ALIGN    (32)
#endif

/* Maximum Ethernet Payload Size. */
#define ETH_MAX_PAYLOAD             (1514)

#define VLAN_TAG_SIZE               (4U)
#define ETH_FRAME_SIZE              (ETH_MAX_PAYLOAD + VLAN_TAG_SIZE)

/* Ethernet Header */
#define ETHHDR_SIZE                 (14)
#define MAX_SG_SEGMENT_COUNT        (4U)

/* ========================================================================== */
/*                            Local Variables                                 */
/* ========================================================================== */

static uint8_t gLwip2LwipIf_rxTaskStack[IC_ETH_MAX_VIRTUAL_IF][LWIP_RX_PACKET_TASK_STACK_SIZE]
__attribute__ ((aligned(LWIP_RX_PACKET_TASK_STACK_ALIGN)));

/* ========================================================================== */
/*                          Extern variables                                  */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                    Local Function Declarations                             */
/* ========================================================================== */

 static void LwipIc_SetSGList(struct pbuf *pbuf, uint32_t* arrLen, void* pDataArr[], uint32_t* dataLenArr);

 /* ========================================================================== */
 /*                    Local Function Definitions                             */
 /* ========================================================================== */

/*!
 *  @b LWIPIF_LWIP_IC_send
 *  @n
 *  This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 *  \param[in]  netif
 *      The lwip network interface structure for this ethernetif
 *  \param[in]  p
 *      the MAC packet to send (e.g. IP packet including MAC addresses and type)
 *
 *  \retval
 *      ERR_OK if the packet could be sent
 *  \retval
 *      an err_t value if the packet couldn't be sent
 */
static err_t LWIPIF_LWIP_IC_send(struct netif *netif,
                                 struct pbuf *pStackBuf)
{
    int32_t status = ERR_OK;
    uint32_t arrLen = 0U;
    uint32_t dataLenArr[MAX_SG_SEGMENT_COUNT];
    void* pDataArr[MAX_SG_SEGMENT_COUNT];

    LwipIc_Handle hLwipIc;
    Ic_Object_Handle   hIcObj;

    /* Get the pointer to the private data */
    hLwipIc = (LwipIc_Handle)netif->state;
    hIcObj  = hLwipIc->hIcObj;

    /* Return if initialization if not complete */
    if(!hLwipIc->initDone)
    {
        LwipIcStats_addOne(&hLwipIc->stats.txStats.pktDropInitErr);
        status = ERR_IF;
        goto error_handling;
    }

    /* Enqueue the packet */
    if(pStackBuf == NULL)
    {
        hLwipIc->stats.txStats.pbufNullErr++;
        status = ERR_BUF;
        goto error_handling;
    }

    /* Get the pbuf payload in one contigous buffer */
    if((ShdMemCircularBufferP_isQFull(hIcObj->shmTxQ) == true))
    {
        LwipIcStats_addOne(&hLwipIc->stats.txStats.txQFullErr);
        status = ERR_BUF;
        goto error_handling;
    }
    /*
     * When transmitting a packet, the buffer may be deleted before transmission by the
     * stack. The stack implements a 'ref' feature within the buffers. The following happens
     * internally:
     *  If p->ref > 1, ref--;
     *  If p->ref == 1, free(p);
     * pbuf_ref(p) increments the ref.
     */
    pbuf_ref(pStackBuf);
    LwipIcStats_addOne(&hLwipIc->stats.driverBufGet);

    /* If the pbuf chain contains only one buffer, then we need to manually
     * copy the payload into our buffer. This is because in this case
     * pbuf_get_contiguous returns a pointer to the payload instead of
     * copying the payload into the user supplied buffer
     */

    LwipIc_SetSGList(pStackBuf, &arrLen, pDataArr, dataLenArr);

    status = ShdMemCircularBufferP_writeElem(hIcObj->shmTxQ, arrLen,
                                             pDataArr,dataLenArr);
    if(status != SHDMEM_CIRCULAR_BUFFER_STATUS_OK)
    {
        LwipIcStats_addOne(&hLwipIc->stats.txStats.txQFullErr);
        status = ERR_BUF;
        goto error_handling;
    }

    //Cache op here

    if(SHDMEM_CIRCULAR_BUFFER_STATUS_OK == status)
    {
        pbuf_free(pStackBuf);
        hLwipIc->stats.txStats.pktEnq++;
        hIcObj->numTxPktsSent++;
#if !(IC_ETH_RX_POLLING_MODE)
        if(0 == (hIcObj->numTxPktsSent % hIcObj->pktNotifyThresh))
        {
            /* Prepare and send a notification to the receiver */
            retVal = LwipIc_remoteCorePktNotify(hLwipIc);
            LwipIc_assert(retVal == LWIPIC_OK); 
        }
#endif
    }
    else
    {
        status = ERR_BUF;
        goto error_handling;
    }

error_handling:
    return status;
}


/*!
 *  @b LWIPIF_LWIP_IC_recv
 *  @n
 *  This is the RX task which consumes the packets retrieved from
 *  the driver and passes them to the LwIP stack via netif->input().
 *
 *  \param[in]  netif
 *      NETIF_DEVICE structure pointer.
 *
 *  \retval
 *      void
 */
static void LWIPIF_LWIP_IC_recv(void *arg0)
{
    int32_t status = ERR_OK;
    struct netif *netif = (struct netif*)arg0;
    LwipIc_Handle hLwipIc;
    struct pbuf *pStackBuf;
    bool unusedPkt = false;
    bool isEmpty = true;

    Ic_Object_Handle hIcObj;

    /* Get the pointer to the private data */
    hLwipIc = (LwipIc_Handle)netif->state;
    hIcObj  = hLwipIc->hIcObj;

    /* Wait for initialization to complete */
    while(!hLwipIc->initDone)
    {
        ClockP_usleep(100);
    }

    while(!hLwipIc->shutDownFlag)
    {
#if !(IC_ETH_RX_POLLING_MODE)
        SemaphoreP_pend(hLwipIc->hRxPacketSem, SystemP_WAIT_FOREVER);
#else
        SemaphoreP_pend(&hIcObj->rxSemObj, SystemP_WAIT_FOREVER);
//        ClockP_usleep(IC_ETH_RX_POLLING_INTERVAL*1000);
#endif

        // ToDo: replace with isQempty and get length
        isEmpty = ShdMemCircularBufferP_isQEmpty(hIcObj->shmRxQ);

        if(unusedPkt)
        {
            status = netif->input(pStackBuf, netif);
            /* Pass pBuf to the stack for processing */
            if (status != ERR_OK)
            {
                /* Break from loop to let other tasks to free MEMP buffers */
                LwipIcStats_addOne(&hLwipIc->stats.rxStats.netifInputErr);
                LwipIcStats_addOne(&hLwipIc->stats.driverBufFree);
                continue;
            }
            unusedPkt = false;
        }
        if(!isEmpty)
        {
            while(!isEmpty)
            {
                hIcObj->numRxPktsPending--;
                LwipIcStats_addOne(&hLwipIc->stats.rxStats.pktDeq);

                /* Allocate a pbuf chain from the pool. */
                pStackBuf = pbufQ_ic_deQ(&hIcObj->freePbufQ);
                if(pStackBuf == NULL)
                {
                    /* Break from loop to let other tasks to free pBufs */
                    LwipIcStats_addOne(&hLwipIc->stats.rxStats.pbufNullErr);
                    LwipIcStats_addOne(&hLwipIc->stats.driverBufNullErr);
                    break;
                }
                pbuf_ref(pStackBuf);

                /* Copy data into pbuf */
                status = ShdMemCircularBufferP_readElem(hIcObj->shmRxQ, &pStackBuf->len, pStackBuf->payload);
                if(ERR_OK != status)
                {
                    /* Free the pBuf, and try again to read from IC queue */
                    LwipIcStats_addOne(&hLwipIc->stats.rxStats.pbufTakeErr);
                    pbuf_free(pStackBuf);
                    pStackBuf = NULL;
                    LwipIcStats_addOne(&hLwipIc->stats.driverBufFree);
                    continue;
                }
                pStackBuf->tot_len = pStackBuf->len;

                /* Packet length check. */
                if(pStackBuf->len > ETH_FRAME_SIZE)
                {
                    /* Don't allow larger packets. Skip this packet */
                    pbuf_free(pStackBuf);
                    LwipIcStats_addOne(&hLwipIc->stats.rxStats.largePktErr);
                    LwipIcStats_addOne(&hLwipIc->stats.driverBufFree);
                    continue;
                }
                status = netif->input(pStackBuf, netif);
                /* Pass pBuf to the stack for processing */
                if (status != ERR_OK)
                {
                    /* Break from loop to let other tasks to free MEMP buffers */
                    /* ToDo: Add method to process unsent Tx packets taken out of IC queue*/
                    LwipIcStats_addOne(&hLwipIc->stats.rxStats.netifInputErr);
                    LwipIcStats_addOne(&hLwipIc->stats.driverBufFree);
                    if(status == ERR_MEM)
                    {
                        unusedPkt = true;
                        break;
                    }
                    else
                    {
                        pbuf_free(pStackBuf);
                        pStackBuf = NULL;
                        continue;
                    }
                }

                hLwipIc->stats.rxStats.pktSentToStk++;
                isEmpty = ShdMemCircularBufferP_isQEmpty(hIcObj->shmRxQ);
            }
        }
        else /* if Queue is empty */
        {
            LwipIcStats_addOne(&hLwipIc->stats.rxStats.rxQEmptyWarn);
        }


    } /* !hLwipIc->shutDownFlag */

    /* We are shutting down, notify that we are done */
    SemaphoreP_post(&hLwipIc->hShutDownSem);
}

/*!
 *  @b LWIPIF_LWIP_IC_start
 *  @n
 *  The function is used to initialize and start the intercore transport.
 *
 *  \param[in]  pNETIFDevice
 *      NETIF_DEVICE structure pointer.
 *
 *  \retval
 *      Success -   0
 *  \retval
 *      Error   -   <0
 */
int LWIPIF_LWIP_IC_start(struct netif *netif, uint32_t instId, Ic_Object_Handle hIcObj)
{
    int retVal = -1;
    int32_t status;
    LwipIc_Handle hLwipIc;
    TaskP_Params params;

    /* Open the translation layer, which opens the intercore driver */
    hLwipIc = LwipIc_open(instId, hIcObj);

    if (NULL != hLwipIc)
    {
        /* Save off a pointer to the translation layer */
        netif->state = (void *)hLwipIc;

        /* Create the LWIP_LWIPIF_input (i.e. packet RX) task */
        TaskP_Params_init(&params);
        params.name = (const char *)"Lwipif_Lwip_ic_recv";
        params.priority       = LWIP_RX_PACKET_TASK_PRI;
        params.stack          = gLwip2LwipIf_rxTaskStack[hLwipIc->instId];
        params.stackSize      = sizeof(gLwip2LwipIf_rxTaskStack[hLwipIc->instId]);
        params.args           = netif;
        params.taskMain       = &LWIPIF_LWIP_IC_recv;

        status = TaskP_construct(&hLwipIc->hLWIPIF2LWIPinput, &params);
        LwipIc_assert(SystemP_SUCCESS == status);

        /* Copy the MAC Address into the network interface object here. */
        memcpy(netif->hwaddr, &hLwipIc->macAddr, (uint32_t)6U);
        netif->hwaddr_len = 6U;

        netif_set_link_up(netif);

        /* Inform the world that we are operational. */
        hLwipIc->print("[LWIPIF_LWIP_IC] Interface started successfully \r\n");

        retVal = 0;
    }
    else
    {
        /* Note - Use System_printf here as we are not sure if Lwip2Enet print
         * is set and not null. */
    	LwipIc_printf("[LWIPIF_LWIP] Failed to start LwipIc \r\n");
    }

    return retVal;
}


/*!
 *  @b LWIPIF_LWIP_IC_stop
 *  @n
 *  The function is used to de-initialize and stop the LwipIc
 *  controller and device.
 *
 *  \param[in] netif
 *      NETIF structure pointer.
 */
static void LWIPIF_LWIP_IC_stop(struct netif *netif)
{
    LwipIc_Handle hLwipIc;

    /* Get the pointer to the private data */
    hLwipIc = (LwipIc_Handle)netif->state;

    /* Call low-level close function */
    LwipIc_close(hLwipIc);

}

static void LwipIc_SetSGList(struct pbuf *pbuf, uint32_t* pArrLen, void* pDataArr[], uint32_t* dataLenArr)
{
    struct pbuf *pbufNext = pbuf;
    uint32_t totalPacketFilledLen = 0U;

    *pArrLen = 0U;
    while (pbufNext != NULL)
    {
        LwipIc_assert(*pArrLen <= MAX_SG_SEGMENT_COUNT);
        pDataArr[*pArrLen] = (uint8_t*) pbufNext->payload;
        dataLenArr[*pArrLen] = pbufNext->len;
        totalPacketFilledLen += pbufNext->len;
        (*pArrLen)++;
        pbufNext = pbufNext->next;
    }
    LwipIc_assert(NULL != pbuf);
    LwipIc_assert(totalPacketFilledLen == pbuf->tot_len);
}

/* ========================================================================== */
/*                    API/Public Function Definitions                         */
/* ========================================================================== */

/*!
 *  @b LWIPIF_LWIP_IC_Init
 *  @n
 *  The function is used to initialize and register the peripheral
 *  with the stack.
 *
 *  \param[in]  *netif
 *      NETIF structure pointer
 *
 *  \retval
 *      Success -   ERR_OK
 */
err_t LWIPIF_LWIP_IC_init(struct netif *netif)
{

    /* Populate the Network Interface Object */
    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;

    /*
     * MTU is the total size of the (IP) packet that can fit into an Ethernet.
     * For Ethernet it is 1500bytes
     */
    netif->mtu = ETH_FRAME_SIZE - ETHHDR_SIZE - VLAN_TAG_SIZE;

    /* Populate the Driver Interface Functions. */
    netif->remove_callback      = LWIPIF_LWIP_IC_stop;
    netif->output               = etharp_output;
    netif->linkoutput           = LWIPIF_LWIP_IC_send;
    netif->flags               |= NETIF_FLAG_ETHARP;
    netif->flags               |= NETIF_FLAG_ETHERNET;
    netif->flags               |= NETIF_FLAG_IGMP;

    LwipIc_printf("[LWIPIF_LWIP_IC] NETIF INIT SUCCESS \r\n");

    return ERR_OK;
}
