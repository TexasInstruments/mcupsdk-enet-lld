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
 * Copyright (c) 2018 Texas Instruments Incorporated
 *
 * This file is dervied from the ``ethernetif.c'' skeleton Ethernet network
 * interface driver for lwIP.
 *
 */

/* Standard language headers */
#include <stdio.h>
#include <assert.h>
#include <string.h>

/* xdc header - should be first file included as due to order dependency */

/* OS/Posix headers */

/**
 * lwIP specific header files
 */
#include "lwip/opt.h"
#include "lwip/pbuf.h"
#include "lwip2enet.h"
#include "lwip/inet_chksum.h"
#include "lwip/prot/ip.h"
#include <lwip/netif.h>

/**
 * Enet device specific header files
 */
#include <enet.h>

#include <enet_appmemutils.h>
#include <networking/enet/core/include/core/enet_utils.h>
#include <enet_cfg.h>

#include <networking/enet/core/lwipif/src/lwip2lwipif_priv.h>
#include <kernel/nortos/dpl/common/printf.h>


/*---------------------------------------------------------------------------*\
 |                             Extern Declarations                             |
 \*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*\
 |                            Local Macros/Defines                             |
 \*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*\
 |                         Local Function Declarations                         |
 \*---------------------------------------------------------------------------*/

static void Lwip2Enet_createRxTxTasks(Lwip2Enet_Handle hLwip2Enet,
                                      Lwip2Enet_AppInfo *appInfo);

static void Lwip2Enet_deinitTxObj(Lwip2Enet_Handle hLwip2Enet);

static void Lwip2Enet_deinitRxObj(Lwip2Enet_Handle hLwip2Enet);

static void Lwip2Enet_saveAppIfCfg(Lwip2Enet_Handle hLwip2Enet,
                                   Lwip2Enet_AppInfo *appInfo);

static Lwip2Enet_TxHandle Lwip2Enet_initTxObj(uint32_t txCh,
                                              Lwip2Enet_Handle hLwip2Enet);

static Lwip2Enet_RxHandle Lwip2Enet_initRxObj(uint32_t rxCh,
                                              Lwip2Enet_Handle hLwip2Enet);

static void Lwip2Enet_mapNetif2Tx(struct netif *netif,
                                  bool isDirected,
                                  Lwip2Enet_TxHandle hTxLwip2Enet,
                                  Lwip2Enet_Handle hLwip2Enet);

static void Lwip2Enet_mapNetif2Rx(struct netif *netif,
                                  bool isDirected,
                                  Lwip2Enet_RxHandle hRxLwip2Enet,
                                  Lwip2Enet_Handle hLwip2Enet);

static void Lwip2Enet_notifyRxPackets(void *cbArg);

static void Lwip2Enet_notifyTxPackets(void *cbArg);

void Lwip2Enet_rxPacketTask(void *arg0);

static void Lwip2Enet_pbufQ2PktInfoQ(Lwip2Enet_TxObj *tx,
                                     pbufQ *pbufPktQ,
                                     EnetDma_PktQ *pDmaPktInfoQ,
                                     Enet_MacPort macPort);

static void Lwip2Enet_pktInfoQ2PbufQ(EnetDma_PktQ *pDmaPktInfoQ,
                                     pbufQ *pbufPktQ);

static uint32_t Lwip2Enet_prepRxPktQ(Lwip2Enet_RxObj *rx,
                                    EnetDma_PktQ *pPktQ);

static void Lwip2Enet_submitRxPktQ(Lwip2Enet_RxObj *rx);

void Lwip2Enet_txPacketTask(void *arg0);

static uint32_t Lwip2Enet_prepTxPktQ(Lwip2Enet_TxObj *tx,
                                    EnetDma_PktQ *pPktQ);

static void Lwip2Enet_freePbufPackets(EnetDma_PktQ *tempQueue);

static void Lwip2Enet_updateRxNotifyStats(Lwip2Enet_PktTaskStats *pktTaskStats,
                                          uint32_t packetCount,
                                          uint32_t timeDiff);

static void Lwip2Enet_updateTxNotifyStats(Lwip2Enet_PktTaskStats *pktTaskStats,
                                          uint32_t packetCount,
                                          uint32_t timeDiff);

static void Lwip2Enet_print(Lwip2Enet_Handle hLwip2Enet,
                            const char *prnStr,
                            ...);

static int32_t Lwip2Enet_startRxTx(Lwip2Enet_Handle hLwip2Enet);

static int32_t Lwip2Enet_startPollTask(Lwip2Enet_Handle hLwip2Enet);

static void Lwip2Enet_stopRxTx(Lwip2Enet_Handle hLwip2Enet);

static void Lwip2Enet_submitTxPackets(Lwip2Enet_TxObj *tx,
                                      EnetDma_PktQ *pSubmitQ);

static void Lwip2Enet_submitRxPackets(Lwip2Enet_RxObj *rx,
                                     EnetDma_PktQ *pSubmitQ);

void Lwip2Enet_retrieveTxPkts(Lwip2Enet_TxObj *tx);

static void Lwip2Enet_timerCb(ClockP_Object *hClk, void * arg);

static void Lwip2Enet_createTimer(Lwip2Enet_Handle hLwip2Enet);

static void Lwip2Enet_initGetTxHandleInArgs(Lwip2Enet_Handle hLwip2Enet,
                                            uint32_t chNum,
                                            LwipifEnetAppIf_GetTxHandleInArgs *inArgs);

static void Lwip2Enet_initGetRxHandleInArgs(Lwip2Enet_Handle hLwip2Enet,
                                            uint32_t chNum,
                                            LwipifEnetAppIf_GetRxHandleInArgs *inArgs);

static void Lwip2Enet_initReleaseTxHandleInArgs(Lwip2Enet_Handle hLwip2Enet,
                                              uint32_t chNum,
                                              LwipifEnetAppIf_ReleaseTxHandleInfo *inArgs);

static void Lwip2Enet_initReleaseRxHandleInArgs(Lwip2Enet_Handle hLwip2Enet,
                                              uint32_t chNum,
                                              LwipifEnetAppIf_ReleaseRxHandleInfo *inArgs);

static void Lwip2Enet_setSGList(EnetDma_Pkt *pCurrDmaPacket, struct pbuf *pbuf, bool isRx);
/*---------------------------------------------------------------------------*\
 |                         Local Variable Declarations                         |
 \*---------------------------------------------------------------------------*/

static Lwip2Enet_Obj  gLwip2EnetObj = { {{0}} };

/*---------------------------------------------------------------------------*\
 |                         Global Variable Declarations                        |
 \*---------------------------------------------------------------------------*/

static Lwip2Enet_Handle Lwip2Enet_getObj(void)
{
    uintptr_t key = EnetOsal_disableAllIntr();

    Lwip2Enet_Handle hLwip2Enet = &gLwip2EnetObj;

	EnetOsal_restoreAllIntr(key);

    return hLwip2Enet;
}

static void Lwip2Enet_putObj(Lwip2Enet_Handle hLwip2Enet)
{
    uintptr_t key = EnetOsal_disableAllIntr();
    /* Clear the allocated translation */
    memset(hLwip2Enet, 0, sizeof(*hLwip2Enet));
    EnetOsal_restoreAllIntr(key);

}

/**
 * Initializes Ethernet peripheral hardware
 */
Lwip2Enet_Handle Lwip2Enet_open(struct netif *netif)
{
    Lwip2Enet_Handle hLwip2Enet;
    Lwip2Enet_TxHandle hTxLwip2Enet;
    Lwip2Enet_RxHandle hRxLwip2Enet;
    LwipifEnetAppIf_GetHandleNetifInfo netifInfo;
    int32_t status;
    uint32_t i;

    hLwip2Enet = Lwip2Enet_getObj();
    if (hLwip2Enet->initDone == false)
    {

        /* MCast List is EMPTY */
        hLwip2Enet->MCastCnt = 0U;

        /* Init internal bookkeeping fields */
        hLwip2Enet->oldMCastCnt = 0U;

        for (i = 0U; i < ENET_CFG_NETIF_MAX; i++)
        {
            hLwip2Enet->mapNetif2Rx[i] = NULL;
            hLwip2Enet->mapNetif2Tx[i] = NULL;
            hLwip2Enet->mapNetif2TxPortNum[i] = ENET_MAC_PORT_INV;
        }

        for (i = 0U; i < CPSW_STATS_MACPORT_MAX; i++)
        {
            hLwip2Enet->mapRxPort2Netif[i] = NULL;
        }


        /* First init tasks, semaphores and clocks. This is required because
         * EnetDma event cb ISR can happen immediately after opening rx flow
         * in LwipifEnetAppCb_getHandle and all semaphores, clocks and tasks should
         * be valid at that point
         */

        LwipifEnetAppCb_getEnetLwipIfInstInfo(&hLwip2Enet->appInfo);

        /* Save params received from application interface */
        Lwip2Enet_saveAppIfCfg(hLwip2Enet, &hLwip2Enet->appInfo);

        Lwip2Enet_assert(hLwip2Enet->appInfo.hEnet != NULL);
        Lwip2Enet_assert(hLwip2Enet->appInfo.isPortLinkedFxn != NULL);
        Lwip2Enet_assert(hLwip2Enet->appInfo.pFreeTx != NULL);

        pbufQ_init_freeQ(hLwip2Enet->appInfo.pFreeTx, hLwip2Enet->appInfo.pFreeTxSize);


        /* set the print function callback if not null */
        hLwip2Enet->print = (Enet_Print) &EnetUtils_printf;
	}

    /* Process netif related parameters*/
    LwipifEnetAppCb_getNetifInfo(netif, &netifInfo);

    for(i=0U; i < netifInfo.numTxChannels; i++)
    {
        hTxLwip2Enet = Lwip2Enet_initTxObj(i, hLwip2Enet);
        Lwip2Enet_mapNetif2Tx(netif, netifInfo.isDirected, hTxLwip2Enet, hLwip2Enet);
    }

    for(i=0U; i < netifInfo.numRxChannels; i++)
    {
        hRxLwip2Enet = Lwip2Enet_initRxObj(i, hLwip2Enet);
        Lwip2Enet_mapNetif2Rx(netif, netifInfo.isDirected, hRxLwip2Enet, hLwip2Enet);
    }
    /* Updating the netif params */
    EnetUtils_copyMacAddr(netif->hwaddr ,&hLwip2Enet->macAddr[netif->num][0U]);
    netif->hwaddr_len = ENET_MAC_ADDR_LEN;
    netif->state = (void *)hLwip2Enet;
    hLwip2Enet->netif[netif->num] = netif;

    /* DMA handles are availablw now, so starting the tasks and completing the initialization of lwipif*/
    if(hLwip2Enet->initDone == false)
    {
        status = Lwip2Enet_startRxTx(hLwip2Enet);
        if (status != ENET_SOK)
        {
            Lwip2Enet_print(hLwip2Enet,"Failed to start the tasks: %d\n", status);
        }

        /* Get initial link/interface status from the driver */
        hLwip2Enet->linkIsUp = hLwip2Enet->appInfo.isPortLinkedFxn(hLwip2Enet->appInfo.hEnet);

        for (i = 0U; i < hLwip2Enet->numTxChannels; i++)
        {
            if (hLwip2Enet->tx[i].disableEvent)
            {
                EnetDma_disableTxEvent(hLwip2Enet->tx[i].hCh);
            }
        }

        for (i = 0U; i < hLwip2Enet->numRxChannels; i++)
        {
            if ((hLwip2Enet->rx[i].enabled) &&  (hLwip2Enet->rx[i].disableEvent))
            {
                EnetDma_disableRxEvent(hLwip2Enet->rx[i].hFlow);
            }
        }
        /* assert if clk period is not valid  */
        Lwip2Enet_assert(0U != hLwip2Enet->appInfo.timerPeriodUs);
        Lwip2Enet_createTimer(hLwip2Enet);
        // ClockP_start(&hLwip2Enet->pacingClkObj);

        hLwip2Enet->initDone = TRUE;
    }

    return hLwip2Enet;
}

static void Lwip2Enet_mapNetif2Tx(struct netif *netif,
                        bool isDirected,
                        Lwip2Enet_TxHandle hTxLwip2Enet,
                        Lwip2Enet_Handle hLwip2Enet)
{
    hLwip2Enet->mapNetif2Tx[netif->num] = hTxLwip2Enet;

    if(isDirected)
    {
        hLwip2Enet->mapNetif2TxPortNum[netif->num] = ENET_MACPORT_DENORM(LWIP_NETIF_IDX_2_TX_PORT(netif->num));
    }
}

static void Lwip2Enet_mapNetif2Rx(struct netif *netif,
                        bool isDirected,
                        Lwip2Enet_RxHandle hRxLwip2Enet,
                        Lwip2Enet_Handle hLwip2Enet)
{

    hLwip2Enet->mapNetif2Rx[netif->num] = hRxLwip2Enet;
    hLwip2Enet->mapRxPort2Netif[LWIP_NETIF_IDX_2_RX_PORT(netif->num)] = netif;
#if (ENET_ENABLE_PER_ICSSG == 1)
    /* ToDo: ICSSG doesnot fill rxPortNum correctly, so using the hRxLwip2Enet->flowIdx to map to netif*/
    hLwip2Enet->mapRxPort2Netif[LWIP_RXFLOW_2_PORTIDX(hRxLwip2Enet->flowIdx)] = netif;
#endif
    /* For non-directed packets, we map both ports to the first netif*/
    if(!isDirected)
    {
        for(uint32_t portIdx = 0U; portIdx < CPSW_STATS_MACPORT_MAX; portIdx++)
        {
            if(portIdx < ENET_CFG_NETIF_MAX)
            {
            hLwip2Enet->mapRxPort2Netif[portIdx] = netif;
            }
        }
    }
}

/*!
 *  @b Lwip2Enet_close
 *  @n
 *      Closes Ethernet peripheral and disables interrupts.
 *
 *  \param[in]  hLwip2Enet
 *      Lwip2Enet_object structure pointer.
 *
 *  \retval
 *      void
 */
void Lwip2Enet_close(Lwip2Enet_Handle hLwip2Enet)
{
    Lwip2Enet_assert(NULL != hLwip2Enet);

    /* Stop and delete the tick timer */
    ClockP_stop(&hLwip2Enet->pacingClkObj);
    ClockP_destruct(&hLwip2Enet->pacingClkObj);

    Lwip2Enet_stopRxTx(hLwip2Enet);

    if (hLwip2Enet->allocPktInfo > 0U)
    {
        Lwip2Enet_print(hLwip2Enet, "Lwip2Enet_close() failed to retrieve all PktInfo\n");
    }

    /* Deinit TX and RX objects, delete task, semaphore, etc */
    Lwip2Enet_deinitTxObj(hLwip2Enet);
    Lwip2Enet_deinitRxObj(hLwip2Enet);

    Lwip2Enet_putObj(hLwip2Enet);
}

static Lwip2Enet_RxHandle Lwip2Enet_initRxObj(uint32_t rxCh,
                                              Lwip2Enet_Handle hLwip2Enet)
{
    Lwip2Enet_RxHandle hRxHandle;
    LwipifEnetAppIf_GetRxHandleInArgs inArgs;
    LwipifEnetAppIf_RxHandleInfo outArgs;

    if(hLwip2Enet->rx[rxCh].enabled == true)
    {
        hLwip2Enet->rx[rxCh].refCount++;
        hRxHandle = &hLwip2Enet->rx[rxCh];
    }
    else{

        Lwip2Enet_initGetRxHandleInArgs(hLwip2Enet, rxCh, &inArgs);
        LwipifEnetAppCb_getRxHandleInfo(&inArgs, &outArgs);

        hLwip2Enet->rx[rxCh].numPackets = outArgs.numPackets;

        hLwip2Enet->rx[rxCh].flowIdx     = outArgs.rxFlowIdx;
        hLwip2Enet->rx[rxCh].flowStartIdx = outArgs.rxFlowStartIdx;
        hLwip2Enet->rx[rxCh].hFlow       = outArgs.hRxFlow;
        Lwip2Enet_assert(hLwip2Enet->rx[rxCh].hFlow != NULL);
        hLwip2Enet->rx[rxCh].disableEvent = outArgs.disableEvent;

        for (uint32_t i = 0U; i < hLwip2Enet->appInfo.maxNumNetif; i++)
        {
            EnetUtils_copyMacAddr(&hLwip2Enet->macAddr[i][0U], &outArgs.macAddr[i][0U]);
        }

        hLwip2Enet->rx[rxCh].stats.freeAppPktEnq = outArgs.numPackets;
        hLwip2Enet->allocPktInfo += outArgs.numPackets;

        hLwip2Enet->rx[rxCh].refCount = 1U;
        hLwip2Enet->rx[rxCh].enabled = true;
        hLwip2Enet->rx[rxCh].hLwip2Enet = hLwip2Enet;

        hRxHandle = &hLwip2Enet->rx[rxCh];
    }

    return hRxHandle;
}

static void Lwip2Enet_saveAppIfCfg(Lwip2Enet_Handle hLwip2Enet,
                                     Lwip2Enet_AppInfo *appInfo)
{
    hLwip2Enet->numRxChannels = appInfo->numRxChannels;
    hLwip2Enet->numTxChannels = appInfo->numTxChannels;
}

static void Lwip2Enet_deinitRxObj(Lwip2Enet_Handle hLwip2Enet)
{
    LwipifEnetAppIf_ReleaseRxHandleInfo inArgs;

    for (uint32_t i = 0U; i < hLwip2Enet->numRxChannels; i++)
    {
        if( hLwip2Enet->rx[i].refCount != 0U)
        {
            hLwip2Enet->rx[i].refCount--;
        }
        else
        {
            Lwip2Enet_initReleaseRxHandleInArgs(hLwip2Enet,i,&inArgs);
            LwipifEnetAppCb_releaseRxHandle(&inArgs);
            hLwip2Enet->rx[i].hFlow = NULL;
            hLwip2Enet->allocPktInfo -= EnetQueue_getQCount(&hLwip2Enet->rx[i].freePktInfoQ);
        }
    }

}

static void Lwip2Enet_deinitTxObj(Lwip2Enet_Handle hLwip2Enet)
{
    struct pbuf *pbuf;
    LwipifEnetAppIf_ReleaseTxHandleInfo inArgs;

    /*Deinit TX*/
    for (uint32_t i = 0U; i < hLwip2Enet->numTxChannels; i++)
    {
        if( hLwip2Enet->tx[i].refCount != 0U)
        {
            hLwip2Enet->tx[i].refCount--;
        }
        else
        {
            Lwip2Enet_initReleaseTxHandleInArgs(hLwip2Enet,i,&inArgs);
            LwipifEnetAppCb_releaseTxHandle(&inArgs);
            hLwip2Enet->tx[i].hCh = NULL;

            /* Free pbuf in ready queue */
            while (pbufQ_count(&hLwip2Enet->tx[i].readyPbufQ) != 0U)
            {
                pbuf = pbufQ_deQ(&hLwip2Enet->tx[i].readyPbufQ);
                Lwip2Enet_assert(NULL != pbuf);
                pbuf_free(pbuf);
            }

            hLwip2Enet->allocPktInfo -= EnetQueue_getQCount(&hLwip2Enet->tx[i].freePktInfoQ);
        }
    }

}

static Lwip2Enet_TxHandle Lwip2Enet_initTxObj(uint32_t txCh,
                                              Lwip2Enet_Handle hLwip2Enet)
{
    Lwip2Enet_TxHandle hTxHandle;
    LwipifEnetAppIf_GetTxHandleInArgs inArgs;
    LwipifEnetAppIf_TxHandleInfo outArgs;

    if(hLwip2Enet->tx[txCh].enabled == true)
    {
        hLwip2Enet->tx[txCh].refCount++;
        hTxHandle = &hLwip2Enet->tx[txCh];
    }
    else{

        Lwip2Enet_initGetTxHandleInArgs(hLwip2Enet, txCh, &inArgs);
        LwipifEnetAppCb_getTxHandleInfo(&inArgs, &outArgs);

        hLwip2Enet->tx[txCh].numPackets = outArgs.numPackets;
        hLwip2Enet->tx[txCh].hCh = outArgs.hTxChannel;
        hLwip2Enet->tx[txCh].chNum = outArgs.txChNum;
        Lwip2Enet_assert(hLwip2Enet->tx[txCh].hCh != NULL);

        hLwip2Enet->tx[txCh].disableEvent = outArgs.disableEvent;

        hLwip2Enet->tx[txCh].stats.freeAppPktEnq = outArgs.numPackets;
        hLwip2Enet->allocPktInfo += outArgs.numPackets;

        /* Initialize the TX pbuf queues */
        pbufQ_init(&hLwip2Enet->tx[txCh].readyPbufQ);
        pbufQ_init(&hLwip2Enet->tx[txCh].unusedPbufQ);

        hLwip2Enet->tx[txCh].refCount = 1U;
        hLwip2Enet->tx[txCh].enabled = true;
        hLwip2Enet->tx[txCh].hLwip2Enet = hLwip2Enet;

        hTxHandle = &hLwip2Enet->tx[txCh];
    }
    return hTxHandle;
}

static void Lwip2Enet_setSGList(EnetDma_Pkt *pCurrDmaPacket, struct pbuf *pbuf, bool isRx)
{
    struct pbuf *pbufNext = pbuf;
    uint32_t totalPacketFilledLen = 0U;

    pCurrDmaPacket->sgList.numScatterSegments = 0;
    while (pbufNext != NULL)
    {
        EnetDma_SGListEntry *list;

        Lwip2Enet_assert(pCurrDmaPacket->sgList.numScatterSegments < ENET_ARRAYSIZE(pCurrDmaPacket->sgList.list));
        list = &pCurrDmaPacket->sgList.list[pCurrDmaPacket->sgList.numScatterSegments];
        list->bufPtr = (uint8_t*) pbufNext->payload;
        list->segmentFilledLen = (isRx == true) ? 0U : pbufNext->len;
        list->segmentAllocLen = pbufNext->len;
        if ((pbufNext->type_internal == PBUF_ROM) || (pbufNext->type_internal == PBUF_REF))
        {
            list->disableCacheOps = true;
        }
        else
        {
            list->disableCacheOps = false;
        }
        totalPacketFilledLen += pbufNext->len;
        pCurrDmaPacket->sgList.numScatterSegments++;
        pbufNext = pbufNext->next;
    }
    Lwip2Enet_assert(totalPacketFilledLen == pbuf->tot_len);
}
/*!
 *  @b Lwip2Enet_sendTxPackets
 *  @n
 *      Routine to send out queued Tx packets to the hardware driver
 *
 *  \param[in]  hLwip2Enet
 *      Lwip2Enet_object structure pointer.
 *
 *  \retval
 *      void
 */
void Lwip2Enet_sendTxPackets(Lwip2Enet_TxObj *tx,
                             Enet_MacPort macPort)
{
    Lwip2Enet_Handle hLwip2Enet;
    EnetDma_Pkt *pCurrDmaPacket;
    struct pbuf *hPbufPkt;

    hLwip2Enet = tx->hLwip2Enet;

    /* If link is not up, simply return */
    if (hLwip2Enet->linkIsUp)
    {
        EnetDma_PktQ txSubmitQ;

        EnetQueue_initQ(&txSubmitQ);

        if (pbufQ_count(&tx->unusedPbufQ))
        {
            /* send any pending TX Q's */
            Lwip2Enet_pbufQ2PktInfoQ(tx, &tx->unusedPbufQ, &txSubmitQ, macPort);
        }

        /* Check if there is anything to transmit, else simply return */
        while (pbufQ_count(&tx->readyPbufQ) != 0U)
        {
            /* Dequeue one free TX Eth packet */
            pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(&tx->freePktInfoQ);

            if (pCurrDmaPacket == NULL)
            {
                /* If we run out of packet info Q, retrieve packets from HW
                * and try to dequeue free packet again */
                Lwip2Enet_retrieveTxPkts(tx);
                pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(&tx->freePktInfoQ);
            }

            if (NULL != pCurrDmaPacket)
            {
                hPbufPkt = pbufQ_deQ(&tx->readyPbufQ);
                EnetDma_initPktInfo(pCurrDmaPacket);

                Lwip2Enet_setSGList(pCurrDmaPacket, hPbufPkt, false);
                pCurrDmaPacket->appPriv    = hPbufPkt;
                pCurrDmaPacket->txPortNum  = macPort;
                pCurrDmaPacket->node.next  = NULL;
                pCurrDmaPacket->chkSumInfo = 0U;

#if ((ENET_CFG_IS_ON(CPSW_CSUM_OFFLOAD_SUPPORT) == 1) && (ENET_ENABLE_PER_CPSW == 1))
                pCurrDmaPacket->chkSumInfo = LWIPIF_LWIP_getChkSumInfo(hPbufPkt);
#endif

                ENET_UTILS_COMPILETIME_ASSERT(offsetof(EnetDma_Pkt, node) == 0U);
                EnetQueue_enq(&txSubmitQ, &(pCurrDmaPacket->node));

                LWIP2ENETSTATS_ADDONE(&tx->stats.freeAppPktDeq);
                LWIP2ENETSTATS_ADDONE(&tx->stats.readyPbufPktDeq);
            }
            else
            {
                break;
            }
        }

        /* Submit the accumulated packets to the hardware for transmission */
        Lwip2Enet_submitTxPackets(tx, &txSubmitQ);
    }
}


/*!
 *  @b Lwip2Enet_ioctl
 *  @n
 *  Low level driver Ioctl interface. This interface can be used for
 *  ALE configuration, control, statistics
 *
 *  \param[in]  hLwip2Enet
 *      Lwip2Enet_object structure pointer.
 *  \param[in]  cmd
 *      Ioctl command.
 *  \param[in]  pBuf
 *      Ioctl buffer with commands and params to set/get
 *      configuration from hardware.
 *  \param[in]  size
 *      Size of Ioctl buffer.
 *
 *  \retval
 *      void
 */
int32_t Lwip2Enet_ioctl(Lwip2Enet_Handle hLwip2Enet,
                       uint32_t cmd,
                       void *param,
                       uint32_t size)
{
    int32_t retVal = 0U;

    // TODO - make IOCTL design consistent with Enet/Enet modules

    /* Decode the command and act on it */
    switch (cmd)
    {
        /* Rx Task Load IOCTL */
        case LWIP2ENET_IOCTL_GET_RXTASK_LOAD:
        {
            break;
        }

        /* Adding Tx Task Load  */
        case LWIP2ENET_IOCTL_GET_TXTASK_LOAD:
        {
            break;
        }

        default:
        {
            Lwip2Enet_assert(FALSE);
        }
    }

    return retVal;
}

/*---------------------------------------------------------------------------*\
 |                           Local Function Definitions                        |
 \*---------------------------------------------------------------------------*/

 void Lwip2Enet_periodicFxn(struct netif* netif)
{
    Lwip2Enet_Handle hLwip2Enet = (Lwip2Enet_Handle)netif->state;

    uint32_t prevLinkState     = hLwip2Enet->linkIsUp;
    uint32_t prevLinkInterface = hLwip2Enet->currLinkedIf;

#if (1U == ENET_CFG_DEV_ERROR)
#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_UDMA)
    int32_t status;
#endif

    for(uint32_t i = 0U; i < hLwip2Enet->numTxChannels; i++)
    {
        EnetQueue_verifyQCount(&hLwip2Enet->tx[i].freePktInfoQ);

#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_UDMA)
        status = EnetUdma_checkTxChSanity(hLwip2Enet->tx[i].hCh, 5U);
        if (status != ENET_SOK)
        {
            Lwip2Enet_print(hLwip2Enet, "EnetUdma_checkTxChSanity Failed\n");
        }
#endif
    }

    for(uint32_t i = 0U; i < hLwip2Enet->numRxChannels; i++)
    {
        EnetQueue_verifyQCount(&hLwip2Enet->rx[i].freePktInfoQ);

#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_UDMA)
        status = EnetUdma_checkRxFlowSanity(hLwip2Enet->rx[i].hFlow, 5U);
        if (status != ENET_SOK)
        {
            Lwip2Enet_print(hLwip2Enet, "EnetUdma_checkRxFlowSanity Failed\n");
        }
#endif
    }


#endif

    /*
     * Return the same DMA packets back to the DMA channel (but now
     * associated with a new PBUF Packet and buffer)
     */
    for(uint32_t i = 0U; i < hLwip2Enet->numRxChannels; i++)
    {
        if (EnetQueue_getQCount(&hLwip2Enet->rx[i].freePktInfoQ) != 0U)
        {
            Lwip2Enet_submitRxPktQ(&hLwip2Enet->rx[i]);
        }
    }
#if 0 //The below CPU load profilling logic has to be re-worked for all OS variants
#if defined(LWIPIF_INSTRUMENTATION_ENABLED)
    static uint32_t loadCount = 0U;
    TaskP_Load stat;

    hLwip2Enet->stats.cpuLoad[loadCount] = TaskP_loadGetTotalCpuLoad();

    TaskP_loadGet(&hLwip2Enet->rxPacketTaskObj, &stat);
    for(uint32_t i = 0U; i < hLwip2Enet->numRxChannels; i++)
    {
        hLwip2Enet->rx[i].stats.pktStats.taskLoad[loadCount] = stat.cpuLoad;
    }

    TaskP_loadGet(&hLwip2Enet->txPacketTaskObj, &stat);
    for(uint32_t i = 0U; i < hLwip2Enet->numRxChannels; i++)
    {
        hLwip2Enet->tx[i].stats.pktStats.taskLoad[loadCount] = stat.cpuLoad;
    }

    loadCount = (loadCount + 1U) & (HISTORY_CNT - 1U);
#endif
#endif
    /* Get current link status as reported by the hardware driver */
    hLwip2Enet->linkIsUp = hLwip2Enet->appInfo.isPortLinkedFxn(hLwip2Enet->appInfo.hEnet);

    /* If the interface changed, discard any queue packets (since the MAC would now be wrong) */
    if (prevLinkInterface != hLwip2Enet->currLinkedIf)
    {
        /* ToDo: Discard all queued packets */
    }

    for(uint32_t i = 0U; i < hLwip2Enet->numTxChannels; i++)
    {
        /* If link status changed from down->up, then send any queued packets */
        if ((prevLinkState == 0U) && (hLwip2Enet->linkIsUp))
        {
            Lwip2Enet_TxHandle hTxHandle;
            Enet_MacPort macPort;

            hTxHandle  = hLwip2Enet->mapNetif2Tx[netif->num];
            macPort    = hLwip2Enet->mapNetif2TxPortNum[netif->num];
            Lwip2Enet_sendTxPackets(hTxHandle, macPort);
        }
    }

}

static void Lwip2Enet_processRxUnusedQ(Lwip2Enet_RxObj *rx,
                                       EnetDma_PktQ *unUsedQ)
{
    EnetDma_Pkt *pDmaPacket;

    pDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(unUsedQ);
    while (pDmaPacket != NULL)
    {
        /* Get the full PBUF packet that needs to be returned to the rx.freePktInfoQ */
        struct pbuf* hPbufPacket = (struct pbuf *)pDmaPacket->appPriv;
        if (hPbufPacket)
        {
            /* Put packet info into free Q as we have removed the Pbuf buffers
             * from the it */
            EnetQueue_enq(&rx->freePktInfoQ, &pDmaPacket->node);
            LWIP2ENETSTATS_ADDONE(&rx->stats.freeAppPktEnq);

            pDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(unUsedQ);
        }
        else
        {
            /* should never happen as this is received from HW */
            Lwip2Enet_assert(FALSE);
        }
    }
}

static void Lwip2Enet_submitRxPackets(Lwip2Enet_RxObj *rx,
                                      EnetDma_PktQ *pSubmitQ)
{
    int32_t retVal;

    retVal = EnetDma_submitRxPktQ(rx->hFlow, pSubmitQ);
    if (ENET_SOK != retVal)
    {
		Lwip2Enet_print(rx->hLwip2Enet,
                        "EnetDma_submitRxPktQ: failed to submit pkts: %d\n",
                        retVal);
    }

    if (EnetQueue_getQCount(pSubmitQ))
    {
        /* Copy unused packets to back to readyQ */
        Lwip2Enet_processRxUnusedQ(rx, pSubmitQ);
    }
}

/* May lead to infinite loop if no free memory
 * available*/
static void Lwip2Enet_pbufQ2PktInfoQ(Lwip2Enet_TxObj *tx,
                                     pbufQ *pbufPktQ,
                                     EnetDma_PktQ *pDmaPktInfoQ,
                                     Enet_MacPort macPort)
{
    EnetDma_Pkt *pCurrDmaPacket;
    struct pbuf *hPbufPkt = NULL;

    while(pbufQ_count(pbufPktQ) != 0U)
    {

        /* Dequeue one free TX Eth packet */
        pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(&tx->freePktInfoQ);

        if (pCurrDmaPacket == NULL)
        {
        /* If we run out of packet info Q, retrieve packets from HW
            * and try to dequeue free packet again */
        Lwip2Enet_retrieveTxPkts(tx);
        pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(&tx->freePktInfoQ);
        }

        if (NULL != pCurrDmaPacket)
        {
            hPbufPkt = pbufQ_deQ(pbufPktQ);
            EnetDma_initPktInfo(pCurrDmaPacket);

            Lwip2Enet_setSGList(pCurrDmaPacket, hPbufPkt, false);
            pCurrDmaPacket->appPriv    = hPbufPkt;

            pCurrDmaPacket->node.next = NULL;
            pCurrDmaPacket->chkSumInfo = 0U;
            pCurrDmaPacket->txPortNum  = macPort;

#if ((ENET_CFG_IS_ON(CPSW_CSUM_OFFLOAD_SUPPORT) == 1) && (ENET_ENABLE_PER_CPSW == 1))
            pCurrDmaPacket->chkSumInfo = LWIPIF_LWIP_getChkSumInfo(hPbufPkt);
#endif

            ENET_UTILS_COMPILETIME_ASSERT(offsetof(EnetDma_Pkt, node) == 0U);
            EnetQueue_enq(pDmaPktInfoQ, &(pCurrDmaPacket->node));

            LWIP2ENETSTATS_ADDONE(&tx->stats.freeAppPktDeq);
        }
        else
        {
            break;
        }
    }
}

static void Lwip2Enet_pktInfoQ2PbufQ(EnetDma_PktQ *pDmaPktInfoQ,
                                     pbufQ *pbufPktQ)
{
    EnetDma_Pkt *pDmaPacket;
    struct pbuf *hPbufPacket;

    while (EnetQueue_getQCount(pDmaPktInfoQ) != 0U)
    {
        pDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(pDmaPktInfoQ);
        hPbufPacket = (struct pbuf *)(pDmaPacket->appPriv);

        Lwip2Enet_assert(hPbufPacket != NULL);
        /*Don't want to make a copy, since it would cause waste memory*/
        pbufQ_enQ(pbufPktQ, hPbufPacket);
    }
}

static void Lwip2Enet_submitTxPackets(Lwip2Enet_TxObj *tx,
                                      EnetDma_PktQ *pSubmitQ)
{
    int32_t retVal;

    retVal = EnetDma_submitTxPktQ(tx->hCh, pSubmitQ);
    if (ENET_SOK != retVal)
    {
        Lwip2Enet_print(tx->hLwip2Enet,
                        "EnetDma_submitTxPktQ: failed to submit pkts: %d\n",
                        retVal);
    }

    if (EnetQueue_getQCount(pSubmitQ))
    {
        /* TODO: txUnUsedPBMPktQ is needed for packets that were not able to be
         *       submitted to driver.  It can be removed if stack supported any
         *       mechanism to enqueue them to the head of the queue. */
        Lwip2Enet_pktInfoQ2PbufQ(pSubmitQ, &tx->unusedPbufQ);
        EnetQueue_append(&tx->freePktInfoQ, pSubmitQ);
        LWIP2ENETSTATS_ADDNUM(&tx->stats.freeAppPktEnq, EnetQueue_getQCount(pSubmitQ));
    }
}

static void Lwip2Enet_freePbufPackets(EnetDma_PktQ *tempQueue)
{
    EnetDma_Pkt *pCurrDmaPacket;
    struct pbuf* hPbufPacket;

    while (EnetQueue_getQCount(tempQueue))
    {
        pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(tempQueue);
        hPbufPacket     = (struct pbuf *)pCurrDmaPacket->appPriv;
        pbuf_free(hPbufPacket);
    }
}

static void Lwip2Enet_notifyRxPackets(void *cbArg)
{
    Lwip2Enet_RxHandle hRxLwip2Enet = (Lwip2Enet_RxHandle)cbArg;
    Lwip2Enet_Handle hLwip2Enet = hRxLwip2Enet->hLwip2Enet;

    /* do not post events if init not done or shutdown in progress */
    if (hLwip2Enet->initDone)
    {
        for(uint32_t i = 0U; i < hLwip2Enet->numRxChannels; i++)
        {
            if (hLwip2Enet->rx[i].enabled)
            {
                EnetDma_disableRxEvent(hLwip2Enet->rx[i].hFlow);
            }
            if (hLwip2Enet->rxPktNotify.cbFxn != NULL)
            {
                hLwip2Enet->rxPktNotify.cbFxn(hLwip2Enet->rxPktNotify.cbArg);
            }
        }
    }
}

static void Lwip2Enet_notifyTxPackets(void *cbArg)
{
    Lwip2Enet_TxHandle hTxLwip2Enet = (Lwip2Enet_TxHandle)cbArg;
    Lwip2Enet_Handle hLwip2Enet = hTxLwip2Enet->hLwip2Enet;

    /* do not post events if init not done or shutdown in progress */
    if ((hLwip2Enet->initDone) && (hLwip2Enet->txPktNotify.cbFxn != NULL))
    {
        /* Notify Callbacks to post event/semephore */
        hLwip2Enet->txPktNotify.cbFxn(hLwip2Enet->txPktNotify.cbArg);
    }
}

void Lwip2Enet_rxPktHandler(Lwip2Enet_Handle hLwip2Enet)
{
    EnetDma_PktQ tempQueue;
    int32_t retVal;
    uint32_t pktCnt, rxChNum;

    for(rxChNum = 0U; rxChNum < hLwip2Enet->numRxChannels; rxChNum++)
    {
        pktCnt = 0U;
        LWIP2ENETSTATS_ADDONE(&hLwip2Enet->rx[rxChNum].stats.pktStats.rawNotifyCnt);

        /* Retrieve the used (filled) packets from the channel */
        {
            EnetQueue_initQ(&tempQueue);
            retVal = EnetDma_retrieveRxPktQ(hLwip2Enet->rx[rxChNum].hFlow, &tempQueue);
            if (ENET_SOK != retVal)
            {
                Lwip2Enet_print(hLwip2Enet,
                                "Lwip2Enet_rxPacketTask: failed to retrieve RX pkts: %d\n",
                                retVal);
            }
        }
        if (tempQueue.count == 0U)
        {
            LWIP2ENETSTATS_ADDONE(&hLwip2Enet->rx[rxChNum].stats.pktStats.zeroNotifyCnt);
        }

        /*
         * Call Lwip2Enet_prepRxPktQ() even if no packets were received.
         * This allows new packets to be submitted if PBUF buffers became
         * newly available and there were outstanding free packets.
         */
        {
            /*
             * Get all used Rx DMA packets from the hardware, then send the buffers
             * of those packets on to the LwIP stack to be parsed/processed.
             */
            pktCnt = Lwip2Enet_prepRxPktQ(&hLwip2Enet->rx[rxChNum], &tempQueue);
        }

        /*
         * We don't want to time the semaphore post used to notify the LwIP stack as that may cause a
         * task transition. We don't want to time the semaphore pend, since that would time us doing
         * nothing but waiting.
         */
        if (pktCnt != 0U)
        {
            Lwip2Enet_updateRxNotifyStats(&hLwip2Enet->rx[rxChNum].stats.pktStats, pktCnt, 0U);
        }

        // ClockP_start(&hLwip2Enet->pacingClkObj);

        if (!hLwip2Enet->rx[rxChNum].disableEvent)
        {
            EnetDma_enableRxEvent(hLwip2Enet->rx[rxChNum].hFlow);
        }
    }


}

void Lwip2Enet_txPktHandler(Lwip2Enet_Handle hLwip2Enet )
{
    uint32_t txChNum;

    for (txChNum = 0U; txChNum < hLwip2Enet->numTxChannels; txChNum++)
    {
        Lwip2Enet_retrieveTxPkts(&hLwip2Enet->tx[txChNum]);
    }


}

/*
 * Enqueue a new packet and make sure that buffer descriptors are properly linked.
 * NOTE: Not thread safe
 */
static void Lwip2Enet_submitRxPktQ(Lwip2Enet_RxObj *rx)
{
    EnetDma_PktQ resubmitPktQ;
    struct pbuf* hPbufPacket;
    EnetDma_Pkt *pCurrDmaPacket;
    uint32_t bufSize;
    Lwip2Enet_Handle hLwip2Enet = rx->hLwip2Enet;

    EnetQueue_initQ(&resubmitPktQ);

    /*
     * Fill in as many packets as we can with new PBUF buffers so they can be
     * returned to the stack to be filled in.
     */
    pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(&rx->freePktInfoQ);

    while (NULL != pCurrDmaPacket)
    {
        hPbufPacket = (struct pbuf*)pCurrDmaPacket->appPriv;
        if (hPbufPacket)
        {
            LwipifEnetAppIf_custom_rx_pbuf* my_pbuf  = (LwipifEnetAppIf_custom_rx_pbuf*)hPbufPacket;

            my_pbuf->p.custom_free_function = LwipifEnetAppCb_pbuf_free_custom;
            my_pbuf->pktInfoMem         = pCurrDmaPacket;
            my_pbuf->freePktInfoQ         = &rx->freePktInfoQ;
            my_pbuf->p.pbuf.flags |= PBUF_FLAG_IS_CUSTOM;

            bufSize = ENET_UTILS_ALIGN(hLwip2Enet->appInfo.hostPortRxMtu, ENETDMA_CACHELINE_ALIGNMENT);
            hPbufPacket = pbuf_alloced_custom(PBUF_RAW, bufSize, PBUF_POOL, &my_pbuf->p, pCurrDmaPacket->sgList.list[0].bufPtr, pCurrDmaPacket->sgList.list[0].segmentAllocLen);

            LWIP2ENETSTATS_ADDONE(&rx->stats.freePbufPktDeq);
            LWIP2ENETSTATS_ADDONE(&rx->stats.freeAppPktDeq);

            EnetQueue_enq(&resubmitPktQ, &pCurrDmaPacket->node);

            pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(&rx->freePktInfoQ);
        }
        else
        {
            EnetQueue_enq(&rx->freePktInfoQ, &pCurrDmaPacket->node);
            break;
        }
    }

    /*
     * Return the same DMA packets back to the DMA channel (but now
     * associated with a new PBM Packet and buffer)
     */
    if (EnetQueue_getQCount(&resubmitPktQ))
    {
        Lwip2Enet_submitRxPackets(rx, &resubmitPktQ);
    }
}

static uint32_t Lwip2Enet_prepRxPktQ(Lwip2Enet_RxObj *rx,
                                     EnetDma_PktQ *pPktQ)
{
    uint32_t packetCount = 0U;
    EnetDma_Pkt *pCurrDmaPacket;
    bool isChksumError = false;
    uint32_t validLen = 0U;

    pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(pPktQ);
    while (pCurrDmaPacket)
    {
        /* Get the full PBUF packet that needs to be returned to the LwIP stack */
        struct pbuf* hPbufPacket = (struct pbuf *)pCurrDmaPacket->appPriv;
        isChksumError = false;
        if (hPbufPacket)
        {
            validLen = pCurrDmaPacket->sgList.list[0].segmentFilledLen;

            /* Fill in PBUF packet length field */
            hPbufPacket->len = validLen;
            hPbufPacket->tot_len = validLen;
            Lwip2Enet_assert(hPbufPacket->payload != NULL);

#if ((ENET_CFG_IS_ON(CPSW_CSUM_OFFLOAD_SUPPORT) == 1) && (ENET_ENABLE_PER_CPSW == 1))
            {
                struct ip_hdr* pIpPkt = (struct ip_hdr* ) LWIPIF_LWIP_getIpPktStart((uint8_t*) hPbufPacket->payload);
                if (IPH_PROTO(pIpPkt) == IP_PROTO_UDPLITE)
                {
                    isChksumError = LWIPIF_LWIP_UdpLiteValidateChkSum(hPbufPacket);
                }
                else
                {
                    /* We don't check if HW checksum offload is enabled while checking for checksum error
                     * as default value of this field when offload not enabled is false */
                    const uint32_t csumInfo =  pCurrDmaPacket->chkSumInfo;

                    if ( ENETDMA_RXCSUMINFO_GET_IPV4_FLAG(csumInfo) ||
                            ENETDMA_RXCSUMINFO_GET_IPV6_FLAG(csumInfo))
                    {
                        isChksumError = ENETDMA_RXCSUMINFO_GET_CHKSUM_ERR_FLAG(csumInfo);
                    }
                }
            }
#endif
            if (!isChksumError)
            {
                /* Pass the received packet to the LwIP stack */
                LWIPIF_LWIP_input(rx, pCurrDmaPacket->rxPortNum, hPbufPacket);
                packetCount++;
            }
            else
            {
                /* Put PBUF buffer in free Q as we are not passing to stack */
                EnetQueue_enq(&rx->freePktInfoQ, &pCurrDmaPacket->node);
                LWIP2ENETSTATS_ADDONE(&rx->stats.freeAppPktEnq);
                LWIP2ENETSTATS_ADDONE(&rx->stats.chkSumErr);
            }

            /* Put packet info into free Q as we have removed the PBUF buffers
             * from the it */
            pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(pPktQ);
        }
        else
        {
            /* Should never happen as this is received from HW */
            Lwip2Enet_assert(FALSE);
        }
    }

    /* return as many packets to driver as we can */
    Lwip2Enet_submitRxPktQ(rx);

    return packetCount;
}

static uint32_t Lwip2Enet_prepTxPktQ(Lwip2Enet_TxObj *tx,
                                     EnetDma_PktQ *pPktQ)
{
    uint32_t packetCount;
    EnetDma_Pkt *pCurrDmaPacket;
    struct pbuf* hPbufPacket;

    packetCount = EnetQueue_getQCount(pPktQ);

    pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(pPktQ);
    while (pCurrDmaPacket)
    {
        hPbufPacket = (struct pbuf *)pCurrDmaPacket->appPriv;

        Lwip2Enet_assert(hPbufPacket != NULL);
        /* Free PBUF buffer as it is transmitted by DMA now */
        pbuf_free(hPbufPacket);
        /* Return packet info to free pool */
        EnetQueue_enq(&tx->freePktInfoQ, &pCurrDmaPacket->node);
        pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(pPktQ);
    }

    LWIP2ENETSTATS_ADDNUM(&tx->stats.freeAppPktEnq, packetCount);

    return packetCount;
}

static void Lwip2Enet_updateTxNotifyStats(Lwip2Enet_PktTaskStats *pktStats,
                                          uint32_t packetCount,
                                          uint32_t timeDiff)
{
#if defined(LWIPIF_INSTRUMENTATION_ENABLED)
    uint32_t notificationCount;
    uint32_t timePerPacket = timeDiff / packetCount;

    notificationCount = pktStats->dataNotifyCnt & (HISTORY_CNT - 1U);
    pktStats->dataNotifyCnt++;

    pktStats->totalPktCnt   += packetCount;
    pktStats->totalCycleCnt += timeDiff;

    pktStats->cycleCntPerNotify[notificationCount] = timeDiff;
    if (timeDiff > pktStats->cycleCntPerNotifyMax)
    {
        pktStats->cycleCntPerNotifyMax = timeDiff;
    }

    pktStats->pktsPerNotify[notificationCount] = packetCount;
    if (packetCount > pktStats->pktsPerNotifyMax)
    {
        pktStats->pktsPerNotifyMax = packetCount;
    }

    pktStats->cycleCntPerPkt[notificationCount] = timePerPacket;
    if (timePerPacket > pktStats->cycleCntPerPktMax)
    {
        pktStats->cycleCntPerPktMax = timePerPacket;
    }
#endif
}

static void Lwip2Enet_updateRxNotifyStats(Lwip2Enet_PktTaskStats *pktTaskStats,
                                          uint32_t packetCount,
                                          uint32_t timeDiff)
{
#if defined(LWIPIF_INSTRUMENTATION_ENABLED)
    uint32_t notificationCount;
    uint32_t timePerPacket = timeDiff / packetCount;

    notificationCount = pktTaskStats->dataNotifyCnt & (HISTORY_CNT - 1U);
    pktTaskStats->dataNotifyCnt++;

    pktTaskStats->totalPktCnt   += packetCount;
    pktTaskStats->totalCycleCnt += timeDiff;

    pktTaskStats->cycleCntPerNotify[notificationCount] = timeDiff;
    if (timeDiff > pktTaskStats->cycleCntPerNotifyMax)
    {
        pktTaskStats->cycleCntPerNotifyMax = timeDiff;
    }

    pktTaskStats->pktsPerNotify[notificationCount] = packetCount;
    if (packetCount > pktTaskStats->pktsPerNotifyMax)
    {
        pktTaskStats->pktsPerNotifyMax = packetCount;
    }

    pktTaskStats->cycleCntPerPkt[notificationCount] = timePerPacket;
    if (timePerPacket > pktTaskStats->cycleCntPerPktMax)
    {
        pktTaskStats->cycleCntPerPktMax = timePerPacket;
    }
#endif
}

static void Lwip2Enet_print(Lwip2Enet_Handle hLwip2Enet,
                            const char *prnStr,
                            ...)
{
    // TODO fix me
    if (NULL != hLwip2Enet->print)
    {
        /* Function is non- reentrant */
        va_list vaArgPtr;
        char *buf;

        buf = &hLwip2Enet->printBuf[0U];
        va_start(vaArgPtr, prnStr);
        vsnprintf(buf, ENET_CFG_PRINT_BUF_LEN, (const char *)prnStr, vaArgPtr);
        va_end(vaArgPtr);

        (*hLwip2Enet->print)("[LWIP2ENET] ");
        (*hLwip2Enet->print)(hLwip2Enet->printBuf);
    }
}

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static int32_t Lwip2Enet_startRxTx(Lwip2Enet_Handle hLwip2Enet)
{
    int32_t status = ENET_SOK;
    uint32_t i;

    for(i = 0U; i< hLwip2Enet->numRxChannels; i++)
    {
        Lwip2Enet_assert(NULL != hLwip2Enet->rx[i].hFlow);
    }

    for(i = 0U; i< hLwip2Enet->numTxChannels; i++)
    {
        Lwip2Enet_assert(NULL != hLwip2Enet->tx[i].hCh);
        status = EnetDma_enableTxEvent(hLwip2Enet->tx[i].hCh);
    }

    for(i = 0U; i< hLwip2Enet->numRxChannels; i++)
    {
        /* Submit all allocated packets to DMA so it can use for packet RX */
        Lwip2Enet_submitRxPktQ(&hLwip2Enet->rx[i]);
    }

    return status;
}

/*
 * Periodically polls for changes in the link status and updates both the abstraction layer
 * as well as the stack
 * arg : netif
 * arg1 : Semaphore
 */


static void Lwip2Enet_stopRxTx(Lwip2Enet_Handle hLwip2Enet)
{
	/* Stop RX packet task */
    /* Post to rx packet task/event so that it will terminate (shutDownFlag flag is already set) */
    if (hLwip2Enet->rxPktNotify.cbFxn != NULL)
    {
        hLwip2Enet->rxPktNotify.cbFxn(hLwip2Enet->rxPktNotify.cbArg);
    }

    /* Stop TX packet task */
    /* Post to tx packet task/event so that it will terminate (shutDownFlag flag is already set) */
    if (hLwip2Enet->txPktNotify.cbFxn != NULL)
    {
        hLwip2Enet->txPktNotify.cbFxn(hLwip2Enet->txPktNotify.cbArg);
    }
}

static void Lwip2Enet_freeTxPktCb(void *cbArg,
                                  EnetDma_PktQ *fqPktInfoQ,
                                  EnetDma_PktQ *cqPktInfoQ)
{
    Lwip2Enet_TxObj *tx = (Lwip2Enet_TxObj *)cbArg;

    EnetQueue_append(&tx->freePktInfoQ, fqPktInfoQ);
    LWIP2ENETSTATS_ADDNUM(&tx->stats.freeAppPktEnq, EnetQueue_getQCount(fqPktInfoQ));
    Lwip2Enet_freePbufPackets(fqPktInfoQ);

    EnetQueue_append(&tx->freePktInfoQ, cqPktInfoQ);
    LWIP2ENETSTATS_ADDNUM(&tx->stats.freeAppPktEnq, EnetQueue_getQCount(cqPktInfoQ));
    Lwip2Enet_freePbufPackets(cqPktInfoQ);
}

static void Lwip2Enet_freeRxPktCb(void *cbArg,
                                  EnetDma_PktQ *fqPktInfoQ,
                                  EnetDma_PktQ *cqPktInfoQ)
{
    Lwip2Enet_RxObj *rx = (Lwip2Enet_RxObj *)cbArg;

    /* Now that we free PBUF buffers, push all freed pktInfo's into Rx
     * free Q */
    EnetQueue_append(&rx->freePktInfoQ, fqPktInfoQ);
    LWIP2ENETSTATS_ADDNUM(&rx->stats.freeAppPktEnq, EnetQueue_getQCount(fqPktInfoQ));
    Lwip2Enet_freePbufPackets(fqPktInfoQ);

    EnetQueue_append(&rx->freePktInfoQ, cqPktInfoQ);
    LWIP2ENETSTATS_ADDNUM(&rx->stats.freeAppPktEnq, EnetQueue_getQCount(cqPktInfoQ));
    Lwip2Enet_freePbufPackets(cqPktInfoQ);

    /* Flush out our pending receive queues */
    while (EnetQueue_getQCount(&rx->freePktInfoQ) != 0U)
    {
        EnetQueue_deq(&rx->freePktInfoQ);
        LWIP2ENETSTATS_ADDONE(&rx->stats.freePbufPktDeq);
    }
}

void Lwip2Enet_retrieveTxPkts(Lwip2Enet_TxObj *tx)
{
    EnetDma_PktQ tempQueue;
    uint32_t packetCount = 0U;
    int32_t retVal;

    LWIP2ENETSTATS_ADDONE(&tx->stats.pktStats.rawNotifyCnt);
    packetCount = 0U;

    /* Retrieve the used (sent/empty) packets from the channel */
    {
        EnetQueue_initQ(&tempQueue);
        /* Retrieve all TX packets and keep them locally */
        retVal = EnetDma_retrieveTxPktQ(tx->hCh, &tempQueue);
        if (ENET_SOK != retVal)
        {
            Lwip2Enet_print(tx->hLwip2Enet,
                            "Lwip2Enet_retrieveTxPkts: Failed to retrieve TX pkts: %d\n",
                            retVal);
        }
    }

    if (tempQueue.count != 0U)
    {
        /*
         * Get all used Tx DMA packets from the hardware, then return those
         * buffers to the txFreePktQ so they can be used later to send with.
         */
        packetCount = Lwip2Enet_prepTxPktQ(tx, &tempQueue);
    }
    else
    {
        LWIP2ENETSTATS_ADDONE(&tx->stats.pktStats.zeroNotifyCnt);
    }

    if (packetCount != 0U)
    {
        Lwip2Enet_updateTxNotifyStats(&tx->stats.pktStats, packetCount, 0U);
    }
}

static void Lwip2Enet_timerCb(ClockP_Object *hClk, void * arg)
{
#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_UDMA)
    /* Post semaphore to rx handling task */
    Lwip2Enet_Handle hLwip2Enet = (Lwip2Enet_Handle)arg;

    if (hLwip2Enet->initDone)
    {
        for (uint32_t i = 0U; i < hLwip2Enet->numTxChannels; i++)
        {
            if (hLwip2Enet->tx[i].enabled)
            {
                Lwip2Enet_retrieveTxPkts(&hLwip2Enet->tx[i]);
            }
        }

        if (hLwip2Enet->rxPktNotify.cbFxn != NULL)
        {
            hLwip2Enet->rxPktNotify.cbFxn(hLwip2Enet->rxPktNotify.cbArg);
        }
    }
#endif
}

static void Lwip2Enet_createTimer(Lwip2Enet_Handle hLwip2Enet)
{
    ClockP_Params clkPrms;
    int32_t status;

    ClockP_Params_init(&clkPrms);
    clkPrms.start  = true;
    clkPrms.timeout = ClockP_usecToTicks(hLwip2Enet->appInfo.timerPeriodUs);
    clkPrms.period = ClockP_usecToTicks(hLwip2Enet->appInfo.timerPeriodUs);
    clkPrms.callback = &Lwip2Enet_timerCb;
    clkPrms.args = hLwip2Enet;

    status =  ClockP_construct(&hLwip2Enet->pacingClkObj, &clkPrms);
    Lwip2Enet_assert(status == SystemP_SUCCESS);
}

static void  Lwip2Enet_initGetTxHandleInArgs(Lwip2Enet_Handle hLwip2Enet,
                                             uint32_t chNum,
                                             LwipifEnetAppIf_GetTxHandleInArgs *inArgs)
{
    inArgs->cbArg      = &hLwip2Enet->tx[chNum];
    inArgs->notifyCb   = &Lwip2Enet_notifyTxPackets;
    inArgs->chId       = chNum;
    inArgs->pktInfoQ   = &hLwip2Enet->tx[chNum].freePktInfoQ;
}

static void Lwip2Enet_initGetRxHandleInArgs(Lwip2Enet_Handle hLwip2Enet,
                                            uint32_t chNum,
                                            LwipifEnetAppIf_GetRxHandleInArgs *inArgs)
{
    inArgs->cbArg      = &hLwip2Enet->rx[chNum];
    inArgs->notifyCb   = &Lwip2Enet_notifyRxPackets;
    inArgs->chId       = chNum;
    inArgs->pktInfoQ   = &hLwip2Enet->rx[chNum].freePktInfoQ;
}

static void Lwip2Enet_initReleaseTxHandleInArgs(Lwip2Enet_Handle hLwip2Enet,
                                              uint32_t chNum,
                                              LwipifEnetAppIf_ReleaseTxHandleInfo *inArgs)
{
    inArgs->txFreePktCb  = &Lwip2Enet_freeTxPktCb;
	inArgs->txFreePktCbArg = &hLwip2Enet->tx[chNum];
    inArgs->txChNum       = chNum;
}

static void Lwip2Enet_initReleaseRxHandleInArgs(Lwip2Enet_Handle hLwip2Enet,
                                              uint32_t chNum,
                                              LwipifEnetAppIf_ReleaseRxHandleInfo *inArgs)
{

    if (hLwip2Enet->rx[chNum].enabled)
    {
        inArgs->rxFreePktCb  = &Lwip2Enet_freeRxPktCb;
        inArgs->rxFreePktCbArg = &hLwip2Enet->rx;
        inArgs->rxChNum       = chNum;
    }
}

void Lwip2Enet_setRxNotifyCallback(Lwip2Enet_Handle hLwip2Enet, Enet_notify_t *pRxPktNotify)
{
    hLwip2Enet->rxPktNotify = *pRxPktNotify;
}

void Lwip2Enet_setTxNotifyCallback(Lwip2Enet_Handle hLwip2Enet, Enet_notify_t *pTxPktNotify)
{
    hLwip2Enet->txPktNotify = *pTxPktNotify;
}
