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
#include "lwip/prot/ip.h"

#include <lwip/netif.h>

/**
 * Enet device specific header files
 */
#include <enet.h>
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

#define OS_TASKPRIHIGH              8

#define LWIPIF_RX_PACKET_TASK_PRI      (OS_TASKPRIHIGH)

#define LWIPIF_TX_PACKET_TASK_PRI      (OS_TASKPRIHIGH)

#define LWIP2ENET_DIVIDER_US_TO_MS  (1000U)

#if ENET_CFG_NETIF_MAX > 1U
#error "Multiple netif is not support in the version"
#endif
/*---------------------------------------------------------------------------*\
 |                         Local Function Declarations                         |
 \*---------------------------------------------------------------------------*/

static void Lwip2Enet_initRxObj(Lwip2Enet_Handle hLwip2Enet,
                                Lwip2Enet_RxObj *rx,
                                uint32_t numPkts);

static void Lwip2Enet_deinitRxObj(Lwip2Enet_Handle hLwip2Enet,
                                  Lwip2Enet_RxObj *rx);

static void Lwip2Enet_saveAppIfRxCfg(Lwip2Enet_RxObj *rx,
                                     LwipifEnetAppIf_RxInfo *rxInfo);

static void Lwip2Enet_prepRxQs(Lwip2Enet_RxObj *rx);

static void Lwip2Enet_stopRxTask(Lwip2Enet_RxObj *rx);

static void Lwip2Enet_initTxObj(Lwip2Enet_Handle hLwip2Enet,
                                Lwip2Enet_TxObj *tx,
                                uint32_t numPkts);
static void Lwip2Enet_saveAppIfTxCfg(Lwip2Enet_TxObj *tx,
                                     LwipifEnetAppIf_TxHandleInfo *txInfo);

static void Lwip2Enet_prepTxQs(Lwip2Enet_TxObj *tx);

static void Lwip2Enet_stopTxTask(Lwip2Enet_TxObj *tx);

static void Lwip2Enet_deinitTxObj(Lwip2Enet_Handle hLwip2Enet,
                                  Lwip2Enet_TxObj *tx);

static void Lwip2Enet_notifyRxPackets(void *cbArg);

static void Lwip2Enet_notifyTxPackets(void *cbArg);

static void Lwip2Enet_rxPacketTask(void *arg0);

static void Lwip2Enet_pbufQ2PktInfoQ(Lwip2Enet_TxObj *tx,
                                     pbufQ *pbufPktQ,
                                     EnetDma_PktQ *pDmaPktInfoQ);

static void Lwip2Enet_pktInfoQ2PbufQ(EnetDma_PktQ *pDmaPktInfoQ,
                                     pbufQ *pbufPktQ);

static void Lwip2Enet_allocRxPackets(Lwip2Enet_RxObj *rx);

static uint32_t Lwip2Enet_prepRxPktQ(Lwip2Enet_RxObj *rx,
                                    EnetDma_PktQ *pPktQ,
                                    uint32_t rxChNum);

static void Lwip2Enet_submitRxPktQ(Lwip2Enet_RxObj *rx);

static void Lwip2Enet_txPacketTask(void *arg0);

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

static void Lwip2Enet_stopRxTx(Lwip2Enet_Handle hLwip2Enet);

static void Lwip2Enet_submitTxPackets(Lwip2Enet_TxObj *tx,
                                      EnetDma_PktQ *pSubmitQ);

static void Lwip2Enet_submitRxPackets(Lwip2Enet_RxObj *rx,
                                     EnetDma_PktQ *pSubmitQ,
                                     uint32_t rxChNum);

static void Lwip2Enet_retrieveTxPkts(Lwip2Enet_TxObj *tx);

static void Lwip2Enet_timerCb(ClockP_Object *hClk, void * arg);

static void Lwip2Enet_createTimer(Lwip2Enet_Handle hLwip2Enet);

static void Lwip2Enet_initGetHandleInArgs(Lwip2Enet_Handle hLwip2Enet,
                                         LwipifEnetAppIf_GetHandleInArgs *inArgs);

static void Lwip2Enet_initReleaseHandleInArgs(Lwip2Enet_Handle hLwip2Enet,
                                             LwipifEnetAppIf_ReleaseHandleInfo *inArgs);

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
    LwipifEnetAppIf_GetHandleInArgs getHandleInArgs;
    Lwip2Enet_Handle hLwip2Enet;
    int32_t status;
    uint32_t i;

    hLwip2Enet = Lwip2Enet_getObj();
    if (hLwip2Enet->initDone == false)
    {

        /* MCast List is EMPTY */
        hLwip2Enet->MCastCnt = 0;

        /* Init internal bookkeeping fields */
        hLwip2Enet->oldMCastCnt = 0;

		/* For every pbuf sent to LWIP from netif->input we alloc a pbuf to queue to driver.
        * If pbuf alloc fails at that point increment this counted so that buffers can be reclaimed and
        * queued to driver in the periodic timer cb
        */
        for (i = 0U; i < LWIPIF_MAX_RX_CHANNELS; i++)
        {
            hLwip2Enet->rx.rxReclaimCount[i] = 0;
            hLwip2Enet->rx.lastRxReclaimCount[0U] = 0;
        }
	    hLwip2Enet->rx.rxAllocCount = 0;

        /* First init tasks, semaphores and clocks. This is required because
         * EnetDma event cb ISR can happen immediately after opening rx flow
         * in LwipifEnetAppCb_getHandle and all semaphores, clocks and tasks should
         * be valid at that point
         */

        /* Create semaphore objects, init shutDownFlag status */
        hLwip2Enet->shutDownFlag = false;

        status = SemaphoreP_constructBinary(&hLwip2Enet->shutDownSemObj, 0U);
        Lwip2Enet_assert(status == SystemP_SUCCESS);

        /* Initialize RX object, start RX task, create semaphores, etc */
	    Lwip2Enet_initRxObj(hLwip2Enet, &hLwip2Enet->rx, LWIP2ENET_RX_PACKETS);

        /* Initialize TX object, start TX task, create semaphores, etc */
        Lwip2Enet_initTxObj(hLwip2Enet, &hLwip2Enet->tx, LWIP2ENET_TX_PACKETS);

        /* Get Enet & DMA Drv Handle */
        Lwip2Enet_initGetHandleInArgs(hLwip2Enet, &getHandleInArgs);
        LwipifEnetAppCb_getHandle(&getHandleInArgs, &hLwip2Enet->appInfo);

        /* Save params received from application interface */
        Lwip2Enet_saveAppIfRxCfg(&hLwip2Enet->rx, &hLwip2Enet->appInfo.rxInfo);
        Lwip2Enet_saveAppIfTxCfg(&hLwip2Enet->tx, &hLwip2Enet->appInfo.txInfo);

        Lwip2Enet_assert(hLwip2Enet->appInfo.hEnet != NULL);
        Lwip2Enet_assert(hLwip2Enet->appInfo.isPortLinkedFxn != NULL);

        if (NULL != hLwip2Enet->appInfo.print)
        {
            /* set the print function callback if not null */
            hLwip2Enet->print = hLwip2Enet->appInfo.print;
        }
        else
        {
            hLwip2Enet->print = (Enet_Print) &EnetUtils_printf;
        }

        /* Open DMA driver */
        status = Lwip2Enet_startRxTx(hLwip2Enet);
        if (status != ENET_SOK)
        {
            Lwip2Enet_print(hLwip2Enet,"Failed to open DMA: %d\n", status);
        }

        /* Get initial link/interface status from the driver */
        hLwip2Enet->linkIsUp = hLwip2Enet->appInfo.isPortLinkedFxn(hLwip2Enet->appInfo.hEnet);

        if (hLwip2Enet->tx.disableEvent)
        {
            EnetDma_disableTxEvent(hLwip2Enet->tx.hCh);
        }


            if (hLwip2Enet->rx.enabled &&
                hLwip2Enet->rx.disableEvent)
            {
                for (i = 0U; i < hLwip2Enet->rx.numRxChannels; i++)
                {
                   EnetDma_disableRxEvent(hLwip2Enet->rx.hFlow[i]);
                }
            }

        /* assert if clk period is not valid  */
        Lwip2Enet_assert(0U != hLwip2Enet->appInfo.timerPeriodUs);
        Lwip2Enet_createTimer(hLwip2Enet);
        // ClockP_start(&hLwip2Enet->pacingClkObj);

        hLwip2Enet->initDone = TRUE;
	}
        if (hLwip2Enet != NULL)
    {
        /* Filling the netif parameters */
        for (i = 0U; i < hLwip2Enet->appInfo.numNetif; i++)
        {
            if (!hLwip2Enet->netifObj[i].inUse)
            {
                hLwip2Enet->netifObj[i].netif = netif;
                hLwip2Enet->netifObj[i].inUse = true;
                hLwip2Enet->netifObj[i].macPort = ENET_MACPORT_DENORM(i);

                /* Copy the MAC Address into the network interface object here. */
                memcpy(netif->hwaddr, &hLwip2Enet->netifObj[i].macAddr[0U], (uint32_t)6U);
                netif->hwaddr_len = 6U;
                netif->state = (void *)hLwip2Enet;

                break;
            }
        }
    }
    return hLwip2Enet;
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
    LwipifEnetAppIf_ReleaseHandleInfo releaseHandleInfo;

    Lwip2Enet_assert(NULL != hLwip2Enet);

    /* Set the translation layer shutdown flag */
    hLwip2Enet->shutDownFlag = true;

    /* Stop and delete the tick timer */
    ClockP_stop(&hLwip2Enet->pacingClkObj);
    ClockP_destruct(&hLwip2Enet->pacingClkObj);

    Lwip2Enet_stopRxTx(hLwip2Enet);

    /* Close RX flow and TX channel */
    Lwip2Enet_initReleaseHandleInArgs(hLwip2Enet, &releaseHandleInfo);
    LwipifEnetAppCb_releaseHandle(&releaseHandleInfo);

    if (hLwip2Enet->allocPktInfo > 0U)
    {
        Lwip2Enet_print(hLwip2Enet, "Lwip2Enet_close() failed to retrieve all PktInfo\n");
    }

    /* Deinit TX and RX objects, delete task, semaphore, etc */
    Lwip2Enet_deinitRxObj(hLwip2Enet, &hLwip2Enet->rx);
    Lwip2Enet_deinitTxObj(hLwip2Enet, &hLwip2Enet->tx);

    SemaphoreP_destruct(&hLwip2Enet->shutDownSemObj);

    Lwip2Enet_putObj(hLwip2Enet);
}

static void Lwip2Enet_initRxObj(Lwip2Enet_Handle hLwip2Enet,
                                Lwip2Enet_RxObj *rx,
                                uint32_t numPkts)
{
	TaskP_Params params;
	int32_t status;

    rx->hLwip2Enet = hLwip2Enet;
    rx->numPkts = numPkts;
    Lwip2Enet_assert(rx->numPkts <= ENET_ARRAYSIZE(rx->pktInfoMem));

    /* Create RX packet semaphore */
    status = SemaphoreP_constructBinary(&rx->rxPacketSemObj, 0U);
    Lwip2Enet_assert(status == SystemP_SUCCESS);

    /* Create RX packet task */
    TaskP_Params_init(&params);
    params.name = "Lwip2Enet_RxPacketTask";
    params.priority       = LWIPIF_RX_PACKET_TASK_PRI;
    params.stack          = &rx->pktTaskStack[0U];
    params.stackSize      = sizeof(rx->pktTaskStack);
    params.args           = &hLwip2Enet->rx;
    params.taskMain       = &Lwip2Enet_rxPacketTask;

    status = TaskP_construct(&rx->rxPacketTaskObj , &params);
    Lwip2Enet_assert(status == SystemP_SUCCESS);
}

static void Lwip2Enet_saveAppIfRxCfg(Lwip2Enet_RxObj *rx,
                                     LwipifEnetAppIf_RxInfo *rxInfo)
{
    rx->numRxChannels = rxInfo->numRxChannels;
    rx->disableEvent = rxInfo->disableEvent;

    for (uint32_t i = 0U; i < rx->numRxChannels; i++)
    {
        rx->flowIdx[i]     = rxInfo->rxHandle[i].rxFlowIdx;
        rx->hFlow[i]        = rxInfo->rxHandle[i].hRxFlow;
    	Lwip2Enet_assert(rx->hFlow[i] != NULL);
	    rx->enabled      = (rx->hFlow[i] != NULL);
        rx->flowStartIdx[i] = rxInfo->rxHandle[i].rxFlowStartIdx;
    }

    for (uint32_t i = 0U; i < rx->hLwip2Enet->appInfo.numNetif; i++)
    {
        EnetUtils_copyMacAddr(&rx->hLwip2Enet->netifObj[i].macAddr[0U], &rxInfo->macAddr[i][0U]);
    }
}

static void Lwip2Enet_prepRxQs(Lwip2Enet_RxObj *rx)
{
    Lwip2Enet_Handle hLwip2Enet = rx->hLwip2Enet;
    EnetDma_Pkt *pPktInfo;
    uint32_t i;

    /* Initialize queue of RX packet info */
    for (uint32_t i = 0U; i <  rx->numRxChannels; i++)
    {
        EnetQueue_initQ(&rx->freePktInfoQ[i]);
    }

    /* Allocate the free pkt info Q. This is used to exchange buffer info from
     * DMA basically for submitting free packets and retrieving ready packets */
    if(rx->numRxChannels == 2U)
    {
        for (i = 0U; i < rx->numPkts/2; i++)
        {
            pPktInfo = &rx->pktInfoMem[i];

            EnetDma_initPktInfo(pPktInfo);
            EnetQueue_enq(&rx->freePktInfoQ[0U], &pPktInfo->node);

            Lwip2EnetStats_addOne(&rx->stats.freeAppPktEnq);
            hLwip2Enet->allocPktInfo++;
        }

        for (i = rx->numPkts/2; i < rx->numPkts; i++)
        {
            pPktInfo = &rx->pktInfoMem[i];

            EnetDma_initPktInfo(pPktInfo);
            EnetQueue_enq(&rx->freePktInfoQ[1U], &pPktInfo->node);

            Lwip2EnetStats_addOne(&rx->stats.freeAppPktEnq);
            hLwip2Enet->allocPktInfo++;
        }
    }
    else
    {
        for (i = 0U; i < rx->numPkts; i++)
        {
            pPktInfo = &rx->pktInfoMem[i];

            EnetDma_initPktInfo(pPktInfo);
            EnetQueue_enq(&rx->freePktInfoQ[0U], &pPktInfo->node);

            Lwip2EnetStats_addOne(&rx->stats.freeAppPktEnq);
            hLwip2Enet->allocPktInfo++;
        }
    }

    for (i = 0U; i <  rx->numRxChannels; i++)
    {
        /* Initialize the RX pbuf queue */
        pbufQ_init(&rx->freePbufQ[i]);
    }

    /* Allocate the pbufs for the RX channel, primes freePbufQ */
    Lwip2Enet_allocRxPackets(rx);
}

static void Lwip2Enet_stopRxTask(Lwip2Enet_RxObj *rx)
{
    /* Post to rx packet task so that it will terminate (shutDownFlag flag is already set) */
    {
        SemaphoreP_post(&rx->rxPacketSemObj);
    }
}

static void Lwip2Enet_deinitRxObj(Lwip2Enet_Handle hLwip2Enet,
                                  Lwip2Enet_RxObj *rx)
{
    for (uint32_t i = 0U; i <  rx->numRxChannels; i++)
    {
        rx->hFlow[i] = NULL;
        hLwip2Enet->allocPktInfo -= EnetQueue_getQCount(&rx->freePktInfoQ[i]);
    }

    SemaphoreP_pend(&hLwip2Enet->shutDownSemObj, SystemP_WAIT_FOREVER);

    SemaphoreP_destruct(&rx->rxPacketSemObj);
}

static void Lwip2Enet_initTxObj(Lwip2Enet_Handle hLwip2Enet,
                                Lwip2Enet_TxObj *tx,
                                uint32_t numPkts)
{
	TaskP_Params params;
	int32_t status;

    tx->hLwip2Enet = hLwip2Enet;
    tx->numPkts = numPkts;
    Lwip2Enet_assert(tx->numPkts <= ENET_ARRAYSIZE(tx->pktInfoMem));

    /* Create TX packet semaphore */
    status = SemaphoreP_constructBinary(&hLwip2Enet->tx.txPacketSemObj, 0U);
    Lwip2Enet_assert(status == SystemP_SUCCESS);

    /* Create TX packet task */
    TaskP_Params_init(&params);
    params.name = "Lwip2Enet_TxPacketTask";
    params.priority       = LWIPIF_TX_PACKET_TASK_PRI;
    params.stack          = &tx->pktTaskStack[0U];
    params.stackSize      = sizeof(tx->pktTaskStack);
    params.args           = &hLwip2Enet->tx;
    params.taskMain       = &Lwip2Enet_txPacketTask;

    status = TaskP_construct(&hLwip2Enet->tx.txPacketTaskObj , &params);
    Lwip2Enet_assert(status == SystemP_SUCCESS);
}

static void Lwip2Enet_saveAppIfTxCfg(Lwip2Enet_TxObj *tx,
                                     LwipifEnetAppIf_TxHandleInfo *txInfo)
{
    tx->hCh   = txInfo->hTxChannel;
    tx->chNum = txInfo->txChNum;
	tx->portNum = txInfo->txPortNum;
    tx->disableEvent = txInfo->disableEvent;

    Lwip2Enet_assert(tx->hCh != NULL);
}

static void Lwip2Enet_prepTxQs(Lwip2Enet_TxObj *tx)
{
    Lwip2Enet_Handle hLwip2Enet = tx->hLwip2Enet;
    EnetDma_Pkt *pPktInfo;
    uint32_t i;

    /* Initialize the DMA free queue */
    EnetQueue_initQ(&tx->freePktInfoQ);

    /* Allocate the free pkt info Q. This is used to exchange buffer info from
     * DMA basically for submitting free packets and retrieving ready packets */
    for (i = 0U; i < tx->numPkts; i++)
    {
        /* Initialize Pkt info Q from allocated memory */
        pPktInfo = &tx->pktInfoMem[i];

        EnetDma_initPktInfo(pPktInfo);
        EnetQueue_enq(&tx->freePktInfoQ, &pPktInfo->node);

        Lwip2EnetStats_addOne(&tx->stats.freeAppPktEnq);
        hLwip2Enet->allocPktInfo++;
    }

    /* Initialize the TX pbuf queues */
    pbufQ_init(&tx->readyPbufQ);
    pbufQ_init(&tx->unusedPbufQ);
}

static void Lwip2Enet_stopTxTask(Lwip2Enet_TxObj *tx)
{
    /* Post to tx packet task so that it will terminate (shutDownFlag flag is already set) */
    {
        SemaphoreP_post(&tx->txPacketSemObj);
    }
}

static void Lwip2Enet_deinitTxObj(Lwip2Enet_Handle hLwip2Enet,
                                  Lwip2Enet_TxObj *tx)
{
    struct pbuf *pbuf;

    tx->hCh = NULL;

    /* Free pbuf in ready queue */
    while (pbufQ_count(&tx->readyPbufQ) != 0U)
    {
        pbuf = pbufQ_deQ(&tx->readyPbufQ);
        Lwip2Enet_assert(NULL != pbuf);
        pbuf_free(pbuf);
    }

    hLwip2Enet->allocPktInfo -= EnetQueue_getQCount(&tx->freePktInfoQ);

    SemaphoreP_pend(&hLwip2Enet->shutDownSemObj, SystemP_WAIT_FOREVER);

    SemaphoreP_destruct(&hLwip2Enet->tx.txPacketSemObj);
}

/*!
 *  @b Lwip2Enet_setRx
 *  @n
 *      Sets the filter for Ethernet peripheral. Sets up the multicast addresses in
 *      the ALE.
 *
 *  \param[in]  hLwip2Enet
 *      Lwip2Enet_object structure pointer.
 *
 *  \retval
 *      void
 */
void Lwip2Enet_setRx(Lwip2Enet_Handle hLwip2Enet)
{
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
void Lwip2Enet_sendTxPackets(Lwip2Enet_TxObj *tx)
{
    Lwip2Enet_Handle hLwip2Enet = tx->hLwip2Enet;
    EnetDma_Pkt *pCurrDmaPacket;
    struct pbuf *hPbufPkt;

    /* If link is not up, simply return */
    if (hLwip2Enet->linkIsUp)
    {
        EnetDma_PktQ txSubmitQ;

        EnetQueue_initQ(&txSubmitQ);

        if (pbufQ_count(&tx->unusedPbufQ))
        {
            /* send any pending TX Q's */
            Lwip2Enet_pbufQ2PktInfoQ(tx, &tx->unusedPbufQ, &txSubmitQ);
        }

        /* Check if there is anything to transmit, else simply return */
        while (pbufQ_count(&tx->readyPbufQ) != 0)
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

                pCurrDmaPacket->bufPtr     = (uint8_t *) hPbufPkt->payload;
                pCurrDmaPacket->appPriv    = hPbufPkt;
                pCurrDmaPacket->orgBufLen  = PBUF_POOL_BUFSIZE;
                pCurrDmaPacket->userBufLen = hPbufPkt->len;
                pCurrDmaPacket->txPortNum  = tx->portNum;
                pCurrDmaPacket->node.next  = NULL;
#if ((ENET_CFG_IS_ON(CPSW_CSUM_OFFLOAD_SUPPORT) == 1) && (ENET_ENABLE_PER_CPSW == 1))
                pCurrDmaPacket->chkSumInfo = LWIPIF_LWIP_getChkSumInfo(hPbufPkt);
#else
                pCurrDmaPacket->chkSumInfo = 0;
#endif
                ENET_UTILS_COMPILETIME_ASSERT(offsetof(EnetDma_Pkt, node) == 0);
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

 void Lwip2Enet_periodicFxn(Lwip2Enet_Handle hLwip2Enet)
{
    uint32_t prevLinkState     = hLwip2Enet->linkIsUp;
    uint32_t prevLinkInterface = hLwip2Enet->currLinkedIf;

#if (1U == ENET_CFG_DEV_ERROR)
    int32_t status, i;

    EnetQueue_verifyQCount(&hLwip2Enet->tx.freePktInfoQ);

    for(i = 0U; i < hLwip2Enet->rx.numRxChannels; i++)
    {
        EnetQueue_verifyQCount(&hLwip2Enet->rx.freePktInfoQ[i]);

        status = EnetUdma_checkRxFlowSanity(hLwip2Enet->rx.hFlow[i], 5U);
        if (status != ENET_SOK)
        {
            Lwip2Enet_print(hLwip2Enet, "EnetUdma_checkRxFlowSanity Failed\n");
        }
    }

    status = EnetUdma_checkTxChSanity(hLwip2Enet->tx.hCh, 5U);
    if (status != ENET_SOK)
    {
        Lwip2Enet_print(hLwip2Enet, "EnetUdma_checkTxChSanity Failed\n");
    }
#endif

    /*
     * Return the same DMA packets back to the DMA channel (but now
     * associated with a new PBUF Packet and buffer)
     */
    if(hLwip2Enet->rx.numRxChannels == 2U)
    {
        if ((pbufQ_count(&hLwip2Enet->rx.freePbufQ[0U]) != 0U) ||   (pbufQ_count(&hLwip2Enet->rx.freePbufQ[1U]) != 0U))
        {
            Lwip2Enet_submitRxPktQ(&hLwip2Enet->rx);
        }
    }
    else
    {
        if (pbufQ_count(&hLwip2Enet->rx.freePbufQ[0U]) != 0U)
        {
            Lwip2Enet_submitRxPktQ(&hLwip2Enet->rx);
        }
    }

#if defined(LWIPIF_INSTRUMENTATION_ENABLED)
    static uint32_t loadCount = 0;
    TaskP_Load stat;

    hLwip2Enet->stats.cpuLoad[loadCount] = TaskP_loadGetTotalCpuLoad();

    TaskP_loadGet(&hLwip2Enet->rx.rxPacketTaskObj, &stat);
    hLwip2Enet->rx.stats.pktStats.taskLoad[loadCount] = stat.cpuLoad;

    TaskP_loadGet(&hLwip2Enet->tx.txPacketTaskObj, &stat);
    hLwip2Enet->tx.stats.pktStats.taskLoad[loadCount] = stat.cpuLoad;

    loadCount = (loadCount + 1U) & (HISTORY_CNT - 1U);
#endif

    /* Get current link status as reported by the hardware driver */
    hLwip2Enet->linkIsUp = hLwip2Enet->appInfo.isPortLinkedFxn(hLwip2Enet->appInfo.hEnet);

    /* If the interface changed, discard any queue packets (since the MAC would now be wrong) */
    if (prevLinkInterface != hLwip2Enet->currLinkedIf)
    {
        /* ToDo: Discard all queued packets */
    }

    /* If link status changed from down->up, then send any queued packets */
    if ((prevLinkState == 0) && (hLwip2Enet->linkIsUp))
    {
        Lwip2Enet_sendTxPackets(&hLwip2Enet->tx);
    }
}

static void Lwip2Enet_processRxUnusedQ(Lwip2Enet_RxObj *rx,
                                       EnetDma_PktQ *unUsedQ,
                                       uint32_t rxChNum)
{
    EnetDma_Pkt *pDmaPacket;

    pDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(unUsedQ);
    while (pDmaPacket != NULL)
    {
        /* Get the full PBUF packet that needs to be returned to the rx.freePbufQ */
        struct pbuf* hPbufPacket = (struct pbuf *)pDmaPacket->appPriv;
        if (hPbufPacket)
        {
            /* Enqueue the received packet to rx.freePbufQ*/
            pbufQ_enQ(&rx->freePbufQ[rxChNum], hPbufPacket);

            /* Put packet info into free Q as we have removed the Pbuf buffers
             * from the it */
            EnetQueue_enq(&rx->freePktInfoQ[rxChNum], &pDmaPacket->node);
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
                                      EnetDma_PktQ *pSubmitQ,
                                      uint32_t rxChNum)
{
    int32_t retVal;

    retVal = EnetDma_submitRxPktQ(rx->hFlow[rxChNum], pSubmitQ);
    if (ENET_SOK != retVal)
    {
		Lwip2Enet_print(rx->hLwip2Enet,
                        "EnetDma_submitRxPktQ: failed to submit pkts: %d\n",
                        retVal);
    }

    if (EnetQueue_getQCount(pSubmitQ))
    {
        /* Copy unused packets to back to readyQ */
        Lwip2Enet_processRxUnusedQ(rx, pSubmitQ, rxChNum);
    }
}

/* May lead to infinite loop if no free memory
 * available*/
static void Lwip2Enet_pbufQ2PktInfoQ(Lwip2Enet_TxObj *tx,
                                     pbufQ *pbufPktQ,
                                     EnetDma_PktQ *pDmaPktInfoQ)
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

            pCurrDmaPacket->bufPtr = (uint8_t *) hPbufPkt->payload;
            pCurrDmaPacket->appPriv    = hPbufPkt;
            pCurrDmaPacket->orgBufLen  = PBUF_POOL_BUFSIZE;
            pCurrDmaPacket->userBufLen = hPbufPkt->len;
            pCurrDmaPacket->node.next = NULL;
#if ((ENET_CFG_IS_ON(CPSW_CSUM_OFFLOAD_SUPPORT) == 1) && (ENET_ENABLE_PER_CPSW == 1))
                pCurrDmaPacket->chkSumInfo = LWIPIF_LWIP_getChkSumInfo(hPbufPkt);
#else
                pCurrDmaPacket->chkSumInfo = 0;
#endif
            ENET_UTILS_COMPILETIME_ASSERT(offsetof(EnetDma_Pkt, node) == 0);
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
    Lwip2Enet_RxObj *rx = (Lwip2Enet_RxObj *)cbArg;
    Lwip2Enet_Handle hLwip2Enet = rx->hLwip2Enet;

    /* do not post events if init not done or shutdown in progress */
    if ((hLwip2Enet->initDone) && (hLwip2Enet->shutDownFlag == false))
    {
		if (rx->enabled)
        {
            for(uint32_t i = 0U; i < rx->numRxChannels; i++)
            {
                EnetDma_disableRxEvent(rx->hFlow[i]);
            }

            /* Post semaphore to RX handling task */
            SemaphoreP_post(&rx->rxPacketSemObj);
        }
    }
}

static void Lwip2Enet_notifyTxPackets(void *cbArg)
{
    Lwip2Enet_TxObj *tx = (Lwip2Enet_TxObj *)cbArg;
    Lwip2Enet_Handle hLwip2Enet = tx->hLwip2Enet;

    /* do not post events if init not done or shutdown in progress */
    if ((hLwip2Enet->initDone) && (hLwip2Enet->shutDownFlag == false))
    {
		/* Post semaphore to TX handling task */
        SemaphoreP_post(&tx->txPacketSemObj);
    }
}

static void Lwip2Enet_rxPacketTask(void *arg0)
{
	Lwip2Enet_RxObj *rx = (Lwip2Enet_RxObj *)arg0;
    Lwip2Enet_Handle hLwip2Enet = rx->hLwip2Enet;
    EnetDma_PktQ tempQueue;
    int32_t retVal;
    uint32_t pktCnt, rxChNum;

    while (!hLwip2Enet->shutDownFlag)
    {
        /* Wait for the Rx ISR to notify us that packets are available with data */
        SemaphoreP_pend(&rx->rxPacketSemObj, SystemP_WAIT_FOREVER);

        if (hLwip2Enet->shutDownFlag)
        {
            /* This translation layer is shutting down, don't give anything else to the stack */
            break;
        }

        LWIP2ENETSTATS_ADDONE(&rx->stats.pktStats.rawNotifyCnt);
        pktCnt = 0;

        for(rxChNum = 0; rxChNum < rx->numRxChannels; rxChNum++)
        {
            /* Retrieve the used (filled) packets from the channel */
            {
                EnetQueue_initQ(&tempQueue);
                retVal = EnetDma_retrieveRxPktQ(rx->hFlow[rxChNum], &tempQueue);
                if (ENET_SOK != retVal)
                {
                    Lwip2Enet_print(hLwip2Enet,
                                    "Lwip2Enet_rxPacketTask: failed to retrieve RX pkts: %d\n",
                                    retVal);
                }
            }
            if (tempQueue.count == 0)
            {
                LWIP2ENETSTATS_ADDONE(&rx->stats.pktStats.zeroNotifyCnt);
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
                pktCnt = Lwip2Enet_prepRxPktQ(rx, &tempQueue, rxChNum);
            }

            /*
             * We don't want to time the semaphore post used to notify the LwIP stack as that may cause a
             * task transition. We don't want to time the semaphore pend, since that would time us doing
             * nothing but waiting.
             */
            if (pktCnt != 0)
            {
                Lwip2Enet_updateRxNotifyStats(&rx->stats.pktStats, pktCnt, 0U);
            }

            // ClockP_start(&hLwip2Enet->pacingClkObj);

            if (!rx->disableEvent)
            {
                EnetDma_enableRxEvent(rx->hFlow[rxChNum]);
            }
        }
    }

    /* We are shutting down, notify that we are done */
    SemaphoreP_post(&hLwip2Enet->shutDownSemObj);
}

static void Lwip2Enet_txPacketTask(void *arg0)
{
    Lwip2Enet_TxObj *tx = (Lwip2Enet_TxObj *)arg0;
    Lwip2Enet_Handle hLwip2Enet = tx->hLwip2Enet;

    while (!hLwip2Enet->shutDownFlag)
    {
        /*
         * Wait for the Tx ISR to notify us that empty packets are available
         * that were used to send data
         */
        SemaphoreP_pend(&tx->txPacketSemObj, SystemP_WAIT_FOREVER);
        Lwip2Enet_retrieveTxPkts(tx);
    }

    /* We are shutting down, notify that we are done */
    SemaphoreP_post(&hLwip2Enet->shutDownSemObj);
}

static void Lwip2Enet_allocRxPackets(Lwip2Enet_RxObj *rx)
{
	Lwip2Enet_Handle hLwip2Enet = rx->hLwip2Enet;
    struct pbuf* hPbufPacket;
    uint32_t bufSize;
    uint32_t i;

    Lwip2Enet_assert(hLwip2Enet->appInfo.hostPortRxMtu != 0);

    /*
     * Pre-allocate twice as many lwIP stack packets as we plan to give to/get from the hardware.
     * The idea here is that even if we fill up all the DMA descriptors submit queue,
     * we will have another complete set to swap in right away.
     */
    // Make sure this is defined as twice what is required from the hardware
    /* Creates just as many Rx pbufs as the driver free packets*/
    if(rx->numRxChannels == 2U)
    {
        for (i = 0U; i < ((uint32_t)LWIP2ENET_RX_PACKETS/2); i++)
        {
            bufSize = ENET_UTILS_ALIGN(hLwip2Enet->appInfo.hostPortRxMtu, ENETDMA_CACHELINE_ALIGNMENT);

            hPbufPacket = pbuf_alloc(PBUF_RAW, bufSize, PBUF_POOL);
            if (hPbufPacket != NULL)
            {
                Lwip2Enet_assert(hPbufPacket->payload != NULL);
                Lwip2Enet_assert(ENET_UTILS_IS_ALIGNED(hPbufPacket->payload, ENETDMA_CACHELINE_ALIGNMENT));

                /* Enqueue to the free queue */
                pbufQ_enQ(&rx->freePbufQ[0U], hPbufPacket);

                LWIP2ENETSTATS_ADDONE(&rx->stats.freePbufPktEnq);
            }
            else
            {
                Lwip2Enet_print(hLwip2Enet, "ERROR: Pbuf_alloc() failure...exiting!\n");
                Lwip2Enet_assert(FALSE);
            }
        }

        for (i = (LWIP2ENET_RX_PACKETS/2); i < ((uint32_t)LWIP2ENET_RX_PACKETS); i++)
        {
            bufSize = ENET_UTILS_ALIGN(hLwip2Enet->appInfo.hostPortRxMtu, ENETDMA_CACHELINE_ALIGNMENT);

            hPbufPacket = pbuf_alloc(PBUF_RAW, bufSize, PBUF_POOL);
            if (hPbufPacket != NULL)
            {
                Lwip2Enet_assert(hPbufPacket->payload != NULL);
                Lwip2Enet_assert(ENET_UTILS_IS_ALIGNED(hPbufPacket->payload, ENETDMA_CACHELINE_ALIGNMENT));

                /* Enqueue to the free queue */
                pbufQ_enQ(&rx->freePbufQ[1U], hPbufPacket);

                LWIP2ENETSTATS_ADDONE(&rx->stats.freePbufPktEnq);
            }
            else
            {
                Lwip2Enet_print(hLwip2Enet, "ERROR: Pbuf_alloc() failure...exiting!\n");
                Lwip2Enet_assert(FALSE);
            }
        }
    }
    else
    {
        for (i = 0U; i < ((uint32_t)LWIP2ENET_RX_PACKETS); i++)
        {
            bufSize = ENET_UTILS_ALIGN(hLwip2Enet->appInfo.hostPortRxMtu, ENETDMA_CACHELINE_ALIGNMENT);

            hPbufPacket = pbuf_alloc(PBUF_RAW, bufSize, PBUF_POOL);
            if (hPbufPacket != NULL)
            {
                Lwip2Enet_assert(hPbufPacket->payload != NULL);
                Lwip2Enet_assert(ENET_UTILS_IS_ALIGNED(hPbufPacket->payload, ENETDMA_CACHELINE_ALIGNMENT));

                /* Enqueue to the free queue */
                pbufQ_enQ(&rx->freePbufQ[0U], hPbufPacket);

                LWIP2ENETSTATS_ADDONE(&rx->stats.freePbufPktEnq);
            }
            else
            {
                Lwip2Enet_print(hLwip2Enet, "ERROR: Pbuf_alloc() failure...exiting!\n");
                Lwip2Enet_assert(FALSE);
            }
        }
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
    uint32_t rxChNum;

    for(rxChNum = 0; rxChNum < rx->numRxChannels; rxChNum++)
    {
        EnetQueue_initQ(&resubmitPktQ);

        /*
         * Fill in as many packets as we can with new PBUF buffers so they can be
         * returned to the stack to be filled in.
         */
        pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(&rx->freePktInfoQ[rxChNum]);

        while (NULL != pCurrDmaPacket)
        {
            hPbufPacket = pbufQ_deQ(&rx->freePbufQ[rxChNum]);
            if (hPbufPacket)
            {
                LWIP2ENETSTATS_ADDONE(&rx->stats.freePbufPktDeq);
                LWIP2ENETSTATS_ADDONE(&rx->stats.freeAppPktDeq);

                EnetDma_initPktInfo(pCurrDmaPacket);
                pCurrDmaPacket->bufPtr = (uint8_t *) hPbufPacket->payload;
                pCurrDmaPacket->orgBufLen = hPbufPacket->len;
                pCurrDmaPacket->userBufLen = hPbufPacket->len;

                /* Save off the PBM packet handle so it can be handled by this layer later */
                pCurrDmaPacket->appPriv = (void *)hPbufPacket;
                EnetQueue_enq(&resubmitPktQ, &pCurrDmaPacket->node);

                pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(&rx->freePktInfoQ[rxChNum]);
            }
            else
            {
                EnetQueue_enq(&rx->freePktInfoQ[rxChNum], &pCurrDmaPacket->node);
                break;
            }
        }

        /*
         * Return the same DMA packets back to the DMA channel (but now
         * associated with a new PBM Packet and buffer)
         */
        if (EnetQueue_getQCount(&resubmitPktQ))
        {
            Lwip2Enet_submitRxPackets(rx, &resubmitPktQ, rxChNum);
        }
    }
}

static uint32_t Lwip2Enet_prepRxPktQ(Lwip2Enet_RxObj *rx,
                                     EnetDma_PktQ *pPktQ,
                                     uint32_t rxChNum)
{
    Lwip2Enet_Handle hLwip2Enet = rx->hLwip2Enet;
    uint32_t packetCount = 0;
    EnetDma_Pkt *pCurrDmaPacket;

    pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(pPktQ);
    while (pCurrDmaPacket)
    {
        /* Get the full PBUF packet that needs to be returned to the LwIP stack */
        struct pbuf* hPbufPacket = (struct pbuf *)pCurrDmaPacket->appPriv;
        bool isChksumError = false;
        if (hPbufPacket)
        {
            uint32_t validLen = pCurrDmaPacket->userBufLen;

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
#if ((ENET_CFG_NETIF_MAX == 2U) && (ENET_ENABLE_PER_CPSW == 1U))
                if(pCurrDmaPacket->rxPortNum == ENET_MAC_PORT_1)
                {
                    LWIPIF_LWIP_input(hLwip2Enet->netifObj[0U].netif, rx, hPbufPacket, rxChNum);
                }
                else if(pCurrDmaPacket->rxPortNum == ENET_MAC_PORT_2)
                {
                    LWIPIF_LWIP_input(hLwip2Enet->netifObj[1U].netif, rx, hPbufPacket, rxChNum);
                }
#else
                LWIPIF_LWIP_input(hLwip2Enet->netifObj[0U].netif, rx, hPbufPacket, rxChNum);
#endif
                packetCount++;
            }
            else
            {
                /* Put PBUF buffer in free Q as we are not passing to stack */
                pbufQ_enQ(&rx->freePbufQ[rxChNum], hPbufPacket);
                LWIP2ENETSTATS_ADDONE(&rx->stats.chkSumErr);
            }

            /* Put packet info into free Q as we have removed the PBUF buffers
             * from the it */
            EnetQueue_enq(&rx->freePktInfoQ[rxChNum], &pCurrDmaPacket->node);
            LWIP2ENETSTATS_ADDONE(&rx->stats.freeAppPktEnq);

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

        buf = &hLwip2Enet->printBuf[0];
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

    Lwip2Enet_assert(NULL != hLwip2Enet->tx.hCh);

    for(uint32_t i = 0U; i< hLwip2Enet->rx.numRxChannels; i++)
    {
        Lwip2Enet_assert(NULL != hLwip2Enet->rx.hFlow[i]);
    }

    status = EnetDma_enableTxEvent(hLwip2Enet->tx.hCh);

    /* Initialize free DMA packet queues */
    Lwip2Enet_prepTxQs(&hLwip2Enet->tx);
    Lwip2Enet_prepRxQs(&hLwip2Enet->rx);

    /* Submit all allocated packets to DMA so it can use for packet RX */
    Lwip2Enet_submitRxPktQ(&hLwip2Enet->rx);

    return status;
}

static void Lwip2Enet_stopRxTx(Lwip2Enet_Handle hLwip2Enet)
{
	/* Stop RX packet task */
    Lwip2Enet_stopRxTask(&hLwip2Enet->rx);

    /* Stop TX packet task */
    Lwip2Enet_stopTxTask(&hLwip2Enet->tx);
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
                                  EnetDma_PktQ *cqPktInfoQ,
                                  uint32_t rxChNum)
{
    Lwip2Enet_RxObj *rx = (Lwip2Enet_RxObj *)cbArg;

    /* Now that we free PBUF buffers, push all freed pktInfo's into Rx
     * free Q */
    EnetQueue_append(&rx->freePktInfoQ[rxChNum], fqPktInfoQ);
    LWIP2ENETSTATS_ADDNUM(&rx->stats.freeAppPktEnq, EnetQueue_getQCount(fqPktInfoQ));
    Lwip2Enet_freePbufPackets(fqPktInfoQ);

    EnetQueue_append(&rx->freePktInfoQ[rxChNum], cqPktInfoQ);
    LWIP2ENETSTATS_ADDNUM(&rx->stats.freeAppPktEnq, EnetQueue_getQCount(cqPktInfoQ));
    Lwip2Enet_freePbufPackets(cqPktInfoQ);

    /* Flush out our pending receive queues */
    while (pbufQ_count(&rx->freePbufQ[rxChNum]) != 0)
    {
        pbuf_free(pbufQ_deQ(&rx->freePbufQ[rxChNum]));
        LWIP2ENETSTATS_ADDONE(&rx->stats.freePbufPktDeq);
    }
}

static void Lwip2Enet_retrieveTxPkts(Lwip2Enet_TxObj *tx)
{
    EnetDma_PktQ tempQueue;
    uint32_t packetCount = 0;
    int32_t retVal;

    LWIP2ENETSTATS_ADDONE(&tx->stats.pktStats.rawNotifyCnt);
    packetCount = 0;

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

    if (packetCount != 0)
    {
        Lwip2Enet_updateTxNotifyStats(&tx->stats.pktStats, packetCount, 0U);
    }
}

static void Lwip2Enet_enqueueFreedPbufRxPackets(Lwip2Enet_Handle hLwip2Enet)
{
    uint32_t bufSize;
    struct pbuf *hPbufPacket;
    uint32_t rxReclaimCount;
    uint32_t pbufAllocCount[LWIPIF_MAX_RX_CHANNELS] = {0};
    uint32_t i, rxChNum;

    /* Allocate a new Pbuf packet to be used */
    bufSize = ENET_UTILS_ALIGN(hLwip2Enet->appInfo.hostPortRxMtu, ENETDMA_CACHELINE_ALIGNMENT);

    for(rxChNum = 0; rxChNum < hLwip2Enet->rx.numRxChannels; rxChNum++)
    {
        rxReclaimCount = hLwip2Enet->rx.rxReclaimCount[rxChNum] - hLwip2Enet->rx.lastRxReclaimCount[rxChNum];
        for (i = 0; i < rxReclaimCount;i++)
        {
            hPbufPacket = pbuf_alloc(PBUF_RAW, bufSize, PBUF_POOL);
            if (hPbufPacket != NULL)
            {
                Lwip2Enet_assert(hPbufPacket->payload != NULL);
                /* Ensures that the ethernet frame is always on a fresh cacheline */
                Lwip2Enet_assert(ENET_UTILS_IS_ALIGNED(hPbufPacket->payload, ENETDMA_CACHELINE_ALIGNMENT));
                /* Put the new packet on the free queue */
                pbufQ_enQ(&hLwip2Enet->rx.freePbufQ[rxChNum], hPbufPacket);
                LWIP2ENETSTATS_ADDONE(&hLwip2Enet->rx.stats.freePbufPktEnq);
                pbufAllocCount[rxChNum]++;
            }
            else
            {
                break;
            }
        }
    }
    Lwip2Enet_submitRxPktQ(&hLwip2Enet->rx);

    for (i = 0U; i < hLwip2Enet->rx.numRxChannels; i++)
    {
        hLwip2Enet->rx.rxAllocCount += pbufAllocCount[i];
        hLwip2Enet->rx.lastRxReclaimCount[i]  += pbufAllocCount[i];
    }
}

static void Lwip2Enet_timerCb(ClockP_Object *hClk, void * arg)
{
    /* Post semaphore to rx handling task */
    Lwip2Enet_Handle hLwip2Enet = (Lwip2Enet_Handle)arg;

    if ((hLwip2Enet->initDone) && (hLwip2Enet->shutDownFlag == false))
    {
        if (hLwip2Enet->rx.enabled)
        {
            Lwip2Enet_retrieveTxPkts(&hLwip2Enet->tx);
            Lwip2Enet_enqueueFreedPbufRxPackets(hLwip2Enet);
            SemaphoreP_post(&hLwip2Enet->rx.rxPacketSemObj);
        }
    }
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

static void  Lwip2Enet_initGetHandleInArgs(Lwip2Enet_Handle hLwip2Enet,
                                          LwipifEnetAppIf_GetHandleInArgs *inArgs)
{
    inArgs->txCfg.cbArg      = &hLwip2Enet->tx;
    inArgs->txCfg.notifyCb   = &Lwip2Enet_notifyTxPackets;
    inArgs->txCfg.numPackets = hLwip2Enet->tx.numPkts;

    inArgs->rxCfg.cbArg      = &hLwip2Enet->rx;
    inArgs->rxCfg.notifyCb   = &Lwip2Enet_notifyRxPackets;
    inArgs->rxCfg.numPackets = hLwip2Enet->rx.numPkts;
}

static void Lwip2Enet_initReleaseHandleInArgs(Lwip2Enet_Handle hLwip2Enet,
                                             LwipifEnetAppIf_ReleaseHandleInfo *inArgs)
{
    inArgs->coreId       = hLwip2Enet->appInfo.coreId;
    inArgs->coreKey      = hLwip2Enet->appInfo.coreKey;
    inArgs->hEnet        = hLwip2Enet->appInfo.hEnet;
    inArgs->hUdmaDrv     = hLwip2Enet->appInfo.hUdmaDrv;

    if (hLwip2Enet->rx.enabled)
    {
	inArgs->rxFreePktCb  = &Lwip2Enet_freeRxPktCb;
    inArgs->rxFreePktCbArg = &hLwip2Enet->rx;
    inArgs->rxInfo       = hLwip2Enet->appInfo.rxInfo;
    }

    inArgs->txFreePktCb  = &Lwip2Enet_freeTxPktCb;
	inArgs->txFreePktCbArg = &hLwip2Enet->tx;
    inArgs->txInfo       = hLwip2Enet->appInfo.txInfo;
}
