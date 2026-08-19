/*
 *  Copyright (C) Texas Instruments Incorporated 2026
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
 * \file  tsndemo_dataflow.c
 *
 * \brief TSN demo - datapath: DMA channel open/close, packet pools, RX task
 *        with demo-frame dispatch, TX helpers and the software timestamp
 *        read.
 *
 *        T2/T4 come for free as hardware ingress timestamps in
 *        pktInfo->tsInfo.rxPktTs (all CPDMA RX packets are timestamped).
 *        T1/T3 are software timestamps read here via
 *        ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <string.h>
#include "tsndemo.h"

#include "ti_enet_config.h"
#include "ti_enet_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TSNDEMO_RX_TASK_PRI             (14U)  /* flow 0: bulk + default   */
#define TSNDEMO_RX_PROBE_TASK_PRI       (15U)  /* flow 1: probe/echo, ALE-
                                                   classified, strictly
                                                   above flow 0 (this is
                                                   the whole point)        */
#define TSNDEMO_RX_TASK_STACK           (8U * 1024U)

#define TSNDEMO_VLAN_TPID               (0x8100U)

/* ========================================================================== */
/*                            Local Variables                                 */
/* ========================================================================== */

static uint8_t gTsnDemoRxTaskStack[TSNDEMO_RX_TASK_STACK]
__attribute__ ((aligned(32)));
static uint8_t gTsnDemoRxProbeTaskStack[TSNDEMO_RX_TASK_STACK]
__attribute__ ((aligned(32)));

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void TsnDemoFlow_rxIsrCb(void *appData)
{
    TsnDemo_RxWorker *worker = (TsnDemo_RxWorker *)appData;

    SemaphoreP_post(&worker->semObj);
}

static void TsnDemoFlow_fillTxPool(EnetDma_PktQ *pool, uint32_t count)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };
    uint32_t i;

    for (i = 0U; i < count; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gTsnDemo,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState,
                                     ENET_PKTSTATE_APP_WITH_FREEQ);

        /*
         * Fill the payload area ONCE, here. Bulk frames are pure load: the
         * generator rewrites only the demo header and never touches the
         * payload again. At 1 Gbit a 1450 B frame leaves ~12 us per frame,
         * and a per-frame memset of the payload would be ~117 MB/s of
         * memcpy bandwidth that the R5F cannot sustain.
         */
        memset(pPktInfo->sgList.list[0].bufPtr, 0xA5,
               ENET_MEM_LARGE_POOL_PKT_SIZE);

        EnetQueue_enq(pool, &pPktInfo->node);
    }
}

/* The listener never generates its own bulk traffic, but it echoes
 * whatever PCP it receives (TsnDemoListener_rxFrameCb), including the
 * bulk-PCP EST verification probes from TsnDemoTalker_estVerify. Reclaim
 * routes completed TX packets back by PCP (see TsnDemoFlow_reclaimTxPkts
 * below), so without a real bulk pool such an echo would permanently
 * drain a packet out of the smaller express pool it was actually
 * allocated from. A small dedicated pool is enough for this occasional
 * traffic. */
#define TSNDEMO_LISTENER_BULKPCP_ECHO_POOL     (16U)

static void TsnDemoFlow_initTxFreePktQ(void)
{
#ifdef TSNDEMO_ROLE_LISTENER
    /* The listener never transmits bulk - every buffer on its single TX
     * channel goes to the echo pool - except the small bulk-PCP echo pool
     * above. */
    TsnDemoFlow_fillTxPool(&gTsnDemo.txFreePktInfoQ, ENET_DMA_TX_CH0_NUM_PKTS);
    TsnDemoFlow_fillTxPool(&gTsnDemo.bulkFreePktInfoQ,
                           TSNDEMO_LISTENER_BULKPCP_ECHO_POOL);
#else
    /* Talker: bulk and express/probe each have their own dedicated TX
     * channel and pool (ENET_DMA_TX_CH0 / ENET_DMA_TX_CH1) so a saturating
     * bulk stream cannot queue ahead of express in a shared ring - see
     * gTsnDemo.hTxCh / hTxChExpress. */
    TsnDemoFlow_fillTxPool(&gTsnDemo.bulkFreePktInfoQ, ENET_DMA_TX_CH0_NUM_PKTS);
    TsnDemoFlow_fillTxPool(&gTsnDemo.txFreePktInfoQ, ENET_DMA_TX_CH1_NUM_PKTS);
#endif

    EnetAppUtils_print("TX pools: express %u pkts, bulk %u pkts "
                       "(payload prefilled 0xA5)\r\n",
                       EnetQueue_getQCount(&gTsnDemo.txFreePktInfoQ),
                       EnetQueue_getQCount(&gTsnDemo.bulkFreePktInfoQ));
}

static void TsnDemoFlow_initRxReadyPktQ(EnetDma_RxChHandle hRxCh)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_Pkt *pPktInfo;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };
    int32_t status;
    uint32_t i;

    EnetQueue_initQ(&rxFreeQ);
    for (i = 0U; i < ENET_DMA_RX_CH0_NUM_PKTS; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gTsnDemo,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState,
                                     ENET_PKTSTATE_APP_WITH_FREEQ);
        EnetQueue_enq(&rxFreeQ, &pPktInfo->node);
    }

    EnetQueue_initQ(&rxReadyQ);
    status = EnetDma_retrieveRxPktQ(hRxCh, &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&rxFreeQ,
                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                     ENET_PKTSTATE_APP_WITH_DRIVER);
    EnetDma_submitRxPktQ(hRxCh, &rxFreeQ);
    EnetAppUtils_assert(EnetQueue_getQCount(&rxFreeQ) == 0U);
}

/*
 * Steer probe/echo (EtherType 0x88B5) to RX flow 1 in hardware via an ALE
 * policer/thread classifier, and everything else to flow 0 as the default
 * thread - same pattern as examples/enet_layer2_multi_channel's PTP
 * steering (CpswAle_SetPolicerEntryInArgs with
 * CPSW_ALE_POLICER_MATCH_ETHERTYPE + threadId = target flow's rxChNum),
 * just matching our own EtherType instead of PTP's.
 *
 * This is what makes the priority split (TSNDEMO_RX_PROBE_TASK_PRI above
 * TSNDEMO_RX_TASK_PRI) mean anything: without it, both EtherTypes would
 * land on flow 0 and the probe worker would sit idle.
 */
static int32_t TsnDemoFlow_setupAleClassifier(void)
{
    Enet_IoctlPrms prms;
    CpswAle_SetPolicerEntryInArgs inArgs;
    CpswAle_SetPolicerEntryOutArgs outArgs;
    CpswAle_DfltThreadCfg dfltThreadCfg;
    int32_t status;

    memset(&inArgs, 0, sizeof(inArgs));
    inArgs.policerMatch.policerMatchEnMask = CPSW_ALE_POLICER_MATCH_ETHERTYPE;
    inArgs.policerMatch.etherType = TSNDEMO_ETHERTYPE;
    inArgs.threadIdEn = true;
    inArgs.threadId   = gTsnDemo.rxWorker[TSNDEMO_RXW_PROBE].rxChNum;
    inArgs.peakRateInBitsPerSec   = 0U;   /* 0 = steer only, no policing */
    inArgs.commitRateInBitsPerSec = 0U;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &outArgs);
    ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId, CPSW_ALE_IOCTL_SET_POLICER,
               &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print(
            "ALE classifier: SET_POLICER (EtherType 0x%04x -> flow %u) "
            "failed: %d\r\n", TSNDEMO_ETHERTYPE,
            gTsnDemo.rxWorker[TSNDEMO_RXW_PROBE].rxChNum, status);
    }

    if (status == ENET_SOK)
    {
        dfltThreadCfg.dfltThreadEn = true;
        dfltThreadCfg.threadId     = gTsnDemo.rxWorker[TSNDEMO_RXW_DEFAULT].rxChNum;

        ENET_IOCTL_SET_IN_ARGS(&prms, &dfltThreadCfg);
        ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                   CPSW_ALE_IOCTL_SET_DEFAULT_THREADCFG, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print(
                "ALE classifier: SET_DEFAULT_THREADCFG (flow %u) failed: "
                "%d\r\n", gTsnDemo.rxWorker[TSNDEMO_RXW_DEFAULT].rxChNum,
                status);
        }
    }

    if (status == ENET_SOK)
    {
        EnetAppUtils_print(
            "ALE classifier: EtherType 0x%04x -> RX flow %u (probe/echo, pri "
            "%u); default -> RX flow %u (bulk, pri %u)\r\n",
            TSNDEMO_ETHERTYPE, gTsnDemo.rxWorker[TSNDEMO_RXW_PROBE].rxChNum,
            TSNDEMO_RX_PROBE_TASK_PRI, gTsnDemo.rxWorker[TSNDEMO_RXW_DEFAULT].rxChNum,
            TSNDEMO_RX_TASK_PRI);
    }

    return status;
}

int32_t TsnDemoFlow_open(void)
{
    EnetApp_GetDmaHandleInArgs txInArgs;
    EnetApp_GetTxDmaHandleOutArgs txChInfo;
    EnetApp_GetDmaHandleInArgs rxInArgs;
    EnetApp_GetRxDmaHandleOutArgs rxChInfo;
    TsnDemo_RxWorker *worker;
    int32_t status = ENET_SOK;

    /* TX channel */
    txInArgs.cbArg    = NULL;
    txInArgs.notifyCb = NULL;

    EnetApp_getTxDmaHandle(ENET_DMA_TX_CH0, &txInArgs, &txChInfo);
    gTsnDemo.txChNum = txChInfo.txChNum;
    gTsnDemo.hTxCh   = txChInfo.hTxCh;
    if (gTsnDemo.hTxCh == NULL)
    {
        EnetAppUtils_print("Failed to open TX channel\r\n");
        status = ENET_EFAIL;
    }

    /*
     * Express/probe gets its own TX channel, separate from bulk's CH0, so a
     * saturating bulk stream cannot queue ahead of express in a shared ring
     * before IET/EST ever get a chance to act - same reasoning as
     * ENET_DMA_TX_CH_PTP being kept off CH0 in TsnDemoEnet_lldCfgUpdateCb.
     * The listener never transmits bulk, so it has no separate channel to
     * open - alias to the single channel it does have.
     */
    if (status == ENET_SOK)
    {
#ifdef TSNDEMO_ROLE_LISTENER
        gTsnDemo.hTxChExpress   = gTsnDemo.hTxCh;
        gTsnDemo.txChNumExpress = gTsnDemo.txChNum;
#else
        txInArgs.cbArg    = NULL;
        txInArgs.notifyCb = NULL;

        EnetApp_getTxDmaHandle(ENET_DMA_TX_CH1, &txInArgs, &txChInfo);
        gTsnDemo.txChNumExpress = txChInfo.txChNum;
        gTsnDemo.hTxChExpress   = txChInfo.hTxCh;
        if (gTsnDemo.hTxChExpress == NULL)
        {
            EnetAppUtils_print("Failed to open express TX channel\r\n");
            status = ENET_EFAIL;
        }
#endif
    }

    if (status == ENET_SOK)
    {
        TsnDemoFlow_initTxFreePktQ();
    }

    /* RX flow 0 - default: bulk + anything not classified to flow 1 */
    if (status == ENET_SOK)
    {
        worker = &gTsnDemo.rxWorker[TSNDEMO_RXW_DEFAULT];

        rxInArgs.notifyCb = TsnDemoFlow_rxIsrCb;
        rxInArgs.cbArg    = worker;

        EnetApp_getRxDmaHandle(ENET_DMA_RX_CH0, &rxInArgs, &rxChInfo);
        worker->rxChNum = rxChInfo.rxChNum;
        worker->hRxCh   = rxChInfo.hRxCh;
        worker->taskPri = TSNDEMO_RX_TASK_PRI;
        worker->name    = "tsndemo_rx0";
        if (worker->hRxCh == NULL)
        {
            EnetAppUtils_print("Failed to open RX flow 0\r\n");
            status = ENET_EFAIL;
        }
        else
        {
            EnetAppUtils_assert(rxChInfo.numValidMacAddress >= 1U);
            EnetUtils_copyMacAddr(gTsnDemo.macAddr,
                    rxChInfo.macAddr[rxChInfo.numValidMacAddress - 1U]);
            EnetAppUtils_print("Host MAC address: ");
            EnetAppUtils_printMacAddr(gTsnDemo.macAddr);
            TsnDemoFlow_initRxReadyPktQ(worker->hRxCh);
        }
    }

    /*
     * RX flow 1 - probe/echo, ALE-classified by EtherType 0x88B5, serviced
     * at a higher task priority than flow 0 so a saturating bulk stream can
     * never delay a T2 capture or an echo build.
     */
    if (status == ENET_SOK)
    {
        worker = &gTsnDemo.rxWorker[TSNDEMO_RXW_PROBE];

        rxInArgs.notifyCb = TsnDemoFlow_rxIsrCb;
        rxInArgs.cbArg    = worker;

        EnetApp_getRxDmaHandle(ENET_DMA_RX_CH1, &rxInArgs, &rxChInfo);
        worker->rxChNum = rxChInfo.rxChNum;
        worker->hRxCh   = rxChInfo.hRxCh;
        worker->taskPri = TSNDEMO_RX_PROBE_TASK_PRI;
        worker->name    = "tsndemo_rx1_probe";
        if (worker->hRxCh == NULL)
        {
            EnetAppUtils_print("Failed to open RX flow 1\r\n");
            status = ENET_EFAIL;
        }
        else
        {
            TsnDemoFlow_initRxReadyPktQ(worker->hRxCh);
        }
    }

    if (status == ENET_SOK)
    {
        status = TsnDemoFlow_setupAleClassifier();
    }
    return status;
}

void TsnDemoFlow_close(void)
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;

    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);
    EnetApp_closeRxDma(ENET_DMA_RX_CH1, gTsnDemo.hEnet, gTsnDemo.coreKey,
                       gTsnDemo.coreId, &fqPktInfoQ, &cqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);
    EnetApp_closeRxDma(ENET_DMA_RX_CH0, gTsnDemo.hEnet, gTsnDemo.coreKey,
                       gTsnDemo.coreId, &fqPktInfoQ, &cqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);
    TsnDemoFlow_reclaimTxPkts();
    EnetApp_closeTxDma(ENET_DMA_TX_CH0, gTsnDemo.hEnet, gTsnDemo.coreKey,
                       gTsnDemo.coreId, &fqPktInfoQ, &cqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

#ifndef TSNDEMO_ROLE_LISTENER
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);
    EnetApp_closeTxDma(ENET_DMA_TX_CH1, gTsnDemo.hEnet, gTsnDemo.coreKey,
                       gTsnDemo.coreId, &fqPktInfoQ, &cqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);
#endif

    EnetAppUtils_freePktInfoQ(&gTsnDemo.txFreePktInfoQ);
    EnetAppUtils_freePktInfoQ(&gTsnDemo.bulkFreePktInfoQ);
}

/*
 * RX worker: retrieve, classify, dispatch, resubmit. One instance of this
 * runs per RX flow (TSNDEMO_RXW_DEFAULT / TSNDEMO_RXW_PROBE), each its own
 * task at its own priority, both feeding the same rxFrameCb.
 *
 * Only frames delivered to the host port arrive here (ALE filters transit
 * traffic in hardware). Demo frames go to the role callback; everything
 * else (e.g. flooded non-demo traffic reaching this flow) is dropped by
 * resubmitting.
 */
static void TsnDemoFlow_rxWorkerTask(void *args)
{
    TsnDemo_RxWorker *worker = (TsnDemo_RxWorker *)args;
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxSubmitQ;
    EnetDma_Pkt *pktInfo;
    EthVlanFrame *frame;
    TsnDemo_Hdr *hdr;
    uint32_t len;
    int32_t status;

    while (worker->running)
    {
        SemaphoreP_pend(&worker->semObj, SystemP_WAIT_FOREVER);

        EnetQueue_initQ(&rxReadyQ);
        status = EnetDma_retrieveRxPktQ(worker->hRxCh, &rxReadyQ);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("%s: RX retrieve failed: %d\r\n", worker->name,
                               status);
            continue;
        }

        pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        while (pktInfo != NULL)
        {
            frame = (EthVlanFrame *)pktInfo->sgList.list[0].bufPtr;
            len   = pktInfo->sgList.list[0].segmentFilledLen;

            /*
             * EtherType 0x88B5 is the sole discriminator - see the comment
             * on TSNDEMO_ETHERTYPE. Normally the ALE classifier delivers
             * those only to TSNDEMO_RXW_PROBE, but accept them on either
             * flow so the datapath still works if the classifier IOCTL
             * failed to install (defect D14).
             *
             * The length check is NOT optional: it is what keeps the
             * TsnDemo_Hdr cast inside the received buffer.
             */
            if ((len >= (sizeof(EthVlanFrameHeader) + sizeof(TsnDemo_Hdr))) &&
                (frame->hdr.tpid == Enet_htons(TSNDEMO_VLAN_TPID)) &&
                (frame->hdr.etherType == Enet_htons(TSNDEMO_ETHERTYPE)))
            {
                hdr = (TsnDemo_Hdr *)&frame->payload[0U];

                gTsnDemo.rxDemoFrameCount++;
                if (gTsnDemo.rxFrameCb != NULL)
                {
                    gTsnDemo.rxFrameCb(pktInfo, frame, hdr);
                }
            }

            EnetQueue_initQ(&rxSubmitQ);
            EnetQueue_enq(&rxSubmitQ, &pktInfo->node);
            EnetDma_submitRxPktQ(worker->hRxCh, &rxSubmitQ);

            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        }
    }
    worker->exited = true;
    TaskP_exit();
}

static int32_t TsnDemoFlow_startWorker(TsnDemo_RxWorker *worker, uint8_t *stack,
                                       uint32_t stackSize)
{
    TaskP_Params taskParams;
    int32_t status;

    status = SemaphoreP_constructBinary(&worker->semObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    worker->running = true;
    worker->exited  = false;
    TaskP_Params_init(&taskParams);
    taskParams.priority  = worker->taskPri;
    taskParams.stack     = stack;
    taskParams.stackSize = stackSize;
    taskParams.args      = worker;
    taskParams.name      = worker->name;
    taskParams.taskMain  = &TsnDemoFlow_rxWorkerTask;
    status = TaskP_construct(&worker->taskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);
    return status;
}

static void TsnDemoFlow_stopWorker(TsnDemo_RxWorker *worker)
{
    uint32_t waitMs = 0U;

    if (worker->running)
    {
        worker->running = false;
        SemaphoreP_post(&worker->semObj);

        /* Wait for the task to actually reach TaskP_exit() rather than
         * assuming a fixed delay is enough - under load it is not. */
        while ((!worker->exited) && (waitMs < 1000U))
        {
            ClockP_usleep(1000U);
            waitMs++;
        }
        if (!worker->exited)
        {
            EnetAppUtils_print("WARNING: %s did not exit in 1 s\r\n", worker->name);
        }
        TaskP_destruct(&worker->taskObj);
        SemaphoreP_destruct(&worker->semObj);
    }
}

int32_t TsnDemoFlow_createRxTask(TsnDemo_RxFrameCb cb)
{
    int32_t status;

    gTsnDemo.rxFrameCb = cb;

    status = TsnDemoFlow_startWorker(&gTsnDemo.rxWorker[TSNDEMO_RXW_DEFAULT],
                                     gTsnDemoRxTaskStack,
                                     sizeof(gTsnDemoRxTaskStack));
    if (status == SystemP_SUCCESS)
    {
        status = TsnDemoFlow_startWorker(
            &gTsnDemo.rxWorker[TSNDEMO_RXW_PROBE],
            gTsnDemoRxProbeTaskStack, sizeof(gTsnDemoRxProbeTaskStack));
    }
    return status;
}

void TsnDemoFlow_destroyRxTask(void)
{
    /* Stop the probe worker first: it is the higher-priority one and has
     * nothing left to feed it once the default worker (which still shares
     * the RX ISR path for flow 0) is being torn down. Order does not affect
     * correctness here, only avoids a moment of one worker running alone. */
    TsnDemoFlow_stopWorker(&gTsnDemo.rxWorker[TSNDEMO_RXW_PROBE]);
    TsnDemoFlow_stopWorker(&gTsnDemo.rxWorker[TSNDEMO_RXW_DEFAULT]);
}

EnetDma_Pkt *TsnDemoFlow_allocTxFrame(EnetDma_PktQ *pool,
                                      const TsnDemo_StreamCfg *stream,
                                      const uint8_t *dstMac,
                                      EthVlanFrame **frame,
                                      TsnDemo_Hdr **hdr)
{
    EnetDma_Pkt *pktInfo;
    EthVlanFrame *txFrame;

    pktInfo = (EnetDma_Pkt *)EnetQueue_deq(pool);

    if (pktInfo != NULL)
    {
        txFrame = (EthVlanFrame *)pktInfo->sgList.list[0].bufPtr;
        memcpy(txFrame->hdr.dstMac, dstMac, ENET_MAC_ADDR_LEN);
        memcpy(txFrame->hdr.srcMac, gTsnDemo.macAddr, ENET_MAC_ADDR_LEN);
        txFrame->hdr.tpid      = Enet_htons(TSNDEMO_VLAN_TPID);
        txFrame->hdr.tci       = Enet_htons(TSNDEMO_VLAN_TCI(stream->pcp,
                                                             stream->vlanId));
        txFrame->hdr.etherType = Enet_htons((stream->kind == TSNDEMO_STREAM_BULK) ?
                                            TSNDEMO_ETHERTYPE_BULK : TSNDEMO_ETHERTYPE);

        pktInfo->sgList.list[0].segmentFilledLen =
            stream->payloadLen + sizeof(EthVlanFrameHeader);
        pktInfo->sgList.numScatterSegments = 1U;
        pktInfo->chkSumInfo = 0U;
        pktInfo->appPriv    = &gTsnDemo;
        pktInfo->tsInfo.enableHostTxTs = false;

        if (frame != NULL)
        {
            *frame = txFrame;
        }
        if (hdr != NULL)
        {
            *hdr = (TsnDemo_Hdr *)&txFrame->payload[0U];
        }
    }
    return pktInfo;
}

int32_t TsnDemoFlow_submitTxPkt(EnetDma_Pkt *pktInfo)
{
    EnetDma_PktQ submitQ;
    EthVlanFrame *frame;
    EnetDma_TxChHandle hTxCh;
    uint8_t pcp;

    /* Route to the packet's own channel by PCP - same pool the packet was
     * allocated from, so bulk stays on CH0 and express/echo stays on the
     * dedicated CH1 (aliased to CH0 on the listener). */
    frame = (EthVlanFrame *)pktInfo->sgList.list[0].bufPtr;
    pcp   = (uint8_t)((Enet_ntohs(frame->hdr.tci) >> 13U) & 0x7U);
    hTxCh = (pcp == TSNDEMO_PCP_BULK) ? gTsnDemo.hTxCh : gTsnDemo.hTxChExpress;

    EnetDma_checkPktState(&pktInfo->pktState,
                          ENET_PKTSTATE_MODULE_APP,
                          ENET_PKTSTATE_APP_WITH_FREEQ,
                          ENET_PKTSTATE_APP_WITH_DRIVER);
    EnetQueue_initQ(&submitQ);
    EnetQueue_enq(&submitQ, &pktInfo->node);
    return EnetDma_submitTxPktQ(hTxCh, &submitQ);
}

/*
 * Reclaim completed TX descriptors and return them to the pool they came
 * from. The pool is identified by reading the PCP back out of the TX buffer,
 * which the DMA does not modify: bulk PCP -> bulk pool, anything else ->
 * express pool.
 *
 * No external locking needed: EnetDma_retrieveTxPktQ()/EnetQueue_enq()/deq()
 * each disable interrupts around their own critical section internally
 * (enet_queue.c, enet_cpdma.c / enet_udma.c), so they're already safe to
 * call concurrently from any task.
 */
uint32_t TsnDemoFlow_reclaimTxPkts(void)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_PktQ exprFreeQ;
    EnetDma_Pkt *pktInfo;
    EthVlanFrame *frame;
    uint32_t count = 0U;
    uint8_t pcp;
    int32_t status;

    EnetQueue_initQ(&txFreeQ);

    status = EnetDma_retrieveTxPktQ(gTsnDemo.hTxCh, &txFreeQ);
    if ((status == ENET_SOK) && (gTsnDemo.hTxChExpress != gTsnDemo.hTxCh))
    {
        status = EnetDma_retrieveTxPktQ(gTsnDemo.hTxChExpress, &exprFreeQ);
        if (status == ENET_SOK)
        {
            EnetQueue_append(&txFreeQ, &exprFreeQ);
        }
    }

    if (status == ENET_SOK)
    {
        pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        while (pktInfo != NULL)
        {
            frame = (EthVlanFrame *)pktInfo->sgList.list[0].bufPtr;
            pcp   = (uint8_t)((Enet_ntohs(frame->hdr.tci) >> 13U) & 0x7U);

            EnetDma_checkPktState(&pktInfo->pktState,
                                  ENET_PKTSTATE_MODULE_APP,
                                  ENET_PKTSTATE_APP_WITH_DRIVER,
                                  ENET_PKTSTATE_APP_WITH_FREEQ);
            if (pcp == TSNDEMO_PCP_BULK)
            {
                EnetQueue_enq(&gTsnDemo.bulkFreePktInfoQ, &pktInfo->node);
            }
            else
            {
                EnetQueue_enq(&gTsnDemo.txFreePktInfoQ, &pktInfo->node);
            }
            count++;
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        }
    }

    return count;
}

uint64_t TsnDemoFlow_getTimeNs(void)
{
    Enet_IoctlPrms prms;
    uint64_t tsNs = 0ULL;
    int32_t status;

    ENET_IOCTL_SET_OUT_ARGS(&prms, &tsNs);
    ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
               ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP, &prms, status);
    if (status != ENET_SOK)
    {
        gTsnDemo.tsReadFailCount++;
        tsNs = 0ULL;
    }
    return tsNs;
}
