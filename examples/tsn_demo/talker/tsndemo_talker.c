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
 * \file  tsndemo_talker.c
 *
 * \brief TSN demo - talker application: express probe generator (paced by
 *        tsndemo_pacing.c), bulk load generator, echo receiver, run
 *        orchestration and UART console.
 *
 *        User guide: docs/networking/enet_lld/tsn_demo_talker.md
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <string.h>
#include "tsndemo_talker.h"

#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"
#include "ti_dpl_config.h"
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/uart/v0/hw_uart.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TSNDEMO_EXPRESS_TASK_PRI        (16U)
#define TSNDEMO_BULK_TASK_PRI           (10U)
#define TSNDEMO_TASK_STACK              (8U * 1024U)

/*! Define (e.g. via Makefile DEFINES_common) to enable the periodic CPU-load
 *  print inside TsnDemoTalker_expressTask() - off by default, adds
 *  DebugP_log() overhead to the express hot path. */
//#define TSNDEMO_ENABLE_CPU_LOAD_PRINT

/*! Define (e.g. via Makefile DEFINES_common) to create and run the separate
 *  bulk-traffic task (TsnDemoTalker_bulkTask) - off by default. */
//#define TSNDEMO_ENABLE_BULK_TASK

/*! In-flight probe table (power of two) */
#define TSNDEMO_INFLIGHT_SIZE           (64U)

/*! Every node other than the talker itself can be a listener */
#define TSNDEMO_MAX_LISTENERS           (TSNDEMO_MAX_NODES - 1U)

/*! Guard time before the express gate window to arm pacing's GENF_START -
 *  covers CONFIG_TIMER0's clock-mux-switch + TimerP_setup latency (see
 *  tsndemo_pacing.c). Ported from ether_ring's AM263x-lp value; re-tune on
 *  hardware if the first tick misses the window. */
#define TSNDEMO_LEAD_TIME_NS            (30000U)

typedef struct TsnDemo_InFlight_s
{
    uint32_t seqNum;
    uint64_t t1Ns;
    uint64_t txLocalNs;     /*!< for timeout, ClockP time base */
    bool     valid;
} TsnDemo_InFlight;

typedef enum TsnDemo_RunState_e
{
    TSNDEMO_RUN_IDLE = 0,
    TSNDEMO_RUN_WARMUP,
    TSNDEMO_RUN_MEASURING,
    TSNDEMO_RUN_DONE,
} TsnDemo_RunState;

/* ========================================================================== */
/*                            Local Variables                                 */
/* ========================================================================== */

static struct
{
    /* generator */
    SemaphoreP_Object expressTxSem;
    TaskP_Object      expressTaskObj;
    TaskP_Object      bulkTaskObj;
    volatile bool     tasksRun;
    volatile bool     expressEnabled;
    volatile bool     bulkEnabled;
    uint32_t          seqNum[TSNDEMO_MAX_LISTENERS];
    uint8_t           runId;
    uint32_t          numListeners;      /*!< gTsnDemoCfg.numNodes - 1       */

    /* in-flight probes, one table per listener */
    TsnDemo_InFlight  inflight[TSNDEMO_MAX_LISTENERS][TSNDEMO_INFLIGHT_SIZE];
    uint32_t          highestEchoSeq[TSNDEMO_MAX_LISTENERS];

    /* current run - tick-driven: one tick fans a burst out to every
     * listener, so ticksElapsed (not echoes received) gates WARMUP/DONE */
    volatile TsnDemo_RunState runState;
    uint32_t          ticksElapsed;
    TsnDemo_RunStats  cur[TSNDEMO_MAX_LISTENERS];

    /* retained per-mode, per-listener results for the comparison table */
    TsnDemo_RunStats  lastTsnOff[TSNDEMO_MAX_LISTENERS];
    TsnDemo_RunStats  lastTsnOn[TSNDEMO_MAX_LISTENERS];

    uint8_t           chainFeatureBits;   /*!< featureBits seen in echoes  */
    volatile uint32_t echoCount;
    volatile uint64_t bulkBytes;
    volatile uint32_t expressAllocFail;
    volatile uint32_t expressSubmitFail;

    /* EST gate verification probes (TsnDemoTalker_estVerify) - entirely
     * separate from the run-state machine and per-listener stats above.
     * Single-outstanding-probe capture: the command sends one probe at a
     * time and waits for its specific echo before sending the next. */
    SemaphoreP_Object estProbeEchoSem;
    uint32_t          estProbeSeq;
    volatile uint32_t estProbeEchoSeq;
    volatile uint64_t estProbeEchoT4Ns;
    volatile bool     estProbeEchoValid;
} gTalker;

/*
 * TEMPORARY DIAGNOSTIC (remove once GENF/RTI periodicity investigation is
 * resolved) - captures a timestamp once per express pacing tick, from
 * TsnDemoTalker_expressTask() (task context) rather than an ISR.
 * TsnDemoFlow_getTimeNs() pushes a CPTS timestamp event and busy-waits for
 * the CPTS interrupt to service it; calling it from inside an ISR
 * (especially the CPTS one itself) risks that wait never completing, since
 * the servicing interrupt can't run while already inside an ISR context.
 * Task context adds a small, constant wake-latency on top of the RTI
 * timer's raw period, but large or irregular deltas still clearly
 * indicate a real periodicity problem, not measurement noise.
 */
/* Bucket thresholds around the nominal 125000 ns cycle - a delta outside
 * [LOW,HIGH] means that tick fired far enough from the expected cycle
 * time to plausibly shift a probe's send instant by tens of us relative
 * to the gate window, not just add measurement noise. */
#define TSNDEMO_PACING_DIAG_LOW_NS   (75000ULL)
#define TSNDEMO_PACING_DIAG_HIGH_NS  (130000ULL)

static struct
{
    volatile bool  active;
    uint32_t       target;
    uint32_t       count;
    uint64_t       lastTs;
    uint64_t       sumDeltaNs;
    uint64_t       maxDeltaNs;
    uint64_t       minDeltaNs;
    uint32_t       countLow;    /*!< delta < TSNDEMO_PACING_DIAG_LOW_NS      */
    uint32_t       countMid;    /*!< LOW <= delta <= HIGH (expected range)   */
    uint32_t       countHigh;   /*!< delta > TSNDEMO_PACING_DIAG_HIGH_NS     */
    SemaphoreP_Object doneSem;
} gPacingDiag;

static uint8_t gExpressTaskStack[TSNDEMO_TASK_STACK]
__attribute__ ((aligned(32)));
#ifdef TSNDEMO_ENABLE_BULK_TASK
static uint8_t gBulkTaskStack[TSNDEMO_TASK_STACK]
__attribute__ ((aligned(32)));
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
 * TEMPORARY CRASH DIAGNOSTIC - remove once the GENF-pacing data abort under
 * investigation is resolved.
 *
 * Confirmed (map file showed PC/LR inside this handler's own address
 * range) that EnetAppUtils_print() -> DebugP_log() -> _DebugP_logZone()
 * (DebugP_freertos.c) silently no-ops when HwiP_inISR() is true, since the
 * abort is dispatched through the same exception path as IRQ/FIQ - not a
 * hang, a deliberate skip that swallowed the output this handler needs.
 *
 * Fix: bypass DebugP_log and write straight to the UART hardware
 * (CONFIG_UART0, CSL_UART0_U_BASE), polling the LSR TX-FIFO-empty bit
 * before each byte - no OS call, no lock, no ISR guard.
 *
 * DFAR = faulting address. This build is Thumb (-mthumb), so for a data
 * abort the actual faulting instruction is at LR-4, not LR-8 (ARM-state
 * only) - check that address against the .map file.
 */
static void TsnDemoTalker_abortPutStr(const char *s)
{
    volatile uint32_t *uartBase =
        (volatile uint32_t *)AddrTranslateP_getLocalAddr(CSL_UART0_U_BASE);

    while (*s != '\0')
    {
        while ((uartBase[UART_LSR / 4U] & UART_LSR_TX_FIFO_E_MASK) == 0U)
        {
        }
        uartBase[UART_THR / 4U] = (uint32_t)(uint8_t)*s;
        s++;
    }
}

static void TsnDemoTalker_abortPutHex32(uint32_t val)
{
    static const char hexDigits[] = "0123456789ABCDEF";
    char buf[11] = "0x00000000";
    int32_t i;

    for (i = 9; i >= 2; i--)
    {
        buf[i] = hexDigits[val & 0xFU];
        val >>= 4;
    }
    TsnDemoTalker_abortPutStr(buf);
}

void HwiP_user_data_abort_handler_c(DFSR dfsr, ADFSR adfsr, volatile uint32_t DFAR,
                                    volatile uint32_t LR, volatile uint32_t SPSR)
{
    TsnDemoTalker_abortPutStr("\r\n*** DATA ABORT *** DFAR=");
    TsnDemoTalker_abortPutHex32((uint32_t)DFAR);
    TsnDemoTalker_abortPutStr(" LR=");
    TsnDemoTalker_abortPutHex32((uint32_t)LR);
    TsnDemoTalker_abortPutStr(" (Thumb build, fault~=LR-4)\r\n SPSR=");
    TsnDemoTalker_abortPutHex32((uint32_t)SPSR);
    TsnDemoTalker_abortPutStr(" DFSR.status=");
    TsnDemoTalker_abortPutHex32((uint32_t)dfsr.status);
    TsnDemoTalker_abortPutStr(" rw=");
    TsnDemoTalker_abortPutHex32((uint32_t)dfsr.rw);
    TsnDemoTalker_abortPutStr(" sd=");
    TsnDemoTalker_abortPutHex32((uint32_t)dfsr.sd);
    TsnDemoTalker_abortPutStr("\r\n");

    {
        volatile uint32_t loop = 1U;
        while (loop != 0U) { ; }
    }
}

static TsnDemo_StreamCfg *TsnDemoTalker_firstStream(TsnDemo_StreamKind kind)
{
    TsnDemo_StreamCfg *stream = NULL;
    uint32_t i;

    for (i = 0U; (i < TSNDEMO_MAX_STREAMS) && (stream == NULL); i++)
    {
        if (gTsnDemoCfg.stream[i].kind == kind)
        {
            stream = &gTsnDemoCfg.stream[i];
        }
    }
    return stream;
}

/* ------------------------------ echo receive ----------------------------- */

static void TsnDemoTalker_rxFrameCb(EnetDma_Pkt *pktInfo,
                                    EthVlanFrame *frame, TsnDemo_Hdr *hdr)
{
    uint64_t t4Ns = pktInfo->tsInfo.rxPktTs;   /* hardware ingress timestamp */
    TsnDemo_InFlight *slot;
    uint32_t nodeIdx;

    if (hdr->streamId == TSNDEMO_STREAM_ID_ESTPROBE)
    {
        /* EST verification probe echo - diverted entirely, never touches
         * the normal per-listener run stats below. */
        gTalker.estProbeEchoSeq   = hdr->seqNum;
        gTalker.estProbeEchoT4Ns  = t4Ns;
        gTalker.estProbeEchoValid = true;
        SemaphoreP_post(&gTalker.estProbeEchoSem);
    }
    else
    {
        gTalker.echoCount++;
        gTalker.chainFeatureBits = hdr->featureBits;

        /* dstNodeId (1..numListeners) identifies which listener echoed this -
         * see TsnDemoTalker_expressTask, which stamps it per send. Underflows
         * harmlessly to a large index (rejected by the bound check) if 0. */
        nodeIdx = (uint32_t)hdr->dstNodeId - 1U;
        if (nodeIdx < gTalker.numListeners)
        {
            if (hdr->runId != gTalker.runId)
            {
                gTalker.cur[nodeIdx].staleCount++;
            }
            else
            {
                slot = &gTalker.inflight[nodeIdx][hdr->seqNum & (TSNDEMO_INFLIGHT_SIZE - 1U)];
                if ((slot->valid) && (slot->seqNum == hdr->seqNum))
                {
                    slot->valid = false;
                }

                if (gTalker.runState == TSNDEMO_RUN_MEASURING)
                {
                    /* Chain state check: a run is only valid if every echo in
                     * the window carries the talker's own feature state. */
                    if (hdr->featureBits != TsnDemoFeature_toBits(&gTsnDemoCfg.features))
                    {
                        gTalker.cur[nodeIdx].valid = false;
                    }

                    if (hdr->seqNum < gTalker.highestEchoSeq[nodeIdx])
                    {
                        gTalker.cur[nodeIdx].outOfOrderCount++;
                    }
                    else
                    {
                        gTalker.highestEchoSeq[nodeIdx] = hdr->seqNum;
                    }

                    gTalker.cur[nodeIdx].rxCount++;
                    if (hdr->t1Ns == 0ULL)
                    {
                        gTalker.cur[nodeIdx].tsFailCount++;
                    }
                    TsnDemoStats_addSample(&gTalker.cur[nodeIdx].fwd, hdr->t1Ns, hdr->t2Ns);
                    TsnDemoStats_addSample(&gTalker.cur[nodeIdx].rev, hdr->t3Ns, t4Ns);
                    TsnDemoStats_addSample(&gTalker.cur[nodeIdx].rtt, hdr->t1Ns, t4Ns);
                    TsnDemoStats_addSample(&gTalker.cur[nodeIdx].turn, hdr->t2Ns, hdr->t3Ns);
                }
                /* WARMUP/IDLE/DONE: run-state transitions are tick-driven now,
                 * not echo-driven - see TsnDemoTalker_expressTask. */
            }
        }
    }
}

/* ---------------------------- express generator --------------------------- */
#ifdef TSNDEMO_ENABLE_CPU_LOAD_PRINT
void EnetApp_printCpuLoad(void)
{
    static uint32_t startTime_ms = 0;
    const  uint32_t currTime_ms  = ClockP_getTimeUsec()/1000;
    const  uint32_t printInterval_ms = 5000;

    if (startTime_ms == 0)
    {
        startTime_ms = currTime_ms;
    }
    else if ((currTime_ms - startTime_ms) > printInterval_ms)
    {
        const uint32_t cpuLoad = TaskP_loadGetTotalCpuLoad();

        DebugP_log(" %6d.%3ds : CPU load = %3d.%02d %%\r\n",
                  currTime_ms/1000, currTime_ms%1000,
                  cpuLoad/100, cpuLoad%100 );

        startTime_ms = currTime_ms;
        TaskP_loadResetAll();
    }
    return;
}
#endif


static void TsnDemoTalker_pacingTick(void)
{
    SemaphoreP_post(&gTalker.expressTxSem);
}

static void TsnDemoTalker_expressTask(void *args)
{
    TsnDemo_StreamCfg *stream;
    TsnDemo_StreamCfg destStream;
    EthVlanFrame *frame;
    TsnDemo_Hdr *hdr;
    EnetDma_Pkt *pktInfo;
    TsnDemo_InFlight *slot;
    uint32_t seq;
    uint32_t nodeIdx;
    uint64_t nowNs;
#ifdef TSNDEMO_ENABLE_CPU_LOAD_PRINT
    static uint32_t tempCount = 0U;
#endif

    while (gTalker.tasksRun)
    {
        SemaphoreP_pend(&gTalker.expressTxSem, SystemP_WAIT_FOREVER);

        /* TEMPORARY DIAGNOSTIC - see gPacingDiag's declaration. Runs on
         * every tick regardless of expressEnabled, since pacing ticks
         * fire whenever TsnDemoPacing_start() is active, not just during
         * a measured run. */
        if (gPacingDiag.active)
        {
            uint64_t diagNowNs = TsnDemoFlow_getTimeNs();

            if (gPacingDiag.lastTs != 0ULL)
            {
                uint64_t deltaNs = diagNowNs - gPacingDiag.lastTs;

                gPacingDiag.sumDeltaNs += deltaNs;
                if (deltaNs > gPacingDiag.maxDeltaNs)
                {
                    gPacingDiag.maxDeltaNs = deltaNs;
                }
                if (deltaNs < gPacingDiag.minDeltaNs)
                {
                    gPacingDiag.minDeltaNs = deltaNs;
                }
                if (deltaNs < TSNDEMO_PACING_DIAG_LOW_NS)
                {
                    gPacingDiag.countLow++;
                }
                else if (deltaNs > TSNDEMO_PACING_DIAG_HIGH_NS)
                {
                    gPacingDiag.countHigh++;
                }
                else
                {
                    gPacingDiag.countMid++;
                }
                gPacingDiag.count++;
            }
            gPacingDiag.lastTs = diagNowNs;
            if (gPacingDiag.count >= gPacingDiag.target)
            {
                gPacingDiag.active = false;
                SemaphoreP_post(&gPacingDiag.doneSem);
            }
        }

#ifdef TSNDEMO_ENABLE_CPU_LOAD_PRINT
        if (tempCount >= 8000U)
        {
            EnetApp_printCpuLoad();
            tempCount = 0U;
        }
        else
        {
            tempCount++;
        }
#endif
        if (!gTalker.expressEnabled)
        {
            continue;
        }
        stream = TsnDemoTalker_firstStream(TSNDEMO_STREAM_EXPRESS);
        if (stream == NULL)
        {
            continue;
        }

        /*
         * Run-state transitions are tick-driven, not echo-driven: one tick
         * fans a burst out to every listener, so counting individual
         * echoes (as before) can't cleanly signal "warmup/run done" once
         * there's more than one listener.
         */
        if (gTalker.runState == TSNDEMO_RUN_WARMUP)
        {
            gTalker.ticksElapsed++;
            if (gTalker.ticksElapsed >= gTsnDemoCfg.warmupSampleCount)
            {
                nowNs = TsnDemoFlow_getTimeNs();
                for (nodeIdx = 0U; nodeIdx < gTalker.numListeners; nodeIdx++)
                {
                    gTalker.cur[nodeIdx].windowStartNs = nowNs;
                }
                gTalker.ticksElapsed = 0U;
                gTalker.runState = TSNDEMO_RUN_MEASURING;
            }
        }
        else if (gTalker.runState == TSNDEMO_RUN_MEASURING)
        {
            gTalker.ticksElapsed++;
        }

        TsnDemoFlow_reclaimTxPkts();

        for (nodeIdx = 0U; nodeIdx < gTalker.numListeners; nodeIdx++)
        {
            const uint8_t dstMac[ENET_MAC_ADDR_LEN] =
                TSNDEMO_NODE_MCAST(nodeIdx + 1U);

            destStream           = *stream;
            destStream.dstNodeId = (uint8_t)(nodeIdx + 1U);

            pktInfo = TsnDemoFlow_allocTxFrame(&gTsnDemo.txFreePktInfoQ,
                                               &destStream, dstMac, &frame, &hdr);
            if (pktInfo == NULL)
            {
                /* pool exhausted; count as lost via timeout. Throttled so
                 * this can't flood the console at the express tick rate. */
                gTalker.expressAllocFail++;
                if ((gTalker.expressAllocFail == 1U) ||
                    ((gTalker.expressAllocFail % 1000U) == 0U))
                {
                    EnetAppUtils_print(
                        "express: TX pool exhausted (count=%u)\r\n",
                        gTalker.expressAllocFail);
                }
            }
            else
            {
                seq = gTalker.seqNum[nodeIdx]++;
                /* Payload beyond the header was prefilled once at init;
                 * only the header fields change per frame. */
                hdr->seqNum      = seq;
                hdr->streamId    = 0U;
                hdr->dstNodeId   = destStream.dstNodeId;
                hdr->runId       = gTalker.runId;
                hdr->runMode     = (uint8_t)gTsnDemoCfg.mode;
                hdr->featureBits = TsnDemoFeature_toBits(&gTsnDemoCfg.features);
                hdr->t2Ns        = 0ULL;
                hdr->t3Ns        = 0ULL;

                slot = &gTalker.inflight[nodeIdx][seq & (TSNDEMO_INFLIGHT_SIZE - 1U)];
                if ((slot->valid) && (gTalker.runState == TSNDEMO_RUN_MEASURING))
                {
                    /* overwritten before echoed: too old, count lost */
                    gTalker.cur[nodeIdx].lostCount++;
                }
                slot->seqNum    = seq;
                slot->valid     = true;
                slot->txLocalNs = ClockP_getTimeUsec() * 1000ULL;

                /* T1 last, immediately before submit */
                hdr->t1Ns  = TsnDemoFlow_getTimeNs();
                slot->t1Ns = hdr->t1Ns;

                if (TsnDemoFlow_submitTxPkt(pktInfo) != ENET_SOK)
                {
                    gTalker.expressSubmitFail++;
                    if ((gTalker.expressSubmitFail == 1U) ||
                        ((gTalker.expressSubmitFail % 1000U) == 0U))
                    {
                        EnetAppUtils_print(
                            "express: TX submit failed (count=%u)\r\n",
                            gTalker.expressSubmitFail);
                    }
                }
                else if (gTalker.runState == TSNDEMO_RUN_MEASURING)
                {
                    gTalker.cur[nodeIdx].txCount++;
                }
            }
        }

        if ((gTalker.runState == TSNDEMO_RUN_MEASURING) &&
            (gTalker.ticksElapsed >= gTsnDemoCfg.runSampleCount))
        {
            nowNs = TsnDemoFlow_getTimeNs();
            for (nodeIdx = 0U; nodeIdx < gTalker.numListeners; nodeIdx++)
            {
                gTalker.cur[nodeIdx].windowEndNs = nowNs;
                gTalker.cur[nodeIdx].bulkTxBytes = gTalker.bulkBytes;
            }
            gTalker.runState = TSNDEMO_RUN_DONE;
        }
    }
    TaskP_exit();
}

/* ------------------------------ bulk generator ---------------------------- */

static void TsnDemoTalker_bulkTask(void *args)
{
    TsnDemo_StreamCfg *stream;
    /* Unregistered multicast: floods MAC-port-to-MAC-port through the whole
     * chain in hardware, contending for egress bandwidth at every hop
     * without ever reaching a host port / CPU. */
    const uint8_t dstMac[ENET_MAC_ADDR_LEN] = TSNDEMO_BULK_MCAST;
    EthVlanFrame *frame;
    TsnDemo_Hdr *hdr;
    EnetDma_Pkt *pktInfo;

    while (gTalker.tasksRun)
    {
        if (!gTalker.bulkEnabled)
        {
            ClockP_usleep(10000U);
            continue;
        }
        stream = TsnDemoTalker_firstStream(TSNDEMO_STREAM_BULK);
        if (stream == NULL)
        {
            ClockP_usleep(10000U);
            continue;
        }

        TsnDemoFlow_reclaimTxPkts();

        pktInfo = TsnDemoFlow_allocTxFrame(&gTsnDemo.bulkFreePktInfoQ,
                                           stream, dstMac, &frame, &hdr);
        if (pktInfo == NULL)
        {
            /* pool empty: descriptors in flight; yield and retry */
            TaskP_yield();
            continue;
        }
        /*
         * Bulk payload was prefilled once at init and is never touched
         * again - no per-frame memset, no timestamps.
         * Nothing in the header needs to be set either: bulk is
         * identified purely by its EtherType (0x88B6, set in
         * TsnDemoFlow_allocTxFrame from stream->kind) and is never
         * parsed by any RX path - it only ever needs to be counted in
         * bytes for the offered-load figure below.
         */
        (void)hdr;

        (void)TsnDemoFlow_submitTxPkt(pktInfo);
        gTalker.bulkBytes += stream->payloadLen +
                             sizeof(EthVlanFrameHeader) + 24U;
    }
    TaskP_exit();
}

/* ------------------------------ run control ------------------------------- */

static void TsnDemoTalker_scanTimeouts(void)
{
    uint64_t nowNs = ClockP_getTimeUsec() * 1000ULL;
    uint64_t timeoutNs = (uint64_t)gTsnDemoCfg.echoTimeoutUs * 1000ULL;
    TsnDemo_InFlight *slot;
    uint32_t nodeIdx;
    uint32_t i;

    if (gTalker.runState == TSNDEMO_RUN_MEASURING)
    {
        for (nodeIdx = 0U; nodeIdx < gTalker.numListeners; nodeIdx++)
        {
            for (i = 0U; i < TSNDEMO_INFLIGHT_SIZE; i++)
            {
                slot = &gTalker.inflight[nodeIdx][i];

                if ((slot->valid) && ((nowNs - slot->txLocalNs) > timeoutNs))
                {
                    slot->valid = false;
                    gTalker.cur[nodeIdx].lostCount++;
                }
            }
        }
    }
}

/* Forward declarations: defined below (near TsnDemoTalker_estVerify, which
 * has the same need), but pacing needs them too - see
 * TsnDemoTalker_getLivePacingSchedule(). */
static bool TsnDemoTalker_findLinkedPort(Enet_MacPort *macPort);
static bool TsnDemoTalker_getLivePacingSchedule(uint64_t *baseNs,
                                                uint64_t *cycleNs);

static void TsnDemoTalker_runMode(TsnDemo_RunMode mode)
{
    TsnDemo_FeatureSet want = { false, false, false };
    bool runOk = true;
    uint32_t nodeIdx;

    if (mode == TSNDEMO_MODE_CONTENTION_TSN)
    {
        if ((!gTsnDemoCfg.features.estEn) && (!gTsnDemoCfg.features.ietEn) &&
            (!gTsnDemoCfg.features.cutThruEn))
        {
            /* Nothing selected yet: default to the full stack (EST + IET +
             * cut-through where supported), matching the original one-key
             * quickstart behaviour. */
            want.estEn     = true;
#if ENET_CFG_IS_ON(CPSW_IET_INCL)
            want.ietEn     = true;
#endif
#if ENET_CFG_IS_ON(CPSW_CUTTHRU)
            want.cutThruEn = true;
#endif
        }
        else
        {
            /* Operator already chose a specific combination via 'e'/'i'/'u'
             * before pressing '3' - respect it exactly. This is what makes
             * the incremental EST-only / EST+cut-through / full-stack
             * comparison runs possible (bring-up guide, Stage I). */
            want = gTsnDemoCfg.features;
        }
    }

    /* Feature change needed? Remind the operator about listener ordering. */
    if (memcmp(&want, &gTsnDemoCfg.features, sizeof(want)) != 0)
    {
        if ((want.ietEn) && (!gTsnDemoCfg.features.ietEn))
        {
            EnetAppUtils_print(
                "NOTE: enable IET on all LISTENERS first (verification is a\r\n"
                "      link-partner handshake), then continue here.\r\n");
        }
        if (TsnDemoFeature_applyAll(&want) != ENET_SOK)
        {
            EnetAppUtils_print("Feature apply failed - run aborted\r\n");
            runOk = false;
        }
    }

    if (runOk)
    {
        uint64_t liveBaseNs, liveCycleNs;

        gTsnDemoCfg.mode = mode;

        /* Pace against the REAL hardware schedule, not the nominal
         * TSNDEMO_EST_CYCLE_NS/gTsnDemo.estBaseTimeNs requested - confirmed
         * on hardware ('v') that the actual gate cycle runs measurably
         * longer than requested, by a different amount per port, so
         * locking to the nominal values would drift out of phase with the
         * real gate boundary over a run. Returns false (liveBaseNs left 0)
         * if EST is off or the readback fails; TsnDemoPacing_start() then
         * takes the plain-timer path, and the !isLocked() check below
         * warns if EST was wanted but locking didn't happen. */
        (void)TsnDemoTalker_getLivePacingSchedule(&liveBaseNs, &liveCycleNs);

        /* Restart pacing against the (possibly new) live EST schedule */
        TsnDemoPacing_stop();
        gTalker.expressEnabled = false;
        gTalker.bulkEnabled    = false;
        if (TsnDemoPacing_start(liveBaseNs, (uint32_t)liveCycleNs,
                                0U /* express window starts the cycle */,
                                TSNDEMO_LEAD_TIME_NS,
                                TsnDemoTalker_pacingTick) != ENET_SOK)
        {
            EnetAppUtils_print("Pacing start failed - run aborted\r\n");
            runOk = false;
        }
        else if (want.estEn)
        {
            /*
             * TsnDemoPacing_start() returns once GENF1 is ARMED, not once
             * it's LOCKED - TsnDemoPacing_startGenf() deliberately arms
             * GENF1's hw-push edge at (now + 2s) rather than immediately,
             * since an immediate arm could already be behind the CPTS clock
             * by the time it executes on re-runs. isLocked() only flips
             * true once that edge fires and starts RTI1, so checking it
             * right away would be a guaranteed false alarm, not a real
             * signal - wait (bounded) for the edge before deciding whether
             * to warn.
             */
            uint32_t lockWaitMs = 0U;

            while ((!TsnDemoPacing_isLocked()) && (lockWaitMs < 3000U))
            {
                ClockP_usleep(50000U);
                lockWaitMs += 50U;
            }
        }

        if ((runOk) && (want.estEn) && (!TsnDemoPacing_isLocked()))
        {
            /*
             * Warn, don't abort. GENF gate-locking has never been
             * hardware-verified yet (blocked on link-up until now) -
             * aborting here would mean a single failed lock attempt gives
             * zero data instead of a still-useful, just noisier,
             * statistical comparison (unlocked pacing samples effectively
             * random phase against the gate, which over runSampleCount
             * samples still shows a real effect if EST is gating - it's
             * only the small-N 'v' test that strictly needs the lock).
             */
            EnetAppUtils_print(
                "WARNING: EST is on but pacing is NOT gate-locked - this\r\n"
                "         run's numbers are noisier (unlocked, random\r\n");
            EnetAppUtils_print(
                "         phase vs. the gate) but still meaningful with\r\n"
                "         enough samples. Not aborting.\r\n");
        }
    }

    if (runOk)
    {
        /* Arm the run. Print BEFORE enabling the generators, not after: once
         * expressEnabled/bulkEnabled go true the express/bulk tasks can start
         * transmitting on their very next semaphore wakeup, so printing first
         * guarantees the UART write (vsnprintf + console I/O, not free)
         * cannot overlap the start of the run. warmupSampleCount normally
         * absorbs this either way, but there is no reason to leave the race
         * in. */
        memset(gTalker.inflight, 0, sizeof(gTalker.inflight));
        gTalker.runId++;
        gTalker.ticksElapsed = 0U;
        gTalker.bulkBytes    = 0ULL;
        for (nodeIdx = 0U; nodeIdx < gTalker.numListeners; nodeIdx++)
        {
            gTalker.highestEchoSeq[nodeIdx] = 0U;
            TsnDemoStats_reset(&gTalker.cur[nodeIdx], mode, &gTsnDemoCfg.features);
        }
        gTalker.runState = TSNDEMO_RUN_WARMUP;

        EnetAppUtils_print("Run started: mode %d, %u listener(s), %u samples "
                           "(+%u warmup) each...\r\n",
                           (int)mode, gTalker.numListeners,
                           gTsnDemoCfg.runSampleCount,
                           gTsnDemoCfg.warmupSampleCount);

        gTalker.expressEnabled = true;
        gTalker.bulkEnabled    = (mode != TSNDEMO_MODE_BASELINE);

        /* Wait for completion, scanning echo timeouts */
        while (gTalker.runState != TSNDEMO_RUN_DONE)
        {
            ClockP_usleep(100000U);
            TsnDemoTalker_scanTimeouts();
        }
        gTalker.expressEnabled = false;
        gTalker.bulkEnabled    = false;

        for (nodeIdx = 0U; nodeIdx < gTalker.numListeners; nodeIdx++)
        {
            if (gTalker.cur[nodeIdx].tsFailCount != 0U)
            {
                EnetAppUtils_print("WARNING: node %u: %u probes had failed "
                                   "T1 reads\r\n", nodeIdx + 1U,
                                   gTalker.cur[nodeIdx].tsFailCount);
            }
            EnetAppUtils_print(
                "--- Listener node %u ---------------------------------------\r\n",
                nodeIdx + 1U);
            TsnDemoStats_print(&gTalker.cur[nodeIdx]);

            /* Retain for comparison */
            if (mode == TSNDEMO_MODE_CONTENTION_NOTSN)
            {
                gTalker.lastTsnOff[nodeIdx] = gTalker.cur[nodeIdx];
            }
            else if (mode == TSNDEMO_MODE_CONTENTION_TSN)
            {
                gTalker.lastTsnOn[nodeIdx] = gTalker.cur[nodeIdx];
            }
        }
        gTalker.runState = TSNDEMO_RUN_IDLE;
    }
}

/* -------------------------------- console -------------------------------- */

static void TsnDemoTalker_printMenu(void)
{
    EnetAppUtils_print("\r\n--- TSN Demo Talker ---------------------------------------\r\n");
    EnetAppUtils_print(" '1'  -  Run BASELINE           (express only, features off)\r\n");
    EnetAppUtils_print(" '2'  -  Run CONTENTION-TSN-OFF (express + bulk, features off)\r\n");
    EnetAppUtils_print(" '3'  -  Run CONTENTION-TSN-ON  (express + bulk; uses whatever\r\n");
    EnetAppUtils_print("                                 e/i/u already selected, or the\r\n");
    EnetAppUtils_print("                                 full stack if nothing selected)\r\n");
    EnetAppUtils_print(" 'e'  -  Toggle EST\r\n");
#if TSNDEMO_IET_SUPPORTED
    EnetAppUtils_print(" 'i'  -  Toggle IET\r\n");
#endif
#if TSNDEMO_CUTTHRU_SUPPORTED
    EnetAppUtils_print(" 'u'  -  Toggle cut-through\r\n");
#endif
    EnetAppUtils_print(" 'g'  -  Show EST schedule\r\n");
    EnetAppUtils_print(" 'p'  -  Print last report     'c'  -  Print A/B comparison\r\n");
    EnetAppUtils_print(" 'd'  -  Status\r\n");
#if defined(TSNDEMO_ENABLE_DEBUG_MENU)
    EnetAppUtils_print(" --- bring-up / capture (no measurement) ---\r\n");
    EnetAppUtils_print(" 's'  -  CPSW statistics\r\n");
    EnetAppUtils_print(" 'T'  -  Toggle free-run EXPRESS transmit\r\n");
    EnetAppUtils_print(" 'B'  -  Toggle free-run BULK transmit\r\n");
    EnetAppUtils_print(" 'v'  -  Verify EST is actually gating (needs EST on)\r\n");
    EnetAppUtils_print(" 'y'  -  Measure pacing tick periodicity (temp diag)\r\n");
#endif
    EnetAppUtils_print(" 'x'  -  Stop\r\n");
}

/*
 * Free-run transmit: generate the stream continuously with no measurement
 * window and no echo requirement, so the frames can be captured on a PC with
 * Wireshark and the on-wire format verified before a second EVM exists.
 * See the bring-up guide, stage B2/B3.
 */
static void TsnDemoTalker_toggleFreeRun(bool express)
{
    if (express)
    {
        bool pacingOk = true;

        if (!gTalker.expressEnabled)
        {
            uint64_t liveBaseNs, liveCycleNs;

            /* Live schedule, not nominal - same reasoning as
             * TsnDemoTalker_runMode() above. */
            (void)TsnDemoTalker_getLivePacingSchedule(&liveBaseNs,
                                                      &liveCycleNs);

            /* pacing must be running to release the express task */
            TsnDemoPacing_stop();
            if (TsnDemoPacing_start(liveBaseNs, (uint32_t)liveCycleNs, 0U,
                                    TSNDEMO_LEAD_TIME_NS,
                                    TsnDemoTalker_pacingTick) != ENET_SOK)
            {
                EnetAppUtils_print("free-run: pacing start failed\r\n");
                pacingOk = false;
            }
            else
            {
                gTalker.runState = TSNDEMO_RUN_IDLE;
            }
        }

        if (pacingOk)
        {
            gTalker.expressEnabled = !gTalker.expressEnabled;
            EnetAppUtils_print("free-run EXPRESS: %s (%s)\r\n",
                               gTalker.expressEnabled ? "ON" : "off",
                               TsnDemoPacing_isLocked() ? "GENF gate-locked" :
                                                          "plain timer, NOT gate-locked");
        }
    }
    else
    {
        gTalker.bulkEnabled = !gTalker.bulkEnabled;
        EnetAppUtils_print("free-run BULK: %s\r\n",
                           gTalker.bulkEnabled ? "ON" : "off");
    }
}

static void TsnDemoTalker_toggleFeature(char which)
{
    TsnDemo_FeatureSet want = gTsnDemoCfg.features;
    bool recognized = true;

    switch (which)
    {
        case 'e': want.estEn     = !want.estEn;     break;
        case 'i': want.ietEn     = !want.ietEn;     break;
        case 'u': want.cutThruEn = !want.cutThruEn; break;
        default: recognized = false; break;
    }

    if (recognized)
    {
        (void)TsnDemoFeature_applyAll(&want);
    }
}

/* ------------------------- EST gate verification -------------------------- */

/*
 * Sends probes tagged with the BULK PCP (excluded from EST windows 0/2,
 * see TsnDemoFeature_buildAdminList), timed via gPTP to land mid-window0
 * (gate CLOSED for this PCP) vs. mid-window1 (gate OPEN), and compares RTT.
 * If EST is really gating, mid-window0 RTT should show a large, roughly
 * cycle-bounded increase over mid-window1; if not, the two groups should
 * look the same. EtherType stays TSNDEMO_STREAM_EXPRESS so the listener
 * still echoes it and the existing T1/T4 RTT machinery applies - only the
 * PCP (and hence the egress gate) differs from a real express probe.
 */
#define TSNDEMO_ESTVERIFY_SAMPLES          (5U)
#define TSNDEMO_ESTVERIFY_ECHO_TIMEOUT_US  (5000U)
#define TSNDEMO_ESTVERIFY_LEAD_CYCLES       (3U)

/*
 * Computes a target instant a few cycles ahead of the CURRENT clock, not
 * relative to an earlier reading - must be called fresh right before each
 * send. Blocking console prints between samples (a few ms, dozens of EST
 * cycles) would otherwise eat into the next sample's margin and leave a
 * pre-computed target stale (see the bug this replaced).
 *
 * baseNs/cycleNs MUST come from a live TsnDemoFeature_getLiveOperList()
 * readback, not the nominal TSNDEMO_EST_CYCLE_NS/gTsnDemo.estBaseTimeNs -
 * confirmed on hardware that the real cycle runs measurably longer than
 * requested (tick quantization), by a different amount per port. The
 * drift compounds every cycle: at 600ns/cycle, just 10-60s between 'e' and
 * 'v' (80k-480k cycles) is 48-288ms of accumulated phase error against a
 * 125us cycle - i.e. total decorrelation if computed from nominal time.
 */
static uint64_t TsnDemoTalker_nextWindowInstant(uint64_t baseNs,
                                                uint64_t cycleNs,
                                                uint64_t windowOffsetNs)
{
    uint64_t nowNs = TsnDemoFlow_getTimeNs();
    uint64_t k;

    k = (nowNs > baseNs) ?
        ((nowNs - baseNs) / cycleNs) + TSNDEMO_ESTVERIFY_LEAD_CYCLES :
        TSNDEMO_ESTVERIFY_LEAD_CYCLES;
    return baseNs + (k * cycleNs) + windowOffsetNs;
}

/* Find the MAC port actually carrying traffic to the listener - EST's
 * live oper list must be read from THIS port specifically, since the two
 * ports on this board have been confirmed (on hardware) to quantize their
 * cycle lengths differently. */
static bool TsnDemoTalker_findLinkedPort(Enet_MacPort *macPort)
{
    Enet_IoctlPrms prms;
    Enet_MacPort port;
    bool linked;
    int32_t status;
    uint32_t i;
    bool found = false;

    for (i = 0U; (i < gTsnDemo.numMacPorts) && (!found); i++)
    {
        port   = gTsnDemo.macPorts[i];
        linked = false;
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &port, &linked);
        ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                   ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
        if ((status == ENET_SOK) && (linked))
        {
            *macPort = port;
            found = true;
        }
    }
    return found;
}

/*
 * Reads the REAL hardware EST schedule (same readback as
 * TsnDemoTalker_estVerify()) for pacing (tsndemo_pacing.c) to lock against -
 * the nominal TSNDEMO_EST_CYCLE_NS/gTsnDemo.estBaseTimeNs are what was
 * *requested*, not what's actually running: tick quantization makes the
 * real cycle measurably longer, by a different amount per port, and
 * locking to the nominal values would silently drift out of phase every
 * cycle.
 *
 * Returns false (*baseNs left 0, *cycleNs at the nominal
 * TSNDEMO_EST_CYCLE_NS) if EST is off or the live readback fails for any
 * reason - callers pass that straight through to TsnDemoPacing_start(),
 * which takes its plain-timer path on baseNs == 0, exactly as if EST were
 * off.
 */
static bool TsnDemoTalker_getLivePacingSchedule(uint64_t *baseNs,
                                                uint64_t *cycleNs)
{
    Enet_MacPort linkedPort;
    EnetTas_ControlList liveList;
    uint32_t i;
    bool ok;

    *baseNs  = 0ULL;
    *cycleNs = TSNDEMO_EST_CYCLE_NS;

    if ((!gTsnDemoCfg.features.estEn) || (gTsnDemo.estBaseTimeNs == 0ULL))
    {
        ok = false;
    }
    else if (!TsnDemoTalker_findLinkedPort(&linkedPort))
    {
        ok = false;
    }
    else if ((!TsnDemoFeature_getLiveOperList(linkedPort, &liveList)) ||
             (liveList.listLength < 1U))
    {
        ok = false;
    }
    else
    {
        *baseNs  = liveList.baseTime;
        *cycleNs = 0ULL;
        for (i = 0U; i < liveList.listLength; i++)
        {
            *cycleNs += liveList.gateCmdList[i].timeInterval;
        }
        ok = true;
    }
    return ok;
}

static bool TsnDemoTalker_estVerifySendOne(uint64_t targetNs, uint32_t seq,
                                           const char *label, uint64_t *rttNs)
{
    TsnDemo_StreamCfg probeStream =
    {
        .kind       = TSNDEMO_STREAM_EXPRESS,  /* EtherType 0x88B5 - echoed */
        .payloadLen = TSNDEMO_MIN_PAYLOAD_LEN,
        .pcp        = TSNDEMO_PCP_BULK,        /* gated like bulk, not express */
        .vlanId     = TSNDEMO_VLAN_ID,
        .dstNodeId  = 1U,
    };
    const uint8_t dstMac[ENET_MAC_ADDR_LEN] = TSNDEMO_NODE_MCAST(1U);
    EnetDma_Pkt *pktInfo;
    EthVlanFrame *frame;
    TsnDemo_Hdr *hdr;
    int32_t waitStatus;
    int32_t pendStatus;
    uint64_t t1Ns;
    bool ok = false;

    waitStatus = TsnDemoEnet_waitUntilTs64(targetNs);
    if (waitStatus == 1)
    {
        EnetAppUtils_print(
            "  [%s seq %u] WARNING: missed own timing target (scheduling "
            "delay) - this sample may not land where intended\r\n",
            label, seq);
    }

    TsnDemoFlow_reclaimTxPkts();
    /* Allocate from the BULK pool, not the express pool: TsnDemoFlow_
     * reclaimTxPkts() returns completed packets to a pool chosen by
     * reading the PCP back out of the buffer (bulk PCP -> bulk pool,
     * tsndemo_dataflow.c), so a bulk-PCP packet allocated from the express
     * pool would get reclaimed into the wrong pool and permanently drain
     * the (much smaller, 8-packet) express pool. */
    pktInfo = TsnDemoFlow_allocTxFrame(&gTsnDemo.bulkFreePktInfoQ, &probeStream,
                                       dstMac, &frame, &hdr);
    if (pktInfo == NULL)
    {
        EnetAppUtils_print("  [%s seq %u] alloc failed - pool exhausted\r\n",
                           label, seq);
    }
    else
    {
        gTalker.estProbeEchoValid = false;
        t1Ns             = TsnDemoFlow_getTimeNs();
        hdr->seqNum      = seq;
        hdr->streamId    = TSNDEMO_STREAM_ID_ESTPROBE;
        hdr->dstNodeId   = 1U;
        hdr->runId       = gTalker.runId;
        hdr->runMode     = (uint8_t)gTsnDemoCfg.mode;
        hdr->featureBits = TsnDemoFeature_toBits(&gTsnDemoCfg.features);
        hdr->t1Ns        = t1Ns;
        hdr->t2Ns        = 0ULL;
        hdr->t3Ns        = 0ULL;

        if (TsnDemoFlow_submitTxPkt(pktInfo) != ENET_SOK)
        {
            EnetAppUtils_print("  [%s seq %u] submit failed\r\n", label, seq);
        }
        else
        {
            pendStatus = SemaphoreP_pend(
                &gTalker.estProbeEchoSem,
                ClockP_usecToTicks(TSNDEMO_ESTVERIFY_ECHO_TIMEOUT_US));
            if ((pendStatus == SystemP_SUCCESS) && (gTalker.estProbeEchoValid) &&
                (gTalker.estProbeEchoSeq == seq))
            {
                *rttNs = gTalker.estProbeEchoT4Ns - t1Ns;
                ok = true;
            }
            else
            {
                EnetAppUtils_print("  [%s seq %u] no echo (timeout)\r\n", label, seq);
            }
        }
    }
    return ok;
}

static void TsnDemoTalker_estVerify(void)
{
    uint64_t cycleNs, baseNs, targetNs, rttNs;
    uint64_t gatedSum = 0ULL, gatedMin = UINT64_MAX, gatedMax = 0ULL;
    uint64_t openSum  = 0ULL, openMin  = UINT64_MAX, openMax  = 0ULL;
    uint32_t gatedOk = 0U, openOk = 0U;
    uint32_t i;

    Enet_MacPort linkedPort;
    EnetTas_ControlList liveList;
    uint64_t win0MidOffsetNs, win1MidOffsetNs;

    if ((!gTsnDemoCfg.features.estEn) || (gTsnDemo.estBaseTimeNs == 0ULL))
    {
        EnetAppUtils_print("EST is not enabled - press 'e' first\r\n");
    }
    else if (gTalker.numListeners < 1U)
    {
        EnetAppUtils_print("Need at least 1 listener (node 1)\r\n");
    }
    else if (gTalker.bulkEnabled)
    {
        EnetAppUtils_print(
            "WARNING: free-run BULK is still on - it will confound this "
            "test (software queueing delay vs. gate delay look the same). "
            "Press 'B' to stop it, then retry.\r\n");
    }
    else if (!TsnDemoTalker_findLinkedPort(&linkedPort))
    {
        EnetAppUtils_print("No linked MAC port found\r\n");
    }
    else if ((!TsnDemoFeature_getLiveOperList(linkedPort, &liveList)) ||
             (liveList.listLength < 2U))
    {
        EnetAppUtils_print(
            "GET_OPER_LIST failed or unexpected list on port %u\r\n",
            ENET_MACPORT_ID(linkedPort));
    }
    else
    {
        /*
         * Use the REAL hardware schedule, not the nominal
         * TSNDEMO_EST_CYCLE_NS/gTsnDemo.estBaseTimeNs - confirmed on hardware
         * (via 'g') that hardware's actual cycle can run measurably longer
         * than requested, by a different amount per port, and that error
         * compounds every cycle. baseNs/cycleNs here are what
         * TsnDemoTalker_nextWindowInstant() actually schedules against.
         */
        baseNs  = liveList.baseTime;
        cycleNs = 0ULL;
        for (i = 0U; i < liveList.listLength; i++)
        {
            cycleNs += liveList.gateCmdList[i].timeInterval;
        }
        win0MidOffsetNs = (uint64_t)liveList.gateCmdList[0].timeInterval / 2U;
        win1MidOffsetNs = (uint64_t)liveList.gateCmdList[0].timeInterval +
                          ((uint64_t)liveList.gateCmdList[1].timeInterval / 2U);

        /* Split across multiple EnetAppUtils_print() calls - a single call
         * combining all of this easily exceeds ENET_CFG_PRINT_BUF_LEN (200
         * bytes, enet_cfg.h), which silently truncates via vsnprintf rather
         * than erroring, and was quietly eating the baseTime/real-cycle
         * numbers (the exact ones this readback exists to show) plus the
         * whole second sentence below. */
        EnetAppUtils_print(
            "EST gate verification vs. listener node 1, port %u: %u probes "
            "timed mid-window0 (bulk PCP, expected GATE-CLOSED) + %u control "
            "probes timed mid-window1 (bulk PCP, gate OPEN).\r\n",
            ENET_MACPORT_ID(linkedPort),
            TSNDEMO_ESTVERIFY_SAMPLES, TSNDEMO_ESTVERIFY_SAMPLES);
        EnetAppUtils_print(
            "Using LIVE schedule: baseTime=%llu.%09llu real cycle=%llu ns "
            "(nominal %u).\r\n",
            baseNs / 1000000000ULL, baseNs % 1000000000ULL, cycleNs,
            gTsnDemo.estCycleTimeNs);
        EnetAppUtils_print(
            "Real gating looks like: gated-group RTT visibly larger than "
            "open-group RTT, by up to about one cycle. Near-zero difference "
            "means EST likely isn't blocking this PCP where expected.\r\n");

        /*
         * Untimed, discarded warm-up probe before either counted group -
         * confirmed on hardware that the first probe after this function's
         * setup runs ~4-6us slower than every subsequent one, regardless of
         * gate state. Sending all 5 "gated" probes before any "open" probe
         * (the old layout) let that one-time cost land exclusively on the
         * gated group, entirely accounting for the apparent gating signal
         * in early runs (samples 2-5 of each group showed no difference).
         * This warm-up absorbs that cost once, up front, so it can't bias
         * either group.
         */
        EnetAppUtils_print("-- warm-up (untimed, discarded) --\r\n");
        targetNs = TsnDemoTalker_nextWindowInstant(baseNs, cycleNs,
                                                   win1MidOffsetNs);
        (void)TsnDemoTalker_estVerifySendOne(targetNs, gTalker.estProbeSeq,
                                             "warmup", &rttNs);
        gTalker.estProbeSeq++;

        /*
         * Interleaved (gated, open, gated, open, ...), not all-gated-then-
         * all-open - same reasoning as the warm-up above: any remaining
         * per-probe drift (cache state, scheduler jitter) should land on
         * both groups roughly equally this way instead of systematically
         * favoring whichever group runs second.
         */
        EnetAppUtils_print(
            "-- interleaved: gated (mid window0) / open (mid window1) --\r\n");
        for (i = 0U; i < TSNDEMO_ESTVERIFY_SAMPLES; i++)
        {
            targetNs = TsnDemoTalker_nextWindowInstant(baseNs, cycleNs,
                                                        win0MidOffsetNs);
            if (TsnDemoTalker_estVerifySendOne(targetNs, gTalker.estProbeSeq,
                                              "gated", &rttNs))
            {
                EnetAppUtils_print("  [gated seq %u] rtt=%llu ns\r\n",
                                   gTalker.estProbeSeq, rttNs);
                gatedSum += rttNs;
                gatedMin  = (rttNs < gatedMin) ? rttNs : gatedMin;
                gatedMax  = (rttNs > gatedMax) ? rttNs : gatedMax;
                gatedOk++;
            }
            gTalker.estProbeSeq++;

            targetNs = TsnDemoTalker_nextWindowInstant(baseNs, cycleNs,
                                                        win1MidOffsetNs);
            if (TsnDemoTalker_estVerifySendOne(targetNs, gTalker.estProbeSeq,
                                              "open", &rttNs))
            {
                EnetAppUtils_print("  [open  seq %u] rtt=%llu ns\r\n",
                                   gTalker.estProbeSeq, rttNs);
                openSum += rttNs;
                openMin  = (rttNs < openMin) ? rttNs : openMin;
                openMax  = (rttNs > openMax) ? rttNs : openMax;
                openOk++;
            }
            gTalker.estProbeSeq++;
        }

        EnetAppUtils_print("-- summary --\r\n");
        if (gatedOk > 0U)
        {
            EnetAppUtils_print(
                "  gated (%u/%u ok): min=%llu avg=%llu max=%llu ns\r\n",
                gatedOk, TSNDEMO_ESTVERIFY_SAMPLES, gatedMin,
                gatedSum / gatedOk, gatedMax);
        }
        if (openOk > 0U)
        {
            EnetAppUtils_print(
                "  open  (%u/%u ok): min=%llu avg=%llu max=%llu ns\r\n",
                openOk, TSNDEMO_ESTVERIFY_SAMPLES, openMin,
                openSum / openOk, openMax);
        }
        if ((gatedOk > 0U) && (openOk > 0U))
        {
            EnetAppUtils_print(
                "  delta (gated avg - open avg) = %lld ns\r\n",
                (int64_t)(gatedSum / gatedOk) - (int64_t)(openSum / openOk));
        }
    }
}

/*
 * TEMPORARY DIAGNOSTIC (remove once GENF/RTI periodicity investigation is
 * resolved) - see gPacingDiag's declaration for why this captures from
 * TsnDemoTalker_expressTask() (task context) rather than any ISR. Needs
 * pacing to already be ticking (start a run with '3', or free-run express
 * with 'T') - this only observes ticks that are already happening, it
 * does not start pacing itself.
 */
#define TSNDEMO_PACING_DIAG_SAMPLES     (1000U)
#define TSNDEMO_PACING_DIAG_TIMEOUT_US  (2000000U)

static void TsnDemoTalker_pacingPeriodicity(void)
{
    int32_t pendStatus;

    if (!gTalker.expressEnabled)
    {
        EnetAppUtils_print(
            "pacing ticks aren't running - start a run ('1'/'2'/'3') or "
            "free-run express ('T') first\r\n");
    }
    else
    {
        gPacingDiag.target     = TSNDEMO_PACING_DIAG_SAMPLES;
        gPacingDiag.count      = 0U;
        gPacingDiag.lastTs     = 0ULL;
        gPacingDiag.sumDeltaNs = 0ULL;
        gPacingDiag.maxDeltaNs = 0ULL;
        gPacingDiag.minDeltaNs = UINT64_MAX;
        gPacingDiag.countLow   = 0U;
        gPacingDiag.countMid   = 0U;
        gPacingDiag.countHigh  = 0U;
        gPacingDiag.active     = true;

        EnetAppUtils_print("pacing periodicity: capturing %u tick deltas "
                           "(GENF-locked: %s)...\r\n",
                           TSNDEMO_PACING_DIAG_SAMPLES,
                           TsnDemoPacing_isLocked() ? "yes" : "no");

        pendStatus = SemaphoreP_pend(
            &gPacingDiag.doneSem,
            ClockP_usecToTicks(TSNDEMO_PACING_DIAG_TIMEOUT_US));
        if (pendStatus != SystemP_SUCCESS)
        {
            gPacingDiag.active = false;
            EnetAppUtils_print(
                "pacing periodicity: timed out after %u/%u samples - pacing "
                "may have stopped mid-capture\r\n",
                gPacingDiag.count, TSNDEMO_PACING_DIAG_SAMPLES);
        }
        else
        {
            /* Split across multiple EnetAppUtils_print() calls - a single call
             * combining all of this can exceed ENET_CFG_PRINT_BUF_LEN (200 bytes,
             * enet_cfg.h), which silently truncates via vsnprintf. */
            EnetAppUtils_print(
                "pacing periodicity: min=%llu avg=%llu max=%llu ns over %u "
                "deltas\r\n",
                gPacingDiag.minDeltaNs, gPacingDiag.sumDeltaNs / gPacingDiag.count,
                gPacingDiag.maxDeltaNs, gPacingDiag.count);
            EnetAppUtils_print(
                "  (task-context capture - includes some wake latency on top of "
                "the raw RTI period)\r\n");
            EnetAppUtils_print(
                "  buckets: <%llu us: %u   %llu-%llu us: %u   >%llu us: %u\r\n",
                TSNDEMO_PACING_DIAG_LOW_NS / 1000ULL, gPacingDiag.countLow,
                TSNDEMO_PACING_DIAG_LOW_NS / 1000ULL,
                TSNDEMO_PACING_DIAG_HIGH_NS / 1000ULL, gPacingDiag.countMid,
                TSNDEMO_PACING_DIAG_HIGH_NS / 1000ULL, gPacingDiag.countHigh);
        }
    }
}

static void TsnDemoTalker_printStatus(void)
{
    EnetAppUtils_print("--- Status -------------------------------------------------\r\n");
    EnetAppUtils_print(" gPTP           : %s\r\n",
                       TsnDemoEnet_isClockStable() ? "sync-stable" : "NOT stable");
    TsnDemoEnet_printClockDiag();
    EnetAppUtils_print(" features       : EST=%s IET=%s CUTTHRU=%s\r\n",
                       gTsnDemoCfg.features.estEn ? "on" : "off",
                       gTsnDemoCfg.features.ietEn ? "on" : "off",
                       gTsnDemoCfg.features.cutThruEn ? "on" : "off");
    EnetAppUtils_print(" chain features : 0x%02x (from last echo)\r\n",
                       gTalker.chainFeatureBits);
    EnetAppUtils_print(" pacing         : %s\r\n",
                       TsnDemoPacing_isLocked() ? "GENF gate-locked" :
                                                  "plain timer, NOT gate-locked");
    EnetAppUtils_print(" echoes seen    : %u\r\n", gTalker.echoCount);
    EnetAppUtils_print(" ts-read-fail   : %u\r\n", gTsnDemo.tsReadFailCount);
}

/* ------------------------------ entry point ------------------------------- */

static void TsnDemoTalker_promptNodeCfg(void)
{
    int32_t num;

    gTsnDemoCfg.nodeId = 0U;
    /* Fixed cabling convention (see TsnDemoListener_promptNodeCfg): MAC2
     * always faces downstream into the chain. The talker's MAC1, if wired
     * to anything, is an external, non-chain link (e.g. a Linux box
     * driving bulk load) - TsnDemoFeature_applyAll() reads this to keep
     * IET off there regardless of console input (see IET loop comment). */
    gTsnDemoCfg.downstreamPort = ENET_MAC_PORT_2;
    while (true)
    {
        EnetAppUtils_print("Enter number of nodes [2..%u]: ", TSNDEMO_MAX_NODES);
        DebugP_scanf("%d", &num);
        if ((num >= 2) && (num <= (int32_t)TSNDEMO_MAX_NODES))
        {
            gTsnDemoCfg.numNodes = (uint8_t)num;
            break;
        }
        EnetAppUtils_print("invalid\r\n");
    }
    /* Probes now fan out to every listener each tick (nodeId 1..numNodes-1),
     * not just the last node - see TsnDemoTalker_expressTask. */
    gTalker.numListeners = gTsnDemoCfg.numNodes - 1U;
}

static void TsnDemoTalker_createTasks(void)
{
    TaskP_Params taskParams;
    int32_t status;

    status = SemaphoreP_constructBinary(&gTalker.expressTxSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    status = SemaphoreP_constructBinary(&gTalker.estProbeEchoSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    status = SemaphoreP_constructBinary(&gPacingDiag.doneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    gTalker.tasksRun = true;

    TaskP_Params_init(&taskParams);
    taskParams.priority  = TSNDEMO_EXPRESS_TASK_PRI;
    taskParams.stack     = gExpressTaskStack;
    taskParams.stackSize = sizeof(gExpressTaskStack);
    taskParams.name      = "tsndemo_express";
    taskParams.taskMain  = &TsnDemoTalker_expressTask;
    status = TaskP_construct(&gTalker.expressTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);

#ifdef TSNDEMO_ENABLE_BULK_TASK
    TaskP_Params_init(&taskParams);
    taskParams.priority  = TSNDEMO_BULK_TASK_PRI;
    taskParams.stack     = gBulkTaskStack;
    taskParams.stackSize = sizeof(gBulkTaskStack);
    taskParams.name      = "tsndemo_bulk";
    taskParams.taskMain  = &TsnDemoTalker_bulkTask;
    status = TaskP_construct(&gTalker.bulkTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
}

void TsnDemoTalker_mainTask(void *args)
{
    char option;
    bool done = false;
    uint32_t nodeIdx;

    EnetAppUtils_print("==========================\r\n");
    EnetAppUtils_print("  TSN Demo - Talker\r\n");
    EnetAppUtils_print("==========================\r\n");

    memset(&gTalker, 0, sizeof(gTalker));
    TsnDemoTalker_promptNodeCfg();

    EnetAppUtils_assert(TsnDemoEnet_open() == ENET_SOK);
    EnetAppUtils_assert(TsnDemoEnet_initTsn() == 0);

    TsnDemoFlow_createRxTask(TsnDemoTalker_rxFrameCb);
    TsnDemoTalker_createTasks();

    /* Bring-up: block until any MAC port actually has link, independent of
     * gPTP - lets a PC/Wireshark be cabled straight to either port with no
     * peer at all (see TSNDEMO_SKIP_SYNC_WAIT below). */
    TsnDemoEnet_waitForAnyLinkUp();

    /* This node is the gPTP grandmaster (GPTP_MASTER). NOTE: on a
     * grandmaster, gPTP gives no protocol feedback on whether any listener
     * has actually locked to us - this only confirms the local BMCA
     * election settled, not that the chain is synced (see
     * TsnDemoEnet_printClockDiag / docs). Operator must check each
     * listener's own console for "gPTP sync-stable" - which IS a real,
     * measured lock - before starting a run on the talker. */
    TsnDemoEnet_waitClockStable(false);
    if (!TsnDemoEnet_checkLinkSpeed())
    {
        EnetAppUtils_print("Fix link speeds before running (see docs)\r\n");
    }

    TsnDemoTalker_printMenu();
    while (!done)
    {
        DebugP_scanf("%c", &option);
        switch (option)
        {
            case '1':
                TsnDemoTalker_runMode(TSNDEMO_MODE_BASELINE);
                break;
            case '2':
                TsnDemoTalker_runMode(TSNDEMO_MODE_CONTENTION_NOTSN);
                break;
            case '3':
                TsnDemoTalker_runMode(TSNDEMO_MODE_CONTENTION_TSN);
                break;
            case 'e':
#if TSNDEMO_IET_SUPPORTED
            case 'i':
#endif
#if TSNDEMO_CUTTHRU_SUPPORTED
            case 'u':
#endif
                TsnDemoTalker_toggleFeature(option);
                break;
            case 'g':
                TsnDemoFeature_printSchedule(&gTsnDemoCfg.features);
                break;
            case 'p':
                for (nodeIdx = 0U; nodeIdx < gTalker.numListeners; nodeIdx++)
                {
                    EnetAppUtils_print(
                        "--- Listener node %u ---------------------------------------\r\n",
                        nodeIdx + 1U);
                    TsnDemoStats_print(&gTalker.cur[nodeIdx]);
                }
                break;
            case 'c':
                for (nodeIdx = 0U; nodeIdx < gTalker.numListeners; nodeIdx++)
                {
                    EnetAppUtils_print(
                        "--- Listener node %u ---------------------------------------\r\n",
                        nodeIdx + 1U);
                    TsnDemoStats_printComparison(&gTalker.lastTsnOff[nodeIdx],
                                                 &gTalker.lastTsnOn[nodeIdx]);
                }
                break;
            case 'd':
                TsnDemoTalker_printStatus();
                break;
#if defined(TSNDEMO_ENABLE_DEBUG_MENU)
            case 's':
                TsnDemoEnet_printStats();
                break;
            case 'T':
                TsnDemoTalker_toggleFreeRun(true);
                break;
            case 'B':
                TsnDemoTalker_toggleFreeRun(false);
                break;
            case 'v':
                TsnDemoTalker_estVerify();
                break;
            case 'y':
                TsnDemoTalker_pacingPeriodicity();
                break;
#endif
            case 'x':
                done = true;
                break;
            default:
                TsnDemoTalker_printMenu();
                break;
        }
    }

    TsnDemoPacing_stop();
    gTalker.tasksRun = false;
    TsnDemoEnet_close();
}
