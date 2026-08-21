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
 * \file  tsndemo_listener.c
 *
 * \brief TSN demo - listener application: echoes express probes back to the
 *        talker with T2 (hardware RX timestamp) and T3 (software TX
 *        timestamp), and applies EST/IET/cut-through on its own ports from
 *        a UART menu.
 *
 *        Transit traffic toward other nodes never reaches this code - the
 *        CPSW ALE switches it in hardware.
 *
 *        User guide: docs/networking/enet_lld/tsn_demo_listener.md
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <string.h>
#include "tsndemo.h"

#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"

/* ========================================================================== */
/*                            Local Variables                                 */
/* ========================================================================== */

/*! Verbose trace: number of frames dumped per 'v' activation. Bounded so a
 *  sustained 8000/s probe rate cannot flood the console. */
#define TSNDEMO_TRACE_MAX       (10U)

static struct
{
    volatile uint32_t probesReceived;
    volatile uint32_t echoesSent;
    volatile uint32_t echoAllocFail;
    volatile uint32_t traceLeft;    /*!< >0 = dump the next N probes */
} gListener;

/*! Bulk-EtherType reflector state - only meaningful/allowed on the last
 *  node in the chain (see the 'L' command below). */
static bool gBulkReflectorEn = false;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
 * Echo responder, runs in the RX task. The RX buffer belongs to the RX
 * channel, so the echo is built in a TX packet: header copied through, T2/T3
 * filled in, MACs/PCP set, sent back toward the talker.
 */
static void TsnDemoListener_rxFrameCb(EnetDma_Pkt *pktInfo,
                                      EthVlanFrame *rxFrame,
                                      TsnDemo_Hdr *rxHdr)
{
    uint64_t t2Ns = pktInfo->tsInfo.rxPktTs;   /* hardware ingress timestamp */
    /* Echo goes to the talker's probe multicast (node 0) with the same
     * PCP/VLAN as the probe, so it competes in the same egress class. */
    const uint8_t talkerMac[ENET_MAC_ADDR_LEN] = TSNDEMO_NODE_MCAST(0U);
    uint16_t rxTci = Enet_ntohs(rxFrame->hdr.tci);
    TsnDemo_StreamCfg echoStream =
    {
        .kind       = TSNDEMO_STREAM_EXPRESS,
        .payloadLen = (uint16_t)(pktInfo->sgList.list[0].segmentFilledLen
                                 - sizeof(EthVlanFrameHeader)),
        .pcp        = (uint8_t)((rxTci >> 13U) & 0x7U),
        .vlanId     = (uint16_t)(rxTci & 0xFFFU),
        .dstNodeId  = 0U,
    };
    EthVlanFrame *txFrame;
    TsnDemo_Hdr *txHdr;
    EnetDma_Pkt *txPkt;
    /* Match TsnDemoFlow_reclaimTxPkts()'s PCP-based routing: a bulk-PCP
     * echo (EST verification probes) must be allocated from the bulk pool,
     * or reclaim will return it to the wrong pool and drain the express
     * one instead. */
    EnetDma_PktQ *echoPool = (echoStream.pcp == TSNDEMO_PCP_BULK) ?
                             &gTsnDemo.bulkFreePktInfoQ : &gTsnDemo.txFreePktInfoQ;

    /*
     * No message-type or destination check needed here: this node's ALE
     * registers only its own probe multicast on the host port, so anything
     * arriving here is addressed to us. Bulk uses an unregistered
     * multicast that never reaches a host port, and echoes are addressed
     * to the talker's multicast, not this node's - so EtherType 0x88B5 at
     * a listener can only ever be a PROBE (see TSNDEMO_ETHERTYPE in
     * tsndemo.h).
     */
    gListener.probesReceived++;

    TsnDemoFlow_reclaimTxPkts();
    txPkt = TsnDemoFlow_allocTxFrame(echoPool, &echoStream, talkerMac,
                                     &txFrame, &txHdr);
    if (txPkt == NULL)
    {
        gListener.echoAllocFail++;
    }
    else
    {
        /* Copy the probe header through, then stamp */
        memcpy(txHdr, rxHdr, sizeof(TsnDemo_Hdr));
        txHdr->featureBits = TsnDemoFeature_toBits(&gTsnDemoCfg.features);
        /*
         * T2/T3 stamping disabled for now: one-way (fwd/rev) and turnaround
         * depend on gPTP sync between talker and listener, which is under
         * investigation. Leaving t2Ns/t3Ns at 0 (inherited from the probe,
         * already zeroed by the talker) makes TsnDemoStats_addSample()
         * treat every fwd/rev/turn sample as a failed read and skip it, so
         * only RTT (T4-T1, talker's own clock) stays meaningful. See
         * docs/networking/enet_lld/tsn_demo_percentile_stats.md to restore.
         */
         txHdr->t2Ns = t2Ns;
        // txHdr->t3Ns = TsnDemoFlow_getTimeNs();

        if (TsnDemoFlow_submitTxPkt(txPkt) == ENET_SOK)
        {
            gListener.echoesSent++;
        }

        /* Bring-up trace: shows the timestamps without decoding a capture.
         * See the bring-up guide, stage B2/B3. */
        if (gListener.traceLeft > 0U)
        {
            gListener.traceLeft--;
            EnetAppUtils_print(
                "[probe] seq=%u pcp=%u len=%u t1=%llu t2=%llu t3=%llu "
                "turn=%llu ns\r\n",
                rxHdr->seqNum, echoStream.pcp, echoStream.payloadLen,
                rxHdr->t1Ns, t2Ns, txHdr->t3Ns,
                (txHdr->t3Ns > t2Ns) ? (txHdr->t3Ns - t2Ns) : 0ULL);
        }
    }
}

/* -------------------------------- console -------------------------------- */

static void TsnDemoListener_printMenu(void)
{
    EnetAppUtils_print("\r\n--- TSN Demo Listener -------------------------------------\r\n");
    EnetAppUtils_print(" 'e'  -  Toggle EST\r\n");
#if TSNDEMO_IET_SUPPORTED
    EnetAppUtils_print(" 'i'  -  Toggle IET\r\n");
#endif
#if TSNDEMO_CUTTHRU_SUPPORTED
    EnetAppUtils_print(" 'u'  -  Toggle cut-through\r\n");
#endif
    EnetAppUtils_print(" 'g'  -  Show EST schedule\r\n");
    EnetAppUtils_print(" 'd'  -  Status\r\n");
#if defined(TSNDEMO_ENABLE_DEBUG_MENU)
    EnetAppUtils_print(" 's'  -  CPSW statistics\r\n");
#endif
    EnetAppUtils_print(" 'v'  -  Trace next %u probes  'r'  -  Reset counters\r\n",
                       TSNDEMO_TRACE_MAX);
    EnetAppUtils_print(" 'L'  -  Toggle bulk reflector (last node in chain only)\r\n");
    EnetAppUtils_print(" 'x'  -  Stop\r\n");
}

static void TsnDemoListener_toggleFeature(char which)
{
    TsnDemo_FeatureSet want = gTsnDemoCfg.features;

    switch (which)
    {
        case 'e': want.estEn     = !want.estEn;     break;
        case 'i': want.ietEn     = !want.ietEn;     break;
        case 'u': want.cutThruEn = !want.cutThruEn; break;
        default: return;
    }
    (void)TsnDemoFeature_applyAll(&want);
}

static void TsnDemoListener_printStatus(void)
{
    EnetAppUtils_print("--- Status -------------------------------------------------\r\n");
    EnetAppUtils_print(" gPTP            : %s\r\n",
                       TsnDemoEnet_isClockStable() ? "sync-stable"
                                                   : "NOT stable");
    TsnDemoEnet_printClockDiag();
    EnetAppUtils_print(" features        : EST=%s IET=%s CUTTHRU=%s\r\n",
                       gTsnDemoCfg.features.estEn ? "on" : "off",
                       gTsnDemoCfg.features.ietEn ? "on" : "off",
                       gTsnDemoCfg.features.cutThruEn ? "on" : "off");
    EnetAppUtils_print(" demo frames rx  : %u  (all valid 0x88B5+magic)\r\n",
                       gTsnDemo.rxDemoFrameCount);
    EnetAppUtils_print(" probes received : %u  (addressed to this node)\r\n",
                       gListener.probesReceived);
    EnetAppUtils_print(" echoes sent     : %u\r\n", gListener.echoesSent);
    EnetAppUtils_print(" echo alloc fail : %u\r\n", gListener.echoAllocFail);
    EnetAppUtils_print(" ts-read-fail    : %u\r\n", gTsnDemo.tsReadFailCount);
    EnetAppUtils_print(" bulk reflector  : %s\r\n",
                       gBulkReflectorEn ? "ENABLED" : "disabled");
}

/* ------------------------------ entry point ------------------------------- */

static void TsnDemoListener_promptNodeCfg(void)
{
    int32_t num;

    while (true)
    {
        EnetAppUtils_print("Enter node id [1..%u]: ", TSNDEMO_MAX_NODES - 1U);
        DebugP_scanf("%d", &num);
        if ((num >= 1) && (num < (int32_t)TSNDEMO_MAX_NODES))
        {
            gTsnDemoCfg.nodeId = (uint8_t)num;
            break;
        }
        EnetAppUtils_print("invalid\r\n");
    }

    /* Needed for chain-position-staggered EST
     * (TsnDemoFeature_computePortWindow) - this node's distance from the
     * far end of the chain. */
    while (true)
    {
        EnetAppUtils_print("Enter number of nodes [2..%u]: ", TSNDEMO_MAX_NODES);
        DebugP_scanf("%d", &num);
        if ((num > (int32_t)gTsnDemoCfg.nodeId) &&
            (num <= (int32_t)TSNDEMO_MAX_NODES))
        {
            gTsnDemoCfg.numNodes = (uint8_t)num;
            break;
        }
        EnetAppUtils_print("invalid (must be > node id)\r\n");
    }

    /* Fixed cabling convention, not asked: every node's MAC2 faces away
     * from the talker (downstream) into the next node's MAC1 - talker
     * MAC2 -> listener1 MAC1, listener1 MAC2 -> listener2 MAC1, etc.
     * The last node in the chain (nodeId == numNodes-1, now derivable
     * since numNodes is known) has no downstream neighbor - its MAC2 is
     * unused/unconnected, so it gets no downstream EST schedule. */
    gTsnDemoCfg.downstreamPort =
        ((uint32_t)gTsnDemoCfg.nodeId == ((uint32_t)gTsnDemoCfg.numNodes - 1U)) ?
        ENET_MAC_PORT_INV : ENET_MAC_PORT_2;
}

void TsnDemoListener_mainTask(void *args)
{
    char option;
    bool done = false;

    EnetAppUtils_print("==========================\r\n");
    EnetAppUtils_print("  TSN Demo - Listener\r\n");
    EnetAppUtils_print("==========================\r\n");

    memset(&gListener, 0, sizeof(gListener));
    TsnDemoListener_promptNodeCfg();

    EnetAppUtils_assert(TsnDemoEnet_open() == ENET_SOK);
    EnetAppUtils_assert(TsnDemoEnet_initTsn() == 0);

    /* Echo responder is live from here; timestamps become meaningful once
     * gPTP is stable (the talker gates its runs on that). */
    TsnDemoFlow_createRxTask(TsnDemoListener_rxFrameCb);

    EnetAppUtils_print("Echo responder running, nodeId=%u\r\n",
                       gTsnDemoCfg.nodeId);
    /* requireSlaveRole=true: every gPTP device defaults to self-elected
     * grandmaster before it has heard the talker's Announce, so don't let
     * the menu open on that trivial/premature "stable" state - wait until
     * this node has genuinely ceded GM role to the real master. */
    TsnDemoEnet_waitClockStable(true);
    (void)TsnDemoEnet_checkLinkSpeed();

    TsnDemoListener_printMenu();
    while (!done)
    {
        DebugP_scanf("%c", &option);
        switch (option)
        {
            case 'e':
#if TSNDEMO_IET_SUPPORTED
            case 'i':
#endif
#if TSNDEMO_CUTTHRU_SUPPORTED
            case 'u':
#endif
                TsnDemoListener_toggleFeature(option);
                break;
            case 'g':
                TsnDemoFeature_printSchedule(&gTsnDemoCfg.features);
                break;
            case 'd':
                TsnDemoListener_printStatus();
                break;
            case 'v':
                gListener.traceLeft = TSNDEMO_TRACE_MAX;
                EnetAppUtils_print("Tracing next %u probes...\r\n",
                                   TSNDEMO_TRACE_MAX);
                break;
            case 'r':
                gListener.probesReceived = 0U;
                gListener.echoesSent     = 0U;
                gListener.echoAllocFail  = 0U;
                gTsnDemo.tsReadFailCount = 0U;
                gTsnDemo.rxDemoFrameCount = 0U;
                EnetAppUtils_print("Counters reset\r\n");
                break;
#if defined(TSNDEMO_ENABLE_DEBUG_MENU)
            case 's':
                TsnDemoEnet_printStats();
                break;
#endif
            case 'L':
                if (gTsnDemoCfg.downstreamPort != ENET_MAC_PORT_INV)
                {
                    EnetAppUtils_print(
                        "Bulk reflector refused: this node has a "
                        "downstream neighbor - only the last node in the "
                        "chain (downstream port = 0) should reflect bulk "
                        "traffic back upstream\r\n");
                }
                else
                {
                    Enet_MacPort upstreamPort = TsnDemoEnet_findLinkedPort();

                    if (upstreamPort == ENET_MAC_PORT_INV)
                    {
                        EnetAppUtils_print(
                            "Bulk reflector refused: expected exactly one "
                            "linked MAC port on the last node, found "
                            "none/more than one\r\n");
                    }
                    else
                    {
                        gBulkReflectorEn = !gBulkReflectorEn;
                        TsnDemoEnet_setBulkReflector(upstreamPort,
                                                     gBulkReflectorEn);
                    }
                }
                break;
            case 'x':
                done = true;
                break;
            default:
                TsnDemoListener_printMenu();
                break;
        }
    }

    TsnDemoEnet_close();
}
