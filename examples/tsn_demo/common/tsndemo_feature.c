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
 * \file  tsndemo_feature.c
 *
 * \brief TSN demo - runtime configuration of EST (802.1Qbv), IET frame
 *        preemption (802.3br) and cut-through switching.
 *
 *        EST uses the direct TAS IOCTLs (ENET_TAS_IOCTL_SET_ADMIN_LIST /
 *        SET_STATE) - no YANG/uniconf database involved.
 *        The driver programs the CPTS ESTF from the admin list
 *        automatically (Cpsw_setupEstf), so baseTime/cycleTime are also the
 *        phase/period reference for the talker's generator pacing.
 *
 *        The EST base time is DERIVED, not distributed: every node rounds
 *        gPTP time up to the next 1 s boundary plus a fixed lead, so all
 *        nodes configured within the lead window compute the same value.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <string.h>
#include "tsndemo.h"

#include <tsn_gptp/gptpmasterclock.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*! Cut-through priority mask: bit N = enable for switch priority N.
 *  Applied to all 8 priorities so bulk (PCP_BULK) frames also get cut through -
 *  otherwise a large in-flight bulk frame still fully occupies each hop before
 *  an express frame behind it can be relayed, even with cut-through "on". */
#define TSNDEMO_CUTTHRU_PRI_MASK        (0xFFU)

#define TSNDEMO_EST_OPER_POLL_RETRIES   (100U)
#define TSNDEMO_IET_VERIFY_ATTEMPTS     (20U)

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
 * gptpmasterclock_getts64() requires gptpmasterclock_init() to have been
 * called once, after the gPTP task is up. Every other TSN example in the SDK
 * does this in a retry loop (see examples/tsn/aafpcm_app/aaf_pcm_app.c:123).
 * Done lazily here so it also works in TSNDEMO_SKIP_SYNC_WAIT bring-up
 * builds, where the caller may reach EST configuration very early.
 */
static bool gTsnDemoMasterClockReady = false;

static bool TsnDemoFeature_ensureMasterClock(void)
{
    uint32_t attempt;
    bool ready = gTsnDemoMasterClockReady;

    if (!ready)
    {
        for (attempt = 0U; (attempt < 50U) && (!ready); attempt++)
        {
            if (gptpmasterclock_init(NULL) == 0)
            {
                gTsnDemoMasterClockReady = true;
                ready = true;
            }
            else
            {
                ClockP_usleep(100000U);
            }
        }
        if (!ready)
        {
            EnetAppUtils_print(
                "ERROR: gptpmasterclock_init() failed - cannot derive "
                "an EST base time\r\n");
        }
    }
    return ready;
}

uint64_t TsnDemoFeature_deriveBaseTime(void)
{
    uint64_t now;
    uint64_t targetNs;
    uint64_t baseTimeNs = 0ULL;

    if (TsnDemoFeature_ensureMasterClock())
    {
        now      = gptpmasterclock_getts64();
        targetNs = now + TSNDEMO_EST_BASETIME_MARGIN_NS;

        /* Round up to the next cycle boundary. Every node computes this
         * independently from its own gPTP-synced clock, so they all land on
         * the same cycle phase with no need to exchange a value. */
        baseTimeNs = ((targetNs + TSNDEMO_EST_CYCLE_NS - 1ULL) /
                     TSNDEMO_EST_CYCLE_NS) * TSNDEMO_EST_CYCLE_NS;
    }
    return baseTimeNs;
}

uint32_t TsnDemoFeature_guardBandNs(const TsnDemo_FeatureSet *features)
{
    /* With preemption the guard band only has to cover the minimum
     * non-final fragment instead of a full MTU frame. */
    return features->ietEn ? TSNDEMO_EST_GUARD_EST_IET_NS
                    : TSNDEMO_EST_GUARD_EST_ONLY_NS;
}

/*
 * Worst-case time a max-size bulk frame occupies the internal switch
 * fabric/wire without cut-through or preemption to bound it - scales
 * linearly with the actual configured bulk payload size, not a fixed
 * guess. Without either feature, a large bulk frame already mid-relay can
 * block the express frame from even reaching its egress queue for up to
 * its own store-and-forward time - that (not the express frame's own,
 * much smaller size) is the relevant worst case for staggering.
 */
static uint64_t TsnDemoFeature_hopIncNs(const TsnDemo_FeatureSet *features)
{
    uint32_t bulkPayloadLen = gTsnDemoCfg.stream[1].payloadLen;
    /* +38: 6+6 MAC addrs + 4 VLAN tag + 2 EtherType + 4 FCS + 8 preamble/
     * SFD + 12 IPG - total wire-occupying bytes beyond the payload. */
    uint64_t slowNs = ((uint64_t)bulkPayloadLen + 38ULL) * 8ULL;

    return (features->ietEn || features->cutThruEn) ?
           (uint64_t)TSNDEMO_EST_HOPINC_FAST_NS : slowNs;
}

/*
 * Bidirectional, chain-position-staggered window0 (express) start
 * offset + width for one MAC port (design discussion, see continuation
 * notes). Downstream (facing away from the talker, hop index = nodeId):
 * fixed-width window that starts later by hopInc per hop. Upstream
 * (facing the talker, carrying the echo): starts TSNDEMO_EST_ECHO_MIN_NS
 * after the far end's own arrival time, growing both start and width by
 * hopInc per hop further back - echo timing uncertainty compounds hop by
 * hop since each relay doesn't know exactly when within the previous
 * hop's window the frame actually left.
 */
void TsnDemoFeature_computePortWindow(Enet_MacPort macPort,
                                      const TsnDemo_FeatureSet *features,
                                      uint64_t *startOffsetNs,
                                      uint64_t *windowWidthNs)
{
    uint64_t hopInc = TsnDemoFeature_hopIncNs(features);
    uint64_t nodeId = (uint64_t)gTsnDemoCfg.nodeId;

    if (nodeId == 0ULL)
    {
        /* Talker: always hop index 0, downstream, unstaggered - it has no
         * downstreamPort/numNodes-as-listener semantics to look up. */
        *startOffsetNs = 0ULL;
        *windowWidthNs = TSNDEMO_EST_DOWNSTREAM_WIDTH_NS;
    }
    else if (macPort == gTsnDemoCfg.downstreamPort)
    {
        *startOffsetNs = nodeId * hopInc;
        *windowWidthNs = TSNDEMO_EST_DOWNSTREAM_WIDTH_NS;
    }
    else
    {
        /* upstream: k = hops back from the far end (nodeId numNodes-1) */
        uint64_t arrivalAtFarEndNs = ((uint64_t)gTsnDemoCfg.numNodes - 1ULL) * hopInc;
        uint64_t k = ((uint64_t)gTsnDemoCfg.numNodes - 1ULL) - nodeId;

        *startOffsetNs = arrivalAtFarEndNs + TSNDEMO_EST_ECHO_MIN_NS + (k * hopInc);
        *windowWidthNs = (TSNDEMO_EST_ECHO_MAX_NS - TSNDEMO_EST_ECHO_MIN_NS) +
                         (k * hopInc);
    }
}

/*
 * Three-entry schedule per cycle:
 *   1. express window : gate open for express PCP + gPTP only (protected)
 *   2. open window     : gate open for ALL priorities (bulk, and anything
 *                        else not explicitly protected)
 *   3. guard band      : only gPTP open (sized per IET state)
 *
 * baseTimeNs is the unstaggered reference (same for every node/port);
 * startOffsetNs is this specific port's chain-position stagger (0 for the
 * talker), added on top; windowWidthNs is this port's window0 width
 * (TsnDemoFeature_computePortWindow() - fixed downstream, growing
 * upstream).
 */
static void TsnDemoFeature_buildAdminList(EnetTas_ControlList *list,
                                          uint64_t baseTimeNs,
                                          uint64_t startOffsetNs,
                                          uint64_t windowWidthNs,
                                          bool ietActive)
{
    uint32_t guardNs = ietActive ? TSNDEMO_EST_GUARD_EST_IET_NS
                                 : TSNDEMO_EST_GUARD_EST_ONLY_NS;

    memset(list, 0, sizeof(*list));
    list->baseTime  = baseTimeNs + startOffsetNs;
    list->cycleTime = TSNDEMO_EST_CYCLE_NS;

    /* ENET_TAS_GATE_MASK(pri7, pri6, pri5, pri4, pri3, pri2, pri1, pri0) */
    list->gateCmdList[0].gateStateMask =
        ENET_TAS_GATE_MASK(1U, 0U, 1U, 0U, 0U, 0U, 0U, 0U); /* gPTP+express */
    list->gateCmdList[0].timeInterval = (uint32_t)windowWidthNs;

    /*
     * TEMPORARY DIAGNOSTIC (revert once used): pri5 (TSNDEMO_PCP_EXPRESS)
     * forced to 0 here, excluding express from window1. Window1 normally
     * opens every priority (see "Original comment" below), so an express
     * frame that misses its window0 slot can still go out during window1
     * instead of waiting a full cycle. That contention is the leading
     * theory for node2's wide RTT variance in the 2-hop CONTENTION-TSN-ON
     * runs (docs/networking/enet_lld/tsn_demo_test_continuation_notes.md):
     * observed variance is too small for a full-cycle miss (~+125000ns)
     * but too large for a clean window0 hit. Excluding express from
     * window1 turns any missed-window0 sample into an unambiguous
     * ~+125000ns bump, confirming/refuting the theory before touching
     * hop-staggering margins. Restore
     * ENET_TAS_GATE_MASK(1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U) once used.
     *
     * Original comment, for when this is reverted:
     * Window 1 opens every priority, not just bulk (PCP 1) - the
     * "everything else" window. Bulk uses it because that's the only PCP
     * the talker's bulk generator sends, but any other priority (e.g. a
     * transit node or manual packETH test) also gets through here rather
     * than being silently blocked. Only window 2 (the guard band) is
     * exclusive to gPTP.
     */
    list->gateCmdList[1].gateStateMask =
        ENET_TAS_GATE_MASK(1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U); /* bulk+gPTP only - express excluded */
    list->gateCmdList[1].timeInterval =
        (uint32_t)(TSNDEMO_EST_CYCLE_NS - windowWidthNs - guardNs);

    list->gateCmdList[2].gateStateMask =
        ENET_TAS_GATE_MASK(1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U); /* gPTP only */
    list->gateCmdList[2].timeInterval = guardNs;

    list->listLength = 3U;
}

int32_t TsnDemoFeature_estApply(Enet_MacPort macPort, bool enable,
                                uint64_t baseTimeNs, uint64_t startOffsetNs,
                                uint64_t windowWidthNs, bool ietActive)
{
    Enet_IoctlPrms prms;
    EnetTas_SetAdminListInArgs adminInArgs;
    EnetTas_OperStatus operStatus = ENET_TAS_OPER_LIST_NOT_YET_UPDATED;
    EnetTas_GenericInArgs genArgs;
    EnetTas_SetStateInArgs stateInArgs;
    bool linked;
    uint32_t retries;
    int32_t status = ENET_SOK;
    bool adminListOk = true;
    uint32_t guardNs;
    bool skipRemaining = false;

    if (enable)
    {
        /* No link partner: the driver validates gate interval widths
         * against the port's OWN reported link speed
         * (CpswEst_checkGateCmdList, cpsw_macport_est_ioctl.c) - an
         * unlinked port defaults to ENET_SPEED_10MBIT, raising the
         * minimum allowed interval to 6400 ns and rejecting the IET guard
         * band (1500 ns) with EINVALIDPARAMS. Confirmed on hardware (the
         * last node in a chain has one unconnected port - same case IET
         * handles below). Skip here too: nothing to gate without a link
         * partner, and without this check TsnDemoFeature_applyAll's
         * per-port loop would abort on the first unconnected port and
         * never reach the connected one. */
        linked = false;
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &linked);
        ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                   ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
        if (!linked)
        {
            EnetAppUtils_print(
                "EST: port %u has no link - skipping (nothing to gate)\r\n",
                ENET_MACPORT_ID(macPort));
            status = ENET_SOK;
            skipRemaining = true;
        }
        else
        {
            /* Upstream window0 width grows unbounded with chain depth (see
             * TsnDemoFeature_computePortWindow) - guard against it eating the
             * whole cycle (or underflowing window1's unsigned subtraction)
             * for a long, slow-hopInc chain, instead of silently programming
             * garbage. */
            guardNs = ietActive ? TSNDEMO_EST_GUARD_EST_IET_NS
                                 : TSNDEMO_EST_GUARD_EST_ONLY_NS;
            if ((windowWidthNs + guardNs) >= TSNDEMO_EST_CYCLE_NS)
            {
                EnetAppUtils_print(
                    "EST: port %u window0+guard (%llu ns) too wide for the "
                    "%u ns cycle - chain too long/slow for this cycle time\r\n",
                    ENET_MACPORT_ID(macPort),
                    (unsigned long long)(windowWidthNs + guardNs),
                    TSNDEMO_EST_CYCLE_NS);
                status = ENET_EINVALIDPARAMS;
                skipRemaining = true;
            }
            else
            {
                memset(&adminInArgs, 0, sizeof(adminInArgs));
                adminInArgs.macPort = macPort;
                TsnDemoFeature_buildAdminList(&adminInArgs.adminList, baseTimeNs,
                                              startOffsetNs, windowWidthNs, ietActive);

                ENET_IOCTL_SET_IN_ARGS(&prms, &adminInArgs);
                ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                           ENET_TAS_IOCTL_SET_ADMIN_LIST, &prms, status);
                if (status != ENET_SOK)
                {
                    EnetAppUtils_print("EST: SET_ADMIN_LIST port %u failed: %d\r\n",
                                       ENET_MACPORT_ID(macPort), status);
                    adminListOk = false;
                }

                if (adminListOk)
                {
                    /* SET_ADMIN_LIST is asynchronous - poll for the oper-list update */
                    retries = TSNDEMO_EST_OPER_POLL_RETRIES;
                    genArgs.macPort = macPort;
                    do
                    {
                        ENET_IOCTL_SET_INOUT_ARGS(&prms, &genArgs, &operStatus);
                        ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                                   ENET_TAS_IOCTL_GET_OPER_LIST_STATUS, &prms, status);
                        if (operStatus == ENET_TAS_OPER_LIST_UPDATED)
                        {
                            break;
                        }
                        ClockP_usleep(10000U);
                    } while (--retries > 0U);

                    if (operStatus != ENET_TAS_OPER_LIST_UPDATED)
                    {
                        EnetAppUtils_print("EST: oper list not updated on port %u\r\n",
                                           ENET_MACPORT_ID(macPort));
                        /* base time is in the future - the state machine holds the
                         * list as pending; still enable the gate control below */
                    }
                }
            }
        }
    }

    if ((!skipRemaining) && (adminListOk))
    {
        stateInArgs.macPort = macPort;
        stateInArgs.state   = enable ? ENET_TAS_ENABLE : ENET_TAS_DISABLE;

        ENET_IOCTL_SET_IN_ARGS(&prms, &stateInArgs);
        ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId, ENET_TAS_IOCTL_SET_STATE,
                   &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("EST: SET_STATE port %u failed: %d\r\n",
                               ENET_MACPORT_ID(macPort), status);
        }
    }
    return status;
}

#if ENET_CFG_IS_ON(CPSW_IET_INCL)
/*
 * Same shape the SysConfig template generates; queues at and above the
 * express priority stay express, the rest are preemptable.
 * minFragSize = 1 -> 128-byte minimum non-final fragment.
 */
static const EnetApp_IET_Config gTsnDemoIetCfg =
{
    .mac_verify_enable = true,
    .queueMode =
    {
        .preemptMode =
        {
            [0] = ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT,
            [1] = ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT,
            [2] = ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT,
            [3] = ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT,
            [4] = ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT,
            [5] = ENET_MAC_QUEUE_PREEMPT_MODE_EXPRESS,
            [6] = ENET_MAC_QUEUE_PREEMPT_MODE_EXPRESS,
            [7] = ENET_MAC_QUEUE_PREEMPT_MODE_EXPRESS,
        },
    },
    .minFragSize = 1U,
};

/*
 * Poll verification status; the handshake needs the link partner's
 * preemption enabled too (enable listeners first, talker last).
 * Same flow as EnetApp_doIetVerification() in the SysConfig template.
 */
static EnetMacPort_PreemptVerifyStatus TsnDemoFeature_ietVerify(
    Enet_MacPort macPort)
{
    Enet_IoctlPrms prms;
    EnetMacPort_GenericInArgs fpe = { .macPort = macPort };
    EnetMacPort_PreemptVerifyStatus verifyStatus =
        ENET_MAC_VERIFYSTATUS_UNKNOWN;
    uint32_t attempt = TSNDEMO_IET_VERIFY_ATTEMPTS;
    int32_t status;

    do
    {
        ENET_IOCTL_SET_IN_ARGS(&prms, &fpe);
        ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                   ENET_MACPORT_IOCTL_ENABLE_PREEMPT_VERIFICATION,
                   &prms, status);
        if (status != ENET_SOK)
        {
            break;
        }
        ClockP_usleep(50000U);
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &fpe, &verifyStatus);
        ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                   ENET_MACPORT_IOCTL_GET_PREEMPT_VERIFY_STATUS,
                   &prms, status);
        if (verifyStatus == ENET_MAC_VERIFYSTATUS_SUCCEEDED)
        {
            break;
        }
        attempt--;
    } while (attempt > 0U);

    return verifyStatus;
}
#endif /* CPSW_IET_INCL */

int32_t TsnDemoFeature_ietApply(Enet_MacPort macPort, bool enable)
{
#if ENET_CFG_IS_ON(CPSW_IET_INCL)
    Enet_IoctlPrms prms;
    EnetMacPort_GenericInArgs fpe = { .macPort = macPort };
    EnetMacPort_PreemptVerifyStatus vs;
    bool linked;
    int32_t status;

    if (!enable)
    {
        ENET_IOCTL_SET_IN_ARGS(&prms, &fpe);
        ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                   ENET_MACPORT_IOCTL_DISABLE_PREEMPT_VERIFICATION,
                   &prms, status);
        ENET_IOCTL_SET_IN_ARGS(&prms, &fpe);
        ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                   ENET_MACPORT_IOCTL_DISABLE_PREEMPTION, &prms, status);
    }
    else
    {
        linked = false;
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &linked);
        ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                   ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
        if (!linked)
        {
            /* No link partner to run the preempt-verify handshake with
             * (e.g. the last node in a chain has one unconnected port,
             * see TsnDemoEnet_checkLinkSpeed) - nothing to enable here,
             * not an error. Without this check, TsnDemoFeature_applyAll's
             * per-port loop would abort on the first unconnected port and
             * never even attempt the connected one. */
            EnetAppUtils_print(
                "IET: port %u has no link - skipping (nothing to verify "
                "against)\r\n", ENET_MACPORT_ID(macPort));
            status = ENET_SOK;
        }
        else
        {
            /* Enables preemption + min frag size + per-queue express/preempt
             * modes (utils/enet_apputils.c) */
            status = EnetAppUtils_enableIET(gTsnDemo.hEnet, gTsnDemo.coreId,
                                            macPort, &gTsnDemoIetCfg);
            if ((status == ENET_SOK) && (gTsnDemoIetCfg.mac_verify_enable))
            {
                vs = TsnDemoFeature_ietVerify(macPort);
                if (vs == ENET_MAC_VERIFYSTATUS_SUCCEEDED)
                {
                    EnetAppUtils_print("IET: verification SUCCEEDED on port %u\r\n",
                                       ENET_MACPORT_ID(macPort));
                }
                else
                {
                    EnetAppUtils_print("IET: verification failed on port %u: %d "
                                       "(is the link partner's IET enabled?)\r\n",
                                       ENET_MACPORT_ID(macPort), vs);
                    status = ENET_EFAIL;
                }
            }
        }
    }
    return status;
#else
    int32_t status = ENET_SOK;

    (void)macPort;
    if (enable)
    {
        EnetAppUtils_print("IET not supported on this SoC (CPSW_IET_INCL off)\r\n");
        status = ENET_ENOTSUPPORTED;
    }
    return status;
#endif
}

int32_t TsnDemoFeature_cutThruApply(Enet_MacPort macPort, bool enable)
{
#if ENET_CFG_IS_ON(CPSW_CUTTHRU)
    Enet_IoctlPrms prms;
    int32_t status;
    EnetMacPort_CutThruParams cutThruInArgs;

    memset(&cutThruInArgs, 0, sizeof(cutThruInArgs));
    cutThruInArgs.macPort = macPort;
    /* rx/txPriCutThruEn are 8-bit priority bitmasks (bit N = priority N) */
    cutThruInArgs.cutThruCfg.rxPriCutThruEn =
        enable ? TSNDEMO_CUTTHRU_PRI_MASK : 0U;
    cutThruInArgs.cutThruCfg.txPriCutThruEn =
        enable ? TSNDEMO_CUTTHRU_PRI_MASK : 0U;
    /* Must be 1: the IOCTL rejects portSpeedAutoEn == 0 */
    cutThruInArgs.cutThruCfg.portSpeedAutoEn = 1U;

    ENET_IOCTL_SET_IN_ARGS(&prms, &cutThruInArgs);
    ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
               ENET_MACPORT_IOCTL_SET_CUT_THRU_PARAMS, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Cut-through: config port %u failed: %d\r\n",
                           ENET_MACPORT_ID(macPort), status);
    }
    return status;
#else
    int32_t status = ENET_SOK;

    (void)macPort;
    if (enable)
    {
        EnetAppUtils_print("Cut-through not supported on this SoC\r\n");
        status = ENET_ENOTSUPPORTED;
    }
    return status;
#endif
}

int32_t TsnDemoFeature_applyAll(const TsnDemo_FeatureSet *features)
{
    int32_t status = ENET_SOK;
    int32_t ietStatus = ENET_SOK;
    int32_t cutThruStatus = ENET_SOK;
    int32_t estStatus = ENET_SOK;
    uint64_t baseTimeNs = 0ULL;
    uint64_t adminBaseTimeNs;
    uint64_t startOffsetNs;
    uint64_t windowWidthNs;
    uint32_t i;

    /* IET first: the EST guard band depends on it. Exception: on the
     * talker only, never ENABLE IET on a port other than the chain-facing
     * (downstream) one - that port may carry external, non-chain traffic
     * (e.g. a Linux box driving bulk load) that might not support IET's
     * mPacketVerify/mPacketResponse handshake, which requires the link
     * partner's cooperation. EST and cut-through are local-only (no
     * link-partner negotiation) and still apply uniformly per console
     * input. Disabling IET is always safe/idempotent, so it's applied
     * uniformly too - only the enable path is restricted. Listeners have
     * no such exception: both their ports are genuine chain links. */
    for (i = 0U; (i < gTsnDemo.numMacPorts) && (ietStatus == ENET_SOK); i++)
    {
        if ((features->ietEn) && (gTsnDemoCfg.nodeId == 0U) &&
            (gTsnDemo.macPorts[i] != gTsnDemoCfg.downstreamPort))
        {
            EnetAppUtils_print(
                "IET: port %u is the talker's external (non-chain) port - "
                "not enabling IET here regardless of console input\r\n",
                ENET_MACPORT_ID(gTsnDemo.macPorts[i]));
            continue;
        }
        ietStatus = TsnDemoFeature_ietApply(gTsnDemo.macPorts[i], features->ietEn);
    }

    for (i = 0U; (i < gTsnDemo.numMacPorts) && (cutThruStatus == ENET_SOK); i++)
    {
        cutThruStatus = TsnDemoFeature_cutThruApply(gTsnDemo.macPorts[i],
                                                    features->cutThruEn);
    }

    if ((ietStatus == ENET_SOK) && (cutThruStatus == ENET_SOK))
    {
        /* Re-apply EST whenever its own enabled state changes (off<->on), OR
         * when it's staying on but IET/cut-through changed underneath it -
         * both TsnDemoFeature_hopIncNs() (upstream window/offset staggering)
         * and TsnDemoFeature_buildAdminList()'s guard band depend on ietEn
         * (and hopIncNs also on cutThruEn), so leaving the hardware schedule
         * untouched here would silently run a stale schedule computed under
         * the old IET/cut-through state. Skip only when EST is staying off,
         * or staying on with neither IET nor cut-through changing - that's
         * the only case where re-deriving/re-applying is truly unnecessary. */
        bool estStateChanging = (features->estEn != gTsnDemoCfg.features.estEn);
        bool estScheduleStale = (features->estEn) &&
                                ((features->ietEn != gTsnDemoCfg.features.ietEn) ||
                                 (features->cutThruEn != gTsnDemoCfg.features.cutThruEn));
        bool estNeedsApply = (estStateChanging) || (estScheduleStale);

        if ((estStateChanging) && (features->estEn))
        {
            baseTimeNs = TsnDemoFeature_deriveBaseTime();
            if (baseTimeNs == 0ULL)
            {
                EnetAppUtils_print("EST not enabled: no usable clock\r\n");
                estStatus = ENET_EFAIL;
            }
            else
            {
                EnetAppUtils_print(
                    "EST base time (derived): %llu.%09llu, cycle %u ns\r\n",
                    baseTimeNs / 1000000000ULL, baseTimeNs % 1000000000ULL,
                    TSNDEMO_EST_CYCLE_NS);
            }
        }
        else if ((estStateChanging) && (!features->estEn))
        {
            baseTimeNs = 0ULL;
        }
        else if ((!estStateChanging) && (features->estEn))
        {
            /* EST state not changing, but EST is enabled - reuse existing base time */
            baseTimeNs = gTsnDemo.estBaseTimeNs;
            EnetAppUtils_print(
                "EST: already active, keeping existing base time: %llu.%09llu, cycle %u ns\r\n",
                baseTimeNs / 1000000000ULL, baseTimeNs % 1000000000ULL,
                TSNDEMO_EST_CYCLE_NS);
        }

        /* SET_ADMIN_LIST rejects a non-zero baseTime whenever the port's TAS
         * state is already ENET_TAS_ENABLE (CpswEst_ioctl_handler_ENET_TAS_
         * IOCTL_SET_ADMIN_LIST, cpsw_est_ioctl.c: "AdminBasetime in the
         * future is not supported, must pass 0") - a live/hitless list swap
         * on an already-running port must pass baseTime=0; only a genuine
         * disable->enable transition may carry a real future base time.
         * baseTimeNs itself (bookkeeping/prints/gTsnDemo.estBaseTimeNs
         * below) stays the real value either way - only what's sent to the
         * IOCTL differs. */
        adminBaseTimeNs = estStateChanging ? baseTimeNs : 0ULL;

        /* Apply EST if its enabled state is changing, or if it's staying on
         * but the schedule underneath it (IET/cut-through) changed. */
        if (estNeedsApply)
        {
            for (i = 0U; (i < gTsnDemo.numMacPorts) && (estStatus == ENET_SOK); i++)
            {
                startOffsetNs = 0ULL;
                windowWidthNs = TSNDEMO_EST_DOWNSTREAM_WIDTH_NS;
                if (features->estEn)
                {
                    TsnDemoFeature_computePortWindow(gTsnDemo.macPorts[i], features,
                                                     &startOffsetNs, &windowWidthNs);
                }
                estStatus = TsnDemoFeature_estApply(gTsnDemo.macPorts[i], features->estEn,
                                                    adminBaseTimeNs, startOffsetNs,
                                                    windowWidthNs, features->ietEn);
            }
        }

        if ((estStatus != ENET_SOK) && (features->estEn))
        {
            /*
             * Partial enable failure: the loop above stops at the first
             * failing port, so an earlier port can be left ENABLE while a
             * later one failed. Confirmed on hardware this turns one bad
             * attempt into a cascade of different-looking errors on retry
             * (e.g. SET_ADMIN_LIST EINVALIDPARAMS on an unlinked port,
             * then a different port failing ENOTSUPPORTED next time,
             * because the driver rejects a fresh non-zero base time on a
             * port already ENABLE from the failed attempt). Disable every
             * port unconditionally so a retry starts clean - disabling an
             * already-disabled port is a safe no-op.
             */
            EnetAppUtils_print(
                "EST: enable failed partway through - disabling all "
                "ports to avoid a mixed enabled/disabled state\r\n");
            for (i = 0U; i < gTsnDemo.numMacPorts; i++)
            {
                (void)TsnDemoFeature_estApply(gTsnDemo.macPorts[i], false,
                                              0ULL, 0ULL, 0ULL,
                                              features->ietEn);
            }
        }
    }

    status = (ietStatus != ENET_SOK) ? ietStatus :
             (cutThruStatus != ENET_SOK) ? cutThruStatus : estStatus;

    if (status == ENET_SOK)
    {
        gTsnDemoCfg.features  = *features;
        gTsnDemo.estBaseTimeNs = features->estEn ? baseTimeNs : 0ULL;
        gTsnDemo.estCycleTimeNs = features->estEn ? TSNDEMO_EST_CYCLE_NS : 0U;
        EnetAppUtils_print("Features applied: EST=%s IET=%s CUTTHRU=%s\r\n",
                           features->estEn ? "on" : "off",
                           features->ietEn ? "on" : "off",
                           features->cutThruEn ? "on" : "off");

        if (features->estEn)
        {
            /* Don't let the operator start a run before the schedule's
             * base time is actually reached. */
            ClockP_usleep(TSNDEMO_EST_BASETIME_MARGIN_NS / 1000ULL);
            EnetAppUtils_print(
                "EST: schedule active as of now (waited %llu ms for base "
                "time)\r\n", TSNDEMO_EST_BASETIME_MARGIN_NS / 1000000ULL);
        }
    }
    else
    {
        EnetAppUtils_print(
            "Features NOT applied due to error (status=%d). "
            "Current state: EST=%s IET=%s CUTTHRU=%s\r\n",
            status,
            gTsnDemoCfg.features.estEn ? "on" : "off",
            gTsnDemoCfg.features.ietEn ? "on" : "off",
            gTsnDemoCfg.features.cutThruEn ? "on" : "off");
    }
    return status;
}

uint8_t TsnDemoFeature_toBits(const TsnDemo_FeatureSet *features)
{
    return (uint8_t)((features->estEn ? TSNDEMO_FEATBIT_EST : 0U) |
                     (features->ietEn ? TSNDEMO_FEATBIT_IET : 0U) |
                     (features->cutThruEn ? TSNDEMO_FEATBIT_CUTTHRU : 0U));
}

void TsnDemoFeature_printSchedule(const TsnDemo_FeatureSet *features)
{
    EnetTas_ControlList list;
    uint32_t start = 0U;
    uint32_t i;
    uint8_t mask;
    uint32_t dur;

    /* Generic/reference shape only (hop index 0, unstaggered) - actual
     * per-port schedules now vary by chain position/role
     * (TsnDemoFeature_computePortWindow); see the live per-port readback
     * below for what's really running on each port. */
    TsnDemoFeature_buildAdminList(&list, 0ULL, 0ULL,
                                  TSNDEMO_EST_DOWNSTREAM_WIDTH_NS,
                                  features->ietEn);

    EnetAppUtils_print(
        "EST schedule, reference shape at hop index 0 (cycle %u ns, "
        "guard %u ns) - actual per-port schedules are staggered by chain "
        "position, see live readback below:\r\n",
        (uint32_t)list.cycleTime, TsnDemoFeature_guardBandNs(features));
    for (i = 0U; i < list.listLength; i++)
    {
        mask = list.gateCmdList[i].gateStateMask;
        dur  = list.gateCmdList[i].timeInterval;

        EnetAppUtils_print(
            "  [%u] gates[7..0]=%c%c%c%c%c%c%c%c  %7u..%7u ns\r\n", i,
            (mask & 0x80U) ? 'o' : 'C', (mask & 0x40U) ? 'o' : 'C',
            (mask & 0x20U) ? 'o' : 'C', (mask & 0x10U) ? 'o' : 'C',
            (mask & 0x08U) ? 'o' : 'C', (mask & 0x04U) ? 'o' : 'C',
            (mask & 0x02U) ? 'o' : 'C', (mask & 0x01U) ? 'o' : 'C',
            start, start + dur - 1U);
        start += dur;
    }
    EnetAppUtils_print("  PCP %u (express) -> window [0] (protected); "
                       "all other PCPs (incl. %u, bulk) -> window [1]; "
                       "PCP 7 (gPTP) -> all windows\r\n",
                       TSNDEMO_PCP_EXPRESS, TSNDEMO_PCP_BULK);

    /* Everything above is the software's own intended recipe, rebuilt
     * locally - it says nothing about what the hardware is actually
     * running. Read that back for real, per port, via the TAS module's own
     * GET IOCTLs (never before called anywhere in this demo). */
    TsnDemoFeature_printOperList(features);
}

bool TsnDemoFeature_getLiveOperList(Enet_MacPort macPort,
                                   EnetTas_ControlList *operList)
{
    Enet_IoctlPrms prms;
    EnetTas_GenericInArgs genArgs;
    int32_t status;

    genArgs.macPort = macPort;
    memset(operList, 0, sizeof(*operList));
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &genArgs, operList);
    ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
               ENET_TAS_IOCTL_GET_OPER_LIST, &prms, status);
    return (status == ENET_SOK);
}

void TsnDemoFeature_printOperList(const TsnDemo_FeatureSet *features)
{
    Enet_IoctlPrms prms;
    EnetTas_GenericInArgs genArgs;
    EnetTas_TasState tasState;
    EnetTas_ControlList operList;
    uint64_t baseTimes[TSNDEMO_MAX_MAC_PORTS];
    bool haveBaseTime[TSNDEMO_MAX_MAC_PORTS] = {false};
    uint32_t start;
    uint32_t i, j;
    uint8_t mask;
    uint32_t dur;
    int32_t status;

    for (i = 0U; i < gTsnDemo.numMacPorts; i++)
    {
        genArgs.macPort = gTsnDemo.macPorts[i];

        ENET_IOCTL_SET_INOUT_ARGS(&prms, &genArgs, &tasState);
        ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                   ENET_TAS_IOCTL_GET_STATE, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print(
                "Live TAS state port %u: GET_STATE failed: %d\r\n",
                ENET_MACPORT_ID(gTsnDemo.macPorts[i]), status);
            continue;
        }

        EnetAppUtils_print("Live TAS state port %u: %s\r\n",
                           ENET_MACPORT_ID(gTsnDemo.macPorts[i]),
                           (tasState == ENET_TAS_ENABLE) ? "ENABLE" :
                           (tasState == ENET_TAS_DISABLE) ? "DISABLE" :
                           "RESET");

        if (!TsnDemoFeature_getLiveOperList(gTsnDemo.macPorts[i], &operList))
        {
            EnetAppUtils_print(
                "Live TAS oper list port %u: GET_OPER_LIST failed\r\n",
                ENET_MACPORT_ID(gTsnDemo.macPorts[i]));
            continue;
        }

        baseTimes[i] = operList.baseTime;
        haveBaseTime[i] = true;

        EnetAppUtils_print(
            "Live TAS oper list port %u: baseTime=%llu.%09llu cycle=%llu ns\r\n",
            ENET_MACPORT_ID(gTsnDemo.macPorts[i]),
            operList.baseTime / 1000000000ULL, operList.baseTime % 1000000000ULL,
            operList.cycleTime);
        start = 0U;
        for (j = 0U; j < operList.listLength; j++)
        {
            mask = operList.gateCmdList[j].gateStateMask;
            dur  = operList.gateCmdList[j].timeInterval;

            EnetAppUtils_print(
                "  [%u] gates[7..0]=%c%c%c%c%c%c%c%c  %7u..%7u ns\r\n", j,
                (mask & 0x80U) ? 'o' : 'C', (mask & 0x40U) ? 'o' : 'C',
                (mask & 0x20U) ? 'o' : 'C', (mask & 0x10U) ? 'o' : 'C',
                (mask & 0x08U) ? 'o' : 'C', (mask & 0x04U) ? 'o' : 'C',
                (mask & 0x02U) ? 'o' : 'C', (mask & 0x01U) ? 'o' : 'C',
                start, start + dur - 1U);
            start += dur;
        }

        if (features != NULL)
        {
            uint64_t startOffsetNs, windowWidthNs;
            bool isDownstream = (gTsnDemo.macPorts[i] == gTsnDemoCfg.downstreamPort);

            TsnDemoFeature_computePortWindow(gTsnDemo.macPorts[i], features,
                                             &startOffsetNs, &windowWidthNs);
            EnetAppUtils_print(
                "  role=%s (chain-position stagger) computed "
                "startOffset=%llu ns windowWidth=%llu ns\r\n",
                isDownstream ? "downstream" : "upstream",
                startOffsetNs, windowWidthNs);
        }
    }

    /* The stagger itself lives entirely in each port's baseTime (the gate
     * list shape above is always printed relative to that port's own
     * baseTime, so it can't show drift) - print the raw baseTime delta
     * between the two live ports directly, so upstream-vs-downstream drift
     * doesn't need to be worked out by hand from two absolute epoch
     * timestamps. */
    if ((gTsnDemo.numMacPorts == 2U) && (haveBaseTime[0]) && (haveBaseTime[1]))
    {
        int64_t driftNs = (int64_t)baseTimes[1] - (int64_t)baseTimes[0];

        EnetAppUtils_print(
            "Live baseTime drift, port %u vs port %u: %lld ns\r\n",
            ENET_MACPORT_ID(gTsnDemo.macPorts[1]),
            ENET_MACPORT_ID(gTsnDemo.macPorts[0]), (long long)driftNs);
    }
}
