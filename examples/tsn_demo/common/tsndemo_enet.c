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
 * \file  tsndemo_enet.c
 *
 * \brief TSN demo - Enet driver bring-up: driver open, core attach, ALE and
 *        VLAN configuration, TSN stack (uniconf + gPTP) start, gPTP state
 *        query and link speed check.
 *
 *        Modeled on examples/tsn/enetapp_cpsw.c and
 *        examples/ether_ring/enetapp_cpsw.c; EST/IET/cut-through live in
 *        tsndemo_feature.c, the datapath in tsndemo_dataflow.c.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "tsndemo.h"

#include <tsn_combase/combase.h>
#include <tsn_combase/tilld/cb_lld_ethernet.h>
#include <tsn_gptp/gptpclock.h>
#include <tsn_gptp/gptpmasterclock.h>

#include "ti_enet_config.h"
#include "ti_enet_open_close.h"

/* Shared TSN framework (examples/tsn) */
#include "tsninit.h"
#include "debug_log.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TSNDEMO_LINK_SPEED_MBPS         (1000U)

#define ETH_P_1588_LOCAL                (0x88F7U)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

TsnDemo_Ctx gTsnDemo;

TsnDemo_Cfg gTsnDemoCfg =
{
    .nodeId            = 0U,
    .numNodes          = 2U,
    /* Overwritten during boot prompt for both roles - talker sets this to
     * ENET_MAC_PORT_2 unconditionally (TsnDemoTalker_promptNodeCfg),
     * listener derives it from nodeId/numNodes (TsnDemoListener_
     * promptNodeCfg) - before EST/IET can be enabled. Explicit non-zero-
     * port default since ENET_MAC_PORT_1 == 0 - a plain zero-init would
     * silently look like a real port choice. */
    .downstreamPort    = ENET_MAC_PORT_INV,
    .features          = { .estEn = false, .ietEn = false, .cutThruEn = false },
    .mode              = TSNDEMO_MODE_BASELINE,
    .stream =
    {
        [0] = { .kind = TSNDEMO_STREAM_EXPRESS, .payloadLen = 256U,
                .periodNs = TSNDEMO_EST_CYCLE_NS, .burstCount = 1U,
                .pcp = TSNDEMO_PCP_EXPRESS, .vlanId = TSNDEMO_VLAN_ID,
                .dstNodeId = 1U },
        [1] = { .kind = TSNDEMO_STREAM_BULK, .payloadLen = 1450U,
                .periodNs = 0U, .burstCount = 1U,
                .pcp = TSNDEMO_PCP_BULK, .vlanId = TSNDEMO_VLAN_ID,
                .dstNodeId = 1U },
        [2] = { .kind = TSNDEMO_STREAM_OFF },
        [3] = { .kind = TSNDEMO_STREAM_OFF },
    },
    .runSampleCount    = 10000U,
    .warmupSampleCount = 1000U,
    .echoTimeoutUs     = 1000U,
};

/* netdev names shared with the gPTP task; must stay in global memory */
static char gNetDevices[TSNDEMO_MAX_MAC_PORTS][CB_MAX_NETDEVNAME] = {0};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

#define TSNDEMO_LOG_BUFSIZE (512U)
static void TsnDemoEnet_consolePrint(const char *pcString, ...)
{
    va_list args;
    char buffer[TSNDEMO_LOG_BUFSIZE];

    va_start(args, pcString);
    vsnprintf(buffer, sizeof(buffer), pcString, args);
    va_end(args);

    DebugP_log("%s", buffer);
}

/*
 * Route gPTP (EtherType 0x88F7) to the dedicated PTP DMA channel pair so the
 * demo traffic on CH0 cannot head-of-line block sync messages.
 */
static int TsnDemoEnet_lldCfgUpdateCb(cb_socket_lldcfg_update_t *update_cfg)
{
    if (update_cfg->proto == ETH_P_1588_LOCAL)
    {
        update_cfg->numRxChannels = 1;
        update_cfg->dmaTxChId     = ENET_DMA_TX_CH_PTP;
        update_cfg->dmaRxChId[0]  = ENET_DMA_RX_CH_PTP;
        update_cfg->nTxPkts       = ENET_DMA_TX_CH_PTP_NUM_PKTS;
        update_cfg->nRxPkts[0]    = ENET_DMA_RX_CH_PTP_NUM_PKTS;
        update_cfg->pktSize       = ENET_MEM_LARGE_POOL_PKT_SIZE;
    }
    return 0;
}

static int32_t TsnDemoEnet_addMcastEntry(const uint8_t *addr, uint16_t vlanId,
                                         uint32_t portMask)
{
    Enet_IoctlPrms prms;
    CpswAle_SetMcastEntryInArgs inArgs;
    uint32_t outArgs;
    int32_t status;

    memset(&inArgs, 0, sizeof(inArgs));
    memcpy(&inArgs.addr.addr[0U], addr, ENET_MAC_ADDR_LEN);
    inArgs.addr.vlanId   = vlanId;
    inArgs.info.super    = false;
    inArgs.info.numIgnBits = 0U;
    inArgs.info.fwdState = CPSW_ALE_FWDSTLVL_FWD;
    inArgs.info.portMask = portMask;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &outArgs);
    ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId, CPSW_ALE_IOCTL_ADD_MCAST,
               &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("addMcastEntry failed: %d\r\n", status);
    }
    return status;
}

/*
 * secure=true locks this address to portNum: the ALE drops any frame whose
 * source address matches but whose ingress port doesn't (cpsw_ale.h,
 * CpswAle_UcastEntryInfo.secure). Used to lock each node's own MAC address
 * to its host port, so a frame that loops back around the chain carrying
 * that address (e.g. bulk traffic bounced off a loopback-enabled dangling
 * port further down the chain) is hardware-dropped the instant it
 * re-enters, instead of corrupting the ALE's forwarding entry via
 * re-learning or being re-flooded.
 */
static int32_t TsnDemoEnet_addUcastEntry(const uint8_t *addr, uint16_t vlanId,
                                         uint32_t portNum, bool secure,
                                         bool ageable)
{
    Enet_IoctlPrms prms;
    CpswAle_SetUcastEntryInArgs inArgs;
    uint32_t outArgs;
    int32_t status;

    memset(&inArgs, 0, sizeof(inArgs));
    memcpy(&inArgs.addr.addr[0U], addr, ENET_MAC_ADDR_LEN);
    inArgs.addr.vlanId  = vlanId;
    inArgs.info.portNum = portNum;
    inArgs.info.blocked = false;
    inArgs.info.secure  = secure;
    inArgs.info.super   = false;
    inArgs.info.ageable = ageable;
    inArgs.info.trunk   = false;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &outArgs);
    ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId, CPSW_ALE_IOCTL_ADD_UCAST,
               &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("addUcastEntry failed: %d\r\n", status);
    }
    return status;
}

static int32_t TsnDemoEnet_addVlanEntry(uint16_t vlanId,
                                        uint32_t memberMask,
                                        uint32_t unregMcastFloodMask,
                                        uint32_t forceUntaggedEgressMask)
{
    Enet_IoctlPrms prms;
    CpswAle_VlanEntryInfo inArgs;
    uint32_t outArgs;
    int32_t status;

    memset(&inArgs, 0, sizeof(inArgs));
    inArgs.vlanIdInfo.vlanId       = vlanId;
    inArgs.vlanIdInfo.tagType      = ENET_VLAN_TAG_TYPE_INNER;
    inArgs.vlanMemberList          = memberMask;
    inArgs.unregMcastFloodMask     = unregMcastFloodMask;
    inArgs.regMcastFloodMask       = memberMask;
    inArgs.forceUntaggedEgressMask = forceUntaggedEgressMask;
    inArgs.noLearnMask             = 0U;
    inArgs.vidIngressCheck         = false;
    inArgs.limitIPNxtHdr           = false;
    inArgs.disallowIPFrag          = false;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &outArgs);
    ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId, CPSW_ALE_IOCTL_ADD_VLAN,
               &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("addVlanEntry(%u) failed: %d\r\n", vlanId, status);
    }
    return status;
}

/*
 * ALE/VLAN plan:
 *  - Demo VLAN: member = all ports; unreg-mcast flood = MAC ports only, so
 *    transit demo traffic is switched in hardware and never hits the CPU.
 *  - Own probe mcast (01:00:5E:7F:FE:<nodeId>): host port only.
 *  - Talker echo mcast (01:00:5E:7F:FE:00): host port only on the talker.
 *  - gPTP peer mcast (01:80:C2:00:00:0E): host port only (per-hop protocol).
 *  - VLAN 0 with forceUntaggedEgress on all ports, for untagged gPTP.
 *  - Own MAC address: secure unicast entry locked to the host port (see
 *    TsnDemoEnet_addUcastEntry above) - applies to every node, talker and
 *    every listener alike, since this function is shared by both roles.
 */
static int32_t TsnDemoEnet_setupAle(void)
{
    const uint8_t ptpMcast[ENET_MAC_ADDR_LEN] =
        { 0x01U, 0x80U, 0xC2U, 0x00U, 0x00U, 0x0EU };
    const uint8_t ownMcast[ENET_MAC_ADDR_LEN] =
        TSNDEMO_NODE_MCAST(gTsnDemoCfg.nodeId);
    int32_t status;

    /* All ports = host(0) + MAC ports; CPSW3G: 0x7 */
    status = TsnDemoEnet_addVlanEntry(TSNDEMO_VLAN_ID,
                                      0x7U /* member: all */,
                                      0x6U /* flood: MAC ports only */,
                                      0x0U);
    if (status == ENET_SOK)
    {
        /* default VLAN 0, untagged egress for PTP */
        status = TsnDemoEnet_addVlanEntry(0U, 0x7U, 0x7U, 0x7U);
    }
    if (status == ENET_SOK)
    {
        status = TsnDemoEnet_addMcastEntry(ownMcast, TSNDEMO_VLAN_ID,
                                           CPSW_ALE_HOST_PORT_MASK);
    }
    if (status == ENET_SOK)
    {
        status = TsnDemoEnet_addMcastEntry(ptpMcast, 0U,
                                           CPSW_ALE_HOST_PORT_MASK);
    }
    if (status == ENET_SOK)
    {
        status = TsnDemoEnet_addUcastEntry(gTsnDemo.macAddr, 0U,
                                           CPSW_ALE_HOST_PORT_NUM,
                                           true /* secure */,
                                           false /* ageable */);
    }
    return status;
}

int32_t TsnDemoEnet_open(void)
{
    EnetPer_AttachCoreOutArgs attachCoreOutArgs;
    EnetApp_HandleInfo handleInfo;
    int32_t status;

    memset(&gTsnDemo, 0, sizeof(gTsnDemo));

    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &gTsnDemo.enetType,
                            &gTsnDemo.instId);
    EnetApp_getEnetInstMacInfo(gTsnDemo.enetType, gTsnDemo.instId,
                               gTsnDemo.macPorts, &gTsnDemo.numMacPorts);

    gTsnDemo.coreId = EnetSoc_getCoreId();
    EnetQueue_initQ(&gTsnDemo.txFreePktInfoQ);
    EnetQueue_initQ(&gTsnDemo.bulkFreePktInfoQ);

    EnetAppUtils_enableClocks(gTsnDemo.enetType, gTsnDemo.instId);
    EnetApp_driverInit();
    status = EnetApp_driverOpen(gTsnDemo.enetType, gTsnDemo.instId);
    EnetAppUtils_assert(status == ENET_SOK);

    EnetApp_acquireHandleInfo(gTsnDemo.enetType, gTsnDemo.instId, &handleInfo);
    gTsnDemo.hEnet = handleInfo.hEnet;

    EnetApp_coreAttach(gTsnDemo.enetType, gTsnDemo.instId, gTsnDemo.coreId,
                       &attachCoreOutArgs);
    gTsnDemo.coreKey = attachCoreOutArgs.coreKey;

    /* DMA channels + packet pools + ALE entries */
    status = TsnDemoFlow_open();
    if (status == ENET_SOK)
    {
        status = TsnDemoEnet_setupAle();
    }
    return status;
}

int32_t TsnDemoEnet_initTsn(void)
{
    lld_ethdev_t ethdevs[MAX_NUMBER_ENET_DEVS] = {0};
    AppTsnCfg_t appCfg =
    {
        .consoleOutCb = TsnDemoEnet_consolePrint,
    };
    int32_t res = 0;
    int i;

    for (i = 0; i < gTsnDemo.numMacPorts; i++)
    {
        snprintf(&gNetDevices[i][0], CB_MAX_NETDEVNAME, "tilld%d", i);
        appCfg.netdevs[i]  = &gNetDevices[i][0];
        ethdevs[i].netdev  = gNetDevices[i];
        ethdevs[i].macport = gTsnDemo.macPorts[i];
        if (i == 0)
        {
            /* tilld0 reuses the host MAC; other netdevs allocate their own */
            memcpy(ethdevs[i].srcmac, gTsnDemo.macAddr, ENET_MAC_ADDR_LEN);
        }
    }
    appCfg.netdevs[i] = NULL;

    if (EnetApp_initTsnByCfg(&appCfg) < 0)
    {
        EnetAppUtils_print("Failed to init TSN stack!\r\n");
        res = -1;
    }
    if ((res == 0) &&
        (cb_lld_init_devs_table(ethdevs, i, gTsnDemo.enetType,
                                gTsnDemo.instId,
                                ENET_SYSCFG_TIMESTAMP_SOURCE) < 0))
    {
        EnetAppUtils_print("Failed to init devs table!\r\n");
        res = -1;
    }
    if (res == 0)
    {
        cb_socket_set_lldcfg_update_cb(TsnDemoEnet_lldCfgUpdateCb);
        if (EnetApp_startTsn() < 0)
        {
            EnetAppUtils_print("Failed to start TSN stack!\r\n");
            res = -1;
        }
    }
    if (res == 0)
    {
        EnetAppUtils_print("TSN app start done!\r\n");
    }
    return res;
}

bool TsnDemoEnet_isClockStable(void)
{
    return (gptpclock_get_gmsync(TSNDEMO_GPTP_INSTANCE, TSNDEMO_GPTP_DOMAIN)
            == GMSYNC_SYNC_STABLE);
}

void TsnDemoEnet_waitClockStable(bool requireSlaveRole)
{
    uint32_t waitedMs = 0U;
    bool weAreGm;

#ifdef TSNDEMO_SKIP_SYNC_WAIT
    /*
     * Bring-up escape hatch (see the bring-up guide, stage B1-B4). Lets the
     * datapath be exercised against a PC with packETH/Wireshark when there is
     * no gPTP peer at all. Timestamps are NOT on a common time base in this
     * mode, so any latency figure produced is meaningless - never enable this
     * for a measurement build.
     */
    EnetAppUtils_print("*** TSNDEMO_SKIP_SYNC_WAIT: gPTP gate bypassed, "
                       "latency results are INVALID ***\r\n");
#else
    while (true)
    {
        weAreGm = gptpclock_we_are_gm(TSNDEMO_GPTP_INSTANCE, TSNDEMO_GPTP_DOMAIN);
        if ((TsnDemoEnet_isClockStable()) && ((!requireSlaveRole) || (!weAreGm)))
        {
            break;
        }
        ClockP_usleep(500000U);
        waitedMs += 500U;
        if ((waitedMs % 5000U) == 0U)
        {
            /*
             * Every gPTP device self-elects as grandmaster before hearing
             * any peer's Announce, so gmsync can read SYNC_STABLE at boot
             * with zero real synchronization. A slave node must wait until
             * it has genuinely ceded GM role to a real remote master, not
             * just until its own local flag says so.
             */
            EnetAppUtils_print(
                "Waiting for gPTP sync-stable (%u s)%s...\r\n",
                waitedMs / 1000U,
                (requireSlaveRole && weAreGm) ?
                " - still self-elected grandmaster, no master heard yet" : "");
        }
    }
    EnetAppUtils_print("gPTP sync-stable.\r\n");
    TsnDemoEnet_printClockDiag();
#endif
}

void TsnDemoEnet_printClockDiag(void)
{
    bool weAreGm = gptpclock_we_are_gm(TSNDEMO_GPTP_INSTANCE, TSNDEMO_GPTP_DOMAIN);

    EnetAppUtils_print(
        " gPTP role      : %s\r\n",
        weAreGm ? "GRANDMASTER (sync-stable here is local-clock-only - "
                  "does NOT confirm any listener has locked to us)"
                : "slave (sync-stable here reflects measured offset/rate "
                  "convergence to the master)");
    /* Vendor debug dump (offset/adjrate/gmsync per domain). UBL_WARN is used
     * deliberately: the demo's default TSNAPP_LOGLEVEL gates the "gptp" log
     * category at WARN, so this level is guaranteed to actually print
     * without needing to raise gptp's log verbosity build-wide. */
    gptpclock_print_clkpara(TSNDEMO_GPTP_INSTANCE, UBL_WARN);
}

int32_t TsnDemoEnet_waitUntilTs64(uint64_t targetNs)
{
    /* vclose=1000ns: once within 1us of targetNs, stop sleeping rather than
     * risk a final short nanosleep overshooting past it. toofar=10ms: a
     * caller asking to wait longer than that for an EST-probe timing target
     * almost certainly computed it wrong (targets here are only a few EST
     * cycles - low hundreds of us - ahead of now). */
    return gptpmasterclock_wait_until_ts64((int64_t)targetNs, 1000LL, 10000000LL);
}

void TsnDemoEnet_waitForLinkUp(Enet_MacPort macPort)
{
    Enet_IoctlPrms prms;
    uint32_t waitedMs = 0U;
    bool linked = false;
    int32_t status;

    while (!linked)
    {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &linked);
        ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                   ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
        if ((status == ENET_SOK) && (linked))
        {
            break;
        }

        ClockP_usleep(500000U);
        waitedMs += 500U;
        if ((waitedMs % 5000U) == 0U)
        {
            EnetAppUtils_print("Waiting for port %u link up (%u s)...\r\n",
                               ENET_MACPORT_ID(macPort), waitedMs / 1000U);
        }
    }
    EnetAppUtils_print("Port %u link up.\r\n", ENET_MACPORT_ID(macPort));
}

void TsnDemoEnet_waitForAnyLinkUp(void)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    uint32_t waitedMs = 0U;
    bool linked;
    int32_t status;
    uint32_t i;
    bool found = false;

    while (!found)
    {
        for (i = 0U; (i < gTsnDemo.numMacPorts) && (!found); i++)
        {
            linked  = false;
            macPort = gTsnDemo.macPorts[i];

            ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &linked);
            ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                       ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
            if ((status == ENET_SOK) && (linked))
            {
                EnetAppUtils_print("Port %u link up.\r\n",
                                   ENET_MACPORT_ID(macPort));
                found = true;
            }
        }

        if (!found)
        {
            ClockP_usleep(500000U);
            waitedMs += 500U;
            if ((waitedMs % 5000U) == 0U)
            {
                EnetAppUtils_print("Waiting for any port link up (%u s)...\r\n",
                                   waitedMs / 1000U);
            }
        }
    }
}

bool TsnDemoEnet_checkLinkSpeed(void)
{
    Enet_IoctlPrms prms;
    EnetMacPort_GenericInArgs inArgs;
    EnetMacPort_LinkCfg linkCfg;
    Enet_MacPort macPort;
    bool ok = true;
    bool linked;
    int32_t status;
    uint32_t mbps;
    uint32_t i;

    for (i = 0U; i < gTsnDemo.numMacPorts; i++)
    {
        linked  = false;
        macPort = gTsnDemo.macPorts[i];

        ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &linked);
        ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                   ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
        if ((status != ENET_SOK) || (!linked))
        {
            /* Last node in a line has one unconnected port - not an error */
            continue;
        }

        inArgs.macPort = macPort;
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &linkCfg);
        ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                   ENET_MACPORT_IOCTL_GET_LINK_CFG, &prms, status);
        if (status == ENET_SOK)
        {
            mbps = (linkCfg.speed == ENET_SPEED_1GBIT) ? 1000U :
                   (linkCfg.speed == ENET_SPEED_100MBIT) ? 100U : 10U;
            EnetAppUtils_print("Port %u link speed: %u Mbps\r\n",
                               ENET_MACPORT_ID(macPort), mbps);
            if (mbps != TSNDEMO_LINK_SPEED_MBPS)
            {
                EnetAppUtils_print(
                    "ERROR: EST schedule assumes %u Mbps\r\n",
                    TSNDEMO_LINK_SPEED_MBPS);
                ok = false;
            }
        }
    }
    return ok;
}

Enet_MacPort TsnDemoEnet_findLinkedPort(void)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    Enet_MacPort found = ENET_MAC_PORT_INV;
    bool linked;
    int32_t status;
    uint32_t linkedCount = 0U;
    uint32_t i;

    for (i = 0U; i < gTsnDemo.numMacPorts; i++)
    {
        linked  = false;
        macPort = gTsnDemo.macPorts[i];

        ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &linked);
        ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                   ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
        if ((status == ENET_SOK) && (linked))
        {
            found = macPort;
            linkedCount++;
        }
    }
    /* Ambiguous on a node with more than one link up (e.g. a middle node
     * in the chain) - callers only care about the last node, which has
     * exactly one. */
    return (linkedCount == 1U) ? found : ENET_MAC_PORT_INV;
}

/*
 * Reverse/upstream bulk contention via last-node loopback - implemented as
 * an ALE inter-VLAN route instead of an earlier attempt that put the whole
 * port into MAC loopback via ENET_PER_IOCTL_CLOSE/OPEN_PORT_LINK, which
 * disturbed other port features (link/PHY state affects everything
 * transiting the port, not just bulk). This version is a pure ALE
 * classify+redirect rule scoped to one EtherType - it never touches the
 * port's link, PHY, or loopback state.
 *
 * Mechanics (see include/per/cpsw.h,
 * CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_UNI_EGRESS): an inter-VLAN route only
 * fires for a packet whose *normal* ALE forwarding decision already sends
 * it to the host port - it hijacks that host-bound copy and redirects it
 * to the given egress port instead. TSNDEMO_BULK_MCAST is unregistered
 * everywhere (see tsndemo.h), so its normal decision is "flood to MAC
 * ports, never host", which would never trigger this rule. So on this
 * node only, we first register it as a real ALE multicast entry with
 * host-only membership, changing this node's baseline decision to "host
 * port" - which the inter-VLAN rule then intercepts and sends back out
 * upstreamPort (the port it just arrived on), so it never reaches this
 * node's host port or CPU; it just bounces.
 */
int32_t TsnDemoEnet_setBulkReflector(Enet_MacPort upstreamPort, bool enable)
{
    const uint8_t bulkMcast[ENET_MAC_ADDR_LEN] = TSNDEMO_BULK_MCAST;
    Enet_IoctlPrms prms;
    int32_t status;

    if (enable)
    {
        status = TsnDemoEnet_addMcastEntry(bulkMcast, TSNDEMO_VLAN_ID,
                                           CPSW_ALE_HOST_PORT_MASK);
        if (status == ENET_SOK)
        {
            Cpsw_SetInterVlanRouteUniEgressInArgs inArgs;
            Cpsw_SetInterVlanRouteUniEgressOutArgs outArgs;

            memset(&inArgs, 0, sizeof(inArgs));
            inArgs.inPktMatchCfg.packetMatchEnMask =
                CPSW_INTERVLAN_INGRESSPKT_MATCH_PORT |
                CPSW_INTERVLAN_INGRESSPKT_MATCH_ETHERTYPE;
            inArgs.inPktMatchCfg.ttlCheckEn  = false;
            inArgs.inPktMatchCfg.ingressPort = upstreamPort;
            inArgs.inPktMatchCfg.vlanId      = TSNDEMO_VLAN_ID;
            inArgs.inPktMatchCfg.etherType   = TSNDEMO_ETHERTYPE_BULK;

            inArgs.egressCfg.egressPort              = upstreamPort;
            inArgs.egressCfg.outPktModCfg.replaceDASA = false;
            inArgs.egressCfg.outPktModCfg.forceUntaggedEgress = false;
            inArgs.egressCfg.outPktModCfg.decrementTTL = false;
            inArgs.egressCfg.outPktModCfg.vlanId       = TSNDEMO_VLAN_ID;

            ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &outArgs);
            ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                       CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_UNI_EGRESS,
                       &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print(
                    "Bulk reflector: SET_INTERVLAN_ROUTE_UNI_EGRESS port "
                    "%u failed: %d\r\n", ENET_MACPORT_ID(upstreamPort),
                    status);
            }
        }
    }
    else
    {
        Cpsw_ClearInterVlanRouteUniEgressInArgs inArgs;

        memset(&inArgs, 0, sizeof(inArgs));
        inArgs.inPktMatchCfg.packetMatchEnMask =
            CPSW_INTERVLAN_INGRESSPKT_MATCH_PORT |
            CPSW_INTERVLAN_INGRESSPKT_MATCH_ETHERTYPE;
        inArgs.inPktMatchCfg.ingressPort = upstreamPort;
        inArgs.inPktMatchCfg.vlanId      = TSNDEMO_VLAN_ID;
        inArgs.inPktMatchCfg.etherType   = TSNDEMO_ETHERTYPE_BULK;
        /* Mirror the enable-side egress config exactly, in case the clear
         * lookup keys off it too, not just inPktMatchCfg. */
        inArgs.egressCfg.egressPort               = upstreamPort;
        inArgs.egressCfg.outPktModCfg.replaceDASA  = false;
        inArgs.egressCfg.outPktModCfg.forceUntaggedEgress = false;
        inArgs.egressCfg.outPktModCfg.decrementTTL = false;
        inArgs.egressCfg.outPktModCfg.vlanId       = TSNDEMO_VLAN_ID;
        inArgs.delAleEntryMask           = CPSW_INTERVLAN_INGRESSPKT_MATCH_ETHERTYPE;

        ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
        ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                   CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_UNI_EGRESS,
                   &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print(
                "Bulk reflector: CLEAR_INTERVLAN_ROUTE_UNI_EGRESS port %u "
                "failed: %d\r\n", ENET_MACPORT_ID(upstreamPort), status);
        }

        /* Revert bulk to unregistered - falls back to the VLAN's normal
         * unregMcastFloodMask (MAC ports only, never host) on this node,
         * same as every other node. */
        {
            CpswAle_MacAddrInfo addrInfo;
            uint32_t delStatus;

            memset(&addrInfo, 0, sizeof(addrInfo));
            memcpy(addrInfo.addr, bulkMcast, ENET_MAC_ADDR_LEN);
            addrInfo.vlanId = TSNDEMO_VLAN_ID;

            ENET_IOCTL_SET_IN_ARGS(&prms, &addrInfo);
            ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                       CPSW_ALE_IOCTL_REMOVE_ADDR, &prms, delStatus);
            if (delStatus != ENET_SOK)
            {
                EnetAppUtils_print(
                    "Bulk reflector: REMOVE_ADDR failed: %d\r\n", delStatus);
            }
        }
    }

    if (status == ENET_SOK)
    {
        EnetAppUtils_print("Bulk reflector on port %u %s\r\n",
                           ENET_MACPORT_ID(upstreamPort),
                           enable ? "ENABLED" : "disabled");
    }
    return status;
}

/*
 * CPSW_3G is an "Ng" part - the 9G printers and CpswStats_*_Ng types are the
 * correct ones. See examples/enet_layer2_cpsw_switch/V1/l2_cpsw_cfg.c, which
 * uses the same pair. Using the 2G variants here silently prints misaligned
 * fields.
 */
void TsnDemoEnet_printStats(void)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    int32_t status;
    uint32_t i;
    static CpswStats_PortStats portStats;

    ENET_IOCTL_SET_OUT_ARGS(&prms, &portStats);
    ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
               ENET_STATS_IOCTL_GET_HOSTPORT_STATS, &prms, status);
    if (status == ENET_SOK)
    {
        EnetAppUtils_print("\r\n Host port statistics\r\n");
        EnetAppUtils_print("--------------------------------\r\n");
        EnetAppUtils_printHostPortStats9G((CpswStats_HostPort_Ng *)&portStats);
    }

    for (i = 0U; i < gTsnDemo.numMacPorts; i++)
    {
        macPort = gTsnDemo.macPorts[i];

        ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &portStats);
        ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                   ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
        if (status == ENET_SOK)
        {
            EnetAppUtils_print("\r\n Port %u statistics\r\n",
                               ENET_MACPORT_ID(macPort));
            EnetAppUtils_print("--------------------------------\r\n");
            EnetAppUtils_printMacPortStats9G((CpswStats_MacPort_Ng *)&portStats);
        }
    }
}

/* ========================================================================== */
/*            Hooks called from the SysConfig-generated Enet init             */
/* ========================================================================== */

static void TsnDemoEnet_portLinkStatusChangeCb(Enet_MacPort macPort,
                                               bool isLinkUp, void *appArg)
{
    uint64_t baseTimeNs;
    uint64_t startOffsetNs;
    uint64_t windowWidthNs;
    int32_t status;

    EnetAppUtils_print("MAC Port %u: link %s\r\n",
                       ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
    /* Let the TSN stack re-evaluate its ports */
    cb_lld_notify_linkchange();

    /* Hardware drops the EST/TAS gate schedule on link down/up. Reapply it
     * only if the operator had actually enabled EST from the console -
     * IET/cut-through aren't reapplied here (IET's verify handshake needs
     * manual operator involvement across boards, see design notes). */
    if ((isLinkUp) && (gTsnDemoCfg.features.estEn))
    {
        baseTimeNs = TsnDemoFeature_deriveBaseTime();
        if (baseTimeNs != 0ULL)
        {
            EnetAppUtils_print(
                "EST: link up on port %u - reapplying schedule, base time "
                "%llu.%09llu\r\n", ENET_MACPORT_ID(macPort),
                baseTimeNs / 1000000000ULL, baseTimeNs % 1000000000ULL);
            TsnDemoFeature_computePortWindow(macPort, &gTsnDemoCfg.features,
                                            &startOffsetNs, &windowWidthNs);
            status = TsnDemoFeature_estApply(macPort, true, baseTimeNs,
                                             startOffsetNs, windowWidthNs,
                                             gTsnDemoCfg.features.ietEn);
            if (status == ENET_SOK)
            {
                gTsnDemo.estBaseTimeNs  = baseTimeNs;
                gTsnDemo.estCycleTimeNs = TSNDEMO_EST_CYCLE_NS;
            }
        }
        else
        {
            EnetAppUtils_print(
                "EST: link up on port %u - reapply skipped, no usable "
                "clock\r\n", ENET_MACPORT_ID(macPort));
        }
    }
}

void EnetApp_updateCpswInitCfg(Enet_Type enetType, uint32_t instId,
                               Cpsw_Cfg *cpswCfg)
{
    CpswHostPort_Cfg *hostPortCfg = &cpswCfg->hostPortCfg;
    CpswAle_Cfg *aleCfg = &cpswCfg->aleCfg;
    EnetDma_Cfg *dmaCfg = (EnetDma_Cfg *)cpswCfg->dmaCfg;

    hostPortCfg->removeCrc      = true;
    hostPortCfg->padShortPacket = true;
    hostPortCfg->passCrcErrors  = false;

    /*
     * PCP -> hardware switch priority remap. This single line is what makes
     * the demo work: the frame's VLAN PCP selects the EST gate and the IET
     * express/preempt queue.
     */
    hostPortCfg->rxVlanRemapEn     = true;
    hostPortCfg->rxDscpIPv4RemapEn = true;
    hostPortCfg->rxDscpIPv6RemapEn = true;

    cpswCfg->vlanCfg.vlanAware = true;

    aleCfg->modeFlags                          = CPSW_ALE_CFG_MODULE_EN;
    aleCfg->agingCfg.autoAgingEn               = true;
    aleCfg->agingCfg.agingPeriodInMs           = 1000U;
    aleCfg->nwSecCfg.vid0ModeEn                = true;
    aleCfg->vlanCfg.aleVlanAwareMode           = true;
    aleCfg->vlanCfg.cpswVlanAwareMode          = true;
    aleCfg->vlanCfg.unknownUnregMcastFloodMask = CPSW_ALE_ALL_MACPORTS_MASK;
    aleCfg->vlanCfg.unknownRegMcastFloodMask   = CPSW_ALE_ALL_MACPORTS_MASK;
    aleCfg->vlanCfg.unknownVlanMemberListMask  = CPSW_ALE_ALL_MACPORTS_MASK;

#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined(SOC_AM261X)
    /* Per-priority TX channel steering */
    dmaCfg->enChOverrideFlag = true;
#else
    (void)dmaCfg;
#endif

    cpswCfg->portLinkStatusChangeCb    = &TsnDemoEnet_portLinkStatusChangeCb;
    cpswCfg->portLinkStatusChangeCbArg = NULL;
}

void TsnDemoEnet_close(void)
{
    EnetApp_stopTsn();
    EnetApp_deInitTsn();

    TsnDemoFlow_destroyRxTask();
    TsnDemoFlow_close();

    EnetApp_coreDetach(gTsnDemo.enetType, gTsnDemo.instId,
                       gTsnDemo.coreId, gTsnDemo.coreKey);
    EnetApp_releaseHandleInfo(gTsnDemo.enetType, gTsnDemo.instId);
    gTsnDemo.hEnet = NULL;
    EnetAppUtils_disableClocks(gTsnDemo.enetType, gTsnDemo.instId);
}
