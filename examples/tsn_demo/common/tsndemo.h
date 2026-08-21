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
 * \file  tsndemo.h
 *
 * \brief TSN integrated demo (talker/listener) - single shared header.
 *
 *        Wire format, configuration structures, shared context and the
 *        role-agnostic API implemented in examples/tsn_demo/common.
 *        Design doc: docs/networking/enet_lld/tsn_integrated_demo_design.md
 */

#ifndef TSNDEMO_H_
#define TSNDEMO_H_

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include <enet.h>
#include <enet_apputils.h>
#include <enet_appmemutils.h>
#include <enet_ethutils.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/ClockP.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*!
 * Experimental EtherTypes. Probe/echo (real-time) and bulk (background load)
 * are given DIFFERENT EtherTypes on purpose: an ALE policer/thread
 * classifier (CPSW_ALE_POLICER_MATCH_ETHERTYPE + threadId, same mechanism
 * enet_layer2_multi_channel uses for PTP) steers 0x88B5 to its own RX flow
 * serviced at higher task priority, so bulk load never delays a probe echo.
 *
 * EtherType is also the ONLY frame discriminator. There is no payload magic
 * or message-type field: a frame reaching a host port on 0x88B5 is a demo
 * frame by construction, because
 *   - the ALE only delivers a node's OWN probe multicast to its host port,
 *     so a listener can only ever receive PROBEs and the talker can only
 *     ever receive ECHOes, and
 *   - bulk uses an unregistered multicast (TSNDEMO_BULK_MCAST) that never
 *     reaches any host port at all.
 * Direction is implied by role, so nothing in the payload needs to restate
 * it.
 */
#define TSNDEMO_ETHERTYPE               (0x88B5U)   /* probe / echo        */
#define TSNDEMO_ETHERTYPE_BULK          (0x88B6U)   /* bulk / background   */

/*! VLAN used by all demo streams */
#define TSNDEMO_VLAN_ID                 (100U)

/*! PCP of the measured express stream (mapped to the protected EST gate) */
#define TSNDEMO_PCP_EXPRESS             (5U)
/*! PCP of the background/bulk stream */
#define TSNDEMO_PCP_BULK                (1U)
/*! PCP/priority reserved for gPTP (gate for it stays always open) */
#define TSNDEMO_PRI_GPTP                (7U)

/*! Per-node probe multicast address: 01:00:5E:7F:FE:<nodeId>.
 *  Each node registers ONLY its own, to its host port. */
#define TSNDEMO_NODE_MCAST(nodeId) \
    { 0x01U, 0x00U, 0x5EU, 0x7FU, 0xFEU, (uint8_t)(nodeId) }

/*!
 * Bulk/background load multicast address - deliberately in a different
 * range (0xFD) that NO node registers in its ALE table.
 *
 * Consequence: bulk is unregistered multicast everywhere, so it floods
 * MAC-port-to-MAC-port in hardware and is never delivered to any host port.
 * It still traverses every link and still contends for egress bandwidth at
 * every hop - which is the entire purpose of the load stream - but costs
 * zero CPU on every node in the chain.
 *
 * Previously bulk was addressed to the last node's own probe address, which
 * meant that node's CPU received and discarded ~81k frames/s of load in the
 * same RX callback that services probe echoes.
 */
#define TSNDEMO_BULK_MCAST \
    { 0x01U, 0x00U, 0x5EU, 0x7FU, 0xFDU, 0x00U }

#define TSNDEMO_MAX_NODES               (4U)
#define TSNDEMO_MAX_STREAMS             (4U)
#define TSNDEMO_MAX_EXPRESS_STREAMS     (2U)

/*! EST schedule parameters, 1 Gbit link (see design doc 8.2.1) */
#define TSNDEMO_EST_CYCLE_NS            (125000U)
#define TSNDEMO_EST_GUARD_EST_ONLY_NS   (12500U)
#define TSNDEMO_EST_GUARD_EST_IET_NS    (1500U)

/*!
 * Bidirectional, chain-position-staggered EST window0 (express) parameters.
 * A downstream port (facing away from the talker) opens window0 for a
 * fixed TSNDEMO_EST_DOWNSTREAM_WIDTH_NS, shifted later by hopIndex*hopInc.
 * An upstream port (facing the talker, carrying the echo) opens window0
 * starting TSNDEMO_EST_ECHO_MIN_NS after the far end's own arrival time
 * (the software echo-turnaround latency floor), with both its start and
 * width growing by hopInc per hop further back from the far end (echo
 * timing uncertainty compounds hop by hop). hopInc itself is the
 * worst-case per-hop relay/blocking latency - a small fixed value if IET
 * or cut-through is active, else computed from the actual bulk payload
 * size (TsnDemoFeature_hopIncNs()) since without either, a large bulk
 * frame already mid-relay can block the express frame from reaching its
 * egress queue for up to its own store-and-forward time.
 */
#define TSNDEMO_EST_HOPINC_FAST_NS      (2000U)   /* IET or cut-through active */
#define TSNDEMO_EST_DOWNSTREAM_WIDTH_NS (20000U)  /* fixed window0 width, downstream */
#define TSNDEMO_EST_ECHO_MIN_NS         (20000U)  /* SW echo latency floor */
#define TSNDEMO_EST_ECHO_MAX_NS         (30000U)  /* SW echo latency worst case */

/*! Base time derivation: margin ahead of now, rounded up to the next cycle
 *  boundary (design doc 8.2). gPTP-synced nodes need only cover local
 *  apply latency here, not a full run duration. */
#define TSNDEMO_EST_BASETIME_MARGIN_NS  (2000000000ULL)

/*! gPTP instance/domain used by all state queries */
#define TSNDEMO_GPTP_INSTANCE           (0)
#define TSNDEMO_GPTP_DOMAIN             (0)

/*
 * TEMPORARY bring-up override: no gPTP peer is connected yet, so
 * TsnDemoEnet_waitClockStable() would block forever. Remove this once a
 * peer is on the wire, or move it to a build define instead of hardcoding
 * it here - timestamps are NOT on a common time base while this is set.
 */
//#define TSNDEMO_SKIP_SYNC_WAIT

/*
 * Debug Macro to toggle Probe and Bulk traffic
 */
//#define TSNDEMO_ENABLE_DEBUG_MENU

/*! Feature bits, used both in TsnDemo_FeatureSet and on the wire */
#define TSNDEMO_FEATBIT_EST             (1U << 0U)
#define TSNDEMO_FEATBIT_IET             (1U << 1U)
#define TSNDEMO_FEATBIT_CUTTHRU         (1U << 2U)

/*! Per-SoC capability, single readable name for the console menu layer -
 *  aliases the same enet-lld config the IOCTL layer already keys off
 *  (TsnDemoFeature_ietApply()/cutThruApply(), tsndemo_feature.c). AM263x/
 *  AM263Px have neither in hardware; keep menu options for them from ever
 *  being printed/reachable rather than just refusing at runtime. */
#define TSNDEMO_IET_SUPPORTED           ENET_CFG_IS_ON(CPSW_IET_INCL)
#define TSNDEMO_CUTTHRU_SUPPORTED       ENET_CFG_IS_ON(CPSW_CUTTHRU)

/*! VLAN TCI helper */
#define TSNDEMO_VLAN_TCI(pcp, vid) \
    ((uint16_t)((((pcp) & 0x7U) << 13U) | ((vid) & 0xFFFU)))

/* ========================================================================== */
/*                              Wire format                                   */
/* ========================================================================== */

/*!
 * \brief Demo frame payload header. 33 bytes, little-endian, packed.
 *
 * Every field here is READ by the receiver. There is no magic or
 * version field: EtherType 0x88B5 plus the ALE addressing already
 * identify the frame and its direction (see TSNDEMO_ETHERTYPE above), so
 * restating that in the payload would be dead weight.
 *
 * T4 (talker RX of the echo) is never transmitted - it is the hardware RX
 * timestamp captured at the talker.
 */
typedef struct __attribute__((packed)) TsnDemo_Hdr_s
{
    uint32_t seqNum;        /*!< per-stream, monotonic                      */
    uint8_t  streamId;
    uint8_t  dstNodeId;     /*!< node this probe was addressed to; echoed
                                 back unchanged so the talker can tell
                                 multiple listeners' echoes apart            */
    uint8_t  runId;         /*!< increments per run; stale echoes dropped   */
    uint8_t  runMode;       /*!< TsnDemo_RunMode active at TX               */
    uint8_t  featureBits;   /*!< TSNDEMO_FEATBIT_*, sender's active state   */

    uint64_t t1Ns;          /*!< talker TX, software, embedded before submit*/
    uint64_t t2Ns;          /*!< listener RX, hardware rxPktTs              */
    uint64_t t3Ns;          /*!< listener echo TX, software                 */
} TsnDemo_Hdr;

#define TSNDEMO_MIN_PAYLOAD_LEN         ((uint16_t)sizeof(TsnDemo_Hdr))
#define TSNDEMO_MAX_PAYLOAD_LEN         (1496U)

/*! \brief streamId sentinel for talker-side EST gate verification probes
 *  (TsnDemoTalker_estVerify) - lets TsnDemoTalker_rxFrameCb() recognize and
 *  divert these echoes to their own capture, entirely bypassing the normal
 *  per-listener run stats. Every other streamId in normal use is 0, so any
 *  non-zero value is free; this one is deliberately distinct/memorable. */
#define TSNDEMO_STREAM_ID_ESTPROBE      (0xE5U)

/* ========================================================================== */
/*                             Configuration                                  */
/* ========================================================================== */

typedef enum TsnDemo_RunMode_e
{
    TSNDEMO_MODE_BASELINE = 0,       /*!< express only, features off        */
    TSNDEMO_MODE_CONTENTION_NOTSN,   /*!< express + bulk, features off      */
    TSNDEMO_MODE_CONTENTION_TSN,     /*!< express + bulk, features on       */
    TSNDEMO_MODE_COUNT
} TsnDemo_RunMode;

typedef struct TsnDemo_FeatureSet_s
{
    bool estEn;
    bool ietEn;
    bool cutThruEn;
    /* gPTP is always on - everything depends on the shared clock */
} TsnDemo_FeatureSet;

typedef enum TsnDemo_StreamKind_e
{
    TSNDEMO_STREAM_OFF = 0,
    TSNDEMO_STREAM_EXPRESS,          /*!< measured, echoed by dstNodeId     */
    TSNDEMO_STREAM_BULK,             /*!< load only                         */
} TsnDemo_StreamKind;

typedef struct TsnDemo_StreamCfg_s
{
    TsnDemo_StreamKind kind;
    uint16_t payloadLen;             /*!< bytes after the VLAN header       */
    uint32_t periodNs;               /*!< 0 = back-to-back (bulk only)      */
    uint8_t  burstCount;             /*!< frames per period, default 1      */
    uint8_t  pcp;
    uint16_t vlanId;
    /*! Node whose probe multicast address express frames are sent to, i.e.
     *  the node that echoes. Ignored for BULK, which always uses the
     *  unregistered TSNDEMO_BULK_MCAST. */
    uint8_t  dstNodeId;
} TsnDemo_StreamCfg;

typedef struct TsnDemo_Cfg_s
{
    uint8_t            nodeId;       /*!< 0 = talker                        */
    uint8_t            numNodes;
    /*! Listener only: which of gTsnDemo.macPorts[] faces away from the
     *  talker (downstream), for chain-position-staggered EST.
     *  ENET_MAC_PORT_INV (not 0 - ENET_MAC_PORT_1 == 0) if this node is
     *  the last one in the chain (no downstream neighbor) - downstream
     *  scheduling is skipped in that case. Unused on the talker (its own
     *  port is always hop index 0, no lookup needed). */
    Enet_MacPort       downstreamPort;
    TsnDemo_FeatureSet features;     /*!< currently applied feature set     */
    TsnDemo_RunMode    mode;
    TsnDemo_StreamCfg  stream[TSNDEMO_MAX_STREAMS];
    uint32_t           runSampleCount;
    uint32_t           warmupSampleCount;
    uint32_t           echoTimeoutUs;
} TsnDemo_Cfg;

/* ========================================================================== */
/*                            Shared context                                  */
/* ========================================================================== */

/*!
 * \brief Callback invoked by an RX worker for every valid demo frame
 *        (EtherType + magic verified) addressed to this node.
 *
 * \param pktInfo  DMA packet (tsInfo.rxPktTs = hardware ingress timestamp).
 *                 Owned by the RX worker; the callback must not retain it.
 * \param frame    Frame view of the packet buffer.
 * \param hdr      Demo header inside the payload (writable in place).
 */
typedef void (*TsnDemo_RxFrameCb)(EnetDma_Pkt *pktInfo,
                                  EthVlanFrame *frame,
                                  TsnDemo_Hdr *hdr);

#define TSNDEMO_MAX_MAC_PORTS           (2U)

/*!
 * Probe/echo (0x88B5) and bulk (0x88B6) are steered to separate RX DMA
 * flows in hardware via an ALE policer/thread classifier
 * (CPSW_ALE_POLICER_MATCH_ETHERTYPE + threadId - the pattern
 * examples/enet_layer2_multi_channel uses for PTP), each serviced by its
 * own RX task. TSNDEMO_RXW_PROBE runs at a higher priority than
 * TSNDEMO_RXW_DEFAULT so a saturating bulk stream can never delay a T2
 * capture or an echo build - that is the entire reason for the split.
 */
#define TSNDEMO_RXW_DEFAULT             (0U)  /* ENET_DMA_RX_CH0: bulk +
                                                  anything unclassified    */
#define TSNDEMO_RXW_PROBE               (1U)  /* ENET_DMA_RX_CH1: probe/
                                                  echo, ALE-classified,
                                                  higher task priority     */
#define TSNDEMO_RX_WORKER_COUNT         (2U)

typedef struct TsnDemo_RxWorker_s
{
    EnetDma_RxChHandle hRxCh;
    uint32_t           rxChNum;      /*!< also the ALE policer threadId   */
    SemaphoreP_Object  semObj;
    TaskP_Object       taskObj;
    volatile bool      running;      /*!< clear to request exit          */
    volatile bool      exited;       /*!< set by the task at exit        */
    uint32_t           taskPri;
    const char        *name;
} TsnDemo_RxWorker;

typedef struct TsnDemo_Ctx_s
{
    /* Enet */
    Enet_Type         enetType;
    uint32_t          instId;
    uint32_t          coreId;
    uint32_t          coreKey;
    Enet_Handle       hEnet;
    uint8_t           macAddr[ENET_MAC_ADDR_LEN];
    Enet_MacPort      macPorts[TSNDEMO_MAX_MAC_PORTS];
    uint8_t           numMacPorts;

    /* DMA */
    uint32_t          txChNum;
    EnetDma_TxChHandle hTxCh;            /*!< bulk TX channel (CH0)          */
    uint32_t          txChNumExpress;
    EnetDma_TxChHandle hTxChExpress;     /*!< express/echo TX channel (CH1 on
                                              the talker; aliased to hTxCh on
                                              the listener, which has no bulk) */
    EnetDma_PktQ      txFreePktInfoQ;   /*!< express/echo TX pool           */
    EnetDma_PktQ      bulkFreePktInfoQ; /*!< bulk TX pool (talker only)     */

    /* RX: two DMA flows, two tasks - see TSNDEMO_RXW_* above */
    TsnDemo_RxWorker  rxWorker[TSNDEMO_RX_WORKER_COUNT];
    TsnDemo_RxFrameCb rxFrameCb;        /*!< shared by both workers        */

    /* Counters */
    volatile uint32_t tsReadFailCount;
    volatile uint32_t rxDemoFrameCount;

    /* Derived EST base time currently applied (0 = EST not applied) */
    uint64_t          estBaseTimeNs;
    uint32_t          estCycleTimeNs;
} TsnDemo_Ctx;

extern TsnDemo_Ctx gTsnDemo;
extern TsnDemo_Cfg gTsnDemoCfg;

/* ========================================================================== */
/*                     tsndemo_enet.c - bring-up / teardown                   */
/* ========================================================================== */

/*! \brief Open Enet driver, attach core, open DMA, install ALE/VLAN entries. */
int32_t TsnDemoEnet_open(void);

/*! \brief Start uniconf + gPTP (shared examples/tsn framework). */
int32_t TsnDemoEnet_initTsn(void);

/*! \brief true when gptpclock reports GMSYNC_SYNC_STABLE. */
bool    TsnDemoEnet_isClockStable(void);

/*! \brief Block (with log) until the gPTP clock is sync-stable.
 *  \param requireSlaveRole every gPTP device defaults to self-elected
 *  grandmaster before it has heard any peer's Announce, so gmsync can read
 *  SYNC_STABLE trivially at boot with zero real synchronization. Pass
 *  true on a listener/slave node to also require it has genuinely ceded
 *  GM role (gptpclock_we_are_gm() == false) before considering the wait
 *  satisfied. Pass false on the grandmaster/talker node, where staying
 *  self-elected GM is the expected steady state. */
void    TsnDemoEnet_waitClockStable(bool requireSlaveRole);

/*! \brief Print gPTP role (grandmaster vs slave) and numeric offset/adjrate/
 *  gmsync diagnostics via the vendor stack's own debug dump, so sync-stable
 *  can be verified with real numbers instead of trusting one boolean.
 *  NOTE: on a grandmaster, "sync-stable" reflects only the local clock's own
 *  BMCA role - gPTP masters get no protocol feedback on whether any slave
 *  has actually locked to them. On a slave it does reflect real measured
 *  offset/rate convergence. */
void    TsnDemoEnet_printClockDiag(void);

/*! \brief Block until the local gPTP clock reaches targetNs, for tightly
 *  timing a TX submit to land inside a specific EST gate window (e.g. EST
 *  verification probes). Thin wrapper over gptpmasterclock_wait_until_ts64().
 *  \return 0 on success (waited to close to targetNs), 1 if targetNs was
 *  already in the past, 3 if targetNs was too far in the future to bother
 *  waiting, -1 on error - see gptpmasterclock_wait_until_ts64() for exact
 *  semantics. */
int32_t TsnDemoEnet_waitUntilTs64(uint64_t targetNs);

/*! \brief Block (with log) until the given MAC port reports link up. */
void    TsnDemoEnet_waitForLinkUp(Enet_MacPort macPort);

/*! \brief Block (with log) until any one of the configured MAC ports
 *         reports link up. */
void    TsnDemoEnet_waitForAnyLinkUp(void);

/*! \brief Read back negotiated speed of both ports; false on mismatch
 *         with TSNDEMO_LINK_SPEED_MBPS or ports disagreeing. */
bool    TsnDemoEnet_checkLinkSpeed(void);

/*! \brief Print CPSW statistics for host port + both MAC ports. */
void    TsnDemoEnet_printStats(void);

/*! \brief Return the one MAC port currently reporting link up, or
 *  ENET_MAC_PORT_INV if none/more than one is (used to find "the" upstream
 *  port on the last node in a chain, whose other port is dangling). */
Enet_MacPort TsnDemoEnet_findLinkedPort(void);

/*!
 * \brief Enable/disable the bulk-EtherType reflector on upstreamPort: an
 *  ALE inter-VLAN route that bounces TSNDEMO_ETHERTYPE_BULK frames
 *  ingressing on upstreamPort straight back out the same port, for
 *  reverse-direction bulk contention on the last node in a chain (see
 *  continuation notes). Pure ALE classify+redirect - does not touch the
 *  port's link/PHY/loopback state at all, unlike an earlier MAC-loopback
 *  attempt that turned out to disturb other port features. Every node
 *  runs the same firmware image, so this must stay a runtime toggle, not
 *  a per-board build-time setting.
 */
int32_t TsnDemoEnet_setBulkReflector(Enet_MacPort upstreamPort, bool enable);

void    TsnDemoEnet_close(void);

/* ========================================================================== */
/*                    tsndemo_dataflow.c - datapath                           */
/* ========================================================================== */

/*! \brief Open TX/RX DMA channels, allocate and split the packet pools. */
int32_t TsnDemoFlow_open(void);
void    TsnDemoFlow_close(void);

/*! \brief Create RX task; cb runs in RX task context for every demo frame. */
int32_t TsnDemoFlow_createRxTask(TsnDemo_RxFrameCb cb);
void    TsnDemoFlow_destroyRxTask(void);

/*!
 * \brief Take a TX packet from the given pool (NULL if exhausted) and
 *        pre-fill the L2+VLAN header for the given stream.
 *        Returns the payload/header pointer via *hdr / *frame.
 */
EnetDma_Pkt *TsnDemoFlow_allocTxFrame(EnetDma_PktQ *pool,
                                      const TsnDemo_StreamCfg *stream,
                                      const uint8_t *dstMac,
                                      EthVlanFrame **frame,
                                      TsnDemo_Hdr **hdr);

/*! \brief Submit one packet to the TX channel. */
int32_t TsnDemoFlow_submitTxPkt(EnetDma_Pkt *pktInfo);

/*! \brief Reclaim completed TX packets back into their pools. */
uint32_t TsnDemoFlow_reclaimTxPkts(void);

/*!
 * \brief Software timestamp read (CPTS via
 *        ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP). Returns 0 and bumps
 *        gTsnDemo.tsReadFailCount on failure. Must NOT be called with
 *        interrupts disabled and is not re-entrant (see design doc 6.2):
 *        only express frames are timestamped.
 */
uint64_t TsnDemoFlow_getTimeNs(void);

/* ========================================================================== */
/*                    tsndemo_feature.c - EST / IET / cut-through             */
/* ========================================================================== */

/*! \brief Derive the chain-wide EST base time from gPTP time (design 8.2). */
uint64_t TsnDemoFeature_deriveBaseTime(void);

/*! \brief Guard band for the active feature set (design doc 8.5). */
uint32_t TsnDemoFeature_guardBandNs(const TsnDemo_FeatureSet *features);

/*! \brief Compute one MAC port's chain-position-staggered window0 start
 *  offset + width (downstream: fixed width, offset = nodeId*hopInc;
 *  upstream: both grow with hop count back from the far end). See
 *  tsndemo_feature.c for the full derivation. */
void    TsnDemoFeature_computePortWindow(Enet_MacPort macPort,
                                         const TsnDemo_FeatureSet *features,
                                         uint64_t *startOffsetNs,
                                         uint64_t *windowWidthNs);

/*! \brief Apply/remove the EST admin list + state on one MAC port.
 *  \param startOffsetNs this port's chain-position stagger, added on top
 *  of baseTimeNs (0 for the talker/unstaggered case).
 *  \param windowWidthNs this port's window0 (express) width - fixed
 *  downstream, grows with hop count upstream. See
 *  TsnDemoFeature_computePortWindow() (tsndemo_feature.c, internal). */
int32_t TsnDemoFeature_estApply(Enet_MacPort macPort, bool enable,
                                uint64_t baseTimeNs, uint64_t startOffsetNs,
                                uint64_t windowWidthNs, bool ietActive);

/*! \brief Enable/disable IET frame preemption (with verification). */
int32_t TsnDemoFeature_ietApply(Enet_MacPort macPort, bool enable);

/*! \brief Enable/disable cut-through for the express priority. */
int32_t TsnDemoFeature_cutThruApply(Enet_MacPort macPort, bool enable);

/*!
 * \brief Apply a full feature set on all MAC ports (EST last so its guard
 *        band matches the IET state). Updates gTsnDemoCfg.features and
 *        gTsnDemo.estBaseTimeNs on success.
 */
int32_t TsnDemoFeature_applyAll(const TsnDemo_FeatureSet *features);

/*! \brief Current feature state as wire featureBits. */
uint8_t TsnDemoFeature_toBits(const TsnDemo_FeatureSet *features);

/*! \brief Print the EST gate list and PCP->gate mapping. */
void    TsnDemoFeature_printSchedule(const TsnDemo_FeatureSet *features);

/*! \brief Read back the LIVE TAS operational list for one MAC port
 *  (ENET_TAS_IOCTL_GET_OPER_LIST) - the actual baseTime/cycle/gate list
 *  hardware is really running, which can differ from the nominal
 *  requested values due to hardware tick quantization (confirmed on
 *  hardware: cycle can run measurably longer than requested, and by a
 *  different amount per port). \return true on success. */
bool    TsnDemoFeature_getLiveOperList(Enet_MacPort macPort,
                                       EnetTas_ControlList *operList);

/*! \brief Read back and print the LIVE TAS state/oper list per MAC port
 *  directly from hardware (ENET_TAS_IOCTL_GET_STATE / GET_OPER_LIST) -
 *  the actual ground truth, unlike TsnDemoFeature_printSchedule() which
 *  only re-displays what the software intended to configure. Also prints
 *  each port's computed role/offset and, when both ports are live, the
 *  upstream-vs-downstream baseTime drift, so the chain-position stagger
 *  is directly visible without diffing timestamps by hand. Called
 *  automatically at the end of TsnDemoFeature_printSchedule(). */
void    TsnDemoFeature_printOperList(const TsnDemo_FeatureSet *features);

#ifdef __cplusplus
}
#endif

#endif /* TSNDEMO_H_ */
