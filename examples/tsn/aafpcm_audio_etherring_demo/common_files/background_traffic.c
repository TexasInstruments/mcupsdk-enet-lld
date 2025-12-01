/*
 *  Copyright (c) Texas Instruments Incorporated 2025
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

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include <tsn_combase/combase.h>
// #include "dataflow.h"
#include "tsnapp_porting.h"
#include <enet.h>
#include <kernel/dpl/ClockP.h>
#include "FreeRTOS.h"
#include <kernel/dpl/TaskP.h>
#include <task.h>
#include <enet_apputils.h>
#include <enet_appmemutils.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"
#include <tsn_combase/tilld/cb_lld_ethernet.h>
#include "tsnapp_porting.h"
#include "enetapp_cpsw.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define BACKGROUND_VLAN_ID                   (220)
#define VLAN_TPID                            (0x8100U)
#define VLAN_PCP_OFFSET                      (13U)
#define VLAN_PCP_MASK                        (0x7U)
#define VLAN_DEI_OFFSET                      (12U)
#define VLAN_DEI_MASK                        (0x1U)
#define VLAN_VID_MASK                        (0xFFFU)
#define VLAN_TCI(pcp, dei, vid)              ((((pcp) & VLAN_PCP_MASK) << VLAN_PCP_OFFSET) | \
                                                (((dei) & VLAN_DEI_MASK) << VLAN_DEI_OFFSET) | \
                                                (((vid) & VLAN_VID_MASK)))
#define BACKGROUND_DEST_MAC {0x91, 0xE0, 0xF0, 0x00, 0xFD, 0x00};

/* ========================================================================== */
/*                                Function Declarations                       */
/* ========================================================================== */
extern EnetApp_Cfg gEnetAppCfg;
static uint8_t gEnetAppTaskStackRx[16*1024] __attribute__ ((aligned(32)));

static void EnetApp_initTxFreePktQ(void);
static uint32_t EnetApp_retrieveFreeTxPkts();
static void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh);
static void EnetApp_fillPacket(EnetDma_Pkt *pPktInfo);
static int32_t backgroundTraffic_addVlanEntries(Enet_Handle hEnet, uint32_t coreId, uint32_t vlan);
static int32_t EnetApp_configMcastAddr(Enet_Handle hEnet, uint32_t coreId, uint8_t *mcast, uint32_t vlanID, uint8_t portmask);

static void EnetApp_rxIsrFxn(void *appData)
{
    SemaphoreP_post(&gEnetAppCfg.rxSemObj);
}

static int32_t EnetApp_openDma()
{
    EnetApp_GetDmaHandleInArgs     txInArgs;
    EnetApp_GetTxDmaHandleOutArgs  txChInfo;
    int32_t status = ENET_SOK;

    /* Open the TX channel */
    txInArgs.cbArg   = NULL;
    txInArgs.notifyCb = NULL;

    EnetApp_getTxDmaHandle((ENET_DMA_TX_CH0),
                           &txInArgs,
                           &txChInfo);

    gEnetAppCfg.txChNum = txChInfo.txChNum;
    gEnetAppCfg.hTxCh   = txChInfo.hTxCh;

    if (gEnetAppCfg.hTxCh == NULL)
    {
#if FIX_RM
        /* Free the channel number if open Tx channel failed */
        EnetAppUtils_freeTxCh(gEnetAppCfg.hEnet,
                              gEnetAppCfg.coreKey,
                              gEnetAppCfg.coreId,
                              gEnetAppCfg.txChNum);
#endif
        EnetAppUtils_print("EnetApp_openDma() failed to open TX channel\r\n");
        status = ENET_EFAIL;
        EnetAppUtils_assert(gEnetAppCfg.hTxCh != NULL);
    }

    /* Allocate TX packets and keep them locally enqueued */
    if (status == ENET_SOK)
    {
        EnetApp_initTxFreePktQ();
    }

    /* Open the RX flow for Regular frames */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs     rxInArgs;
        EnetApp_GetRxDmaHandleOutArgs  rxChInfo;

        rxInArgs.notifyCb = EnetApp_rxIsrFxn;
        rxInArgs.cbArg   = NULL;

        EnetApp_getRxDmaHandle(ENET_DMA_RX_CH0,
                               &rxInArgs,
                               &rxChInfo);
#if defined(ENET_SOC_HOSTPORT_DMA_TYPE_UDMA)
        gEnetAppCfg.rxStartFlowIdx = rxChInfo.rxFlowStartIdx;
        gEnetAppCfg.rxFlowIdx = rxChInfo.rxFlowIdx;
#else
        gEnetAppCfg.rxFlowIdx = rxChInfo.rxChNum;
#endif
        gEnetAppCfg.hRxCh  = rxChInfo.hRxCh;
        EnetAppUtils_assert(rxChInfo.numValidMacAddress == 1);
        EnetUtils_copyMacAddr(gEnetAppCfg.macAddr, rxChInfo.macAddr[rxChInfo.numValidMacAddress - 1]);
        EnetAppUtils_print("MAC port addr: ");
        EnetAppUtils_printMacAddr(gEnetAppCfg.macAddr);

        if (gEnetAppCfg.hRxCh == NULL)
        {
            EnetAppUtils_print("EnetApp_openRxCh() failed to open RX flow\r\n");
            status = ENET_EFAIL;
            EnetAppUtils_assert(gEnetAppCfg.hRxCh != NULL);
        }
    }

    /* Submit all ready RX buffers to DMA */
    if (status == ENET_SOK)
    {

        EnetApp_initRxReadyPktQ(gEnetAppCfg.hRxCh);
    }

     return status;
}

static void EnetApp_closeDma()
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;

    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* Close Regular RX channel */
    EnetApp_closeRxDma(ENET_DMA_RX_CH0,
                       gEnetAppCfg.hEnet,
                       gEnetAppCfg.coreKey,
                       gEnetAppCfg.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    /* Close TX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* Retrieve any pending TX packets from driver */
    EnetApp_retrieveFreeTxPkts();

    EnetApp_closeTxDma(ENET_DMA_TX_CH0,
                       gEnetAppCfg.hEnet,
                       gEnetAppCfg.coreKey,
                       gEnetAppCfg.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&gEnetAppCfg.txFreePktInfoQ);
}

static void EnetApp_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < ENET_DMA_TX_CH0_NUM_PKTS; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetAppCfg,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);

        EnetApp_fillPacket(pPktInfo);

        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&gEnetAppCfg.txFreePktInfoQ, &pPktInfo->node);
    }

    EnetAppUtils_print("initQs() txFreePktInfoQ initialized with %d pkts\r\n",
                       EnetQueue_getQCount(&gEnetAppCfg.txFreePktInfoQ));
}

static void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    int32_t status;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    EnetQueue_initQ(&rxFreeQ);

    for (i = 0U; i < ENET_DMA_RX_CH0_NUM_PKTS; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetAppCfg,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);

        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&rxFreeQ, &pPktInfo->node);
    }

    /* Retrieve any packets which are ready */
    EnetQueue_initQ(&rxReadyQ);
    status = EnetDma_retrieveRxPktQ(hRxCh, &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);

    /* There should not be any packet with DMA during init */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&rxFreeQ,
                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                     ENET_PKTSTATE_APP_WITH_DRIVER);

    EnetDma_submitRxPktQ(hRxCh, &rxFreeQ);

    /* Assert here, as during init, the number of DMA descriptors should be equal to
     * the number of free Ethernet buffers available with app */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxFreeQ) == 0U);
}

static uint32_t EnetApp_retrieveFreeTxPkts()
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    uint32_t txFreeQCnt = 0U;
    int32_t status;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any packets that may be free now */
    status = EnetDma_retrieveTxPktQ(gEnetAppCfg.hTxCh, &txFreeQ);
    if (status == ENET_SOK)
    {
        txFreeQCnt = EnetQueue_getQCount(&txFreeQ);

        pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        while (NULL != pktInfo)
        {
            EnetDma_checkPktState(&pktInfo->pktState,
                                    ENET_PKTSTATE_MODULE_APP,
                                    ENET_PKTSTATE_APP_WITH_DRIVER,
                                    ENET_PKTSTATE_APP_WITH_FREEQ);

            EnetQueue_enq(&gEnetAppCfg.txFreePktInfoQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        }
    }
    else
    {
        EnetAppUtils_print("retrieveFreeTxPkts() failed to retrieve pkts: %d\r\n", status);
    }

    return txFreeQCnt;
}

/* Rx Echo task for non-gPTP traffic */
static void EnetApp_rxTask(void *args)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_PktQ txSubmitQ;
    EnetDma_Pkt *rxPktInfo;
    EnetDma_Pkt *txPktInfo;
    int32_t status = ENET_SOK;

    /* Configure Background VLAN... */
    backgroundTraffic_addVlanEntries(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, BACKGROUND_VLAN_ID);

#if DEF_NODE_CENTRAL
    uint8_t destMcast[] = BACKGROUND_DEST_MAC;
    EnetApp_configMcastAddr(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, destMcast, BACKGROUND_VLAN_ID, 0x01);
#endif

    EnetAppUtils_print("%s: default RX flow started\r\n", gEnetAppCfg.name);

    while ((ENET_SOK == status))
    {
        ClockP_usleep(10*1000);
        EnetQueue_initQ(&rxReadyQ);
        EnetQueue_initQ(&rxFreeQ);
        EnetQueue_initQ(&txSubmitQ);
#if DEF_NODE_CENTRAL
        /* Retrieve TX packets from driver and recycle them */
        EnetApp_retrieveFreeTxPkts();
        do
        {
            /* Dequeue one free TX Eth packet */
            txPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetAppCfg.txFreePktInfoQ);
            if (txPktInfo != NULL)
            {
                txPktInfo->sgList.numScatterSegments = 1;
                txPktInfo->chkSumInfo = 0U;
                txPktInfo->appPriv = &gEnetAppCfg;
                txPktInfo->tsInfo.enableHostTxTs = BFALSE;
                txPktInfo->txPortNum = ENET_MAC_PORT_2;

                EnetDma_checkPktState(&txPktInfo->pktState,
                                        ENET_PKTSTATE_MODULE_APP,
                                        ENET_PKTSTATE_APP_WITH_FREEQ,
                                        ENET_PKTSTATE_APP_WITH_DRIVER);

                /* Enqueue the packet for later transmission */
                EnetQueue_enq(&txSubmitQ, &txPktInfo->node);
            }
            else
            {
                break;
            }
        } while (1);

        /* Transmit all enqueued packets */
        status = EnetDma_submitTxPktQ(gEnetAppCfg.hTxCh, &txSubmitQ);
#else
        (void)txPktInfo;
#endif
        /* Get the packets received so far */
        status = EnetDma_retrieveRxPktQ(gEnetAppCfg.hRxCh, &rxReadyQ);
        if (status != ENET_SOK)
        {
            /* Should we bail out here? */
            EnetAppUtils_print("Failed to retrieve RX pkt queue: %d\r\n", status);
            continue;
        }
        /* Consume the received packets and send them back */
        rxPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        while (rxPktInfo != NULL)
        {
            EnetDma_checkPktState(&rxPktInfo->pktState,
                                    ENET_PKTSTATE_MODULE_APP,
                                    ENET_PKTSTATE_APP_WITH_DRIVER,
                                    ENET_PKTSTATE_APP_WITH_FREEQ);

            /* Release the received packet */
            EnetQueue_enq(&rxFreeQ, &rxPktInfo->node);
            rxPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        }
        EnetAppUtils_validatePacketState(&rxFreeQ,
                                            ENET_PKTSTATE_APP_WITH_FREEQ,
                                            ENET_PKTSTATE_APP_WITH_DRIVER);

        /* Submit now processed buffers */
        EnetDma_submitRxPktQ(gEnetAppCfg.hRxCh, &rxFreeQ);
    }
    TaskP_exit();
}

static void EnetApp_fillPacket(EnetDma_Pkt *pPktInfo)
{
    EthVlanFrame* frame = (EthVlanFrame*)pPktInfo->sgList.list[0].bufPtr;

    DebugP_assert(sizeof(EthVlanFrame) <= pPktInfo->sgList.list[0].segmentAllocLen);
    DebugP_assert(pPktInfo->sgList.numScatterSegments == 1);

    // pPktInfo->sgList.list[0].disableCacheOps = true;

    uint8_t destMacAddr[] = BACKGROUND_DEST_MAC;
    memcpy(frame->hdr.srcMac, gEnetAppCfg.macAddr, sizeof(frame->hdr.srcMac));
    memcpy(frame->hdr.dstMac, destMacAddr, sizeof(frame->hdr.srcMac));
    frame->hdr.tpid   = Enet_htons(VLAN_TPID);
    frame->hdr.tci    = Enet_htons(VLAN_TCI(0, 0, BACKGROUND_VLAN_ID));
    frame->hdr.etherType = Enet_htons(0x9000);

    for (int i = 0; i < sizeof(frame->payload); i++)
    {
        frame->payload[i] = (uint8_t)i;
    }

    pPktInfo->sgList.list[0].segmentFilledLen = sizeof(EthVlanFrame);
}

void EnetApp_createRxTask()
{
    TaskP_Params taskParams;
    int32_t status;

    status = EnetApp_openDma();
    DebugP_assert(ENET_SOK == status);

    status = SemaphoreP_constructBinary(&gEnetAppCfg.rxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    TaskP_Params_init(&taskParams);
    taskParams.priority       = 2U;
    taskParams.stack          = gEnetAppTaskStackRx;
    taskParams.stackSize      = sizeof(gEnetAppTaskStackRx);
    taskParams.args           = (void*)&gEnetAppCfg;
    taskParams.name           = "Rx Task";
    taskParams.taskMain       = &EnetApp_rxTask;

    status = TaskP_construct(&gEnetAppCfg.rxTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);
}

void EnetApp_destroyRxTask()
{
    SemaphoreP_destruct(&gEnetAppCfg.rxSemObj);
    TaskP_destruct(&gEnetAppCfg.rxTaskObj);
    EnetApp_closeDma();
}

CpswStats_PortStats gEnetApp_cpswStats;
uint64_t prevBytes;
uint64_t prevStatsTime = 0;

void EnetApp_printStats(uint64_t currentTime)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    int32_t status;

    macPort = ENET_MAC_PORT_2;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &gEnetApp_cpswStats);

    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("%s: Failed to get port %u stats\r\n", ENET_MACPORT_ID(macPort));
    }
    CpswStats_MacPort_Ng *stats = (CpswStats_MacPort_Ng *)&gEnetApp_cpswStats;

    uint64_t currentBytes = stats->txPriBcnt[0];

    if (prevStatsTime != 0)
    {
        double bitrate = ((currentBytes-prevBytes)*8)/(double)(currentTime-prevStatsTime);
        DebugP_log("prevBytes = %llu, currentBytes %llu, diff %lld, bitrate %0.2lf\r\n", prevBytes, currentBytes, currentBytes-prevBytes, bitrate);
    }

    prevBytes = currentBytes;
    prevStatsTime = currentTime;
}

static int32_t backgroundTraffic_addVlanEntries(Enet_Handle hEnet, uint32_t coreId, uint32_t vlan)
{
        /* Add the original un-modified vlanId to ALE table */
        Enet_IoctlPrms prms;
        int32_t status = ENET_SOK;
        CpswAle_VlanEntryInfo vlanInArgs;
        uint32_t vlanOutArgs;

        /* Adding vlan entry for traffic generation */
        memset(&vlanInArgs, 0, sizeof(vlanInArgs));
        vlanInArgs.vlanIdInfo.vlanId        = vlan;
        vlanInArgs.vlanIdInfo.tagType       = ENET_VLAN_TAG_TYPE_INNER;
        vlanInArgs.vlanMemberList           = 0x7; /* allow for all ports */
        vlanInArgs.unregMcastFloodMask      = 0x6; /* except host port */
        vlanInArgs.regMcastFloodMask        = 0x7;
        vlanInArgs.forceUntaggedEgressMask  = 0U;
        vlanInArgs.noLearnMask              = 0U;
        vlanInArgs.vidIngressCheck          = false;
        vlanInArgs.limitIPNxtHdr            = false;
        vlanInArgs.disallowIPFrag  = false;

        ENET_IOCTL_SET_INOUT_ARGS(&prms, &vlanInArgs, &vlanOutArgs);
        EnetAppUtils_assert(status == ENET_SOK);
        ENET_IOCTL(hEnet, coreId, CPSW_ALE_IOCTL_ADD_VLAN, &prms, status);

        return status;
}

static int32_t EnetApp_configMcastAddr(Enet_Handle hEnet, uint32_t coreId, uint8_t *mcast, uint32_t vlanID, uint8_t portmask)
{
    /* Adding multicast entry for traffic generation */
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    uint32_t setMcastoutArgs;

    CpswAle_SetMcastEntryInArgs setMcastInArgs = {
            .addr =
            {
                .vlanId = vlanID,
            },
            .info =
            {
                .portMask = portmask, /* allow only for host port */
                .super = false,
                .fwdState = CPSW_ALE_FWDSTLVL_FWD,
                .numIgnBits =0U,
            },
    };
    memcpy(&setMcastInArgs.addr.addr, mcast, sizeof(setMcastInArgs.addr.addr));
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setMcastInArgs, &setMcastoutArgs);
    ENET_IOCTL(hEnet,
               coreId,
               CPSW_ALE_IOCTL_ADD_MCAST,
               &prms,
               status);
    EnetAppUtils_assert(status == ENET_SOK);

    return status;
}