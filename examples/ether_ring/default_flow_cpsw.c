/*
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
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include <tsn_combase/combase.h>
#include "tsnapp_porting.h"
#include "tsn_gptp/gptpmasterclock.h"
#include "enet_osal.h"
#include <priv/mod/cpsw_cpts_priv.h>
#include <hw_include/cpts/V1/cslr_cpts.h>
#include <priv/mod/cpsw_cpts_ioctl_priv.h>
#include <est/est_configure.h>
#include <enetapp_cpsw.h>
#include <config.h>
#include <dataflow.h>
/* ========================================================================== */
/*                                Function Declarations                       */
/* ========================================================================== */

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define ENETAPP_VLAN_TPID                            (0x8100U)
#define ENETAPP_VLAN_PCP_OFFSET                      (13U)
#define ENETAPP_VLAN_PCP_MASK                        (0x7U)
#define ENETAPP_VLAN_DEI_OFFSET                      (12U)
#define ENETAPP_VLAN_DEI_MASK                        (0x1U)
#define ENETAPP_VLAN_VID_MASK                        (0xFFFU)
#define ENETAPP_VLAN_TCI(pcp, dei, vid)              ((((pcp) & ENETAPP_VLAN_PCP_MASK) << ENETAPP_VLAN_PCP_OFFSET) | \
                                                      (((dei) & ENETAPP_VLAN_DEI_MASK) << ENETAPP_VLAN_DEI_OFFSET) | \
                                                      (((vid) & ENETAPP_VLAN_VID_MASK)))
#define ENETAPP_STREAM_VLANID                         255U
#define SEND_PACKETS_PER_STREAM                       30000
#define TX_TASK_PRIORITY                              14U
#define RX_TASK_PRIORIY                               TX_TASK_PRIORITY + 1U

/* \brief Index of StreamId in CB Packet */
#define ETHERRINGAPP_STREAM_ID_INDEX                                   30U

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/* Experimental EtherType used in TX test packets */
const uint16_t gTxEtherType = 0x88B5U;
const uint16_t gTxVlanTciClassA = Enet_htons(ENETAPP_VLAN_TCI(3, 0, 255));
const uint16_t gTxVlanTciClassD = Enet_htons(ENETAPP_VLAN_TCI(2, 0, 255));
const uint16_t gEthVlanHdrSize = sizeof(EthVlanFrameHeader);

static TaskP_Params taskParamsStreamGen;
static uint8_t gEnetAppTaskStackRx[ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));
static uint8_t gEnetAppStreamTaskStack[MAX_CLASSA_STREAMS + MAX_CLASSD_STREAMS][ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));
static uint8_t gEnetAppEtherRingTaskStack[ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));
static int8_t gEtherRingStreamIdpool[MAX_CLASSA_STREAMS + MAX_CLASSD_STREAMS] = {0,1,2,3,4,5};

static uint8_t gEtherRingStreamToMcast[NODES_COUNT_IN_ETHERRING - 1];
static EtherRing_Cfg gEtherRingCfg;

static uint32_t payLoadLength = 100;

static uint32_t gSendPacketsClassStream[MAX_CLASSA_STREAMS + MAX_CLASSD_STREAMS] = {SEND_PACKETS_PER_STREAM,SEND_PACKETS_PER_STREAM,SEND_PACKETS_PER_STREAM,
                                              SEND_PACKETS_PER_STREAM/8,SEND_PACKETS_PER_STREAM/8,SEND_PACKETS_PER_STREAM/8};
static uint32_t gPacketCountClassStream[MAX_CLASSA_STREAMS + MAX_CLASSD_STREAMS] = {0,0,0,0,0,0};

volatile uint32_t isStreamsEnabled = 0;
extern EnetDma_Handle ghEnetDma;
extern EnetApp_Cfg gEnetAppCfg;
#ifdef ETHERRING_PROFILING
static uint32_t gIsLatencyPrintDone = 0;
extern EtherRingAppRxTs_obj gEtherRingRxTs;
#endif

EtherRingAppRxTs_obj gEtherRingRxTs =
{
        .rxTsIndex = 0,
};
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static void EnetApp_initTxFreePktQ(void);
static uint32_t EnetApp_retrieveFreeTxPkts();
static void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh);
static void EnetApp_scheduleClassAStream(void *stream_id);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

#ifdef PTP_ENABLED
#ifdef HAVE_GPTP_READY_NOTICE

extern CB_SEM_T g_gptpd_ready_semaphore;

static void waitGptpReady()
{
    int gptpReadyCounter = 0;
    EnetAppUtils_print("Waiting for GPTP ready!!\n");
    while(BTRUE)
    {
        if (g_gptpd_ready_semaphore != NULL)
        {
            CB_SEM_GETVALUE(&g_gptpd_ready_semaphore, &gptpReadyCounter);
            if (gptpReadyCounter > 0)
            {
                EnetAppUtils_print("GPTP ready!!\n");
                break;
            }
        }
        CB_USLEEP(100000);
    }
}
#endif
#endif

static void EnetApp_waitSystemStable()
{
    #ifdef HAVE_GPTP_READY_NOTICE
        waitGptpReady();
    #endif // HAVE_GPTP_READY_NOTICE
}

void EnetApp_updateEtherRingCfg(EtherRing_Cfg *etherRingCfg)
{
    /* Last byte of Host Mac address is stored to add it in CB Header*/
    etherRingCfg->hostMacAddLastByte = gEnetAppCfg.macAddr[ENET_MAC_ADDR_LEN - 1];
    etherRingCfg->isCfg = true;
}

int32_t EnetApp_etherRingInit()
{
    int32_t status = ENET_SOK;
    EtherRing_Cfg *pEtherRingCfg = &gEtherRingCfg;

    EnetApp_updateEtherRingCfg(pEtherRingCfg);
    gEnetAppCfg.hEtherRing = EtherRing_open(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, pEtherRingCfg);
    if (gEnetAppCfg.hEtherRing == NULL)
    {
        EnetAppUtils_print("EtherRing Handle is NULL!!\n");
        Enet_assert(gEnetAppCfg.hEtherRing != NULL);
        status = ENET_EFAIL;
    }

    EtherRing_attachTxDmaHandle((void *)gEnetAppCfg.hEtherRing, gEnetAppCfg.hTxCh, gEnetAppCfg.txChNum);
    EtherRing_attachRxDmaHandle((void *)gEnetAppCfg.hEtherRing, gEnetAppCfg.hRxCh, gEnetAppCfg.rxChNum);

    return status;
}


void EnetApp_startHwTimer()
{
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);
}

/* Rx Isr for non-gPTP traffic */
static void EnetApp_rxIsrFxn(void *appData)
{
    SemaphoreP_post(&gEnetAppCfg.rxSemObj);
}

static void EnetApp_mapMcastAndStreamId(int8_t nodeId)
{
    uint32_t streamIndex = 0;
    for (streamIndex = 0; streamIndex < NODES_COUNT_IN_ETHERRING-1 ; streamIndex++)
    {
        if (nodeId != NODES_COUNT_IN_ETHERRING - 1)
        {
            gEtherRingStreamToMcast[streamIndex] = nodeId + 1;
        }
        else
        {
            gEtherRingStreamToMcast[streamIndex] = 0;
        }
    }
}

int32_t EnetApp_configureNodeMcastAddress(Enet_Handle hEnet, uint32_t coreId)
{
    /* Adding multicast entry for traffic generation */
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    uint32_t setMcastoutArgs;

    CpswAle_SetMcastEntryInArgs setMcastInArgs = {
            .addr =
            {
                .addr = {0x01,0x00,0x5E,0x7F,0xFF,gEnetAppCfg.nodeId},
                .vlanId = ENETAPP_STREAM_VLANID,
            },
            .info =
            {
                .portMask = 0x01, /* allow only for host port */
                .super = false,
                .fwdState = CPSW_ALE_FWDSTLVL_FWD,
                .numIgnBits =0U,
            },
    };
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setMcastInArgs, &setMcastoutArgs);
    ENET_IOCTL(hEnet,
            gEnetAppCfg.coreId,
               CPSW_ALE_IOCTL_ADD_MCAST,
               &prms,
               status);
    EnetAppUtils_assert(status == ENET_SOK);

    return status;
}

int32_t EnetApp_updatePtpMcastAddress(Enet_Handle hEnet, uint32_t coreId)
{
        /* Adding multicast entry for ptp to update the mask only to hostPort */
        int32_t status = ENET_SOK;
        Enet_IoctlPrms prms;
        uint32_t setMcastoutArgs;
        CpswAle_SetMcastEntryInArgs setMcastPtpInArgs = {
                .addr =
                {
                    .addr = {0x01,0x80,0xC2,0x00,0x00,0x0E},
                },
                .info =
                {
                    .portMask = 0x01, /* allow for host port */
                    .super = false,
                    .fwdState = CPSW_ALE_FWDSTLVL_FWD,
                    .numIgnBits =0U,
                },
        };
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &setMcastPtpInArgs, &setMcastoutArgs);
        ENET_IOCTL(gEnetAppCfg.hEnet,
                  gEnetAppCfg.coreId,
                   CPSW_ALE_IOCTL_ADD_MCAST,
                   &prms,
                   status);
    EnetAppUtils_assert(status == ENET_SOK);

    return status;
}

int32_t EnetApp_updateDefaultPortVlan(Enet_Handle hEnet, uint32_t coreId)
{
    /* Add the original un-modified vlanId to ALE table */
        Enet_IoctlPrms prms;
        int32_t status = ENET_SOK;
        CpswAle_VlanEntryInfo vlanInArgs;
        uint32_t vlanOutArgs;

        /* Adding vlan entry for default port vlan to enable forceUntaggedEgressMask for ptp */
        memset(&vlanInArgs, 0, sizeof(vlanInArgs));
        vlanInArgs.vlanIdInfo.vlanId        = 0x0;
        vlanInArgs.vlanIdInfo.tagType       = ENET_VLAN_TAG_TYPE_INNER;
        vlanInArgs.vlanMemberList           = 0x7; /* allow for all ports */
        vlanInArgs.unregMcastFloodMask      = 0x7;
        vlanInArgs.regMcastFloodMask        = 0x7;
        vlanInArgs.forceUntaggedEgressMask  = 7U;
        vlanInArgs.noLearnMask              = 0U;
        vlanInArgs.vidIngressCheck          = false;
        vlanInArgs.limitIPNxtHdr            = false;
        vlanInArgs.disallowIPFrag  = false;

        ENET_IOCTL_SET_INOUT_ARGS(&prms, &vlanInArgs, &vlanOutArgs);
        ENET_IOCTL(hEnet, gEnetAppCfg.coreId, CPSW_ALE_IOCTL_ADD_VLAN, &prms, status);
        EnetAppUtils_assert(status == ENET_SOK);

        return status;
}

int32_t EnetApp_addVlanEntries(Enet_Handle hEnet, uint32_t coreId, uint32_t vlan)
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
        ENET_IOCTL(hEnet, gEnetAppCfg.coreId, CPSW_ALE_IOCTL_ADD_VLAN, &prms, status);
        EnetAppUtils_assert(status == ENET_SOK);

        return status;
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

        gEnetAppCfg.rxChNum = rxChInfo.rxChNum;

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

int32_t EnetApp_open()
{
    int32_t status = ENET_SOK;
    status = EnetApp_openDma();

    status = EnetApp_etherRingInit();

    /* Adding vlan entry for Ether-Ring stream traffic */
    EnetApp_addVlanEntries(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, 255);

    EnetApp_updateDefaultPortVlan(gEnetAppCfg.hEnet, gEnetAppCfg.coreId);

    EnetApp_configureNodeMcastAddress(gEnetAppCfg.hEnet, gEnetAppCfg.coreId);

    gEnetAppCfg.etherRingRxPktCnt = 0;

    return status;
}

static void EnetApp_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < 48; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetAppCfg,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
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

static void EnetApp_dmaRxIsr()
{
    int32_t status;
    uintptr_t key;

    key = EnetOsal_disableAllIntr();
    Enet_assert(ghEnetDma != NULL);

    status = EnetCpdma_rxIsr(ghEnetDma);

    /* TODO: Add ISR safe error:
     * failed to handle Rx intr: %d\r\n", status); */
    ENET_UNUSED(status);
    EnetOsal_restoreAllIntr(key);
}

/* Rx Task for stream traffic received on RX Channel 0 */
static void EnetApp_rxTask(void *args)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxSubmitQ;
    EnetDma_Pkt *pktInfo = NULL;
    int32_t status = ENET_SOK;

#ifdef ETHERRING_PROFILING
    uint8_t streamId;
    uint8_t* currTimeStampPtr;
    uint64_t currTimeStampValue;
#endif

    EnetAppUtils_print("%s: default RX flow started\r\n",
                       gEnetAppCfg.name);

    EnetApp_waitSystemStable();

    while(gptpmasterclock_init(NULL)){
        UB_LOG(UBL_INFO,"Waiting for tsn_gptpd to be ready...\n");
        CB_USLEEP(100000);
    }

    while ((ENET_SOK == status))
    {
        /* Wait for packet reception */
        SemaphoreP_pend(&gEnetAppCfg.rxSemObj, SystemP_WAIT_FOREVER);

        /* All peripherals have single hardware RX channel, so we only need to retrieve
         * packets from a single flow.*/
        EnetQueue_initQ(&rxReadyQ);
        EnetQueue_initQ(&rxSubmitQ);

        /* Get the packets received so far */
        status = EtherRing_retrieveRxPktQ(gEnetAppCfg.hEtherRing, &rxReadyQ);

        if (gEnetAppCfg.etherRingRxPktCnt < UINT64_MAX)
        {
            gEnetAppCfg.etherRingRxPktCnt += EnetQueue_getQCount(&rxReadyQ);
        }
        else
        {
            gEnetAppCfg.etherRingRxPktCnt = 0;
        }

        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&rxReadyQ);
        while(pktInfo)
        {
#ifdef ETHERRING_PROFILING
            /* capturing the rx timestamps and currentTimeStamp for received original packets for a stream */
            if (pktInfo->tsInfo.rxPktTs)
            {
                streamId = pktInfo->sgList.list[0].bufPtr[ETHERRINGAPP_STREAM_ID_INDEX - ETHERRING_HEADER_SIZE];

                if ((gEtherRingRxTs.rxTsIndex
                        < ETHERRINGAPP_MAX_RX_TIMESTAMPS_STORED) && (streamId == ETHERRING_PROFILE_STREAMID))
                {
                    /* Storing the rxTs for current packet*/
                    gEtherRingRxTs.timeStampsRx[gEtherRingRxTs.rxTsIndex] =
                            pktInfo->tsInfo.rxPktTs;

                    currTimeStampPtr = pktInfo->sgList.list[0].bufPtr + gEthVlanHdrSize;
                    currTimeStampValue = *(uint64_t*)currTimeStampPtr;

                    /* Storing the current timestamp received with CB packet*/
                    gEtherRingRxTs.currentTimeStamps[gEtherRingRxTs.rxTsIndex] = currTimeStampValue;

                    gEtherRingRxTs.rxTsIndex++;
                }
            }
#endif
            EnetQueue_enq(&rxSubmitQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&rxReadyQ);
        }

        if (status != ENET_SOK)
        {
            /* Should we bail out here? */
            EnetAppUtils_print("Failed to retrieve RX pkt queue: %d\r\n", status);
            Enet_assert(false);
            continue;
        }

        /* Submit now processed buffers */
        EtherRing_submitRxPktQ(gEnetAppCfg.hEtherRing, &rxSubmitQ);

        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to submit RX pkt queue: %d\r\n", status);
            Enet_assert(false);
        }
    }

    TaskP_exit();
}

void EnetApp_createRxTask()
{
    TaskP_Params taskParams;
    int32_t status = ENET_SOK;

    DebugP_assert(ENET_SOK == status);

    status = SemaphoreP_constructBinary(&gEnetAppCfg.rxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    TaskP_Params_init(&taskParams);
    taskParams.priority       = 13;
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

static void EnetApp_scheduleClassAStream(void *stream_id)
{
    EnetDma_PktQ txSubmitQ;
    EthVlanFrame *frame;
    int32_t retVal = ENET_SOK;
    int8_t streamId = *(int8_t*)stream_id;
    uint64_t tsValCurrent = 0ULL;
    uint8_t multicastAddr[ENET_MAC_ADDR_LEN] = {0x01,0x00,0x5E,0x7F,0xFF,gEtherRingStreamToMcast[streamId]};
    EnetDma_PktQ txFreeQ;
    EnetQueue_initQ(&txFreeQ);
    uint32_t key;
    Enet_IoctlPrms prms;
    EnetApp_waitSystemStable();

    while (gptpmasterclock_init(NULL))
    {
        UB_LOG(UBL_INFO,"Waiting for tsn_gptpd to be ready...\n");
        CB_USLEEP(100000);
    }

    wait_est_configured();

    isStreamsEnabled = 1;
    while (true)
    {
        SemaphoreP_pend(&gEnetAppCfg.streamSemObj[streamId], SystemP_WAIT_FOREVER);
        key = EnetOsal_disableAllIntr();

        if (gPacketCountClassStream[streamId] < gSendPacketsClassStream[streamId])
        {
            EnetQueue_initQ(&txSubmitQ);

            if (EnetQueue_getQCount(&gEnetAppCfg.txFreePktInfoQ) > 0 )
            {
                 EnetDma_Pkt *pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&gEnetAppCfg.txFreePktInfoQ);

                 if (pktInfo != NULL)
                 {
                     frame = (EthVlanFrame *)pktInfo->sgList.list[0].bufPtr;
                     memcpy(frame->hdr.dstMac, multicastAddr, 6U);
                     memcpy(frame->hdr.srcMac, gEnetAppCfg.macAddr, 6U);
                     frame->hdr.tpid = Enet_htons(ENETAPP_VLAN_TPID);
                     payLoadLength = CLASSA_PAYLOAD_LENGTH;

                     frame->hdr.etherType = gTxEtherType;
                     frame->hdr.tci  = gTxVlanTciClassA;

                     pktInfo->sgList.list[0].segmentFilledLen = payLoadLength + gEthVlanHdrSize;
#ifdef ETHERRING_PROFILING
                    /* Software Time stamp Push event */
                    ENET_IOCTL_SET_OUT_ARGS(&prms, &tsValCurrent);
                    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId,
                           ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP, &prms, retVal);
#endif
                     /* 8bytes current timestamp(adding zeros for now) + streamId (1 byte) + 191bytes payload */
                     memcpy(&frame->payload[0U], &tsValCurrent, 8U);

                     memset(&frame->payload[8U], (uint8_t)(streamId), 1);
                     memset(&frame->payload[9U], (uint8_t)(0xA5 +
                             EnetQueue_getQCount(&gEnetAppCfg.txFreePktInfoQ)), (payLoadLength - 9));

                     pktInfo->sgList.numScatterSegments = 1;
                     pktInfo->chkSumInfo = 0U;
                     pktInfo->appPriv    = &gEnetAppCfg;
                     EnetDma_checkPktState(&pktInfo->pktState,
                                         ENET_PKTSTATE_MODULE_APP,
                                           ENET_PKTSTATE_APP_WITH_FREEQ,
                                           ENET_PKTSTATE_APP_WITH_DRIVER);

                     /* Enqueue the packet for later transmission */
                     EnetQueue_enq(&txSubmitQ, &pktInfo->node);

                     retVal = EtherRing_submitTxPktQ(gEnetAppCfg.hEtherRing, &txSubmitQ);

                     gPacketCountClassStream[streamId]++;

                     retVal = EtherRing_retrieveTxPktQ(gEnetAppCfg.hEtherRing, &txFreeQ);

                     if (retVal == ENET_SOK)
                     {
                         EnetDma_Pkt *pktInfo;
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
                     EnetOsal_restoreAllIntr(key);
                 }
            }
            else
            {
                EnetAppUtils_print("no free pktInfo for stream: %d\r\n", streamId);
                retVal = EtherRing_retrieveTxPktQ(gEnetAppCfg.hEtherRing, &txFreeQ);

                if (retVal == ENET_SOK)
                {
                    EnetDma_Pkt *pktInfo;
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
            }
        }
        else
        {
#ifdef ETHERRING_PROFILING
            if (gIsLatencyPrintDone == 0 && streamId == 0)
            {
                EnetAppUtils_print("RxTs and CurrentTs values stored\r\n");
                ClockP_sleep(10);
                int16_t timeStampIndex;
                for (timeStampIndex=10; timeStampIndex < ETHERRINGAPP_MAX_RX_TIMESTAMPS_STORED; timeStampIndex++)
                {
                    EnetAppUtils_print("[RXTS]: %llu\r\n",
                            gEtherRingRxTs.timeStampsRx[timeStampIndex]);
                    ClockP_usleep(990);
                }
                for (timeStampIndex=10; timeStampIndex < ETHERRINGAPP_MAX_RX_TIMESTAMPS_STORED; timeStampIndex++)
                {
                    EnetAppUtils_print("[LAT]: %llu\r\n",
                            gEtherRingRxTs.currentTimeStamps[timeStampIndex]);
                    ClockP_usleep(990);
                }
                ClockP_usleep(990);
                gIsLatencyPrintDone = 1;
                EnetAppUtils_print("\r\n\n");
                EnetAppUtils_print("----------ETHERRING DEMONSTRATION COMPLETED----------\r\n");
                EnetAppUtils_print("\r\n\n");
            }
#endif
        }

        if (retVal != ENET_SOK)
        {
            EnetAppUtils_print("Etherring Tx submit failed\r\n");
        }
    }
}

void EnetApp_scheduleClassDStream(void *stream_id)
{
    EnetDma_PktQ txSubmitQ;
    EthVlanFrame *frame;
    int32_t retVal = ENET_SOK;
    int8_t streamId = *(int8_t*)stream_id;
    uint64_t tsValCurrent = 0ULL;
    uint64_t tsValCurrentvalue = 0ULL;
    uint8_t multicastAddr[ENET_MAC_ADDR_LEN] = {0x01,0x00,0x5E,0x7F,0xFF,gEtherRingStreamToMcast[streamId - MAX_CLASSA_STREAMS]};
    EnetDma_PktQ txFreeQ;
    EnetQueue_initQ(&txFreeQ);

    EnetApp_waitSystemStable();

    while (gptpmasterclock_init(NULL))
    {
        UB_LOG(UBL_INFO,"Waiting for tsn_gptpd to be ready...\n");
        CB_USLEEP(100000);
    }

    wait_est_configured();

    isStreamsEnabled = 1;
    while (true)
    {
        SemaphoreP_pend(&gEnetAppCfg.streamSemObj[streamId], SystemP_WAIT_FOREVER);

        EnetQueue_initQ(&txSubmitQ);

        if (EnetQueue_getQCount(&gEnetAppCfg.txFreePktInfoQ) > 0 )
        {
            uint32_t classDstreamIndex = 0;
            for(classDstreamIndex = 0; classDstreamIndex < NUM_CLASSD_STREAMS ;classDstreamIndex++)
            {
                if (gPacketCountClassStream[classDstreamIndex + MAX_CLASSA_STREAMS]
                    < gSendPacketsClassStream[MAX_CLASSA_STREAMS + classDstreamIndex])
                {
                    multicastAddr[ENET_MAC_ADDR_LEN -1 ] = gEtherRingStreamToMcast[classDstreamIndex];
                    EnetDma_Pkt *pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&gEnetAppCfg.txFreePktInfoQ);

                    if (pktInfo != NULL)
                    {
                        /* Packet sent from each Stream: vlan Header(18Bytes) + 8bytes current timestamp + streamId (1 byte)
                         *  + (CLASSD_PAYLOAD_LENGTH - 9)Bytes application payload */
                        frame = (EthVlanFrame *)pktInfo->sgList.list[0].bufPtr;
                        memcpy(frame->hdr.dstMac, multicastAddr, 6U);
                        memcpy(frame->hdr.srcMac, gEnetAppCfg.macAddr, 6U);
                        frame->hdr.tpid = Enet_htons(ENETAPP_VLAN_TPID);

                        payLoadLength = CLASSD_PAYLOAD_LENGTH;
                        EnetAppUtils_assert(payLoadLength >= 9U);

                        frame->hdr.etherType = gTxEtherType;
                        frame->hdr.tci  = gTxVlanTciClassD;

                        pktInfo->sgList.list[0].segmentFilledLen = payLoadLength + gEthVlanHdrSize;

                        /* 8bytes current timestamp(adding zeros for now) + streamId (1 byte) + 191bytes payload */
                        tsValCurrentvalue = tsValCurrent + 1000000;
                        memcpy(&frame->payload[0U], &tsValCurrentvalue, 8U);

                        memset(&frame->payload[8U], (uint8_t)(MAX_CLASSA_STREAMS + classDstreamIndex), 1);
                        memset(&frame->payload[9U], (uint8_t)(0xA5 +
                                EnetQueue_getQCount(&gEnetAppCfg.txFreePktInfoQ)), (payLoadLength - 9));

                        pktInfo->sgList.numScatterSegments = 1;
                        pktInfo->chkSumInfo = 0U;
                        pktInfo->appPriv    = &gEnetAppCfg;
                        EnetDma_checkPktState(&pktInfo->pktState,
                                            ENET_PKTSTATE_MODULE_APP,
                                              ENET_PKTSTATE_APP_WITH_FREEQ,
                                              ENET_PKTSTATE_APP_WITH_DRIVER);

                        /* Enqueue the packet for later transmission */
                        EnetQueue_enq(&txSubmitQ, &pktInfo->node);
                       gPacketCountClassStream[3 + classDstreamIndex]++;
                    }
                }
             }
            retVal = EtherRing_submitTxPktQ(gEnetAppCfg.hEtherRing, &txSubmitQ);

            retVal = EtherRing_retrieveTxPktQ(gEnetAppCfg.hEtherRing, &txFreeQ);

            if (retVal == ENET_SOK)
            {
                EnetDma_Pkt *pktInfo;
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
        }
        else
        {
            EnetAppUtils_print("no free pktInfo for stream: %d\r\n", streamId);
        }

        if (retVal != ENET_SOK)
        {
            EnetAppUtils_print("Etherring Tx submit failed\r\n");
        }
    }
}

void EnetApp_createStreamTask()
{
    int8_t stream_id;
    int32_t status = ENET_SOK;

    EnetApp_mapMcastAndStreamId(gEnetAppCfg.nodeId);

    EnetAppUtils_print("ClassA stream count:%d\r\n", NUM_CLASSA_STREAMS);
    EnetAppUtils_print("ClassD stream count:%d\r\n", NUM_CLASSD_STREAMS);

    for(stream_id = 0 ;stream_id < (NUM_CLASSA_STREAM_TASKS) ;stream_id++)
    {
        status = SemaphoreP_constructBinary(&gEnetAppCfg.streamSemObj[stream_id], 0);
        DebugP_assert(SystemP_SUCCESS == status);
        TaskP_Params_init(&taskParamsStreamGen);
        taskParamsStreamGen.priority       = TX_TASK_PRIORITY;
        taskParamsStreamGen.stack          = gEnetAppStreamTaskStack[stream_id];
        taskParamsStreamGen.stackSize      = sizeof(gEnetAppStreamTaskStack[stream_id]);
        taskParamsStreamGen.args           = (void*)&gEtherRingStreamIdpool[stream_id];
        taskParamsStreamGen.name           = "ClassA Task";
        taskParamsStreamGen.taskMain       = &EnetApp_scheduleClassAStream;

        status = TaskP_construct(&gEnetAppCfg.streamTaskObj[stream_id], &taskParamsStreamGen);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    for(stream_id = MAX_CLASSA_STREAMS ;stream_id < (MAX_CLASSA_STREAMS + NUM_CLASSD_STREAM_TASKS) ;stream_id++)
    {
        status = SemaphoreP_constructBinary(&gEnetAppCfg.streamSemObj[stream_id], 0);
        DebugP_assert(SystemP_SUCCESS == status);
        TaskP_Params_init(&taskParamsStreamGen);
        taskParamsStreamGen.priority       = 10;
        taskParamsStreamGen.stack          = gEnetAppStreamTaskStack[stream_id];
        taskParamsStreamGen.stackSize      = sizeof(gEnetAppStreamTaskStack[stream_id]);
        taskParamsStreamGen.args           = (void*)&gEtherRingStreamIdpool[stream_id];
        taskParamsStreamGen.name           = "ClassD Task";
        taskParamsStreamGen.taskMain       = &EnetApp_scheduleClassDStream;

        status = TaskP_construct(&gEnetAppCfg.streamTaskObj[stream_id], &taskParamsStreamGen);
        DebugP_assert(SystemP_SUCCESS == status);
    }
}

void EnetApp_clearLookupTable()
{
    while(true)
    {
        SemaphoreP_pend(&gEnetAppCfg.etherringSemObj, SystemP_WAIT_FOREVER);
        EtherRing_periodicTick(gEnetAppCfg.hEtherRing);
    }
}

void EnetApp_createEtherRingClearTask()
{
    TaskP_Params taskParams;
    int32_t status = ENET_SOK;

    status = SemaphoreP_constructBinary(&gEnetAppCfg.etherringSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    TaskP_Params_init(&taskParams);
    taskParams.priority       = 1;
    taskParams.stack          = gEnetAppEtherRingTaskStack;
    taskParams.stackSize      = sizeof(gEnetAppEtherRingTaskStack);
    taskParams.args           = (void*)&gEnetAppCfg;
    taskParams.name           = "Etherring Task";
    taskParams.taskMain       = &EnetApp_clearLookupTable;

    status = TaskP_construct(&gEnetAppCfg.etherringTaskObj, &taskParams);

    DebugP_assert(SystemP_SUCCESS == status);
}

void timerIsrClassA(void)
{
    /* This Timer Callback is called at the periodicity of 125us as configured in syscfg */
    static int counter = 0;
    static uint32_t etherRingCounter  = 0;
    int32_t stream_id;
    if(isStreamsEnabled)
    {
            /* Periodicity of Class A Traffic is CLASSA_STREAM_TRAFFIC_PERIODICITY(125000us) */
            for(stream_id = 0 ;stream_id < (NUM_CLASSA_STREAM_TASKS); stream_id++)
            {
                SemaphoreP_post(&gEnetAppCfg.streamSemObj[stream_id]);
            }

            counter++;
            etherRingCounter++;
            if (counter % 8 == 0)
            {
                for(stream_id = 3 ;stream_id<(3 + NUM_CLASSD_STREAM_TASKS); stream_id++)
                {
                    SemaphoreP_post(&gEnetAppCfg.streamSemObj[stream_id]);
                }
                counter=0;
            }

            if(etherRingCounter % 256 == 0)
            {
                SemaphoreP_post(&gEnetAppCfg.etherringSemObj);
                etherRingCounter = 0;
            }
    }
    EnetApp_dmaRxIsr();
}
