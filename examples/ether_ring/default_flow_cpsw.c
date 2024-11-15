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
#include "est/est_configure.h"
#include "enetapp_cpsw.h"
#include "dataflow.h"
#include "tsnapp_porting.h"
#include "tsn_gptp/gptpmasterclock.h"
#include "enet_osal.h"
#include <priv/mod/cpsw_cpts_priv.h>
#include <hw_include/cpts/V1/cslr_cpts.h>
#include <priv/mod/cpsw_cpts_ioctl_priv.h>
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
/* Experimental EtherType used in TX test packets */
#define ENETAPP_TEST_TX_ETHERTYPE                    (0x88B5U)
#define ENETAPP_TEST_TX_ETHERTYPE_2                  (0xB588U)

#define SEND_PACKETS_PER_STREAM 100000
#define TX_TASK_PRIORITY                              14U
#define TX_RETRIEVE_TASK_PRIORITY                     TX_TASK_PRIORITY+1
#define RX_TASK_PRIORIY                               TX_TASK_PRIORITY+1
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
extern EnetApp_Cfg gEnetAppCfg;
extern EtherRingRxTs_obj gEtherRingRxTs;

static TaskP_Params taskParamsStreamGen;
static uint8_t gEnetAppTaskStackRx[ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));
static uint8_t gEnetAppStreamTaskStack[MAX_CLASSA_STREAMS + MAX_CLASSD_STREAMS][ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));
static uint8_t gEnetAppEtherRingTaskStack[ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));
static int8_t gEtherRingStreamIdpool[MAX_CLASSA_STREAMS + MAX_CLASSD_STREAMS] = {0,1,2,3,4,5};

//static uint8_t gEtherRingStreamToMcast[MAX_NODES_IN_LOOP - 1];
static uint8_t gEtherRingStreamToMcast[MAX_NODES_IN_LOOP];
//static uint8_t gEtherRingStreamToMcast[3];

static EtherRing_Cfg gEtherRingCfg;

static uint32_t payLoadLength = 100;
// static void *gEtherRingprintLock = NULL;

#if 1
static uint32_t gSendPacketsClassStream[6] = {SEND_PACKETS_PER_STREAM,SEND_PACKETS_PER_STREAM,SEND_PACKETS_PER_STREAM,
                                              SEND_PACKETS_PER_STREAM/8,SEND_PACKETS_PER_STREAM/8,SEND_PACKETS_PER_STREAM/8};
static uint32_t gPacketCountClassStream[6] = {0,0,0,0,0,0};
#endif

extern CpswCpts_Handle ghCptsEtherRing;
extern CSL_cptsRegs *gCptsRegsEtherRing;

//#define CYCLEP_DEBUG
//#define SUB_JITTER
//#define SUB_PROFILE
//#define CYCLEP_MAX_MIN_JITTER
//#define SUBPEND_JITTER
//#define RXPEND_JITTER
//#define RX_PROFILE
#define TX_DEBUG_ARRAY_SIZE  40000

#ifdef CYCLEP_MAX_MIN_JITTER
	static uint32_t gPrevTimeCb = 0;
#endif

#ifdef SUB_JITTER
	static uint32_t gPrevTimeCb = 0;
	volatile static uint32_t gTimerDebugIndex = 0;
	static uint32_t gTimerArray[TX_DEBUG_ARRAY_SIZE];
	volatile static uint32_t gIsTxArrayPrinted = 0;
#endif

#ifdef SUB_PROFILE
//	static uint32_t gPrevTimeCb = 0;
	volatile static uint32_t gTimerDebugIndex = 0;
	static uint32_t gTimerArray[TX_DEBUG_ARRAY_SIZE];
	volatile static uint32_t gIsTxArrayPrinted = 0;
#endif

#ifdef RX_PROFILE
	volatile static uint32_t gTimerDebugIndex = 0;
	static uint32_t gTimerArray[TX_DEBUG_ARRAY_SIZE];
	volatile static uint32_t gIsTxArrayPrinted = 0;
#endif

#ifdef CYCLEP_DEBUG
	static uint32_t gPrevTimeCb = 0;
	volatile static uint32_t gTimerDebugIndex = 0;
	static uint32_t gTimerArray[TX_DEBUG_ARRAY_SIZE];
	volatile static uint32_t gIsTxArrayPrinted = 0;
#endif

#ifdef SUBPEND_JITTER
	static uint32_t gPrevTimeCbA0 = 0;
	static uint32_t gPrevTimeCbA1 = 0;
	volatile static uint32_t gTimerDebugIndexA0 = 0;
	volatile static uint32_t gTimerDebugIndexA1 = 0;
	static uint32_t gTimerArrayA0[TX_DEBUG_ARRAY_SIZE/2];
	static uint32_t gTimerArrayA1[TX_DEBUG_ARRAY_SIZE/2];
	volatile static uint32_t gIsTxArrayPrinted = 0;
#endif

#ifdef RXPEND_JITTER
	static uint32_t gPrevTimeCb = 0;
	volatile static uint32_t gTimerDebugIndex = 0;
	static uint32_t gTimerArray[TX_DEBUG_ARRAY_SIZE];
	volatile static uint32_t gIsTxArrayPrinted = 0;
#endif

//uint8_t isPrintedRxIsr = 0;
SemaphoreP_Object gRxIsrObj;
volatile uint32_t isStreamsEnabled = 0;
volatile uint32_t stopCapture = 0;
extern EnetDma_Handle ghEnetDma;
extern volatile uint32_t isIsrArrayCapture;
static volatile uint32_t staticRxTaskPoints[2];
//static volatile uint32_t gMinTimerJitter = UINT32_MAX;
//static volatile uint32_t gMaxTimerJitter = 0;

static uint32_t gIsPrintDone = 0;
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static void EnetApp_initTxFreePktQ(void);
static uint32_t EnetApp_retrieveFreeTxPkts();
static void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh);
void EnetApp_scheduleStream(void *stream_id);

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

    EtherRing_TxDmaHdle_Attach((void *)gEnetAppCfg.hEtherRing, gEnetAppCfg.hTxCh, gEnetAppCfg.txChNum);
    EtherRing_RxDmaHdle_Attach((void *)gEnetAppCfg.hEtherRing, gEnetAppCfg.hRxCh, gEnetAppCfg.rxFlowIdx);

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

int32_t EnetApp_mapMcastAndStreamId(int8_t nodeId)
{
#if 0
    uint8_t streamId = 0;
    uint32_t nodeIndex = nodeId+1;
    uint32_t streamsAssigned = 0;

    while(nodeIndex != nodeId && streamsAssigned < MAX_NODES_IN_LOOP-1)
    {
        if (nodeIndex >= MAX_NODES_IN_LOOP)
        {
            nodeIndex = 0;
        }
        gEtherRingStreamToMcast[streamId++] = nodeIndex;
        nodeIndex++;
        streamsAssigned++;
    }
#endif
    if(nodeId == 0)
    {
    	gEtherRingStreamToMcast[0] = 1;
    	gEtherRingStreamToMcast[1] = 1;
    	gEtherRingStreamToMcast[2] = 1;
    }
    else if(nodeId == 1)
    {
    	gEtherRingStreamToMcast[0] = 2;
    	gEtherRingStreamToMcast[1] = 2;
    	gEtherRingStreamToMcast[2] = 2;
    }
    else if(nodeId ==2)
    {
    	gEtherRingStreamToMcast[0] = 3;
    	gEtherRingStreamToMcast[1] = 3;
    	gEtherRingStreamToMcast[2] = 3;
    }
    else if(nodeId ==3)
    {
        gEtherRingStreamToMcast[0] = 0;
        gEtherRingStreamToMcast[1] = 0;
        gEtherRingStreamToMcast[2] = 0;
    }
    return 0;
}

int32_t EnetApp_configureMcastAddress(Enet_Handle hEnet, uint32_t coreId)
{
    // Adding multicast entry for traffic generation
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    uint32_t setMcastoutArgs;

    CpswAle_SetMcastEntryInArgs setMcastInArgs = {
            .addr =
            {
                .addr = {0x01,0x00,0x5E,0x7F,0xFF,gEnetAppCfg.nodeId},
                .vlanId = 255,
            },
            .info =
            {
//                .portMask = 0x07, /* allow for all ports */
                .portMask = 0x01, /* allow for all ports */
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

    // Adding multicast entry for ptp to update the mask only to hostPort
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
    ENET_IOCTL(hEnet,
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

        // Adding vlan entry for default port vlan to enable forceUntaggedEgressMask for ptp
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

        // Adding vlan entry for traffic generation
        memset(&vlanInArgs, 0, sizeof(vlanInArgs));
        vlanInArgs.vlanIdInfo.vlanId        = vlan;
        vlanInArgs.vlanIdInfo.tagType       = ENET_VLAN_TAG_TYPE_INNER;
        vlanInArgs.vlanMemberList           = 0x7; /* allow for all ports */
//        vlanInArgs.unregMcastFloodMask      = 0x7;
        vlanInArgs.unregMcastFloodMask      = 0x6;
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
//        rxInArgs.notifyCb = NULL;
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

int32_t EnetApp_open()
{
    int32_t status = ENET_SOK;
    status = EnetApp_openDma();

    status = EnetApp_etherRingInit();

    EnetApp_addVlanEntries(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, 255);
    EnetApp_addVlanEntries(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, 30);
    EnetApp_addVlanEntries(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, 40);
    EnetApp_updateDefaultPortVlan(gEnetAppCfg.hEnet, gEnetAppCfg.coreId);

    EnetApp_configureMcastAddress(gEnetAppCfg.hEnet, gEnetAppCfg.coreId);

    // EtherRing_createClearLookupPollTask();

    gEnetAppCfg.txSubmittedCount = 0;
    gEnetAppCfg.totalRxCnt = 0;

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

static void EtherRing_dmaRxIsr()
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

/* Rx Echo task for non-gPTP traffic */
static void EnetApp_rxTask(void *args)
{
    EnetDma_PktQ rxReadyQ;
    int32_t status = ENET_SOK;

    EnetAppUtils_print("%s: default RX flow started\r\n",
                       gEnetAppCfg.name);
#ifdef PTP_ENABLED
    EnetApp_waitSystemStable();

    while(gptpmasterclock_init(NULL)){
        UB_LOG(UBL_INFO,"Waiting for tsn_gptpd to be ready...\n");
        CB_USLEEP(100000);
    }
#endif
    while ((ENET_SOK == status))
    {
        /* Wait for packet reception */
        SemaphoreP_pend(&gEnetAppCfg.rxSemObj, SystemP_WAIT_FOREVER);
#ifdef RXPEND_JITTER
        uint32_t gCurrentTime = CycleCounterP_getCount32();
        if(gPrevTimeCb != 0)
        {
			if(isStreamsEnabled && (gTimerDebugIndex < TX_DEBUG_ARRAY_SIZE))
			{
				if(gCurrentTime > gPrevTimeCb)
				{
					gTimerArray[gTimerDebugIndex] = gCurrentTime-gPrevTimeCb;
				}
				else
				{
					gTimerArray[gTimerDebugIndex] = (0xFFFFFFFF -gPrevTimeCb + gCurrentTime)&0xFFFFFFFF;
				}
				gTimerDebugIndex++;
			}
        }
        gPrevTimeCb = gCurrentTime;
        if(gIsTxArrayPrinted == 0 && gTimerDebugIndex >= (TX_DEBUG_ARRAY_SIZE))
        {
        	uint32_t printArrayIndex = 0;
        	for(printArrayIndex = 0; printArrayIndex<(TX_DEBUG_ARRAY_SIZE); printArrayIndex++)
			{
				EnetAppUtils_print("RXPEND: %lu\r\n", gTimerArray[printArrayIndex]);
			}
			gIsTxArrayPrinted = 1;
        }
#endif
#ifdef RX_PROFILE
		uint32_t gPrevTime = CycleCounterP_getCount32();
        // printing the jitter for submitTx
        if(gIsTxArrayPrinted == 0 && gTimerDebugIndex >= (TX_DEBUG_ARRAY_SIZE))
        {
        	uint32_t printArrayIndex = 0;
        	for(printArrayIndex = 0; printArrayIndex<(TX_DEBUG_ARRAY_SIZE); printArrayIndex++)
			{
				EnetAppUtils_print("RX_PF: %lu\r\n", gTimerArray[printArrayIndex]);
			}
			gIsTxArrayPrinted = 1;
        }
#endif

#if 0
		if (gIsPrintDone == 0 && gEnetAppCfg.totalRxCnt >= 90000)
		{
			int16_t timeStampIndex;
			for(timeStampIndex=10; timeStampIndex<10000; timeStampIndex++)
			{
				EnetAppUtils_print("[EL_RXTS_A]: %llu\r\n",
						gEtherRingRxTs.etherRingTimeStampsRx[0][timeStampIndex]);
			}
			 gIsPrintDone = 1;
		}
#endif
		uint32_t startTime = CycleCounterP_getCount32();
        /* All peripherals have single hardware RX channel, so we only need to retrieve
         * packets from a single flow.*/
        EnetQueue_initQ(&rxReadyQ);

        /* Get the packets received so far */
        status = EtherRing_retrieveRxPktQ(gEnetAppCfg.hEtherRing, &rxReadyQ);

        staticRxTaskPoints[0] = (CycleCounterP_getCount32() - startTime)/400;
        if(staticRxTaskPoints[0] > 22)
        {
//                    	 DebugP_assert(0);
        }
        gEnetAppCfg.totalRxCnt += EnetQueue_getQCount(&rxReadyQ);
        if (status != ENET_SOK)
        {
            /* Should we bail out here? */
            EnetAppUtils_print("Failed to retrieve RX pkt queue: %d\r\n", status);
            Enet_assert(false);
            continue;
        }

        /* Submit now processed buffers */
        EtherRing_submitRxPktQ(gEnetAppCfg.hEtherRing, &rxReadyQ);
        staticRxTaskPoints[1] = (CycleCounterP_getCount32() - startTime)/400;
        if(staticRxTaskPoints[1] > 22)
        {
//                    	 DebugP_assert(0);
        }
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to submit RX pkt queue: %d\r\n", status);
            Enet_assert(false);
        }
#ifdef RX_PROFILE
					 uint32_t gCurrentTime = CycleCounterP_getCount32();
					if(isStreamsEnabled && (gTimerDebugIndex < TX_DEBUG_ARRAY_SIZE))
					{
						if(gCurrentTime > gPrevTime)
						{
							gTimerArray[gTimerDebugIndex] = gCurrentTime-gPrevTime;
						}
						else
						{
							gTimerArray[gTimerDebugIndex] = (0xFFFFFFFF -gPrevTime + gCurrentTime)&0xFFFFFFFF;
						}
						gTimerDebugIndex++;
					}
#endif
    }


    TaskP_exit();
}

void EnetApp_createRxTask()
{
    TaskP_Params taskParams;
    int32_t status = ENET_SOK;

    DebugP_assert(ENET_SOK == status);

//    status = SemaphoreP_constructBinary(&gRxIsrObj, 0);
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

void EnetApp_scheduleStream(void *stream_id)
{
    EnetDma_PktQ txSubmitQ;
    EthVlanFrame *frame;
    int32_t retVal = ENET_SOK;
    int8_t streamId = *(int8_t*)stream_id;
    uint64_t tsValCurrent = 0ULL;
    uint64_t tsValCurrentvalue = 0ULL;
    uint8_t multicastAddr[ENET_MAC_ADDR_LEN] = {0x01,0x00,0x5E,0x7F,0xFF,0x00};
    EnetDma_PktQ txFreeQ;
    EnetQueue_initQ(&txFreeQ);
    uint32_t key;

    if (streamId < 3)
    {
        multicastAddr[ENET_MAC_ADDR_LEN-1] = gEtherRingStreamToMcast[streamId];
    }
    else if (streamId >=3 && streamId < 6)
    {
        multicastAddr[ENET_MAC_ADDR_LEN-1] = gEtherRingStreamToMcast[streamId-3];
    }

    int8_t priority = 0;

#ifdef PTP_ENABLED
    EnetApp_waitSystemStable();

    while(gptpmasterclock_init(NULL)){
        UB_LOG(UBL_INFO,"Waiting for tsn_gptpd to be ready...\n");
        CB_USLEEP(100000);
    }
#endif

    #ifdef WITH_EST_CONFIG
        wait_est_configured();
    #endif

    // EnetAppUtils_print("Schedule started for StreamId: %d\r\n", streamId);

    isStreamsEnabled = 1;
    while(true)
    {
        SemaphoreP_pend(&gEnetAppCfg.streamSemObj[streamId], SystemP_WAIT_FOREVER);
         key = EnetOsal_disableAllIntr();
#ifdef SUBPEND_JITTER
         if(streamId == 0)
         {
             uint32_t gCurrentTime = CycleCounterP_getCount32();
             if(gPrevTimeCbA0 != 0)
             {
                 if(isStreamsEnabled && (gTimerDebugIndexA0 < TX_DEBUG_ARRAY_SIZE))
                 {
                     if(gCurrentTime > gPrevTimeCbA0)
                     {
                         gTimerArrayA0[gTimerDebugIndexA0] = gCurrentTime-gPrevTimeCbA0;
                     }
                     else
                     {
                         gTimerArrayA0[gTimerDebugIndexA0] = (0xFFFFFFFF -gPrevTimeCbA0 + gCurrentTime)&0xFFFFFFFF;
                     }
                     gTimerDebugIndexA0++;
                 }
             }
             gPrevTimeCbA0 = gCurrentTime;
         }
         else if(streamId == 1)
         {
             uint32_t gCurrentTime = CycleCounterP_getCount32();
             if(gPrevTimeCbA1 != 0)
             {
                 if(isStreamsEnabled && (gTimerDebugIndexA1 < TX_DEBUG_ARRAY_SIZE))
                 {
                     if(gCurrentTime > gPrevTimeCbA1)
                     {
                         gTimerArrayA1[gTimerDebugIndexA1] = gCurrentTime-gPrevTimeCbA1;
                     }
                     else
                     {
                         gTimerArrayA1[gTimerDebugIndexA1] = (0xFFFFFFFF -gPrevTimeCbA1 + gCurrentTime)&0xFFFFFFFF;
                     }
                     gTimerDebugIndexA1++;
                 }
             }
             gPrevTimeCbA1 = gCurrentTime;
         }
         if(gIsTxArrayPrinted == 0 && gTimerDebugIndexA1 >= (TX_DEBUG_ARRAY_SIZE/2))
         {
             uint32_t printArrayIndex = 0;
             for(printArrayIndex = 0; printArrayIndex<(TX_DEBUG_ARRAY_SIZE/2); printArrayIndex++)
             {
                 EnetAppUtils_print("SPEND0: %lu\r\n", gTimerArrayA0[printArrayIndex]);
                 EnetAppUtils_print("SPEND1: %lu\r\n", gTimerArrayA1[printArrayIndex]);
             }
             gIsTxArrayPrinted = 1;
         }
#endif
if (streamId == 0)
{
    #ifdef SUB_PROFILE
         uint32_t gPrevTime;
            gPrevTime = CycleCounterP_getCount32();
            // printing the jitter for submitTx
            if(gIsTxArrayPrinted == 0 && gTimerDebugIndex >= (TX_DEBUG_ARRAY_SIZE))
            {
                uint32_t printArrayIndex = 0;
                for(printArrayIndex = 0; printArrayIndex<(TX_DEBUG_ARRAY_SIZE); printArrayIndex++)
                {
                    EnetAppUtils_print("SP: %lu\r\n", gTimerArray[printArrayIndex]);
                }
                gIsTxArrayPrinted = 1;
            }
    #endif
}
#ifdef SUB_JITTER
        // printing the jitter for submitTx
        if(gIsTxArrayPrinted == 0 && gTimerDebugIndex >= (TX_DEBUG_ARRAY_SIZE))
        {
        	uint32_t printArrayIndex = 0;
        	for(printArrayIndex = 0; printArrayIndex<(TX_DEBUG_ARRAY_SIZE); printArrayIndex++)
			{
				EnetAppUtils_print("ST: %lu\r\n", gTimerArray[printArrayIndex]);
			}
			gIsTxArrayPrinted = 1;
        }
#endif
#if 0
        if(stopCapture == 1/*&& isIsrArrayCapture*/)
        {
        	uint32_t printArrayIndex = 0;
        	for(printArrayIndex = 0; printArrayIndex<(TX_DEBUG_ARRAY_SIZE); printArrayIndex++)
			{
				EnetAppUtils_print("ISR: %llu\r\n", gProfileArray[printArrayIndex]);
			}
        	isIsrArrayCapture = 0;
        	while(1);
        }
#endif
#ifdef CYCLEP_DEBUG
        if(gIsTxArrayPrinted == 0 && gTimerDebugIndex >= (TX_DEBUG_ARRAY_SIZE))
        {
        	uint32_t printArrayIndex = 0;
        	for(printArrayIndex = 0; printArrayIndex<(TX_DEBUG_ARRAY_SIZE); printArrayIndex++)
			{
				EnetAppUtils_print("TCB: %lu\r\n", gTimerArray[printArrayIndex]);
			}
			gIsTxArrayPrinted = 1;
        }
#endif
#if 0
        if(streamId == 0)
		{
			if((gProfileIndexEl >= (TX_DEBUG_ARRAY_SIZE) &&(isTxPrinted == 0)))
			{
				for(int32_t index =0; index<(TX_DEBUG_ARRAY_SIZE); index++)
				{
					EnetAppUtils_print("PF: %llu\r\n", gProfileArray[index]);
				}
				isTxPrinted = 1;
			}
		}
#endif
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
                     if (streamId <3)
                     {
                         priority = 3;
//                         payLoadLength = 1000;
                         payLoadLength = 1000;
                     }
                     else
                     {
                         priority =0;
                     }
					 frame->hdr.etherType = Enet_htons(ENETAPP_TEST_TX_ETHERTYPE);
                     frame->hdr.tci  = Enet_htons(ENETAPP_VLAN_TCI(priority, 0, 255));

                     pktInfo->sgList.list[0].segmentFilledLen = payLoadLength + sizeof(EthVlanFrameHeader);

                     //8bytes current timestamp(adding zeros for now) + streamId (1 byte) + 191bytes payload
                     tsValCurrentvalue = tsValCurrent + 125000;
                     memcpy(&frame->payload[0U], &tsValCurrentvalue, 8U);

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
    #if 1
            if (gIsPrintDone == 0 && streamId == 0)
            {
                EnetAppUtils_print("RxTs and CurrentTs values stored\r\n");
                ClockP_usleep(5000);
                int16_t timeStampIndex;
                for(timeStampIndex=10; timeStampIndex<MAX_RX_TIMESTAMPS_STORED-5020; timeStampIndex++)
                {
                    EnetAppUtils_print("[EL_RXTS_A]: %llu\r\n",
                            gEtherRingRxTs.etherRingTimeStampsRx[timeStampIndex]);
                }

                for(timeStampIndex=10; timeStampIndex<MAX_RX_TIMESTAMPS_STORED-5020; timeStampIndex++)
                {
                    EnetAppUtils_print("[EL_LAT_A]: %llu\r\n",
                            gEtherRingRxTs.etherRingCurrentTimeStamps[timeStampIndex]);
                }

                ClockP_usleep(1000);
                gIsPrintDone = 1;
                EnetAppUtils_print("\r\n\n");
                EnetAppUtils_print("----------ETHERRING DEMONSTRATION COMPLETED----------\r\n");
                EnetAppUtils_print("\r\n\n");
            }
    #endif
//        	stopCapture = 1;
#if 0
        	if(isTxPrinted == 0)
        	{
        	    for(int32_t index = 0; index<100 ;index++)
        	    {
        	    	volatile uint32_t startCountCycleP = CycleCounterP_getCount32();
        	    	uint32_t startCountClockP = ClockP_getTicks();
        	    	uint64_t startus = ClockP_getTimeUsec();
        	    	ClockP_sleep(1);
        	    	volatile uint32_t endCountCycleP = CycleCounterP_getCount32();
        	    	uint32_t endCountClockP = ClockP_getTicks();
        	    	uint64_t endus = ClockP_getTimeUsec();
        	    	EnetAppUtils_print("[LATER]CY_startTick: %u, endTick: %u, CL_startTick: %u, endTick: %u, CL_startTimeUs: %llu, endTimeUs: %llu\r\n",
        	    			startCountCycleP, endCountCycleP, startCountClockP, endCountClockP, startus, endus);
        	    }
        	    isTxPrinted = 1;
        	}
#endif
        }

		if (retVal != ENET_SOK)
		{
			EnetAppUtils_print("Etherring Tx submit failed\r\n");
		}
#if 1
        Enet_IoctlPrms prms;

        ENET_IOCTL_SET_OUT_ARGS(&prms, (void *)&tsValCurrent);
        CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP(ghCptsEtherRing, gCptsRegsEtherRing, &prms);
#endif
    }
}

void EnetApp_scheduleStreamD(void *stream_id)
{
    EnetDma_PktQ txSubmitQ;
    EthVlanFrame *frame;
    int32_t retVal = ENET_SOK;
    int8_t streamId = *(int8_t*)stream_id;
    uint64_t tsValCurrent = 0ULL;
    uint64_t tsValCurrentvalue = 0ULL;
    uint8_t multicastAddr[ENET_MAC_ADDR_LEN] = {0x01,0x00,0x5E,0x7F,0xFF,0x00};
    EnetDma_PktQ txFreeQ;
    EnetQueue_initQ(&txFreeQ);

    multicastAddr[ENET_MAC_ADDR_LEN-1] = gEtherRingStreamToMcast[streamId-3];


    int8_t priority = 0;

#ifdef PTP_ENABLED
    EnetApp_waitSystemStable();

    while(gptpmasterclock_init(NULL)){
        UB_LOG(UBL_INFO,"Waiting for tsn_gptpd to be ready...\n");
        CB_USLEEP(100000);
    }
#endif

    #ifdef WITH_EST_CONFIG
        wait_est_configured();
    #endif

    // EnetAppUtils_print("Schedule started for StreamId: %d\r\n", streamId);

    isStreamsEnabled = 1;
    while(true)
    {
        SemaphoreP_pend(&gEnetAppCfg.streamSemObj[streamId], SystemP_WAIT_FOREVER);

        EnetQueue_initQ(&txSubmitQ);

        if (EnetQueue_getQCount(&gEnetAppCfg.txFreePktInfoQ) > 0 )
        {
            uint32_t classDstreamIndex = 0;
            for(classDstreamIndex = 0; classDstreamIndex<3 ;classDstreamIndex++)
            {
                if (gPacketCountClassStream[classDstreamIndex+3] < gSendPacketsClassStream[3+classDstreamIndex])
                {
                    multicastAddr[ENET_MAC_ADDR_LEN-1] = gEtherRingStreamToMcast[classDstreamIndex];
                    EnetDma_Pkt *pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&gEnetAppCfg.txFreePktInfoQ);

                    if (pktInfo != NULL)
                    {
                        frame = (EthVlanFrame *)pktInfo->sgList.list[0].bufPtr;
                        memcpy(frame->hdr.dstMac, multicastAddr, 6U);
                        memcpy(frame->hdr.srcMac, gEnetAppCfg.macAddr, 6U);
                        frame->hdr.tpid = Enet_htons(ENETAPP_VLAN_TPID);
                        priority = 2;
                        payLoadLength = 1450;
                        frame->hdr.etherType = Enet_htons(ENETAPP_TEST_TX_ETHERTYPE);
                        frame->hdr.tci  = Enet_htons(ENETAPP_VLAN_TCI(priority, 0, 255));
                        pktInfo->sgList.list[0].segmentFilledLen = payLoadLength + sizeof(EthVlanFrameHeader);

                        //8bytes current timestamp(adding zeros for now) + streamId (1 byte) + 191bytes payload
                        tsValCurrentvalue = tsValCurrent + 1000000;
                        memcpy(&frame->payload[0U], &tsValCurrentvalue, 8U);

                        memset(&frame->payload[8U], (uint8_t)(3 + classDstreamIndex), 1);
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
    // gEtherRingprintLock = EnetOsal_createMutex();

    EnetAppUtils_print("ClassA stream count:%d\r\n", gEnetAppCfg.numClassAStreams);
    EnetAppUtils_print("ClassD stream count:%d\r\n", gEnetAppCfg.numClassDStreams);
    for(stream_id = 0 ;stream_id<(gEnetAppCfg.numClassAStreams) ;stream_id++)
    {
        status = SemaphoreP_constructBinary(&gEnetAppCfg.streamSemObj[stream_id], 0);
        DebugP_assert(SystemP_SUCCESS == status);
        TaskP_Params_init(&taskParamsStreamGen);
        taskParamsStreamGen.priority       = TX_TASK_PRIORITY;
        taskParamsStreamGen.stack          = gEnetAppStreamTaskStack[stream_id];
        taskParamsStreamGen.stackSize      = sizeof(gEnetAppStreamTaskStack[stream_id]);
        taskParamsStreamGen.args           = (void*)&gEtherRingStreamIdpool[stream_id];
        taskParamsStreamGen.name           = "ClassA Task";
        taskParamsStreamGen.taskMain       = &EnetApp_scheduleStream;

        status = TaskP_construct(&gEnetAppCfg.streamTaskObj[stream_id], &taskParamsStreamGen);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    for(stream_id = 3 ;stream_id<(3 + gEnetAppCfg.numClassDStreams) ;stream_id++)
    {
        status = SemaphoreP_constructBinary(&gEnetAppCfg.streamSemObj[stream_id], 0);
        DebugP_assert(SystemP_SUCCESS == status);
        TaskP_Params_init(&taskParamsStreamGen);
        taskParamsStreamGen.priority       = 10;
        taskParamsStreamGen.stack          = gEnetAppStreamTaskStack[stream_id];
        taskParamsStreamGen.stackSize      = sizeof(gEnetAppStreamTaskStack[stream_id]);
        taskParamsStreamGen.args           = (void*)&gEtherRingStreamIdpool[stream_id];
        taskParamsStreamGen.name           = "ClassD Task";
        taskParamsStreamGen.taskMain       = &EnetApp_scheduleStreamD;

        status = TaskP_construct(&gEnetAppCfg.streamTaskObj[stream_id], &taskParamsStreamGen);
        DebugP_assert(SystemP_SUCCESS == status);
    }
//     EnetApp_startHwTimer();
}

void EnetApp_clearLookupTable()
{
    while(true)
    {
        SemaphoreP_pend(&gEnetAppCfg.etherringSemObj, SystemP_WAIT_FOREVER);
        EtherRing_clearLookupPollTask();
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

void avbTimerIsrClassA(void)
{
        static int counter = 0;
        static uint32_t etherRingCounter  = 0;
        int32_t stream_id;
#ifdef CYCLEP_MAX_MIN_JITTER
        if(isStreamsEnabled)
        {
			uint32_t gCurrentTime = CycleCounterP_getCount32();
			if(gPrevTimeCb != 0)
			{
					uint32_t timeDiff;
					if(gCurrentTime > gPrevTimeCb)
					{
						timeDiff = (gCurrentTime-gPrevTimeCb)/400;
					}
					else
					{
						timeDiff = ((0xFFFFFFFF -gPrevTimeCb + gCurrentTime)&0xFFFFFFFF)/400;
					}
					if(timeDiff < gMinTimerJitter)
					{
						gMinTimerJitter = timeDiff;
					}
					else if(timeDiff > gMaxTimerJitter)
					{
						gMaxTimerJitter = timeDiff;
					}
			}
			gPrevTimeCb = gCurrentTime;
        }
#endif
#ifdef CYCLEP_DEBUG
        	uint32_t gCurrentTime = CycleCounterP_getCount32();
        if(gPrevTimeCb != 0)
        {
			if(isStreamsEnabled && (gTimerDebugIndex < TX_DEBUG_ARRAY_SIZE))
			{
				if(gCurrentTime > gPrevTimeCb)
				{
					gTimerArray[gTimerDebugIndex] = gCurrentTime-gPrevTimeCb;
				}
				else
				{
					gTimerArray[gTimerDebugIndex] = (0xFFFFFFFF -gPrevTimeCb + gCurrentTime)&0xFFFFFFFF;
				}
				gTimerDebugIndex++;
			}
        }
        gPrevTimeCb = gCurrentTime;
#endif
        if(isStreamsEnabled)
        {
                for(stream_id = 0 ;stream_id<(gEnetAppCfg.numClassAStreams); stream_id++)
                {
                    SemaphoreP_post(&gEnetAppCfg.streamSemObj[stream_id]);
                }

                counter++;
                etherRingCounter++;
                if (counter % 8 == 0)
                {
                    for(stream_id = 3 ;stream_id<(3 + gEnetAppCfg.numClassDStreams); stream_id++)
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
        EtherRing_dmaRxIsr();
}


#if 0
void EnetApp_TxRetrievePoll(void *args)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    int32_t status = ENET_SOK;

    /* Call the driver's periodic polling function */
    EtherRingApp_TxRtrvPollTaskInfo *pPollTxRtrvTaskInfo = &pollTxRtrvTaskInfo;
    EnetQueue_initQ(&txFreeQ);

    while (!pPollTxRtrvTaskInfo->shutDownFlag)
    {
        SemaphoreP_pend(&pPollTxRtrvTaskInfo->sem, SystemP_WAIT_FOREVER);

        status = EtherRing_retrieveTxPktQ(gEnetAppCfg.hEtherRing, &txFreeQ);

        if (status == ENET_SOK)
        {
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
            EnetAppUtils_print("retrieveFreeTxPkts() failed to retrieve pkts: %d\r\n",
                               status);
        }
    }
    SemaphoreP_post(&pPollTxRtrvTaskInfo->shutDownSemObj);
}

void EnetApp_postTxRetrievePollLink(ClockP_Object *clkObj, void *arg)
{
    if (arg != NULL)
    {
        SemaphoreP_Object *hPollSem = (SemaphoreP_Object *) arg;
        SemaphoreP_post(hPollSem);
    }
}

int32_t EnetApp_createTxRetrievePollTask()
{
    TaskP_Params params;
    int32_t status;
    ClockP_Params clkPrms;

    EtherRingApp_TxRtrvPollTaskInfo *pPollTxRtrvTaskInfo = &pollTxRtrvTaskInfo;

    if (NULL != pPollTxRtrvTaskInfo)
    {
        /*Initialize semaphore to call synchronize the poll function with a timer*/
        status = SemaphoreP_constructBinary(&pPollTxRtrvTaskInfo->sem, 0U);
        EnetAppUtils_assert(status == SystemP_SUCCESS);

        /*Initialize semaphore to call synchronize the poll function with a timer*/
        status = SemaphoreP_constructBinary(&pPollTxRtrvTaskInfo->shutDownSemObj, 0U);
        EnetAppUtils_assert(status == SystemP_SUCCESS);

        /* Initialize the poll function as a thread */
        TaskP_Params_init(&params);
        params.name           = "retrieve_tx_poll_task";
        params.priority       = TX_RETRIEVE_TASK_PRIORITY;
        params.stack          = pPollTxRtrvTaskInfo->gEnetAppTaskStackPolling;
        params.stackSize      = sizeof(pPollTxRtrvTaskInfo->gEnetAppTaskStackPolling);
        params.args           = (void*)&gEnetAppCfg;
        params.taskMain       = &EnetApp_TxRetrievePoll;

        status = TaskP_construct(&pPollTxRtrvTaskInfo->task, &params);
        EnetAppUtils_assert(status == SystemP_SUCCESS);

        ClockP_Params_init(&clkPrms);
        clkPrms.start     = 0;
        clkPrms.period    = 1;
        clkPrms.args      = &pPollTxRtrvTaskInfo->sem; // make a proper semaphore structure for this.
        clkPrms.callback  = &EnetApp_postTxRetrievePollLink;
        clkPrms.timeout   = 1;

        /* Creating timer and setting timer callback function*/
        status = ClockP_construct(&pPollTxRtrvTaskInfo->pollLinkClkObj, &clkPrms);
        if (status == SystemP_SUCCESS)
        {
            /* Set timer expiry time in OS ticks */
            ClockP_setTimeout(&pPollTxRtrvTaskInfo->pollLinkClkObj, 1);
            ClockP_start(&pPollTxRtrvTaskInfo->pollLinkClkObj);
        }
        else
        {
            EnetAppUtils_assert(status == SystemP_SUCCESS);
        }

        /* Filter not defined */
        /* Inform the world that we are operational. */
        EnetAppUtils_print("TX Retrieve task started successfully\r\n");

        return 0;
    }
    else
    {
        return -1;
    }
}
#endif
