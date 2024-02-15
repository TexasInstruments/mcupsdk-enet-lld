/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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
 * \file  enet_udma.c
 *
 * \brief This file contains the implementation of the Enet data path with UDMA.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>

#include <enet_cfg.h>
#include <include/core/enet_base.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_soc.h>
#include <include/core/enet_per.h>
#include <include/core/enet_queue.h>
#include <priv/core/enet_base_priv.h>
#include <priv/core/enet_trace_priv.h>
#include <include/common/enet_osal_dflt.h>
#include <include/common/enet_utils_dflt.h>
#include <include/core/enet_rm.h>
#include <include/core/enet_dma.h>

#include "enet_udma_priv.h"
#include <drivers/sciclient.h>

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 0) && (UDMA_SOC_CFG_LCDMA_PRESENT == 0)
#error "UDMA Type not supported)"
#endif

/* TODO: Cleanup Hack. These defines should be moved to public interface udma.h */
#include <drivers/udma/udma_priv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t EnetUdma_checkTxChParams(EnetUdma_OpenTxChPrms *pTxChPrms);

static int32_t EnetUdma_checkRxFlowParams(EnetUdma_OpenRxFlowPrms *pRxFlowPrms);

static void EnetUdma_drainIsrCq(EnetQ *dstQ,
                               EnetQ *isrCq);

#if (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
static uint32_t EnetUdma_getMappedRxChNum(Enet_Type enetType,
                                          uint32_t instId,
                                          uint32_t chIdx);
#endif
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Data Path Common Function Definitions             */
/* ========================================================================== */

void EnetDma_initRxChParams(void *pRxChCfg)
{
    EnetUdma_OpenRxFlowPrms *pRxFlowPrms = (EnetUdma_OpenRxFlowPrms *)pRxChCfg;
    Udma_FlowPrms flowPrms;

    memset(&flowPrms, 0, sizeof(flowPrms));
    UdmaFlowPrms_init(&flowPrms, UDMA_CH_TYPE_RX);

    pRxFlowPrms->flowPrms.psInfoPresent = TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_PSINFO_PRESENT;
    pRxFlowPrms->flowPrms.einfoPresent  = TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_EINFO_PRESENT;
    pRxFlowPrms->flowPrms.sopOffset    = flowPrms.sopOffset;
    pRxFlowPrms->flowPrms.defaultRxCQ  = flowPrms.defaultRxCQ;
    pRxFlowPrms->flowPrms.srcTagHi     = flowPrms.srcTagHi;
    pRxFlowPrms->flowPrms.srcTagLo     = flowPrms.srcTagLo;
    pRxFlowPrms->flowPrms.srcTagHiSel  = flowPrms.srcTagHiSel;
    pRxFlowPrms->flowPrms.srcTagLoSel  = TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_SRC_SELECT_SRC_TAG;
    pRxFlowPrms->flowPrms.destTagHi    = flowPrms.destTagHi;
    pRxFlowPrms->flowPrms.destTagLo    = flowPrms.destTagLo;
    pRxFlowPrms->flowPrms.destTagHiSel = flowPrms.destTagHiSel;
    pRxFlowPrms->flowPrms.destTagLoSel = flowPrms.destTagLoSel;
    pRxFlowPrms->flowPrms.sizeThreshEn =  TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_SIZE_THRESH_MAX;
    pRxFlowPrms->chIdx =  0U;

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    pRxFlowPrms->udmaChPrms.fqRingPrms.orderId    = UDMA_DEFAULT_RING_ORDER_ID;
    /* Use message mode for Rx FQ so at channel/flow teardown time UDMA IP can push
     * inflow descriptors back into the ring.*/
    pRxFlowPrms->udmaChPrms.fqRingPrms.mode       = TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE;
    pRxFlowPrms->udmaChPrms.fqRingPrms.useRingMon = false;
#endif

    pRxFlowPrms->udmaChPrms.cqRingPrms.orderId    = UDMA_DEFAULT_RING_ORDER_ID;
    pRxFlowPrms->udmaChPrms.cqRingPrms.mode       = TISCI_MSG_VALUE_RM_RING_MODE_RING;
    pRxFlowPrms->udmaChPrms.cqRingPrms.useRingMon = false;
    pRxFlowPrms->udmaChPrms.cqRingPrms.ringMonCfg.mode = 0U;
    pRxFlowPrms->udmaChPrms.cqRingPrms.ringMonCfg.data0 = 0U;
    pRxFlowPrms->udmaChPrms.cqRingPrms.ringMonCfg.data1 = 0U;

    /* We initialize callback function pointers to NULL so we check app pass
     * uninitialized  value */
    pRxFlowPrms->hUdmaDrv        = NULL;
    pRxFlowPrms->ringMemAllocFxn = NULL;
    pRxFlowPrms->ringMemFreeFxn  = NULL;
    pRxFlowPrms->dmaDescAllocFxn = NULL;
    pRxFlowPrms->dmaDescFreeFxn  = NULL;

    pRxFlowPrms->useGlobalEvt = false;
    pRxFlowPrms->notifyCb = NULL;
    pRxFlowPrms->cbArg   = NULL;

    pRxFlowPrms->useProxy = false;

    /* auto-reclaim initialization */
    pRxFlowPrms->autoReclaimPrms.enableFlag       = false;
    pRxFlowPrms->autoReclaimPrms.hDmaDescPool          = NULL;
    pRxFlowPrms->autoReclaimPrms.hReclaimRing = NULL;

    return;
}

EnetDma_RxChHandle EnetDma_openRxCh(EnetDma_Handle hDma,
                                    const void *pRxChCfg)
{
    EnetPer_Handle hPer = hDma->hPer;
    EnetUdma_OpenRxFlowPrms *pRxFlowPrms = (EnetUdma_OpenRxFlowPrms *)pRxChCfg;
    int32_t retVal;
    Udma_RingHandle ringHandle;
    Udma_FlowPrms flowPrms;
    Udma_RingPrms ringPrms;
    EnetUdma_RxFlowObj *pRxFlow;
    EnetUdma_udmaInfo udmaInfo;
    EnetUdma_ringAllocInfo ringAllocInfo;
    uint32_t eventType, flowStart;
    EnetUdma_AutoReclaimPrms *reclaimPrms = &pRxFlowPrms->autoReclaimPrms;
    bool allocFlowObj = false, allocCqRing = false;
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    EnetUdma_UdmaRingPrms *pFqRingPrms = &pRxFlowPrms->udmaChPrms.fqRingPrms;
    Udma_RingHandle dropFqRing = NULL;
    bool allocFqRing = false, allocProxy   = false;
    bool allocRingMon = false, allocDropFqRing = false;
#endif
    bool flowAttachFlag = false;
    uintptr_t intrKey;

    intrKey = EnetOsal_disableAllIntr();

    /* Set to NULL so if error condition we return NULL */
    pRxFlow = NULL;

    /* Error check */
    retVal = EnetUdma_checkRxFlowParams(pRxFlowPrms);

    if (UDMA_SOK == retVal)
    {
        pRxFlow = EnetUdma_memMgrAllocRxFlowObj();

        if (pRxFlow == NULL)
        {
            ENETTRACE_ERR("[Enet UDMA] Memory allocation for Rx flow object failed !!\n");
            retVal = UDMA_EALLOC;
        }
        else
        {
            allocFlowObj = true;
            memset(pRxFlow, 0U, sizeof(*pRxFlow));
            /* Save Flow config */
            pRxFlow->rxFlowPrms = *pRxFlowPrms;
            /* Initialize local variables to be used in reminder of this function  */

            pRxFlow->hUdmaDrv  = pRxFlowPrms->hUdmaDrv;
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
            dropFqRing         = &pRxFlow->rxFlowMemObj.dropFqRingObj;
            pRxFlow->fqRing    = &pRxFlow->rxFlowMemObj.fqRingObj;
#endif
            pRxFlow->hUdmaFlow = &pRxFlow->rxFlowMemObj.flowUdmaObj;
            pRxFlow->hUdmaEvt  = &pRxFlow->rxFlowMemObj.udmaEvtObj;
            pRxFlow->useGlobalEvt = pRxFlowPrms->useGlobalEvt;

            if (!reclaimPrms->enableFlag)
            {
                pRxFlow->cqRing       = &pRxFlow->rxFlowMemObj.cqRingObj;
                pRxFlow->hDmaDescPool = &pRxFlow->rxFlowMemObj.rxReadyDmaDescQ;
            }
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
            else
            {
#if ENET_CFG_IS_ON(DEV_ERROR)
                Enet_devAssert(NULL != reclaimPrms->hDmaDescPool);
                Enet_devAssert(NULL != reclaimPrms->hReclaimRing);
                /* Ring mode should be message as both RX flow (pushes to CQ) and
                 * TX channel (pops from FQ) uses this ring */
                Enet_devAssert(TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE == Udma_ringGetMode(reclaimPrms->hReclaimRing));
                /* Filtering of protocol specific and extended info should be enabled as this
                 * information might corrupt TX channel (as the TX PSI and EINFO gets mapped to
                 * RX flow PSI and EINFO which has entirely different meaning)*/
                Enet_devAssert(pRxFlowPrms->flowPrms.psInfoPresent ==
                            TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_PSINFO_PRESENT);
                Enet_devAssert(pRxFlowPrms->flowPrms.einfoPresent ==
                            TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_EINFO_PRESENT);
                /* Notify callback should be NULL when auto recycle is enabled as the
                 * CQ event should be disabled */
                Enet_devAssert(pRxFlowPrms->notifyCb == NULL);
#endif
                pRxFlow->cqRing       = reclaimPrms->hReclaimRing;
                pRxFlow->hDmaDescPool = reclaimPrms->hDmaDescPool;
            }
#endif
        }
    }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    if (UDMA_SOK == retVal)
    {
        UdmaRingPrms_init(&ringPrms);
        ringPrms.elemCnt = pRxFlowPrms->numRxPkts;
        ringPrms.orderId = pRxFlowPrms->udmaChPrms.fqRingPrms.orderId;
        ringPrms.mode    = pRxFlowPrms->udmaChPrms.fqRingPrms.mode;

        ringAllocInfo.allocRingMem    = true;
        ringAllocInfo.ringMemAllocFxn = pRxFlowPrms->ringMemAllocFxn;
        ringAllocInfo.ringMemFreeFxn  = pRxFlowPrms->ringMemFreeFxn;
        ringAllocInfo.cbArg          = pRxFlowPrms->cbArg;
        ringAllocInfo.ringNum         = UDMA_RING_ANY;

        retVal = EnetUdma_allocRing(pRxFlow->hUdmaDrv,
                                   pRxFlow->fqRing,
                                   &ringPrms,
                                   &ringAllocInfo);
        ENETTRACE_ERR_IF((retVal != UDMA_SOK),
                           "[Enet UDMA] Fq ring allocation failed !!: 0x%x\n", retVal);
        if (UDMA_SOK == retVal)
        {
            allocFqRing = true;
        }
    }

    if ((UDMA_SOK == retVal) && (pFqRingPrms->useRingMon))
    {
        pRxFlow->hUdmaRingMon = EnetUdma_allocRingMon(pRxFlow->hUdmaDrv,
                                                     pRxFlow->fqRing,
                                                     pRxFlowPrms->numRxPkts,
                                                     &pFqRingPrms->ringMonCfg);
        if (NULL == pRxFlow->hUdmaRingMon)
        {
            ENETTRACE_ERR("[Enet UDMA] RX FQ Ring Mon Config failed !!\n");
            retVal = UDMA_EALLOC;
        }
        else
        {
            allocRingMon = true;
        }
    }
#endif

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    if ((UDMA_SOK == retVal) &&
        (TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_SIZE_THRESH_MAX == pRxFlowPrms->flowPrms.sizeThreshEn))
    {
        /* PDK-3723 We use drop ring for routing packets larger than flow max. allowed size
         * to avoid invalid data getting spilled into multiple descriptors. */
        UdmaRingPrms_init(&ringPrms);
        ringPrms.elemCnt = 1U;
        ringPrms.mode    = TISCI_MSG_VALUE_RM_RING_MODE_RING;

        ringAllocInfo.allocRingMem    = true;
        ringAllocInfo.ringMemAllocFxn = pRxFlowPrms->ringMemAllocFxn;
        ringAllocInfo.ringMemFreeFxn  = pRxFlowPrms->ringMemFreeFxn;
        ringAllocInfo.cbArg          = pRxFlowPrms->cbArg;
        ringAllocInfo.ringNum         = UDMA_RING_ANY;

        retVal = EnetUdma_allocRing(pRxFlow->hUdmaDrv,
                                   dropFqRing,
                                   &ringPrms,
                                   &ringAllocInfo);
        ENETTRACE_ERR_IF((retVal != UDMA_SOK),
                           "[Enet UDMA] Drop Fq ring allocation failed !!: 0x%x\n", retVal);
        if (UDMA_SOK == retVal)
        {
            allocDropFqRing = true;
        }
    }
#endif

    if (UDMA_SOK == retVal)
    {
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        pRxFlow->hUdmaProxy = NULL;
        if (pRxFlowPrms->useProxy)
        {
            retVal = EnetUdma_allocProxy(pRxFlow->hUdmaDrv, &pRxFlow->rxFlowMemObj.proxyObj, pRxFlow->fqRing);
            if (UDMA_SOK == retVal)
            {
                allocProxy          = true;
                pRxFlow->hUdmaProxy = &pRxFlow->rxFlowMemObj.proxyObj;
            }
        }
#endif
    }

    if ((!reclaimPrms->enableFlag) && (UDMA_SOK == retVal))
    {
        UdmaRingPrms_init(&ringPrms);
        ringPrms.elemCnt = pRxFlowPrms->numRxPkts;
        ringPrms.orderId = pRxFlowPrms->udmaChPrms.cqRingPrms.orderId;
        ringPrms.mode    = pRxFlowPrms->udmaChPrms.cqRingPrms.mode;

        ringAllocInfo.allocRingMem    = true;
        ringAllocInfo.ringMemAllocFxn = pRxFlowPrms->ringMemAllocFxn;
        ringAllocInfo.ringMemFreeFxn  = pRxFlowPrms->ringMemFreeFxn;
        ringAllocInfo.cbArg          = pRxFlowPrms->cbArg;
        ringAllocInfo.ringNum         = UDMA_RING_ANY;
        ringAllocInfo.enetType        = hDma->enetType;
        ringAllocInfo.instId          = hDma->instId;
        ringAllocInfo.mappedChNum     = Udma_chGetNum(&hDma->rxChObj[pRxFlowPrms->chIdx].udmaChObj);
        ringAllocInfo.transferDir     = ENET_UDMA_DIR_RX;

        retVal = EnetUdma_allocRing(pRxFlow->hUdmaDrv,
                                   pRxFlow->cqRing,
                                   &ringPrms,
                                   &ringAllocInfo);
        ENETTRACE_ERR_IF((retVal != UDMA_SOK),
                           "[Enet UDMA] Cq ring allocation failed !!: 0x%x\n", retVal);
        if (UDMA_SOK == retVal)
        {
            allocCqRing = true;
        }
    }

    if (UDMA_SOK == retVal)
    {
        memset(&flowPrms, 0, sizeof(flowPrms));
        UdmaFlowPrms_init(&flowPrms, UDMA_CH_TYPE_RX);

        /* UdmaFlowPrms_init sets errorHandling to retry(1) by default,
         * in case of descriptor/buffer starvation UDMA retries unless it
         * gets a descriptor. In case of multi flow, this results in bottom
         * of FIFO drop, to avoid this errorHandling must be set to drop(0).
         */
        flowPrms.errorHandling = TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_ERR_DROP;

        /* CPPI TX Status Data Word [0..3] are mapped to Host Packet Descriptor Protocol Specific
         * Words if RFLOW[a]_RFA.rx_psinfo_present = 1 and RFLOW[a]_RFA.rx_ps_location = 0 (end) */
        flowPrms.psInfoPresent = pRxFlowPrms->flowPrms.psInfoPresent;
        /* Extended packet info block present CPPI TX info words are mapped */
        flowPrms.einfoPresent  = pRxFlowPrms->flowPrms.einfoPresent;
        flowPrms.psLocation    = TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_PS_END_PD;

        /* Set Src tag low selection to get port number of the received packet (or)
         * flow Id (or) config tag from CPPI. */
        flowPrms.srcTagLoSel   = pRxFlowPrms->flowPrms.srcTagLoSel;

        flowPrms.defaultRxCQ = Udma_ringGetNum(pRxFlow->cqRing);
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        if (TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_SIZE_THRESH_MAX == pRxFlowPrms->flowPrms.sizeThreshEn)
        {
            /* Enable Rx Packet Size Threshold so we can route packets larger than
             * pRxFlowPrms->rxFlowMtu to drop ring */
            flowPrms.sizeThreshEn = pRxFlowPrms->flowPrms.sizeThreshEn;
            ringHandle = dropFqRing;
        }
        else
        {
            ringHandle = pRxFlow->fqRing;
        }
#elif (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
        ringHandle = pRxFlow->cqRing;
#endif

        /* ICSSG peripherals don't support this feature, so explicitly disable in spite of
         * applications request as this parameter could be easy to miss */
        if (Enet_isIcssFamily(hPer->enetType))
        {
            flowPrms.sizeThreshEn = 0U;
        }
        /* Align the RX MTU size to avoid truncation. Refer to ENET_UDMA_RXMTU_ALIGN description for
         * more details */
        flowPrms.sizeThresh0 = ENET_UTILS_ALIGN(pRxFlowPrms->rxFlowMtu, ENET_UDMA_RXMTU_ALIGN);
        flowPrms.sizeThresh1 = ENET_UTILS_ALIGN(ENET_UDMA_JUMBO_PACKET_SIZE, ENET_UDMA_RXMTU_ALIGN);
        flowPrms.sizeThresh2 = ENET_UTILS_ALIGN(ENET_UDMA_JUMBO_PACKET_SIZE, ENET_UDMA_RXMTU_ALIGN);

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        flowPrms.fdq0Sz0Qnum = Udma_ringGetNum(pRxFlow->fqRing);
#elif (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
        flowPrms.fdq0Sz0Qnum = Udma_ringGetNum(pRxFlow->cqRing);
#endif
        flowPrms.fdq0Sz1Qnum = Udma_ringGetNum(ringHandle);
        flowPrms.fdq0Sz2Qnum = Udma_ringGetNum(ringHandle);
        flowPrms.fdq0Sz3Qnum = Udma_ringGetNum(ringHandle);
        flowPrms.fdq1Qnum    = Udma_ringGetNum(ringHandle);
        flowPrms.fdq2Qnum    = Udma_ringGetNum(ringHandle);
        flowPrms.fdq3Qnum    = Udma_ringGetNum(ringHandle);

        flowStart = pRxFlowPrms->startIdx + pRxFlowPrms->flowIdx;

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        /* Attach and configure the flows */
        retVal = Udma_flowAttach(pRxFlow->hUdmaDrv,
                                 pRxFlow->hUdmaFlow,
                                 flowStart,
                                 1U /* flowCnt */);
#elif (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
        Udma_FlowAllocMappedPrms flowAllocMappedPrms;

        flowAllocMappedPrms.mappedChNum = Udma_chGetNum(&hDma->rxChObj[pRxFlowPrms->chIdx].udmaChObj);
        if (Enet_isCpswFamily(hDma->enetType))
        {
            flowAllocMappedPrms.mappedFlowGrp   = UDMA_MAPPED_RX_GROUP_CPSW;
        }
        else if (hDma->enetType == ENET_ICSSG_SWITCH)
        {
            if (0U == hDma->instId)
            {
                flowAllocMappedPrms.mappedFlowGrp   = UDMA_MAPPED_RX_GROUP_ICSSG_0;
            }
            else if (1U == hDma->instId)
            {
                flowAllocMappedPrms.mappedFlowGrp   = UDMA_MAPPED_RX_GROUP_ICSSG_1;
            }
            else
            {
                Enet_assert(false);
            }
        }
        else if (hDma->enetType == ENET_ICSSG_DUALMAC)
        {
            if ((0U == hDma->instId) || (1U == hDma->instId))
            {
                flowAllocMappedPrms.mappedFlowGrp   = UDMA_MAPPED_RX_GROUP_ICSSG_0;
            }
            else if ((2U == hDma->instId) || (3U == hDma->instId))
            {
                flowAllocMappedPrms.mappedFlowGrp   = UDMA_MAPPED_RX_GROUP_ICSSG_1;
            }
            else
            {
                Enet_assert(false);
            }
        }
        else
        {
            Enet_assert(false);
        }

        /* Attach and configure the flows */
        retVal = Udma_flowAttachMapped(pRxFlow->hUdmaDrv,
                                 pRxFlow->hUdmaFlow,
                                 flowStart, /* mappepdFlowNum */
                                 &flowAllocMappedPrms);
#endif
        ENETTRACE_ERR_IF((retVal != UDMA_SOK),
                           "[Enet UDMA] Flow attach failed!!: 0x%x\n", retVal);
        if (UDMA_SOK == retVal)
        {
            flowAttachFlag = true;
            retVal         = Udma_flowConfig(pRxFlow->hUdmaFlow,
                                             0U, /* Flow Index */
                                             &flowPrms);
        }
    }

    if (UDMA_SOK == retVal)
    {
        /* Initialize the completion queue.
         * Must be done before enabling CQ events */
        EnetQueue_initQ(&pRxFlow->cqIsrQ);
    }

    if (UDMA_SOK == retVal)
    {
        if (!reclaimPrms->enableFlag)
        {
            EnetUdma_initRxFreeDescQ(pRxFlow);
        }

        ENETTRACE_VERBOSE_IF((pRxFlowPrms->notifyCb == NULL),
                    "[Enet UDMA] Event callback function is null !!\n");
        if (pRxFlowPrms->notifyCb != NULL)
        {
            eventType   = UDMA_EVENT_TYPE_RING;
            ringHandle = pRxFlow->cqRing;
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
            if (pFqRingPrms->useRingMon && allocRingMon)
            {
                /* If ring mon is enabled on FQ, we register event for FQ low
                 * threshold and not CQ */
                eventType   = UDMA_EVENT_TYPE_RING_MON;
                ringHandle = pRxFlow->fqRing;
            }
#endif

            EnetUdma_getRxFlowUdmaInfo(pRxFlow, &udmaInfo);
            retVal = EnetUdma_registerEvent(&udmaInfo,
                                           ringHandle,
                                           EnetUdma_rxCqIsr,
                                           pRxFlow,
                                           eventType);
            ENETTRACE_ERR_IF((retVal != UDMA_SOK),
                           "[Enet UDMA] Rx Flow Open: event registration failed!!: 0x%x\n", retVal);
            if (ENET_SOK == retVal)
            {
                pRxFlow->evtInitFlag = true;
            }
        }
    }

    if (UDMA_SOK != retVal)
    {
        if ((pRxFlow != NULL) && pRxFlow->evtInitFlag)
        {
            retVal = EnetUdma_unregisterEvent(pRxFlow->hUdmaEvt);
            Enet_assert(UDMA_SOK == retVal);
        }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        /* Error. Free-up resource if allocated */
        if (allocFqRing)
        {
            retVal = EnetUdma_freeRing(pRxFlow->fqRing,
                                      pRxFlow->rxFlowPrms.numRxPkts,
                                      pRxFlow->rxFlowPrms.ringMemFreeFxn,
                                      &pRxFlow->rxFlowPrms.cbArg);
            Enet_assert(UDMA_SOK == retVal);
        }

        if (allocDropFqRing)
        {
            retVal = EnetUdma_freeRing(&pRxFlow->rxFlowMemObj.dropFqRingObj,
                                      1U,
                                      pRxFlow->rxFlowPrms.ringMemFreeFxn,
                                      &pRxFlow->rxFlowPrms.cbArg);
            Enet_assert(UDMA_SOK == retVal);
        }
#endif

        if (allocCqRing)
        {
            /* We free cqRing without retrieving packets here as packets are
             * not submitted by application yet so there wouldn't be any packets
             * in the cqRing */
            retVal = EnetUdma_freeRing(pRxFlow->cqRing,
                                      pRxFlow->rxFlowPrms.numRxPkts,
                                      pRxFlow->rxFlowPrms.ringMemFreeFxn,
                                      &pRxFlow->rxFlowPrms.cbArg);
            Enet_assert(UDMA_SOK == retVal);
        }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        if (allocRingMon)
        {
            retVal = EnetUdma_freeRingMon(pRxFlow->hUdmaRingMon);
            Enet_assert(UDMA_SOK == retVal);
        }
#endif

        if (flowAttachFlag)
        {
            retVal = Udma_flowDetach(pRxFlow->hUdmaFlow);
            Enet_assert(UDMA_SOK == retVal);

            /* Free up ready DMA descriptors */
            if (!reclaimPrms->enableFlag)
            {
                EnetUdma_deInitRxFreeDescQ(pRxFlow);
            }
        }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        if (allocProxy)
        {
            retVal = EnetUdma_freeProxy(pRxFlow->hUdmaProxy);
            Enet_assert(retVal == UDMA_SOK);
            pRxFlow->hUdmaProxy = NULL;
        }
#endif

        /* Free the rxFlowObj last */
        if (allocFlowObj)
        {
            EnetUdma_memMgrFreeRxFlowObj(pRxFlow);
        }

        /* As flow open is failed return NULL */
        pRxFlow = NULL;
    }
    else
    {
        pRxFlow->initFlag    = true;
        pRxFlow->hDma = hDma;
    }

    EnetOsal_restoreAllIntr(intrKey);

    return pRxFlow;
}

int32_t EnetDma_closeRxCh(EnetDma_RxChHandle hRxFlow,
                            EnetDma_PktQ *pFqPktInfoQ,
                            EnetDma_PktQ *pCqPktInfoQ)
{
    int32_t retVal = UDMA_SOK;
    uintptr_t intrKey;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hRxFlow) ||
        (NULL == pFqPktInfoQ) ||
        (NULL == pCqPktInfoQ))
    {
        ENETTRACE_ERR_IF((NULL == hRxFlow), "[Enet UDMA] hRxFlow is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == pFqPktInfoQ), "[Enet UDMA] pFqPktInfoQ is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == pFqPktInfoQ), "[Enet UDMA] pFqPktInfoQ is NULL!!\n");
        Enet_assert(FALSE);
        retVal = UDMA_EBADARGS;
    }
    else
#endif
    {
        intrKey = EnetOsal_disableAllIntr();

        /* Error check */
        if (hRxFlow == NULL)
        {
            retVal = UDMA_EBADARGS;
        }

        if (UDMA_SOK == retVal)
        {
            if (hRxFlow->evtInitFlag)
            {
                /* Disable event */
                retVal = EnetDma_disableRxEvent(hRxFlow);
                Enet_assert(UDMA_SOK == retVal);
            }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
            /* Flush packets in Rx FQ - retrieve any packets that haven't been
             * processed yet */
            retVal += EnetUdma_flushRxFlowRing(hRxFlow, hRxFlow->fqRing, pFqPktInfoQ);
            if (UDMA_SOK == retVal)
            {
                retVal = EnetUdma_freeRing(hRxFlow->fqRing,
                                          hRxFlow->rxFlowPrms.numRxPkts,
                                          hRxFlow->rxFlowPrms.ringMemFreeFxn,
                                          &hRxFlow->rxFlowPrms.cbArg);
                Enet_assert(UDMA_SOK == retVal);

                if (TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_SIZE_THRESH_MAX ==  hRxFlow->rxFlowPrms.flowPrms.sizeThreshEn)
                {
                    retVal = EnetUdma_freeRing(&hRxFlow->rxFlowMemObj.dropFqRingObj,
                                              1U,
                                              hRxFlow->rxFlowPrms.ringMemFreeFxn,
                                              &hRxFlow->rxFlowPrms.cbArg);
                    Enet_assert(UDMA_SOK == retVal);
                }
            }
#endif
        }

        if (UDMA_SOK == retVal)
        {
            /* Retrieve packets in Rx CQ so app can free them */
            retVal += EnetUdma_flushRxFlowRing(hRxFlow, hRxFlow->cqRing, pCqPktInfoQ);
            if ((UDMA_SOK == retVal) && (hRxFlow->evtInitFlag))
            {
                /* Return if any packets retrieved in the ISR */
                EnetQueue_append(pCqPktInfoQ, &hRxFlow->cqIsrQ);

                /* Unregister event - make sure you unregister the event after flushing
                 * the ring. This is because unregister event resets the ring and all the
                 * descriptors in the ring gets discarded causing resource leakage */
                retVal              += EnetUdma_unregisterEvent(hRxFlow->hUdmaEvt);
                hRxFlow->evtInitFlag = false;
            }

            if (UDMA_SOK == retVal)
            {
                retVal = EnetUdma_freeRing(hRxFlow->cqRing,
                                          hRxFlow->rxFlowPrms.numRxPkts,
                                          hRxFlow->rxFlowPrms.ringMemFreeFxn,
                                          &hRxFlow->rxFlowPrms.cbArg);
                Enet_assert(UDMA_SOK == retVal);
            }
        }

        if (UDMA_SOK == retVal)
        {
            /* Free if any DMA descriptors in the flows ready Q which application has
             * not consumed */
            if (!hRxFlow->rxFlowPrms.autoReclaimPrms.enableFlag)
            {
                EnetUdma_deInitRxFreeDescQ(hRxFlow);
            }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
            if (hRxFlow->rxFlowPrms.useProxy)
            {
                retVal += EnetUdma_freeProxy(hRxFlow->hUdmaProxy);
                Enet_assert(retVal == UDMA_SOK);
            }

            if (NULL != hRxFlow->hUdmaRingMon)
            {
                retVal = EnetUdma_freeRingMon(hRxFlow->hUdmaRingMon);
                Enet_assert(UDMA_SOK == retVal);
            }
#endif

            // TODO should detach happen at beginning?
            /* Detach UDMA flow so data reception starts */
            retVal           += Udma_flowDetach(hRxFlow->hUdmaFlow);
            hRxFlow->initFlag = false;

            /* Free Tx channel driver object memory */
            EnetUdma_memMgrFreeRxFlowObj(hRxFlow);
        }

        EnetOsal_restoreAllIntr(intrKey);
    }

    return retVal;
}

void EnetDma_initTxChParams(void *pTxChCfg)
{
    Udma_ChTxPrms txPrms = {0};
    EnetUdma_OpenTxChPrms *pTxChPrms = (EnetUdma_OpenTxChPrms *)pTxChCfg;

    UdmaChTxPrms_init(&txPrms, UDMA_CH_TYPE_TX);
    pTxChPrms->udmaTxChPrms.filterPsWords = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_FILT_PSWORDS_DISABLED;
    /* By default - don't filter extended packet info block where CPPI RX info words are mapped.
       App can enable filtering in case auto-reclaim use-case */
    pTxChPrms->udmaTxChPrms.filterEinfo   = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_FILT_EINFO_DISABLED;
    pTxChPrms->udmaTxChPrms.addrType    = txPrms.addrType;
    pTxChPrms->udmaTxChPrms.chanType    = txPrms.chanType;
    pTxChPrms->udmaTxChPrms.busPriority = txPrms.busPriority;
    pTxChPrms->udmaTxChPrms.busQos      = txPrms.busQos;
    pTxChPrms->udmaTxChPrms.busOrderId  = txPrms.busOrderId;
    pTxChPrms->udmaTxChPrms.dmaPriority = txPrms.dmaPriority;
    pTxChPrms->udmaTxChPrms.txCredit    = txPrms.txCredit;
    pTxChPrms->udmaTxChPrms.fifoDepth   = txPrms.fifoDepth;

    pTxChPrms->chNum = UDMA_DMA_CH_INVALID;

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    pTxChPrms->udmaChPrms.fqRingPrms.orderId    = UDMA_DEFAULT_RING_ORDER_ID;
    /* Use message mode for Tx FQ so at channel/flow teardown time UDMA IP can push
     * inflow descriptors back into the ring.*/
    pTxChPrms->udmaChPrms.fqRingPrms.mode       = TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE;
    pTxChPrms->udmaChPrms.fqRingPrms.useRingMon = false;
#endif

    pTxChPrms->udmaChPrms.cqRingPrms.orderId    = UDMA_DEFAULT_RING_ORDER_ID;
    pTxChPrms->udmaChPrms.cqRingPrms.mode       = TISCI_MSG_VALUE_RM_RING_MODE_RING;
    pTxChPrms->udmaChPrms.cqRingPrms.useRingMon = false;
    pTxChPrms->udmaChPrms.cqRingPrms.ringMonCfg.mode = 0U;
    pTxChPrms->udmaChPrms.cqRingPrms.ringMonCfg.data0 = 0U;
    pTxChPrms->udmaChPrms.cqRingPrms.ringMonCfg.data1 = 0U;

    /* We initialize callback function pointers to NULL so we check app pass
     * uninitialized  value */
    pTxChPrms->hUdmaDrv        = NULL;
    pTxChPrms->ringMemAllocFxn = NULL;
    pTxChPrms->ringMemFreeFxn  = NULL;
    pTxChPrms->dmaDescAllocFxn = NULL;
    pTxChPrms->dmaDescFreeFxn  = NULL;

    pTxChPrms->useGlobalEvt = false;
    pTxChPrms->notifyCb = NULL;
    pTxChPrms->cbArg   = NULL;

    pTxChPrms->useProxy = false;

    /* auto-reclaim initialization */
    pTxChPrms->autoReclaimPrms.enableFlag       = false;
    pTxChPrms->autoReclaimPrms.hDmaDescPool          = NULL;
    pTxChPrms->autoReclaimPrms.hReclaimRing = NULL;

    return;
}

EnetDma_TxChHandle EnetDma_openTxCh(EnetDma_Handle hDma,
                                    const void *pTxChCfg)
{
    EnetUdma_OpenTxChPrms *pTxChPrms = (EnetUdma_OpenTxChPrms *)pTxChCfg;
    int32_t retVal         = UDMA_SOK;
    EnetUdma_TxChObj *pTxCh = NULL;
    Udma_ChPrms chPrms;
    Udma_ChTxPrms txPrms;
    EnetUdma_udmaInfo udmaInfo;
    uint32_t chType;
    bool allocChObj    = false, chOpenFlag = false, chEnFlag = false;
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    bool allocCqRing = false;
    Udma_RingPrms ringPrms;
    EnetUdma_ringAllocInfo ringAllocInfo;
    bool allocProxy = false, allocFqRing  = false;
    void *pTdCqRingMem;
#endif
    EnetUdma_AutoReclaimPrms *reclaimPrms = &pTxChPrms->autoReclaimPrms;
    Udma_RingHandle ringHandle;
    uintptr_t intrKey;

    intrKey = EnetOsal_disableAllIntr();

    /* Set to NULL so if error condition we return NULL */
    pTxCh = NULL;

    /* Error check */
    retVal = EnetUdma_checkTxChParams(pTxChPrms);

    if (UDMA_SOK == retVal)
    {
        /* allocate Tx channel object */
        pTxCh = EnetUdma_memMgrAllocTxChObj();
        if (pTxCh == NULL)
        {
            ENETTRACE_ERR("[Enet UDMA] Memory allocation for TX channel object failed !!\n");
            retVal = UDMA_EALLOC;
        }
        else
        {
            allocChObj = true;
            memset(pTxCh, 0U, sizeof(*pTxCh));
            /* Save channel config */
            pTxCh->txChPrms = *pTxChPrms;
            /* Initialize local variables to be used in reminder of this function  */
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
            chType   = UDMA_CH_TYPE_TX;
#elif (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
            chType   = UDMA_CH_TYPE_TX_MAPPED;
#endif
            pTxCh->hUdmaDrv = pTxChPrms->hUdmaDrv;
            pTxCh->hUdmaCh  = &pTxCh->txChMemObj.udmaChObj;
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
            pTxCh->fqRing   = &pTxCh->txChMemObj.fqRingObj;
#endif
            pTxCh->hUdmaEvt = &pTxCh->txChMemObj.udmaEvtObj;
            pTxCh->useGlobalEvt = pTxChPrms->useGlobalEvt;

            if (!reclaimPrms->enableFlag)
            {
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
                pTxCh->cqRing       = &pTxCh->txChMemObj.cqRingObj;
#endif
                pTxCh->hDmaDescPool = &pTxCh->txChMemObj.txFreeDmaDescQ;
            }
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
            else
            {
#if ENET_CFG_IS_ON(DEV_ERROR)
                Enet_assert(NULL != reclaimPrms->hDmaDescPool);
                Enet_assert(NULL != reclaimPrms->hReclaimRing);
                /* Ring mode should be message as both TX channel (pushes to CQ) and RX flow
                 * (pops from FQ) uses this ring */
                Enet_assert(TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE == Udma_ringGetMode(reclaimPrms->hReclaimRing));
#endif
                /* Filtering of protocol specific and extended info should be enabled as this
                 * information might corrupt RX flow (as the RX flow PSI and EINFO gets mapped to
                 * Tx channel PSI and EINFO which has entirely different meaning) */
                Enet_assert(pTxChPrms->udmaTxChPrms.filterEinfo ==
                            TISCI_MSG_VALUE_RM_UDMAP_TX_CH_FILT_EINFO_DISABLED);
                Enet_assert(pTxChPrms->udmaTxChPrms.filterPsWords ==
                            TISCI_MSG_VALUE_RM_UDMAP_TX_CH_FILT_PSWORDS_ENABLED);
                /* Notify callback should be NULL when auto recycle is enabled as the
                 * CQ event should be disabled */
                Enet_assert(pTxChPrms->notifyCb == NULL);
                pTxCh->cqRing       = reclaimPrms->hReclaimRing;
                pTxCh->hDmaDescPool = reclaimPrms->hDmaDescPool;
            }
#endif
        }
    }

    if (UDMA_SOK == retVal)
    {
        /* Initialize channel params (PSI-L thread and RingAcc memories) */
        UdmaChPrms_init(&chPrms, chType);
        chPrms.peerChNum = pTxChPrms->chNum;
    }

    if (UDMA_SOK == retVal)
    {
        chPrms.chNum     = UDMA_DMA_CH_ANY;
#if (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
        if (Enet_isCpswFamily(hDma->enetType))
        {
            chPrms.mappedChGrp   = UDMA_MAPPED_TX_GROUP_CPSW;
        }
        else if (hDma->enetType == ENET_ICSSG_SWITCH)
        {
            if (0U == hDma->instId)
            {
                chPrms.mappedChGrp   = UDMA_MAPPED_TX_GROUP_ICSSG_0;
            }
            else if (1U == hDma->instId)
            {
                chPrms.mappedChGrp   = UDMA_MAPPED_TX_GROUP_ICSSG_1;
            }
            else
            {
                Enet_assert(false);
            }
        }
        else if (hDma->enetType == ENET_ICSSG_DUALMAC)
        {
            if ((0U == hDma->instId) || (1U == hDma->instId))
            {
                chPrms.mappedChGrp   = UDMA_MAPPED_TX_GROUP_ICSSG_0;
            }
            else if ((2U == hDma->instId) || (3U == hDma->instId))
            {
                chPrms.mappedChGrp   = UDMA_MAPPED_TX_GROUP_ICSSG_1;
            }
            else
            {
                Enet_assert(false);
            }
        }
        else
        {
            Enet_assert(false);
        }
#elif (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        pTdCqRingMem = EnetUdma_memMgrAllocTdCqRingMemObj();
        ENETTRACE_ERR_IF((pTdCqRingMem == NULL),
                           "[Enet UDMA] Teardown CQ ring memory allocation for Tx channel failed\n");
        if (pTdCqRingMem != NULL)
        {
            chPrms.tdCqRingPrms.ringMem = pTdCqRingMem;
            chPrms.tdCqRingPrms.elemCnt = ENET_UDMA_TDCQ_RING_ELE_CNT;
            chPrms.tdCqRingPrms.mode    = TISCI_MSG_VALUE_RM_RING_MODE_RING;
        }
        else
        {
            retVal = UDMA_EALLOC;
        }
#endif

#if (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
        Udma_RingPrms *pRingPrms = &chPrms.fqRingPrms;

        UdmaRingPrms_init(pRingPrms);
        pRingPrms->elemCnt = pTxChPrms->numTxPkts;
        pRingPrms->orderId = pTxChPrms->udmaChPrms.cqRingPrms.orderId;
        pRingPrms->mode    = pTxChPrms->udmaChPrms.cqRingPrms.mode;

        pRingPrms->ringMem = pTxChPrms->ringMemAllocFxn(pTxChPrms->cbArg,
                                             pRingPrms->elemCnt,
                                             UDMA_CACHELINE_ALIGNMENT);
        if (pRingPrms->ringMem == NULL)
        {
            ENETTRACE_ERR("[Enet UDMA Error] TX ring memory allocation failed !!\n");
            retVal = UDMA_EALLOC;
        }
        else if (!ENET_UTILS_IS_ALIGNED(pRingPrms->ringMem, UDMA_CACHELINE_ALIGNMENT))
        {
            ENETTRACE_ERR("[Enet UDMA] TX fq ring memory not aligned!!\n");
            pTxChPrms->ringMemFreeFxn(pTxChPrms->cbArg, pRingPrms->ringMem, pRingPrms->elemCnt);
            pRingPrms->ringMem = NULL;
            retVal  = UDMA_EALLOC;
        }
        else
        {
            /* Clear and invalidate ring memory */
            memset(pRingPrms->ringMem, 0, (pRingPrms->elemCnt * ENET_UDMA_RING_MEM_SIZE));
            if (FALSE == Enet_isCacheCoherent())
            {
                EnetOsal_cacheWbInv(pRingPrms->ringMem, (pRingPrms->elemCnt * ENET_UDMA_RING_MEM_SIZE));
            }
        }

        if (Enet_isCpswFamily(hDma->enetType))
        {
                pRingPrms->mappedRingGrp   = UDMA_MAPPED_TX_GROUP_CPSW;
        }
        else if (hDma->enetType == ENET_ICSSG_SWITCH)
        {
            if (0U == hDma->instId)
            {
                pRingPrms->mappedRingGrp   = UDMA_MAPPED_TX_GROUP_ICSSG_0;
            }
            else if (1U == hDma->instId)
            {
                pRingPrms->mappedRingGrp   = UDMA_MAPPED_TX_GROUP_ICSSG_1;
            }
            else
            {
                Enet_assert(false);
            }
        }
        else if (hDma->enetType == ENET_ICSSG_DUALMAC)
        {
            if ((0U == hDma->instId) || (1U == hDma->instId))
            {
                pRingPrms->mappedRingGrp   = UDMA_MAPPED_TX_GROUP_ICSSG_0;
            }
            else if ((2U == hDma->instId) || (3U == hDma->instId))
            {
                pRingPrms->mappedRingGrp   = UDMA_MAPPED_TX_GROUP_ICSSG_1;
            }
            else
            {
                Enet_assert(false);
            }
        }
        else
        {
            Enet_assert(false);
        }
#endif

        if (UDMA_SOK == retVal)
        {
            /* Open the UDMA channel */
            retVal = Udma_chOpen(pTxCh->hUdmaDrv, pTxCh->hUdmaCh, chType, &chPrms);
            ENETTRACE_ERR_IF((retVal != UDMA_SOK),
                               "[Enet UDMA] UDMA Tx channel failed !!: 0x%x\n", retVal);
            if (UDMA_SOK == retVal)
            {
                chOpenFlag = true;
#if (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
                /* Use handle to internally allocated ring */
            pTxCh->cqRing       = Udma_chGetFqRingHandle(pTxCh->hUdmaCh);
#endif
            }
        }
    }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    if (UDMA_SOK == retVal)
    {
        UdmaRingPrms_init(&ringPrms);
        ringPrms.elemCnt = pTxChPrms->numTxPkts;
        ringPrms.orderId = pTxChPrms->udmaChPrms.fqRingPrms.orderId;
        ringPrms.mode    = pTxChPrms->udmaChPrms.fqRingPrms.mode;

        ringAllocInfo.allocRingMem    = true;
        ringAllocInfo.ringMemAllocFxn = pTxChPrms->ringMemAllocFxn;
        ringAllocInfo.ringMemFreeFxn  = pTxChPrms->ringMemFreeFxn;
        ringAllocInfo.cbArg          = pTxChPrms->cbArg;
        ringAllocInfo.ringNum         = Udma_chGetNum(pTxCh->hUdmaCh);

        retVal = EnetUdma_allocRing(pTxCh->hUdmaDrv,
                                   pTxCh->fqRing,
                                   &ringPrms,
                                   &ringAllocInfo);
        ENETTRACE_ERR_IF((retVal != UDMA_SOK),
                           "[Enet UDMA] Fq ring allocation failed !!: 0x%x\n", retVal);
        if (UDMA_SOK == retVal)
        {
            allocFqRing = true;
        }
    }
#endif

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    if ((!reclaimPrms->enableFlag) && (UDMA_SOK == retVal))
    {
        UdmaRingPrms_init(&ringPrms);
        ringPrms.elemCnt = pTxChPrms->numTxPkts;
        ringPrms.orderId = pTxChPrms->udmaChPrms.cqRingPrms.orderId;
        ringPrms.mode    = pTxChPrms->udmaChPrms.cqRingPrms.mode;

        ringAllocInfo.allocRingMem    = true;
        ringAllocInfo.ringMemAllocFxn = pTxChPrms->ringMemAllocFxn;
        ringAllocInfo.ringMemFreeFxn  = pTxChPrms->ringMemFreeFxn;
        ringAllocInfo.cbArg          = pTxChPrms->cbArg;
        ringAllocInfo.ringNum         = UDMA_RING_ANY;
        ringAllocInfo.enetType        = hDma->enetType;
        ringAllocInfo.instId         = hDma->instId;
        ringAllocInfo.mappedChNum    = pTxCh->hUdmaCh->txChNum;
        ringAllocInfo.transferDir     = ENET_UDMA_DIR_TX;

        retVal = EnetUdma_allocRing(pTxCh->hUdmaDrv,
                                   pTxCh->cqRing,
                                   &ringPrms,
                                   &ringAllocInfo);
        ENETTRACE_ERR_IF((retVal != UDMA_SOK),
                           "[Enet UDMA] Cq ring allocation failed !!: 0x%x\n", retVal);
        if (UDMA_SOK == retVal)
        {
            allocCqRing = true;
        }
    }
#endif

    if (UDMA_SOK == retVal)
    {
        /* Configure TX channel */
        UdmaChTxPrms_init(&txPrms, chType);

        /* TODO - Should this be enabled? */
        txPrms.pauseOnError    = TISCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERROR_DISABLED;

        /* CPPI RX (host TX) Control Data Words [0..2] are mapped to Host Packet Descriptor
         * Protocol Specific Words if (TCHAN[a]_TCFG.tx_filt_pswords = 0) and
         * (Host PD Word 1.Protocol Specific Region Location.bit[28] = 0h) and
         * (Host PD Word 1.Protocol Specific Valid Word Count.bits[22-27] =4h)
         */
        txPrms.filterPsWords = pTxChPrms->udmaTxChPrms.filterPsWords;
        txPrms.filterEinfo   = pTxChPrms->udmaTxChPrms.filterEinfo;
        txPrms.chanType      = pTxChPrms->udmaTxChPrms.chanType;
        txPrms.busPriority   = pTxChPrms->udmaTxChPrms.busPriority;
        txPrms.busQos        = pTxChPrms->udmaTxChPrms.busQos;
        txPrms.busOrderId    = pTxChPrms->udmaTxChPrms.busOrderId;
        txPrms.dmaPriority   = pTxChPrms->udmaTxChPrms.dmaPriority;
        txPrms.txCredit      = pTxChPrms->udmaTxChPrms.txCredit;
        txPrms.fifoDepth     = pTxChPrms->udmaTxChPrms.fifoDepth;

        retVal               = Udma_chConfigTx(pTxCh->hUdmaCh, &txPrms);

        ENETTRACE_ERR_IF((retVal != UDMA_SOK),
                           "[Enet UDMA] UDMA Tx channel config failed !!: 0x%x\n", retVal);
        if (UDMA_SOK == retVal)
        {
            chOpenFlag = true;
        }
    }

    if (UDMA_SOK == retVal)
    {
        /* Initialize the completion queue.
         * Must be done before enabling CQ events */
        EnetQueue_initQ(&pTxCh->cqIsrQ);
    }

    if (UDMA_SOK == retVal)
    {
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        pTxCh->hUdmaProxy = NULL;

        if (NULL != pTxCh->fqRing)
        {
            if (pTxChPrms->useProxy)
            {
                retVal = EnetUdma_allocProxy(pTxCh->hUdmaDrv, &pTxCh->txChMemObj.proxyObj, pTxCh->fqRing);
                if (UDMA_SOK == retVal)
                {
                    allocProxy        = true;
                    pTxCh->hUdmaProxy = &pTxCh->txChMemObj.proxyObj;
                }
            }
        }
#endif
    }

    if (UDMA_SOK == retVal)
    {
        if (!reclaimPrms->enableFlag)
        {
            /* Initialize the descriptor Q.*/
            EnetUdma_initTxFreeDescQ(pTxCh);
        }
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        else
        {
            /* In auto-reclaim case we "re" configure the app passed desc Q
             * to change return Q index (retQIdx) configured in descriptor */
            EnetUdma_reconfigureDescQ(pTxCh);
        }
#endif
        /* Enable the UDMA channel */
        retVal = Udma_chEnable(pTxCh->hUdmaCh);

        ENETTRACE_VERBOSE_IF((pTxChPrms->notifyCb == NULL),
                    "[Enet UDMA] Event callback function is null !!\n");
        if ((UDMA_SOK == retVal) && (pTxChPrms->notifyCb != NULL))
        {
            chEnFlag = true;
            EnetUdma_getTxChUdmaInfo(pTxCh, &udmaInfo);
            ringHandle = pTxCh->cqRing;
            retVal = EnetUdma_registerEvent(&udmaInfo,
                                           ringHandle,
                                           &EnetUdma_txCqIsr,
                                           pTxCh,
                                           UDMA_EVENT_TYPE_RING);
            ENETTRACE_ERR_IF((retVal != UDMA_SOK),
                           "[Enet UDMA] TX Ch Open: event registration failed!!: 0x%x\n", retVal);
            if (ENET_SOK == retVal)
            {
                pTxCh->evtInitFlag = true;
            }
        }
    }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    /* Get teardown cqRing handle */
    if (UDMA_SOK == retVal)
    {
        pTxCh->tdCqRing = Udma_chGetTdCqRingHandle(pTxCh->hUdmaCh);
        if(NULL == pTxCh->tdCqRing)
        {
            retVal = UDMA_EALLOC;
        }
    }
#endif

    if (UDMA_SOK != retVal)
    {
        if (allocChObj)
        {
            /* Error. Free-up resource if allocated - order for freeing is important due
             * to dependencies */
            if (pTxCh->evtInitFlag)
            {
                retVal = EnetUdma_unregisterEvent(pTxCh->hUdmaEvt);
                Enet_assert(UDMA_SOK == retVal);
            }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
            if (allocFqRing)
            {
                retVal = EnetUdma_freeRing(pTxCh->fqRing,
                                           pTxChPrms->numTxPkts,
                                           pTxChPrms->ringMemFreeFxn,
                                           &pTxChPrms->cbArg);
                Enet_assert(UDMA_SOK == retVal);
            }

            if (allocCqRing)
            {
                /* We free cqRing without retrieving packets here as packets are
                 * not submitted by application yet so there wouldn't be any packets
                 * in the cqRing */
                retVal = EnetUdma_freeRing(pTxCh->cqRing,
                                           pTxChPrms->numTxPkts,
                                           pTxChPrms->ringMemFreeFxn,
                                           &pTxChPrms->cbArg);
                Enet_assert(UDMA_SOK == retVal);
            }
#endif

            /* Free up ready DMA descriptors */
            if (!reclaimPrms->enableFlag)
            {
                /* Free DMA descriptors in txFreeDmaDescQ */
                EnetUdma_deInitTxFreeDescQ(pTxCh);
            }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
            if (allocProxy)
            {
                retVal = EnetUdma_freeProxy(pTxCh->hUdmaProxy);
                Enet_assert(retVal == UDMA_SOK);
                pTxCh->hUdmaProxy = NULL;
            }
#endif

            if (chEnFlag)
            {
                retVal = Udma_chDisable(pTxCh->hUdmaCh, ENET_UDMA_TEARDOWN_TIME_MS);
                Enet_assert(retVal == UDMA_SOK);
            }

            if (chOpenFlag)
            {
                retVal = Udma_chClose(pTxCh->hUdmaCh);
                Enet_assert(retVal == UDMA_SOK);
            }

            EnetUdma_memMgrFreeTxChObj(pTxCh);
        }
    }
    else
    {
        pTxCh->hDma = hDma;
        pTxCh->initFlag = true;
    }


    EnetOsal_restoreAllIntr(intrKey);

    return pTxCh;
}

int32_t EnetDma_closeTxCh(EnetDma_TxChHandle hTxCh,
                          EnetDma_PktQ *pFqPktInfoQ,
                          EnetDma_PktQ *pCqPktInfoQ)
{
    int32_t retVal = ENET_SOK;
    uintptr_t intrKey;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hTxCh) ||
        (NULL == pFqPktInfoQ) ||
        (NULL == pCqPktInfoQ))
    {
        ENETTRACE_ERR_IF((NULL == hTxCh), "[Enet UDMA] hTxCh is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == pFqPktInfoQ), "[Enet UDMA] pFqPktInfoQ is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == pFqPktInfoQ), "[Enet UDMA] pFqPktInfoQ is NULL!!\n");
        Enet_assert(FALSE);
        retVal = UDMA_EBADARGS;
    }
    else
#endif
    {
        intrKey = EnetOsal_disableAllIntr();

        /* Error check */
        if (hTxCh == NULL)
        {
            retVal = UDMA_EBADARGS;
        }

        if (UDMA_SOK == retVal)
        {
            if (!hTxCh->initFlag)
            {
                retVal = UDMA_EFAIL;
            }
        }

        /* Decode response information. Return an error if tear down is incomplete
         * or if tear-down was not graceful and data was potentially lost */
        if (UDMA_SOK == retVal)
        {
            /* Disable the UDMA channel */
            retVal += Udma_chDisable(hTxCh->hUdmaCh, ENET_UDMA_TEARDOWN_TIME_MS);

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
            /* Dequeue the teardown response */
            if (UDMA_SOK == retVal)
            {
                CSL_UdmapTdResponse tdResp;
                void *tdCqRingMemPtr = NULL;

                /* Store the ring memory pointers so we can free ring memory when UDMA frees
                 * Ring via Udma_ringFree internally in Udma_chClose */
                tdCqRingMemPtr = Udma_ringGetMemPtr(hTxCh->tdCqRing);

                do
                {
                    retVal = Udma_chDequeueTdResponse(hTxCh->hUdmaCh, &tdResp);
                }
                while (UDMA_SOK != retVal);

                if (FALSE == tdResp.tdIndicator)
                {
                    ENETTRACE_ERR("[Enet UDMA] TX channel TearDown failed!!\n");
                    retVal += UDMA_EFAIL;
                }

                /* Free tdCq ring memory */
                EnetUdma_memMgrFreeTdCqRingMemObj(tdCqRingMemPtr);
            }
#endif
        }

        if (UDMA_SOK == retVal)
        {
            if (hTxCh->evtInitFlag)
            {
                /* Disable channel Event */
                retVal += EnetDma_disableTxEvent(hTxCh);
            }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
            /* Flush packets in Rx FQ - retrieve any packets that haven't been
             * processed yet */
            retVal += EnetUdma_flushTxChRing(hTxCh, hTxCh->fqRing, pFqPktInfoQ);
            retVal += EnetUdma_freeRing(hTxCh->fqRing,
                                       hTxCh->txChPrms.numTxPkts,
                                       hTxCh->txChPrms.ringMemFreeFxn,
                                       &hTxCh->txChPrms.cbArg);
            Enet_assert(UDMA_SOK == retVal);
#endif
        }

        if (UDMA_SOK == retVal)
        {
            /* Retrieve packets in Rx CQ */
            retVal += EnetUdma_flushTxChRing(hTxCh, hTxCh->cqRing, pCqPktInfoQ);

            if ((UDMA_SOK == retVal) && (hTxCh->evtInitFlag))
            {
                /* Return if any packets retrieved in the ISR */
                EnetQueue_append(pCqPktInfoQ, &hTxCh->cqIsrQ);

                /* Unregister event - make sure you unregister the event after flushing the ring.
                 * This is because unregister event resets the ring and all the descriptors
                 * in the ring gets discarded causing resource leakage */
                retVal            += EnetUdma_unregisterEvent(hTxCh->hUdmaEvt);
                hTxCh->evtInitFlag = false;
            }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1U)
            retVal += EnetUdma_freeRing(hTxCh->cqRing,
                                       hTxCh->txChPrms.numTxPkts,
                                       hTxCh->txChPrms.ringMemFreeFxn,
                                       &hTxCh->txChPrms.cbArg);
            Enet_assert(UDMA_SOK == retVal);
#elif (UDMA_SOC_CFG_LCDMA_PRESENT == 1U)
            uint8_t * ringMemPtr = (uint8_t *)Udma_ringGetMemPtr(hTxCh->cqRing);
            Enet_assert(NULL != ringMemPtr);
            hTxCh->txChPrms.ringMemFreeFxn(hTxCh->txChPrms.cbArg, ringMemPtr, hTxCh->txChPrms.numTxPkts);
#endif
        }

        if (UDMA_SOK == retVal)
        {
            /* Free up ready DMA descriptors */
            if (!hTxCh->txChPrms.autoReclaimPrms.enableFlag)
            {
                /* Free DMA descriptors in txFreeDmaDescQ */
                EnetUdma_deInitTxFreeDescQ(hTxCh);
            }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
            /* Free proxies if allocated */
            if (hTxCh->txChPrms.useProxy)
            {
                retVal += EnetUdma_freeProxy(hTxCh->hUdmaProxy);
                Enet_assert(retVal == UDMA_SOK);
            }
#endif

            if (UDMA_SOK == retVal)
            {
                /* Close the UDMA channel */
                retVal         += Udma_chClose(hTxCh->hUdmaCh);
                hTxCh->initFlag = false;
            }

            /* Free Tx channel driver object memory */
            EnetUdma_memMgrFreeTxChObj(hTxCh);
        }

        EnetOsal_restoreAllIntr(intrKey);
    }

    return retVal;
}

int32_t EnetDma_enableRxEvent(EnetDma_RxChHandle hRxFlow)
{
    int32_t retVal;

    if ((NULL != hRxFlow) && (true == hRxFlow->evtInitFlag))
    {
        retVal = Udma_eventEnable(hRxFlow->hUdmaEvt);
    }
    else
    {
        retVal = UDMA_EBADARGS;
    }

    return retVal;
}

int32_t EnetDma_disableRxEvent(EnetDma_RxChHandle hRxFlow)
{
    int32_t retVal;

    if ((NULL != hRxFlow) && (true == hRxFlow->evtInitFlag))
    {
        retVal = Udma_eventDisable(hRxFlow->hUdmaEvt);
    }
    else
    {
        retVal = UDMA_EBADARGS;
    }

    return retVal;
}

int32_t EnetDma_getRxChStats(EnetDma_RxChHandle hRxFlow,
                               EnetDma_RxChStats *rxFlowStats)
{
    int32_t retVal = ENET_ENOTSUPPORTED;

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
    *rxFlowStats = hRxFlow->stats;
#endif

    return retVal;
}

int32_t EnetDma_getTxChStats(EnetDma_TxChHandle hTxCh,
                             EnetDma_TxChStats *txChStats)
{
    int32_t retVal = ENET_ENOTSUPPORTED;

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
    *txChStats = hTxCh->stats;
#endif

    return retVal;
}

int32_t EnetDma_resetRxChStats(EnetDma_RxChHandle hRxFlow)
{
    int32_t retVal = ENET_ENOTSUPPORTED;

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
    memset(&hRxFlow->stats, 0, sizeof(hRxFlow->stats));
#endif

    return retVal;
}

int32_t EnetDma_resetTxChStats(EnetDma_TxChHandle hTxCh)
{
    int32_t retVal = ENET_ENOTSUPPORTED;

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
    memset(&hTxCh->stats, 0, sizeof(hTxCh->stats));
#endif

    return retVal;
}

int32_t EnetDma_enableTxEvent(EnetDma_TxChHandle hTxCh)
{
    int32_t retVal;

    if ((NULL != hTxCh) && (true == hTxCh->evtInitFlag))
    {
        retVal = Udma_eventEnable(hTxCh->hUdmaEvt);
    }
    else
    {
        retVal = UDMA_EBADARGS;
    }

    return retVal;
}

int32_t EnetDma_disableTxEvent(EnetDma_TxChHandle hTxCh)
{
    int32_t retVal;

    if ((NULL != hTxCh) && (true == hTxCh->evtInitFlag))
    {
        retVal = Udma_eventDisable(hTxCh->hUdmaEvt);
    }
    else
    {
        retVal = UDMA_EBADARGS;
    }

    return retVal;
}

int32_t EnetDma_retrieveRxPktQ(EnetDma_RxChHandle hRxFlow,
                               EnetDma_PktQ *pRetrieveQ)
{
    EnetPer_Handle hPer = hRxFlow->hDma->hPer;
    int32_t retVal = UDMA_SOK;
    EnetQ tempQ;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hRxFlow) ||
        (NULL == pRetrieveQ))
    {
        ENETTRACE_ERR_IF((NULL == hRxFlow), "[Enet UDMA] hRxFlow is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == pRetrieveQ), "[Enet UDMA] pRetrieveQ is NULL!!\n");
        Enet_assert(FALSE);
        retVal = UDMA_EBADARGS;
    }
    else
#endif
    {
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        uint32_t startTime, diffTime;
        uint32_t pktCnt;
        startTime = EnetOsal_timerRead();
#endif

        EnetQueue_initQ(pRetrieveQ);
        EnetUdma_drainIsrCq(pRetrieveQ, &hRxFlow->cqIsrQ);
        EnetQueue_initQ(&tempQ);

        /* EnetUdma_retrievePkts initializes the queue so cannot pass
         * pRetrieveQ as it contains packets drained from isrq
         */
        retVal = EnetUdma_retrievePkts(hPer,
                                       hRxFlow->cqRing,
                                      &tempQ,
                                      hRxFlow->hDmaDescPool,
                                      hRxFlow->rxFlowPrms.disableCacheOpsFlag,
                                      ENET_UDMA_DIR_RX);

        if (ENET_SOK == retVal)
        {
            EnetQueue_append(pRetrieveQ, &tempQ);
        }

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        pktCnt = EnetQueue_getQCount(pRetrieveQ);
        EnetUdmaStats_addCnt(&hRxFlow->stats.rxRetrievePktDeq, pktCnt);
        diffTime = EnetOsal_timerGetDiff(startTime);
        EnetUdmaStats_updateNotifyStats(&hRxFlow->stats.retrievePktStats, pktCnt, diffTime);
#endif
    }

    return retVal;
}

int32_t EnetDma_retrieveRxPkt(EnetDma_RxChHandle hRxFlow,
                               EnetDma_Pkt **ppPkt)
{
    EnetPer_Handle hPer = hRxFlow->hDma->hPer;
    int32_t retVal = UDMA_SOK;
    EnetQ tempQ;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hRxFlow) ||
        (NULL == ppPkt))
    {
        ENETTRACE_ERR_IF((NULL == hRxFlow), "[Enet UDMA] hRxFlow is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == ppPkt), "[Enet UDMA] ppPkt is NULL!!\n");
        Enet_assert(FALSE);
        retVal = UDMA_EBADARGS;
    }
    else
#endif
    {
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        uint32_t startTime, diffTime;
        startTime = EnetOsal_timerRead();
#endif
        if(EnetQueue_getQCount(&hRxFlow->cqIsrQ) == 0)
        {
            EnetQueue_initQ(&tempQ);

            /* EnetUdma_retrievePkts initializes the queue so cannot pass
             * pRetrieveQ as it contains packets drained from isrq
             */
            retVal = EnetUdma_retrievePkts(hPer,
                                           hRxFlow->cqRing,
                                           &tempQ,
                                           hRxFlow->hDmaDescPool,
                                           hRxFlow->rxFlowPrms.disableCacheOpsFlag,
                                           ENET_UDMA_DIR_RX);

            if (ENET_SOK == retVal)
            {
                EnetQueue_append(&hRxFlow->cqIsrQ, &tempQ);
            }

        }

        *ppPkt = (EnetUdma_PktInfo *)EnetQueue_deq(&hRxFlow->cqIsrQ);
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        EnetUdmaStats_addCnt(&hRxFlow->stats.rxRetrievePktDeq, 1U);
        diffTime = EnetOsal_timerGetDiff(startTime);
        EnetUdmaStats_updateNotifyStats(&hRxFlow->stats.retrievePktStats, 1U, diffTime);
#endif
    }

    return retVal;
}

int32_t EnetDma_submitRxPktQ(EnetDma_RxChHandle hRxFlow,
                                EnetDma_PktQ *pSubmitQ)
{
    EnetPer_Handle hPer = hRxFlow->hDma->hPer;
    int32_t retVal = UDMA_SOK;
    Udma_RingHandle ringHandle;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hRxFlow) ||
        (NULL == pSubmitQ))
    {
        ENETTRACE_ERR_IF((NULL == hRxFlow), "[Enet UDMA] hRxFlow is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == pSubmitQ), "[Enet UDMA] pSubmitQ is NULL!!\n");
        Enet_assert(FALSE);
        retVal = UDMA_EBADARGS;
    }
    else
#endif
    {
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        uint32_t startTime, diffTime;
        uint32_t pktCnt, notifyCount;
        startTime = EnetOsal_timerRead();
        pktCnt    = EnetQueue_getQCount(pSubmitQ);
#endif
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        ringHandle = hRxFlow->fqRing;
#else
        ringHandle = hRxFlow->cqRing;
#endif

        /* Enqueue descs to fqRing regardless of caller's queue state */
        if (EnetQueue_getQCount(pSubmitQ) > 0U)
        {
            retVal = EnetUdma_submitPkts(hPer,
                                         ringHandle,
                                        pSubmitQ,
                                        hRxFlow->hDmaDescPool,
                                        hRxFlow->rxFlowPrms.disableCacheOpsFlag,
                                        ENET_UDMA_DIR_RX
#if (UDMA_SOC_CFG_PROXY_PRESENT == 1)
                                        ,
                                        hRxFlow->hUdmaProxy
#endif
                                        );
        }

        /* If fqRing ran out of space is not an error, packets will be re-submitted from application */
        if (UDMA_ETIMEOUT == retVal)
        {
            ENETTRACE_INFO("RX FLOW FQ underflow had occurred\n");
            retVal = UDMA_SOK;
        }

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        EnetUdmaStats_addCnt(&hRxFlow->stats.rxSubmitPktUnderFlowCnt,
                            EnetQueue_getQCount(pSubmitQ));
        pktCnt -= EnetQueue_getQCount(pSubmitQ);
        EnetUdmaStats_addCnt(&hRxFlow->stats.rxSubmitPktEnq, pktCnt);
        diffTime = EnetOsal_timerGetDiff(startTime);
        notifyCount = hRxFlow->stats.submitPktStats.dataNotifyCnt & (ENET_DMA_STATS_HISTORY_CNT - 1U);
        hRxFlow->stats.submitPktStats.readyDmaDescQCnt[notifyCount] = EnetUdma_dmaDescQCount(hRxFlow->hDmaDescPool);
        EnetUdmaStats_updateNotifyStats(&hRxFlow->stats.submitPktStats, pktCnt, diffTime);
#endif
    }

    return retVal;
}

int32_t EnetDma_submitRxPkt(EnetDma_RxChHandle hRxFlow,
                            EnetDma_Pkt *pPkt)
{
    EnetPer_Handle hPer = hRxFlow->hDma->hPer;
    int32_t retVal = UDMA_SOK;
    Udma_RingHandle ringHandle;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hRxFlow) ||
        (NULL == pPkt))
    {
        ENETTRACE_ERR_IF((NULL == hRxFlow), "[Enet UDMA] hRxFlow is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == pPkt), "[Enet UDMA] pPkt is NULL!!\n");
        Enet_assert(FALSE);
        retVal = UDMA_EBADARGS;
    }
    else
#endif
    {
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        uint32_t startTime, diffTime;
        uint32_t notifyCount;
        startTime = EnetOsal_timerRead();
#endif
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        ringHandle = hRxFlow->fqRing;
#else
        ringHandle = hRxFlow->cqRing;
#endif

        /* Enqueue descs to fqRing regardless of caller's queue state */

        retVal = EnetUdma_submitSingleRxPkt(hPer,
                                          ringHandle,
                                          pPkt,
                                          hRxFlow->hDmaDescPool,
                                          hRxFlow->rxFlowPrms.disableCacheOpsFlag
#if (UDMA_SOC_CFG_PROXY_PRESENT == 1)
                                          ,
                                          hRxFlow->hUdmaProxy
#endif
                                          );

        if (retVal == UDMA_EALLOC)
        {
            ENETTRACE_INFO("Descriptor unavailable. Please transmit again\n");
            retVal = UDMA_SOK;
        }

        /* If fqRing ran out of space is not an error, packets will be re-submitted from application */
        if (UDMA_ETIMEOUT == retVal)
        {
            ENETTRACE_INFO("RX FLOW FQ underflow had occurred\n");
            retVal = UDMA_SOK;
        }

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        EnetUdmaStats_addCnt(&hRxFlow->stats.rxSubmitPktUnderFlowCnt, 1U);
        EnetUdmaStats_addCnt(&hRxFlow->stats.rxSubmitPktEnq, 1U);
        diffTime = EnetOsal_timerGetDiff(startTime);
        notifyCount = hRxFlow->stats.submitPktStats.dataNotifyCnt & (ENET_DMA_STATS_HISTORY_CNT - 1U);
        hRxFlow->stats.submitPktStats.readyDmaDescQCnt[notifyCount] = EnetUdma_dmaDescQCount(hRxFlow->hDmaDescPool);
        EnetUdmaStats_updateNotifyStats(&hRxFlow->stats.submitPktStats, 1U, diffTime);
#endif
    }

    return retVal;
}

int32_t EnetDma_retrieveTxPktQ(EnetDma_TxChHandle hTxCh,
                                   EnetDma_PktQ *pRetrieveQ)
{
    EnetPer_Handle hPer = hTxCh->hDma->hPer;
    int32_t retVal = UDMA_SOK;
    EnetQ tempQ;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hTxCh) ||
        (NULL == pRetrieveQ))
    {
        ENETTRACE_ERR_IF((NULL == hTxCh), "[Enet UDMA] hTxCh is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == pRetrieveQ), "[Enet UDMA] pRetrieveQ is NULL!!\n");
        Enet_assert(FALSE);
        retVal = UDMA_EBADARGS;
    }
    else
#endif
    {
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        uint32_t startTime, diffTime;
        uint32_t pktCnt;
        startTime = EnetOsal_timerRead();
#endif

        EnetQueue_initQ(pRetrieveQ);
        EnetUdma_drainIsrCq(pRetrieveQ, &hTxCh->cqIsrQ);
        EnetQueue_initQ(&tempQ);
        /* EnetUdma_retrievePkts initializes the queue so cannot pass
         * pRetrieveQ as it contains packets drained from isrq
         */
        retVal = EnetUdma_retrievePkts(hPer,
                                       hTxCh->cqRing,
                                      &tempQ,
                                      hTxCh->hDmaDescPool,
                                      hTxCh->txChPrms.disableCacheOpsFlag,
                                      ENET_UDMA_DIR_TX);

        if (ENET_SOK == retVal)
        {
            EnetQueue_append(pRetrieveQ, &tempQ);
        }

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        pktCnt = EnetQueue_getQCount(pRetrieveQ);
        EnetUdmaStats_addCnt(&hTxCh->stats.txRetrievePktDeq, pktCnt);
        diffTime = EnetOsal_timerGetDiff(startTime);
        EnetUdmaStats_updateNotifyStats(&hTxCh->stats.retrievePktStats, pktCnt, diffTime);
#endif
    }

    return retVal;
}

int32_t EnetDma_retrieveTxPkt(EnetDma_TxChHandle hTxCh,
                              EnetDma_Pkt **ppPkt)
{
    EnetPer_Handle hPer = hTxCh->hDma->hPer;
    int32_t retVal = UDMA_SOK;
    EnetQ tempQ;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hTxCh) ||
        (NULL == ppPkt))
    {
        ENETTRACE_ERR_IF((NULL == hTxCh), "[Enet UDMA] hTxCh is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == ppPkt), "[Enet UDMA] ppPkt is NULL!!\n");
        Enet_assert(FALSE);
        retVal = UDMA_EBADARGS;
    }
    else
#endif
    {
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        uint32_t startTime, diffTime;
        startTime = EnetOsal_timerRead();
#endif
        if(EnetQueue_getQCount(&hTxCh->cqIsrQ) == 0)
        {
            EnetQueue_initQ(&tempQ);

            /* EnetUdma_retrievePkts initializes the queue so cannot pass
             * pRetrieveQ as it contains packets drained from isrq
             */
            retVal = EnetUdma_retrievePkts(hPer,
                                           hTxCh->cqRing,
                                           &tempQ,
                                           hTxCh->hDmaDescPool,
                                           hTxCh->txChPrms.disableCacheOpsFlag,
                                           ENET_UDMA_DIR_TX);

            if (ENET_SOK == retVal)
            {
                EnetQueue_append(&hTxCh->cqIsrQ, &tempQ);
            }

        }

        *ppPkt = (EnetUdma_PktInfo *)EnetQueue_deq(&hTxCh->cqIsrQ);

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        EnetUdmaStats_addCnt(&hTxCh->stats.txRetrievePktDeq, 1U);
        diffTime = EnetOsal_timerGetDiff(startTime);
        EnetUdmaStats_updateNotifyStats(&hTxCh->stats.retrievePktStats, 1U, diffTime);
#endif
    }

    return retVal;
}

int32_t EnetDma_submitTxPktQ(EnetDma_TxChHandle hTxCh,
                                  EnetDma_PktQ *pSubmitQ)

{
    EnetPer_Handle hPer = hTxCh->hDma->hPer;
    int32_t retVal = UDMA_SOK;
    Udma_RingHandle ringHandle;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hTxCh) ||
        (NULL == pSubmitQ))
    {
        ENETTRACE_ERR_IF((NULL == hTxCh), "[Enet UDMA] hTxCh is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == pSubmitQ), "[Enet UDMA] pSubmitQ is NULL!!\n");
        Enet_assert(FALSE);
        retVal = UDMA_EBADARGS;
    }
    else
#endif
    {
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        uint32_t startTime, diffTime;
        uint32_t pktCnt, notifyCount;
        startTime = EnetOsal_timerRead();
        pktCnt    = EnetQueue_getQCount(pSubmitQ);
#endif
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        ringHandle = hTxCh->fqRing;
#else
        ringHandle = hTxCh->cqRing;
#endif
        /* Enqueue descs to fqRing regardless of caller's queue state */
        if (EnetQueue_getQCount(pSubmitQ) > 0U)
        {
            retVal = EnetUdma_submitPkts(hPer,
                                         ringHandle,
                                         pSubmitQ,
                                         hTxCh->hDmaDescPool,
                                         hTxCh->txChPrms.disableCacheOpsFlag,
                                         ENET_UDMA_DIR_TX
#if (UDMA_SOC_CFG_PROXY_PRESENT == 1)
                                         ,
                                         hTxCh->hUdmaProxy
#endif
                                         );
        }

        /* If fqRing ran out of space it is not an error, packets will be re-submitted by application*/
        if (UDMA_ETIMEOUT == retVal)
        {
            ENETTRACE_INFO("TX Channel FQ underflow had occurred\n");
            retVal = UDMA_SOK;
        }

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        EnetUdmaStats_addCnt(&hTxCh->stats.txSubmitPktOverFlowCnt, EnetQueue_getQCount(pSubmitQ));
        pktCnt -= EnetQueue_getQCount(pSubmitQ);
        EnetUdmaStats_addCnt(&hTxCh->stats.txSubmitPktEnq, pktCnt);
        diffTime                                                  = EnetOsal_timerGetDiff(startTime);
        notifyCount                                               = hTxCh->stats.submitPktStats.dataNotifyCnt & (ENET_DMA_STATS_HISTORY_CNT - 1U);
        hTxCh->stats.submitPktStats.readyDmaDescQCnt[notifyCount] = hTxCh->hDmaDescPool->count;

        EnetUdmaStats_updateNotifyStats(&hTxCh->stats.submitPktStats, pktCnt, diffTime);
#endif
    }

    return retVal;
}

int32_t EnetDma_submitTxPkt(EnetDma_TxChHandle hTxCh,
                                  EnetDma_Pkt *pPkt)

{
    EnetPer_Handle hPer = hTxCh->hDma->hPer;
    int32_t retVal = UDMA_SOK;
    Udma_RingHandle ringHandle;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hTxCh) ||
        (NULL == pPkt))
    {
        ENETTRACE_ERR_IF((NULL == hTxCh), "[Enet UDMA] hTxCh is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == pPkt), "[Enet UDMA] pPkt is NULL!!\n");
        Enet_assert(FALSE);
        retVal = UDMA_EBADARGS;
    }
    else
#endif
    {
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        uint32_t startTime, diffTime;
        uint32_t notifyCount;
        startTime = EnetOsal_timerRead();
#endif
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        ringHandle = hTxCh->fqRing;
#else
        ringHandle = hTxCh->cqRing;
#endif
        /* Enqueue descs to fqRing regardless of caller's queue state */
        retVal = EnetUdma_submitSingleTxPkt(hPer,
                                            ringHandle,
                                            pPkt,
                                            hTxCh->hDmaDescPool,
                                            hTxCh->txChPrms.disableCacheOpsFlag
#if (UDMA_SOC_CFG_PROXY_PRESENT == 1)
                                            ,
                                            hTxCh->hUdmaProxy
#endif
                                            );

        /* If fqRing ran out of space it is not an error, packets will be re-submitted by application*/
        if (UDMA_ETIMEOUT == retVal)
        {
            ENETTRACE_INFO("TX Channel FQ underflow had occurred\n");
            retVal = UDMA_SOK;
        }

#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
        EnetUdmaStats_addCnt(&hTxCh->stats.txSubmitPktOverFlowCnt, 1U);
        EnetUdmaStats_addCnt(&hTxCh->stats.txSubmitPktEnq, 1U);
        diffTime                                                  = EnetOsal_timerGetDiff(startTime);
        notifyCount                                               = hTxCh->stats.submitPktStats.dataNotifyCnt & (ENET_DMA_STATS_HISTORY_CNT - 1U);
        hTxCh->stats.submitPktStats.readyDmaDescQCnt[notifyCount] = hTxCh->hDmaDescPool->count;

        EnetUdmaStats_updateNotifyStats(&hTxCh->stats.submitPktStats, 1U, diffTime);
#endif
    }

    return retVal;
}

void EnetDma_initPktInfo(EnetDma_Pkt *pktInfo)
{
    uint32_t i;

    memset(&pktInfo->node, 0U, sizeof(pktInfo->node));
    for(i = 0; i < ENET_UDMA_CPSW_MAX_SG_LIST; i++)
    {
        pktInfo->sgList.list[i].bufPtr    = NULL;
        pktInfo->sgList.list[i].segmentFilledLen = 0U;
        pktInfo->sgList.list[i].segmentAllocLen = 0U;
        pktInfo->sgList.list[i].disableCacheOps = false;
    }
    pktInfo->sgList.numScatterSegments = 0U;
    pktInfo->appPriv               = NULL;
    pktInfo->tsInfo.enableHostTxTs = false;
    pktInfo->txPortNum             = ENET_MAC_PORT_INV;
    pktInfo->txPktTc               = ENET_TRAFFIC_CLASS_INV;
    pktInfo->chkSumInfo            = 0U;
    ENET_UTILS_SET_PKT_DRIVER_STATE(&pktInfo->pktState,
                                    (uint32_t)ENET_PKTSTATE_DMA_NOT_WITH_HW);
}

/* ========================================================================== */
/*                          DMA peripheral specific Function Definitions      */
/* ========================================================================== */

void EnetUdma_initCfg(Enet_Type enetType, void *cfg)
{
    EnetUdma_Cfg *pDmaConfig = (EnetUdma_Cfg *)cfg;

    pDmaConfig->hUdmaDrv                 = NULL;
    pDmaConfig->rxChInitPrms.dmaPriority = UDMA_DEFAULT_RX_CH_DMA_PRIORITY;
}

EnetDma_Handle EnetUdma_open(Enet_Type enetType,
                             uint32_t instId,
                             const EnetUdma_Cfg *pEnetUdmaCfg)
{
    EnetUdma_DrvObj *pEnetUdmaObj = NULL;
    int32_t retVal;

    ENET_UTILS_COMPILETIME_ASSERT(ENET_UDMA_RING_MEM_SIZE == sizeof(uint64_t));

    /* Error check */
    retVal = UDMA_EBADARGS;
    if (NULL != pEnetUdmaCfg)
    {
        if (NULL != pEnetUdmaCfg->hUdmaDrv)
        {
            retVal = UDMA_SOK;
        }
        else
        {
            ENETTRACE_ERR("[Enet UDMA Error] pEnetUdmaCfg has null UDMA handle!!\n");
        }
    }
    else
    {
        ENETTRACE_ERR("[Enet UDMA Error] pEnetUdmaCfg NULL !!\n");
    }

    if (UDMA_SOK == retVal)
    {
        pEnetUdmaObj = EnetSoc_getDmaHandle(enetType, instId);
        Enet_assert(pEnetUdmaObj != NULL,
                "No DMA object for peripheral (eneType=%u instId=%u)\n",
                enetType, instId);


        pEnetUdmaObj->initFlag         = true;
        pEnetUdmaObj->hUdmaDrv         = pEnetUdmaCfg->hUdmaDrv;
        pEnetUdmaObj->numRxCh = (enetType == ENET_ICSSG_SWITCH) ? 2U : 1U;
        Enet_assert(pEnetUdmaObj->numRxCh <= ENET_UDMA_NUM_RXCHAN_MAX);
        pEnetUdmaObj->enetType         = enetType;
        pEnetUdmaObj->instId           = instId;

        /* Initialize memory manager module managing driver object memories */
        EnetUdma_memMgrInit();
    }

    return pEnetUdmaObj;
}

int32_t EnetUdma_close(EnetDma_Handle hEnetUdma)
{
    int32_t retVal = UDMA_SOK;

    // TODO check all Rx & Tx channels & are closed
    /* Error check */
    if (hEnetUdma == NULL)
    {
        ENETTRACE_ERR("[Enet UDMA] Enet UDMA handle is NULL!!\n");
        retVal = UDMA_EBADARGS;
    }

    if (UDMA_SOK == retVal)
    {
        if (hEnetUdma->initFlag == false)
        {
            ENETTRACE_ERR("[Enet UDMA] DMA is not initialized before close !!\n");
            retVal = UDMA_EFAIL;
        }
    }

    if (UDMA_SOK == retVal)
    {
        /* De-initialize driver memory manager */
        EnetUdma_memMgrDeInit();
        hEnetUdma->initFlag = false;
    }

    return retVal;
}

int32_t EnetUdma_openRxCh(EnetDma_Handle hEnetUdma,
                         const EnetUdma_RxChInitPrms *pRxChInitPrms,
                          uint32_t totalRxFlowCount,
                          uint32_t chIdx)
{
    int32_t retVal;
    EnetPer_Handle hPer = NULL;
    Udma_DrvHandle hUdmaDrv;
    EnetUdma_RxChObj *pRxCh;
    Udma_ChHandle hUdmaCh;
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    Udma_FlowHandle hFlow;
    bool allocFlow = false;
#endif
    Udma_ChPrms chPrms;
    Udma_ChRxPrms rxPrms;

    bool chOpenFlag = false;
    uint32_t chType, rxFlowCount;
    uintptr_t intrKey;

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    void *pTdCqRingMem;
#endif

    /* Error check */
    retVal = UDMA_EBADARGS;
    if (hEnetUdma != NULL)
    {
        if (hEnetUdma->initFlag)
        {
            hPer = hEnetUdma->hPer;
            retVal = UDMA_SOK;
        }
        else
        {
            ENETTRACE_ERR("[Enet UDMA Error] Enet UDMA driver not initialized !!\n");
        }
    }
    else
    {
        ENETTRACE_ERR("[Enet UDMA Error] Enet UDMA handle is NULL !!\n");
    }

    if (UDMA_SOK == retVal)
    {
        rxFlowCount = EnetSoc_getRxFlowCount(hPer->enetType, hPer->instId);
        if (totalRxFlowCount > rxFlowCount)
        {
            ENETTRACE_ERR("[Enet UDMA] Invalid Rx channel flow count "
                          "Requested = %d, SOC Available = %d !!\n",
                          totalRxFlowCount, rxFlowCount);
            /* flows num greater than CPSW instance max. supported */
            retVal = UDMA_EBADARGS;
        }
    }

    intrKey = EnetOsal_disableAllIntr();

    if (UDMA_SOK == retVal)
    {
        hUdmaDrv = hEnetUdma->hUdmaDrv;
        pRxCh    = &hEnetUdma->rxChObj[chIdx];
        pRxCh->chIdx = chIdx;
        hUdmaCh  = &pRxCh->udmaChObj;
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        chType   = UDMA_CH_TYPE_RX;
#elif (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
        chType   = UDMA_CH_TYPE_RX_MAPPED;
#endif
        /* Initialize channel params (PSI-L thread and RingAcc memories) */
        UdmaChPrms_init(&chPrms, chType);
        chPrms.peerChNum = EnetSoc_getRxChPeerId(hPer->enetType, hPer->instId, chIdx);
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        chPrms.chNum         = UDMA_DMA_CH_ANY;
        pTdCqRingMem = EnetUdma_memMgrAllocTdCqRingMemObj();
        if (pTdCqRingMem != NULL)
        {
            UdmaRingPrms_init(&chPrms.tdCqRingPrms);
            chPrms.tdCqRingPrms.ringMem = pTdCqRingMem;
            chPrms.tdCqRingPrms.elemCnt = ENET_UDMA_TDCQ_RING_ELE_CNT;
            chPrms.tdCqRingPrms.mode    = TISCI_MSG_VALUE_RM_RING_MODE_RING;
        }
        else
        {
            retVal = UDMA_EALLOC;
        }
#else
        chPrms.chNum = EnetUdma_getMappedRxChNum(hPer->enetType, hPer->instId, chIdx);
#endif

#if (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
        if (Enet_isCpswFamily(hEnetUdma->enetType))
        {
            chPrms.mappedChGrp   = UDMA_MAPPED_RX_GROUP_CPSW;
        }
        else if (hEnetUdma->enetType == ENET_ICSSG_SWITCH)
        {
            if (0U == hEnetUdma->instId)
            {
                chPrms.mappedChGrp   = UDMA_MAPPED_RX_GROUP_ICSSG_0;
            }
            else if (1U == hEnetUdma->instId)
            {
                chPrms.mappedChGrp   = UDMA_MAPPED_RX_GROUP_ICSSG_1;
            }
            else
            {
                Enet_assert(false);
            }
        }
        else if (hEnetUdma->enetType == ENET_ICSSG_DUALMAC)
        {
            if ((0U == hEnetUdma->instId) || (1U == hEnetUdma->instId))
            {
                chPrms.mappedChGrp   = UDMA_MAPPED_RX_GROUP_ICSSG_0;
            }
            else if ((2U == hEnetUdma->instId) || (3U == hEnetUdma->instId))
            {
                chPrms.mappedChGrp   = UDMA_MAPPED_RX_GROUP_ICSSG_1;
            }
            else
            {
                Enet_assert(false);
            }
        }
        else
        {
            Enet_assert(false);
        }
#endif

        if (UDMA_SOK == retVal)
        {
            /* Open the UDMA channel */
            retVal = Udma_chOpen(hUdmaDrv, hUdmaCh, chType, &chPrms);
            ENETTRACE_ERR_IF((retVal != UDMA_SOK),
                               "[Enet UDMA] UDMA RX Channel open failed: 0x%x\n", retVal);
            if (UDMA_SOK == retVal)
            {
                chOpenFlag = true;
            }
        }
        else
        {
            ENETTRACE_ERR("[Enet UDMA] Teardown CQ ring memory allocation for Rx channel failed\n");
            retVal = UDMA_EALLOC;
        }
    }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    if (UDMA_SOK == retVal)
    {
        /* Configure RX parameters */
        hFlow  = &pRxCh->flowUdmaObj;
        retVal = Udma_flowAlloc(hUdmaDrv, hFlow, totalRxFlowCount);

        ENETTRACE_ERR_IF((retVal != UDMA_SOK),
                           "[Enet UDMA] UDMA flow alloc failed!!: 0x%x\n", retVal);
        if (UDMA_SOK == retVal)
        {
            allocFlow = true;

            if (totalRxFlowCount != Udma_flowGetCount(hFlow))
            {
                ENETTRACE_ERR("[Enet UDMA] Allocated flow Count doesn't match num requested!!\n");
                retVal = UDMA_EALLOC;
            }
        }
    }
#endif

    if (UDMA_SOK == retVal)
    {
        UdmaChRxPrms_init(&rxPrms, chType);

        rxPrms.dmaPriority        = pRxChInitPrms->dmaPriority;
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        rxPrms.flowIdFwRangeStart = Udma_flowGetNum(hFlow);
        rxPrms.flowIdFwRangeCnt   = Udma_flowGetCount(hFlow);
#endif

        rxPrms.configDefaultFlow = FALSE;
        retVal = Udma_chConfigRx(hUdmaCh, &rxPrms);
        if (UDMA_SOK == retVal)
        {
            /* Enable the UDMA channel */
            retVal = Udma_chEnable(&pRxCh->udmaChObj);
        }
        else
        {
            ENETTRACE_ERR("[Enet UDMA] UDMA RX channel enable failed, closing channel !!\n");
            retVal     = Udma_chClose(hUdmaCh);
            chOpenFlag = false;
        }
    }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    /* Get teardown cqRing handle */
    if (UDMA_SOK == retVal)
    {
        pRxCh->tdCqRing = Udma_chGetTdCqRingHandle(hUdmaCh);

        if(NULL == pRxCh->tdCqRing)
        {
            retVal = UDMA_EALLOC;
        }
    }
#endif

    if (UDMA_SOK == retVal)
    {
        /* Save channel config */
        hEnetUdma->rxChObj[chIdx].rxChInitPrms = *pRxChInitPrms;
        pRxCh->initFlag                 = true;
        pRxCh->enetType                 = hEnetUdma->enetType;
        pRxCh->instId                   = hEnetUdma->instId;
    }
    else
    {
        /* Error. Free-up resource if allocated */
        if (chOpenFlag)
        {
            retVal = Udma_chClose(hUdmaCh);
        }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        if (allocFlow)
        {
            retVal = Udma_flowFree(&pRxCh->flowUdmaObj);
        }
#endif

        /* return error as channel open has failed */
        retVal = UDMA_EFAIL;
    }

    EnetOsal_restoreAllIntr(intrKey);

    return retVal;
}

int32_t EnetUdma_closeRxCh(EnetDma_Handle hEnetUdma,
                           uint32_t chIdx)
{
    int32_t retVal = UDMA_SOK;
    EnetUdma_RxChObj *pRxCh;
    uintptr_t intrKey;

    // TODO how to check all flows are closed?
    // TODO check all Rx flows & Tx channels & are closed

    /* Error check */
    if (hEnetUdma == NULL)
    {
        ENETTRACE_ERR("[Enet UDMA Error] DMA handle is NULL !!\n");
        retVal = UDMA_EBADARGS;
    }

    if ((UDMA_SOK == retVal) && (!hEnetUdma->initFlag))
    {
        ENETTRACE_ERR("[Enet UDMA Error] DMA not initialized!!\n");
        retVal = UDMA_EBADARGS;
    }

    if (UDMA_SOK == retVal)
    {
        pRxCh = &hEnetUdma->rxChObj[chIdx];
        if (!pRxCh->initFlag)
        {
            ENETTRACE_ERR("[Enet UDMA Error] CPSW RX Channel not initialized!!\n");
            retVal = UDMA_EFAIL;
        }
    }

    intrKey = EnetOsal_disableAllIntr();

    if (UDMA_SOK == retVal)
    {
        /* Disable the UDMA channel */
        retVal += Udma_chDisable(&pRxCh->udmaChObj, ENET_UDMA_TEARDOWN_TIME_MS);
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        /* Dequeue the teardown response */
        if (UDMA_SOK == retVal)
        {
            CSL_UdmapTdResponse tdResp;
            void *tdCqRingMemPtr = NULL;

            tdCqRingMemPtr = Udma_ringGetMemPtr(pRxCh->tdCqRing);

            do
            {
                retVal = Udma_chDequeueTdResponse(&pRxCh->udmaChObj, &tdResp);
            }
            while (UDMA_SOK != retVal);

            if (FALSE == tdResp.tdIndicator)
            {
                ENETTRACE_ERR("[Enet UDMA] RX channel TearDown failed!!\n");
                retVal += UDMA_EFAIL;
            }

            /* Free tdCq ring memory */
            EnetUdma_memMgrFreeTdCqRingMemObj(tdCqRingMemPtr);
        }
#endif
    }

    // TODO flush FQ ring
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    if (UDMA_SOK == retVal)
    {
        /* free all flow ids allocated for this channel */
        retVal += Udma_flowFree(&pRxCh->flowUdmaObj);
    }
#endif

    if (UDMA_SOK == retVal)
    {
        /* Close the UDMA channel */
        retVal += Udma_chClose(&pRxCh->udmaChObj);
    }

    EnetOsal_restoreAllIntr(intrKey);

    return retVal;
}

Udma_RingHandle EnetUdma_getTxChFqHandle(EnetDma_TxChHandle hTxCh)
{
    Enet_assert(NULL != hTxCh);
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    Enet_assert(NULL != hTxCh->fqRing);
    return hTxCh->fqRing;
#elif (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
    Enet_assert(NULL != hTxCh->cqRing);
    return hTxCh->cqRing;
#endif

}

EnetUdma_DmaDescQHandle EnetUdma_getTxChDescPoolHandle(EnetDma_TxChHandle hTxCh)
{
    Enet_assert(NULL != hTxCh);
    return hTxCh->hDmaDescPool;
}

Udma_RingHandle EnetUdma_getRxFlowFqHandle(EnetDma_RxChHandle hRxFlow)
{
    Enet_assert(NULL != hRxFlow);
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    Enet_assert(NULL != hRxFlow->fqRing);
    return hRxFlow->fqRing;
#elif (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
    Enet_assert(NULL != hRxFlow->cqRing);
    return hRxFlow->cqRing;
#endif

}

EnetUdma_DmaDescQHandle EnetUdma_getRxFlowDescPoolHandle(EnetDma_RxChHandle hRxFlow)
{
    Enet_assert(NULL != hRxFlow);
    return hRxFlow->hDmaDescPool;
}

#if (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
static uint32_t EnetUdma_getMappedRxChNum(Enet_Type enetType,
                                   uint32_t instId,
                                   uint32_t chIdx)
{
    uint32_t chNum = 0U;

    if (Enet_isCpswFamily(enetType))
    {
        chNum = CSL_DMSS_PKTDMA_RX_CHANS_CPSW_START;
    }
#if defined(SOC_AM243X) || defined(SOC_AM64X)
    else if (enetType == ENET_ICSSG_SWITCH)
    {
        if (0U == instId)
        {
            if (0U == chIdx)
            {
                /* UDMA_MAPPED_RX_GROUP_ICSSG_0 Ch 0. Mapping set in enet_soc.h and
                 * returned via EnetSoc_getRxChPeerId based on "chID" */
                chNum = CSL_DMSS_PKTDMA_RX_CHANS_ICSSG0_START;
            }
            else if (1U == chIdx)
            {
                /* UDMA_MAPPED_RX_GROUP_ICSSG_0 Ch 0. Mapping set in enet_soc.h and
                 * returned via EnetSoc_getRxChPeerId based on "chID" */
                chNum = CSL_DMSS_PKTDMA_RX_CHANS_ICSSG0_START + 1U;
            }
        }
        else if (1U == instId)
        {
            if (0U == chIdx)
            {
                /* UDMA_MAPPED_RX_GROUP_ICSSG_0 Ch 0. Mapping set in enet_soc.h and
                 * returned via EnetSoc_getRxChPeerId based on "chID" */
                chNum = CSL_DMSS_PKTDMA_RX_CHANS_ICSSG1_START;
            }
            else if (1U == chIdx)
            {
                /* UDMA_MAPPED_RX_GROUP_ICSSG_0 Ch 0. Mapping set in enet_soc.h and
                 * returned via EnetSoc_getRxChPeerId based on "chID" */
                chNum = CSL_DMSS_PKTDMA_RX_CHANS_ICSSG1_START + 1U;
            }
        }
        else
        {
            Enet_assert(false);
        }
    }
    else if (enetType == ENET_ICSSG_DUALMAC)
    {
        if (0U == instId)
        {
            /* UDMA_MAPPED_RX_GROUP_ICSSG_0 Ch 0. Mapping set in enet_soc.h and
             * returned via EnetSoc_getRxChPeerId based on "instId" in dual MAC &
             * chId in Switch mode */
            chNum = CSL_DMSS_PKTDMA_RX_CHANS_ICSSG0_START;
        }
        else if (1U == instId)
        {
            /* UDMA_MAPPED_RX_GROUP_ICSSG_0 Ch 1. Mapping set in enet_soc.h and
             * returned via EnetSoc_getRxChPeerId based on "instId" in dual MAC &
             * chId in Switch mode*/
            chNum = CSL_DMSS_PKTDMA_RX_CHANS_ICSSG0_START + 1U;
        }
        else if (2U == instId)
        {
            /* UDMA_MAPPED_RX_GROUP_ICSSG_1 Ch 0. Mapping set in enet_soc.h and
             * returned via EnetSoc_getRxChPeerId based on "instId" in dual MAC &
             * chId in Switch mode*/
            chNum = CSL_DMSS_PKTDMA_RX_CHANS_ICSSG1_START;
        }
        else if (3U == instId)
        {
            /* UDMA_MAPPED_RX_GROUP_ICSSG_1 Ch 1. Mapping set in enet_soc.h and
             * returned via EnetSoc_getRxChPeerId based on "instId" in dual MAC &
             * chId in Switch mode*/
            chNum = CSL_DMSS_PKTDMA_RX_CHANS_ICSSG1_START + 1U;
        }
        else
        {
            Enet_assert(false);
        }
    }
#endif
    else
    {
        Enet_assert(false);
    }

    return chNum;
}
#endif

uint32_t EnetUdma_getMappedRxChStartIdx(EnetUdma_RxChObj *pRxCh)
{
    uint32_t chNum, chStartIdx = 0U, index;
    /* Pass RX channel number in case of AM64x */
    extern const Udma_MappedChRingAttributes gUdmaRxMappedChRingAttributes[];

    chNum = EnetUdma_getMappedRxChNum(pRxCh->enetType,
                                      pRxCh->instId,
                                      pRxCh->chIdx);
    /* Calculate index by subtracting the start idx of mapped channels
     * (For AM64x, mapped channel starts with CPSW channel.) */
    index = chNum - CSL_DMSS_PKTDMA_RX_CHANS_CPSW_START;
#if defined(SOC_AM62AX)
    /*TODO!: Remove hard coding of numbers */
    chStartIdx = (gUdmaRxMappedChRingAttributes[index].startFreeRing - 99U);
#else
    chStartIdx = (gUdmaRxMappedChRingAttributes[index].startFreeRing - 112U);
#endif

    return chStartIdx;
}

uint32_t EnetUdma_getRxChFlowStartIdx(EnetDma_Handle hEnetUdma,
                                      uint32_t chIdx)
{
    EnetUdma_RxChObj *pRxCh = &hEnetUdma->rxChObj[chIdx];
    uint32_t flowStartIdx;

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    flowStartIdx = Udma_flowGetNum(&pRxCh->flowUdmaObj);
#else
    flowStartIdx = EnetUdma_getMappedRxChStartIdx(pRxCh);
#endif

    return flowStartIdx;
}

uint32_t EnetUdma_getRxFlowCnt(EnetDma_Handle hEnetUdma,
                               uint32_t chIdx)
{
    uint32_t flowCnt;
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    EnetUdma_RxChObj *pRxCh = &hEnetUdma->rxChObj[chIdx];


    flowCnt = Udma_flowGetCount(&pRxCh->flowUdmaObj);
#else
    /* Pass RX channel number in case of AM64x
     * Note - In AM64x/AM243x, UDMA allocates 16 flows for CPSW and 64 flows for
     * ICSSG(each instance), which are split into 16 for each of 4 RX channels in
     * udma_soc (gUdmaRxMappedChRingAttributes). So we can have upto 16 flows per channel
     * but as ICSSG_DUALMAC_RX_FLOW_NUM is set to 8 and we know we won't use more than
     * 8 flows for CPSW as well */
    flowCnt = 8U;
#endif

    return flowCnt;
}

int32_t EnetUdma_checkRxFlowSanity(EnetDma_RxChHandle hRxFlow,
                                  uint32_t margin)
{
    int32_t retVal       = ENET_SOK;
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    uint32_t allocPkts   = hRxFlow->rxFlowPrms.numRxPkts;
    uint32_t fqRingHwOcc = CSL_ringaccGetRingHwOcc(&hRxFlow->fqRing->drvHandle->raRegs,
                                                   Udma_ringGetNum(hRxFlow->fqRing));
    uint32_t cqRingHwOcc = CSL_ringaccGetRingHwOcc(&hRxFlow->cqRing->drvHandle->raRegs,
                                                   Udma_ringGetNum(hRxFlow->cqRing));
    uint32_t withHwPkts = fqRingHwOcc + cqRingHwOcc;

    /* App allocated should match packets with DMA module and HW.
     * Also we consider margin packets in fly while reading the HW status */
    if ((allocPkts - margin) < (EnetUdma_dmaDescQCount(hRxFlow->hDmaDescPool) + withHwPkts))
    {
        retVal = ENET_SOK;
    }
    else
    {
        ENETTRACE_WARN("hRxFlow->rxFlowPrms.numTxPkts   = %d\n", hRxFlow->rxFlowPrms.numRxPkts);
        ENETTRACE_WARN("EnetUdma_dmaDescQCount(hRxFlow->hDmaDescPool) = %d\n", EnetUdma_dmaDescQCount(hRxFlow->hDmaDescPool));
        ENETTRACE_WARN("hRxFlow->fqRing->pRtRegs->HWOCC = %d\n", fqRingHwOcc);
        ENETTRACE_WARN("hRxFlow->cqRing->pRtRegs->HWOCC = %d\n", cqRingHwOcc);

        retVal = ENET_EFAIL;
    }
#endif

    return retVal;
}

int32_t EnetUdma_checkTxChSanity(EnetDma_TxChHandle hTxCh,
                                uint32_t margin)
{
    int32_t retVal       = ENET_SOK;
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    uint32_t allocPkts   = hTxCh->txChPrms.numTxPkts;
    uint32_t fqRingHwOcc = CSL_ringaccGetRingHwOcc(&hTxCh->fqRing->drvHandle->raRegs,
                                                   Udma_ringGetNum(hTxCh->fqRing));
    uint32_t cqRingHwOcc = CSL_ringaccGetRingHwOcc(&hTxCh->cqRing->drvHandle->raRegs,
                                                   Udma_ringGetNum(hTxCh->cqRing));
    uint32_t withHwPkts = fqRingHwOcc + cqRingHwOcc;

    /* App allocated should match packets with DMA module and HW.
     * Also we consider margin packets in fly while reading the HW status */
    if ((allocPkts - margin) < (hTxCh->hDmaDescPool->count + withHwPkts))
    {
        retVal = ENET_SOK;
    }
    else
    {
        ENETTRACE_WARN("hTxCh->txChPrms.numTxPkts     = %d\n", hTxCh->txChPrms.numTxPkts);
        ENETTRACE_WARN("hTxCh->hDmaDescPool->count   = %d\n", hTxCh->hDmaDescPool->count);
        ENETTRACE_WARN("hTxCh->fqRing->pRtRegs->HWOCC = %d\n", fqRingHwOcc);
        ENETTRACE_WARN("hTxCh->cqRing->pRtRegs->HWOCC = %d\n", cqRingHwOcc);
        retVal = ENET_EFAIL;
    }
#endif

    return retVal;
}

static int32_t EnetUdma_checkRxFlowParams(EnetUdma_OpenRxFlowPrms *pRxFlowPrms)
{
    int32_t retVal = UDMA_SOK;

    if (NULL == pRxFlowPrms)
    {
        ENETTRACE_ERR("[Enet UDMA] Flow params is NULL!!\n");
        retVal = UDMA_EBADARGS;
    }
    else
    {
        if (NULL == pRxFlowPrms->hUdmaDrv)
        {
            ENETTRACE_ERR("[Enet UDMA] UDMA not opened !!\n");
            retVal = UDMA_EBADARGS;
        }
    }

    if (UDMA_SOK == retVal)
    {
        if ((NULL == pRxFlowPrms->ringMemAllocFxn) ||
            (NULL == pRxFlowPrms->ringMemFreeFxn) ||
            (NULL == pRxFlowPrms->dmaDescAllocFxn) ||
            (NULL == pRxFlowPrms->dmaDescFreeFxn))
        {
            ENETTRACE_ERR_IF((NULL == pRxFlowPrms->ringMemAllocFxn), "[Enet UDMA] ringMemAllocFxn is NULL!!\n");
            ENETTRACE_ERR_IF((NULL == pRxFlowPrms->ringMemFreeFxn), "[Enet UDMA] ringMemFreeFxn is NULL!!\n");
            ENETTRACE_ERR_IF((NULL == pRxFlowPrms->dmaDescAllocFxn), "[Enet UDMA] dmaDescAllocFxn is NULL!!\n");
            ENETTRACE_ERR_IF((NULL == pRxFlowPrms->dmaDescFreeFxn), "[Enet UDMA] dmaDescFreeFxn is NULL!!\n");
            retVal = UDMA_EBADARGS;
        }

        if (ENET_RM_RXFLOWIDX_INVALID == pRxFlowPrms->flowIdx)
        {
            ENETTRACE_ERR("[Enet UDMA] Invalid flow id for CPSW RX flow open !!\n", pRxFlowPrms->flowIdx);
            retVal = UDMA_EBADARGS;
        }
    }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    if ((UDMA_SOK == retVal) && (pRxFlowPrms->udmaChPrms.cqRingPrms.useRingMon))
    {
        ENETTRACE_ERR("[Enet UDMA] Driver supports Ring monitor only for RX FQ !!\n");
        retVal = UDMA_EBADARGS;
    }
#endif

    return retVal;
}

static int32_t EnetUdma_checkTxChParams(EnetUdma_OpenTxChPrms *pTxChPrms)
{
    int32_t retVal = UDMA_SOK;

    if (NULL == pTxChPrms)
    {
        ENETTRACE_ERR("[Enet UDMA] TX channel params can't be NULL !!\n");
        retVal = UDMA_EBADARGS;
    }
    else
    {
        if (NULL == pTxChPrms->hUdmaDrv)
        {
            /* DMA should be opened before opening Ch/Flow */
            ENETTRACE_ERR("[Enet UDMA] UDMA not opened !!\n");
            retVal = UDMA_EBADARGS;
        }
    }

    if (UDMA_SOK == retVal)
    {
        if ((NULL == pTxChPrms->ringMemAllocFxn) ||
            (NULL == pTxChPrms->ringMemFreeFxn) ||
            (NULL == pTxChPrms->dmaDescAllocFxn) ||
            (NULL == pTxChPrms->dmaDescFreeFxn))
        {
            ENETTRACE_ERR_IF((NULL == pTxChPrms->ringMemAllocFxn), "[Enet UDMA] ringMemAllocFxn is NULL!!\n");
            ENETTRACE_ERR_IF((NULL == pTxChPrms->ringMemFreeFxn), "[Enet UDMA] ringMemFreeFxn is NULL!!\n");
            ENETTRACE_ERR_IF((NULL == pTxChPrms->dmaDescAllocFxn), "[Enet UDMA] dmaDescAllocFxn is NULL!!\n");
            ENETTRACE_ERR_IF((NULL == pTxChPrms->dmaDescFreeFxn), "[Enet UDMA] dmaDescFreeFxn is NULL!!\n");
            retVal = UDMA_EBADARGS;
        }

        if (ENET_RM_TXCHNUM_INVALID == pTxChPrms->chNum)
        {
            ENETTRACE_ERR("[Enet UDMA] Invalid channel number for CPSW TX channel open !!: %d\n", pTxChPrms->chNum);
            retVal = UDMA_EBADARGS;
        }
    }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    if ((UDMA_SOK == retVal) &&
        ((pTxChPrms->udmaChPrms.fqRingPrms.useRingMon) || (pTxChPrms->udmaChPrms.cqRingPrms.useRingMon)))
    {
        ENETTRACE_ERR("[Enet UDMA] Driver supports Ring monitor only for RX FQ !!\n");
        retVal = UDMA_EBADARGS;
    }
#endif

    return retVal;
}

/*
 * Drain packets in ISR due to ring level fix
 * Udma ring completion (CQ) interrupt is a self clearing level interrupt.
 * The CQ ISR must drain the ring in Udma callback. This requires a fix in Udma driver
 * to not clear ring interrupt explicitly and change in CpswDma to drain buffers in ISR callback
 * Without this change, Ring interrupt will randomly stop occurring
 */
static void EnetUdma_drainIsrCq(EnetQ *dstQ,
                               EnetQ *isrCq)
{
    uintptr_t key = EnetOsal_disableAllIntr();

    EnetQueue_append(dstQ, isrCq);
    EnetQueue_initQ(isrCq);

    EnetOsal_restoreAllIntr(key);
}

void EnetUdma_initDataPathParams(EnetDma_initCfg *pDmaConfig)
{
    pDmaConfig->hUdmaDrv = NULL;
}

EnetDma_Handle EnetUdma_initDataPath(Enet_Type enetType,
                                     uint32_t instId,
                                     const EnetDma_initCfg *pDmaInitCfg)
{
    EnetUdma_Cfg cfg;
    Enet_Handle hEnet = NULL;
    EnetDma_Handle hDmaHandle = NULL;

    EnetUdma_initCfg(enetType, &cfg);

    if (pDmaInitCfg != NULL)
    {
        cfg.hUdmaDrv = pDmaInitCfg->hUdmaDrv;

        hDmaHandle = EnetUdma_open(enetType, instId, &cfg);
        if (hDmaHandle != NULL)
        {
            hEnet = EnetSoc_getEnetHandle(enetType, instId);
            if (hEnet != NULL)
            {
                hDmaHandle->hPer = hEnet->enetPer;
            }
            else
            {
                Enet_devAssert(hEnet != NULL);
            }
        }
    }

    return hDmaHandle;
}

int32_t EnetUdma_deInitDataPath(EnetDma_Handle hEnetUdma)
{
    int32_t status;

    status = EnetUdma_close(hEnetUdma);
    return status;
}

int32_t EnetDma_registerRxEventCb(EnetDma_RxChHandle hRxCh, EnetDma_PktNotifyCb notifyCb, void *cbArg)
{
    uintptr_t             key;
    int32_t status        = ENET_SOK;

    key = EnetOsal_disableAllIntr();

    hRxCh->rxFlowPrms.cbArg = cbArg;
    hRxCh->rxFlowPrms.notifyCb = notifyCb;

    EnetOsal_restoreAllIntr(key);
    return status;
}

int32_t EnetDma_registerTxEventCb(EnetDma_TxChHandle hTxCh, EnetDma_PktNotifyCb notifyCb, void *cbArg)
{
    uintptr_t             key;
    int32_t status        = ENET_SOK;

    key = EnetOsal_disableAllIntr();

    hTxCh->txChPrms.cbArg = cbArg;
    hTxCh->txChPrms.notifyCb = notifyCb;

    EnetOsal_restoreAllIntr(key);
    return status;
}

/* End of file */
