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
 *  \file enet_udma_priv.c
 *
 *  \brief This file contains UDMA private function definations.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <include/core/enet_base.h>
#include <include/core/enet_soc.h>
#include <include/core/enet_types.h>
#include <include/core/enet_queue.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_osal.h>
#include <priv/core/enet_trace_priv.h>
#include <include/mod/cpsw_ale.h>
#include <include/core/enet_rm.h>
#include <include/core/enet_dma.h>
#include <src/per/firmware/icssg/fw_mem_map.h>
#include <hw_include/cslr_icss.h>

#include <priv/per/enet_hostport_udma.h>
#include <drivers/sciclient.h>
#include <drivers/udma/soc/udma_soc.h>

#include "enet_udma_priv.h"
/* hack to access gUdmaTxMappedChRingAttributes */
#include "drivers/udma/udma_priv.h"
#include "enet_udma_memcfg.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

typedef EnetQ EnetUdma_RxFlowObjMemQ;

typedef EnetQ EnetUdma_TxChObjMemQ;

typedef EnetQ EnetUdma_RingMonObjMemQ;

typedef EnetQ EnetUdma_tdCqRingObjMemQ;

#define ENET_UDMA_CPSW_IS_HBD_IDX(i) ( i > ENET_UDMA_CPSW_HOSTPKTDESC_INDEX)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief
 */
typedef struct EnetUdma_MemMgrObj_s
{
    /*! Mem manager init flag. Used to track no. of clients using mem manager */
    uint32_t clientCnt;

    /*! RX flow driver object Q */
    EnetUdma_RxFlowObjMemQ rxFlowObjMemQ;

    /*! TX channel driver object Q */
    EnetUdma_TxChObjMemQ txChObjMemQ;

    /*! TX channel driver object Q */
    EnetUdma_RingMonObjMemQ ringMonObjMemQ;

    /*! Teardown CQ memory Q */
    EnetUdma_tdCqRingObjMemQ tdCqRingObjMemQ;
} EnetUdma_MemMgrObj;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
static uint8_t gRsvdFlowRingMemFQ[ENET_UDMA_RSVDFLOW_RX_PKTS_NUM][ENET_UDMA_RING_MEM_SIZE]
__attribute__ ((aligned(UDMA_CACHELINE_ALIGNMENT), section(".bss:ENET_DMA_OBJ_MEM")));
#endif

static uint8_t gRsvdFlowRingMemCQ[ENET_UDMA_RSVDFLOW_RX_PKTS_NUM][ENET_UDMA_RING_MEM_SIZE]
__attribute__ ((aligned(UDMA_CACHELINE_ALIGNMENT), section(".bss:ENET_DMA_OBJ_MEM")));

/* Cpsw mem utils driver object */
static EnetUdma_MemMgrObj gEnetUdmaMemMgrObj = {.clientCnt = 0U};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static int32_t EnetUdma_processRetrievedDesc(EnetPer_Handle hPer,
                                             EnetUdma_DmaDesc *pDmaDesc,
                                            EnetDma_PktQ *pFromHwQueue,
                                            EnetUdma_DmaDescQ *pDmaDescQ,
                                            EnetUdma_Dir transferDir)
{
    int32_t retVal = UDMA_SOK;
    EnetUdma_CpswHpdDesc *pHpdDesc;
    EnetDma_Pkt *dmaPkt;
    EnetUdma_CppiTxStatus *cppiTxStatus;
    uint32_t srcTag;
    uint32_t scatterSegmentIndex;
    uint32_t lastFilledSegmentIndex;
    uint32_t totalPacketFilledLen = 0U;

    if (NULL != pDmaDesc)
    {
        pHpdDesc = &pDmaDesc->hpdDesc;

        /* Check that the desc address from cqRing is valid */
        if (NULL != pHpdDesc)
        {
            dmaPkt = pDmaDesc->dmaPkt;

            if (ENET_UDMA_DIR_RX == transferDir)
            {
                scatterSegmentIndex = 0;
                totalPacketFilledLen = CSL_udmapCppi5GetPktLen(&pHpdDesc->hostDesc);
                lastFilledSegmentIndex = 0;

                while (scatterSegmentIndex < dmaPkt->sgList.numScatterSegments)
                {
                    if (scatterSegmentIndex == ENET_UDMA_CPSW_HOSTPKTDESC_INDEX)
                    {
                        dmaPkt->sgList.list[scatterSegmentIndex].segmentFilledLen =
                                EnetUtils_min(CSL_udmapCppi5GetOrgBufferLen(&pHpdDesc->hostDesc),
                                              totalPacketFilledLen);
                    }
                    else
                    {
                        dmaPkt->sgList.list[scatterSegmentIndex].segmentFilledLen =
                                EnetUtils_min(CSL_udmapCppi5GetOrgBufferLen(&(pDmaDesc->hostBufDesc[(scatterSegmentIndex- ENET_UDMA_CPSW_HOSTBUFDESC_INDEX)].desc)),
                                              totalPacketFilledLen);
                    }
                    if (dmaPkt->sgList.list[scatterSegmentIndex].segmentFilledLen != 0)
                    {
                        lastFilledSegmentIndex = scatterSegmentIndex;
                    }
                    totalPacketFilledLen -= dmaPkt->sgList.list[scatterSegmentIndex].segmentFilledLen;
                    scatterSegmentIndex++;
                }
                if (Enet_isIcssFamily(hPer->enetType))
                {
#if ENET_CFG_IS_ON(DEV_ERROR)
                   Enet_assert((lastFilledSegmentIndex < ENET_ARRAYSIZE(dmaPkt->sgList.list))
                           && (dmaPkt->sgList.list[lastFilledSegmentIndex].segmentFilledLen >= 4));
#endif
                   dmaPkt->sgList.list[lastFilledSegmentIndex].segmentFilledLen -= 4;
                }
#if ENET_CFG_IS_ON(DEV_ERROR)
                /* Protocol-specific data is should be in the descriptor (psLocation set to
                 * TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_PS_END_PD */
                Enet_assert(0U == CSL_udmapCppi5GetPsDataLoc(&pHpdDesc->hostDesc));
                /* EPI(Extended packet info) block should be present in the descriptor (einfoPresent set to
                 * TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_EINFO_PRESENT */
                Enet_assert(true == CSL_udmapCppi5IsEpiDataPresent(&pHpdDesc->hostDesc));
#endif
                cppiTxStatus           = (EnetUdma_CppiTxStatus *)pHpdDesc->psInfo;

                dmaPkt->chkSumInfo     = Enet_isIcssFamily(hPer->enetType) ? 0U : cppiTxStatus->chkSumInfo;
                dmaPkt->tsInfo.rxPktTs = ((((uint64_t)cppiTxStatus->tsHigh) << 32U) |
                                                ((uint64_t)cppiTxStatus->tsLow));

                /* Convert timestamp to nanosecs if peripheral provides a conversion function */
                if (hPer->convertTs != NULL)
                {
                    dmaPkt->tsInfo.rxPktTs = hPer->convertTs(hPer, dmaPkt->tsInfo.rxPktTs);
                }

                srcTag = CSL_udmapCppi5GetSrcTag(&pHpdDesc->hostDesc) & ENET_UDMA_HPD_SRC_TAG_LOW_MASK;
                if ((hPer->enetType == ENET_ICSSG_DUALMAC) || (hPer->enetType == ENET_ICSSG_SWITCH))
                {
                    dmaPkt->rxPortNum = CPSW_ALE_ALEPORT_TO_MACPORT(srcTag + 1);
                }
                else
                {
                    dmaPkt->rxPortNum = CPSW_ALE_ALEPORT_TO_MACPORT(srcTag);
                }

            }

#if ENET_CFG_IS_ON(DEV_ERROR)
            Enet_assert((uint64_t)dmaPkt->sgList.list[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].bufPtr == CSL_udmapCppi5GetBufferAddr(&pHpdDesc->hostDesc));
            Enet_assert((uint64_t)dmaPkt->sgList.list[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].bufPtr == pHpdDesc->hostDesc.orgBufPtr);
#endif

            EnetDma_checkPktState(&dmaPkt->pktState,
                                    ENET_PKTSTATE_MODULE_DRIVER,
                                    (uint32_t)ENET_PKTSTATE_DMA_WITH_HW,
                                    (uint32_t)ENET_PKTSTATE_DMA_NOT_WITH_HW);

            EnetQueue_enq(pFromHwQueue, &dmaPkt->node);

            /* Enq free Tx packet in txFreeDmaDescQ and full Rx into rxReadyDmaDescQ */
            EnetUdma_dmaDescEnque(pDmaDescQ, pDmaDesc);
        }
        else
        {
            retVal = UDMA_EFAIL;
        }
    }
    else
    {
        retVal = UDMA_EFAIL;
    }

    return retVal;
}

int32_t EnetUdma_retrievePkts(EnetPer_Handle hPer,
                              Udma_RingHandle hUdmaRing,
                              EnetDma_PktQ *pFromHwQueue,
                              EnetUdma_DmaDescQ *pDmaDescQ,
                              bool disableCacheOpsFlag,
                              EnetUdma_Dir transferDir)
{
    int32_t retVal = UDMA_SOK;
    EnetUdma_DmaDesc *pDmaDesc;
    bool isExposedRing;
    uint32_t ringOcc = 0U;
    uint64_t *ringMemPtr = NULL, *currRingMemPtr = NULL;
    uint32_t ringMemEleCnt = 0U, ringRdIdx = 0U;
    uint32_t i = 0U;
    int32_t retrieveCnt = 0;

    EnetQueue_initQ(pFromHwQueue);
    isExposedRing = (Udma_ringGetMode(hUdmaRing) == TISCI_MSG_VALUE_RM_RING_MODE_RING);
#if defined(SOC_AM64X) || defined(SOC_AM243X) || defined(SOC_AM62AX)
    isExposedRing = 0U;
#endif

    if (isExposedRing == true)
    {
        /* Disable interrupts before retrieving descriptors using Prime APIs to prevent race condition.
         * As ring occupancy is updated by the driver only at the end of this function, CQ interrupts
         * occurring in b/w might result in context switch and update the occupancy value more than once
         * for same set of descriptors. */
        uintptr_t key = EnetOsal_disableAllIntr();

        ringOcc = Udma_ringGetReverseRingOcc(hUdmaRing);
        while (ringOcc != 0U)
        {
            ringMemPtr = (uint64_t *)Udma_ringGetMemPtr(hUdmaRing);
            ringRdIdx = Udma_ringGetRdIdx(hUdmaRing);
            ringMemEleCnt = Udma_ringGetElementCnt(hUdmaRing);
            currRingMemPtr = ringMemPtr + ringRdIdx;

            /* Invalidate the Ring memory before reading from the ring */
            if ((ringRdIdx + ringOcc) > ringMemEleCnt)
            {
                EnetOsal_cacheInv(currRingMemPtr,
                                  ((ringMemEleCnt - ringRdIdx) * ENET_UDMA_RING_MEM_SIZE));
                EnetOsal_cacheInv(ringMemPtr,
                                  ((ringOcc - ringMemEleCnt + ringRdIdx) * ENET_UDMA_RING_MEM_SIZE));
            }
            else
            {
                EnetOsal_cacheInv(currRingMemPtr,
                                  (ringOcc * ENET_UDMA_RING_MEM_SIZE));
            }

            for (i = 0U; i < ringOcc; i++)
            {
                retVal = EnetUdma_ringDequeue(hUdmaRing,
                                             &pDmaDesc,
                                             disableCacheOpsFlag,
                                             transferDir);

                if ((UDMA_SOK == retVal) && (NULL != pDmaDesc))
                {
                    retVal = EnetUdma_processRetrievedDesc(hPer,
                                                           pDmaDesc,
                                                          pFromHwQueue,
                                                          pDmaDescQ,
                                                          transferDir);
                    if (UDMA_SOK != retVal)
                    {
                        break;
                    }
                }
                else
                {
                    break;
                }
            }

            if ((retVal == UDMA_SOK) && (ringOcc > 0U))
            {
                /* Make the count negative as the occupancy needs to be reduced after popping from ring */
                retrieveCnt = -1 * (int32_t)ringOcc;
                /* Set Ring door bell register with count of number of descriptors de-queued */
                Udma_ringSetDoorBell(hUdmaRing, retrieveCnt);
            }

            ringOcc = Udma_ringGetReverseRingOcc(hUdmaRing);
        }

        /* Re-enable interrupts */
        EnetOsal_restoreAllIntr(key);
    }
    /* If ring is in message mode */
    else
    {
        /* Dequeue descs from cqRing */
        retVal = EnetUdma_ringDequeue(hUdmaRing,
                                     &pDmaDesc,
                                     disableCacheOpsFlag,
                                     transferDir);

        while ((retVal == UDMA_SOK) && (NULL != pDmaDesc))
        {
            retVal = EnetUdma_processRetrievedDesc(hPer,
                                                   pDmaDesc,
                                                  pFromHwQueue,
                                                  pDmaDescQ,
                                                  transferDir);
            if (UDMA_SOK != retVal)
            {
                break;
            }

            retVal = EnetUdma_ringDequeue(hUdmaRing,
                                         &pDmaDesc,
                                         disableCacheOpsFlag,
                                         transferDir);
        }
    }

    /* It's ok if there are no more descs in cqRing */
    if (UDMA_ETIMEOUT == retVal)
    {
        retVal = UDMA_SOK;
    }

    return retVal;
}

int32_t EnetUdma_submitPkts(EnetPer_Handle hPer,
                            Udma_RingHandle hUdmaRing,
                           EnetDma_PktQ *pToHwQueue,
                           EnetUdma_DmaDescQ *pDmaDescQ,
                           bool disableCacheOpsFlag,
                           EnetUdma_Dir transferDir
#if (UDMA_SOC_CFG_PROXY_PRESENT == 1)
                           ,
                           Udma_ProxyHandle hUdmaProxy
#endif
                           )
{
    int32_t retVal = UDMA_SOK;
    EnetUdma_DmaDesc *pDmaDesc;
    EnetDma_Pkt *dmaPkt;
    uint32_t dstTag;
    EnetUdma_CppiRxControl *cppiRxCntr;
    uint32_t *iccsgTxTsId, *tsInfo;
    int32_t submitCnt = 0;
    bool isExposedRing;
    uint64_t *ringMemPtr = NULL, *currRingMemPtr = NULL;
    uint32_t ringWrIdx = 0U, ringMemEleCnt = 0U;
    uint32_t i;
#if defined(SOC_AM64X) || defined(SOC_AM243X)
    uint32_t tosIndex;
    uint8_t dscpIPv4En, tosVal;
#endif
    uint32_t totalPacketFilledLen = 0U;
    EnetUdma_SGListEntry *sgList;

    isExposedRing = (Udma_ringGetMode(hUdmaRing) == TISCI_MSG_VALUE_RM_RING_MODE_RING);
#if defined(SOC_AM64X) || defined(SOC_AM243X) || defined(SOC_AM62AX)
    isExposedRing = 0U;
#endif
    if (isExposedRing == true)
    {
        ringMemPtr = (uint64_t *)Udma_ringGetMemPtr(hUdmaRing);
        ringWrIdx = Udma_ringGetWrIdx(hUdmaRing);
        ringMemEleCnt = Udma_ringGetElementCnt(hUdmaRing);
    }

    /* Enqueue packets until fqRing is full */
    dmaPkt = (EnetDma_Pkt *)EnetQueue_deq(pToHwQueue);
    while (NULL != dmaPkt)
    {
        /* Enqueue desc to fqRing */
        pDmaDesc = EnetUdma_dmaDescDeque(pDmaDescQ);
        totalPacketFilledLen = 0U;

        if (NULL != pDmaDesc)
        {
            EnetUdma_CpswHpdDesc *pHpdDesc = (EnetUdma_CpswHpdDesc*) pDmaDesc;
            CSL_UdmapCppi5HMPD *pHostDesc = &pHpdDesc->hostDesc;

            pDmaDesc->dmaPkt = dmaPkt;
            sgList = dmaPkt->sgList.list;
            EnetDma_checkPktState(&pDmaDesc->dmaPkt->pktState,
                                  ENET_PKTSTATE_MODULE_DRIVER,
                                  (uint32_t) ENET_PKTSTATE_DMA_NOT_WITH_HW,
                                  (uint32_t) ENET_PKTSTATE_DMA_WITH_HW);

            CSL_UdmapCppi5HMPD *pHDesc = pHostDesc;
            CSL_UdmapCppi5HMPD *pHDescPrev = NULL;
            for (i = 0; i < dmaPkt->sgList.numScatterSegments; i++)
            {
                if (ENET_UDMA_CPSW_IS_HBD_IDX(i))
                {
                    pHDesc = &pDmaDesc->hostBufDesc[i - ENET_UDMA_CPSW_HOSTBUFDESC_INDEX].desc;
                    /* Link the Descs */
                    CSL_udmapCppi5LinkDesc(pHDescPrev, (uint64_t) pHDesc);
                }
                CSL_udmapCppi5SetBufferAddr(pHDesc, (uint64_t) sgList[i].bufPtr);
                if (ENET_UDMA_DIR_TX == transferDir)
                {
                    CSL_udmapCppi5SetBufferLen(pHDesc, sgList[i].segmentFilledLen);
                }
                else
                {
                    CSL_udmapCppi5SetBufferLen(pHDesc, sgList[i].segmentAllocLen);
                }
                CSL_udmapCppi5SetOrgBufferAddr(pHDesc, (uint64_t) sgList[i].bufPtr);
                CSL_udmapCppi5SetOrgBufferLen(pHDesc, sgList[i].segmentAllocLen);
                totalPacketFilledLen += sgList[i].segmentFilledLen;
                pHDescPrev = pHDesc;
            }
            /* Link the last segment's Desc to NULL */
            CSL_udmapCppi5LinkDesc(pHDesc, 0U);

            CSL_udmapCppi5HostSetPktLen(pHostDesc, totalPacketFilledLen);

            if (ENET_UDMA_DIR_TX == transferDir)
            {
#if ENET_CFG_IS_ON(DEV_ERROR)
                /* For TX all defaults are set in EnetUdma_buffDescInit function. We just confirm those
                 * are not changed here */
                Enet_assert(CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST == CSL_udmapCppi5GetDescType(pHostDesc));
                Enet_assert(0U == CSL_udmapCppi5GetPsDataLoc(pHostDesc));
                Enet_assert(true == CSL_udmapCppi5IsEpiDataPresent(pHostDesc));
                Enet_assert(ENET_UDMA_PROTOCOL_SPECIFIC_INFO_BLOCK_SIZE ==
                                CSL_udmapCppi5GetPsDataLen(pHostDesc));
#endif
                cppiRxCntr = (EnetUdma_CppiRxControl *)pHpdDesc->psInfo;

                if (Enet_isIcssFamily(hPer->enetType))
                {
                    cppiRxCntr->chkSumInfo = 0U;
                    tsInfo = (uint32_t *)&pHpdDesc->extendedPktInfo[4U];
                }
                else
                {
                    cppiRxCntr->chkSumInfo = dmaPkt->chkSumInfo;
                    tsInfo = (uint32_t *)&cppiRxCntr->tsInfo;
                }

                if (dmaPkt->tsInfo.enableHostTxTs == true)
                {
                    ENETUDMA_CPPIPSI_SET_TSEN(*tsInfo, 1U);

                    /* Below fields are only valid for CPSW. In ICSSG, whole tsInfo word is
                     * used for enabling timestamp.
                     * This is don't care/reserved word for ICSSG so we set without any check */
                    ENETUDMA_CPPIPSI_SET_DOMAIN(*tsInfo,
                                               dmaPkt->tsInfo.txPktDomain);
                    ENETUDMA_CPPIPSI_SET_MSGTYPE(*tsInfo,
                                                dmaPkt->tsInfo.txPktMsgType);
                    ENETUDMA_CPPIPSI_SET_SEQID(*tsInfo,
                                              dmaPkt->tsInfo.txPktSeqId);

                    /* Set host Tx timestamp flag back to false. */
                    dmaPkt->tsInfo.enableHostTxTs = false;

                    /* For ICSSG psinfo word 0 is used for passing cookie to the firmwareiccsgTxTsId
                     * This is don't care/reserved word for CPSW so we set without any check */
                    iccsgTxTsId = (uint32_t *)&pHpdDesc->extendedPktInfo[0U];
                    *iccsgTxTsId = dmaPkt->txTsId;
                }
                else
                {
                    ENETUDMA_CPPIPSI_SET_TSEN(*tsInfo, 0U);
                }
#if defined(SOC_AM64X) || defined(SOC_AM243X)
                if (Enet_isIcssFamily(hPer->enetType))
                {
                    uintptr_t baseAddr = (uintptr_t)hPer->virtAddr;

                    if (hPer->enetType == ENET_ICSSG_DUALMAC)
                    {
                        if ((hPer->instId == 0) || (hPer->instId == 2))
                        {
                            baseAddr += CSL_ICSS_G_DRAM0_SLV_RAM_REGS_BASE;
                        }
                        else
                        {
                            /*DRAM1*/
                            baseAddr += CSL_ICSS_G_DRAM1_SLV_RAM_REGS_BASE;
                        }
                    }
                    else
                    {
                        /* In switch mode expecting both ports have same dscp priorities
                           If in case of undirected traffic dscp priorities are taken from
                           Port1
                        */
                        /*DRAM0*/
                        baseAddr += CSL_ICSS_G_DRAM0_SLV_RAM_REGS_BASE;
                        if (dmaPkt->txPortNum == ENET_MAC_PORT_2)
                        {
                            /*DRAM1*/
                            baseAddr += CSL_ICSS_G_DRAM1_SLV_RAM_REGS_BASE;
                        }
                    }
                    baseAddr = baseAddr + DSCP_ENABLE_DISABLE_STATUS;
                    dscpIPv4En = CSL_REG8_RD(baseAddr);
                    if (dscpIPv4En == 1)
                    {
                        dmaPkt->txPktTc = 0;
                        Enet_assert(sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].segmentFilledLen > 13);
                        /*check for Vlan tagging*/
                        if ((sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].bufPtr[12] == 0x81U) &&
                                (sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].bufPtr[13] == 0x00U))
                        {
                            Enet_assert(sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].segmentFilledLen > 17);
                            /*Check for ipv4 ethertype*/
                            if ((sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].bufPtr[16] == 0x08U) &&
                                    (sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].bufPtr[17] == 0x00U))
                            {
                                Enet_assert(sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].segmentFilledLen > 19);
                                /* upper 6 bits has the dscp index */
                                tosIndex = ((sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].bufPtr[19] >> 2) & 0x3f);

                                for (i = 0; i < ENET_PRI_NUM; i++)
                                {
                                    tosVal = CSL_REG8_RD(baseAddr + (i+1));
                                    if(tosVal == tosIndex)
                                    {
                                        break;
                                    }
                                }
                                dmaPkt->txPktTc = i;
                            }
                        }
                        else
                        {
                            dmaPkt->txPktTc = 0;
                            if ((sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].bufPtr[12] == 0x08U) &&
                                    (sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].bufPtr[13] == 0x00U))
                            {
                                /* upper 6 bits has the dscp index */
                                Enet_assert(sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].segmentFilledLen > 15);
                                tosIndex = ((sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].bufPtr[15] >> 2) & 0x3f);
                                for (i = 0; i < ENET_PRI_NUM; i++)
                                {
                                    tosVal = CSL_REG8_RD(baseAddr + (i+1));
                                    if(tosVal == tosIndex)
                                    {
                                        break;
                                    }
                                }
                                dmaPkt->txPktTc = i;
                            }
                        }
                    }
                }
#endif
                if (dmaPkt->txPortNum != ENET_MAC_PORT_INV)
                {
                    dstTag = CPSW_ALE_MACPORT_TO_ALEPORT(dmaPkt->txPortNum);
                    /* Set txPortNum back to invalid to reset directed packet configuration */
                    dmaPkt->txPortNum = ENET_MAC_PORT_INV;
                }
                else
                {
                    dstTag = 0U;
                }

                if (dmaPkt->txPktTc != ENET_TRAFFIC_CLASS_INV)
                {
                    dstTag |= dmaPkt->txPktTc << 8U;
                }

                CSL_udmapCppi5SetDstTag(pHostDesc, dstTag);
            }

            /* Check that the buffer pointer and length are correct */
            retVal = EnetUdma_dmaDescCheck(pDmaDesc, transferDir);

            if (UDMA_SOK == retVal)
            {
                retVal = EnetUdma_ringEnqueue(hUdmaRing,
                                             pDmaDesc,
                                             disableCacheOpsFlag,
                                             transferDir
#if (UDMA_SOC_CFG_PROXY_PRESENT == 1)
                                             ,
                                             hUdmaProxy
#endif
                                             );
            }

            /* Dequeue from toHwQ only if the packet was actually queued to the fqRing */
            if (UDMA_SOK == retVal)
            {
                dmaPkt = (EnetDma_Pkt *)EnetQueue_deq(pToHwQueue);
                submitCnt++;
            }
            else
            {
                // TODO - based on error we should add back dmadesc and pktInfo to orig Queues
                EnetDma_checkPktState(&pDmaDesc->dmaPkt->pktState,
                                        ENET_PKTSTATE_MODULE_DRIVER,
                                        (uint32_t)ENET_PKTSTATE_DMA_WITH_HW,
                                        (uint32_t)ENET_PKTSTATE_DMA_NOT_WITH_HW);
                break;
            }
        }
        else
        {
            /* Return dequeued packet buffer as couldn't get free dma desc to attach
             *  packet to */
            EnetQueue_enqHead(pToHwQueue, &dmaPkt->node);
            retVal = UDMA_EALLOC;
            break;
        }
    }

    /* Wb the Ring memory cache before commiting to the ring in case of exposed ring mode */
    if ((UDMA_SOK == retVal) && (isExposedRing == true))
    {
        if ((ringWrIdx + submitCnt) > ringMemEleCnt)
        {
            currRingMemPtr = ringMemPtr + ringWrIdx;
            EnetOsal_cacheWb(currRingMemPtr,
                             (ringMemEleCnt - ringWrIdx) * ENET_UDMA_RING_MEM_SIZE);
            EnetOsal_cacheWb(ringMemPtr,
                             (submitCnt - ringMemEleCnt + ringWrIdx) * ENET_UDMA_RING_MEM_SIZE);
        }
        else
        {
            EnetOsal_cacheWb(currRingMemPtr,
                             submitCnt * ENET_UDMA_RING_MEM_SIZE);
        }

        /* Set Ring door bell register with count of number of descriptors queued */
        Udma_ringSetDoorBell(hUdmaRing, submitCnt);
    }

    /* If fqRing ran out of space is not an error, packets will be re-submitted from toHwQ */
    if (UDMA_ETIMEOUT == retVal)
    {
        retVal = UDMA_SOK;
    }

    return retVal;
}

int32_t EnetUdma_submitSingleRxPkt(EnetPer_Handle hPer,
                                   Udma_RingHandle hUdmaRing,
                                   EnetDma_Pkt *pPkt,
                                   EnetUdma_DmaDescQ *pDmaDescQ,
                                   bool disableCacheOpsFlag
#if (UDMA_SOC_CFG_PROXY_PRESENT == 1)
                                   ,
                                   Udma_ProxyHandle hUdmaProxy
#endif
                           )
{
    int32_t retVal = UDMA_SOK;
    EnetUdma_DmaDesc *pDmaDesc;
    int32_t submitCnt = 0, i;
    bool isExposedRing;
    uint64_t *ringMemPtr = NULL, *currRingMemPtr = NULL;
    uint32_t ringWrIdx = 0U, ringMemEleCnt = 0U;
    uint32_t totalPacketFilledLen = 0;
    EnetUdma_SGListEntry *sgList;

    isExposedRing = (Udma_ringGetMode(hUdmaRing) == TISCI_MSG_VALUE_RM_RING_MODE_RING);
#if defined(SOC_AM64X) || defined(SOC_AM243X) || defined(SOC_AM62AX)
    isExposedRing = 0U;
#endif
    if (isExposedRing == true)
    {
        ringMemPtr = (uint64_t *)Udma_ringGetMemPtr(hUdmaRing);
        ringWrIdx = Udma_ringGetWrIdx(hUdmaRing);
        ringMemEleCnt = Udma_ringGetElementCnt(hUdmaRing);
    }

    /* Enqueue packets until fqRing is full */
    if (NULL != pPkt)
    {
        /* Enqueue desc to fqRing */
        pDmaDesc = EnetUdma_dmaDescDeque(pDmaDescQ);

        if (NULL != pDmaDesc)
        {
            EnetUdma_CpswHpdDesc *pHpdDesc = (EnetUdma_CpswHpdDesc*) pDmaDesc;
            CSL_UdmapCppi5HMPD *pHostDesc = &pHpdDesc->hostDesc;

            pDmaDesc->dmaPkt = pPkt;
            sgList = pPkt->sgList.list;
            EnetDma_checkPktState(&pDmaDesc->dmaPkt->pktState,
                                  ENET_PKTSTATE_MODULE_DRIVER,
                                  (uint32_t) ENET_PKTSTATE_DMA_NOT_WITH_HW,
                                  (uint32_t) ENET_PKTSTATE_DMA_WITH_HW);

            CSL_UdmapCppi5HMPD *pHDesc = pHostDesc;
            CSL_UdmapCppi5HMPD *pHDescPrev = NULL;
            for (i = 0; i < pPkt->sgList.numScatterSegments; i++)
            {
                if (ENET_UDMA_CPSW_IS_HBD_IDX(i))
                {
                    pHDesc = &pDmaDesc->hostBufDesc[i - ENET_UDMA_CPSW_HOSTBUFDESC_INDEX].desc;
                    /* Link the Descs */
                    CSL_udmapCppi5LinkDesc(pHDescPrev, (uint64_t) pHDesc);
                }
                CSL_udmapCppi5SetBufferAddr(pHDesc, (uint64_t) sgList[i].bufPtr);
                CSL_udmapCppi5SetBufferLen(pHDesc, sgList[i].segmentAllocLen);
                CSL_udmapCppi5SetOrgBufferAddr(pHDesc, (uint64_t) sgList[i].bufPtr);
                CSL_udmapCppi5SetOrgBufferLen(pHDesc, sgList[i].segmentAllocLen);
                totalPacketFilledLen += sgList[i].segmentFilledLen;
                pHDescPrev = pHDesc;
            }
            /* Link the last segment's Desc to NULL */
            CSL_udmapCppi5LinkDesc(pHDesc, 0U);
            CSL_udmapCppi5HostSetPktLen(pHostDesc, totalPacketFilledLen);

            /* Check that the buffer pointer and length are correct */
            retVal = EnetUdma_dmaDescCheck(pDmaDesc, ENET_UDMA_DIR_RX);

            if (UDMA_SOK == retVal)
            {
                retVal = EnetUdma_ringEnqueue(hUdmaRing,
                                             pDmaDesc,
                                             disableCacheOpsFlag,
                                             ENET_UDMA_DIR_RX
#if (UDMA_SOC_CFG_PROXY_PRESENT == 1)
                                             ,
                                             hUdmaProxy
#endif
                                             );
            }

            /* Dequeue from toHwQ only if the packet was actually queued to the fqRing */
            if (UDMA_SOK == retVal)
            {
                submitCnt++;
            }
            else
            {
                // TODO - based on error we should add back dmadesc and pktInfo to orig Queues
                EnetDma_checkPktState(&pDmaDesc->dmaPkt->pktState,
                                        ENET_PKTSTATE_MODULE_DRIVER,
                                        (uint32_t)ENET_PKTSTATE_DMA_WITH_HW,
                                        (uint32_t)ENET_PKTSTATE_DMA_NOT_WITH_HW);
            }
        }
        else
        {
            /* Return dequeued packet buffer as couldn't get free dma desc to attach
             *  packet to */
            retVal = UDMA_EALLOC;
        }
    }

    /* Wb the Ring memory cache before commiting to the ring in case of exposed ring mode */
    if ((UDMA_SOK == retVal) && (isExposedRing == true))
    {
        if ((ringWrIdx + submitCnt) > ringMemEleCnt)
        {
            currRingMemPtr = ringMemPtr + ringWrIdx;
            EnetOsal_cacheWb(currRingMemPtr,
                             (ringMemEleCnt - ringWrIdx) * ENET_UDMA_RING_MEM_SIZE);
            EnetOsal_cacheWb(ringMemPtr,
                             (submitCnt - ringMemEleCnt + ringWrIdx) * ENET_UDMA_RING_MEM_SIZE);
        }
        else
        {
            EnetOsal_cacheWb(currRingMemPtr,
                             submitCnt * ENET_UDMA_RING_MEM_SIZE);
        }

        /* Set Ring door bell register with count of number of descriptors queued */
        Udma_ringSetDoorBell(hUdmaRing, submitCnt);
    }

    /* If fqRing ran out of space is not an error, packets will be re-submitted from toHwQ */
    if (UDMA_ETIMEOUT == retVal)
    {
        retVal = UDMA_SOK;
    }

    return retVal;
}

int32_t EnetUdma_submitSingleTxPkt(EnetPer_Handle hPer,
                            Udma_RingHandle hUdmaRing,
                           EnetDma_Pkt *pPkt,
                           EnetUdma_DmaDescQ *pDmaDescQ,
                           bool disableCacheOpsFlag
#if (UDMA_SOC_CFG_PROXY_PRESENT == 1)
                           ,
                           Udma_ProxyHandle hUdmaProxy
#endif
                           )
{
    int32_t retVal = UDMA_SOK;
    EnetUdma_DmaDesc *pDmaDesc;
    uint32_t dstTag;
    EnetUdma_CppiRxControl *cppiRxCntr;
    uint32_t *iccsgTxTsId, *tsInfo;
    int32_t submitCnt = 0;
    bool isExposedRing;
    uint64_t *ringMemPtr = NULL, *currRingMemPtr = NULL;
    uint32_t ringWrIdx = 0U, ringMemEleCnt = 0U;
    uint32_t i;
#if defined(SOC_AM64X) || defined(SOC_AM243X)
    uint32_t tosIndex;
    uint8_t dscpIPv4En, tosVal;
#endif
    uint32_t totalPacketFilledLen = 0;
    EnetUdma_SGListEntry *sgList;

    isExposedRing = (Udma_ringGetMode(hUdmaRing) == TISCI_MSG_VALUE_RM_RING_MODE_RING);
#if defined(SOC_AM64X) || defined(SOC_AM243X) || defined(SOC_AM62AX)
    isExposedRing = 0U;
#endif
    if (isExposedRing == true)
    {
        ringMemPtr = (uint64_t *)Udma_ringGetMemPtr(hUdmaRing);
        ringWrIdx = Udma_ringGetWrIdx(hUdmaRing);
        ringMemEleCnt = Udma_ringGetElementCnt(hUdmaRing);
    }

    /* Enqueue sintgle packet */
    if (NULL != pPkt)
    {
        /* Enqueue desc to fqRing */
        pDmaDesc = EnetUdma_dmaDescDeque(pDmaDescQ);

        if (NULL != pDmaDesc)
        {
            EnetUdma_CpswHpdDesc *pHpdDesc = (EnetUdma_CpswHpdDesc*) pDmaDesc;
            CSL_UdmapCppi5HMPD *pHostDesc = &pHpdDesc->hostDesc;

            pDmaDesc->dmaPkt = pPkt;
            sgList = pPkt->sgList.list;
            EnetDma_checkPktState(&pDmaDesc->dmaPkt->pktState,
                                  ENET_PKTSTATE_MODULE_DRIVER,
                                  (uint32_t) ENET_PKTSTATE_DMA_NOT_WITH_HW,
                                  (uint32_t) ENET_PKTSTATE_DMA_WITH_HW);

            CSL_UdmapCppi5HMPD *pHDesc = pHostDesc;
            CSL_UdmapCppi5HMPD *pHDescPrev = NULL;
            for (i = 0; i < pPkt->sgList.numScatterSegments; i++)
            {
                if (ENET_UDMA_CPSW_IS_HBD_IDX(i))
                {
                    pHDesc = &pDmaDesc->hostBufDesc[i - ENET_UDMA_CPSW_HOSTBUFDESC_INDEX].desc;
                    /* Link the Descs */
                    CSL_udmapCppi5LinkDesc(pHDescPrev, (uint64_t) pHDesc);
                }
                CSL_udmapCppi5SetBufferAddr(pHDesc, (uint64_t) sgList[i].bufPtr);
                CSL_udmapCppi5SetBufferLen(pHDesc, sgList[i].segmentFilledLen);
                CSL_udmapCppi5SetOrgBufferAddr(pHDesc, (uint64_t) sgList[i].bufPtr);
                CSL_udmapCppi5SetOrgBufferLen(pHDesc, sgList[i].segmentAllocLen);
                totalPacketFilledLen += sgList[i].segmentFilledLen;
                pHDescPrev = pHDesc;
            }
            /* Link the last segment's Desc to NULL */
            CSL_udmapCppi5LinkDesc(pHDesc, 0U);

            CSL_udmapCppi5HostSetPktLen(pHostDesc, totalPacketFilledLen);

#if ENET_CFG_IS_ON(DEV_ERROR)
            /* For TX all defaults are set in EnetUdma_buffDescInit function. We just confirm those
             * are not changed here */
            Enet_assert(CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST == CSL_udmapCppi5GetDescType(pHostDesc));
            Enet_assert(0U == CSL_udmapCppi5GetPsDataLoc(pHostDesc));
            Enet_assert(true == CSL_udmapCppi5IsEpiDataPresent(pHostDesc));
            Enet_assert(ENET_UDMA_PROTOCOL_SPECIFIC_INFO_BLOCK_SIZE ==
                            CSL_udmapCppi5GetPsDataLen(pHostDesc));
#endif
            cppiRxCntr = (EnetUdma_CppiRxControl *)pHpdDesc->psInfo;

            if (Enet_isIcssFamily(hPer->enetType))
            {
                cppiRxCntr->chkSumInfo = 0U;
                tsInfo = (uint32_t *)&pHpdDesc->extendedPktInfo[4U];
            }
            else
            {
                cppiRxCntr->chkSumInfo = pPkt->chkSumInfo;
                tsInfo = (uint32_t *)&cppiRxCntr->tsInfo;
            }

            if (pPkt->tsInfo.enableHostTxTs == true)
            {
                ENETUDMA_CPPIPSI_SET_TSEN(*tsInfo, 1U);

                /* Below fields are only valid for CPSW. In ICSSG, whole tsInfo word is
                 * used for enabling timestamp.
                 * This is don't care/reserved word for ICSSG so we set without any check */
                ENETUDMA_CPPIPSI_SET_DOMAIN(*tsInfo,
                                           pPkt->tsInfo.txPktDomain);
                ENETUDMA_CPPIPSI_SET_MSGTYPE(*tsInfo,
                                            pPkt->tsInfo.txPktMsgType);
                ENETUDMA_CPPIPSI_SET_SEQID(*tsInfo,
                                          pPkt->tsInfo.txPktSeqId);

                /* Set host Tx timestamp flag back to false. */
                pPkt->tsInfo.enableHostTxTs = false;

                /* For ICSSG psinfo word 0 is used for passing cookie to the firmwareiccsgTxTsId
                 * This is don't care/reserved word for CPSW so we set without any check */
                iccsgTxTsId = (uint32_t *)&pHpdDesc->extendedPktInfo[0U];
                *iccsgTxTsId = pPkt->txTsId;
            }
            else
            {
                ENETUDMA_CPPIPSI_SET_TSEN(*tsInfo, 0U);
            }

#if defined(SOC_AM64X) || defined(SOC_AM243X)
            if (Enet_isIcssFamily(hPer->enetType))
            {
                uintptr_t baseAddr = (uintptr_t)hPer->virtAddr;

                if (hPer->enetType == ENET_ICSSG_DUALMAC)
                {
                    if ((hPer->instId == 0) || (hPer->instId == 2))
                    {
                        baseAddr += CSL_ICSS_G_DRAM0_SLV_RAM_REGS_BASE;
                    }
                    else
                    {
                        /*DRAM1*/
                        baseAddr += CSL_ICSS_G_DRAM1_SLV_RAM_REGS_BASE;
                    }
                }
                else
                {
                    /* In switch mode expecting both ports have same dscp priorities
                       If in case of undirected traffic dscp priorities are taken from
                       Port1
                    */
                    /*DRAM0*/
                    baseAddr += CSL_ICSS_G_DRAM0_SLV_RAM_REGS_BASE;
                    if (pPkt->txPortNum == ENET_MAC_PORT_2)
                    {
                        /*DRAM1*/
                        baseAddr += CSL_ICSS_G_DRAM1_SLV_RAM_REGS_BASE;
                    }
                }
                baseAddr = baseAddr + DSCP_ENABLE_DISABLE_STATUS;
                dscpIPv4En = CSL_REG8_RD(baseAddr);
                if (dscpIPv4En == 1)
                {
                    pPkt->txPktTc = 0;
                    Enet_assert(sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].segmentFilledLen > 13);
                    /*check for Vlan tagging*/
                    if ((sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].bufPtr[12] == 0x81U) &&
                            (sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].bufPtr[13] == 0x00U))
                    {
                        Enet_assert(sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].segmentFilledLen > 17);
                        /*Check for ipv4 ethertype*/
                        if ((sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].bufPtr[16] == 0x08U) &&
                                (sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].bufPtr[17] == 0x00U))
                        {
                            Enet_assert(sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].segmentFilledLen > 19);
                            /* upper 6 bits has the dscp index */
                            tosIndex = ((sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].bufPtr[19] >> 2) & 0x3f);

                            for (i = 0; i < ENET_PRI_NUM; i++)
                            {
                                tosVal = CSL_REG8_RD(baseAddr + (i+1));
                                if(tosVal == tosIndex)
                                {
                                    break;
                                }
                            }
                            pPkt->txPktTc = i;
                        }
                    }
                    else
                    {
                        pPkt->txPktTc = 0;
                        if ((sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].bufPtr[12] == 0x08U) &&
                                (sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].bufPtr[13] == 0x00U))
                        {
                            /* upper 6 bits has the dscp index */
                            Enet_assert(sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].segmentFilledLen > 15);
                            tosIndex = ((sgList[ENET_UDMA_CPSW_HOSTPKTDESC_INDEX].bufPtr[15] >> 2) & 0x3f);
                            for (i = 0; i < ENET_PRI_NUM; i++)
                            {
                                tosVal = CSL_REG8_RD(baseAddr + (i+1));
                                if(tosVal == tosIndex)
                                {
                                    break;
                                }
                            }
                            pPkt->txPktTc = i;
                        }
                    }
                }
            }
#endif

            if (pPkt->txPortNum != ENET_MAC_PORT_INV)
            {
                dstTag = CPSW_ALE_MACPORT_TO_ALEPORT(pPkt->txPortNum);
                /* Set txPortNum back to invalid to reset directed packet configuration */
                pPkt->txPortNum = ENET_MAC_PORT_INV;
            }
            else
            {
                dstTag = 0U;
            }

            if (pPkt->txPktTc != ENET_TRAFFIC_CLASS_INV)
            {
                dstTag |= pPkt->txPktTc << 8U;
            }

            CSL_udmapCppi5SetDstTag(pHostDesc, dstTag);

            /* Check that the buffer pointer and length are correct */
            retVal = EnetUdma_dmaDescCheck(pDmaDesc, ENET_UDMA_DIR_TX);

            if (UDMA_SOK == retVal)
            {
                retVal = EnetUdma_ringEnqueue(hUdmaRing,
                                             pDmaDesc,
                                             disableCacheOpsFlag,
                                             ENET_UDMA_DIR_TX
#if (UDMA_SOC_CFG_PROXY_PRESENT == 1)
                                             ,
                                             hUdmaProxy
#endif
                                             );
            }

            /* Dequeue from toHwQ only if the packet was actually queued to the fqRing */
            if (UDMA_SOK == retVal)
            {
                submitCnt++;
            }
            else
            {
                // TODO - based on error we should add back dmadesc and pktInfo to orig Queues
                EnetDma_checkPktState(&pDmaDesc->dmaPkt->pktState,
                                        ENET_PKTSTATE_MODULE_DRIVER,
                                        (uint32_t)ENET_PKTSTATE_DMA_WITH_HW,
                                        (uint32_t)ENET_PKTSTATE_DMA_NOT_WITH_HW);
            }
        }
        else
        {
            /* Return dequeued packet buffer as couldn't get free dma desc to attach
             *  packet to */
            //TODO: surbhi handle this case
            retVal = UDMA_EALLOC;
        }
    }

    /* Wb the Ring memory cache before commiting to the ring in case of exposed ring mode */
    if ((UDMA_SOK == retVal) && (isExposedRing == true))
    {
        if ((ringWrIdx + submitCnt) > ringMemEleCnt)
        {
            currRingMemPtr = ringMemPtr + ringWrIdx;
            EnetOsal_cacheWb(currRingMemPtr,
                             (ringMemEleCnt - ringWrIdx) * ENET_UDMA_RING_MEM_SIZE);
            EnetOsal_cacheWb(ringMemPtr,
                             (submitCnt - ringMemEleCnt + ringWrIdx) * ENET_UDMA_RING_MEM_SIZE);
        }
        else
        {
            EnetOsal_cacheWb(currRingMemPtr,
                             submitCnt * ENET_UDMA_RING_MEM_SIZE);
        }

        /* Set Ring door bell register with count of number of descriptors queued */
        Udma_ringSetDoorBell(hUdmaRing, submitCnt);
    }

    /* If fqRing ran out of space is not an error, packets will be re-submitted from toHwQ */
    if (UDMA_ETIMEOUT == retVal)
    {
        retVal = UDMA_SOK;
    }

    return retVal;
}

int32_t EnetUdma_flushRxFlowRing(EnetDma_RxChHandle hRxFlow,
                                Udma_RingHandle hUdmaRing,
                                EnetDma_PktQ *pPktInfoQ)
{
    int32_t retVal = UDMA_SOK;
    uint64_t pDesc = 0;
    EnetUdma_DmaDesc *pDmaDesc;

    EnetQueue_initQ(pPktInfoQ);

    /* Dequeue the packets still sitting in the cqRing */
    retVal = Udma_ringDequeueRaw(hUdmaRing, &pDesc);
    while ((UDMA_SOK == retVal) && (0ULL != pDesc))
    {
        pDmaDesc = (EnetUdma_DmaDesc *)EnetUtils_physToVirt(pDesc, NULL);
        EnetDma_checkPktState(&pDmaDesc->dmaPkt->pktState,
                                ENET_PKTSTATE_MODULE_DRIVER,
                                (uint32_t)ENET_PKTSTATE_DMA_WITH_HW,
                                (uint32_t)ENET_PKTSTATE_DMA_NOT_WITH_HW);
        EnetQueue_enq(pPktInfoQ, &pDmaDesc->dmaPkt->node);

        hRxFlow->rxFlowPrms.dmaDescFreeFxn(hRxFlow->rxFlowPrms.cbArg,
                                           pDmaDesc);

        retVal = Udma_ringDequeueRaw(hUdmaRing, &pDesc);
    }

#if (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
    /* Dequeue the packets still sitting in the cqRing */
    retVal = Udma_ringFlushRaw(hUdmaRing, &pDesc);
    while ((UDMA_SOK == retVal) && (0ULL != pDesc))
    {
        pDmaDesc = (EnetUdma_DmaDesc *)EnetUtils_physToVirt(pDesc, NULL);
        EnetDma_checkPktState(&pDmaDesc->dmaPkt->pktState,
                                ENET_PKTSTATE_MODULE_DRIVER,
                                (uint32_t)ENET_PKTSTATE_DMA_WITH_HW,
                                (uint32_t)ENET_PKTSTATE_DMA_NOT_WITH_HW);
        EnetQueue_enq(pPktInfoQ, &pDmaDesc->dmaPkt->node);

        hRxFlow->rxFlowPrms.dmaDescFreeFxn(hRxFlow->rxFlowPrms.cbArg,
                                           pDmaDesc);

        retVal = Udma_ringFlushRaw(hUdmaRing, &pDesc);
    }
#endif

    /* It's ok if there are no more descs in cqRing */
    if (UDMA_ETIMEOUT == retVal)
    {
        retVal = UDMA_SOK;
    }

    return retVal;
}

int32_t EnetUdma_flushTxChRing(EnetDma_TxChHandle hTxCh,
                              Udma_RingHandle hUdmaRing,
                              EnetDma_PktQ *pFqPktInfoQ)
{
    int32_t retVal = UDMA_SOK;
    uint64_t pDesc = 0;
    EnetUdma_DmaDesc *pDmaDesc;

    EnetQueue_initQ(pFqPktInfoQ);

    /* Dequeue the packets still sitting in the ring */
    retVal = Udma_ringDequeueRaw(hUdmaRing, &pDesc);
    while ((UDMA_SOK == retVal) && (0ULL != pDesc))
    {
        pDmaDesc = (EnetUdma_DmaDesc *)EnetUtils_physToVirt(pDesc, NULL);
        EnetDma_checkPktState(&pDmaDesc->dmaPkt->pktState,
                                ENET_PKTSTATE_MODULE_DRIVER,
                                (uint32_t)ENET_PKTSTATE_DMA_WITH_HW,
                                (uint32_t)ENET_PKTSTATE_DMA_NOT_WITH_HW);
        EnetQueue_enq(pFqPktInfoQ, &pDmaDesc->dmaPkt->node);

        hTxCh->txChPrms.dmaDescFreeFxn(hTxCh->txChPrms.cbArg,
                                       pDmaDesc);

        retVal = Udma_ringDequeueRaw(hUdmaRing, &pDesc);
    }

#if (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
    /* Dequeue the packets still sitting in the ring */
    retVal = Udma_ringFlushRaw(hUdmaRing, &pDesc);
    while ((UDMA_SOK == retVal) && (0ULL != pDesc))
    {
        pDmaDesc = (EnetUdma_DmaDesc *)EnetUtils_physToVirt(pDesc, NULL);
        EnetDma_checkPktState(&pDmaDesc->dmaPkt->pktState,
                                ENET_PKTSTATE_MODULE_DRIVER,
                                (uint32_t)ENET_PKTSTATE_DMA_WITH_HW,
                                (uint32_t)ENET_PKTSTATE_DMA_NOT_WITH_HW);
        EnetQueue_enq(pFqPktInfoQ, &pDmaDesc->dmaPkt->node);

        hTxCh->txChPrms.dmaDescFreeFxn(hTxCh->txChPrms.cbArg,
                                       pDmaDesc);

        retVal = Udma_ringFlushRaw(hUdmaRing, &pDesc);
    }
#endif

    /* It's ok if there are no more descs in ring */
    if (UDMA_ETIMEOUT == retVal)
    {
        retVal = UDMA_SOK;
    }

    return retVal;
}

int32_t EnetUdma_freeRing(Udma_RingHandle hUdmaRing,
                        uint32_t numPkts,
                        EnetUdma_FreeRingMemFxn ringMemFreeFxn,
                        void *cbArg)
{
    uint8_t *ringMemPtr;
    int32_t retVal = UDMA_SOK;

    ringMemPtr = (uint8_t *)Udma_ringGetMemPtr(hUdmaRing);
    retVal     = Udma_ringFree(hUdmaRing);

    if (NULL != ringMemFreeFxn)
    {
        ringMemFreeFxn(cbArg, ringMemPtr, numPkts);
    }

    return retVal;
}

int32_t EnetUdma_allocRing(Udma_DrvHandle hUdmaDrv,
                          Udma_RingHandle hUdmaRing,
                          Udma_RingPrms *pRingPrms,
                          EnetUdma_ringAllocInfo *pRingAllocInfo)
{
    int32_t retVal = UDMA_SOK;
    bool freeMem   = false;

    if ((pRingAllocInfo != NULL ) && (pRingAllocInfo->allocRingMem))
    {
        Enet_assert(NULL != pRingAllocInfo);
        Enet_assert(NULL != pRingAllocInfo->ringMemAllocFxn);
        Enet_assert(NULL != pRingAllocInfo->ringMemFreeFxn);

        pRingPrms->ringMem = pRingAllocInfo->ringMemAllocFxn(pRingAllocInfo->cbArg,
                                             pRingPrms->elemCnt,
                                             UDMA_CACHELINE_ALIGNMENT);
        if (pRingPrms->ringMem == NULL)
        {
            ENETTRACE_ERR("[Enet UDMA Error] RX ring memory allocation failed !!\n");
            retVal = UDMA_EALLOC;
        }
        else if (!ENET_UTILS_IS_ALIGNED(pRingPrms->ringMem, UDMA_CACHELINE_ALIGNMENT))
        {
            ENETTRACE_ERR("[Enet UDMA] TX fq ring memory not aligned!!\n");
            freeMem = true;
            retVal  = UDMA_EALLOC;
        }
        else
        {
        }
    }
    else
    {
        /* ring memory should not be NULL if allocRingMem flag is false */
        Enet_assert (NULL != pRingPrms->ringMem);
    }


    if ((UDMA_SOK == retVal) && (pRingAllocInfo != NULL))
    {
        /* Clear and invalidate ring memory */
        memset(pRingPrms->ringMem, 0, (pRingPrms->elemCnt * ENET_UDMA_RING_MEM_SIZE));
        if (FALSE == Enet_isCacheCoherent())
        {
            EnetOsal_cacheWbInv(pRingPrms->ringMem, (pRingPrms->elemCnt * ENET_UDMA_RING_MEM_SIZE));
        }
#if (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
        pRingPrms->mappedChNum   = pRingAllocInfo->mappedChNum;
        if (ENET_UDMA_DIR_TX == pRingAllocInfo->transferDir)
        {
            if (Enet_isCpswFamily(pRingAllocInfo->enetType))
            {
                    pRingPrms->mappedRingGrp   = UDMA_MAPPED_TX_GROUP_CPSW;
            }
            else
            {
                if ((0U == pRingAllocInfo->instId) || (1U == pRingAllocInfo->instId))
                {
                    pRingPrms->mappedRingGrp   = UDMA_MAPPED_TX_GROUP_ICSSG_0;
                }
                else if ((2U == pRingAllocInfo->instId) || (3U == pRingAllocInfo->instId))
                {
                    pRingPrms->mappedRingGrp   = UDMA_MAPPED_TX_GROUP_ICSSG_1;
                }
                else
                {
                    Enet_assert(false);
                }
            }
        }
        else
        {
            if (Enet_isCpswFamily(pRingAllocInfo->enetType))
            {
                    pRingPrms->mappedRingGrp   = UDMA_MAPPED_RX_GROUP_CPSW;
            }
            else if (pRingAllocInfo->enetType == ENET_ICSSG_SWITCH)
            {
                if (0U == pRingAllocInfo->instId)
                {
                    pRingPrms->mappedRingGrp   = UDMA_MAPPED_RX_GROUP_ICSSG_0;
                }
                else if (1U == pRingAllocInfo->instId)
                {
                    pRingPrms->mappedRingGrp   = UDMA_MAPPED_RX_GROUP_ICSSG_1;
                }
                else
                {
                    Enet_assert(false);
                }
            }
            else if (pRingAllocInfo->enetType == ENET_ICSSG_DUALMAC)
            {
                if ((0U == pRingAllocInfo->instId) || (1U == pRingAllocInfo->instId))
                {
                    pRingPrms->mappedRingGrp   = UDMA_MAPPED_RX_GROUP_ICSSG_0;
                }
                else if ((2U == pRingAllocInfo->instId) || (3U == pRingAllocInfo->instId))
                {
                    pRingPrms->mappedRingGrp   = UDMA_MAPPED_RX_GROUP_ICSSG_1;
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
        }
#endif
        /* need to allocate rxfree/rxcompletion ring pair */
        retVal = Udma_ringAlloc(hUdmaDrv,
                                hUdmaRing,
                                pRingAllocInfo->ringNum,
                                pRingPrms);
        if (UDMA_SOK != retVal)
        {
            freeMem = true;
        }
    }

    if ((pRingAllocInfo != NULL) && (pRingAllocInfo->allocRingMem) && (freeMem))
    {
        pRingAllocInfo->ringMemFreeFxn(pRingAllocInfo->cbArg, pRingPrms->ringMem, pRingPrms->elemCnt);
        pRingPrms->ringMem = NULL;
    }

    return retVal;
}

void EnetUdma_getTxChUdmaInfo(EnetUdma_TxChObj *pTxCh,
                             EnetUdma_udmaInfo *pUdmaInfo)
{
    pUdmaInfo->hUdmaDrv     = pTxCh->hUdmaDrv;
    pUdmaInfo->hUdmaCh      = pTxCh->hUdmaCh;
    pUdmaInfo->hUdmaEvt     = pTxCh->hUdmaEvt;
#if (UDMA_SOC_CFG_RING_MON_PRESENT == 1)
    pUdmaInfo->hUdmaRingMon = pTxCh->hUdmaRingMon;
#endif
    pUdmaInfo->useGlobalEvt = pTxCh->useGlobalEvt;
}

void EnetUdma_getRxFlowUdmaInfo(EnetUdma_RxFlowObj *pRxFlow,
                               EnetUdma_udmaInfo *pUdmaInfo)
{
    pUdmaInfo->hUdmaDrv     = pRxFlow->hUdmaDrv;
    pUdmaInfo->hUdmaEvt     = pRxFlow->hUdmaEvt;
#if (UDMA_SOC_CFG_RING_MON_PRESENT == 1)
    pUdmaInfo->hUdmaRingMon = pRxFlow->hUdmaRingMon;
#endif
    pUdmaInfo->useGlobalEvt = pRxFlow->useGlobalEvt;
}
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
static int32_t EnetUdma_ringProxyEnqueue(Udma_RingHandle hUdmaRing,
                                        Udma_ProxyHandle hUdmaProxy,
                                        uint64_t phyDescMem)
{
    int32_t retVal = UDMA_EFAIL;
    uint32_t ringHwOcc;
    uintptr_t key;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if ((NULL == hUdmaProxy) ||
        (NULL == hUdmaRing))
    {
        ENETTRACE_ERR_IF((NULL == hUdmaProxy), "[Enet UDMA] hUdmaProxy is NULL!!\n");
        ENETTRACE_ERR_IF((NULL == hUdmaRing), "[Enet UDMA] hUdmaRing is NULL!!\n");
        retVal = UDMA_EBADARGS;
    }
    else
#endif
    {
        key = EnetOsal_disableAllIntr();

        /* Get ring occupancy. We should write to FIFO (through proxy) only
         * when there is room available. Otherwise ring will overflow */
        ringHwOcc = CSL_ringaccGetRingHwOcc(&hUdmaRing->drvHandle->raRegs,
                                            Udma_ringGetNum(hUdmaRing));
        if (ringHwOcc >= hUdmaRing->cfg.elCnt)
        {
            /* Ring full */
            retVal = UDMA_EFAIL;
        }
        else
        {
            Udma_proxyQueue(hUdmaProxy, phyDescMem);
            retVal = UDMA_SOK;
        }
        EnetOsal_restoreAllIntr(key);
    }

    return(retVal);
}
#endif

EnetDma_RxChHandle EnetUdma_openRxRsvdFlow(EnetDma_Handle hDma,
                                           EnetUdma_RsvdRxFlowPrms *pRxFlowPrms)
{
    int32_t retVal = UDMA_SOK;
    Udma_FlowPrms flowPrms;
    Udma_RingPrms ringPrms;
    Udma_RingHandle ringHandle;
    EnetUdma_RxFlowObj *pRxFlow;
    EnetUdma_ringAllocInfo ringAllocInfo;
    bool allocFlowObj = false, allocCqRing = false;
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    bool allocFqRing  = false;
#endif
    bool flowAttachFlag = false;
    uintptr_t intrKey;
    uint32_t flowStart;

    intrKey = EnetOsal_disableAllIntr();

    /* Set to NULL so if error condition we return NULL */
    pRxFlow = NULL;

    /* Error check */
    retVal = UDMA_EBADARGS;
    if (NULL != pRxFlowPrms)
    {
        if (NULL != pRxFlowPrms->hUdmaDrv)
        {
            retVal = UDMA_SOK;
        }
        else
        {
            ENETTRACE_ERR("[Enet UDMA Error] pRxFlowPrms has null UDMA handle!!\n");
        }
    }
    else
    {
        ENETTRACE_ERR("[Enet UDMA Error] pRxFlowPrms NULL !!\n");
    }

    if (UDMA_SOK == retVal)
    {
        pRxFlow = EnetUdma_memMgrAllocRxFlowObj();

        if (pRxFlow == NULL)
        {
            ENETTRACE_ERR("[Enet UDMA Error] Rx flow Memory alloc object failed !!\n");
            retVal = UDMA_EALLOC;
        }
        else
        {
            allocFlowObj = true;
            memset(pRxFlow, 0U, sizeof(*pRxFlow));
            /* Save Flow config */
            pRxFlow->rxFlowPrms.startIdx = pRxFlowPrms->startIdx;
            pRxFlow->rxFlowPrms.flowIdx  = pRxFlowPrms->flowIdx;
            pRxFlow->rxFlowPrms.hUdmaDrv = pRxFlowPrms->hUdmaDrv;

            /* Initialize local variables to be used in reminder of this function  */
            pRxFlow->hUdmaDrv  = pRxFlowPrms->hUdmaDrv;
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
            pRxFlow->fqRing    = &pRxFlow->rxFlowMemObj.fqRingObj;
#endif
            pRxFlow->hUdmaFlow = &pRxFlow->rxFlowMemObj.flowUdmaObj;
            pRxFlow->cqRing    = &pRxFlow->rxFlowMemObj.cqRingObj;

        }
    }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    if (UDMA_SOK == retVal)
    {
        UdmaRingPrms_init(&ringPrms);
        ringPrms.elemCnt = ENET_ARRAYSIZE(gRsvdFlowRingMemFQ);
        ringPrms.mode    = TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE;
        ringPrms.ringMem = &gRsvdFlowRingMemFQ[0U][0U];

        ringAllocInfo.allocRingMem = false;
        ringAllocInfo.ringNum      = UDMA_RING_ANY;

        retVal = EnetUdma_allocRing(pRxFlow->hUdmaDrv,
                                   pRxFlow->fqRing,
                                   &ringPrms,
                                   &ringAllocInfo);
        ENETTRACE_ERR_IF((retVal != UDMA_SOK),
                           "[Enet UDMA Error] Fq ring allocation failed !!: %d\n", retVal);
        if (UDMA_SOK == retVal)
        {
            allocFqRing = true;
        }
    }
#endif

    if (UDMA_SOK == retVal)
    {
        UdmaRingPrms_init(&ringPrms);
        ringPrms.elemCnt = ENET_ARRAYSIZE(gRsvdFlowRingMemCQ);
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        ringPrms.mode    = TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE;
#elif (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
        ringPrms.mode    = TISCI_MSG_VALUE_RM_RING_MODE_RING;
#endif
        ringPrms.ringMem = &gRsvdFlowRingMemCQ[0U][0U];

        ringAllocInfo.allocRingMem = false;
        ringAllocInfo.ringNum      = UDMA_RING_ANY;
        ringAllocInfo.enetType     = hDma->enetType;
        ringAllocInfo.instId       = hDma->instId;
        ringAllocInfo.mappedChNum  = Udma_chGetNum(&hDma->rxChObj[pRxFlowPrms->chIdx].udmaChObj);;
        ringAllocInfo.transferDir  = ENET_UDMA_DIR_RX;

        retVal = EnetUdma_allocRing(pRxFlow->hUdmaDrv,
                                   pRxFlow->cqRing,
                                   &ringPrms,
                                   &ringAllocInfo);
        ENETTRACE_ERR_IF((retVal != UDMA_SOK),
                           "[Enet UDMA Error] Cq ring allocation failed !!: %d\n", retVal);
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
        flowPrms.psInfoPresent = TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_PSINFO_PRESENT;

        flowPrms.defaultRxCQ = Udma_ringGetNum(pRxFlow->cqRing);
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        ringHandle = pRxFlow->fqRing;
#elif (UDMA_SOC_CFG_LCDMA_PRESENT == 1)
        ringHandle = pRxFlow->cqRing;
#endif
        flowPrms.fdq0Sz0Qnum = Udma_ringGetNum(ringHandle);
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
        flowAllocMappedPrms.mappedChNum = pRxFlowPrms->startIdx;

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

        ENETTRACE_ERR_IF((retVal != UDMA_SOK), "[Enet UDMA Error] Flow attach failed!!: %d\n", retVal);
        if (UDMA_SOK == retVal)
        {
            flowAttachFlag = true;
            retVal         = Udma_flowConfig(pRxFlow->hUdmaFlow,
                                             0U,
                                             &flowPrms);
        }
    }

    /* Save flow config */
    if (UDMA_SOK == retVal)
    {
        pRxFlow->initFlag    = true;
        pRxFlow->evtInitFlag = false;
    }
    else
    {
        /* Error. Free-up resources if allocated */
#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        if (allocFqRing)
        {
            retVal = EnetUdma_freeRing(pRxFlow->fqRing, 0U, NULL, NULL);
            Enet_assert(UDMA_SOK == retVal);
        }
#endif

        if (allocCqRing)
        {
            retVal = EnetUdma_freeRing(pRxFlow->cqRing, 0U, NULL, NULL);
            Enet_assert(UDMA_SOK == retVal);
        }

        if (flowAttachFlag)
        {
            retVal = Udma_flowDetach(pRxFlow->hUdmaFlow);
            Enet_assert(UDMA_SOK == retVal);
        }

        /* Free the rxFlowObj last */
        if (allocFlowObj)
        {
            EnetUdma_memMgrFreeRxFlowObj(pRxFlow);
        }

        /* As flow open is failed return NULL */
        pRxFlow = NULL;
    }

    EnetOsal_restoreAllIntr(intrKey);

    return pRxFlow;
}

int32_t EnetUdma_closeRxRsvdFlow(EnetDma_RxChHandle hRxFlow)
{
    int32_t retVal = UDMA_SOK;
    uintptr_t intrKey;

#if ENET_CFG_IS_ON(DEV_ERROR)
    if (NULL == hRxFlow)
    {
        ENETTRACE_ERR("[Enet UDMA Error] Input params can't be NULL!!\n");
        Enet_assert(false);
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
            Enet_assert(hRxFlow->evtInitFlag == false);

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
            retVal = EnetUdma_freeRing(hRxFlow->fqRing, 0U, NULL, NULL);
            Enet_assert(UDMA_SOK == retVal);
#endif

            retVal = EnetUdma_freeRing(hRxFlow->cqRing, 0U, NULL, NULL);
            Enet_assert(UDMA_SOK == retVal);

            retVal = Udma_flowDetach(hRxFlow->hUdmaFlow);
            Enet_assert(UDMA_SOK == retVal);
            hRxFlow->initFlag = false;

            /* Free Rx Flow driver object memory */
            EnetUdma_memMgrFreeRxFlowObj(hRxFlow);
        }

        EnetOsal_restoreAllIntr(intrKey);
    }

    return retVal;
}

int32_t EnetUdma_ringEnqueue(Udma_RingHandle hUdmaRing,
                            EnetUdma_DmaDesc *pDmaDesc,
                            bool disableCacheOpsFlag,
                            EnetUdma_Dir transferDir
#if (UDMA_SOC_CFG_PROXY_PRESENT == 1)
                            ,
                            Udma_ProxyHandle hUdmaProxy
#endif
                            )
{
    int32_t retVal = UDMA_SOK;
    bool isExposedRing;
    void *virtBufPtr;
    uint64_t physDescPtr;

    if ((pDmaDesc != NULL) && (hUdmaRing != NULL))
    {
        EnetDma_Pkt *dmaPkt = pDmaDesc->dmaPkt;
        uint32_t numScatterSegments = dmaPkt->sgList.numScatterSegments;

        EnetUdma_CpswHpdDesc *pHpdDesc = (EnetUdma_CpswHpdDesc *)pDmaDesc;
        CSL_UdmapCppi5HMPD *pHDesc = &pHpdDesc->hostDesc;
        isExposedRing = (Udma_ringGetMode(hUdmaRing) == TISCI_MSG_VALUE_RM_RING_MODE_RING);
#if defined(SOC_AM64X) || defined(SOC_AM243X) || defined(SOC_AM62AX)
        isExposedRing = 0U;
#endif
        physDescPtr = (uint64_t)EnetUtils_virtToPhys((void *)&pHpdDesc->hostDesc, NULL);

        for (uint32_t i = 0; i < numScatterSegments; i++)
        {
            if (ENET_UDMA_CPSW_IS_HBD_IDX(i))
            {
                pHDesc =  &pDmaDesc->hostBufDesc[i - ENET_UDMA_CPSW_HOSTBUFDESC_INDEX].desc;
            }
            virtBufPtr = (void *)(uintptr_t)pHDesc->bufPtr;

            pHDesc->bufPtr =
                    EnetUtils_virtToPhys((void *)(uintptr_t)pHDesc->bufPtr, NULL);

            pHDesc->orgBufPtr =
                    EnetUtils_virtToPhys((void *)(uintptr_t)pHDesc->orgBufPtr, NULL);

            if(pHDesc->nextDescPtr != 0U)
            {
                pHDesc->nextDescPtr =
                        EnetUtils_virtToPhys((void *)(uintptr_t)pHDesc->nextDescPtr, NULL);
            }

            if ((false == disableCacheOpsFlag) && (FALSE == Enet_isCacheCoherent()))
            {
                /* Wb Invalidate data cache for Tx */
                if (ENET_UDMA_DIR_RX == transferDir)
                {
                    EnetOsal_cacheInv(virtBufPtr, CSL_udmapCppi5GetOrgBufferLen(pHDesc));
                }
                else
                {
                    if (dmaPkt->sgList.list[i].disableCacheOps != true)
                    {
                        EnetOsal_cacheWbInv(virtBufPtr, CSL_udmapCppi5GetBufferLen(pHDesc));
                    }
                }

            }
        }

        if (FALSE == Enet_isCacheCoherent())
        {
            EnetOsal_cacheWbInv((void *)pDmaDesc, sizeof(*pDmaDesc));
        }

        if (isExposedRing == true)
        {
           /* Udma_ringPrime doesn't check for invalid parameters and also doesn't return
            * any errors */
           Udma_ringPrime(hUdmaRing, physDescPtr);
        }
        else
        {
#if (UDMA_SOC_CFG_PROXY_PRESENT == 1)
            if (hUdmaProxy == NULL)
            {
                retVal = Udma_ringQueueRaw(hUdmaRing, physDescPtr);
            }
            else
            {
                retVal = EnetUdma_ringProxyEnqueue(hUdmaRing, hUdmaProxy, physDescPtr);
            }
#else
                retVal = Udma_ringQueueRaw(hUdmaRing, physDescPtr);
#endif
        }
    }
    else
    {
        retVal = UDMA_EFAIL;
    }

    return retVal;
}

int32_t EnetUdma_ringDequeue(Udma_RingHandle hUdmaRing,
                            EnetUdma_DmaDesc **pDmaDesc,
                            bool disableCacheOpsFlag,
                            EnetUdma_Dir transferDir)
{
    int32_t retVal = UDMA_SOK;
    bool isExposedRing;
    uint64_t pDesc = 0;
    CSL_UdmapCppi5HMPD *pDescEntry;
    CSL_UdmapCppi5HMPD *pNextDescEntryVirt;
    EnetUdma_DmaDesc *pVirtDmaDesc;

    if ((pDmaDesc != NULL) && (hUdmaRing != NULL))
    {
        isExposedRing = (Udma_ringGetMode(hUdmaRing) == TISCI_MSG_VALUE_RM_RING_MODE_RING);
#if defined(SOC_AM64X) || defined(SOC_AM243X) || defined(SOC_AM62AX)
        isExposedRing = 0U;
#endif
        if (isExposedRing == true)
        {
            /* Udma_ringPrimeRead doesn't check for invalid parameters and also doesn't return
             * any errors */
            Udma_ringPrimeRead(hUdmaRing, &pDesc);
            if (pDesc == 0U)
            {
                *pDmaDesc = NULL;
                retVal = UDMA_ETIMEOUT;
            }
        }
        else
        {
            retVal = Udma_ringDequeueRaw(hUdmaRing, &pDesc);
            if (UDMA_SOK != retVal)
            {
                *pDmaDesc = NULL;
            }
        }

        if (UDMA_SOK == retVal)
        {
            pVirtDmaDesc = (EnetUdma_DmaDesc*) EnetUtils_physToVirt(pDesc, NULL);
            if (FALSE == Enet_isCacheCoherent())
            {
#if ENET_CFG_IS_ON(DEV_ERROR)
                Enet_assert(ENET_UTILS_IS_ALIGNED(pVirtDmaDesc, UDMA_CACHELINE_ALIGNMENT));
#endif
                EnetOsal_cacheInv((void*) pVirtDmaDesc, sizeof(*pVirtDmaDesc));
            }
            *pDmaDesc = pVirtDmaDesc;
            pDescEntry = &pVirtDmaDesc->hpdDesc.hostDesc;
            while (pDescEntry != NULL)
            {
                pDescEntry->bufPtr = (uint64_t) EnetUtils_physToVirt(pDescEntry->bufPtr, NULL);
                pDescEntry->orgBufPtr = (uint64_t) EnetUtils_physToVirt(pDescEntry->orgBufPtr, NULL);
                if (pDescEntry->nextDescPtr != 0ULL)
                {
                    pNextDescEntryVirt = (CSL_UdmapCppi5HMPD*) EnetUtils_physToVirt(pDescEntry->nextDescPtr, NULL);
                    pDescEntry->nextDescPtr = (uintptr_t) pNextDescEntryVirt;
                    pDescEntry = pNextDescEntryVirt;
                }
                else
                {
                    pDescEntry = (CSL_UdmapCppi5HMPD*) pDescEntry->nextDescPtr;
                }
            }

            /* Debug point -> The Cortex A cores can do speculative reads which
             * can mandate to do cache_invalidate before packets are received.
             * If Cortex-A doesn't support coherency, invalidate the data buffer
             * so the CPU is forced to read new data from HW.
             * We only need to so this on the Cortex v7A devices because of possible
             * speculative reads done by the CPU core while the buffers are owned by
             * the DMA hardware
             */
        }
    }
    else
    {
        retVal = UDMA_EFAIL;
    }

    return retVal;
}

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
void EnetUdma_reconfigureDescQ(EnetUdma_TxChObj *pTxCh)
{
    EnetUdma_DmaDescQ tempQ;
    EnetUdma_DmaDesc *dmaDesc;
    EnetUdma_CpswHpdDesc *pHpdDesc;

    Enet_assert(pTxCh != NULL);
    Enet_assert(pTxCh->hDmaDescPool != NULL);

    EnetUdma_DmaDescQ *descQPtr = pTxCh->hDmaDescPool;
    uint32_t retQIdx = Udma_ringGetNum(pTxCh->cqRing);

    EnetUdma_dmaDescQInit(&tempQ);

    while (EnetUdma_dmaDescQCount(descQPtr) > 0U)
    {
        dmaDesc = EnetUdma_dmaDescDeque(descQPtr);
        Enet_assert(dmaDesc != NULL);

        pHpdDesc = (EnetUdma_CpswHpdDesc *)dmaDesc;
        CSL_udmapCppi5SetReturnPolicy(&pHpdDesc->hostDesc,
                                      ENET_UDMA_UDMAP_DESC_TYPE,
                                      CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPOLICY_VAL_ENTIRE_PKT,
                                      CSL_UDMAP_CPPI5_PD_PKTINFO2_EARLYRET_VAL_NO,
                                      CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPUSHPOLICY_VAL_TO_TAIL,
                                      retQIdx);
        EnetUdma_dmaDescEnque(&tempQ, dmaDesc);
    }

    EnetUdma_dmaDescCopyQ(descQPtr, &tempQ);

    return;
}
#endif

void EnetUdma_initTxFreeDescQ(EnetUdma_TxChObj *pTxCh)
{
    uint32_t i;
    uint32_t descType;
    uint32_t alignSize = UDMA_CACHELINE_ALIGNMENT;

    Enet_assert(pTxCh != NULL);
    Enet_assert(pTxCh->hDmaDescPool != NULL);

    EnetUdma_dmaDescQInit(pTxCh->hDmaDescPool);

    for (i = 0; i < pTxCh->txChPrms.numTxPkts; i++)
    {
        EnetUdma_DmaDesc *dmaDesc =
            pTxCh->txChPrms.dmaDescAllocFxn(pTxCh->txChPrms.cbArg, alignSize);
        if (dmaDesc == NULL)
        {
            ENETTRACE_ERR("[Enet UDMA Error] Tx DMA descriptor memory allocation failed !!\n");
            Enet_assert(false);
        }
        else if (!ENET_UTILS_IS_ALIGNED(dmaDesc, alignSize))
        {
            ENETTRACE_ERR("[Enet UDMA Error] dmaDesc memory not aligned!!\n");
            Enet_assert(false);
        }
        else
        {
            EnetUdma_CpswHpdDesc *pHpdDesc = (EnetUdma_CpswHpdDesc *)dmaDesc;
            CSL_UdmapCppi5HMPD *hostDesc  = &pHpdDesc->hostDesc;

            descType = ENET_UDMA_UDMAP_DESC_TYPE;

            EnetUdma_dmaDescInit(dmaDesc);

            CSL_udmapCppi5SetDescType(hostDesc, descType);

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
            CSL_udmapCppi5SetReturnPolicy(hostDesc,
                                          descType,
                                          CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPOLICY_VAL_ENTIRE_PKT,
                                          CSL_UDMAP_CPPI5_PD_PKTINFO2_EARLYRET_VAL_NO,
                                          CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPUSHPOLICY_VAL_TO_TAIL,
                                          Udma_ringGetNum(pTxCh->cqRing));
#else
            //TODO IMP - what to do?
#endif
            EnetUdma_dmaDescEnque(pTxCh->hDmaDescPool, dmaDesc);
        }
    }
}

void EnetUdma_deInitTxFreeDescQ(EnetUdma_TxChObj *pTxCh)
{
    EnetUdma_DmaDesc *pDmaDesc;

    pDmaDesc = EnetUdma_dmaDescDeque(pTxCh->hDmaDescPool);
    while (NULL != pDmaDesc)
    {
        pTxCh->txChPrms.dmaDescFreeFxn(pTxCh->txChPrms.cbArg, pDmaDesc);
        pDmaDesc = EnetUdma_dmaDescDeque(pTxCh->hDmaDescPool);
    }
}

void EnetUdma_initRxFreeDescQ(EnetUdma_RxFlowObj *pRxFlow)
{
    uint32_t i;
    uint32_t descType;
    uint32_t alignSize = UDMA_CACHELINE_ALIGNMENT;

    EnetUdma_dmaDescQInit(pRxFlow->hDmaDescPool);

    for (i = 0; i < pRxFlow->rxFlowPrms.numRxPkts; i++)
    {
        EnetUdma_DmaDesc *dmaDesc =
            pRxFlow->rxFlowPrms.dmaDescAllocFxn(pRxFlow->rxFlowPrms.cbArg, alignSize);
        if (dmaDesc == NULL)
        {
            ENETTRACE_ERR("[Enet UDMA Error] Rx DMA descriptor memory allocation failed !!\n");
            Enet_assert(dmaDesc != NULL);
        }
        else if (!ENET_UTILS_IS_ALIGNED(dmaDesc, alignSize))
        {
            ENETTRACE_ERR("[Enet UDMA Error] dmaDesc memory not aligned!!\n");
            Enet_assert(false);
        }
        else
        {
            EnetUdma_CpswHpdDesc *pHpdDesc = (EnetUdma_CpswHpdDesc *)dmaDesc;
            CSL_UdmapCppi5HMPD *hostDesc  = &pHpdDesc->hostDesc;

            EnetUdma_dmaDescInit(dmaDesc);

            descType = ENET_UDMA_UDMAP_DESC_TYPE;

            CSL_udmapCppi5SetDescType(hostDesc, descType);

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
            CSL_udmapCppi5SetReturnPolicy(hostDesc,
                                          descType,
                                          CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPOLICY_VAL_ENTIRE_PKT,
                                          CSL_UDMAP_CPPI5_PD_PKTINFO2_EARLYRET_VAL_NO,
                                          CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPUSHPOLICY_VAL_TO_TAIL,
                                          Udma_ringGetNum(pRxFlow->cqRing));
#else
            //TODO IMP - what to do?
#endif
            EnetUdma_dmaDescEnque(pRxFlow->hDmaDescPool, dmaDesc);
        }
    }
}

void EnetUdma_deInitRxFreeDescQ(EnetUdma_RxFlowObj *pRxFlow)
{
    EnetUdma_DmaDesc *pDmaDesc;

    pDmaDesc = EnetUdma_dmaDescDeque(pRxFlow->hDmaDescPool);

    while (NULL != pDmaDesc)
    {
        pRxFlow->rxFlowPrms.dmaDescFreeFxn(pRxFlow->rxFlowPrms.cbArg, pDmaDesc);
        pDmaDesc = EnetUdma_dmaDescDeque(pRxFlow->hDmaDescPool);
    }
}

/*!
 * \defgroup EnetUdma_HELPERS_BUFF_DESC_QUEUE EnetUdma Buffer Descriptor Queue Helpers
 * @{
 */

/*! Zero init the buffer descriptor queue */
void EnetUdma_buffDescInit(EnetUdma_CpswHpdDesc *hpdDesc)
{
    CSL_UdmapCppi5HMPD *hostDesc = &hpdDesc->hostDesc;

    uint32_t descType = (uint32_t)CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST;

    CSL_udmapCppi5SetDescType(hostDesc, descType);
    CSL_udmapCppi5SetEpiDataPresent(hostDesc, true);
    CSL_udmapCppi5SetPsDataLoc(hostDesc, TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_PS_END_PD);
    CSL_udmapCppi5SetPsDataLen(hostDesc, ENET_UDMA_PROTOCOL_SPECIFIC_INFO_BLOCK_SIZE);
    CSL_udmapCppi5HostSetPktLen(hostDesc, 0U);
    CSL_udmapCppi5SetPsFlags(hostDesc, 0U);
    CSL_udmapCppi5SetIds(hostDesc, descType, 0U, 0U);
    CSL_udmapCppi5SetSrcTag(hostDesc, 0x0000U);
    CSL_udmapCppi5SetDstTag(hostDesc, 0x0000U);
    CSL_udmapCppi5LinkDesc(hostDesc, 0U);
    CSL_udmapCppi5SetBufferAddr(hostDesc, (uint64_t)NULL);
    CSL_udmapCppi5SetBufferLen(hostDesc, 0U);
    CSL_udmapCppi5SetOrgBufferAddr(hostDesc, (uint64_t)NULL);
    CSL_udmapCppi5SetOrgBufferLen(hostDesc, 0U);
}

/*  Packet operations  */

/*!
 * \defgroup EnetUdma_HELPERS_PACKET EnetUdma Packet Helpers
 * @{
 */
/*! DMA descriptor Q init */
void EnetUdma_dmaDescQInit(EnetUdma_DmaDescQ *pDmaDescQ)
{
    pDmaDescQ->head = NULL;
    pDmaDescQ->tail = NULL;
    pDmaDescQ->count = 0;
}

uint32_t EnetUdma_dmaDescQCount(EnetUdma_DmaDescQ *pDmaDescQ)
{
    return pDmaDescQ->count;
}

/*! Zero init the packet */
void EnetUdma_dmaDescInit(EnetUdma_DmaDesc *pDmaDesc)
{
    memset(pDmaDesc, 0, sizeof(*pDmaDesc));
    pDmaDesc->pNextDesc   = NULL;
    pDmaDesc->dmaPkt = NULL;

    EnetUdma_buffDescInit(&pDmaDesc->hpdDesc);
}

void EnetUdma_dmaDescEnque(EnetUdma_DmaDescQ *pDmaDescQ,
                          EnetUdma_DmaDesc *pDmaDesc)
{
    if (pDmaDesc != NULL)
    {
        uintptr_t key = EnetOsal_disableAllIntr();

        pDmaDesc->pNextDesc = NULL;

        if (NULL == pDmaDescQ->head)
        {
            /* Queue is empty, make head point to new packet */
            pDmaDescQ->head = pDmaDesc;
        }
        else
        {
            /* Queue is not empty, make previous tail point to new packet */
            pDmaDescQ->tail->pNextDesc = pDmaDesc;
        }

        /* Make tail of queue point to new */
        pDmaDescQ->count++;
        pDmaDescQ->tail = pDmaDesc;

        EnetOsal_restoreAllIntr(key);
    }
    else
    {
        Enet_assert(false);
    }
}

EnetUdma_DmaDesc *EnetUdma_dmaDescDeque(EnetUdma_DmaDescQ *pDmaDescQ)
{
    EnetUdma_DmaDesc *pDmaDesc = NULL;

    if (NULL != pDmaDescQ)
    {
        uintptr_t key = EnetOsal_disableAllIntr();
        pDmaDesc = pDmaDescQ->head;

        if (NULL != pDmaDesc)
        {
            pDmaDescQ->head = pDmaDesc->pNextDesc;
            if (NULL == pDmaDescQ->head)
            {
                pDmaDescQ->tail = NULL;
            }

            pDmaDescQ->count--;
            pDmaDesc->pNextDesc = NULL;
        }

        EnetOsal_restoreAllIntr(key);
    }

    return(pDmaDesc);
}

void EnetUdma_dmaDescCopyQ(EnetUdma_DmaDescQ *pDstQueue,
                          const EnetUdma_DmaDescQ *pSrcQueue)
{
    uintptr_t key = EnetOsal_disableAllIntr();

    pDstQueue->head = pSrcQueue->head;
    pDstQueue->tail = pSrcQueue->tail;
    pDstQueue->count = pSrcQueue->count;

    EnetOsal_restoreAllIntr(key);
}

int32_t EnetUdma_registerEvent(EnetUdma_udmaInfo *pUdmaInfo,
                              Udma_RingHandle hUdmaRing,
                              Udma_EventCallback eventCb,
                              void *cbArg,
                              uint32_t eventType)
{
    int32_t retVal = UDMA_SOK;

    Udma_EventPrms evtPrms;

    /* Register for the UDMA completion event. The UDMA driver sets up
     * the Int Aggr and Int Router according to our preferred interrupt
     * number.  The interrupt registration itself doesn't happen here */
    UdmaEventPrms_init(&evtPrms);

    switch (eventType)
    {
        case UDMA_EVENT_TYPE_DMA_COMPLETION:
            evtPrms.chHandle = pUdmaInfo->hUdmaCh;
            break;

        case UDMA_EVENT_TYPE_RING:
            Enet_assert(NULL != hUdmaRing);
            evtPrms.ringHandle = hUdmaRing;
            break;
#if (UDMA_SOC_CFG_RING_MON_PRESENT == 1)
        case UDMA_EVENT_TYPE_RING_MON:
            Enet_assert(NULL != hUdmaRing);
            Enet_assert(NULL != pUdmaInfo->hUdmaRingMon);
            evtPrms.ringHandle = hUdmaRing;
            evtPrms.monHandle  = pUdmaInfo->hUdmaRingMon;
            break;
#endif
        default:
            ENETTRACE_ERR("[Enet UDMA Error] Event registration failed allocation failed !!\n");
            Enet_assert(false);
    }

    evtPrms.eventType         = eventType;
    evtPrms.eventMode         = UDMA_EVENT_MODE_SHARED;

    /* Global event can be used when number of events is too large
     * (i.e. large number of RX flows and/or TX channels) */
    if (pUdmaInfo->useGlobalEvt)
    {
#if defined(SOC_AM62AX)
        evtPrms.masterEventHandle = Udma_eventGetGlobalHandle(pUdmaInfo->hUdmaDrv);
#else
        evtPrms.controllerEventHandle = Udma_eventGetGlobalHandle(pUdmaInfo->hUdmaDrv);
#endif
    }
    else
    {
#if defined(SOC_AM62AX)
        evtPrms.masterEventHandle = NULL;
#else
        evtPrms.controllerEventHandle = NULL;
#endif
    }

    evtPrms.eventCb           = eventCb;
    evtPrms.appData           = cbArg;

    retVal = Udma_eventRegister(pUdmaInfo->hUdmaDrv, pUdmaInfo->hUdmaEvt, &evtPrms);

    return retVal;
}

int32_t EnetUdma_unregisterEvent(Udma_EventHandle hUdmaEvt)
{
    int32_t retVal = UDMA_SOK;

    retVal = Udma_eventUnRegister(hUdmaEvt);

    return retVal;
}

int32_t EnetUdma_dmaDescCheck(EnetUdma_DmaDesc *dmaDesc,
                             EnetUdma_Dir transferDir)
{
    int32_t retVal = UDMA_SOK;

#if ENET_CFG_IS_ON(DEV_ERROR)
    EnetUdma_CpswHpdDesc *pHpdDesc = (EnetUdma_CpswHpdDesc *)dmaDesc;
    CSL_UdmapCppi5HMPD *hostDesc  = &pHpdDesc->hostDesc;

    /* Is the desc memory aligned? */
    if (0ULL != ((uint64_t)hostDesc % ENET_UDMA_HPD_SIZE))
    {
        retVal = UDMA_EBADARGS;
    }

    /* Is the desc of Host Packet type? */
    if (retVal == UDMA_SOK)
    {
        if ((uint32_t)CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST !=
            CSL_udmapCppi5GetDescType(hostDesc))
        {
            retVal = UDMA_EBADARGS;
        }
    }

    /* Is the buffer address not NULL? */
    if (retVal == UDMA_SOK)
    {
        if (0ULL == CSL_udmapCppi5GetBufferAddr(hostDesc))
        {
            retVal = UDMA_EBADARGS;
        }
    }

    /* Is the buffer length > 0? */
    if (retVal == UDMA_SOK)
    {
        if (0U == CSL_udmapCppi5GetBufferLen(hostDesc))
        {
            if (ENET_UDMA_DIR_TX == transferDir)
            {
                retVal = UDMA_EBADARGS;
            }
        }
    }
#endif
    return retVal;
}

/*! Rx flow object allocation function  */
EnetUdma_RxFlowObj *EnetUdma_memMgrAllocRxFlowObj(void)
{
    EnetUdma_RxFlowObj *pRxFlow = NULL;
    EnetUdma_RxFlowObjMem *pRxFlowMemObj;

    pRxFlowMemObj = (EnetUdma_RxFlowObjMem *)EnetQueue_deq(&gEnetUdmaMemMgrObj.rxFlowObjMemQ);
#if ENET_CFG_IS_ON(DEV_ERROR)
    Enet_assert(pRxFlowMemObj != NULL);
#endif
    if (pRxFlowMemObj != NULL)
    {
        pRxFlow = &pRxFlowMemObj->rxFlowObj;
    }

    return pRxFlow;
}

/*! Rx flow object free function  */
void EnetUdma_memMgrFreeRxFlowObj(EnetUdma_RxFlowObj *pRxFlow)
{
    EnetUdma_RxFlowObjMem *pRxFlowMemObj;

#if ENET_CFG_IS_ON(DEV_ERROR)
    Enet_assert(NULL != pRxFlow);
#endif

    pRxFlowMemObj = container_of(pRxFlow, EnetUdma_RxFlowObjMem, rxFlowObj);
    EnetQueue_enq(&gEnetUdmaMemMgrObj.rxFlowObjMemQ, &pRxFlowMemObj->node);
    pRxFlowMemObj = NULL;

    return;
}

/*! TX channel object allocation function  */
EnetUdma_TxChObj *EnetUdma_memMgrAllocTxChObj(void)
{
    EnetUdma_TxChObj *pTxChObj = NULL;
    EnetUdma_TxChObjMem *pTxChMemObj;

    pTxChMemObj = (EnetUdma_TxChObjMem *)EnetQueue_deq(&gEnetUdmaMemMgrObj.txChObjMemQ);
#if ENET_CFG_IS_ON(DEV_ERROR)
    Enet_assert(pTxChMemObj != NULL);
#endif
    if (pTxChMemObj != NULL)
    {
        pTxChObj = &pTxChMemObj->txChObj;
    }

    return pTxChObj;
}

/*! TX channel object free function  */
void EnetUdma_memMgrFreeTxChObj(EnetUdma_TxChObj *pTxChObj)
{
    EnetUdma_TxChObjMem *pTxChMemObj;

#if ENET_CFG_IS_ON(DEV_ERROR)
    Enet_assert(NULL != pTxChObj);
#endif

    pTxChMemObj = container_of(pTxChObj, EnetUdma_TxChObjMem, txChObj);
    EnetQueue_enq(&gEnetUdmaMemMgrObj.txChObjMemQ, &pTxChMemObj->node);
    pTxChObj = NULL;

    return;
}

#if (UDMA_SOC_CFG_RING_MON_PRESENT == 1)
/*! Ring Monitor object allocation function  */
Udma_RingMonHandle EnetUdma_memMgrAllocRingMonObj(void)
{
    struct Udma_RingMonObj *pRingMonObj = NULL;
    EnetUdma_RingMonObjMem *pRingMemObj;

    pRingMemObj = (EnetUdma_RingMonObjMem *)
                  EnetQueue_deq(&gEnetUdmaMemMgrObj.ringMonObjMemQ);
#if ENET_CFG_IS_ON(DEV_ERROR)
    Enet_assert(pRingMemObj != NULL);
#endif
    if (pRingMemObj != NULL)
    {
        pRingMonObj = &pRingMemObj->ringMonObj;
    }

    return pRingMonObj;
}

/*! Ring Monitor object free function  */
void EnetUdma_memMgrFreeRingMonObj(Udma_RingMonHandle hUdmaRingMon)
{
    EnetUdma_RingMonObjMem *pRingMemObj;

#if ENET_CFG_IS_ON(DEV_ERROR)
    Enet_assert(NULL != hUdmaRingMon);
#endif

    pRingMemObj = container_of(hUdmaRingMon, EnetUdma_RingMonObjMem, ringMonObj);
    EnetQueue_enq(&gEnetUdmaMemMgrObj.ringMonObjMemQ, &pRingMemObj->node);
    hUdmaRingMon = NULL;

    return;
}
#endif

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
/*! TD Cq Ring memory object allocation function  */
void *EnetUdma_memMgrAllocTdCqRingMemObj(void)
{
    void *pTdCqRingMem = NULL;
    EnetUdma_TdCqRingObjMem *pTdCqRingMemObj;

    pTdCqRingMemObj = (EnetUdma_TdCqRingObjMem *)
                      EnetQueue_deq(&gEnetUdmaMemMgrObj.tdCqRingObjMemQ);
#if ENET_CFG_IS_ON(DEV_ERROR)
    Enet_assert(NULL != pTdCqRingMemObj);
#endif
    if (pTdCqRingMemObj != NULL)
    {
        pTdCqRingMem = (void *)&pTdCqRingMemObj->tdCqRingMemObj[0U];
        Enet_assert(ENET_UTILS_IS_ALIGNED(pTdCqRingMem, UDMA_CACHELINE_ALIGNMENT));
        memset(pTdCqRingMem, 0U, sizeof(uint64_t));
    }

    return pTdCqRingMem;
}
#endif

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
/*! TD Cq Ring memory object free function  */
void EnetUdma_memMgrFreeTdCqRingMemObj(void *pTdCqRingMem)
{
    EnetUdma_TdCqRingObjMem *pTdCqRingMemObj;

#if ENET_CFG_IS_ON(DEV_ERROR)
    Enet_assert(NULL != pTdCqRingMem);
#endif

    pTdCqRingMemObj = container_of((const uint64_t (*)[ENET_UTILS_ALIGN(ENET_UDMA_TDCQ_RING_ELE_CNT,
                                                                        UDMA_CACHELINE_ALIGNMENT)]) pTdCqRingMem,
                                   EnetUdma_TdCqRingObjMem,
                                   tdCqRingMemObj);
    EnetQueue_enq(&gEnetUdmaMemMgrObj.tdCqRingObjMemQ, &pTdCqRingMemObj->node);
    pTdCqRingMem = NULL;
}
#endif

void EnetUdma_memMgrInit(void)
{
    uint32_t i;

    const EnetUdma_MemCfg *ethUdmaCfg = EnetUdmaMem_getCfg();
    EnetUdma_RxFlowObjMem *rxFlowObjMem = ethUdmaCfg->rxFlowObjMemCfg.rxFlowObjMemContainerBase;
    EnetUdma_TxChObjMem *txChObjMem = ethUdmaCfg->txChObjMemCfg.txChObjContainerBase;

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
    EnetUdma_RingMonObjMem *ringMonObjMem = ethUdmaCfg->ringMonObjMemCfg.ringMonObjMemContainerBase;
    EnetUdma_TdCqRingObjMem *tdCqRingObjMem = ethUdmaCfg->tdCqRingObjMemCfg.tdCqRingObjMemContainerBase;
#endif

    if (gEnetUdmaMemMgrObj.clientCnt == 0U)
    {
        memset(&gEnetUdmaMemMgrObj, 0U, sizeof(gEnetUdmaMemMgrObj));
        memset(rxFlowObjMem, 0U, (ethUdmaCfg->rxFlowObjMemCfg.rxFlowObjMemContainerSize));
        memset(txChObjMem, 0U, (ethUdmaCfg->txChObjMemCfg.txChObjContainerSize));

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        memset(ringMonObjMem, 0U, (ethUdmaCfg->ringMonObjMemCfg.ringMonObjMemContainerSize));
        memset(tdCqRingObjMem, 0U, (ethUdmaCfg->tdCqRingObjMemCfg.tdCqRingObjMemContainerSize));
#endif
        /*********************** Rx flow object memory  Q ************************/
        EnetQueue_initQ(&gEnetUdmaMemMgrObj.rxFlowObjMemQ);
        for (i = 0U; i < ethUdmaCfg->rxFlowObjMemCfg.numFlows; i++)
        {
            EnetQueue_enq(&gEnetUdmaMemMgrObj.rxFlowObjMemQ, &rxFlowObjMem[i].node);
        }

        /*********************** Tx channel object memory  Q ************************/
        EnetQueue_initQ(&gEnetUdmaMemMgrObj.txChObjMemQ);
        for (i = 0U; i < ethUdmaCfg->txChObjMemCfg.numCh; i++)
        {
            EnetQueue_enq(&gEnetUdmaMemMgrObj.txChObjMemQ, &txChObjMem[i].node);
        }

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
        /*********************** Ring monitor object memory  Q ************************/
        EnetQueue_initQ(&gEnetUdmaMemMgrObj.ringMonObjMemQ);
        for (i = 0U; i < ethUdmaCfg->ringMonObjMemCfg.numRings; i++)
        {
            EnetQueue_enq(&gEnetUdmaMemMgrObj.ringMonObjMemQ, &ringMonObjMem[i].node);
        }

        /*********************** Teardown CQ object memory  Q ************************/
        EnetQueue_initQ(&gEnetUdmaMemMgrObj.tdCqRingObjMemQ);
        for (i = 0U; i < (ethUdmaCfg->tdCqRingObjMemCfg.numTxCh + ENET_UDMA_RX_CH_NUM); i++)
        {
            EnetQueue_enq(&gEnetUdmaMemMgrObj.tdCqRingObjMemQ, tdCqRingObjMem[i].node);
        }
#endif
    }

    gEnetUdmaMemMgrObj.clientCnt++;
}

void EnetUdma_memMgrDeInit(void)
{
    gEnetUdmaMemMgrObj.clientCnt--;

    if (gEnetUdmaMemMgrObj.clientCnt == 0U)
    {
        memset(&gEnetUdmaMemMgrObj, 0U, sizeof(gEnetUdmaMemMgrObj));
    }
}

void EnetUdma_txCqIsr(Udma_EventHandle hUdmaEvt,
                     uint32_t eventType,
                     void *appData)
{
    int32_t retVal;
    EnetUdma_TxChObj *pTxCh = (EnetUdma_TxChObj *)appData;
    EnetPer_Handle hPer = pTxCh->hDma->hPer;
    EnetQ curIsrCQ;

    EnetQueue_initQ(&curIsrCQ);

    retVal = EnetUdma_retrievePkts(hPer,
                                   pTxCh->cqRing,
                                  &curIsrCQ,
                                  pTxCh->hDmaDescPool,
                                  pTxCh->txChPrms.disableCacheOpsFlag,
                                  ENET_UDMA_DIR_TX);
    Enet_assert(retVal == UDMA_SOK);

    EnetQueue_append(&pTxCh->cqIsrQ, &curIsrCQ);

    if ((pTxCh->txChPrms.notifyCb != NULL)
        &&
        (EnetQueue_getQCount(&curIsrCQ) != 0))
    {
        pTxCh->txChPrms.notifyCb(pTxCh->txChPrms.cbArg);
    }
}

void EnetUdma_rxCqIsr(Udma_EventHandle hUdmaEvt,
                     uint32_t eventType,
                     void *appData)
{
    int32_t retVal;
    EnetUdma_RxFlowObj *pRxFlow = (EnetUdma_RxFlowObj *)appData;
    EnetPer_Handle hPer = pRxFlow->hDma->hPer;
    EnetQ curIsrCQ;

    EnetQueue_initQ(&curIsrCQ);

    retVal = EnetUdma_retrievePkts(hPer,
                                   pRxFlow->cqRing,
                                  &curIsrCQ,
                                  pRxFlow->hDmaDescPool,
                                  pRxFlow->rxFlowPrms.disableCacheOpsFlag,
                                  ENET_UDMA_DIR_RX);
    Enet_assert(UDMA_SOK == retVal);

    EnetQueue_append(&pRxFlow->cqIsrQ, &curIsrCQ);

    if ((pRxFlow->rxFlowPrms.notifyCb != NULL) &&
        (EnetQueue_getQCount(&curIsrCQ) != 0))
    {
        pRxFlow->rxFlowPrms.notifyCb(pRxFlow->rxFlowPrms.cbArg);
    }
}

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
int32_t EnetUdma_allocProxy(Udma_DrvHandle hUdmaDrv,
                           Udma_ProxyHandle hUdmaProxy,
                           Udma_RingHandle hUdmaRing)
{
    int32_t retVal;
    Udma_ProxyCfg proxyCfg;

    /* Allocate a proxy for queue operation */
    retVal = Udma_proxyAlloc(hUdmaDrv, hUdmaProxy, UDMA_PROXY_ANY);
    ENETTRACE_ERR_IF((UDMA_SOK != retVal), "CPSWDMA:Proxy Alloc Failed Status=0x%x\n", retVal);

    if (UDMA_SOK == retVal)
    {
        /* Config proxy for queue operation */
        proxyCfg.proxyMode = CSL_PROXY_QUEUE_ACCESS_MODE_TAIL;
        proxyCfg.elemSize  = UDMA_RING_ES_8BYTES;
        proxyCfg.ringNum   = Udma_ringGetNum(hUdmaRing);

        retVal = Udma_proxyConfig(hUdmaProxy, &proxyCfg);

        if (UDMA_SOK != retVal)
        {
            ENETTRACE_ERR("CPSWDMA:Proxy Config Failed Status=0x%x\n", retVal);
            Enet_assert(UDMA_SOK == Udma_proxyFree(hUdmaProxy));
        }
    }

    return retVal;
}
#endif

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
int32_t EnetUdma_freeProxy(Udma_ProxyHandle hUdmaProxy)
{
    int32_t retVal = UDMA_EBADARGS;

    if (NULL != hUdmaProxy)
    {
        retVal = Udma_proxyFree(hUdmaProxy);
    }

    return retVal;
}
#endif

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
Udma_RingMonHandle EnetUdma_allocRingMon(Udma_DrvHandle hUdmaDrv,
                                        Udma_RingHandle hUdmaRing,
                                        uint32_t elemCnt,
                                        EnetUdma_RingMonCfg *pRingMonCfg)
{
    int32_t retVal = UDMA_SOK;
    Udma_RingMonPrms monPrms;
    Udma_RingMonHandle hRingMon;

    /* Allocate ring monitor object memory*/
    hRingMon = EnetUdma_memMgrAllocRingMonObj();
    if (NULL == hRingMon)
    {
        ENETTRACE_ERR("Ring monitor object memory allocation failed!!\n");
        retVal = UDMA_EALLOC;
    }

    if (UDMA_SOK == retVal)
    {
        /* Allocate and configure a ring monitor */
        retVal = Udma_ringMonAlloc(hUdmaDrv, hRingMon, UDMA_RING_MON_ANY);
        if (UDMA_SOK != retVal)
        {
            ENETTRACE_ERR("Ring monitor alloc failed Status=0x%x\n", retVal);
            EnetUdma_memMgrFreeRingMonObj(hRingMon);
        }
    }

    if (UDMA_SOK == retVal)
    {
        UdmaRingMonPrms_init(&monPrms);
        monPrms.source  = TISCI_MSG_VALUE_RM_MON_SRC_ELEM_CNT;
        monPrms.mode    = pRingMonCfg->mode;
        monPrms.ringNum = Udma_ringGetNum(hUdmaRing);
        monPrms.data0   = pRingMonCfg->data0;
        monPrms.data1   = pRingMonCfg->data1;

        retVal = Udma_ringMonConfig(hRingMon, &monPrms);
        if (UDMA_SOK != retVal)
        {
            ENETTRACE_ERR(" Ring monitor config failed Status=0x%x\n", retVal);
            /*If free also fails, we can't do much but assert */
            Enet_assert(UDMA_SOK == Udma_ringMonFree(hRingMon));
            hRingMon = NULL;
        }
    }

    return hRingMon;
}
#endif

#if (UDMA_SOC_CFG_UDMAP_PRESENT == 1)
int32_t EnetUdma_freeRingMon(Udma_RingMonHandle hRingMon)
{
    int32_t retVal = UDMA_EBADARGS;

    if (NULL != hRingMon)
    {
        retVal = Udma_ringMonFree(hRingMon);
        EnetUdma_memMgrFreeRingMonObj(hRingMon);
    }

    return retVal;
}
#endif

void EnetUdmaStats_updateNotifyStats(EnetDma_CbStats *pktStats,
                                    uint32_t pktCnt,
                                    uint32_t timeDiff)
{
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
    uint32_t notifyCount;
    uint32_t timePerPkt = timeDiff / pktCnt;

    if (pktCnt == 0)
    {
        pktStats->zeroNotifyCnt++;
    }

    notifyCount = pktStats->dataNotifyCnt & (ENET_DMA_STATS_HISTORY_CNT - 1U);
    pktStats->dataNotifyCnt++;

    pktStats->totalPktCnt   += pktCnt;
    pktStats->totalCycleCnt += timeDiff;

    pktStats->cycleCntPerNotify[notifyCount] = timeDiff;
    pktStats->pktsPerNotify[notifyCount]     = pktCnt;
    pktStats->cycleCntPerPkt[notifyCount]    = timePerPkt;

    if (notifyCount != 0)
    {
        /* As app submits all packets for Rx during init, we ignore that to get run-time stats */
        if (timeDiff > pktStats->cycleCntPerNotifyMax)
        {
            pktStats->cycleCntPerNotifyMax = timeDiff;
        }

        if (pktCnt > pktStats->pktsPerNotifyMax)
        {
            pktStats->pktsPerNotifyMax = pktCnt;
        }

        if (timePerPkt > pktStats->cycleCntPerPktMax)
        {
            pktStats->cycleCntPerPktMax = timePerPkt;
        }
    }
#endif
}

/*!
 * @}
 */
