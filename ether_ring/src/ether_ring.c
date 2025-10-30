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

/*!
 * \file  ether_ring.c
 *
 * \brief This file contains the implementation of the Ether-ring Driver.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "ether_ring.h"
#include <enet_utils.h>
#include <enet_ethutils.h>
#include <core/enet_dma.h>
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/*! \brief Maximum Instances of EtherRing supported */
#define ETHERRING_MAX_ETHERRING_INSTANCES                           (1U)

/* \brief Ethernet Vlan Frame Header size */
#define ETHERRING_UNTAGGED_VLAN_ETH_HEADER_SIZE                     (sizeof(EthFrameHeader))

/* \brief Ethernet Vlan Frame Header size */
#define ETHERRING_VLAN_HEADER_SIZE                                  (sizeof(EthVlanFrameHeader))

/* \brief Total size of Ether-Ring Header and Vlan Header */
#define ETHERRING_PACKET_HDR_PLUS_ETHERRING_HDR_LENGTH              (ETHERRING_VLAN_HEADER_SIZE + ETHERRING_HEADER_SIZE)

/* \brief Value of Memory blocks count in the memory pool*/
#define ETHERRING_MEMBLOCKS_COUNT                                   (128U)

/* \brief Memory pool array size */
#define ETHERRING_MEMPOOL_SIZE                                      (ETHERRING_MEMBLOCKS_COUNT * ETHERRING_PACKET_HDR_PLUS_ETHERRING_HDR_LENGTH)

/* \brief Count of Maximum sequence number in the Etherring packetInfo*/
#define ETHERRING_MAX_SEQUENCE_NUMBER                               (255U)

/* \brief Last byte of Host Mac Address Index sent from application */
#define ETHERRING_HOSTMAC_LASTBYTE_INDEX                            (20U)

/* \brief Sequence Id index in Etherring Packet */
#define ETHERRING_SEQUENCE_NUMBER_INDEX                             (21U)

/* \brief Maximum CPDMA channel count */
#define ETHERRING_MAX_CPDMA_CHANNELS                                (8U)

/* \brief Ethertype Index in Etherring  Packet */
#define ETHERRING_VLAN_TPID_INDEX                                   (12U)

/* \brief Ethertype in Etherring Header*/
#define ETHERRING_ETHERTYPE_IN_ETHERRING_HEADER                     (0x88B5)
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
typedef struct
{
    EnetQ etherRingFreeQueue;
    bool etherRingIsMemPoolInitialised;
} EtherRingPool;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static void EtherRing_initMemPool(EtherRingPool *memPool);
static void EtherRing_addEtherringHeader(EnetDma_Pkt *pktInfo,
                                         uint16_t seqNumber);
static void EtherRing_removeEtherringHeader(EnetDma_Pkt *pktInfo);
static void EtherRing_calculateEthHeaderSize(EnetDma_Pkt *pktInfo, uint32_t *ethHeaderSize);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
EtherRing_Obj EtherRing_ObjectList[ETHERRING_MAX_ETHERRING_INSTANCES] =
{
    [0] =
    {
        .isAllocated = false,
        .prevSequenceNumber = 0,
        .etherRingStats.etherRingNonDuplicatedPktCount = 0,
        .etherRingStats.etherRingDuplicatedRxPacketCount = 0,
    },
};

static EtherRingPool gEtherRingPool = {
        .etherRingIsMemPoolInitialised = false,
};

static EtherRing_Cfg *gEtherRingCfg;

static uint8_t gEtherRing_MemPool[ETHERRING_MEMPOOL_SIZE];
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

EtherRing_Handle EtherRing_open(Enet_Handle hEnet,
                                uint32_t appCoreId,
                                const void *pEtherRingCfg)
{
    int32_t etherRingIndex;
    EtherRing_Handle hEtherRing = NULL;

    Enet_assert(hEnet != NULL);
    Enet_assert(pEtherRingCfg != NULL);

    gEtherRingCfg = (EtherRing_Cfg*) pEtherRingCfg;
    gEtherRingCfg->isCfg = true;

    EtherRing_initMemPool(&gEtherRingPool);

    for (etherRingIndex = 0U; etherRingIndex < ETHERRING_MAX_ETHERRING_INSTANCES;
            etherRingIndex++)
    {
        if (EtherRing_ObjectList[etherRingIndex].isAllocated == false)
        {
            hEtherRing = &EtherRing_ObjectList[etherRingIndex];
            hEtherRing->hEnet = hEnet;
            EtherRing_ObjectList[etherRingIndex].isAllocated = true;
            break;
        }
    }

    return hEtherRing;
}

void EtherRing_close(void *hEtherRing)
{
    Enet_assert(hEtherRing != NULL);

    EtherRing_Handle pRingHandle = (EtherRing_Handle) hEtherRing;

    if(pRingHandle->isAllocated == true)
    {
        pRingHandle->isAllocated = false;
        pRingHandle->hEnet = NULL;
        pRingHandle->hTxCh = NULL;
        pRingHandle->hRxCh = NULL;
    }
}

void EtherRing_attachTxDmaHandle(void *hEtherRing,
                                EnetDma_TxChHandle hTxCh,
                                int32_t txChNum)
{
    Enet_assert(hEtherRing != NULL);

    EtherRing_Handle pRingHandle = (EtherRing_Handle) hEtherRing;
    pRingHandle->hTxCh = hTxCh;

    Enet_assert(txChNum < ETHERRING_MAX_CPDMA_CHANNELS);
    pRingHandle->txChNum = txChNum;
}

void EtherRing_attachRxDmaHandle(void *hEtherRing,
                                EnetDma_RxChHandle hRxCh,
                                int32_t rxChNum)
{
    Enet_assert(hEtherRing != NULL);

    EtherRing_Handle pRingHandle = (EtherRing_Handle) hEtherRing;
    pRingHandle->hRxCh = hRxCh;

    Enet_assert(rxChNum < ETHERRING_MAX_CPDMA_CHANNELS);
    pRingHandle->rxChNum = rxChNum;
}

int32_t EtherRing_submitTxPktQ(void *hEtherRing,
                               EtherRing_pktQ *pSubmitQ)
{
    int32_t retVal = ENET_SOK;
    Enet_assert(hEtherRing != NULL);

    EtherRing_pktQ txSubmitQ;
    EtherRing_Handle pRingHandle = (EtherRing_Handle) hEtherRing;
    EnetDma_Pkt *pktInfo = NULL;

    EnetQueue_initQ(&txSubmitQ);

    pktInfo = (EnetDma_Pkt*) EnetQueue_deq(pSubmitQ);

    while (pktInfo != NULL)
    {
        /*  Ether-Ring only handles tx pktInfo with 1 scatterSegment */
        Enet_assert(pktInfo->sgList.numScatterSegments == 1);

        pRingHandle->prevSequenceNumber++;
        EtherRing_addEtherringHeader(pktInfo, pRingHandle->prevSequenceNumber);
        if (pRingHandle->prevSequenceNumber >= ETHERRING_MAX_SEQUENCE_NUMBER)
        {
            pRingHandle->prevSequenceNumber = 0;
        }
        EnetQueue_enq(&txSubmitQ, &pktInfo->node);
        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(pSubmitQ);
    }
    retVal = EnetDma_submitTxPktQ(pRingHandle->hTxCh,
                                  &txSubmitQ);
    return retVal;
}

int32_t EtherRing_retrieveTxPktQ(void *hEtherRing,
                                 EtherRing_pktQ *pRetrieveQ)
{
    int32_t retVal = ENET_SOK;
    Enet_assert(hEtherRing != NULL);

    EtherRing_Handle pRingHandle = (EtherRing_Handle) hEtherRing;
    EnetDma_Pkt *pktInfo = NULL;

    EtherRing_pktQ retrieveQ;
    EnetQueue_initQ(&retrieveQ);


    retVal = EnetDma_retrieveTxPktQ(pRingHandle->hTxCh,
                                    &retrieveQ);
    while(EnetQueue_getQCount(&retrieveQ))
    {
        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&retrieveQ);

        EtherRing_removeEtherringHeader(pktInfo);
        EnetQueue_enq(pRetrieveQ, &pktInfo->node);
    }
    return retVal;
}

int32_t EtherRing_submitRxPktQ(void *hEtherRing,
                               EtherRing_pktQ *pSubmitQ)
{
    int32_t retVal = ENET_SOK;
    Enet_assert(hEtherRing != NULL);

    EtherRing_Handle pRingHandle = (EtherRing_Handle) hEtherRing;
    EtherRing_pktQ rxSubmitQ;
    EnetDma_Pkt *pktInfo = NULL;

    EnetQueue_initQ(&rxSubmitQ);

    pktInfo = (EnetDma_Pkt*) EnetQueue_deq(pSubmitQ);

    while (pktInfo != NULL)
    {
        /* After retrieving RX packet the bufPtr point is moved by size of EtherRing Header(4bytes)
         * to remove the EtherRing Header before giving the queue to application. The bufPtr is
         * updated back while submitting the pktInfo to Hardware(CPDMA) */
        pktInfo->sgList.list[0].bufPtr -= ETHERRING_HEADER_SIZE;
        EnetQueue_enq(&rxSubmitQ, &pktInfo->node);
        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(pSubmitQ);
    }
    retVal = EnetDma_submitRxPktQ(pRingHandle->hRxCh, &rxSubmitQ);

    return retVal;
}

int32_t EtherRing_retrieveRxPktQ(void *hEtherRing,
                                 EtherRing_pktQ *pRetrieveQ)
{
    int32_t retVal = ENET_SOK;
    Enet_assert(hEtherRing != NULL);

    EtherRing_Handle pRingHandle = (EtherRing_Handle) hEtherRing;
    EnetDma_Pkt *pktInfo = NULL;
    uint8_t lastByteMac;
    uint8_t seqNumber;
    uint16_t lookupIndex;
    uint32_t ethHeaderSize = 0U;
    uint16_t etherringEtherType = 0U;

    EtherRing_pktQ rxRetrieveQ;
    EtherRing_pktQ rxDupPktQ;
    EnetQueue_initQ(&rxRetrieveQ);
    EnetQueue_initQ(&rxDupPktQ);

    retVal = EnetDma_retrieveRxPktQ(pRingHandle->hRxCh, &rxRetrieveQ);

    pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&rxRetrieveQ);
    while (pktInfo != NULL)
    {
        EtherRing_calculateEthHeaderSize(pktInfo, &ethHeaderSize);
        memcpy(&etherringEtherType, &(pktInfo->sgList.list[0].bufPtr[ethHeaderSize]), sizeof(uint16_t));

        /* look-up process for only EtherRing packets */
        if (etherringEtherType == Enet_htons(ETHERRING_ETHERTYPE_IN_ETHERRING_HEADER))
        {
            lastByteMac = pktInfo->sgList.list[0].bufPtr[ETHERRING_HOSTMAC_LASTBYTE_INDEX];
            seqNumber = pktInfo->sgList.list[0].bufPtr[ETHERRING_SEQUENCE_NUMBER_INDEX];

            lookupIndex = (uint16_t) (((uint16_t) lastByteMac << 8) | seqNumber);
            if (pRingHandle->etherRingStats.etherRingSeqLookUp[lookupIndex] == 0)
            {
                /* remove the EtherRing header and updating the bufPtr before giving to application */
                memmove(pktInfo->sgList.list[0].bufPtr + ETHERRING_HEADER_SIZE,
                        pktInfo->sgList.list[0].bufPtr, ethHeaderSize);
                pktInfo->sgList.list[0].bufPtr += ETHERRING_HEADER_SIZE;

                pRingHandle->etherRingStats.etherRingSeqLookUp[lookupIndex]++;
                pRingHandle->etherRingStats.etherRingNonDuplicatedPktCount++;
                pRingHandle->etherRingStats.etherRingDuplicatedRxPacketCount++;
                EnetQueue_enq(pRetrieveQ, &pktInfo->node);
            }
            else if (pRingHandle->etherRingStats.etherRingSeqLookUp[lookupIndex] == 1)
            {
                /* Submitting the duplicate EtherRing packets back to the Hardware(CPDMA) */
                pRingHandle->etherRingStats.etherRingSeqLookUp[lookupIndex] = 0;
                pRingHandle->etherRingStats.etherRingDuplicatedRxPacketCount++;
                EnetQueue_enq(&rxDupPktQ, &pktInfo->node);
                EnetDma_submitRxPktQ(pRingHandle->hRxCh, &rxDupPktQ);
            }
            else
            {
                EnetQueue_enq(pRetrieveQ, &pktInfo->node);
            }
        }
        else
        {
            /* Submitting the non-Etherring packets back to the Hardware(CPDMA) */
            EnetQueue_enq(&rxDupPktQ, &pktInfo->node);
            EnetDma_submitRxPktQ(pRingHandle->hRxCh, &rxDupPktQ);
        }
        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&rxRetrieveQ);
    }

    return retVal;
}

static void EtherRing_initMemPool(EtherRingPool *memPool)
{
    uint32_t memPoolIndex = 0;

    Enet_assert(memPool != NULL);
    if (memPool->etherRingIsMemPoolInitialised == false)
    {
        EnetQueue_initQ(&gEtherRingPool.etherRingFreeQueue);

        for (memPoolIndex = 0; memPoolIndex < ETHERRING_MEMBLOCKS_COUNT; memPoolIndex++)
        {
            uint8_t* pMemBlock =  &gEtherRing_MemPool[memPoolIndex * ETHERRING_PACKET_HDR_PLUS_ETHERRING_HDR_LENGTH];
            EnetQueue_enq(&gEtherRingPool.etherRingFreeQueue, (EnetQ_Node*)pMemBlock);
        }

        Enet_assert(EnetQueue_getQCount(&gEtherRingPool.etherRingFreeQueue) == ETHERRING_MEMBLOCKS_COUNT);
        gEtherRingPool.etherRingIsMemPoolInitialised = true;
    }
}

static void EtherRing_addEtherringHeader(EnetDma_Pkt *pktInfo,
                                         uint16_t seqNumber)
{
    Enet_assert(pktInfo != NULL);

    uint8_t *etherRingHeader = NULL;
    uint32_t ethHeaderSize = 0U;
    /*  Vlan and Non-Vlan Packets needs different handling due to
    * variation in start index of Payload */
    EtherRing_calculateEthHeaderSize(pktInfo, &ethHeaderSize);

    if (EnetQueue_getQCount(&gEtherRingPool.etherRingFreeQueue) > 0)
    {
        etherRingHeader = (uint8_t*)EnetQueue_deq(&gEtherRingPool.etherRingFreeQueue);
    }

    Enet_assert(etherRingHeader != NULL);

    pktInfo->sgList.list[1] = pktInfo->sgList.list[0];
    pktInfo->sgList.list[1].bufPtr += ethHeaderSize;
    pktInfo->sgList.list[1].segmentFilledLen -= ethHeaderSize;
    pktInfo->sgList.list[0].segmentFilledLen = (ethHeaderSize + ETHERRING_HEADER_SIZE);

    memcpy(etherRingHeader, pktInfo->sgList.list[0].bufPtr, ethHeaderSize);

    /* 88-B5 EtherType in Ether-Ring Header */
    *(uint16_t*)((uint8_t*)&etherRingHeader[ethHeaderSize]) = Enet_htons(ETHERRING_ETHERTYPE_IN_ETHERRING_HEADER);

    /* Adding seq_number to the Header */
    etherRingHeader[ethHeaderSize + 3] = seqNumber & 0xFF;

    /* Last byte of host macAddr */
    etherRingHeader[ethHeaderSize + 2] = gEtherRingCfg->hostMacAddLastByte;

    pktInfo->sgList.list[0].bufPtr = etherRingHeader;

    pktInfo->sgList.numScatterSegments = 2;
}

static void EtherRing_calculateEthHeaderSize(EnetDma_Pkt *pktInfo, uint32_t *ethHeaderSize)
{

    /* Based on whether packet contains vlanTag, ethernet header size is calculated */
    if ((uint16_t)(pktInfo->sgList.list[0].bufPtr[ETHERRING_VLAN_TPID_INDEX]) == Enet_htons(0x8100))
    {
        *ethHeaderSize = ETHERRING_VLAN_HEADER_SIZE;
    }
    else
    {
        *ethHeaderSize = ETHERRING_UNTAGGED_VLAN_ETH_HEADER_SIZE;
    }
}

static void EtherRing_removeEtherringHeader(EnetDma_Pkt *pktInfo)
{
    Enet_assert(pktInfo != NULL);
    uint32_t ethHeaderSize = 0U;

    EtherRing_calculateEthHeaderSize(pktInfo, &ethHeaderSize);

    EnetQueue_enq(&gEtherRingPool.etherRingFreeQueue, (EnetQ_Node*)pktInfo->sgList.list[0].bufPtr);
    pktInfo->sgList.list[0].bufPtr = pktInfo->sgList.list[1].bufPtr - ethHeaderSize;
    pktInfo->sgList.numScatterSegments = 1;
}

void EtherRing_periodicTick(void *hEtherRing)
{
    /* This API needs to be called from application with periodicity of 1ms */
    Enet_assert(hEtherRing != NULL);

    EtherRing_Handle pRingHandle = (EtherRing_Handle) hEtherRing;
    EtherRingStats* etherRingStats = &pRingHandle->etherRingStats;

    memset(etherRingStats->etherRingSeqLookUp, 0, ETHERRING_LOOKUP_TABLE_SIZE);
}
