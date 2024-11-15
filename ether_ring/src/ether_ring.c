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
#include <kernel/dpl/SystemP.h>
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/* \brief Size of CB Header */
#define ETHERRING_CB_HEADER_SIZE                                    4U

/* \brief Ethernet Vlan Frame Header size */
#define ETHERRING_VLAN_HEADER_SIZE                                  sizeof(EthVlanFrameHeader)

/* \brief Total size of CB Header and Vlan Header */
#define ETHERRING_PACKET_HDR_PLUS_CBLIKE_HDR_LENGTH                 (ETHERRING_VLAN_HEADER_SIZE + ETHERRING_CB_HEADER_SIZE)

/* \brief Value of Memory blocks count in the memory pool*/
#define ETHERRING_MEMBLOCKS_COUNT                                   128U

/* \brief Memory pool array size */
#define ETHERRING_MEMPOOL_SIZE                                      (ETHERRING_MEMBLOCKS_COUNT * ETHERRING_PACKET_HDR_PLUS_CBLIKE_HDR_LENGTH)

/* \brief Count of Maximum sequence number in the CB packetInfo*/
#define ETHERRING_MAX_SEQUENCE_NUMBER                               255U

/* \brief Lookup table clear task Priority */
#define ETHERRING_LOOKUP_CLEAR_TASK_PRIORITY                        1U

/* \brief Lookup table clear task Polling period */
#define ETHERRING_LOOKUP_CLEAR_TASK_POOL_PERIOD_USEC                8000U

/* \brief Last byte of Host Mac Address Index sent from application */
#define ETHERRING_HOSTMAC_LASTBYTE_INDEX                            20U

/* \brief Sequence Id index in CB Packet */
#define ETHERRING_SEQUENCE_NUMBER_INDEX                             21U

/* \brief Minimum StreamId for ClassA */
#define ETHERRING_MIN_STREAMID_CLASSA                               0U

/* \brief Minimum StreamId for ClassD */
#define ETHERRING_MIN_STREAMID_CLASSD                               3U

/* \brief Index of StreamId in CB Packet */
#define ETHERRING_STREAM_ID_INDEX                                   30U

/* \brief Maximum CPDMA channel count */
#define ETHERRING_MAX_CPDMA_CHANNELS                                8U
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
void EtherRing_initMemPool (EtherRingPool *memPool);

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

EtherRingRxTs_obj gEtherRingRxTs =
{
        .etherRingRxClassATsIndex = 0,
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

void EtherRing_attachtxDmaHandle(void *hEtherRing,
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
        pRingHandle->prevSequenceNumber++;
        EtherRing_addCBLikeHeader(pktInfo, pRingHandle->prevSequenceNumber);
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

        EtherRing_removeCBLikeHeader(pktInfo);
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
        /* After retrieving RX packet the bufPtr point is moved by size of CB Header(4bytes)
         * to remove the CB Header before giving the queue to application. The bufPtr is
         * updated back while submitting the pktInfo to Hardware(CPDMA) */
        pktInfo->sgList.list[0].bufPtr -= ETHERRING_CB_HEADER_SIZE;
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
    uint8_t streamId;
    uint16_t lookupIndex;
    uint8_t* currTimeStampPtr;
    uint64_t currTimeStampValue;

    EtherRing_pktQ rxRetrieveQ;
    EtherRing_pktQ rxDupPktQ;
    EnetQueue_initQ(&rxRetrieveQ);
    EnetQueue_initQ(&rxDupPktQ);

    retVal = EnetDma_retrieveRxPktQ(pRingHandle->hRxCh, &rxRetrieveQ);

    pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&rxRetrieveQ);
    while (pktInfo != NULL)
    {
        /* look-up process for only CB packets */
        if (pktInfo->sgList.list[0].bufPtr[ETHERRING_VLAN_HEADER_SIZE + 1] == 0xC1
                & pktInfo->sgList.list[0].bufPtr[ETHERRING_VLAN_HEADER_SIZE] == 0xF1)
        {
            lastByteMac = pktInfo->sgList.list[0].bufPtr[ETHERRING_HOSTMAC_LASTBYTE_INDEX];
            seqNumber = pktInfo->sgList.list[0].bufPtr[ETHERRING_SEQUENCE_NUMBER_INDEX];

            lookupIndex = (uint16_t) (((uint16_t) lastByteMac << 8) | seqNumber);
            if (pRingHandle->etherRingStats.etherRingSeqLookUp[lookupIndex] == 0)
            {
                /* capturing the rx timestamps and currentTimeStamp for retrieved CB packets */
                if (pktInfo->tsInfo.rxPktTs)
                {
                    streamId = pktInfo->sgList.list[0].bufPtr[ETHERRING_STREAM_ID_INDEX];

                    pRingHandle->etherRingStats.etherRingClassRxCount[streamId]++;
                    if ((gEtherRingRxTs.etherRingRxClassATsIndex
                            < ETHERRING_MAX_RX_TIMESTAMPS_STORED) && (streamId == ETHERRING_MIN_STREAMID_CLASSA))
                    {
                        /* Storing the rxTs for current packet*/
                        gEtherRingRxTs.etherRingTimeStampsRx[gEtherRingRxTs.etherRingRxClassATsIndex] =
                                pktInfo->tsInfo.rxPktTs;

                        currTimeStampPtr = pktInfo->sgList.list[0].bufPtr + ETHERRING_PACKET_HDR_PLUS_CBLIKE_HDR_LENGTH;
                        currTimeStampValue = *(uint64_t*)currTimeStampPtr;

                        /* Storing the current timestamp received with CB packet*/
                        gEtherRingRxTs.etherRingCurrentTimeStamps[gEtherRingRxTs.etherRingRxClassATsIndex] = currTimeStampValue;

                        gEtherRingRxTs.etherRingRxClassATsIndex++;
                    }
                }

                /* remove the CB header and updating the bufPtr before giving to application */
                memmove(pktInfo->sgList.list[0].bufPtr + ETHERRING_CB_HEADER_SIZE,
                        pktInfo->sgList.list[0].bufPtr, ETHERRING_VLAN_HEADER_SIZE);
                pktInfo->sgList.list[0].bufPtr += ETHERRING_CB_HEADER_SIZE;

                pRingHandle->etherRingStats.etherRingSeqLookUp[lookupIndex]++;
                pRingHandle->etherRingStats.etherRingNonDuplicatedPktCount++;
                pRingHandle->etherRingStats.etherRingDuplicatedRxPacketCount++;
                EnetQueue_enq(pRetrieveQ, &pktInfo->node);
            }
            else if (pRingHandle->etherRingStats.etherRingSeqLookUp[lookupIndex] == 1)
            {
                /* Submitting the duplicate CB packets back to the Hardware(CPDMA) */
                pRingHandle->etherRingStats.etherRingSeqLookUp[lookupIndex] = 0;
                pRingHandle->etherRingStats.etherRingDuplicatedRxPacketCount++;
                EnetQueue_enq(&rxDupPktQ, &pktInfo->node);
                EtherRing_submitRxPktQ(pRingHandle, &rxDupPktQ);
            }
            else
            {
                EnetQueue_enq(pRetrieveQ, &pktInfo->node);
            }
        }
        else
        {
            /* Submitting the non-CB packets back to the Hardware(CPDMA) */
            EnetQueue_enq(&rxDupPktQ, &pktInfo->node);
            EtherRing_submitRxPktQ(pRingHandle, &rxDupPktQ);
        }
        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&rxRetrieveQ);
    }

    return retVal;
}

void EtherRing_initMemPool (EtherRingPool *memPool)
{
    uint32_t memPoolIndex = 0;

    Enet_assert(memPool != NULL);
    if (memPool->etherRingIsMemPoolInitialised == false)
    {
        EnetQueue_initQ(&gEtherRingPool.etherRingFreeQueue);

        for (memPoolIndex = 0; memPoolIndex < ETHERRING_MEMBLOCKS_COUNT; memPoolIndex++)
        {
            uint8_t* pMemBlock =  &gEtherRing_MemPool[memPoolIndex*ETHERRING_PACKET_HDR_PLUS_CBLIKE_HDR_LENGTH];
            EnetQueue_enq(&gEtherRingPool.etherRingFreeQueue, (EnetQ_Node*)pMemBlock);
        }

        Enet_assert(EnetQueue_getQCount(&gEtherRingPool.etherRingFreeQueue) == ETHERRING_MEMBLOCKS_COUNT);
        gEtherRingPool.etherRingIsMemPoolInitialised = true;
    }
}

void EtherRing_addCBLikeHeader(EnetDma_Pkt *pktInfo,
                             uint16_t seqNumber)
{
    uint8_t *headerWithCB = NULL;

    if (EnetQueue_getQCount(&gEtherRingPool.etherRingFreeQueue) > 0)
    {
        headerWithCB = (uint8_t*)EnetQueue_deq(&gEtherRingPool.etherRingFreeQueue);
    }

    Enet_assert(headerWithCB != NULL);
    Enet_assert(pktInfo != NULL);

    pktInfo->sgList.list[1] = pktInfo->sgList.list[0];
    pktInfo->sgList.list[1].bufPtr += ETHERRING_VLAN_HEADER_SIZE;
    pktInfo->sgList.list[1].segmentFilledLen -= ETHERRING_VLAN_HEADER_SIZE;
    pktInfo->sgList.list[0].segmentFilledLen = ETHERRING_PACKET_HDR_PLUS_CBLIKE_HDR_LENGTH;

    memcpy(headerWithCB, pktInfo->sgList.list[0].bufPtr, ETHERRING_VLAN_HEADER_SIZE);

    /* F1-C1 EtherType for CB */
    headerWithCB[ETHERRING_VLAN_HEADER_SIZE + 1] = 0xC1;
    headerWithCB[ETHERRING_VLAN_HEADER_SIZE] = 0xF1;

    /* Adding seq_number to the Header */
    headerWithCB[ETHERRING_VLAN_HEADER_SIZE + 3] = seqNumber & 0xFF;

    /* Last byte of host macAddr */
    headerWithCB[ETHERRING_VLAN_HEADER_SIZE + 2] = gEtherRingCfg->hostMacAddLastByte;

    pktInfo->sgList.list[0].bufPtr = headerWithCB;

    pktInfo->sgList.numScatterSegments = 2;
}

void EtherRing_removeCBLikeHeader(EnetDma_Pkt *pktInfo)
{
    Enet_assert(pktInfo != NULL);

    EnetQueue_enq(&gEtherRingPool.etherRingFreeQueue, (EnetQ_Node*)pktInfo->sgList.list[0].bufPtr);
    pktInfo->sgList.list[0].bufPtr = pktInfo->sgList.list[1].bufPtr - ETHERRING_VLAN_HEADER_SIZE;
    pktInfo->sgList.numScatterSegments = 1;
}

void EtherRing_periodicTick(void *hEtherRing)
{
    /* This API needs to be called from application with periodicity of 1ms */
    Enet_assert(hEtherRing != NULL);

    EtherRing_Handle pRingHandle = (EtherRing_Handle) hEtherRing;
    EtherRingStats etherRingStats = pRingHandle->etherRingStats;

    memset(etherRingStats.etherRingSeqLookUp, 0, ETHERRING_LOOKUP_TABLE_SIZE);
}
