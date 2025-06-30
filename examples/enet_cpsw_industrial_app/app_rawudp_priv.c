/*
 *  Copyright (c) 2025 Texas Instruments Incorporated 
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
 * \file  app_rawudp_priv.c
 *
 * \brief This file contains the implementation of packet queue processing for raw UDP channel
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "app_cfg.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

uint32_t RawUdpApp_retrieveFreeTxPkts(EnetDma_TxChHandle hTxCh, EnetDma_PktQ* txFreePktInfoQ);

void RawUdpApp_initTxFreePktQ(EnetApp_ChInfo* hRawUdp);

void RawUdpApp_initRxReadyPktQ(EnetApp_ChInfo* hRawUdp);

int32_t RawUdpApp_receivePkts(EnetApp_ChInfo* hRawUdp, EnetDma_PktQ* rxReadyQ);

int32_t RawUdpApp_transmitPkts(EnetApp_ChInfo* hRawUdp, EnetDma_PktQ* txSubmitQ);

void RawUdpApp_modifyPkts(EnetApp_ChInfo* hRawUdp, EnetDma_PktQ *rxReadyQ, EnetDma_PktQ *txSubmitQ);

void RawUdpApp_modifyPkt(EnetApp_ChInfo* hRawUdp,EnetDma_Pkt* txPktInfo,EnetDma_Pkt* RxPktInfo,int32_t totalLenReceived);

uint32_t RawUdpApp_checkPkt(EnetDma_Pkt* rxPktInfo);

extern bool RawUdpApp_parseFrame(EthFrame *frame, uint32_t dataLen, EnetApp_ChInfo* hRawUdp);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static volatile uint32_t FrameParsePassed = 0;
static volatile uint32_t FrameParseFailed = 0;
extern EnetApp_Obj gEnetApp;

#ifdef ENETAPP_PROFILE_EN /*Enable in app_cfg.h*/
    float averageProfileTime = 0,maxProfileTime = 0;
    uint32_t profileIndex = 0,subTxTime,rxIsrTime;
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

uint32_t RawUdpApp_retrieveFreeTxPkts(EnetDma_TxChHandle hTxCh, EnetDma_PktQ* txFreePktInfoQ)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    uint32_t txFreeQCnt = 0U;
    int32_t status;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any packets that may be free now */
    status = EnetDma_retrieveTxPktQ(hTxCh, &txFreeQ);
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

            EnetQueue_enq(txFreePktInfoQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        }
    }
    else
    {
        EnetAppUtils_print("retrieveFreeTxPkts() failed to retrieve pkts: %d\r\n", status);
    }

    return txFreeQCnt;
}

int32_t RawUdpApp_receivePkts(EnetApp_ChInfo* hRawUdp,EnetDma_PktQ* rxReadyQ){
    int32_t status = ENET_SOK;
    EnetQueue_initQ(rxReadyQ);
    EnetQueue_initQ(&hRawUdp->rxFreePktInfoQ);

    /* Retrieve received packets */
    status = EnetDma_retrieveRxPktQ(hRawUdp->hRxCh, rxReadyQ);

    if (status != ENET_SOK)
    {
        /* Should we bail out here? */
        EnetAppUtils_print("Failed to retrieve RX pkt queue: %d\r\n", status);
        return status;
    }
#if DEBUG
    EnetAppUtils_print("Received %u packets\r\n",  EnetQueue_getQCount(rxReadyQ));
    totalRxCnt += EnetQueue_getQCount(rxReadyQ);
#endif

    return status;
}

int32_t RawUdpApp_transmitPkts(EnetApp_ChInfo* hRawUdp,EnetDma_PktQ* txSubmitQ){

    int32_t status = ENET_SOK;
    uint32_t submittedPktCount = 0;

    submittedPktCount = EnetQueue_getQCount(txSubmitQ);

    /*Submit packet queue for transmission*/
    status = EnetDma_submitTxPktQ(hRawUdp->hTxCh, txSubmitQ);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to submit TX pkt queue: %d\r\n", status);
    }
    else
    {
#ifdef ENETAPP_PROFILE_EN 
        subTxTime = CycleCounterP_getCount32();
        uint32_t cpuMHz = SOC_getSelfCpuClk()/1000000;/*Frequency of SOC CPU in MHz*/
        float profileCurr = (subTxTime - rxIsrTime)/cpuMHz;/* Profile in usec*/
        averageProfileTime = (profileCurr+averageProfileTime*(profileIndex))/(profileIndex+1);
        if(profileCurr > maxProfileTime)
        {
            maxProfileTime = profileCurr;
        }
        profileIndex += submittedPktCount;
#endif
    }

    EnetAppUtils_validatePacketState(&hRawUdp->rxFreePktInfoQ,
                                        ENET_PKTSTATE_APP_WITH_FREEQ,
                                        ENET_PKTSTATE_APP_WITH_DRIVER);

    /* Submit now processed buffers */
    EnetDma_submitRxPktQ(hRawUdp->hRxCh, &hRawUdp->rxFreePktInfoQ);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to submit RX pkt queue: %d\r\n", status);
    }
    return status;
} 

void RawUdpApp_modifyPkts(EnetApp_ChInfo* hRawUdp, EnetDma_PktQ *rxReadyQ, EnetDma_PktQ *txSubmitQ){

    EnetQueue_initQ(txSubmitQ);
    EnetCpdma_PktInfo *rxPktInfo,*txPktInfo;
    uint32_t EthPayloadLen = 0;

/* Consume the received packets and send them back */
    rxPktInfo = (EnetDma_Pkt *)EnetQueue_deq(rxReadyQ);
    
    while (rxPktInfo != NULL)
    {
        uint32_t totalLenReceived = RawUdpApp_checkPkt(rxPktInfo);

        EthPayloadLen = rxPktInfo->sgList.list[0].segmentFilledLen - ENETAPP_ETH_HDR_SIZE - ENETAPP_IPV4_HDR_SIZE - ENETAPP_UDP_HDR_SIZE;
        if(RawUdpApp_parseFrame((EthFrame*)(rxPktInfo->sgList.list[0].bufPtr),EthPayloadLen,hRawUdp))
        {
            FrameParsePassed++;
            /* Retrieve TX packets from driver and recycle them */
            RawUdpApp_retrieveFreeTxPkts(hRawUdp->hTxCh,&hRawUdp->txFreePktInfoQ);

            /* Dequeue one free TX Eth packet */
            txPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&hRawUdp->txFreePktInfoQ);
            if (txPktInfo != NULL)
            {
                /* Fill the TX Eth frame with test content */
                RawUdpApp_modifyPkt(hRawUdp,txPktInfo, rxPktInfo, totalLenReceived);

                /* Enqueue the packet for later transmission */
                EnetQueue_enq(txSubmitQ, &txPktInfo->node);
            }
            else
            {
                EnetAppUtils_print("Drop due to TX pkt not available\r\n");
            }
        }
        else
        {
            FrameParseFailed++;
        }

        EnetDma_checkPktState(&rxPktInfo->pktState,
                                ENET_PKTSTATE_MODULE_APP,
                                ENET_PKTSTATE_APP_WITH_READYQ,
                                ENET_PKTSTATE_APP_WITH_FREEQ);

        /* Release the received packet */
        EnetQueue_enq(&hRawUdp->rxFreePktInfoQ, &rxPktInfo->node);
        rxPktInfo = (EnetDma_Pkt *)EnetQueue_deq(rxReadyQ);
    }

}

uint32_t RawUdpApp_checkPkt(EnetDma_Pkt* rxPktInfo){

    EnetDma_checkPktState(&rxPktInfo->pktState,
                        ENET_PKTSTATE_MODULE_APP,
                        ENET_PKTSTATE_APP_WITH_DRIVER,
                        ENET_PKTSTATE_APP_WITH_READYQ);
    int32_t totalLenReceived = 0;
    for (uint32_t i = 0; i < rxPktInfo->sgList.numScatterSegments; i++)
    {
    totalLenReceived += rxPktInfo->sgList.list[i].segmentFilledLen;
    }
    EnetAppUtils_assert(totalLenReceived <= ENET_MEM_LARGE_POOL_PKT_SIZE);
    return totalLenReceived;
}

void RawUdpApp_modifyPkt(EnetApp_ChInfo* hRawUdp,EnetDma_Pkt* txPktInfo,EnetDma_Pkt* rxPktInfo,int32_t totalLenReceived){

    EthUdpFrame* rxFrame = (EthUdpFrame *)rxPktInfo->sgList.list[0].bufPtr;
    EthUdpFrame* txFrame = (EthUdpFrame *)txPktInfo->sgList.list[0].bufPtr;

    /*Filling the ethernet header of packet to be transmitted*/
    memcpy(txFrame->hdr.dstMac,rxFrame->hdr.srcMac,ENET_MAC_ADDR_LEN);
    memcpy(txFrame->hdr.srcMac,rxFrame->hdr.dstMac,ENET_MAC_ADDR_LEN);
    txFrame->hdr.etherType = rxFrame->hdr.etherType;

    /*Filling the IPv4 header of packet to be transmitted*/
    EthIPv4Header *ipv4TxFrame = &txFrame->ipv4Header;
    EthIPv4Header *ipv4RxFrame = &rxFrame->ipv4Header;
    memcpy(ipv4TxFrame,ipv4RxFrame,sizeof(EthIPv4Header));
    memcpy(&ipv4TxFrame->srcIP, &ipv4RxFrame->dstIP, 4U);
    memcpy(&ipv4TxFrame->dstIP, &ipv4RxFrame->srcIP, 4U);

    /*Filling the Tx frame UDP header of packet to be transmitted*/
    EthUdpLiteHeader *udpTxFrame = (EthUdpLiteHeader *) &txFrame->udpHeader;
    EthUdpLiteHeader *udpRxFrame = (EthUdpLiteHeader *) &rxFrame->udpHeader;
    memcpy(udpTxFrame,udpRxFrame,sizeof(EthUdpLiteHeader));
    memcpy(&udpTxFrame->srcPort, &udpRxFrame->dstPort, 2U);
    memcpy(&udpTxFrame->dstPort, &udpRxFrame->srcPort, 2U);

    txPktInfo->sgList.list[0].segmentFilledLen = ENETAPP_TOTAL_HEADER_SIZE;
    EnetAppUtils_assert(txPktInfo->sgList.list[0].segmentAllocLen >= txPktInfo->sgList.list[0].segmentFilledLen);

    /*Modifying payload*/
    rxFrame->payload[ENETAPP_IPV4_HDR_SIZE+ENETAPP_UDP_HDR_SIZE+10U] = 0xFF;
    rxFrame->payload[txPktInfo->sgList.list[0].segmentFilledLen - 10] = 0xFF;

    /*The txPkt buffer pointer of the next segment is updated to point to the payload of rxPkt*/
    txPktInfo->sgList.list[1].bufPtr= rxPktInfo->sgList.list[0].bufPtr+ENETAPP_TOTAL_HEADER_SIZE;
    txPktInfo->sgList.list[1].segmentFilledLen = totalLenReceived - ENETAPP_TOTAL_HEADER_SIZE;

    txPktInfo->sgList.numScatterSegments = 2;
    txPktInfo->chkSumInfo = 0U;
    txPktInfo->appPriv = hRawUdp;
    txPktInfo->tsInfo.enableHostTxTs = false;

    EnetDma_checkPktState(&txPktInfo->pktState,
                            ENET_PKTSTATE_MODULE_APP,
                            ENET_PKTSTATE_APP_WITH_FREEQ,
                            ENET_PKTSTATE_APP_WITH_DRIVER);

}

void RawUdpApp_initTxFreePktQ(EnetApp_ChInfo* hRawUdp)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    /* Initialize all queues */
    EnetQueue_initQ(&hRawUdp->txFreePktInfoQ);

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
     for (i = 0U; i < (ENET_DMA_TX_CH1_NUM_PKTS); i++)
    {
        pPktInfo = EnetMem_allocEthPkt(hRawUdp,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&hRawUdp->txFreePktInfoQ, &pPktInfo->node);
    }

    EnetAppUtils_print("initQs() txFreePktinfoQ initialized with %d pkts\r\n",
                       EnetQueue_getQCount(&hRawUdp->txFreePktInfoQ));
}

void RawUdpApp_initRxReadyPktQ(EnetApp_ChInfo* hRawUdp)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    int32_t status;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    EnetQueue_initQ(&rxFreeQ);

    for (i = 0U; i < (ENET_DMA_RX_CH1_NUM_PKTS); i++)
    {
        pPktInfo = EnetMem_allocEthPkt(hRawUdp,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);

        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&rxFreeQ, &pPktInfo->node);
    }

    /* Retrieve any packets which are ready */
    EnetQueue_initQ(&rxReadyQ);
    status = EnetDma_retrieveRxPktQ(hRawUdp->hRxCh, &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);

    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&rxFreeQ,
                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                     ENET_PKTSTATE_APP_WITH_DRIVER);

    EnetDma_submitRxPktQ(hRawUdp->hRxCh, &rxFreeQ);

    /* Assert here, as during init, the number of DMA descriptors should be equal to
     * the number of free Ethernet buffers available with app */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxFreeQ) == 0U);
}
