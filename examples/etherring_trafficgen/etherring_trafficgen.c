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

/*!
 * \file  etherring_flow_cpsw.c
 *
 * \brief This file contains the implementation of the Ether-Ring Redundancy
 *           Traffic Generator and Rx Task
 */
/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include <enet_appmemutils.h>
#include "cpsw_config/etherring_cpsw_config.h"
#include "etherring_trafficgen_config.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/* Task stack size */
#define ENETAPP_TASK_STACK_SZ                               (5U * 1024U)

/* Total packet sent from each Node */
#define ENETAPP_TOTAL_TX_PACKET_COUNT                       (ENETAPP_NUM_CLASSA_STREAMS * ENETAPP_PACKETS_SENT_PER_STREAM)

/* Index of Timestamp in the Tx packet payload */
#define ENETAPP_TX_PKT_TIMESTAMP_PAYLOAD_INDEX               0U

/* Index of Timestamp sequenceNumber in the Tx packet payload */
#define ENETAPP_TX_PKT_TIMESTAMP_SEQNUM_PAYLOAD_INDEX        4U

/* Index of source NodeId in the Tx packet payload */
#define ENETAPP_TX_PKT_SRC_NODEID_INDEX                      8U

/* Index of Profiling Magic Number in the payload */
#define ENETAPP_TX_PKT_PROFILING_INDEX                       9U

/* Index of Actual payload start */
#define ENETAPP_TX_PKT_ACTUAL_PAYLOAD_START_INDEX            10U
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                              Global Variables                              */
/* ========================================================================== */
EnetApp_Cfg gEnetAppCfg =
{
    .txCount = 0,
};

static EnetApp_LatencyProfile gLatencyProfile =
{
        .maxLatency = 0,
        .maxLatencyRT = 0,
        .avgLatency = 0,
        .sumLatency = 0,
        .tsSeqNumber = 0,
};

/* EtherType and Vlan for Redundancy packets */
const uint16_t gTxEtherType = 0x88B5U;
const uint16_t gTxVlanTciClassA = Enet_htons(ENETAPP_VLAN_TCI(3, 0, 255));
const uint16_t gEthVlanHdrSize = sizeof(EthVlanFrameHeader);

/* Stack for the Ether-Ring Tx,Rx Tasks */
static uint8_t gEnetAppStreamTaskStack[ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));
static uint8_t gEnetAppEtherRingTaskStack[ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));
static uint8_t gEnetAppTaskStackRx[ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));

EtherRing_Cfg gEtherRingCfg;

static volatile uint32_t isStreamsEnabled = 0;

extern EnetApp_TrafficGen_Obj TrafficGen_ClassAList[ENETAPP_MAX_CLASSA_STREAMS];
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void EnetApp_sendEchoPacket(EnetDma_Pkt *rxPktInfo, uint8_t destNodeId);
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


/* Rx task for Ether-Ring Redundancy traffic */
static void EnetApp_rxTrafficHandler(void *args)
{
    int32_t status = ENET_SOK;
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxSubmitQ;
    EnetDma_Pkt *pktInfo = NULL;

    EnetAppUtils_print("Rx Traffic Handler started\r\n");

    while ((ENET_SOK == status))
    {
        /* Wait for packet reception */
        SemaphoreP_pend(&gEnetAppCfg.rxSemObj, SystemP_WAIT_FOREVER);

        /* Retrieve Packets from Ether-Ring Rx Channel */
        EnetQueue_initQ(&rxReadyQ);
        EnetQueue_initQ(&rxSubmitQ);

        status = EtherRing_retrieveRxPktQ(gEnetAppCfg.hEtherRing, &rxReadyQ);

        pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&rxReadyQ);
        uint32_t rxSeqNumber = 0U;
        uint8_t srcNodeId = 0U;

        while(pktInfo)
        {
            /* Store the Timestamp of Rx Packet If Device Node is 0 and If Profiling is Enabled */
            if(gEnetAppCfg.nodeId == 0 &&
               pktInfo->sgList.list[0].bufPtr[sizeof(EthVlanFrameHeader) + ENETAPP_TX_PKT_SRC_NODEID_INDEX] == 0U &&
               pktInfo->sgList.list[0].bufPtr[sizeof(EthVlanFrameHeader) + ENETAPP_TX_PKT_PROFILING_INDEX] == 0xF )
            {
                memcpy(&rxSeqNumber, &pktInfo->sgList.list[0].bufPtr[sizeof(EthVlanFrameHeader)
                                      + ENETAPP_TX_PKT_TIMESTAMP_SEQNUM_PAYLOAD_INDEX], 4U);
                if(rxSeqNumber >= 0 && rxSeqNumber < ENETAPP_PACKETS_SENT_PER_STREAM)
                {
                    gLatencyProfile.rxTsArray[rxSeqNumber] = CycleCounterP_getCount32()/400;

                    /* Calculate the Time Difference of RxPacket and TxPacket for Round-Trip Latency */
                    gLatencyProfile.sumLatency = gLatencyProfile.sumLatency +
                                                (gLatencyProfile.rxTsArray[rxSeqNumber] - gLatencyProfile.sentTimeArray[rxSeqNumber]);

                    /* Update the Average Latency */
                    gLatencyProfile.avgLatency = (gLatencyProfile.sumLatency/2)/(rxSeqNumber);

                    if (gLatencyProfile.rxTsArray[rxSeqNumber] - gLatencyProfile.sentTimeArray[rxSeqNumber]
                                                               > gLatencyProfile.maxLatencyRT)
                    {
                        /* Update the Maximum Round_trip Latency */
                        gLatencyProfile.maxLatencyRT = gLatencyProfile.rxTsArray[rxSeqNumber]- gLatencyProfile.sentTimeArray[rxSeqNumber];

                        /* Update the Maximum Latency */
                        gLatencyProfile.maxLatency = gLatencyProfile.maxLatencyRT/2;

                    }
                    if (rxSeqNumber == ENETAPP_PACKETS_SENT_PER_STREAM - 1)
                    {
                        EnetAppUtils_print("\r\n********Ether-Ring Demonstration Completed********\r\n");
                        EnetAppUtils_print("Average Latency: %lu us\r\n", gLatencyProfile.avgLatency);
                        EnetAppUtils_print("Maximum Latency: %lu us\r\n", gLatencyProfile.maxLatency);
                    }
                }
            }
            /* Echo the packet If DeviceNodeId is "NodeCount/2" and SrcNodeId in packet is "0" when profiling is enabled */
            else if(gEnetAppCfg.nodeId == ENETAPP_MAX_NODES_IN_RING/2 &&
                    pktInfo->sgList.list[0].bufPtr[sizeof(EthVlanFrameHeader) + ENETAPP_TX_PKT_SRC_NODEID_INDEX] == 0U &&
                    pktInfo->sgList.list[0].bufPtr[sizeof(EthVlanFrameHeader) + ENETAPP_TX_PKT_PROFILING_INDEX] == 0xF )
            {
                memcpy(&srcNodeId, &pktInfo->sgList.list[0].bufPtr[sizeof(EthVlanFrameHeader)
                                      + ENETAPP_TX_PKT_SRC_NODEID_INDEX], 1U);

                /* creates a packet and copies the payload and echo it back to the srcNodeID */
                EnetApp_sendEchoPacket(pktInfo, srcNodeId);
            }

            if (gEnetAppCfg.totalRxCnt < UINT64_MAX)
            {
                gEnetAppCfg.totalRxCnt ++;
            }
            else
            {
                gEnetAppCfg.totalRxCnt = 0;
            }

            EnetQueue_enq(&rxSubmitQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt*) EnetQueue_deq(&rxReadyQ);
        }

        if (status != ENET_SOK)
        {
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

static void EnetApp_sendRedundancyPacket(EtherRing_Handle hEtherRing, uint32_t destinationNodeId, bool isProfilingEnabled)
{
    Enet_assert(hEtherRing != NULL);

    EnetDma_PktQ txSubmitQ;
    EthVlanFrame *frame;
    uint32_t tsValCurrentvalue = 0ULL;
    uint8_t multicastAddr[ENET_MAC_ADDR_LEN] = {0x01,0x00,0x5E,0x7F,0xFF,destinationNodeId};
    EnetDma_PktQ txFreeQ;
    uint32_t key;
    int32_t retVal = ENET_SOK;

    EnetQueue_initQ(&txFreeQ);
    key = EnetOsal_disableAllIntr();
    tsValCurrentvalue = 0ULL;

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
             frame->hdr.tci  = gTxVlanTciClassA;
             frame->hdr.etherType = gTxEtherType;

             pktInfo->sgList.list[0].segmentFilledLen = ENETAPP_CLASSA_PAYLOAD_LENGTH + gEthVlanHdrSize;
             EnetAppUtils_assert(ENETAPP_CLASSA_PAYLOAD_LENGTH >= 9U);

             /* Payload[0:3] - timestamp of packet when it is created
              * Payload[4:7] - Sequence number of the tx packet
              * Payload[8]   - Source NodeId
              * Payload[9]   - 0xF when profiling enabled or 0x0 when profiling is disabled
              * Payload[10:] - Rest of the payload */

             /* If profiling is enabled, then store the current timestamp in sentTimeArray and
              *   store the tsSeqNumber and txTimeStamp in the packet payload */
             if(isProfilingEnabled && (gLatencyProfile.tsSeqNumber < ENETAPP_PACKETS_SENT_PER_STREAM))
             {
                 tsValCurrentvalue = CycleCounterP_getCount32()/400;
                 memcpy(&frame->payload[ENETAPP_TX_PKT_TIMESTAMP_PAYLOAD_INDEX], &tsValCurrentvalue, 4U);
                 memcpy(&frame->payload[ENETAPP_TX_PKT_TIMESTAMP_SEQNUM_PAYLOAD_INDEX], &gLatencyProfile.tsSeqNumber, 4U);
                 gLatencyProfile.sentTimeArray[gLatencyProfile.tsSeqNumber++] = tsValCurrentvalue;
                 memset(&frame->payload[ENETAPP_TX_PKT_PROFILING_INDEX], 0xF, 1);
             }
             else
             {
                 memset(&frame->payload[ENETAPP_TX_PKT_TIMESTAMP_PAYLOAD_INDEX], 0x0, 8U);
                 memset(&frame->payload[ENETAPP_TX_PKT_PROFILING_INDEX], 0x0, 1);
             }

             frame->payload[ENETAPP_TX_PKT_SRC_NODEID_INDEX] = (uint8_t)(gEnetAppCfg.nodeId);
             memset(&frame->payload[ENETAPP_TX_PKT_ACTUAL_PAYLOAD_START_INDEX], (uint8_t)(0xA5 +
                     EnetQueue_getQCount(&gEnetAppCfg.txFreePktInfoQ)), (ENETAPP_CLASSA_PAYLOAD_LENGTH - ENETAPP_TX_PKT_ACTUAL_PAYLOAD_START_INDEX));

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
             EnetAppUtils_assert(retVal == ENET_SOK);

             gEnetAppCfg.txCount++;

             retVal = EtherRing_retrieveTxPktQ(gEnetAppCfg.hEtherRing, &txFreeQ);
             EnetAppUtils_assert(retVal == ENET_SOK);

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
        EnetAppUtils_print("no free pktInfo to send stream\r\n");
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

/* Creates and sends packet for the enabled steams per timer tick */
static void EnetApp_periodicTrafficGen(void *args)
{
    int32_t retVal = ENET_SOK;
    ClockP_sleep(10);
    isStreamsEnabled = 1;

    while (true)
    {
        SemaphoreP_pend(&gEnetAppCfg.streamSemObj[0U], SystemP_WAIT_FOREVER);

        if (gEnetAppCfg.txCount < ENETAPP_TOTAL_TX_PACKET_COUNT)
        {
            for(int32_t index = 0 ;index < ENETAPP_NUM_CLASSA_STREAMS ;index++)
            {
                /* Sends the stream packets only If stream is enabled in the configuration */
                if (TrafficGen_ClassAList[index].isEnabled == true)
                {
                    EnetApp_sendRedundancyPacket(TrafficGen_ClassAList[index].hEtherRing, TrafficGen_ClassAList[index].destinationNodeId,
                                       TrafficGen_ClassAList[index].isProfilingEnabled);
                }
            }

            if (retVal != ENET_SOK)
            {
                EnetAppUtils_print("Ether-ring Tx submit failed\r\n");
            }
        }
    }
}

/* Send Echo Packet to Destination NodeId */
static void EnetApp_sendEchoPacket(EnetDma_Pkt *rxPktInfo, uint8_t destNodeId)
{
    int32_t status = ENET_SOK;
    EnetDma_PktQ txSubmitQ;
    EnetDma_PktQ rxFreeQ, txFreeQ;
    EnetDma_Pkt *txPktInfo = NULL;
    EthVlanFrame *rxFrame, *txFrame;
    uint32_t totalLenReceived = 0;
    uint32_t index = 0U;
    uint8_t multicastAddr[ENET_MAC_ADDR_LEN] = {0x01,0x00,0x5E,0x7F,0xFF,destNodeId};

    EnetQueue_initQ(&rxFreeQ);
    EnetQueue_initQ(&txSubmitQ);
    EnetQueue_initQ(&txFreeQ);

    if (rxPktInfo != NULL)
    {
        rxFrame = (EthVlanFrame *)rxPktInfo->sgList.list[0].bufPtr;
        EnetDma_checkPktState(&rxPktInfo->pktState,
                                ENET_PKTSTATE_MODULE_APP,
                                ENET_PKTSTATE_APP_WITH_DRIVER,
                                ENET_PKTSTATE_APP_WITH_READYQ);
        totalLenReceived = 0;
        for (int32_t i = 0; i < rxPktInfo->sgList.numScatterSegments; i++)
        {
            totalLenReceived += rxPktInfo->sgList.list[i].segmentFilledLen;
        }
        EnetAppUtils_assert(totalLenReceived <= ENET_MEM_LARGE_POOL_PKT_SIZE);

        /* Dequeue one free TX Eth packet */
        txPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetAppCfg.txFreePktInfoQ);
        if (txPktInfo != NULL)
        {
            /* Fill the TX Eth frame with test content */
            txFrame = (EthVlanFrame *)txPktInfo->sgList.list[0].bufPtr;
            memcpy(txFrame->hdr.dstMac, multicastAddr, ENET_MAC_ADDR_LEN);
            memcpy(txFrame->hdr.srcMac, &gEnetAppCfg.macAddr[0U], ENET_MAC_ADDR_LEN);
            txFrame->hdr.tpid = rxFrame->hdr.tpid;
            txFrame->hdr.etherType = rxFrame->hdr.etherType;;
            txFrame->hdr.tci  = rxFrame->hdr.tci;

            txPktInfo->sgList.list[0].segmentFilledLen = totalLenReceived;
            EnetAppUtils_assert(txPktInfo->sgList.list[0].segmentAllocLen >= txPktInfo->sgList.list[0].segmentFilledLen);

            memcpy(&txFrame->payload[0U],
                   &rxFrame->payload[0U],
                   rxPktInfo->sgList.list[0].segmentFilledLen - sizeof(EthFrameHeader));
            index = rxPktInfo->sgList.list[0].segmentFilledLen - sizeof(EthFrameHeader);
            for (int32_t i = 1; i < rxPktInfo->sgList.numScatterSegments; i++)
            {
                memcpy(&txFrame->payload[index],
                       rxPktInfo->sgList.list[i].bufPtr,
                       rxPktInfo->sgList.list[i].segmentFilledLen);
                index += rxPktInfo->sgList.list[i].segmentFilledLen;
            }

            txPktInfo->sgList.numScatterSegments = 1;
            txPktInfo->chkSumInfo = 0U;
            txPktInfo->appPriv = &gEnetAppCfg;
            txPktInfo->tsInfo.enableHostTxTs = false;

            EnetDma_checkPktState(&txPktInfo->pktState,
                                    ENET_PKTSTATE_MODULE_APP,
                                    ENET_PKTSTATE_APP_WITH_FREEQ,
                                    ENET_PKTSTATE_APP_WITH_DRIVER);

            /* Enqueue the packet for later transmission */
            EnetQueue_enq(&txSubmitQ, &txPktInfo->node);
        }
        else
        {
            EnetAppUtils_print("ECHO: Drop due to TX pkt not available\r\n");
        }

        EnetDma_checkPktState(&rxPktInfo->pktState,
                                ENET_PKTSTATE_MODULE_APP,
                                ENET_PKTSTATE_APP_WITH_READYQ,
                                ENET_PKTSTATE_APP_WITH_FREEQ);
    }

    /* Retrieve any packets that may be free now */
    status = EtherRing_retrieveTxPktQ(gEnetAppCfg.hEtherRing, &txFreeQ);
    if (status == ENET_SOK)
    {
        EnetDma_Pkt *pktInfo = NULL;

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

    /* Transmit all enqueued packets */
    status = EtherRing_submitTxPktQ(gEnetAppCfg.hEtherRing, &txSubmitQ);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("ECHO: Failed to submit TX pkt queue\r\n");
    }

    EnetAppUtils_validatePacketState(&rxFreeQ,
                                        ENET_PKTSTATE_APP_WITH_FREEQ,
                                        ENET_PKTSTATE_APP_WITH_DRIVER);
}

void EnetApp_createRxTrafficTask()
{
    TaskP_Params taskParams;
    int32_t status = ENET_SOK;

    DebugP_assert(ENET_SOK == status);

    status = SemaphoreP_constructBinary(&gEnetAppCfg.rxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    TaskP_Params_init(&taskParams);
    taskParams.priority       = 8U;
    taskParams.stack          = gEnetAppTaskStackRx;
    taskParams.stackSize      = sizeof(gEnetAppTaskStackRx);
    taskParams.args           = (void*)&gEnetAppCfg;
    taskParams.name           = "ER RX Task";
    taskParams.taskMain       = &EnetApp_rxTrafficHandler;

    status = TaskP_construct(&gEnetAppCfg.rxTaskObj, &taskParams);

    DebugP_assert(SystemP_SUCCESS == status);
}

void EnetApp_createPeriodicTrafficTask()
{
    int8_t stream_id = 0;
    int32_t status = ENET_SOK;
    TaskP_Params taskParamsStreamGen;

	status = SemaphoreP_constructBinary(&gEnetAppCfg.streamSemObj[stream_id], 0);
	DebugP_assert(SystemP_SUCCESS == status);
	TaskP_Params_init(&taskParamsStreamGen);
	taskParamsStreamGen.priority       = 7U;
	taskParamsStreamGen.stack          = gEnetAppStreamTaskStack;
	taskParamsStreamGen.stackSize      = sizeof(gEnetAppStreamTaskStack);
	taskParamsStreamGen.args           = (void*)&gEnetAppCfg;
	taskParamsStreamGen.name           = "ER Tick Task";
	taskParamsStreamGen.taskMain       = &EnetApp_periodicTrafficGen;

	status = TaskP_construct(&gEnetAppCfg.streamTaskObj[stream_id], &taskParamsStreamGen);
	DebugP_assert(SystemP_SUCCESS == status);

    EnetAppUtils_print("Redundancy Traffic Task Creation \r\n");
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
    taskParams.priority       = 3;
    taskParams.stack          = gEnetAppEtherRingTaskStack;
    taskParams.stackSize      = sizeof(gEnetAppEtherRingTaskStack);
    taskParams.args           = (void*)&gEnetAppCfg;
    taskParams.name           = "ER Clear Task";
    taskParams.taskMain       = &EnetApp_clearLookupTable;

    status = TaskP_construct(&gEnetAppCfg.etherringTaskObj, &taskParams);

    DebugP_assert(SystemP_SUCCESS == status);
}

void EnetApp_hwTimerISR(void)
{
    static uint32_t etherRingCounter  = 0;
#ifdef ENETAPP_ENABLE_TCP_BG_TRAFFIC
    static uint32_t lwipCounter  = 0;
#endif

    if(isStreamsEnabled)
    {
        SemaphoreP_post(&gEnetAppCfg.streamSemObj[0U]);

        etherRingCounter++;
        if(etherRingCounter % 256 == 0)
        {
            /* Clears Ether-Ring Look-up table periodically to support the Single point of Failure */
            SemaphoreP_post(&gEnetAppCfg.etherringSemObj);
            etherRingCounter = 0;
        }
#ifdef ENETAPP_ENABLE_TCP_BG_TRAFFIC
        lwipCounter++;
        /* Sends BackGround TCP packet periodically */
        if(lwipCounter % 8)
        {
            SemaphoreP_post(&gEnetAppCfg.lwipSemObj);
            lwipCounter = 0;
        }
#endif
    }
}
