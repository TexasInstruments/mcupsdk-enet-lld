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

#include "avtpcf.h"
#include "enet_ethutils.h"
#include "enet_apputils.h"
#include "enet_apputils_k3.h"
#include "enet_appmemutils.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define AVTPCF_VLAN_TPID                            (0x8100U)
#define AVTPCF_VLAN_PCP_OFFSET                      (13U)
#define AVTPCF_VLAN_PCP_MASK                        (0x7U)
#define AVTPCF_VLAN_DEI_OFFSET                      (12U)
#define AVTPCF_VLAN_DEI_MASK                        (0x1U)
#define AVTPCF_VLAN_VID_MASK                        (0xFFFU)
#define AVTPCF_VLAN_TCI(pcp, dei, vid)              ((((pcp) & AVTPCF_VLAN_PCP_MASK) << AVTPCF_VLAN_PCP_OFFSET) | \
                                                      (((dei) & AVTPCF_VLAN_DEI_MASK) << AVTPCF_VLAN_DEI_OFFSET) | \
                                                      (((vid) & AVTPCF_VLAN_VID_MASK)))

#define AVTPCF_FILL_COMMON_FIELD(commonField, sv, version, r, ntscf_data_length)    \
                                                do {                                                                \
                                                        commonField[0] = ((sv&0x01)<<7) | ((version&0x07)<<4)       \
                                                                | ((r&0x01)<<3) | ((ntscf_data_length>>8)&0x07);    \
                                                        commonField[1] = (ntscf_data_length&0xFF);                  \
                                                    } while (0);

#define  AVTPCF_HEADER_SV                           (1)
#define  AVTPCF_HEADER_VERSION                      (0x01)
#define  AVTPCF_HEADER_R                            (0)

/*
 *  1 - Uses EtherRing_submit/Retrieve APIs
 *  Handles Duplicate Rejection.
 *  0 - Uses EnetDma_submit/Retrieve APIs
 *  Does not Handle Duplicate Rejection.
*/
#define USE_ETHERRING                               (1)

EtherRing_Cfg gEtherRingCfg;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t avtpcf_configDmaChCb(avtpcf_enetInfo_t* enetInfo, void* args);

static void avtpcf_initTxFreePktQ(avtpcf_enetInfo_t* enetInfo);

static void avtpcf_initRxReadyPktQ(avtpcf_enetInfo_t* enetInfo);

static EtherRing_Handle avtpcf_etherRingInit(avtpcf_enetInfo_t* enetInfo);

static uint8_t avtpcf_getKeyFromStreamID(uint8_t *streamID);

static void avtpcf_rxCallback(void* args);

int32_t avtpcf_init(avtpcf_data_t* avtpData)
{
    int32_t status = ENET_SOK;

    /* Configure DMA Ch Callback. */
    avtpcf_configDmaChCb(&avtpData->enetInfo, avtpData);

    /* Initialize Tx Packets. */
    avtpcf_initTxFreePktQ(&avtpData->enetInfo);

    /* Initialize Rx Packets. */
    avtpcf_initRxReadyPktQ(&avtpData->enetInfo);

    #if USE_ETHERRING
    avtpData->enetInfo.hEtherRing = avtpcf_etherRingInit(&avtpData->enetInfo);

    if (avtpData->enetInfo.hEtherRing == NULL)
    {
        status = ENET_EFAIL;
    }
    #endif

    memset(avtpData->cbArgsArray, 0, sizeof(avtpData->cbArgsArray));
    memset(avtpData->cbArray, 0, sizeof(avtpData->cbArray));

    return status;
}

/* Provide stream ID. */
int32_t avtpcf_talker_init(avtpcf_data_t* avtpcfData, avtpcf_talker_Obj* talkerObj, avtpcf_talker_cfg* talkerCfg)
{
    /* Create L2 Header. */
    memcpy(&talkerObj->l2Header.dstMac, &talkerCfg->mcastMac, sizeof(talkerObj->l2Header.dstMac));
    memcpy(&talkerObj->l2Header.srcMac, &talkerCfg->srcMac, sizeof(talkerObj->l2Header.srcMac));
    talkerObj->l2Header.tpid      = Enet_htons(AVTPCF_VLAN_TPID);
    talkerObj->l2Header.tci       = Enet_htons(AVTPCF_VLAN_TCI(talkerCfg->priority, 0, talkerCfg->vlanID));
    talkerObj->l2Header.etherType = Enet_htons(0x22F0); /* AVTP EtherType */

    /* Create avtpcf Header */
    talkerObj->avtpcf_header.subtype = 0x82; /* NTSCF Ref: 4.4.3.2 */
    talkerObj->avtpcf_header.sequence_num = 0;
    memcpy(talkerObj->avtpcf_header.stream_id, talkerCfg->streamID, sizeof(talkerObj->avtpcf_header.stream_id));
    talkerObj->avtpcf_data = avtpcfData;
    return 0;
}

/* Provide stream ID, Callback function, callback function args. */
int32_t avtpcf_listener_init(avtpcf_data_t* avtpcfData, avtpcf_listener_Obj* listenerObj, avtpcf_listener_cfg* listenerCfg)
{
    listenerObj->avtpcf_data = avtpcfData;
    uint8_t key = avtpcf_getKeyFromStreamID(listenerCfg->streamID);
    listenerObj->avtpcf_data->cbArray[key] = listenerCfg->callback;
    listenerObj->avtpcf_data->cbArgsArray[key] = listenerCfg->cbArgs;
    return 0;
}

int32_t avtpcf_talker_enqueue(avtpcf_talker_Obj* talkerObj, uint8_t *payload, uint32_t size)
{
    avtpcf_enetInfo_t* enetInfo = &talkerObj->avtpcf_data->enetInfo;

    /* Dequeue one packet from Tx Free Queue. */
    if (EnetQueue_getQCount(&enetInfo->txFreePktInfoQ) > 0)
    {
        EnetDma_Pkt *pktInfo = (EnetDma_Pkt*)EnetQueue_deq(&enetInfo->txFreePktInfoQ);
        if (pktInfo != NULL)
        {
            EthVlanFrame *frame = (EthVlanFrame *)pktInfo->sgList.list[0].bufPtr;
            memcpy(&frame->hdr, &talkerObj->l2Header, sizeof(talkerObj->l2Header));
            AVTPCF_FILL_COMMON_FIELD(talkerObj->avtpcf_header.commonField, AVTPCF_HEADER_SV, AVTPCF_HEADER_VERSION, AVTPCF_HEADER_R, size);
            talkerObj->avtpcf_header.sequence_num += 1;
            memcpy(&frame->payload[0], &talkerObj->avtpcf_header, sizeof(talkerObj->avtpcf_header));
            memcpy(&frame->payload[sizeof(talkerObj->avtpcf_header)], payload, size);
            pktInfo->sgList.list[0].segmentFilledLen = sizeof(EthVlanFrameHeader) + sizeof(talkerObj->avtpcf_header) + size;

            pktInfo->sgList.numScatterSegments = 1;
            pktInfo->chkSumInfo = 0U;

            EnetDma_checkPktState(&pktInfo->pktState,
                            ENET_PKTSTATE_MODULE_APP,
                            ENET_PKTSTATE_APP_WITH_FREEQ,
                            ENET_PKTSTATE_APP_WITH_DRIVER);
            EnetQueue_enq(&enetInfo->txSubmitQ, &pktInfo->node);
        }
    }
    return 0;
}

int32_t avtpcf_submit_packets(avtpcf_data_t* avtpcf_data)
{
#if USE_ETHERRING
    EtherRing_submitTxPktQ(avtpcf_data->enetInfo.hEtherRing, \
                            &avtpcf_data->enetInfo.txSubmitQ);
#else
    EnetDma_submitTxPktQ(avtpcf_data->enetInfo.hTxCh, \
                            &avtpcf_data->enetInfo.txSubmitQ);
#endif
    return 0;
}

int32_t avtpcf_recycle_packets(avtpcf_data_t* avtpcf_data)
{
    EnetDma_PktQ txFreeQ;
    int32_t retVal;
    EnetQueue_initQ(&txFreeQ);
#if USE_ETHERRING
    retVal = EtherRing_retrieveTxPktQ(avtpcf_data->enetInfo.hEtherRing, &txFreeQ);
#else
    retVal = EnetDma_retrieveTxPktQ(avtpcf_data->enetInfo.hTxCh, &txFreeQ);
#endif
    if (retVal == ENET_SOK)
    {
        EnetDma_Pkt *pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        while (NULL != pktInfo)
        {
            EnetDma_checkPktState(&pktInfo->pktState,
                                    ENET_PKTSTATE_MODULE_APP,
                                    ENET_PKTSTATE_APP_WITH_DRIVER,
                                    ENET_PKTSTATE_APP_WITH_FREEQ);

            EnetQueue_enq(&avtpcf_data->enetInfo.txFreePktInfoQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        }
    }
    return 0;
}

int32_t avtpcf_process_frame(avtpcf_data_t* avtpData, EthVlanFrame *frame)
{
    /* Check if the EtherType is AVTP. */
    if (frame->hdr.etherType == Enet_htons(0x22F0))
    {
        /* Extract Stream ID. */
        avtpcf_ntscf_header* avtpcf_header = (avtpcf_ntscf_header*)frame->payload;
        avtpcf_info info;
        info.subtype = avtpcf_header->subtype;
        info.seqID = avtpcf_header->sequence_num;
        info.streamID = avtpcf_header->stream_id;
        info.sv       = avtpcf_header->commonField[0]>>7;
        info.version  = (avtpcf_header->commonField[0]>>4)&0x7;
        info.r        = (avtpcf_header->commonField[0]>>3)&0x1;
        info.ntscf_data_length  = ((avtpcf_header->commonField[0]&0x07)<<8) | \
                                    (avtpcf_header->commonField[1]);
        int size = info.ntscf_data_length;
        int key = avtpcf_getKeyFromStreamID(info.streamID);
        uint8_t* payload = &frame->payload[sizeof(avtpcf_ntscf_header)];

        if (avtpData->cbArray[key] != NULL)
        {
            avtpData->cbArray[key](&info, payload, size, avtpData->cbArgsArray[key]);
        }
    }
    return 0;
}

int32_t avtpcf_listeners_spin(avtpcf_data_t* avtpData)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxSubmitQ;
    EnetDma_Pkt *pktInfo = NULL;
    int32_t status = ENET_SOK;
    EnetQueue_initQ(&rxReadyQ);
    EnetQueue_initQ(&rxSubmitQ);
#if USE_ETHERRING
    status = EtherRing_retrieveRxPktQ(avtpData->enetInfo.hEtherRing, &rxReadyQ);
#else
    status = EnetDma_retrieveRxPktQ(avtpData->enetInfo.hRxCh, &rxReadyQ);
#endif

    pktInfo = (EnetDma_Pkt*)EnetQueue_deq(&rxReadyQ);

    while (pktInfo)
    {
        EthVlanFrame *frame = (EthVlanFrame *)pktInfo->sgList.list[0].bufPtr;

        avtpcf_process_frame(avtpData, frame);

        EnetQueue_enq(&rxSubmitQ, &pktInfo->node);
        pktInfo = (EnetDma_Pkt*)EnetQueue_deq(&rxReadyQ);
    }
    if (EnetQueue_getQCount(&rxSubmitQ) > 0)
    {
#if USE_ETHERRING
        status = EtherRing_submitRxPktQ(avtpData->enetInfo.hEtherRing, &rxSubmitQ);
#else
        EnetDma_submitRxPktQ(avtpData->enetInfo.hRxCh, &rxSubmitQ);
#endif

    }
    return status;
}

static uint8_t avtpcf_getKeyFromStreamID(uint8_t *streamID)
{
    return streamID[7];
}

static int32_t avtpcf_configDmaChCb(avtpcf_enetInfo_t* enetInfo, void* args)
{
    EnetApp_GetDmaHandleInArgs     txInArgs;
    EnetApp_GetTxDmaHandleOutArgs  txChInfo;
    int32_t status = ENET_SOK;

    /* Open the TX channel */
    txInArgs.cbArg   = NULL;
    txInArgs.notifyCb = NULL;

    EnetApp_getTxDmaHandle(enetInfo->cfg.txChCfgIdx, &txInArgs, &txChInfo);

    if (txChInfo.hTxCh == NULL)
    {
        EnetAppUtils_freeTxCh(enetInfo->hEnet, enetInfo->coreKey,
                             enetInfo->coreId, txChInfo.txChNum);
        status = ENET_EFAIL;
    }
    else
    {
        enetInfo->txChNum = txChInfo.txChNum;
        enetInfo->hTxCh   = txChInfo.hTxCh;
    }

    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs     rxInArgs;
        EnetApp_GetRxDmaHandleOutArgs  rxChInfo;

        rxInArgs.notifyCb = avtpcf_rxCallback;
        rxInArgs.cbArg    = args;

        EnetApp_getRxDmaHandle(enetInfo->cfg.rxChCfgIdx, &rxInArgs, &rxChInfo);

        if (rxChInfo.hRxCh == NULL)
        {
            EnetAppUtils_freeRxFlow(enetInfo->hEnet,
                enetInfo->coreKey, enetInfo->coreId, rxChInfo.rxFlowIdx);
            status = ENET_EFAIL;
        }
        else
        {
            enetInfo->hRxCh   = rxChInfo.hRxCh;
            enetInfo->rxFlowIdx = rxChInfo.rxFlowIdx;
        }
    }
    return status;
}

static void avtpcf_initTxFreePktQ(avtpcf_enetInfo_t* enetInfo)
{
    EnetQueue_initQ(&enetInfo->txFreePktInfoQ);
    EnetQueue_initQ(&enetInfo->txSubmitQ);
    uint32_t scatterSegments[] = { enetInfo->cfg.packetSize };
    for (uint32_t i = 0U; i < enetInfo->cfg.numTxPkts; i++)
    {
        EnetDma_Pkt *pPktInfo = EnetMem_allocEthPkt(enetInfo,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);
        EnetQueue_enq(&enetInfo->txFreePktInfoQ, &pPktInfo->node);
    }
}

static void avtpcf_initRxReadyPktQ(avtpcf_enetInfo_t* enetInfo)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    int32_t status;

    uint32_t scatterSegments[] = { enetInfo->cfg.packetSize };
    EnetQueue_initQ(&rxFreeQ);

    for (uint32_t i = 0U; i < enetInfo->cfg.numRxPkts; i++)
    {
        EnetDma_Pkt *pPktInfo = EnetMem_allocEthPkt(enetInfo,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);
        EnetQueue_enq(&rxFreeQ, &pPktInfo->node);
    }

    /* Retrieve any packets which are ready */
    EnetQueue_initQ(&rxReadyQ);
    status = EnetDma_retrieveRxPktQ(enetInfo->hRxCh, &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);

    /* There should not be any packet with DMA during init */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&rxFreeQ,
                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                     ENET_PKTSTATE_APP_WITH_DRIVER);

    EnetDma_submitRxPktQ(enetInfo->hRxCh, &rxFreeQ);

    /* Assert here, as during init, the number of DMA descriptors should be equal to
     * the number of free Ethernet buffers available with app */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxFreeQ) == 0U);
}

static EtherRing_Handle avtpcf_etherRingInit(avtpcf_enetInfo_t* enetInfo)
{
    gEtherRingCfg.hostMacAddLastByte = enetInfo->macAddr[5];
    gEtherRingCfg.isCfg = true;

    DebugP_log("EtherRing Host Mac Address: ");
    EnetAppUtils_printMacAddr(enetInfo->macAddr);

    EtherRing_Handle hEtherRing = EtherRing_open(enetInfo->hEnet, enetInfo->coreId, &gEtherRingCfg);
    if (hEtherRing == NULL)
    {
        EnetAppUtils_print("EtherRing Handle is NULL!!\n");
        Enet_assert(hEtherRing != NULL);
    }
    else
    {
        EtherRing_attachTxDmaHandle((void *)hEtherRing, enetInfo->hTxCh, enetInfo->txChNum);
        EtherRing_attachRxDmaHandle((void *)hEtherRing, enetInfo->hRxCh, enetInfo->rxFlowIdx);
    }
    return hEtherRing;
}

static void avtpcf_rxCallback(void* args)
{
    avtpcf_data_t* avtpcData = (avtpcf_data_t*)args;
    avtpcf_listeners_spin(avtpcData);
}
