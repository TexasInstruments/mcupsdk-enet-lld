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
 * \file app_rawudp_handler.c
 *
 * \brief This file contains the configuration and setup of raw-udp channel used for real-time data tranmission
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "app_rawudp_handler.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void RawUdpApp_openDMA(EnetApp_Obj* gEnetApp);

void RawUdpApp_createRxTxTask(EnetApp_ChInfo* hRawUdp);

void RawUdpApp_rxTxTask(void* hRawUdp);

uint32_t RawUdpApp_verifyRxRTFrameIp(uint8_t* rxIp, EnetApp_ChInfo* hRawUdp);

bool RawUdpApp_parseFrame(EthFrame *frame, uint32_t dataLen, EnetApp_ChInfo* hRawUdp);

int32_t RawUdpApp_applyClassifier(EnetApp_Obj* gEnetApp);

void RawUdpApp_rxIsrFxn(void *arg);

extern void RawUdpApp_initTxFreePktQ(EnetApp_ChInfo* hRawUdp);

extern void RawUdpApp_initRxReadyPktQ(EnetApp_ChInfo* hRawUdp);

extern int32_t RawUdpApp_receivePkts(EnetApp_ChInfo* hRawUdp, EnetDma_PktQ* rxReadyQ);

extern int32_t RawUdpApp_transmitPkts(EnetApp_ChInfo* hRawUdp, EnetDma_PktQ* txSubmitQ);

extern void RawUdpApp_modifyPkts(EnetApp_ChInfo* hRawUdp, EnetDma_PktQ *rxReadyQ, EnetDma_PktQ *txSubmitQ);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static uint8_t gEnetAppTaskStackRxRT[ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));

#if DEBUG
volatile static uint32_t gMatchedCriteria = 0;
volatile static uint32_t gipv4DestVerify= 0;
#endif

#ifdef ENETAPP_PROFILE_EN/*Enable in app_cfg.h*/
    extern float averageProfileTime,maxProfileTime;
    extern uint32_t profileIndex,rxIsrTime,subTxTime;
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void RawUdpApp_setup(EnetApp_Obj* gEnetApp)
{
    EnetAppUtils_print("Setting up real-time channel \r\n");

    /* Open Rx and Tx DMA channels for real-time app */
    RawUdpApp_openDMA(gEnetApp);

    /* Create Rx task for real-time channel */
    RawUdpApp_createRxTxTask(&gEnetApp->hRawUdp);

    /* Configure the classifier */
    RawUdpApp_applyClassifier(gEnetApp);
    EnetAppUtils_print("IP for Realtime Traffic is 10.24.0.%d\r\n\n", gEnetApp->nodeId);
}

void RawUdpApp_openDMA(EnetApp_Obj* gEnetApp)
{
    int32_t status = ENET_SOK;
    EnetApp_GetDmaHandleInArgs     txInArgs;
    EnetApp_GetTxDmaHandleOutArgs  txChInfo;

    /* Open the TX channel */
    txInArgs.cbArg   = NULL;
    txInArgs.notifyCb = NULL;

    EnetApp_getTxDmaHandle((ENET_DMA_TX_CH1),
                           &txInArgs,
                           &txChInfo);

    gEnetApp->hRawUdp.txChNum = txChInfo.txChNum;
    gEnetApp->hRawUdp.hTxCh   = txChInfo.hTxCh;


    /* Allocate TX packets and keep them locally enqueued */
    if (status == ENET_SOK)
    {
        /* Initialize the packet queue used for storing free Tx packets */
        RawUdpApp_initTxFreePktQ(&gEnetApp->hRawUdp);
    }

    if (gEnetApp->hRawUdp.hTxCh  == NULL)
    {
#if FIX_RM
        /* Free the channel number if open Tx channel failed */
        EnetAppUtils_freeTxCh(gEnetApp->hEnet,
                              gEnetApp->coreKey,
                              gEnetApp->coreId,
                              gEnetApp->hRawUdp.txChNum);
#endif
        EnetAppUtils_print("EnetApp_openDma() failed to open realtime TX channel\r\n");
        status = ENET_EFAIL;
        EnetAppUtils_assert(gEnetApp->hRawUdp.hTxCh != NULL);
    }

    /* Open the RX flow for Regular frames */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs     rxInArgs;
        EnetApp_GetRxDmaHandleOutArgs  rxChInfo;

        rxInArgs.notifyCb = RawUdpApp_rxIsrFxn;
        rxInArgs.cbArg   = &gEnetApp->hRawUdp.rxSemObj;

        EnetApp_getRxDmaHandle((ENET_DMA_RX_CH1),
                               &rxInArgs,
                               &rxChInfo);
        gEnetApp->hRawUdp.rxChNum = rxChInfo.rxChNum;
        gEnetApp->hRawUdp.hRxCh  = rxChInfo.hRxCh;

        if (gEnetApp->hRawUdp.hRxCh == NULL)
        {
            EnetAppUtils_print("EnetApp_openRxCh() failed to open realtime RX flow\r\n");
            status = ENET_EFAIL;
            EnetAppUtils_assert(gEnetApp->hRawUdp.hRxCh != NULL);
        }
    }
    if (status == ENET_SOK)
    {
        /* Initialize the packet queue used for receiving packets */
        RawUdpApp_initRxReadyPktQ(&gEnetApp->hRawUdp);
    }
    else
    {
        EnetAppUtils_print("Failed to open DMA for real-time traffic\r\n");
    }

    return;

}

int32_t RawUdpApp_applyClassifier(EnetApp_Obj* gEnetApp)
{
    Enet_IoctlPrms prms;
    CpswAle_SetPolicerEntryOutArgs setPolicerEntryOutArgs;
    CpswAle_SetPolicerEntryInArgs setPolicerEntryInArgs;
    int32_t status;

    gEnetApp->hRawUdp.IpAddr[0] = 10U;
    gEnetApp->hRawUdp.IpAddr[1] = 24U;
    gEnetApp->hRawUdp.IpAddr[2] = 0U;
    gEnetApp->hRawUdp.IpAddr[3] = (uint8_t)gEnetApp->nodeId;

    memset(&setPolicerEntryInArgs, 0, sizeof(setPolicerEntryInArgs));

    setPolicerEntryInArgs.policerMatch.policerMatchEnMask |=
            CPSW_ALE_POLICER_MATCH_IPDST;
    setPolicerEntryInArgs.policerMatch.dstIpInfo.ipv4Info.ipv4Addr[0] = 10U;
    setPolicerEntryInArgs.policerMatch.dstIpInfo.ipv4Info.ipv4Addr[1] = 24U;
    setPolicerEntryInArgs.policerMatch.dstIpInfo.ipv4Info.ipv4Addr[2] = 0U;
    setPolicerEntryInArgs.policerMatch.dstIpInfo.ipv4Info.ipv4Addr[3] = (uint8_t)gEnetApp->nodeId;
    setPolicerEntryInArgs.policerMatch.dstIpInfo.ipv4Info.numLSBIgnoreBits = 0U;

    setPolicerEntryInArgs.threadIdEn = true;
    setPolicerEntryInArgs.threadId = gEnetApp->hRawUdp.rxChNum;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setPolicerEntryInArgs,
            &setPolicerEntryOutArgs);
    ENET_IOCTL(gEnetApp->hEnet, gEnetApp->coreId, CPSW_ALE_IOCTL_SET_POLICER, &prms, status);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("%s():CPSW_ALE_IOCTL_SET_POLICER failed: %d\r\n",
                __func__, status);
    }
    return status;
}

void RawUdpApp_rxIsrFxn(void *arg) {
#ifdef ENETAPP_PROFILE_EN 
    rxIsrTime = CycleCounterP_getCount32();
#endif
    SemaphoreP_Object *semObj = (SemaphoreP_Object *)arg;
    SemaphoreP_post(semObj);
}

void RawUdpApp_createRxTxTask(EnetApp_ChInfo* hRawUdp)
{
    TaskP_Params taskParams;
    int32_t status;
    status = SemaphoreP_constructBinary(&hRawUdp->rxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructCounting(&hRawUdp->rxDoneSemObj, 0, ENETAPP_COUNTING_SEM_COUNT);
    DebugP_assert(SystemP_SUCCESS == status);

    TaskP_Params_init(&taskParams);
    taskParams.priority       = 8U;
    taskParams.stack          = gEnetAppTaskStackRxRT;
    taskParams.stackSize      = sizeof(gEnetAppTaskStackRxRT);
    taskParams.args           = hRawUdp;
    taskParams.name           = "Real Time Rx Task";
    taskParams.taskMain       = &RawUdpApp_rxTxTask;

    status = TaskP_construct(&hRawUdp->rxTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);
    EnetAppUtils_print("Created Rx task for real time channel %d\r\n", hRawUdp->rxChNum);

}

uint32_t RawUdpApp_verifyRxRTFrameIp(uint8_t* rxIp, EnetApp_ChInfo* hRawUdp)
{
    return (rxIp[0]==hRawUdp->IpAddr[0] && rxIp[1]==hRawUdp->IpAddr[1] &&
            rxIp[2]==hRawUdp->IpAddr[2] && rxIp[3]==hRawUdp->IpAddr[3]);
}

/* Rx task for real-time channel */
void RawUdpApp_rxTxTask(void* arg)
{
    EnetApp_ChInfo* hRawUdp = (EnetApp_ChInfo*)arg;
    EnetDma_PktQ rxPktQ;
    EnetDma_PktQ txPktQ;

#if DEBUG
    totalRxCnt = 0U;
#endif
    int32_t status = ENET_SOK;
    while ((ENET_SOK == status))
    {
        /* Wait for packet reception */
        SemaphoreP_pend(&hRawUdp->rxSemObj, SystemP_WAIT_FOREVER);

        /*Retrieve the recevied packets*/
        status  = RawUdpApp_receivePkts(hRawUdp,&rxPktQ) ;
        if( status != ENET_SOK ){
            continue;
        }
        /*Modify the received packets*/
        RawUdpApp_modifyPkts(hRawUdp,&rxPktQ, &txPktQ);

        /*Submit the modified packets for transmission*/
        status = RawUdpApp_transmitPkts(hRawUdp,&txPktQ);

#ifdef ENETAPP_PROFILE_EN
        if((profileIndex >= ENETAPP_PKT_PROFILE) && (status == ENET_SOK))
        {
            EnetAppUtils_print("=======================================================\r\n");
            EnetAppUtils_print("      SOFTWARE PROFILING FOR REALTIME CHANNEL \r\n");
            EnetAppUtils_print("=======================================================\r\n");
            EnetAppUtils_print("Number of Packets: %d \r\n", profileIndex);
            EnetAppUtils_print("Average Profile Time: %f us\r\n", averageProfileTime);
            EnetAppUtils_print("Max Profile Time: %f us\r\n", maxProfileTime);
            EnetAppUtils_print("======================================================\r\n");
            profileIndex = 0;
            averageProfileTime = 0;
            maxProfileTime = 0;
        }
#endif
    }

#if DEBUG
    EnetAppUtils_print("Received %u packets\r\n", totalRxCnt);
#endif

    SemaphoreP_post(&hRawUdp->rxDoneSemObj);
    TaskP_exit();
}

bool RawUdpApp_parseFrame(EthFrame *frame, uint32_t dataLen, EnetApp_ChInfo* hRawUdp)
{
    uint8_t *payload;
    bool status = false;

    payload = frame->payload;

    EthIPv4Header *ipv4Frame = (EthIPv4Header *) payload;

        if ((frame->hdr.etherType == Enet_htons(ENETAPP_IPV4_ETHERTYPE)) && (ipv4Frame->protocol == ENETAPP_IPV4_HDR_UDP))
        {
#if DEBUG
            gMatchedCriteria++; 
#endif
            /* Verify if frame is valid */
            if (RawUdpApp_verifyRxRTFrameIp(ipv4Frame->dstIP, hRawUdp))
            {
#if DEBUG
                gipv4DestVerify++;
#endif
                status = true;
            }

        }

    return status;
}
