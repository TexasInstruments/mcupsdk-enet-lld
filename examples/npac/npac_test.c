/*
 *  Copyright (c) Texas Instruments Incorporated 2022
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
 * \file  npac_test.c
 *
 * \brief This file contains the NPAC Datapath test implementation.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "npac_common.h"
#include "FreeRTOS.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define NPAC_DATAPATH_TEST_PKT_NUM                      (8U)

#define NPAC_DATAPATH_TEST_PKT_LEN                      (500U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void EnetApp_createRxTxTasks(void);

static uint32_t EnetApp_retrieveFreeTxPkts(void);

static void EnetApp_txTask(void *args);

static uint32_t EnetApp_receivePkts(void);

static void EnetApp_rxTask(void *args);

static void EnetApp_initCpswCfg(Cpsw_Cfg *cpswCfg);

static int32_t EnetApp_setupCpswAle(void);

static void EnetApp_closeEnet(void);

static int32_t EnetApp_showAlivePhys(void);

static int32_t EnetApp_waitForLinkUp(void);

static void EnetApp_showCpswStats(void);

static int32_t EnetApp_macMode2PhyMii(emac_mode macMode,
                                       EnetPhy_Mii *mii);

static void EnetApp_macMode2MacMii(emac_mode macMode,
                                    EnetMacPort_Interface *mii);

static void EnetApp_rxIsrFxn(void *appData);

static void EnetApp_txIsrFxn(void *appData);

static void EnetApp_initTxFreePktQ(void);

static void EnetApp_initRxReadyPktQ(void);

static int32_t EnetApp_openDma(void);

static void EnetApp_closeDma(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Test application stack */
static uint8_t gNpacDatapathTaskStackTx[NPAC_DATAPATH_TASK_STACK_SZ] __attribute__ ((aligned(32)));
static uint8_t gNpacDatapathTaskStackRx[NPAC_DATAPATH_TASK_STACK_SZ] __attribute__ ((aligned(32)));
extern NpacDatapath_Obj gNpacDatapathObj;
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t EnetApp_NpacDatapathTest(void)
{
    int32_t status;
    EnetApp_HandleInfo handleInfo;
    Enet_IoctlPrms prms;

    if (gNpacDatapathObj.enetType == ENET_CPSW_2G)
    {
        EnetAppUtils_print("CPSW_2G Test\r\n");
    }
    if (gNpacDatapathObj.enetType == ENET_CPSW_3G)
    {
        EnetAppUtils_print("CPSW_3G Test\r\n");
    }

    EnetAppUtils_enableClocks(gNpacDatapathObj.enetType, gNpacDatapathObj.instId);

    /* Create TX/RX semaphores */
    status = SemaphoreP_constructBinary(&gNpacDatapathObj.rxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gNpacDatapathObj.rxDoneSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gNpacDatapathObj.txSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gNpacDatapathObj.txDoneSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Local core id */
    gNpacDatapathObj.coreId = EnetSoc_getCoreId();

    EnetApp_driverInit();

    if (status == ENET_SOK)
    {
        status = EnetApp_driverOpen(gNpacDatapathObj.enetType, gNpacDatapathObj.instId);

        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open ENET: %d\r\n", status);
        }
    }
    gNpacDatapathObj.exitFlag     = false;
    gNpacDatapathObj.exitFlagDone = false;

    EnetApp_acquireHandleInfo(gNpacDatapathObj.enetType, gNpacDatapathObj.instId, &handleInfo);
    gNpacDatapathObj.hEnet = handleInfo.hEnet;

    /* Attach the core with RM */
    if (status == ENET_SOK)
    {
        EnetPer_AttachCoreOutArgs attachCoreOutArgs;

        EnetApp_coreAttach(gNpacDatapathObj.enetType, gNpacDatapathObj.instId, gNpacDatapathObj.coreId, &attachCoreOutArgs);
        gNpacDatapathObj.coreKey = attachCoreOutArgs.coreKey;
    }

    /* Open DMA driver */
    if (status == ENET_SOK)
    {
        status = EnetApp_openDma();
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open DMA: %d\r\n", status);
        }
    }

    if (status == ENET_SOK)
    {
        CpswAle_SetUcastEntryInArgs setUcastInArgs;
        uint8_t npac_port_mac_addr[ENET_MAC_ADDR_LEN] = {0x70, 0xff, 0x76,0x1d, 0xec, 0xf6};
        uint32_t entryIdx;
        /* ALE entry with "secure" bit cleared is required for loopback */
        setUcastInArgs.addr.vlanId  = 0U;
        setUcastInArgs.info.portNum = CPSW_ALE_HOST_PORT_NUM;
        setUcastInArgs.info.blocked = false;
        setUcastInArgs.info.secure  = false;
        setUcastInArgs.info.super   = false;
        setUcastInArgs.info.ageable = false;
        setUcastInArgs.info.trunk   = false;
        EnetUtils_copyMacAddr(&setUcastInArgs.addr.addr[0U], npac_port_mac_addr);
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &setUcastInArgs, &entryIdx);

        ENET_IOCTL(gNpacDatapathObj.hEnet, gNpacDatapathObj.coreId, CPSW_ALE_IOCTL_ADD_UCAST, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to add ucast entry: %d\r\n", status);
        }
    }
    if (status == ENET_SOK)
     {
         uint8_t npac_port_mac_addr[ENET_MAC_ADDR_LEN] = {0x70, 0xff, 0x76,0x1d, 0xec, 0xf8};

         CpswAle_SetUcastEntryInArgs setUcastInArgs;
         uint32_t entryIdx;
         /* ALE entry with "secure" bit cleared is required for loopback */
         setUcastInArgs.addr.vlanId  = 0U;
         setUcastInArgs.info.portNum = CPSW_ALE_MACPORT_TO_ALEPORT(9);
         setUcastInArgs.info.blocked = false;
         setUcastInArgs.info.secure  = true;
         setUcastInArgs.info.super   = false;
         setUcastInArgs.info.ageable = false;
         setUcastInArgs.info.trunk   = false;
         EnetUtils_copyMacAddr(&setUcastInArgs.addr.addr[0U], npac_port_mac_addr);
         ENET_IOCTL_SET_INOUT_ARGS(&prms, &setUcastInArgs, &entryIdx);

         ENET_IOCTL(gNpacDatapathObj.hEnet, gNpacDatapathObj.coreId, CPSW_ALE_IOCTL_ADD_UCAST, &prms, status);
         if (status != ENET_SOK)
         {
             EnetAppUtils_print("Failed to add ucast entry: %d\r\n", status);
         }
     }

    if (status == ENET_SOK)
     {
         uint8_t npac_port_mac_addr[ENET_MAC_ADDR_LEN] = {0x70, 0xff, 0x76,0x1d, 0xec, 0xf6};

         CpswAle_SetUcastEntryInArgs setUcastInArgs;
         uint32_t entryIdx;
         /* ALE entry with "secure" bit cleared is required for loopback */
         setUcastInArgs.addr.vlanId  = 0U;
         setUcastInArgs.info.portNum = CPSW_ALE_MACPORT_TO_ALEPORT(3);
         setUcastInArgs.info.blocked = false;
         setUcastInArgs.info.secure  = true;
         setUcastInArgs.info.super   = false;
         setUcastInArgs.info.ageable = false;
         setUcastInArgs.info.trunk   = false;
         EnetUtils_copyMacAddr(&setUcastInArgs.addr.addr[0U], npac_port_mac_addr);
         ENET_IOCTL_SET_INOUT_ARGS(&prms, &setUcastInArgs, &entryIdx);

         ENET_IOCTL(gNpacDatapathObj.hEnet, gNpacDatapathObj.coreId, CPSW_ALE_IOCTL_ADD_UCAST, &prms, status);
         if (status != ENET_SOK)
         {
             EnetAppUtils_print("Failed to add ucast entry: %d\r\n", status);
         }
     }

    if (status == ENET_SOK)
     {
        CpswAle_SetMcastEntryInArgs setMcastInArgs;
        uint32_t setMcastoutArgs;
        static const uint8_t peerDlyMsgMAC[ENET_MAC_ADDR_LEN] = {0x01, 0x80, 0xc2, 0x00, 0x00, 0x0E};
        memset(&setMcastInArgs, 0, sizeof(setMcastInArgs));
        memcpy(&setMcastInArgs.addr.addr[0U],
                          &peerDlyMsgMAC[0U],
                          ENET_MAC_ADDR_LEN);
        setMcastInArgs.addr.vlanId  = 0;
        setMcastInArgs.info.super = false;
        setMcastInArgs.info.numIgnBits = 0;
        setMcastInArgs.info.fwdState = CPSW_ALE_FWDSTLVL_FWD;
        setMcastInArgs.info.portMask = 3U;

        ENET_IOCTL_SET_INOUT_ARGS(&prms, &setMcastInArgs, &setMcastoutArgs);
        ENET_IOCTL(gNpacDatapathObj.hEnet,
                gNpacDatapathObj.coreId,
                            CPSW_ALE_IOCTL_ADD_MCAST,
                            &prms,
                            status);

        CpswAle_SetPolicerEntryOutArgs setPolicerEntryOutArgs;
        CpswAle_SetPolicerEntryInArgs setPolicerEntryInArgs;
        setPolicerEntryInArgs.policerMatch.policerMatchEnMask = CPSW_ALE_POLICER_MATCH_ETHERTYPE;
        setPolicerEntryInArgs.policerMatch.etherType = 0x88F7;
        setPolicerEntryInArgs.threadIdEn = true;
        setPolicerEntryInArgs.threadId = 0;
        setPolicerEntryInArgs.peakRateInBitsPerSec = 0U;
        setPolicerEntryInArgs.commitRateInBitsPerSec = 0U;

        ENET_IOCTL_SET_INOUT_ARGS(&prms, &setPolicerEntryInArgs, &setPolicerEntryOutArgs);
        ENET_IOCTL(gNpacDatapathObj.hEnet,
                   gNpacDatapathObj.coreId,
                   CPSW_ALE_IOCTL_SET_POLICER,
                   &prms,
                   status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("EnetApp_setCpswAleClassifier() failed CPSW_ALE_IOCTL_SET_POLICER: %d\n",
                            status);
        }
     }
    uint32_t* npac_flow_id_map = (uint32_t*) 0x540391A0;
    for (uint32_t flowId = 0; flowId < 4; flowId++)
    {
        CSL_REG_WR(npac_flow_id_map, 0xFFFFFFFF);
        npac_flow_id_map++;
    }

    /* Wait for link up */
    if (status == ENET_SOK)
    {
        status = EnetApp_waitForLinkUp();
    }

    /* Do packet transmission and reception */
    if (status == ENET_SOK)
    {
        EnetApp_createRxTxTasks();
        uint32_t printCntr = 0;

        while (SystemP_SUCCESS != SemaphoreP_pend(&gNpacDatapathObj.rxDoneSemObj, SystemP_NO_WAIT))
        {
        	SemaphoreP_pend(&gNpacDatapathObj.rxDoneSemObj, SystemP_NO_WAIT);
            printCntr++;
            EnetApp_rxIsrFxn(NULL);
            EnetApp_txIsrFxn(NULL);
            ClockP_usleep(2000);
            if (printCntr == 1000)
            {
                EnetApp_showCpswStats();
                printCntr = 0;
            }
        }
        SemaphoreP_pend(&gNpacDatapathObj.txDoneSemObj, SystemP_WAIT_FOREVER);
    }

    /* Print network statistics */
    if (status == ENET_SOK)
    {
        if (Enet_isCpswFamily(gNpacDatapathObj.enetType))
        {
            EnetApp_showCpswStats();
        }
    }

    /* Print DMA statistics */
    if (status == ENET_SOK)
    {
        EnetAppUtils_showRxChStats(gNpacDatapathObj.hRxCh);
        EnetAppUtils_showTxChStats(gNpacDatapathObj.hTxCh);
    }

    /* Close Enet DMA driver */
    // EnetApp_closeDma();

    /*Detach Core*/
    EnetApp_coreDetach(gNpacDatapathObj.enetType, gNpacDatapathObj.instId, gNpacDatapathObj.coreId, gNpacDatapathObj.coreKey);

    /*Release Handle Info*/
    // EnetApp_releaseHandleInfo(gNpacDatapathObj.enetType, gNpacDatapathObj.instId);
    gNpacDatapathObj.hEnet = NULL;

    //EnetApp_driverDeInit();

    /* Disable peripheral clocks */
    EnetAppUtils_disableClocks(gNpacDatapathObj.enetType, gNpacDatapathObj.instId);

    /* Delete all TX/RX semaphores */
    SemaphoreP_destruct(&gNpacDatapathObj.rxSemObj);
    SemaphoreP_destruct(&gNpacDatapathObj.rxDoneSemObj);
    SemaphoreP_destruct(&gNpacDatapathObj.txSemObj);
    SemaphoreP_destruct(&gNpacDatapathObj.txDoneSemObj);

    EnetAppUtils_print("Test complete: %s\r\n", (status == ENET_SOK) ? "PASS" : "FAIL");

    return status;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static void EnetApp_createRxTxTasks(void)
{
    TaskP_Params taskParams;
    int32_t status;

    TaskP_Params_init(&taskParams);
    taskParams.priority       = 3U;
    taskParams.stack          = gNpacDatapathTaskStackRx;
    taskParams.stackSize      = sizeof(gNpacDatapathTaskStackRx);
    taskParams.args           = (void*)&gNpacDatapathObj.rxSemObj;
    taskParams.name           = "Rx Task";
    taskParams.taskMain       = &EnetApp_rxTask;

    status = TaskP_construct(&gNpacDatapathObj.rxTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);

    TaskP_Params_init(&taskParams);
    taskParams.priority       = 2U;
    taskParams.stack          = gNpacDatapathTaskStackTx;
    taskParams.stackSize      = sizeof(gNpacDatapathTaskStackTx);
    taskParams.args           = (void*)&gNpacDatapathObj.txSemObj;
    taskParams.name           = "Tx Task";
    taskParams.taskMain       = &EnetApp_txTask;

    status = TaskP_construct(&gNpacDatapathObj.txTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);
}

static uint32_t EnetApp_retrieveFreeTxPkts(void)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    int32_t status;
    uint32_t txFreeQCnt = 0U;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any CPSW packets that may be free now */
    status = EnetDma_retrieveTxPktQ(gNpacDatapathObj.hTxCh, &txFreeQ);
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

            EnetQueue_enq(&gNpacDatapathObj.txFreePktInfoQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        }
    }
    else
    {
        EnetAppUtils_print("retrieveFreeTxPkts() failed to retrieve pkts: %d\r\n",
                           status);
    }

    return txFreeQCnt;
}

static void EnetApp_txTask(void *args)
{
    EnetDma_PktQ txSubmitQ;
    EnetDma_Pkt *pktInfo;
    EthFrame *frame;
    uint32_t txRetrievePktCnt;
    uint32_t loopCnt, pktCnt;
    int32_t status = ENET_SOK;
    uint8_t bcastAddr[ENET_MAC_ADDR_LEN] = {0x00, 0x1B, 0x63, 0x84, 0x45, 0xE6};//{0xffU, 0xffU, 0xffU, 0xffU, 0xffU, 0xffU};

    gNpacDatapathObj.totalTxCnt = 0U;
    for (loopCnt = 0U; loopCnt < NPAC_DATAPATH_NUM_ITERATION; loopCnt++)
    {
        pktCnt = 0U;
        while (pktCnt < NPAC_DATAPATH_TEST_PKT_NUM)
        {
            /* Transmit a single packet */
            EnetQueue_initQ(&txSubmitQ);

            /* Dequeue one free TX Eth packet */
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gNpacDatapathObj.txFreePktInfoQ);

            while (NULL != pktInfo)
            {
                pktCnt++;
                /* Fill the TX Eth frame with test content */
                frame = (EthFrame *)pktInfo->sgList.list[0].bufPtr;
                memcpy(frame->hdr.dstMac, bcastAddr, ENET_MAC_ADDR_LEN);
                memcpy(frame->hdr.srcMac, &gNpacDatapathObj.hostMacAddr[0U], ENET_MAC_ADDR_LEN);
                frame->hdr.etherType = Enet_htons(ETHERTYPE_EXPERIMENTAL1);
                memset(&frame->payload[0U], (uint8_t)(0xA5 + pktCnt), NPAC_DATAPATH_TEST_PKT_LEN);

                pktInfo->sgList.list[0].segmentFilledLen = NPAC_DATAPATH_TEST_PKT_LEN + sizeof(EthFrameHeader);
                pktInfo->sgList.numScatterSegments = 1;
                pktInfo->chkSumInfo = 0U;
                pktInfo->appPriv    = &gNpacDatapathObj;
                EnetDma_checkPktState(&pktInfo->pktState,
                                      ENET_PKTSTATE_MODULE_APP,
                                      ENET_PKTSTATE_APP_WITH_FREEQ,
                                      ENET_PKTSTATE_APP_WITH_DRIVER);

                /* Enqueue the packet for later transmission */
                EnetQueue_enq(&txSubmitQ, &pktInfo->node);

                if (pktCnt >= NPAC_DATAPATH_TEST_PKT_NUM)
                {
                    break;
                }

                /* Dequeue one free TX Eth packet */
                pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gNpacDatapathObj.txFreePktInfoQ);
            }

            while (0U != EnetQueue_getQCount(&txSubmitQ))
            {
                uint32_t txCnt = EnetQueue_getQCount(&txSubmitQ);
                status = EnetDma_submitTxPktQ(gNpacDatapathObj.hTxCh,
                                              &txSubmitQ);
                SemaphoreP_pend(&gNpacDatapathObj.txSemObj, SystemP_WAIT_FOREVER);

                /* Retrieve TX free packets */
                if (status == ENET_SOK)
                {
                    txCnt            = txCnt - EnetQueue_getQCount(&txSubmitQ);
                    txRetrievePktCnt = 0U;
                    while (txRetrievePktCnt != txCnt)
                    {
                        /* This is not failure as HW is busy sending packets, we
                         * need to wait and again call retrieve packets */
                        ClockP_usleep(1000);
                        txRetrievePktCnt += EnetApp_retrieveFreeTxPkts();
                    }
                }
                else
                {
                    break;
                }
            }
        }

        gNpacDatapathObj.totalTxCnt += pktCnt;
    }

    EnetAppUtils_print("Transmitted %d packets \r\n", gNpacDatapathObj.totalTxCnt);
    SemaphoreP_post(&gNpacDatapathObj.txDoneSemObj);

    EnetAppUtils_print("Delete EnetApp_txTask() and exit..\r\n");
    TaskP_destruct(&gNpacDatapathObj.txTaskObj);
    TaskP_exit();
}

static uint32_t EnetApp_receivePkts(void)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_Pkt *pktInfo;
    int32_t status;
    uint32_t rxReadyCnt = 0U;

    EnetQueue_initQ(&rxReadyQ);

    /* Retrieve any CPSW packets which are ready */
    status = EnetDma_retrieveRxPktQ(gNpacDatapathObj.hRxCh, &rxReadyQ);
    if (status == ENET_SOK)
    {
        rxReadyCnt = EnetQueue_getQCount(&rxReadyQ);

        /* Queue the received packet to rxReadyQ and pass new ones from rxFreeQ
        **/
        pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        while (pktInfo != NULL)
        {
            EnetDma_checkPktState(&pktInfo->pktState,
                                  ENET_PKTSTATE_MODULE_APP,
                                  ENET_PKTSTATE_APP_WITH_DRIVER,
                                  ENET_PKTSTATE_APP_WITH_READYQ);

            EnetQueue_enq(&gNpacDatapathObj.rxReadyQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        }
    }
    else
    {
        EnetAppUtils_print("receivePkts() failed to retrieve pkts: %d\r\n",
                           status);
    }

    return rxReadyCnt;
}

static bool EnetLpbk_verifyRxFrame(EnetDma_Pkt *pktInfo, uint8_t rxCnt)
{
    uint8_t *rxPayload;
    EthFrame *rxframe;
    uint8_t verifyRxpkt = 0xA5+rxCnt;
    bool retval = false;
    uint32_t i,j;
    uint32_t segmentLen, headerLen;
    bool incorrectPayload = false;

    rxframe = (EthFrame *)pktInfo->sgList.list[0U].bufPtr;
    rxPayload = rxframe->payload;

    if (pktInfo->sgList.numScatterSegments == 1)
    {
        for (i = 0; i < NPAC_DATAPATH_TEST_PKT_LEN; i++)
        {
            if((rxPayload[i] != verifyRxpkt))
            {
                retval = false;
                break;
            }
            retval = true;
        }
    }
    else
    {
        headerLen = rxPayload - pktInfo->sgList.list[0U].bufPtr;
        for (i = 0; i < pktInfo->sgList.numScatterSegments; i++)
        {
            segmentLen = pktInfo->sgList.list[i].segmentFilledLen;
            if(i == 0)
            {
                segmentLen -= headerLen;
            }
            else
            {
                rxPayload = pktInfo->sgList.list[i].bufPtr;
            }
            for (j = 0; j < segmentLen; j++)
            {
                if((rxPayload[j] != verifyRxpkt))
                {
                    retval = false;
                    incorrectPayload = true;
                    break;
                }
                retval = true;
            }
            if(incorrectPayload == true)
            {
                break;
            }
        }
    }

    return retval;
}

static void EnetApp_rxTask(void *args)
{
    EnetDma_Pkt *pktInfo;
    EthFrame *frame;
    uint32_t rxReadyCnt;
    uint32_t loopCnt, loopRxPktCnt;
    int32_t status = ENET_SOK;
    uint32_t rxPktCnt;

    gNpacDatapathObj.totalRxCnt = 0U;

    for (loopCnt = 0U; loopCnt < NPAC_DATAPATH_NUM_ITERATION; loopCnt++)
    {
        loopRxPktCnt = 0U;
        rxPktCnt     = 0U;
        /* Wait for packet reception */
        do
        {
            SemaphoreP_pend(&gNpacDatapathObj.rxSemObj, SystemP_WAIT_FOREVER);
            /* Get the packets received so far */
            rxReadyCnt = EnetApp_receivePkts();
            if (rxReadyCnt > 0U)
            {
                /* Consume the received packets and release them */
                pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gNpacDatapathObj.rxReadyQ);
                while (NULL != pktInfo)
                {
                    rxPktCnt++;
                    EnetDma_checkPktState(&pktInfo->pktState,
                                          ENET_PKTSTATE_MODULE_APP,
                                          ENET_PKTSTATE_APP_WITH_READYQ,
                                          ENET_PKTSTATE_APP_WITH_FREEQ);

                    /* Consume the packet by just printing its content */
                    if (gNpacDatapathObj.printFrame)
                    {
                        uint32_t packetPrintLen;

                        frame = (EthFrame *)pktInfo->sgList.list[0].bufPtr;
                        packetPrintLen = pktInfo->sgList.list[0].segmentFilledLen - sizeof(EthFrameHeader);

                        EnetAppUtils_printFrame(frame,
                                                packetPrintLen);
                    }
                    (void)rxPktCnt;
                    /* Release the received packet */
                    EnetQueue_enq(&gNpacDatapathObj.rxFreeQ, &pktInfo->node);
                    pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gNpacDatapathObj.rxReadyQ);
                }

                /*Submit now processed buffers */
                if (status == ENET_SOK)
                {
                    EnetAppUtils_validatePacketState(&gNpacDatapathObj.rxFreeQ,
                                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                                     ENET_PKTSTATE_APP_WITH_DRIVER);

                    EnetDma_submitRxPktQ(gNpacDatapathObj.hRxCh,
                                         &gNpacDatapathObj.rxFreeQ);
                }
            }

            loopRxPktCnt += rxReadyCnt;
        }
        while (loopRxPktCnt < NPAC_DATAPATH_TEST_PKT_NUM);

        gNpacDatapathObj.totalRxCnt += loopRxPktCnt;
    }

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to transmit/receive packets: %d, transmitted: %d \r\n", NPAC_DATAPATH_TEST_PKT_NUM, gNpacDatapathObj.totalRxCnt);
    }
    else
    {
        EnetAppUtils_print("Received %d packets\r\n", gNpacDatapathObj.totalRxCnt);
    }

    SemaphoreP_post(&gNpacDatapathObj.rxDoneSemObj);

    EnetAppUtils_print("Delete EnetApp_rxTask() and exit..\r\n");
    TaskP_destruct(&gNpacDatapathObj.rxTaskObj);
    TaskP_exit();
}

void EnetApp_updateCpswInitCfg(Enet_Type enetType,  uint32_t instId, Cpsw_Cfg *cpswCfg)
{
    CpswHostPort_Cfg *hostPortCfg = &cpswCfg->hostPortCfg;
    CpswAle_Cfg *aleCfg = &cpswCfg->aleCfg;
    CpswCpts_Cfg *cptsCfg = &cpswCfg->cptsCfg;

    /* Set Enet global runtime log level */
    Enet_setTraceLevel(ENET_TRACE_DEBUG);

    /* Peripheral config */
    cpswCfg->vlanCfg.vlanAware = false;

    /* Host port config */
    hostPortCfg->removeCrc      = true;
    hostPortCfg->padShortPacket = true;
    hostPortCfg->passCrcErrors  = true;

    /* ALE config */
    aleCfg->modeFlags                          = CPSW_ALE_CFG_MODULE_EN;
    aleCfg->agingCfg.autoAgingEn               = true;
    aleCfg->agingCfg.agingPeriodInMs           = 1000;
    aleCfg->nwSecCfg.vid0ModeEn                = true;
    aleCfg->vlanCfg.aleVlanAwareMode           = false;
    aleCfg->vlanCfg.cpswVlanAwareMode          = false;
    aleCfg->vlanCfg.unknownUnregMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownRegMcastFloodMask   = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownVlanMemberListMask  = CPSW_ALE_ALL_PORTS_MASK;

    /* CPTS config */
    /* Note: Timestamping and MAC loopback are not supported together because of
     * IP limitation, so disabling timestamping for this application */
    cptsCfg->hostRxTsEn = false;

}

static void EnetApp_closeEnet(void)
{
    Enet_IoctlPrms prms;
    int32_t status;

    /* Close port link */
    ENET_IOCTL_SET_IN_ARGS(&prms, &gNpacDatapathObj.macPort);

    ENET_IOCTL(gNpacDatapathObj.hEnet, gNpacDatapathObj.coreId, ENET_PER_IOCTL_CLOSE_PORT_LINK, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to close port link: %d\r\n", status);
    }

    /* Close Enet driver */
    Enet_close(gNpacDatapathObj.hEnet);

    gNpacDatapathObj.hEnet = NULL;
}



static int32_t EnetApp_waitForLinkUp(void)
{
    Enet_IoctlPrms prms;
    bool linked = false;
    int32_t status = ENET_SOK;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &gNpacDatapathObj.macPort, &linked);

    while (!linked)
    {
        ENET_IOCTL(gNpacDatapathObj.hEnet, gNpacDatapathObj.coreId, ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to get port %u's link status: %d\r\n",
                            ENET_MACPORT_ID(gNpacDatapathObj.macPort), status);
            linked = false;
            break;
        }

        if (!linked)
        {
            /* wait for 50 ms and poll again*/
            ClockP_usleep(50000);
        }
    }

    return status;
}

static void EnetApp_showCpswStats(void)
{
    Enet_IoctlPrms prms;
    CpswStats_PortStats portStats;
    int32_t status;

    /* Show host port statistics */
    ENET_IOCTL_SET_OUT_ARGS(&prms, &portStats);
    ENET_IOCTL(gNpacDatapathObj.hEnet, gNpacDatapathObj.coreId, ENET_STATS_IOCTL_GET_HOSTPORT_STATS, &prms, status);
    if (status == ENET_SOK)
    {
        EnetAppUtils_print("\r\n Port 0 Statistics\r\n");
        EnetAppUtils_print("-----------------------------------------\r\n");
        EnetAppUtils_printHostPortStats2G((CpswStats_HostPort_2g *)&portStats);
        EnetAppUtils_print("\r\n");
    }
    else
    {
        EnetAppUtils_print("Failed to get host stats: %d\r\n", status);
    }

    /* Show NPAC port statistics */
    ENET_IOCTL_SET_OUT_ARGS(&prms, &portStats);
    ENET_IOCTL(gNpacDatapathObj.hEnet, gNpacDatapathObj.coreId, ENET_STATS_IOCTL_GET_NPACPORT_STATS, &prms, status);
    if (status == ENET_SOK)
    {
        EnetAppUtils_print("\r\n Port 0_Hn Statistics\r\n");
        EnetAppUtils_print("-----------------------------------------\r\n");
        EnetAppUtils_printNpacPortStats((CpswStats_NpacPort_Ng *)&portStats);
        EnetAppUtils_print("\r\n");
    }
    else
    {
        EnetAppUtils_print("Failed to get host stats: %d\r\n", status);
    }


    /* Show MAC port statistics */
    if (status == ENET_SOK)
    {
        gNpacDatapathObj.macPort = ENET_MAC_PORT_2;
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &gNpacDatapathObj.macPort, &portStats);
        ENET_IOCTL(gNpacDatapathObj.hEnet, gNpacDatapathObj.coreId, ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
        if (status == ENET_SOK)
        {
            EnetAppUtils_print("\r\n Port 2 Statistics\r\n");
            EnetAppUtils_print("-----------------------------------------\r\n");
            EnetAppUtils_printMacPortStats2G((CpswStats_MacPort_2g *)&portStats);
            EnetAppUtils_print("\r\n");
        }
        else
        {
            EnetAppUtils_print("Failed to get MAC stats: %d\r\n", status);
        }
    }

    if (status == ENET_SOK)
    {
        gNpacDatapathObj.macPort = ENET_MAC_PORT_1;
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &gNpacDatapathObj.macPort, &portStats);
        ENET_IOCTL(gNpacDatapathObj.hEnet, gNpacDatapathObj.coreId, ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
        if (status == ENET_SOK)
        {
            EnetAppUtils_print("\r\n Port 1 Statistics\r\n");
            EnetAppUtils_print("-----------------------------------------\r\n");
            EnetAppUtils_printMacPortStats2G((CpswStats_MacPort_2g *)&portStats);
            EnetAppUtils_print("\r\n");
        }
        else
        {
            EnetAppUtils_print("Failed to get MAC stats: %d\r\n", status);
        }
    }
}

static int32_t EnetApp_macMode2PhyMii(emac_mode macMode,
                                    EnetPhy_Mii *mii)
{
    int32_t status = ENET_SOK;

    switch (macMode)
    {
        case MII:
            *mii = ENETPHY_MAC_MII_MII;
            break;
        case RMII:
            *mii = ENETPHY_MAC_MII_RMII;
            break;

        case RGMII:
            *mii = ENETPHY_MAC_MII_RGMII;
            break;
        default:
            status = ENET_EFAIL;
            EnetAppUtils_print("Invalid MAC mode: %u\r\n", macMode);
            EnetAppUtils_assert(false);
            break;
    }

    return status;
}

static void EnetApp_macMode2MacMii(emac_mode macMode,
                                    EnetMacPort_Interface *mii)
{
    switch (macMode)
    {
        case MII:
            mii->layerType    = ENET_MAC_LAYER_MII;
            mii->sublayerType = ENET_MAC_SUBLAYER_STANDARD;
            mii->variantType  = ENET_MAC_VARIANT_NONE;
        break;
        case RMII:
            mii->layerType    = ENET_MAC_LAYER_MII;
            mii->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
            mii->variantType  = ENET_MAC_VARIANT_NONE;
            break;

        case RGMII:
            mii->layerType    = ENET_MAC_LAYER_GMII;
            mii->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
            mii->variantType  = ENET_MAC_VARIANT_FORCED;
            break;
        default:
            EnetAppUtils_print("Invalid MAC mode: %u\r\n", macMode);
            EnetAppUtils_assert(false);
            break;
    }
}

static void EnetApp_rxIsrFxn(void *appData)
{
    SemaphoreP_post(&gNpacDatapathObj.rxSemObj);
}

static void EnetApp_txIsrFxn(void *appData)
{
    SemaphoreP_post(&gNpacDatapathObj.txSemObj);
}

static void EnetApp_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] =
    {
       ENET_MEM_LARGE_POOL_PKT_SIZE,
    };

    /* Initialize all queues */
    EnetQueue_initQ(&gNpacDatapathObj.txFreePktInfoQ);

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < ENET_SYSCFG_TOTAL_NUM_TX_PKT; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gNpacDatapathObj,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&gNpacDatapathObj.txFreePktInfoQ, &pPktInfo->node);
    }

    EnetAppUtils_print("initQs() txFreePktInfoQ initialized with %d pkts\r\n",
                       EnetQueue_getQCount(&gNpacDatapathObj.txFreePktInfoQ));
}

static void EnetApp_initRxReadyPktQ(void)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_Pkt *pPktInfo;
    int32_t status;
    uint32_t i;
    uint32_t scatterSegments[] =
    {
       ENET_MEM_LARGE_POOL_PKT_SIZE,
    };

    EnetQueue_initQ(&gNpacDatapathObj.rxFreeQ);
    EnetQueue_initQ(&gNpacDatapathObj.rxReadyQ);
    EnetQueue_initQ(&rxReadyQ);

    for (i = 0U; i < ENET_SYSCFG_TOTAL_NUM_RX_PKT; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gNpacDatapathObj,
                                        ENETDMA_CACHELINE_ALIGNMENT,
                                        ENET_ARRAYSIZE(scatterSegments),
                                        scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);
        EnetQueue_enq(&gNpacDatapathObj.rxFreeQ, &pPktInfo->node);
    }

    /* Retrieve any CPSW packets which are ready */
    status = EnetDma_retrieveRxPktQ(gNpacDatapathObj.hRxCh, &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);
    /* There should not be any packet with DMA during init */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&gNpacDatapathObj.rxFreeQ,
                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                     ENET_PKTSTATE_APP_WITH_DRIVER);

    EnetDma_submitRxPktQ(gNpacDatapathObj.hRxCh,
                         &gNpacDatapathObj.rxFreeQ);

    /* Assert here as during init no. of DMA descriptors should be equal to
     * no. of free Ethernet buffers available with app */

    EnetAppUtils_assert(0U == EnetQueue_getQCount(&gNpacDatapathObj.rxFreeQ));
}

static int32_t EnetApp_openDma(void)
{
    int32_t status = ENET_SOK;

    /* Open the CPSW TX channel  */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs     txInArgs;
        EnetApp_GetTxDmaHandleOutArgs  txChInfo; 

        txInArgs.cbArg   = &gNpacDatapathObj;
        txInArgs.notifyCb = EnetApp_txIsrFxn;

        EnetApp_getTxDmaHandle(ENET_DMA_TX_CH0,
                               &txInArgs,
                               &txChInfo);

        gNpacDatapathObj.hTxCh   = txChInfo.hTxCh;


        EnetApp_initTxFreePktQ();

        if (NULL != gNpacDatapathObj.hTxCh)
        {
            status = ENET_SOK;
            if (ENET_SOK != status)
            {
                EnetAppUtils_print("EnetUdma_startTxCh() failed: %d\r\n", status);
                status = ENET_EFAIL;
            }
        }
        else
        {
            EnetAppUtils_print("EnetDma_openTxCh() failed to open: %d\r\n",
                               status);
            status = ENET_EFAIL;
        }
    }

    /* Open the CPSW RX flow  */
    if (status == ENET_SOK)
    {
        EnetApp_GetRxDmaHandleOutArgs  rxChInfo;
        EnetApp_GetDmaHandleInArgs     rxInArgs;

        rxInArgs.notifyCb = EnetApp_rxIsrFxn;
        rxInArgs.cbArg   = &gNpacDatapathObj;

        EnetApp_getRxDmaHandle(ENET_DMA_RX_CH0,
                              &rxInArgs,
                              &rxChInfo);
        gNpacDatapathObj.hRxCh  = rxChInfo.hRxCh;
        EnetAppUtils_assert(rxChInfo.numValidMacAddress == 1);
        EnetUtils_copyMacAddr(gNpacDatapathObj.hostMacAddr, &rxChInfo.macAddr[rxChInfo.numValidMacAddress-1][0]);
        if (NULL == gNpacDatapathObj.hRxCh)
        {
            EnetAppUtils_print("EnetDma_openRxCh() failed to open: %d\r\n",
                               status);
            EnetAppUtils_assert(NULL != gNpacDatapathObj.hRxCh);
        }
        else
        {
            /* Submit all ready RX buffers to DMA.*/
            EnetApp_initRxReadyPktQ();
        }
    }

    return status;
}

static void EnetApp_closeDma(void)
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;

    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* There should not be any ready packet */
    EnetAppUtils_assert(0U == EnetQueue_getQCount(&gNpacDatapathObj.rxReadyQ));

    /* Close RX channel */
    EnetApp_closeRxDma(ENET_DMA_RX_CH0,
                       gNpacDatapathObj.hEnet,
                       gNpacDatapathObj.coreKey,
                       gNpacDatapathObj.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    /* Close TX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    EnetApp_closeTxDma(ENET_DMA_TX_CH0,
                       gNpacDatapathObj.hEnet,
                       gNpacDatapathObj.coreKey,
                       gNpacDatapathObj.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&gNpacDatapathObj.rxFreeQ);
    EnetAppUtils_freePktInfoQ(&gNpacDatapathObj.txFreePktInfoQ);

}
