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
 * \file  app_enethandler.c
 *
 * \brief This file provides the APIs for opening, closing and sending of data over the ethernet.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "app_enethandler.h"

#include <enet_apputils.h>
#include <enet_appmemutils.h>
#include <include/per/cpsw.h>
#include <kernel/dpl/ClockP.h>

#include "ti_enet_open_close.h"
#include "ti_enet_config.h"

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t EnetApp_waitForLinkUp(void);

static void EnetApp_showCpswStats(void);

static void EnetApp_txIsrFxn(void *appData);

static void EnetApp_initTxFreePktQ(void);

static int32_t EnetApp_openDma(void);

static void EnetApp_closeDma(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EnetApp_cfg gEnetApp;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

EnetApp_cfg* EnetApp_getAppCfg(){
    return &gEnetApp;
}

int32_t EnetApp_initApp(void)
{
    int32_t status = ENET_SOK;
    EnetApp_HandleInfo handleInfo;
    Enet_IoctlPrms prms;

    if (gEnetApp.enetType == ENET_CPSW_2G)
    {
        EnetAppUtils_print("CPSW_2G Test\r\n");
    }
    if (gEnetApp.enetType == ENET_CPSW_3G)
    {
        EnetAppUtils_print("CPSW_3G Test\r\n");
    }


    /* Local core id */
    gEnetApp.coreId = EnetSoc_getCoreId();

    EnetApp_driverInit();

    if (status == ENET_SOK)
    {
        status = EnetApp_driverOpen(gEnetApp.enetType, gEnetApp.instId);

        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open ENET: %d\r\n", status);
        }
    }

    EnetApp_acquireHandleInfo(gEnetApp.enetType, gEnetApp.instId, &handleInfo);
    gEnetApp.hEnet = handleInfo.hEnet;

    /* Attach the core with RM */
    if (status == ENET_SOK)
    {
        EnetPer_AttachCoreOutArgs attachCoreOutArgs;

        EnetApp_coreAttach(gEnetApp.enetType, gEnetApp.instId, gEnetApp.coreId, &attachCoreOutArgs);
        gEnetApp.coreKey = attachCoreOutArgs.coreKey;
    }

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
        uint32_t entryIdx;
    
        setUcastInArgs.addr.vlanId  = 0U;
        setUcastInArgs.info.portNum = CPSW_ALE_HOST_PORT_NUM;
        setUcastInArgs.info.blocked = false;
        setUcastInArgs.info.secure  = false;
        setUcastInArgs.info.super   = false;
        setUcastInArgs.info.ageable = false;
        setUcastInArgs.info.trunk   = false;
        EnetUtils_copyMacAddr(&setUcastInArgs.addr.addr[0U], gEnetApp.hostMacAddr);
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &setUcastInArgs, &entryIdx);

        ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, CPSW_ALE_IOCTL_ADD_UCAST, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to add ucast entry: %d\r\n", status);
        }
    }

    if (status == ENET_SOK)
    {
        EnetAppUtils_print("Waiting for link up...\r\n");
        status = EnetApp_waitForLinkUp();
    }
    EnetAppUtils_print("Link is %s\r\n", status == ENET_SOK ? "up" : "down");

    status = SemaphoreP_constructBinary(&gEnetApp.txSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    return status;
}

int32_t EnetApp_deinitApp(void)
{
    int32_t status = ENET_SOK;

    if (Enet_isCpswFamily(gEnetApp.enetType))
    {
        EnetApp_showCpswStats();
    }

    status = EnetAppUtils_showTxChStats(gEnetApp.hTxCh);

    EnetApp_closeDma();

    EnetApp_coreDetach(gEnetApp.enetType, gEnetApp.instId, gEnetApp.coreId, gEnetApp.coreKey);

    EnetApp_releaseHandleInfo(gEnetApp.enetType, gEnetApp.instId);
    gEnetApp.hEnet = NULL;

    EnetApp_driverDeInit();
   
    SemaphoreP_destruct(&gEnetApp.txSemObj);
    
    EnetAppUtils_print("De-init complete\r\n");
    
    return status;
}

int32_t EnetApp_sendFrame(uint8_t * header, uint32_t header_len, uint8_t * payload, uint32_t payload_len)
{
    EnetDma_Pkt *pktInfo;

    int32_t status = ENET_SOK;

    pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetApp.txFreePktInfoQ);

    pktInfo->sgList.list[0].bufPtr = header;
    pktInfo->sgList.list[0].segmentFilledLen = header_len;

    pktInfo->sgList.list[1].bufPtr = payload;
    pktInfo->sgList.list[1].segmentFilledLen = payload_len;

    pktInfo->sgList.numScatterSegments = 2;
    pktInfo->chkSumInfo = 0U;
    pktInfo->appPriv    = &gEnetApp;

    EnetDma_checkPktState(&pktInfo->pktState,
                            ENET_PKTSTATE_MODULE_APP,
                            ENET_PKTSTATE_APP_WITH_FREEQ,
                            ENET_PKTSTATE_APP_WITH_DRIVER);

    status = EnetDma_submitTxPkt(gEnetApp.hTxCh,pktInfo);

    if (status == ENET_SOK)
    {
        SemaphoreP_pend(&gEnetApp.txSemObj, SystemP_WAIT_FOREVER);

        status = EnetDma_retrieveTxPkt(gEnetApp.hTxCh, &pktInfo);

        if(status==ENET_SOK){
            EnetQueue_enq(&gEnetApp.txFreePktInfoQ, &pktInfo->node);
        }
    }
    
    return status;
}


/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */


void EnetApp_updateCpswInitCfg(Enet_Type enetType,  uint32_t instId, Cpsw_Cfg *cpswCfg)
{

}

static int32_t EnetApp_waitForLinkUp(void)
{
    Enet_IoctlPrms prms;
    bool linked = false;
    int32_t status = ENET_SOK;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &gEnetApp.macPort, &linked);

    while (!linked)
    {
        ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to get port %u's link status: %d\r\n",
                            ENET_MACPORT_ID(gEnetApp.macPort), status);
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
    ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, ENET_STATS_IOCTL_GET_HOSTPORT_STATS, &prms, status);
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

    /* Show MAC port statistics */
    if (status == ENET_SOK)
    {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &gEnetApp.macPort, &portStats);
        ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
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

static void EnetApp_txIsrFxn(void *appData)
{
    SemaphoreP_post(&gEnetApp.txSemObj);
}

static void EnetApp_rxIsrFxn(void *appData)
{
    EnetAppUtils_print("RX ISR: should not be called\r\n");
}

static void EnetApp_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;

    /* Initialize all queues */
    EnetQueue_initQ(&gEnetApp.txFreePktInfoQ);

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < ENET_SYSCFG_TOTAL_NUM_TX_PKT; i++)
    {
        pPktInfo = EnetMem_allocEthPktInfoMem(&gEnetApp,
                                       ENETDMA_CACHELINE_ALIGNMENT);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&gEnetApp.txFreePktInfoQ, &pPktInfo->node);
    }

    EnetAppUtils_print("initQs() txFreePktInfoQ initialized with %d pkts\r\n",
                       EnetQueue_getQCount(&gEnetApp.txFreePktInfoQ));
}

static int32_t EnetApp_openDma(void)
{
    int32_t status = ENET_SOK;

    /* Open the CPSW TX channel  */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs     txInArgs;
        EnetApp_GetTxDmaHandleOutArgs  txChInfo; 

        txInArgs.cbArg   = &gEnetApp;
        txInArgs.notifyCb = EnetApp_txIsrFxn;

        EnetApp_getTxDmaHandle(ENET_DMA_TX_CH0,
                               &txInArgs,
                               &txChInfo);

        gEnetApp.hTxCh   = txChInfo.hTxCh;


        if (NULL != gEnetApp.hTxCh)
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

    EnetApp_initTxFreePktQ();

    if (status == ENET_SOK)
    {
        EnetApp_GetRxDmaHandleOutArgs  rxChInfo;
        EnetApp_GetDmaHandleInArgs     rxInArgs;

        rxInArgs.notifyCb = EnetApp_rxIsrFxn;
        rxInArgs.cbArg   = &gEnetApp;

        EnetApp_getRxDmaHandle(ENET_DMA_RX_CH0,
                              &rxInArgs,
                              &rxChInfo);
        gEnetApp.hRxCh  = rxChInfo.hRxCh;
        EnetAppUtils_assert(rxChInfo.numValidMacAddress == 1);
        EnetUtils_copyMacAddr(gEnetApp.hostMacAddr, &rxChInfo.macAddr[rxChInfo.numValidMacAddress-1][0]);
        if (NULL == gEnetApp.hRxCh)
        {
            EnetAppUtils_print("EnetDma_openRxCh() failed to open: %d\r\n",
                               status);
            EnetAppUtils_assert(NULL != gEnetApp.hRxCh);
        }
    }

    return status;
}

static void EnetApp_closeDma(void)
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;

    /* Close TX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    EnetApp_closeTxDma(ENET_DMA_TX_CH0,
                       gEnetApp.hEnet,
                       gEnetApp.coreKey,
                       gEnetApp.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    /* Close RX channel */
    EnetApp_closeRxDma(ENET_DMA_RX_CH0,
                       gEnetApp.hEnet,
                       gEnetApp.coreKey,
                       gEnetApp.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&gEnetApp.txFreePktInfoQ);
}


