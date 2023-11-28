/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/**
 * Copyright (c) 2018 Texas Instruments Incorporated
 *
 * This file is dervied from the ``ethernetif.c'' skeleton Ethernet network
 * interface driver for lwIP.
 *
 */

/* Standard language headers */
#include <stdio.h>
#include <assert.h>
#include <string.h>

/* xdc header - should be first file included as due to order dependency */

/* OS/Posix headers */

/**
 * lwIP specific header files
 */
#include "lwip/opt.h"
#include "lwip/pbuf.h"
#include "custom_pbuf.h"
#include "lwip2enet.h"
#include "lwip/inet_chksum.h"
#include "lwip/prot/ip.h"
#include <lwip/netif.h>
#include <networking/enet/core/lwipif/inc/lwipif2enet_AppIf.h>

/**
 * Enet device specific header files
 */
#include <enet.h>

#include <enet_appmemutils.h>
#include <networking/enet/core/include/core/enet_utils.h>
#include <enet_cfg.h>
#include "enet_apputils.h"
#include <networking/enet/core/lwipif/src/lwip2lwipif_priv.h>
#include <kernel/nortos/dpl/common/printf.h>


/*---------------------------------------------------------------------------*\
 |                             Extern Declarations                             |
 \*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*\
 |                            Local Macros/Defines                             |
 \*---------------------------------------------------------------------------*/

#define LWIP2ENET_UNKNOWN_MAC_PORT (-1)

/*---------------------------------------------------------------------------*\
 |                         Local Function Declarations                         |
 \*---------------------------------------------------------------------------*/

static void Lwip2Enet_deinitTxObj(Enet_Type enetType, uint32_t instId, Lwip2Enet_TxHandle hTx);

static void Lwip2Enet_deinitRxObj(Enet_Type enetType, uint32_t instId, Lwip2Enet_RxHandle hRx);

static void Lwip2Enet_saveAppIfCfg(Lwip2Enet_Handle hLwip2Enet,
                                   Lwip2Enet_AppInfo *appInfo);

static void Lwip2Enet_initTxObj(Enet_Type enetType, uint32_t instId, uint32_t chEntryIdx, Lwip2Enet_TxObj* pTx);

static void Lwip2Enet_initRxObj(Enet_Type enetType, uint32_t instId, uint32_t chEntryIdx, Lwip2Enet_RxHandle hRx);

static void Lwip2Enet_mapNetif2Tx(struct netif *netif,
                                  bool isDirected,
                                  Lwip2Enet_TxHandle hTxLwip2Enet,
                                  Lwip2Enet_Handle hLwip2Enet);

static void Lwip2Enet_mapNetif2Rx(struct netif *netif,
                                  bool isDirected,
                                  Lwip2Enet_RxHandle hRxLwip2Enet,
                                  Lwip2Enet_Handle hLwip2Enet);

static void Lwip2Enet_notifyRxPackets(void *cbArg);

static void Lwip2Enet_notifyTxPackets(void *cbArg);

static void Lwip2Enet_pbufQ2PktInfoQ(Lwip2Enet_TxObj *tx,
                                     pbufQ *pbufPktQ,
                                     EnetDma_PktQ *pDmaPktInfoQ,
                                     Enet_MacPort macPort);

static void Lwip2Enet_pktInfoQ2PbufQ(EnetDma_PktQ *pDmaPktInfoQ,
                                     pbufQ *pbufPktQ);

static uint32_t Lwip2Enet_prepRxPktQ(Lwip2Enet_RxObj *rx,
                                    EnetDma_PktQ *pPktQ);

static void Lwip2Enet_submitRxPktQ(Lwip2Enet_RxObj *rx);

static uint32_t Lwip2Enet_prepTxPktQ(Lwip2Enet_TxObj *tx,
                                    EnetDma_PktQ *pPktQ);

static void Lwip2Enet_freePbufPackets(EnetDma_PktQ *tempQueue);

static void Lwip2Enet_updateRxNotifyStats(Lwip2Enet_PktTaskStats *pktTaskStats,
                                          uint32_t packetCount,
                                          uint32_t timeDiff);

static void Lwip2Enet_updateTxNotifyStats(Lwip2Enet_PktTaskStats *pktTaskStats,
                                          uint32_t packetCount,
                                          uint32_t timeDiff);

static void Lwip2Enet_print(Lwip2Enet_Handle hLwip2Enet,
                            const char *prnStr,
                            ...);

static int32_t Lwip2Enet_startPollTask(Lwip2Enet_Handle hLwip2Enet);

static void Lwip2Enet_stopRxTx(Lwip2Enet_TxObj* pTx, Lwip2Enet_RxObj* pRx);

static void Lwip2Enet_submitTxPackets(Lwip2Enet_TxObj *tx,
                                      EnetDma_PktQ *pSubmitQ);

static void Lwip2Enet_submitRxPackets(Lwip2Enet_RxObj *rx,
                                     EnetDma_PktQ *pSubmitQ);

void Lwip2Enet_retrieveTxPkts(Lwip2Enet_TxHandle hTx);

static void Lwip2Enet_timerCb(ClockP_Object *hClk, void * arg);

static void Lwip2Enet_createTimer(Lwip2Enet_Handle hLwip2Enet);

static void Lwip2Enet_initReleaseTxHandleInArgs(Lwip2Enet_TxHandle hTx,
        uint32_t chNum,
        LwipifEnetAppIf_ReleaseTxHandleInfo *inArgs);

static void Lwip2Enet_initReleaseRxHandleInArgs(Lwip2Enet_RxHandle hRx,
        uint32_t chNum,
        LwipifEnetAppIf_ReleaseRxHandleInfo *inArgs);

static uint32_t Lwip2Enet_getRxTxObjIdx(Enet_Type enetType, uint32_t instId, Enet_MacPort macPort);

static void Lwip2Enet_setSGList(EnetDma_Pkt *pCurrDmaPacket, struct pbuf *pbuf, bool isRx);

static void Lwip2Enet_freeRxPktCb(void *cbArg,
                                  EnetDma_PktQ *fqPktInfoQ,
                                  EnetDma_PktQ *cqPktInfoQ);

static void Lwip2Enet_freeTxPktCb(void *cbArg,
                                  EnetDma_PktQ *fqPktInfoQ,
                                  EnetDma_PktQ *cqPktInfoQ);

static int32_t Lwip2Enet_setMacAddress(const Enet_Type enetType, uint32_t instId, Enet_Handle hEnet,uint8_t macAddr[ENET_MAC_ADDR_LEN]);

Lwip2Enet_RxMode_t LwipifEnetAppCb_getRxMode(Enet_Type enetType, uint32_t instId);

bool Lwip2Enet_decrementRxRefCount(Lwip2Enet_RxObj* pRx);

bool Lwip2Enet_decrementTxRefCount(Lwip2Enet_TxObj* pTx);

void LwipifEnetApp_getRxChIDs(const Enet_Type enetType, const uint32_t instId, uint32_t netifIdx, uint32_t* pRxChIdCount, uint32_t rxChIdList[LWIPIF_MAX_RX_CHANNELS_PER_PHERIPHERAL]);

void LwipifEnetApp_getTxChIDs(const Enet_Type enetType, const uint32_t instId,  uint32_t netifIdx, uint32_t* pTxChIdCount, uint32_t txChIdList[LWIPIF_MAX_TX_CHANNELS_PER_PHERIPHERAL]);

uint32_t LwipifEnetAppCb_getMacPort(const Enet_Type enetType, const uint32_t instId);

/*---------------------------------------------------------------------------*\
 |                         Local Variable Declarations                         |
 \*---------------------------------------------------------------------------*/

static Lwip2Enet_Obj  gLwip2EnetObj =  {.isInitDone = false, .isAllocated = false };

/*---------------------------------------------------------------------------*\
 |                         Global Variable Declarations                        |
 \*---------------------------------------------------------------------------*/

static Lwip2Enet_Handle Lwip2Enet_allocateObj(void)
{
    uintptr_t key = EnetOsal_disableAllIntr();
    Lwip2Enet_Handle hLwip2Enet = &gLwip2EnetObj;

    if (!hLwip2Enet->isAllocated)
    {
        hLwip2Enet = &gLwip2EnetObj;
        //phn initialize hLwip2Enet
        hLwip2Enet->isAllocated = true;
        hLwip2Enet->isInitDone = false;

        for (uint32_t rxChIdIdx = 0; rxChIdIdx < LWIPIF_MAX_RX_CHANNELS; rxChIdIdx++)
        {
            hLwip2Enet->lwip2EnetRxObj[rxChIdIdx].refCount = 0;
        }

        for (uint32_t txChIdIdx = 0; txChIdIdx < LWIPIF_MAX_TX_CHANNELS; txChIdIdx++)
        {
            hLwip2Enet->lwip2EnetTxObj[txChIdIdx].refCount = 0;
        }
    }

    EnetOsal_restoreAllIntr(key);

    return hLwip2Enet;
}

void Lwip2Enet_assertPriv(bool cond, const int32_t line, const char* file)
{
    volatile static bool waitInLoop = TRUE;

    if (!(cond))
    {
        void EnetAppUtils_print(const char *pcString,...);
        EnetAppUtils_print("Assertion @ Line: %d in %s : failed !!!\r\n", line, file);
        while (waitInLoop)
        {

        }
    }
}
static Lwip2Enet_RxHandle Lwip2Enet_allocateRxHandle(Lwip2Enet_Handle hLwip2Enet, Enet_Type enetType, uint32_t instId, const uint32_t objIdx)
{
    uintptr_t key = EnetOsal_disableAllIntr();
    Lwip2Enet_RxHandle hRx = NULL;
    Lwip2Enet_assert(objIdx < LWIPIF_MAX_RX_CHANNELS);
    if (objIdx < LWIPIF_MAX_RX_CHANNELS)
    {
        hRx = &hLwip2Enet->lwip2EnetRxObj[objIdx];
    }

    EnetOsal_restoreAllIntr(key);

    return hRx;
}

static Lwip2Enet_TxHandle Lwip2Enet_allocateTxHandle(Lwip2Enet_Handle hLwip2Enet, Enet_Type enetType, uint32_t instId, const uint32_t objIdx)
{
    uintptr_t key = EnetOsal_disableAllIntr();
    Lwip2Enet_TxHandle hTx = NULL;
    Lwip2Enet_assert(objIdx < LWIPIF_MAX_TX_CHANNELS);
    if (objIdx < LWIPIF_MAX_TX_CHANNELS)
    {
        hTx = &hLwip2Enet->lwip2EnetTxObj[objIdx];
    }

    EnetOsal_restoreAllIntr(key);

    return hTx;
}


Enet_MacPort Lwip2Enet_findMacPortFromEnet(Enet_Type enetType, uint32_t instId) // move to Cb file autogenerated
{
    Enet_MacPort macPort = ENET_MAC_PORT_INV;
    if (enetType == ENET_ICSSG_DUALMAC)
    {
        macPort = (Enet_MacPort)(instId & 0x01);
    }
    return macPort;
}

Lwip2Enet_Handle Lwip2Enet_open(Enet_Type enetType, uint32_t instId, struct netif *netif, uint32_t netifIdx)
{
    uint32_t txChIdList[LWIPIF_MAX_TX_CHANNELS_PER_PHERIPHERAL];
    uint32_t rxChIdList[LWIPIF_MAX_RX_CHANNELS_PER_PHERIPHERAL];
    EnetApp_GetMacAddrOutArgs          macAddr;
    EnetApp_HandleInfo                 handleInfo;
    Lwip2Enet_Handle    hLwip2Enet = Lwip2Enet_allocateObj();
    Lwip2Enet_netif_t*  pInterface = &(hLwip2Enet->interfaceInfo[hLwip2Enet->numOpenedNetifs++]);
    uint32_t txChIdCount = 0, rxChIdCount = 0;

    /* get TxChID & RxChID and the corresponding channel counts */
    LwipifEnetApp_getTxChIDs(enetType, instId, netifIdx, &txChIdCount, &txChIdList[0]);
    LwipifEnetApp_getRxChIDs(enetType, instId, netifIdx, &rxChIdCount, &rxChIdList[0]);


    EnetApp_acquireHandleInfo(enetType, instId, &handleInfo);
    Lwip2Enet_assert(handleInfo.hEnet != NULL);
    pInterface->hEnet = handleInfo.hEnet;

    /* MCast List is EMPTY */
    hLwip2Enet->MCastCnt = 0U;

    /* Init internal bookkeeping fields */
    hLwip2Enet->oldMCastCnt = 0U;

    /* set the mac address of netif as invalid, as we will set the valid address below */
    netif->hwaddr_len = 0U;
    if (hLwip2Enet->isInitDone == false)
    {
        /* First init tasks, semaphores and clocks. This is required because
         * EnetDma event cb ISR can happen immediately after opening rx flow
         * in LwipifEnetAppCb_getHandle and all semaphores, clocks and tasks should
         * be valid at that point
         */

        LwipifEnetAppCb_getEnetLwipIfInstInfo(enetType, instId, &hLwip2Enet->appInfo);

        /* Save params received from application interface */
        Lwip2Enet_saveAppIfCfg(hLwip2Enet, &hLwip2Enet->appInfo);

        Lwip2Enet_assert(hLwip2Enet->appInfo.isPortLinkedFxn != NULL);
        Lwip2Enet_assert(hLwip2Enet->appInfo.pPbufInfo != NULL);

        pbufQ_init_freeQ(hLwip2Enet->appInfo.pPbufInfo, hLwip2Enet->appInfo.pPbufInfoSize);

        /* set the print function callback if not null */
        hLwip2Enet->print = (Enet_Print) &EnetUtils_printf;
        hLwip2Enet->isInitDone = true;
        Lwip2Enet_createTimer(hLwip2Enet);
    }

    /* Associate with corresponding Tx Channels */
    for (uint32_t txChIdIndex = 0; txChIdIndex < txChIdCount; txChIdIndex++)
    {
        const uint32_t txChId = txChIdList[txChIdIndex];
        pInterface->hTx[txChIdIndex] = Lwip2Enet_allocateTxHandle(hLwip2Enet, enetType, instId, txChId);
        Lwip2Enet_initTxObj(enetType, instId, txChId, pInterface->hTx[txChIdIndex]);
        pInterface->hTx[txChIdIndex]->hLwip2Enet = hLwip2Enet;
        Lwip2Enet_assert(NULL != pInterface->hTx[txChIdIndex]->hCh);
        if (pInterface->hTx[txChIdIndex]->refCount == 1)
        {
            hLwip2Enet->allocPktInfo += pInterface->hTx[txChIdIndex]->numPackets;
            EnetDma_enableTxEvent(pInterface->hTx[txChIdIndex]->hCh);
        }
    }
    pInterface->count_hTx = txChIdCount;

    /* Associate with corresponding Rx Channels */
    for (uint32_t rxChIdIdx = 0; rxChIdIdx < rxChIdCount; rxChIdIdx++)
    {
        const uint32_t rxChId = rxChIdList[rxChIdIdx];
        pInterface->hRx[rxChIdIdx] = Lwip2Enet_allocateRxHandle(hLwip2Enet, enetType, instId, rxChId);
        Lwip2Enet_initRxObj(enetType, instId, rxChId, pInterface->hRx[rxChIdIdx]);

        /* Process netif related parameters*/
        pInterface->hRx[rxChIdIdx]->mode = LwipifEnetAppCb_getRxMode(enetType, instId);
        if ((pInterface->hRx[rxChIdIdx]->mode == Lwip2Enet_RxMode_SwitchSharedChannel) ||
            (pInterface->hRx[rxChIdIdx]->mode == Lwip2Enet_RxMode_SwitchPort1Channel) ||
            (pInterface->hRx[rxChIdIdx]->mode == Lwip2Enet_RxMode_SwitchPort2Channel))
        {
            for (uint32_t portIdx = 0; portIdx < CPSW_STATS_MACPORT_MAX; portIdx++)
            {
                pInterface->hRx[rxChIdIdx]->mapPortToNetif[portIdx] = netif;
            }
            pInterface->macPort = ENET_MAC_PORT_INV;
        }
        else if (pInterface->hRx[rxChIdIdx]->mode == Lwip2Enet_RxMode_MacSharedChannel)
        {
            const Enet_MacPort macPort = (Enet_MacPort)((pInterface->hRx[rxChIdIdx]->refCount -1));
            Lwip2Enet_assert(macPort < LWIPIF_MAX_NUM_MAC_PORTS);
            pInterface->hRx[rxChIdIdx]->mapPortToNetif[macPort] = netif;
            pInterface->macPort = macPort;
        }
        else
        {
            /* rxChIdx is treated as MAC PORT in case of dedicated CH per netif.
             * This is in both ICSSG dual mac and switch usecase */
            const Enet_MacPort macPort = Lwip2Enet_findMacPortFromEnet(enetType, instId);
            Lwip2Enet_assert(macPort < LWIPIF_MAX_NUM_MAC_PORTS);
            pInterface->hRx[rxChIdIdx]->mapPortToNetif[macPort] = netif;
            pInterface->macPort = macPort;
        }

        // MAC address allocation for the Netifs
        EnetApp_getMacAddress(rxChId, &macAddr);
        if (macAddr.macAddressCnt > 0)
        {
            /* fall here only for the Rx Channel that has valid mac address */
            Lwip2Enet_assert(netif->hwaddr_len == 0);
            Lwip2Enet_assert(macAddr.macAddressCnt >= (pInterface->hRx[rxChIdIdx]->refCount - 1));
            EnetUtils_copyMacAddr(netif->hwaddr ,&macAddr.macAddr[pInterface->hRx[rxChIdIdx]->refCount - 1][0U]);
            netif->hwaddr_len = ENET_MAC_ADDR_LEN;
            #if ENET_ENABLE_PER_ICSSG
            Lwip2Enet_setMacAddress(enetType, instId, pInterface->hEnet, &macAddr.macAddr[pInterface->hRx[rxChIdIdx]->refCount - 1][0U]);
            #endif // ENET_ENABLE_PER_ICSSG
        }

        pInterface->hRx[rxChIdIdx]->hLwip2Enet = hLwip2Enet;
        Lwip2Enet_assert(NULL != pInterface->hRx[rxChIdIdx]->hFlow);
        /* Submit all allocated packets to DMA so it can use for packet RX */
        if (pInterface->hRx[rxChIdIdx]->refCount == 1)
        {
            hLwip2Enet->allocPktInfo += pInterface->hRx[rxChIdIdx]->numPackets;
            Lwip2Enet_submitRxPktQ(pInterface->hRx[rxChIdIdx]);
        }
    }
    pInterface->count_hRx = rxChIdCount;

    /* Get initial link/interface status from the driver */
    pInterface->isLinkUp = hLwip2Enet->appInfo.isPortLinkedFxn(pInterface->hEnet);
    pInterface->isPortLinkedFxn = hLwip2Enet->appInfo.isPortLinkedFxn;
    pInterface->pNetif   = netif;

    /* Updating the netif params */
    Lwip2Enet_assert(netif->hwaddr_len == ENET_MAC_ADDR_LEN);
    netif->state = (void *)pInterface;

    /* assert if clk period is not valid  */
    Lwip2Enet_assert(0U != hLwip2Enet->appInfo.timerPeriodUs);

    ClockP_start(&hLwip2Enet->pacingClkObj);
    return hLwip2Enet;
}


Lwip2Enet_netif_t* Lwip2Enet_getInterfaceObj(Lwip2Enet_Handle hLwip2Enet, struct netif *netif)
{
    Lwip2Enet_netif_t* pInterface = NULL;
    for (uint32_t ifIdx = 0; ifIdx < LWIPIF_MAX_NETIFS_SUPPORTED; ifIdx++)
    {
        if (hLwip2Enet->interfaceInfo[ifIdx].pNetif == netif)
        {
            pInterface = &hLwip2Enet->interfaceInfo[ifIdx];
            break;
        }
        else
        {
            continue;
        }
    }
    return pInterface;
}


/*!
 *  @b Lwip2Enet_close
 *  @n
 *      Closes Ethernet peripheral and disables interrupts.
 *
 *  \param[in]  hLwip2Enet
 *      Lwip2Enet_object structure pointer.
 *
 *  \retval
 *      void
 */
void Lwip2Enet_close(Lwip2Enet_Handle hLwip2Enet, struct netif *netif)
{
    Lwip2Enet_assert(NULL != hLwip2Enet);

    /* Stop and delete the tick timer */
    ClockP_stop(&hLwip2Enet->pacingClkObj);
    ClockP_destruct(&hLwip2Enet->pacingClkObj);

    Lwip2Enet_netif_t* pInterface = Lwip2Enet_getInterfaceObj(hLwip2Enet, netif);
    Lwip2Enet_assert(pInterface != NULL);
    Lwip2Enet_assert(pInterface->count_hTx <= LWIPIF_MAX_TX_CHANNELS_PER_PHERIPHERAL);
    Lwip2Enet_assert(pInterface->count_hRx <= LWIPIF_MAX_RX_CHANNELS_PER_PHERIPHERAL);

//    Lwip2Enet_stopRxTx(pInterface->hTx, pInterface->hRx);

    if (hLwip2Enet->allocPktInfo > 0U)
    {
        Lwip2Enet_print(hLwip2Enet, "Lwip2Enet_close() failed to retrieve all PktInfo\n");
    }

    /* Deinit TX and RX objects, delete task, semaphore, etc */
    for (uint32_t idx = 0; idx < pInterface->count_hTx; idx++)
    {
        Lwip2Enet_deinitTxObj(pInterface->hEnet->enetPer->enetType, pInterface->hEnet->enetPer->instId, pInterface->hTx[idx]);
        hLwip2Enet->allocPktInfo -= EnetQueue_getQCount(&pInterface->hTx[idx]->freePktInfoQ);
        pInterface->hTx[idx] = NULL;
    }

    for (uint32_t idx = 0; idx < pInterface->count_hTx; idx++)
    {
        Lwip2Enet_deinitRxObj(pInterface->hEnet->enetPer->enetType, pInterface->hEnet->enetPer->instId, pInterface->hRx[idx]);

        hLwip2Enet->allocPktInfo -= EnetQueue_getQCount(&pInterface->hRx[idx]->readyRxPktQ);
        hLwip2Enet->allocPktInfo -= EnetQueue_getQCount(&pInterface->hRx[idx]->freeRxPktInfoQ);
        pInterface->hRx[idx] = NULL;
    }
    pInterface->isActive = false;
    return;
}

static void Lwip2Enet_initRxObj(Enet_Type enetType, uint32_t instId, uint32_t chEntryIdx, Lwip2Enet_RxHandle hRx)
{

    if (hRx->refCount > 0)
    {
        hRx->refCount++;
    }
    else
    {
        LwipifEnetAppIf_GetRxHandleInArgs inArgs;
        LwipifEnetAppIf_RxHandleInfo outArgs = {0};

        inArgs.enetType        = enetType;
        inArgs.instId          = instId;
        inArgs.cbArg           = hRx;
        inArgs.notifyCb        = &Lwip2Enet_notifyRxPackets;
        inArgs.chId            = chEntryIdx;
        inArgs.pFreePbufInfoQ  = &hRx->freePbufInfoQ;
        inArgs.pReadyRxPktQ    = &hRx->readyRxPktQ;
        inArgs.pFreeRxPktInfoQ = &hRx->freeRxPktInfoQ;

        LwipifEnetAppCb_getRxHandleInfo(&inArgs, &outArgs);

        hRx->numPackets = outArgs.numPackets;
        hRx->flowIdx     = outArgs.rxFlowIdx;
        hRx->flowStartIdx = outArgs.rxFlowStartIdx;
        hRx->hFlow       = outArgs.hRxFlow;
        Lwip2Enet_assert(hRx->hFlow != NULL);
        hRx->disableEvent = outArgs.disableEvent;

        hRx->stats.freeAppPktEnq = outArgs.numPackets;

        hRx->refCount = 1U;
        hRx->chEntryIdx = chEntryIdx;
        for (uint32_t portIdx = 0; portIdx < CPSW_STATS_MACPORT_MAX; portIdx++)
        {
            hRx->mapPortToNetif[portIdx] = NULL;
        }
    }

    return;
}

static void Lwip2Enet_saveAppIfCfg(Lwip2Enet_Handle hLwip2Enet,
                                     Lwip2Enet_AppInfo *appInfo)
{

}

static void Lwip2Enet_deinitRxObj(Enet_Type enetType, uint32_t instId, Lwip2Enet_RxHandle hRx)
{
    if (Lwip2Enet_decrementRxRefCount(hRx))
    {
        LwipifEnetAppIf_ReleaseRxHandleInfo inArgs;
        inArgs.enetType        = enetType;
        inArgs.instId          = instId;
        inArgs.rxFreePktCb    = &Lwip2Enet_freeRxPktCb;
        inArgs.rxFreePktCbArg = hRx;
        inArgs.rxChNum        = hRx->refCount;
        LwipifEnetAppCb_releaseRxHandle(&inArgs);
        hRx->hFlow = NULL;
    }

}

static void Lwip2Enet_deinitTxObj(Enet_Type enetType, uint32_t instId, Lwip2Enet_TxHandle hTx)
{
    if (Lwip2Enet_decrementTxRefCount(hTx))
    {
        LwipifEnetAppIf_ReleaseTxHandleInfo inArgs;
        inArgs.enetType       = enetType;
        inArgs.instId         = instId;
        inArgs.txFreePktCb    = &Lwip2Enet_freeTxPktCb;
        inArgs.txFreePktCbArg = hTx;
        inArgs.txChNum        = hTx->refCount;

        LwipifEnetAppCb_releaseTxHandle(&inArgs);
        hTx->hCh = NULL;

        /* Free pbuf in ready queue */
        while (pbufQ_count(&hTx->readyPbufQ) != 0U)
        {
            struct pbuf *pbuf = pbufQ_deQ(&hTx->readyPbufQ);
            Lwip2Enet_assert(NULL != pbuf);
            pbuf_free(pbuf);
        }
    }
}

static void Lwip2Enet_initTxObj(Enet_Type enetType, uint32_t instId, uint32_t chEntryIdx, Lwip2Enet_TxObj* pTx)
{
    if(pTx->refCount > 0)
    {
        pTx->refCount++;
    }
    else
    {
        LwipifEnetAppIf_GetTxHandleInArgs inArgs;
        LwipifEnetAppIf_TxHandleInfo outArgs;

        inArgs.chId = chEntryIdx;
        inArgs.cbArg = pTx;
        inArgs.notifyCb = Lwip2Enet_notifyTxPackets;
        inArgs.pktInfoQ = &pTx->freePktInfoQ;
        inArgs.enetType = enetType;
        inArgs.instId = instId;
        LwipifEnetAppCb_getTxHandleInfo(&inArgs, &outArgs);

        pTx->numPackets = outArgs.numPackets;
        pTx->hCh = outArgs.hTxChannel;
        Lwip2Enet_assert(pTx->hCh != NULL);
        pTx->disableEvent = outArgs.disableEvent;

        pTx->stats.freeAppPktEnq = outArgs.numPackets;

        /* Initialize the TX pbuf queues */
        pbufQ_init(&pTx->readyPbufQ);
        pbufQ_init(&pTx->unusedPbufQ);

        pTx->refCount = 1U;
        pTx->chEntryIdx = chEntryIdx;
    }
    return;
}

static void Lwip2Enet_setSGList(EnetDma_Pkt *pCurrDmaPacket, struct pbuf *pbuf, bool isRx)
{
    struct pbuf *pbufNext = pbuf;
    uint32_t totalPacketFilledLen = 0U;

    pCurrDmaPacket->sgList.numScatterSegments = 0;
    while (pbufNext != NULL)
    {
        EnetDma_SGListEntry *list;

        Lwip2Enet_assert(pCurrDmaPacket->sgList.numScatterSegments < ENET_ARRAYSIZE(pCurrDmaPacket->sgList.list));
        list = &pCurrDmaPacket->sgList.list[pCurrDmaPacket->sgList.numScatterSegments];
        list->bufPtr = (uint8_t*) pbufNext->payload;
        list->segmentFilledLen = (isRx == true) ? 0U : pbufNext->len;
        list->segmentAllocLen = pbufNext->len;
        if ((pbufNext->type_internal == PBUF_ROM) || (pbufNext->type_internal == PBUF_REF))
        {
            list->disableCacheOps = true;
        }
        else
        {
            list->disableCacheOps = false;
        }
        totalPacketFilledLen += pbufNext->len;
        pCurrDmaPacket->sgList.numScatterSegments++;
        pbufNext = pbufNext->next;
    }
    Lwip2Enet_assert(totalPacketFilledLen == pbuf->tot_len);
}
/*!
 *  @b Lwip2Enet_sendTxPackets
 *  @n
 *      Routine to send out queued Tx packets to the hardware driver
 *
 *  \param[in]  hLwip2Enet
 *      Lwip2Enet_object structure pointer.
 *
 *  \retval
 *      void
 */
void Lwip2Enet_sendTxPackets(Lwip2Enet_netif_t* pInterface, const Enet_MacPort macPort)
{
    /* If link is not up, simply return */
    if (pInterface->isLinkUp)
    {
        EnetDma_PktQ txSubmitQ;
        Lwip2Enet_TxHandle hTx = pInterface->hTx[0];
        EnetQueue_initQ(&txSubmitQ);

        if (pbufQ_count(&hTx->unusedPbufQ))
        {
            /* send any pending TX Q's */
            Lwip2Enet_pbufQ2PktInfoQ(hTx, &hTx->unusedPbufQ, &txSubmitQ, macPort);
        }

        /* Check if there is anything to transmit, else simply return */
        while (pbufQ_count(&hTx->readyPbufQ) != 0U)
        {
            /* Dequeue one free TX Eth packet */
            EnetDma_Pkt *pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(&hTx->freePktInfoQ);

            if (pCurrDmaPacket == NULL)
            {
                /* If we run out of packet info Q, retrieve packets from HW
                * and try to dequeue free packet again */
                Lwip2Enet_retrieveTxPkts(hTx);
                pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(&hTx->freePktInfoQ);
            }

            if (NULL != pCurrDmaPacket)
            {
                struct pbuf *hPbufPkt = pbufQ_deQ(&hTx->readyPbufQ);
                EnetDma_initPktInfo(pCurrDmaPacket);
                Lwip2Enet_assert(hPbufPkt);
                Lwip2Enet_setSGList(pCurrDmaPacket, hPbufPkt, false);
                pCurrDmaPacket->appPriv    = hPbufPkt;
                pCurrDmaPacket->txPortNum  = macPort;
                pCurrDmaPacket->node.next  = NULL;
                pCurrDmaPacket->chkSumInfo = LWIPIF_LWIP_getChkSumInfo(hPbufPkt);

                ENET_UTILS_COMPILETIME_ASSERT(offsetof(EnetDma_Pkt, node) == 0U);
                EnetQueue_enq(&txSubmitQ, &(pCurrDmaPacket->node));

                LWIP2ENETSTATS_ADDONE(&hTx->stats.freeAppPktDeq);
                LWIP2ENETSTATS_ADDONE(&hTx->stats.readyPbufPktDeq);
            }
            else
            {
                break;
            }
        }

        /* Submit the accumulated packets to the hardware for transmission */
        Lwip2Enet_submitTxPackets(hTx, &txSubmitQ);
    }
}


/*!
 *  @b Lwip2Enet_ioctl
 *  @n
 *  Low level driver Ioctl interface. This interface can be used for
 *  ALE configuration, control, statistics
 *
 *  \param[in]  hLwip2Enet
 *      Lwip2Enet_object structure pointer.
 *  \param[in]  cmd
 *      Ioctl command.
 *  \param[in]  pBuf
 *      Ioctl buffer with commands and params to set/get
 *      configuration from hardware.
 *  \param[in]  size
 *      Size of Ioctl buffer.
 *
 *  \retval
 *      void
 */
int32_t Lwip2Enet_ioctl(Lwip2Enet_Handle hLwip2Enet,
                       uint32_t cmd,
                       void *param,
                       uint32_t size)
{

    int32_t retVal = 0U;

    // TODO - make IOCTL design consistent with Enet/Enet modules

    /* Decode the command and act on it */
    switch (cmd)
    {
        /* Rx Task Load IOCTL */
        case LWIP2ENET_IOCTL_GET_RXTASK_LOAD:
        {
            break;
        }

        /* Adding Tx Task Load  */
        case LWIP2ENET_IOCTL_GET_TXTASK_LOAD:
        {
            break;
        }

        default:
        {
            Lwip2Enet_assert(FALSE);
        }
    }

    return retVal;
}

/*---------------------------------------------------------------------------*\
 |                           Local Function Definitions                        |
 \*---------------------------------------------------------------------------*/

 void Lwip2Enet_periodicFxn(struct netif* netif)
{
    Lwip2Enet_netif_t* pInterface = (Lwip2Enet_netif_t*)netif->state;

    Lwip2Enet_assert(pInterface != NULL);
    Lwip2Enet_assert(pInterface->hRx != NULL);
    Lwip2Enet_assert(pInterface->hTx != NULL);
    uint32_t prevLinkState = pInterface->isLinkUp;

#if (1U == ENET_CFG_DEV_ERROR)
#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_UDMA)
    int32_t status;
#endif

    for(uint32_t i = 0U; i < pInterface->count_hTx; i++)
    {
        EnetQueue_verifyQCount(&pInterface->hTx[i]->freePktInfoQ);

#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_UDMA)
        status = EnetUdma_checkTxChSanity(pInterface->hTx[i]->hCh, 5U);
        if (status != ENET_SOK)
        {
            Lwip2Enet_print(pInterface->hTx[i]->hLwip2Enet, "EnetUdma_checkTxChSanity Failed\n");
        }
#endif
    }

    for(uint32_t i = 0U; i <  pInterface->count_hRx; i++)
    {
        EnetQueue_verifyQCount(&pInterface->hRx[i]->readyRxPktQ);

#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_UDMA)
        status = EnetUdma_checkRxFlowSanity(pInterface->hRx[i]->hFlow, 5U);
        if (status != ENET_SOK)
        {
            Lwip2Enet_print(pInterface->hRx[i]->hLwip2Enet, "EnetUdma_checkRxFlowSanity Failed\n");
        }
#endif
    }
#endif

    /*
     * Return the same DMA packets back to the DMA channel (but now
     * associated with a new PBUF Packet and buffer)
     */

    for (uint32_t idx = 0; idx < pInterface->count_hRx; idx++)
    {
        if (EnetQueue_getQCount(&pInterface->hRx[idx]->readyRxPktQ) > 0U)
        {
            Lwip2Enet_submitRxPktQ(pInterface->hRx[idx]);
        }
    }

    /* Get current link status as reported by the hardware driver */
    pInterface->isLinkUp = pInterface->isPortLinkedFxn(pInterface->hEnet);
    for (uint32_t idx = 0; idx < pInterface->count_hRx; idx++)
    {
        const uint32_t prevLinkInterface = pInterface->hRx[idx]->hLwip2Enet->currLinkedIf;
        // get the linked interface details.
        /* If the interface changed, discard any queue packets (since the MAC would now be wrong) */
        if (prevLinkInterface != pInterface->hRx[idx]->hLwip2Enet->currLinkedIf)
        {
            /* ToDo: Discard all queued packets */
        }
    }

    /* If link status changed from down->up, then send any queued packets */
    if ((prevLinkState == 0U) && (pInterface->isLinkUp))
    {
        const Enet_MacPort macPort = pInterface->macPort;
        Lwip2Enet_sendTxPackets(pInterface, macPort);
    }


#if 0 //The below CPU load profilling logic has to be re-worked for all OS variants
#if defined(LWIPIF_INSTRUMENTATION_ENABLED)
    static uint32_t loadCount = 0U;
    TaskP_Load stat;

    hLwip2Enet->stats.cpuLoad[loadCount] = TaskP_loadGetTotalCpuLoad();

    TaskP_loadGet(&hLwip2Enet->rxPacketTaskObj, &stat);
    for(uint32_t i = 0U; i < hLwip2Enet->numRxChannels; i++)
    {
        hLwip2Enet->rx[i].stats.pktStats.taskLoad[loadCount] = stat.cpuLoad;
    }

    TaskP_loadGet(&hLwip2Enet->txPacketTaskObj, &stat);
    for(uint32_t i = 0U; i < hLwip2Enet->numRxChannels; i++)
    {
        hLwip2Enet->tx[i].stats.pktStats.taskLoad[loadCount] = stat.cpuLoad;
    }

    loadCount = (loadCount + 1U) & (HISTORY_CNT - 1U);
#endif
#endif


}

static void Lwip2Enet_processRxUnusedQ(Lwip2Enet_RxObj *rx,
                                       EnetDma_PktQ *unUsedQ)
{
    EnetDma_Pkt *pCurrDmaPacket;

    pCurrDmaPacket = (EnetDma_Pkt*) EnetQueue_deq(unUsedQ);
    while (pCurrDmaPacket != NULL)
    {
        if ((pCurrDmaPacket->sgList.numScatterSegments <= 0)
                || (pCurrDmaPacket->sgList.numScatterSegments > ENET_ARRAYSIZE(pCurrDmaPacket->sgList.list))
                || (pCurrDmaPacket->sgList.list[0].bufPtr == NULL))
        {
            Lwip2Enet_assert(FALSE);
        }
        else
        {
            EnetQueue_enq(&rx->readyRxPktQ, &pCurrDmaPacket->node);
        }

        pCurrDmaPacket = (EnetDma_Pkt*) EnetQueue_deq(unUsedQ);
    }
}

static void Lwip2Enet_submitRxPackets(Lwip2Enet_RxObj *rx,
                                      EnetDma_PktQ *pSubmitQ)
{
    int32_t retVal;

    retVal = EnetDma_submitRxPktQ(rx->hFlow, pSubmitQ);
    if (ENET_SOK != retVal)
    {
        Lwip2Enet_print(rx->hLwip2Enet,
                        "EnetDma_submitRxPktQ: failed to submit pkts: %d\n",
                        retVal);
    }

    if (EnetQueue_getQCount(pSubmitQ))
    {
        /* Copy unused packets to back to readyQ */
        Lwip2Enet_processRxUnusedQ(rx, pSubmitQ);
    }
}

/* May lead to infinite loop if no free memory
 * available*/
static void Lwip2Enet_pbufQ2PktInfoQ(Lwip2Enet_TxObj *tx,
                                     pbufQ *pbufPktQ,
                                     EnetDma_PktQ *pDmaPktInfoQ,
                                     Enet_MacPort macPort)
{
    EnetDma_Pkt *pCurrDmaPacket;
    struct pbuf *hPbufPkt = NULL;

    while (pbufQ_count(pbufPktQ) != 0U)
    {

        /* Dequeue one free TX Eth packet */
        pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(&tx->freePktInfoQ);

        if (pCurrDmaPacket == NULL)
        {
            /* If we run out of packet info Q, retrieve packets from HW
            * and try to dequeue free packet again */
            Lwip2Enet_retrieveTxPkts(tx);
            pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(&tx->freePktInfoQ);
        }

        if (NULL != pCurrDmaPacket)
        {
            hPbufPkt = pbufQ_deQ(pbufPktQ);
            Lwip2Enet_assert(hPbufPkt);
            EnetDma_initPktInfo(pCurrDmaPacket);

            Lwip2Enet_setSGList(pCurrDmaPacket, hPbufPkt, false);
            pCurrDmaPacket->appPriv    = hPbufPkt;

            pCurrDmaPacket->node.next = NULL;
            pCurrDmaPacket->txPortNum  = macPort;
            pCurrDmaPacket->chkSumInfo = LWIPIF_LWIP_getChkSumInfo(hPbufPkt);

            ENET_UTILS_COMPILETIME_ASSERT(offsetof(EnetDma_Pkt, node) == 0U);
            EnetQueue_enq(pDmaPktInfoQ, &(pCurrDmaPacket->node));

            LWIP2ENETSTATS_ADDONE(&tx->stats.freeAppPktDeq);
        }
        else
        {
            break;
        }
    }
}

static void Lwip2Enet_pktInfoQ2PbufQ(EnetDma_PktQ *pDmaPktInfoQ,
                                     pbufQ *pbufPktQ)
{
    EnetDma_Pkt *pDmaPacket;
    struct pbuf *hPbufPacket;

    while (EnetQueue_getQCount(pDmaPktInfoQ) != 0U)
    {
        pDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(pDmaPktInfoQ);
        hPbufPacket = (struct pbuf *)(pDmaPacket->appPriv);

        Lwip2Enet_assert(hPbufPacket != NULL);
        /*Don't want to make a copy, since it would cause waste memory*/
        pbufQ_enQ(pbufPktQ, hPbufPacket);
    }
}

static void Lwip2Enet_submitTxPackets(Lwip2Enet_TxObj *tx,
                                      EnetDma_PktQ *pSubmitQ)
{
    int32_t retVal;

    retVal = EnetDma_submitTxPktQ(tx->hCh, pSubmitQ);
    if (ENET_SOK != retVal)
    {
        Lwip2Enet_print(tx->hLwip2Enet,
                        "EnetDma_submitTxPktQ: failed to submit pkts: %d\n",
                        retVal);
    }

    if (EnetQueue_getQCount(pSubmitQ))
    {
        /* TODO: txUnUsedPBMPktQ is needed for packets that were not able to be
         *       submitted to driver.  It can be removed if stack supported any
         *       mechanism to enqueue them to the head of the queue. */
        Lwip2Enet_pktInfoQ2PbufQ(pSubmitQ, &tx->unusedPbufQ);
        EnetQueue_append(&tx->freePktInfoQ, pSubmitQ);
        LWIP2ENETSTATS_ADDNUM(&tx->stats.freeAppPktEnq, EnetQueue_getQCount(pSubmitQ));
    }
}

static void Lwip2Enet_freePbufPackets(EnetDma_PktQ *tempQueue)
{
    EnetDma_Pkt *pCurrDmaPacket;
    struct pbuf* hPbufPacket;

    while (EnetQueue_getQCount(tempQueue))
    {
        pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(tempQueue);
        hPbufPacket     = (struct pbuf *)pCurrDmaPacket->appPriv;
        pbuf_free(hPbufPacket);
    }
}

static void Lwip2Enet_notifyRxPackets(void *cbArg)
{
    Lwip2Enet_RxHandle pRx = (Lwip2Enet_RxHandle)cbArg;

    /* do not post events if init not done or shutdown in progress */
    if (pRx->refCount > 0)
    {
        if (pRx->disableEvent)
        {
            EnetDma_disableRxEvent(pRx->hFlow);
        }
        if (pRx->rxPktNotify.cbFxn != NULL)
        {
            pRx->rxPktNotify.cbFxn(pRx->rxPktNotify.cbArg);
        }
    }
}

static void Lwip2Enet_notifyTxPackets(void *cbArg)
{
    Lwip2Enet_TxHandle pTx = (Lwip2Enet_TxHandle)cbArg;

    /* do not post events if init not done or shutdown in progress */
    if ((pTx->refCount > 0) && (pTx->txPktNotify.cbFxn != NULL))
    {
        /* Notify Callbacks to post event/semephore */
        pTx->txPktNotify.cbFxn(pTx->txPktNotify.cbArg);
    }
}

void Lwip2Enet_rxPktHandler(Lwip2Enet_RxHandle hRx)
{
    EnetDma_PktQ tempQueue;
    int32_t retVal;
    uint32_t pktCnt = 0U;

    LWIP2ENETSTATS_ADDONE(&hRx->stats.pktStats.rawNotifyCnt);

    /* Retrieve the used (filled) packets from the channel */
    {
        EnetQueue_initQ(&tempQueue);
        retVal = EnetDma_retrieveRxPktQ(hRx->hFlow, &tempQueue);
        if (ENET_SOK != retVal)
        {
            Lwip2Enet_print(hRx->hLwip2Enet,
                    "Lwip2Enet_rxPacketTask: failed to retrieve RX pkts: %d\n",
                    retVal);
        }
    }
    if (tempQueue.count == 0U)
    {
        LWIP2ENETSTATS_ADDONE(&hRx->stats.pktStats.zeroNotifyCnt);
    }

    /*
     * Call Lwip2Enet_prepRxPktQ() even if no packets were received.
     * This allows new packets to be submitted if PBUF buffers became
     * newly available and there were outstanding free packets.
     */
    {
        /*
         * Get all used Rx DMA packets from the hardware, then send the buffers
         * of those packets on to the LwIP stack to be parsed/processed.
         */
        pktCnt = Lwip2Enet_prepRxPktQ(hRx, &tempQueue);
    }

    /*
     * We don't want to time the semaphore post used to notify the LwIP stack as that may cause a
     * task transition. We don't want to time the semaphore pend, since that would time us doing
     * nothing but waiting.
     */
    if (pktCnt != 0U)
    {
        Lwip2Enet_updateRxNotifyStats(&hRx->stats.pktStats, pktCnt, 0U);
    }

    // ClockP_start(&hLwip2Enet->pacingClkObj);

    if (!hRx->disableEvent)
    {
        EnetDma_enableRxEvent(hRx->hFlow);
    }

}

void Lwip2Enet_txPktHandler(Lwip2Enet_TxHandle hTx)
{
    Lwip2Enet_retrieveTxPkts(hTx);
}

/*
 * Enqueue a new packet and make sure that buffer descriptors are properly linked.
 * NOTE: Not thread safe
 */
static void Lwip2Enet_submitRxPktQ(Lwip2Enet_RxObj *rx)
{
    EnetDma_PktQ resubmitPktQ;
    EnetDma_Pkt *pCurrDmaPacket = NULL;

    EnetQueue_initQ(&resubmitPktQ);
    pCurrDmaPacket = (EnetDma_Pkt*) EnetQueue_deq(&rx->readyRxPktQ);
    while (pCurrDmaPacket != NULL)
    {
        EnetDma_checkPktState(&pCurrDmaPacket->pktState,
                               ENET_PKTSTATE_MODULE_APP,
                               ENET_PKTSTATE_APP_WITH_READYQ,
                               ENET_PKTSTATE_APP_WITH_DRIVER);
        if ((pCurrDmaPacket->sgList.numScatterSegments <= 0)
                || (pCurrDmaPacket->sgList.numScatterSegments > ENET_ARRAYSIZE(pCurrDmaPacket->sgList.list))
                || (pCurrDmaPacket->sgList.list[0].bufPtr == NULL))
        {
            Lwip2Enet_assert(FALSE);
        }
        else
        {
            EnetQueue_enq(&resubmitPktQ, &pCurrDmaPacket->node);
        }

        pCurrDmaPacket = (EnetDma_Pkt*) EnetQueue_deq(&rx->readyRxPktQ);
    }
    if (EnetQueue_getQCount(&resubmitPktQ))
    {
        Lwip2Enet_submitRxPackets(rx, &resubmitPktQ);
    }

}
static inline struct pbuf * Lwip2Enet_setCustomPbuf(pbuf_layer l, u16_t length, pbuf_type type, struct pbuf_custom *p,
        void *payload_mem, u16_t payload_mem_len)
{
    return pbuf_alloced_custom(l, length, type, p, payload_mem, payload_mem_len);
}

static uint32_t Lwip2Enet_prepRxPktQ(Lwip2Enet_RxObj *rx,
                                     EnetDma_PktQ *pPktQ)
{
    uint32_t packetCount = 0U;
    EnetDma_Pkt *pCurrDmaPacket;
    bool isChksumError = false;
    uint32_t scatterSegmentIndex = 0U;
    EnetDma_SGListEntry *list = NULL;
    struct pbuf *hPbufPacket = NULL;
    Rx_CustomPbuf *cPbuf = NULL;
    Rx_CustomPbuf *cPbufPrev = NULL;

    pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(pPktQ);
    while (pCurrDmaPacket)
    {
        isChksumError = false;
        Lwip2Enet_assert(pCurrDmaPacket->sgList.numScatterSegments <= ENET_ARRAYSIZE(pCurrDmaPacket->sgList.list));
        Lwip2Enet_assert(pCurrDmaPacket->sgList.numScatterSegments != 0);
        if (pbufQ_count(&rx->freePbufInfoQ) >= pCurrDmaPacket->sgList.numScatterSegments)
        {
            EnetDma_checkPktState(&pCurrDmaPacket->pktState,
                                  ENET_PKTSTATE_MODULE_APP,
                                  ENET_PKTSTATE_APP_WITH_DRIVER,
                                  ENET_PKTSTATE_APP_WITH_FREEQ);
            scatterSegmentIndex = 0;
            Enet_MacPort rxPortNum = pCurrDmaPacket->rxPortNum;
            while (scatterSegmentIndex < pCurrDmaPacket->sgList.numScatterSegments)
            {
                list = &pCurrDmaPacket->sgList.list[scatterSegmentIndex];
                Lwip2Enet_assert(list->bufPtr != NULL);

                cPbuf  = (Rx_CustomPbuf *)pbufQ_deQ(&rx->freePbufInfoQ);
                LWIP2ENETSTATS_ADDONE(&rx->stats.freePbufPktDeq);
                if (scatterSegmentIndex == 0)
                {
                    /* store the head of the pbuf */
                    hPbufPacket = &(cPbuf->p.pbuf);
                }
                Lwip2Enet_assert(hPbufPacket != NULL);
                /* Fill the pbuf with the sg list data */
                if (Lwip2Enet_setCustomPbuf(PBUF_RAW, list->segmentFilledLen, PBUF_POOL, &(cPbuf->p), list->bufPtr, list->segmentAllocLen) == NULL)
                {
                    Lwip2Enet_assert(false);
                }
                cPbuf->orgBufLen = list->segmentAllocLen;
                cPbuf->orgBufPtr = list->bufPtr;
                cPbuf->alivePbufCount = pCurrDmaPacket->sgList.numScatterSegments;
                if (scatterSegmentIndex > 0)
                {
                    cPbufPrev->next = cPbuf;
                    pbuf_cat(hPbufPacket, &(cPbuf->p.pbuf));
                }
                if (scatterSegmentIndex == pCurrDmaPacket->sgList.numScatterSegments - 1)
                {
                    /* Make sure the next pointer of last pbuf is NULL */
                    cPbuf->p.pbuf.next = NULL;
                    /* Point the last to first. If there is only one segment, it points to itself. */
                    cPbuf->next = (Rx_CustomPbuf *) hPbufPacket;
                }
                cPbufPrev = cPbuf;
                scatterSegmentIndex++;
            }


            {
                Lwip2Enet_assert(hPbufPacket->payload != NULL);
                struct ip_hdr* pIpPkt = (struct ip_hdr* ) LWIPIF_LWIP_getIpPktStart((uint8_t*) hPbufPacket->payload);
                if (IPH_PROTO(pIpPkt) == IP_PROTO_UDPLITE)
                {
                    /* As HW is can not compute checksum offload for UDP-lite packet, trigger SW checksum validation */
                    isChksumError = LWIPIF_LWIP_UdpLiteValidateChkSum(hPbufPacket);
                }
                else
                {
                    /* We don't check if HW checksum offload is enabled while checking for checksum error
                     * as default value of this field when offload not enabled is false */
                    const uint32_t csumInfo =  pCurrDmaPacket->chkSumInfo;

                    if (ENETDMA_RXCSUMINFO_GET_IPV4_FLAG(csumInfo) ||
                        ENETDMA_RXCSUMINFO_GET_IPV6_FLAG(csumInfo))
                    {
                        isChksumError = ENETDMA_RXCSUMINFO_GET_CHKSUM_ERR_FLAG(csumInfo);
                    }
                }
            }

            EnetDma_initPktInfo(pCurrDmaPacket);
            EnetQueue_enq(&rx->freeRxPktInfoQ, &pCurrDmaPacket->node);
            LWIP2ENETSTATS_ADDONE(&rx->stats.freeAppPktEnq);

            if (!isChksumError)
            {
                struct netif* netif = NULL;
                /* Pass the received packet to the LwIP stack */
                switch (rx->mode)
                {
                case Lwip2Enet_RxMode_SwitchSharedChannel:
                case Lwip2Enet_RxMode_MacSharedChannel:
                {
                    Lwip2Enet_assert(rxPortNum < LWIPIF_MAX_NUM_MAC_PORTS);
                    netif = rx->mapPortToNetif[rxPortNum];
                    break;
                }
                case Lwip2Enet_RxMode_MacPort1Channel:
                case Lwip2Enet_RxMode_SwitchPort1Channel:
                {
                    netif = rx->mapPortToNetif[ENET_MAC_PORT_1];
                    break;
                }
                case Lwip2Enet_RxMode_MacPort2Channel:
                case Lwip2Enet_RxMode_SwitchPort2Channel:
                {
                    Lwip2Enet_assert(LWIPIF_MAX_NUM_MAC_PORTS == 2);
                    netif = rx->mapPortToNetif[LWIPIF_MAX_NUM_MAC_PORTS - 1];
                    break;
                }
                default:
                {
                    Lwip2Enet_assert(false);
                }
                }

                Lwip2Enet_assert(netif != NULL);
                LWIPIF_LWIP_input(rx, netif, hPbufPacket);
                packetCount++;
            }
            else
            {
                /* Free the pbuf as we are not submitting to the stack */
                pbuf_free(hPbufPacket);
                LWIP2ENETSTATS_ADDONE(&rx->stats.chkSumErr);
            }
        }
        else
        {
            /*! This should not occur as we have equal number of pbufInfos and bufptrs */
            Lwip2Enet_assert(FALSE);
        }
        pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(pPktQ);
    }

    /* return as many packets to driver as we can */
    Lwip2Enet_submitRxPktQ(rx);

    return packetCount;
}

static uint32_t Lwip2Enet_prepTxPktQ(Lwip2Enet_TxObj *tx,
                                     EnetDma_PktQ *pPktQ)
{
    uint32_t packetCount;
    EnetDma_Pkt *pCurrDmaPacket;
    struct pbuf* hPbufPacket;

    packetCount = EnetQueue_getQCount(pPktQ);

    pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(pPktQ);
    while (pCurrDmaPacket)
    {
        hPbufPacket = (struct pbuf *)pCurrDmaPacket->appPriv;

        Lwip2Enet_assert(hPbufPacket != NULL);
        /* Free PBUF buffer as it is transmitted by DMA now */
        pbuf_free(hPbufPacket);
        /* Return packet info to free pool */
        EnetQueue_enq(&tx->freePktInfoQ, &pCurrDmaPacket->node);
        pCurrDmaPacket = (EnetDma_Pkt *)EnetQueue_deq(pPktQ);
    }

    LWIP2ENETSTATS_ADDNUM(&tx->stats.freeAppPktEnq, packetCount);

    return packetCount;
}

static void Lwip2Enet_updateTxNotifyStats(Lwip2Enet_PktTaskStats *pktStats,
                                          uint32_t packetCount,
                                          uint32_t timeDiff)
{
#if defined(LWIPIF_INSTRUMENTATION_ENABLED)
    uint32_t notificationCount;
    uint32_t timePerPacket = timeDiff / packetCount;

    notificationCount = pktStats->dataNotifyCnt & (HISTORY_CNT - 1U);
    pktStats->dataNotifyCnt++;

    pktStats->totalPktCnt   += packetCount;
    pktStats->totalCycleCnt += timeDiff;

    pktStats->cycleCntPerNotify[notificationCount] = timeDiff;
    if (timeDiff > pktStats->cycleCntPerNotifyMax)
    {
        pktStats->cycleCntPerNotifyMax = timeDiff;
    }

    pktStats->pktsPerNotify[notificationCount] = packetCount;
    if (packetCount > pktStats->pktsPerNotifyMax)
    {
        pktStats->pktsPerNotifyMax = packetCount;
    }

    pktStats->cycleCntPerPkt[notificationCount] = timePerPacket;
    if (timePerPacket > pktStats->cycleCntPerPktMax)
    {
        pktStats->cycleCntPerPktMax = timePerPacket;
    }
#endif
}

static void Lwip2Enet_updateRxNotifyStats(Lwip2Enet_PktTaskStats *pktTaskStats,
                                          uint32_t packetCount,
                                          uint32_t timeDiff)
{
#if defined(LWIPIF_INSTRUMENTATION_ENABLED)
    uint32_t notificationCount;
    uint32_t timePerPacket = timeDiff / packetCount;

    notificationCount = pktTaskStats->dataNotifyCnt & (HISTORY_CNT - 1U);
    pktTaskStats->dataNotifyCnt++;

    pktTaskStats->totalPktCnt   += packetCount;
    pktTaskStats->totalCycleCnt += timeDiff;

    pktTaskStats->cycleCntPerNotify[notificationCount] = timeDiff;
    if (timeDiff > pktTaskStats->cycleCntPerNotifyMax)
    {
        pktTaskStats->cycleCntPerNotifyMax = timeDiff;
    }

    pktTaskStats->pktsPerNotify[notificationCount] = packetCount;
    if (packetCount > pktTaskStats->pktsPerNotifyMax)
    {
        pktTaskStats->pktsPerNotifyMax = packetCount;
    }

    pktTaskStats->cycleCntPerPkt[notificationCount] = timePerPacket;
    if (timePerPacket > pktTaskStats->cycleCntPerPktMax)
    {
        pktTaskStats->cycleCntPerPktMax = timePerPacket;
    }
#endif
}

static void Lwip2Enet_print(Lwip2Enet_Handle hLwip2Enet,
                            const char *prnStr,
                            ...)
{
    // TODO fix me
    if (NULL != hLwip2Enet->print)
    {
        /* Function is non- reentrant */
        va_list vaArgPtr;
        char *buf;

        buf = &hLwip2Enet->printBuf[0U];
        va_start(vaArgPtr, prnStr);
        vsnprintf(buf, ENET_CFG_PRINT_BUF_LEN, (const char *)prnStr, vaArgPtr);
        va_end(vaArgPtr);

        (*hLwip2Enet->print)("[LWIP2ENET] ");
        (*hLwip2Enet->print)(hLwip2Enet->printBuf);
    }
}

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

bool Lwip2Enet_decrementRxRefCount(Lwip2Enet_RxObj* pRx)
{
    if (pRx->refCount > 0)
    {
        pRx->refCount--;
    }

    return (pRx->refCount > 0);
}

bool Lwip2Enet_decrementTxRefCount(Lwip2Enet_TxObj* pTx)
{
    if (pTx->refCount > 0)
    {
        pTx->refCount--;
    }

    return (pTx->refCount > 0);

}

/*
 * Periodically polls for changes in the link status and updates both the abstraction layer
 * as well as the stack
 * arg : netif
 * arg1 : Semaphore
 */


static void Lwip2Enet_stopRxTx(Lwip2Enet_TxObj* pTx, Lwip2Enet_RxObj* pRx)
{

    /* Stop TX packet task */
    /* Post to tx packet task/event so that it will terminate (shutDownFlag flag is already set) */
    if (pTx->txPktNotify.cbFxn != NULL)
    {
        pTx->txPktNotify.cbFxn(pTx->txPktNotify.cbArg);
    }

    /* Stop RX packet task */
    /* Post to rx packet task/event so that it will terminate (shutDownFlag flag is already set) */
    if (pRx->rxPktNotify.cbFxn != NULL)
    {
        pRx->rxPktNotify.cbFxn(pRx->rxPktNotify.cbArg);
    }

}

static void Lwip2Enet_freeTxPktCb(void *cbArg,
                                  EnetDma_PktQ *fqPktInfoQ,
                                  EnetDma_PktQ *cqPktInfoQ)
{
    Lwip2Enet_TxObj *tx = (Lwip2Enet_TxObj *)cbArg;

    EnetQueue_append(&tx->freePktInfoQ, fqPktInfoQ);
    LWIP2ENETSTATS_ADDNUM(&tx->stats.freeAppPktEnq, EnetQueue_getQCount(fqPktInfoQ));
    Lwip2Enet_freePbufPackets(fqPktInfoQ);

    EnetQueue_append(&tx->freePktInfoQ, cqPktInfoQ);
    LWIP2ENETSTATS_ADDNUM(&tx->stats.freeAppPktEnq, EnetQueue_getQCount(cqPktInfoQ));
    Lwip2Enet_freePbufPackets(cqPktInfoQ);
}

static void Lwip2Enet_freeRxPktCb(void *cbArg,
                                  EnetDma_PktQ *fqPktInfoQ,
                                  EnetDma_PktQ *cqPktInfoQ)
{
    Lwip2Enet_RxObj *rx = (Lwip2Enet_RxObj *)cbArg;

    /* Now that we free PBUF buffers, push all freed pktInfo's into Rx
     * free Q */
    EnetQueue_append(&rx->readyRxPktQ, fqPktInfoQ);
    LWIP2ENETSTATS_ADDNUM(&rx->stats.freeAppPktEnq, EnetQueue_getQCount(fqPktInfoQ));

    EnetQueue_append(&rx->readyRxPktQ, cqPktInfoQ);
    LWIP2ENETSTATS_ADDNUM(&rx->stats.freeAppPktEnq, EnetQueue_getQCount(cqPktInfoQ));
}

void Lwip2Enet_retrieveTxPkts(Lwip2Enet_TxHandle hTx)
{
    EnetDma_PktQ tempQueue;
    uint32_t packetCount = 0U;
    int32_t retVal;

    LWIP2ENETSTATS_ADDONE(&hTx->stats.pktStats.rawNotifyCnt);
    packetCount = 0U;

    /* Retrieve the used (sent/empty) packets from the channel */
    {
        EnetQueue_initQ(&tempQueue);
        /* Retrieve all TX packets and keep them locally */
        retVal = EnetDma_retrieveTxPktQ(hTx->hCh, &tempQueue);
        if (ENET_SOK != retVal)
        {
            Lwip2Enet_print(hTx->hLwip2Enet,
                            "Lwip2Enet_retrieveTxPkts: Failed to retrieve TX pkts: %d\n",
                            retVal);
        }
    }

    if (tempQueue.count != 0U)
    {
        /*
         * Get all used Tx DMA packets from the hardware, then return those
         * buffers to the txFreePktQ so they can be used later to send with.
         */
        packetCount = Lwip2Enet_prepTxPktQ(hTx, &tempQueue);
    }
    else
    {
        LWIP2ENETSTATS_ADDONE(&hTx->stats.pktStats.zeroNotifyCnt);
    }

    if (packetCount != 0U)
    {
        Lwip2Enet_updateTxNotifyStats(&hTx->stats.pktStats, packetCount, 0U);
    }
}

static void Lwip2Enet_timerCb(ClockP_Object *hClk, void * arg)
{
#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_UDMA)
    /* Post semaphore to Rx and Tx Pkt handling task */
    Lwip2Enet_Handle hLwip2Enet = (Lwip2Enet_Handle)arg;

    if (hLwip2Enet->isInitDone)
    {
        for (uint32_t txChIdx = 0U; txChIdx < LWIPIF_MAX_TX_CHANNELS; txChIdx++)
        {
            if ((hLwip2Enet->lwip2EnetTxObj[txChIdx].refCount > 0) && (hLwip2Enet->lwip2EnetTxObj[txChIdx].txPktNotify.cbFxn != NULL))
            {
                hLwip2Enet->lwip2EnetTxObj[txChIdx].txPktNotify.cbFxn(hLwip2Enet->lwip2EnetTxObj[txChIdx].txPktNotify.cbArg);
            }
        }

        for (uint32_t rxChIdx = 0U; rxChIdx < LWIPIF_MAX_RX_CHANNELS; rxChIdx++)
        {
                if ((hLwip2Enet->lwip2EnetRxObj[rxChIdx].refCount > 0) && (hLwip2Enet->lwip2EnetRxObj[rxChIdx].rxPktNotify.cbFxn != NULL))
                {
                    hLwip2Enet->lwip2EnetRxObj[rxChIdx].rxPktNotify.cbFxn(hLwip2Enet->lwip2EnetRxObj[rxChIdx].rxPktNotify.cbArg);
                }
        }

    }
#endif
}

static void Lwip2Enet_createTimer(Lwip2Enet_Handle hLwip2Enet)
{
    ClockP_Params clkPrms;
    int32_t status;

    ClockP_Params_init(&clkPrms);
    clkPrms.start  = false;
    clkPrms.timeout = ClockP_usecToTicks(hLwip2Enet->appInfo.timerPeriodUs);
    clkPrms.period = ClockP_usecToTicks(hLwip2Enet->appInfo.timerPeriodUs);
    clkPrms.callback = &Lwip2Enet_timerCb;
    clkPrms.args = hLwip2Enet;

    status =  ClockP_construct(&hLwip2Enet->pacingClkObj, &clkPrms);
    Lwip2Enet_assert(status == SystemP_SUCCESS);
}

#if ENET_ENABLE_PER_ICSSG
static int32_t Lwip2Enet_setMacAddress(const Enet_Type enetType, uint32_t instId, Enet_Handle hEnet, uint8_t macAddr[ENET_MAC_ADDR_LEN])
{
    int32_t  status = ENET_SOK;
    uint32_t coreId = EnetSoc_getCoreId();
    /* Add port MAC entry in case of ICSSG dual MAC */
    if (ENET_ICSSG_DUALMAC == enetType)
    {
        Enet_IoctlPrms prms;
        IcssgMacPort_SetMacAddressInArgs inArgs;

        EnetUtils_copyMacAddr(&inArgs.macAddr[0U], &macAddr[0U]);
        inArgs.macPort = Lwip2Enet_findMacPortFromEnet(enetType, instId);

        ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
        ENET_IOCTL(hEnet, coreId, ICSSG_MACPORT_IOCTL_SET_MACADDR, &prms, status);

        if (status != ENET_SOK)
        {
            EnetAppUtils_print(
                    "Lwip2Enet_setMacAddress() failed ICSSG_MACPORT_IOCTL_ADD_INTERFACE_MACADDR: %d\r\n",
                    status);
        }
        EnetAppUtils_assert(status == ENET_SOK);
    }
    else if (ENET_ICSSG_SWITCH == enetType)
    {
        Enet_IoctlPrms prms;
        Icssg_MacAddr addr;

        /* Set host port's MAC address */
        EnetUtils_copyMacAddr(&addr.macAddr[0U], &macAddr[0U]);
        ENET_IOCTL_SET_IN_ARGS(&prms, &addr);
        {
            ENET_IOCTL(hEnet, coreId, ICSSG_HOSTPORT_IOCTL_SET_MACADDR, &prms,
                    status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print(
                        "EnetAppUtils_addHostPortEntry() failed ICSSG_HOSTPORT_IOCTL_SET_MACADDR: %d\r\n",
                        status);
            }
            EnetAppUtils_assert(status == ENET_SOK);
        }
    }
    else
    {
        EnetAppUtils_assert(false);
    }
    return status;
}
#endif

void Lwip2Enet_setRxNotifyCallback(Lwip2Enet_RxHandle hRx, Enet_notify_t *pRxPktNotify)
{
    hRx->rxPktNotify = *pRxPktNotify;
}

void Lwip2Enet_setTxNotifyCallback(Lwip2Enet_TxHandle hTx, Enet_notify_t *pTxPktNotify)
{
    hTx->txPktNotify = *pTxPktNotify;
}
