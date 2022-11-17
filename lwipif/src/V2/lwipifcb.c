/*
 * Copyright (c) 2001,2002 Florian Schulze.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the authors nor the names of the contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * test.c - This file is part of lwIP test
 *
 */

/* C runtime includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/SystemP.h>

#include <enet.h>
#include <networking/enet/core/include/per/cpsw.h>
#include <networking/enet/utils/include/enet_appmemutils_cfg.h>
#include <networking/enet/utils/include/enet_apputils.h>
#include <networking/enet/utils/include/enet_appmemutils.h>
#include <networking/enet/utils/include/enet_appboardutils.h>
#include <networking/enet/utils/include/enet_appsoc.h>
#include <networking/enet/utils/include/enet_apprm.h>

/*
 * Using CPSW defined macros for this specific file
 */
#undef htons
#undef ntohs
#undef htonl
#undef ntohl

#include <networking/enet/core/lwipif/inc/lwip2lwipif.h>
#include <networking/enet/core/lwipif/inc/lwipif2enet_AppIf.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define ENETLWIP_PACKET_POLL_PERIOD_US                                  (1000U)
#define ENETLWIP_USE_DEFAULT_FLOW                                       (true)

#if ENET_CFG_NETIF_MAX > 1U
#error "Multiple netif is not support in the version"
#endif
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                            Function Declaration                            */
/* ========================================================================== */

/* Temporary hack to hard code Tx and Rx DMA channel index until the lwipcb is migrated to syscfg generated code */
#define LWIP_ENET_TX_DMA_CH_IDX (0U)
#define LWIP_ENET_RX_DMA_CH_IDX (0U)



void LwipifEnetAppCb_getHandle(LwipifEnetAppIf_GetHandleInArgs *inArgs,
                               LwipifEnetAppIf_GetHandleOutArgs *outArgs)
{
    EnetPer_AttachCoreOutArgs attachInfo;
    uint32_t coreId          = EnetSoc_getCoreId();
    Enet_Type enetType;
    uint32_t instId;
    EnetApp_HandleInfo handleInfo;
    EnetApp_GetTxDmaHandleOutArgs  txChInfo;
    EnetApp_GetRxDmaHandleOutArgs  rxChInfo;
    EnetApp_GetDmaHandleInArgs     txInArgs;
    EnetApp_GetDmaHandleInArgs     rxInArgs;

    EnetApp_getEnetInstInfo(&enetType,
                            &instId);

    EnetApp_acquireHandleInfo(enetType, instId, &handleInfo);
    EnetApp_coreAttach(enetType,instId, coreId, &attachInfo);

#if ENET_CFG_IS_ON(CPSW_CSUM_OFFLOAD_SUPPORT)
    int32_t status;
    /* Confirm HW checksum offload is enabled when LWIP chksum offload is enabled */
    if (Enet_isCpswFamily(enetType))
    {
        Enet_IoctlPrms prms;
        bool csumOffloadFlg;
        ENET_IOCTL_SET_OUT_ARGS(&prms, &csumOffloadFlg);
        ENET_IOCTL(handleInfo.hEnet,
                   coreId,
                   ENET_HOSTPORT_IS_CSUM_OFFLOAD_ENABLED,
                   &prms,
                   status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("() Failed to get checksum offload info: %d\r\n", status);
        }

        EnetAppUtils_assert(true == csumOffloadFlg);
    }
#endif
    txInArgs.cbArg     = inArgs->txCfg.cbArg;
    txInArgs.notifyCb  = inArgs->txCfg.notifyCb;

    EnetApp_getTxDmaHandle(LWIP_ENET_TX_DMA_CH_IDX,
                          &txInArgs,
                          &txChInfo);

    outArgs->txInfo.txChNum  = txChInfo.txChNum;
    outArgs->txInfo.hTxChannel = txChInfo.hTxCh;
    EnetAppUtils_assert(inArgs->txCfg.numPackets <= txChInfo.maxNumTxPkts);
    /* Open RX Flow */

    rxInArgs.notifyCb  = inArgs->rxCfg.notifyCb;
    rxInArgs.cbArg     = inArgs->rxCfg.cbArg;

    /* To Do: workaround for handling number of RX channels. this should be taken from syscfg*/
    outArgs->rxInfo.numRxChannels = 1U;

    for (uint32_t i = 0U; i < outArgs->rxInfo.numRxChannels; i++)
    {
        EnetApp_getRxDmaHandle(i,
                              &rxInArgs,
                              &rxChInfo);
        EnetAppUtils_assert(inArgs->rxCfg.numPackets <= rxChInfo.maxNumRxPkts);
        outArgs->rxInfo.rxHandle[i].rxFlowIdx = rxChInfo.rxChNum;
        outArgs->rxInfo.rxHandle[i].hRxFlow  = rxChInfo.hRxCh;

        if(rxChInfo.macAddressValid)
        {
            EnetUtils_copyMacAddr(&outArgs->rxInfo.macAddr[i][0U], rxChInfo.macAddr);
        }
    }

    outArgs->coreId        = coreId;
    outArgs->coreKey       = attachInfo.coreKey;
    outArgs->hEnet         = handleInfo.hEnet;
    outArgs->hostPortRxMtu = attachInfo.rxMtu;
    ENET_UTILS_ARRAY_COPY(outArgs->txMtu, attachInfo.txMtu);
    outArgs->print           = &EnetAppUtils_print;
    outArgs->isPortLinkedFxn = &EnetApp_isPortLinked;
    outArgs->timerPeriodUs   = ENETLWIP_PACKET_POLL_PERIOD_US;
	outArgs->rxInfo.disableEvent = true;

    /*To Do: workaround for number of netifs allocated by Application, this will move to syscfg */
    outArgs->numNetif = 1U;


    outArgs->txInfo.txPortNum = ENET_MAC_PORT_INV;
    /* Use optimized processing where TX packets are relinquished in next TX submit call */
    outArgs->txInfo.disableEvent = true;

#if (ENET_CFG_NETIF_MAX == 2U)
    int32_t status;
    /* Allocating mac addresses for number of netifs supported*/
    status = EnetAppUtils_allocMac(handleInfo.hEnet,
                                attachInfo.coreKey,
                                coreId,
                                &outArgs->rxInfo.macAddr[1U][0U]);
    EnetAppUtils_assert(ENET_SOK == status);
    EnetAppUtils_addHostPortEntry(handleInfo.hEnet, coreId,  &outArgs->rxInfo.macAddr[1U][0U]);
#endif

    for (uint32_t i = 0U; i < outArgs->numNetif; i++)
    {
        EnetAppUtils_print("Host MAC address-%d : ",i);
        EnetAppUtils_printMacAddr(&outArgs->rxInfo.macAddr[i][0U]);
    }

}

void LwipifEnetAppCb_releaseHandle(LwipifEnetAppIf_ReleaseHandleInfo *releaseInfo)
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;
    Enet_Type enetType;
    uint32_t instId;

    EnetApp_getEnetInstInfo(&enetType,
                            &instId);

    /* Close TX channel */
    {
        EnetQueue_initQ(&fqPktInfoQ);
        EnetQueue_initQ(&cqPktInfoQ);
        EnetApp_closeTxDma(LWIP_ENET_TX_DMA_CH_IDX,
                           releaseInfo->hEnet,
                           releaseInfo->coreKey,
                           releaseInfo->coreId,
                           &fqPktInfoQ,
                           &cqPktInfoQ);
        releaseInfo->txFreePktCb(releaseInfo->txFreePktCbArg, &fqPktInfoQ, &cqPktInfoQ);
    }

    {
        /* Close RX Flow */
        EnetQueue_initQ(&fqPktInfoQ);
        EnetQueue_initQ(&cqPktInfoQ);

        EnetApp_closeRxDma(LWIP_ENET_RX_DMA_CH_IDX,
                        releaseInfo->hEnet,
                        releaseInfo->coreKey,
                        releaseInfo->coreId,
                        &fqPktInfoQ,
                        &cqPktInfoQ);

        releaseInfo->rxFreePktCb(releaseInfo->rxFreePktCbArg, &fqPktInfoQ, &cqPktInfoQ, 0U);
    }

    EnetApp_coreDetach(enetType, instId, releaseInfo->coreId, releaseInfo->coreKey);
    EnetApp_releaseHandleInfo(enetType, instId);
}


