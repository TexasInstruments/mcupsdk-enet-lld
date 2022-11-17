/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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
 *  \file enet_cpdma_priv.c
 *
 *  \brief This file contains CPDMA private function definitions.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <include/core/enet_base.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_soc.h>
#include <include/core/enet_per.h>
#include <include/core/enet_queue.h>
#include <priv/core/enet_base_priv.h>
#include <priv/core/enet_trace_priv.h>

#include <include/dma/cpdma/enet_cpdma.h>
#include <include/core/enet_dma.h>
#include <include/core/enet_dma_pktutils.h>

#include "enet_cpdma_priv.h"
#include "enet_cdma_memcfg.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

typedef EnetQ EnetCpdma_RxChObjMemQ;

typedef EnetQ EnetCpdma_TxChObjMemQ;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */


/**
 *  \brief
 */
typedef struct EnetCpdma_MemMgrObj_s
{
    /*! Cpdma memory object config info populated by app callback EnetCpdmaMem_getCfg */
    const EnetCpdma_MemCfg *cpdmaMemCfg;
    
    /*! RX flow driver object Q */
    EnetCpdma_RxChObjMemQ rxChObjMemQ;

    /*! TX channel driver object Q */
    EnetCpdma_TxChObjMemQ txChObjMemQ;

} EnetCpdma_MemMgrObj;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Cpsw mem utils driver object */
static EnetCpdma_MemMgrObj gCpswDmaMemMgrObj;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*! Rx flow object allocation function  */
EnetCpdma_RxChObj *EnetCpdma_memMgrAllocRxChObj(void)
{
    EnetCpdma_RxChObj *pRxCh = NULL;
    EnetCpdma_RxChObjMem *pRxChMemObj;

    pRxChMemObj = (EnetCpdma_RxChObjMem *)EnetQueue_deq(&gCpswDmaMemMgrObj.rxChObjMemQ);
    pRxCh    = &pRxChMemObj->rxChObj;

    return pRxCh;
}

/*! Rx flow object free function  */
void EnetCpdma_memMgrFreeRxChObj(EnetCpdma_RxChObj *pRxCh)
{
    EnetCpdma_RxChObjMem *pRxChMemObj;

#if ENET_CFG_IS_ON(DEV_ERROR)
    Enet_assert(NULL != pRxCh);
#endif

    pRxChMemObj = container_of(pRxCh, EnetCpdma_RxChObjMem, rxChObj);
    EnetQueue_enq(&gCpswDmaMemMgrObj.rxChObjMemQ, &pRxChMemObj->node);
    pRxChMemObj = NULL;

    return;
}

/*! TX channel object allocation function  */
EnetCpdma_TxChObj *EnetCpdma_memMgrAllocTxChObj(void)
{
    EnetCpdma_TxChObj *pTxChObj = NULL;
    EnetCpdma_TxChObjMem *pTxChMemObj;

    pTxChMemObj = (EnetCpdma_TxChObjMem *)EnetQueue_deq(&gCpswDmaMemMgrObj.txChObjMemQ);
    pTxChObj    = &pTxChMemObj->txChObj;

    return pTxChObj;
}

/*! TX channel object free function  */
void EnetCpdma_memMgrFreeTxChObj(EnetCpdma_TxChObj *pTxChObj)
{
    EnetCpdma_TxChObjMem *pTxChMemObj;

#if ENET_CFG_IS_ON(DEV_ERROR)
    Enet_assert(NULL != pTxChObj);
#endif

    pTxChMemObj = container_of(pTxChObj, EnetCpdma_TxChObjMem, txChObj);
    EnetQueue_enq(&gCpswDmaMemMgrObj.txChObjMemQ, &pTxChMemObj->node);
    pTxChObj = NULL;

    return;
}

void EnetCpdma_memMgrInit(void)
{
    uint32_t i;
    uintptr_t rxObjMemBase;
    uint32_t  rxObjMemSize;
    uintptr_t txObjMemBase;
    uint32_t  txObjMemSize;
    EnetCpdma_RxChObjMem *rxObj;
    EnetCpdma_TxChObjMem *txObj;

    memset(&gCpswDmaMemMgrObj, 0U, sizeof(gCpswDmaMemMgrObj));
    gCpswDmaMemMgrObj.cpdmaMemCfg = EnetCpdmaMem_getCfg();
    Enet_assert(NULL != gCpswDmaMemMgrObj.cpdmaMemCfg);

    rxObjMemBase = (uintptr_t)gCpswDmaMemMgrObj.cpdmaMemCfg->rxChObjMemCfg.rxChObjMemContainerBase;
    rxObjMemSize = gCpswDmaMemMgrObj.cpdmaMemCfg->rxChObjMemCfg.rxChObjMemContainerSize;
    txObjMemBase = (uintptr_t)gCpswDmaMemMgrObj.cpdmaMemCfg->txChObjMemCfg.txChObjMemContainerBase;
    txObjMemSize = gCpswDmaMemMgrObj.cpdmaMemCfg->txChObjMemCfg.txChObjMemContainerSize;



    memset((void *)rxObjMemBase, 0U, rxObjMemSize);
    memset((void *)txObjMemBase, 0U, txObjMemSize);

    /*********************** Rx flow object memory  Q ************************/
    EnetQueue_initQ(&gCpswDmaMemMgrObj.rxChObjMemQ);
    rxObj = gCpswDmaMemMgrObj.cpdmaMemCfg->rxChObjMemCfg.rxChObjMemContainerBase;
    for (i = 0U; i < gCpswDmaMemMgrObj.cpdmaMemCfg->rxChObjMemCfg.numRxCh; i++)
    {
        Enet_assert(((uintptr_t)rxObj + sizeof(*rxObj)) <= (rxObjMemBase + rxObjMemSize));
        EnetQueue_enq(&gCpswDmaMemMgrObj.rxChObjMemQ, &rxObj->node);
        rxObj++;
    }

    /*********************** Tx channel object memory  Q ************************/
    EnetQueue_initQ(&gCpswDmaMemMgrObj.txChObjMemQ);
    txObj = gCpswDmaMemMgrObj.cpdmaMemCfg->txChObjMemCfg.txChObjMemContainerBase;
    for (i = 0U; i < gCpswDmaMemMgrObj.cpdmaMemCfg->txChObjMemCfg.numTxCh; i++)
    {
        Enet_assert(((uintptr_t)txObj + sizeof(*txObj)) <= (txObjMemBase + txObjMemSize));
        EnetQueue_enq(&gCpswDmaMemMgrObj.txChObjMemQ, &txObj->node);
        txObj++;
    }

}

void EnetCpdma_memMgrDeInit(void)
{
    memset(&gCpswDmaMemMgrObj, 0U, sizeof(gCpswDmaMemMgrObj));
}

void EnetCpdmaStats_updateNotifyStats(EnetDma_CbStats *pktStats,
                                      uint32_t pktCnt,
                                      uint32_t timeDiff)
{
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
    uint32_t notifyCount;
    uint32_t timePerPkt = timeDiff / pktCnt;

    if (pktCnt == 0)
    {
        pktStats->zeroNotifyCnt++;
    }

    notifyCount = pktStats->dataNotifyCnt & (ENET_DMA_STATS_HISTORY_CNT - 1U);
    pktStats->dataNotifyCnt++;

    pktStats->totalPktCnt   += pktCnt;
    pktStats->totalCycleCnt += timeDiff;

    pktStats->cycleCntPerNotify[notifyCount] = timeDiff;
    pktStats->pktsPerNotify[notifyCount]     = pktCnt;
    pktStats->cycleCntPerPkt[notifyCount]    = timePerPkt;

    if (notifyCount != 0)
    {
        /* As app submits all packets for Rx during init, we ignore that to get run-time stats */
        if (timeDiff > pktStats->cycleCntPerNotifyMax)
        {
            pktStats->cycleCntPerNotifyMax = timeDiff;
        }

        if (pktCnt > pktStats->pktsPerNotifyMax)
        {
            pktStats->pktsPerNotifyMax = pktCnt;
        }

        if (timePerPkt > pktStats->cycleCntPerPktMax)
        {
            pktStats->cycleCntPerPktMax = timePerPkt;
        }
    }
#endif
}


/*!
 * @}
 */
