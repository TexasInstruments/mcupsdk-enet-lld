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
 *  \file enet_rm.c
 *
 *  \brief This file contains the implementation of the Enet Resource Manager.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */


#include <enet_cfg.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_soc.h>
#include <include/core/enet_rm.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/core/enet_rm_priv.h>
#include <priv/core/enet_rm_ioctl_priv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static int32_t EnetRm_allocMacAddr(EnetRm_Handle hRm,
                                   uint32_t coreKey,
                                   uint8_t macAddr[]);

static int32_t EnetRm_freeMacAddr(EnetRm_Handle hRm,
                                  uint32_t coreKey,
                                  uint8_t macAddr[]);

static int32_t EnetRm_allocRxFlowIdx(EnetRm_Handle hRm,
                                     uint32_t coreKey,
                                     uint32_t chIdx,
                                     uint32_t *startIdx,
                                     uint32_t *flowIdx);

static int32_t EnetRm_freeRxFlowIdx(EnetRm_Handle hRm,
                                    uint32_t coreKey,
                                    uint32_t chIdx,
                                    uint32_t flowIdx);

static int32_t EnetRm_allocTxChNum(EnetRm_Handle hRm,
                                   uint32_t coreKey,
                                   uint32_t *txPSILThreadId);

static int32_t EnetRm_freeTxChNum(EnetRm_Handle hRm,
                                  uint32_t coreKey,
                                  uint32_t txChNum);

static int32_t EnetRm_validateCoreIoctlPrivilege(EnetRm_Obj *hRm,
                                                 uint32_t cmd,
                                                 uint32_t coreId);

static int32_t EnetRm_attachCore(EnetRm_Handle hRm,
                                 uint32_t coreId,
                                 uint32_t *coreKey);

static int32_t EnetRm_allocInternalRxFlowIdx(EnetRm_RxFlowIdxObj *rxObj,
                                             uint32_t coreId,
                                             uint32_t *flowIdx,
                                             bool internalAllocation);

static int32_t EnetRm_freeInternalRxFlowIdx(EnetRm_RxFlowIdxObj *rxObj,
                                            uint32_t coreId,
                                            uint32_t flowIdx,
                                            bool internalAllocation);

static int32_t EnetRm_validateRxFlow(EnetRm_Handle hRm,
                                     uint32_t coreKey,
                                     uint32_t chIdx,
                                     uint32_t flowIdx);

static EnetQ_Node *EnetRm_allocResourceNode(EnetRm_CoreResTbl_t *resTbl,
                                            uint32_t coreId);

static EnetRm_ResEntry_t *EnetRm_allocResource(EnetRm_CoreResTbl_t *resTbl,
                                                    uint32_t coreId);

static int32_t EnetRm_mapMacAddr2ResourceIdx(EnetRm_MacAddressPool *macPool,
                                             uint8_t macAddr[],
                                             uint32_t *resIndex);

static void EnetRm_addCoreAttached(EnetRm_CoreAttachInfo *coreAttachInfo,
                                   uint32_t coreId);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_ALLOC_MAC_ADDR(EnetRm_Handle hRm, Enet_IoctlPrms *prms)
{
    uint32_t coreKey = *((uint32_t *)prms->inArgs);
    EnetRm_AllocMacAddrOutArgs *outArgs = (EnetRm_AllocMacAddrOutArgs *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = EnetRm_allocMacAddr(hRm, coreKey, &outArgs->macAddr[0U]);
    return status;
}

int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_FREE_MAC_ADDR(EnetRm_Handle hRm, Enet_IoctlPrms *prms)
{
    EnetRm_FreeMacAddrInArgs *inArgs = (EnetRm_FreeMacAddrInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = EnetRm_freeMacAddr(hRm, inArgs->coreKey, &inArgs->macAddr[0U]);
    return status;
}

int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_ALLOC_RX_FLOW(EnetRm_Handle hRm, Enet_IoctlPrms *prms)
{
    EnetRm_AllocRxFlowInArgs *inArgs = (EnetRm_AllocRxFlowInArgs *)prms->inArgs;
    EnetRm_AllocRxFlow *outArgs = (EnetRm_AllocRxFlow *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = EnetRm_allocRxFlowIdx(hRm,
                                   inArgs->coreKey,
                                   inArgs->chIdx,
                                   &outArgs->startIdx,
                                   &outArgs->flowIdx);
    return status;
}

int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_FREE_RX_FLOW(EnetRm_Handle hRm, Enet_IoctlPrms *prms)
{
    EnetRm_FreeRxFlowInArgs *inArgs = (EnetRm_FreeRxFlowInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = EnetRm_freeRxFlowIdx(hRm, inArgs->coreKey, inArgs->chIdx, inArgs->flowIdx);
    return status;
}

int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_ALLOC_TX_CH_PEERID(EnetRm_Handle hRm, Enet_IoctlPrms *prms)
{
    uint32_t coreKey = *((uint32_t *)prms->inArgs);
    uint32_t *txPSILThreadId = (uint32_t *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = EnetRm_allocTxChNum(hRm, coreKey, txPSILThreadId);
    return status;
}

int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_FREE_TX_CH_PEERID(EnetRm_Handle hRm, Enet_IoctlPrms *prms)
{
    EnetRm_FreeTxChInArgs *inArgs = (EnetRm_FreeTxChInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = EnetRm_freeTxChNum(hRm, inArgs->coreKey, inArgs->txChNum);
    return status;
}

int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_VALIDATE_PERMISSION(EnetRm_Handle hRm, Enet_IoctlPrms *prms)
{
    const EnetRm_ValidatePermissionInArgs *inArgs = (const EnetRm_ValidatePermissionInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = EnetRm_validateCoreIoctlPrivilege(hRm, inArgs->cmd, inArgs->coreId);
    return status;
}

int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_ATTACH(EnetRm_Handle hRm, Enet_IoctlPrms *prms)
{
    uint32_t *pCoreId  = (uint32_t *)prms->inArgs;
    uint32_t *pCoreKey = (uint32_t *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = EnetRm_attachCore(hRm, *pCoreId, pCoreKey);
    return status;
}

int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_DETACH(EnetRm_Handle hRm, Enet_IoctlPrms *prms)
{
    uint32_t *pCoreKey = (uint32_t *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = EnetRm_detachCore(hRm, *pCoreKey);
    return status;
}

int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_INTERNAL_ALLOC_RX_FLOW(EnetRm_Handle hRm, Enet_IoctlPrms *prms)
{
    EnetRm_AllocInternalRxFlowInArgs *inArgs = (EnetRm_AllocInternalRxFlowInArgs *)prms->inArgs;
    EnetRm_AllocRxFlow *outArgs = (EnetRm_AllocRxFlow *)prms->outArgs;
    EnetRm_RxFlowIdxObj *rxObj = &hRm->rxObj[inArgs->chIdx];
    int32_t status = ENET_SOK;

    Enet_assert((rxObj->internalAllocCoreId == ENET_RM_INVALIDCORE) ||
                (rxObj->internalAllocCoreId == inArgs->coreId));

    status = EnetRm_allocInternalRxFlowIdx(rxObj,
                                           inArgs->coreId,
                                           &outArgs->flowIdx,
                                           true);
    if (ENET_SOK == status)
    {
        outArgs->startIdx = hRm->cfg.rxStartFlowIdx[inArgs->chIdx];
        Enet_assert(outArgs->flowIdx < hRm->cfg.rxFlowIdxCnt[inArgs->chIdx]);
        rxObj->internalAllocCoreId = inArgs->coreId;
        rxObj->internalAllocCount++;
    }
    return status;
}

int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_INTERNAL_FREE_RX_FLOW(EnetRm_Handle hRm, Enet_IoctlPrms *prms)
{
    EnetRm_FreeInternalRxFlowInArgs *inArgs = (EnetRm_FreeInternalRxFlowInArgs *)prms->inArgs;
    EnetRm_RxFlowIdxObj *rxObj = &hRm->rxObj[inArgs->chIdx];
    int32_t status = ENET_SOK;

    status = EnetRm_freeInternalRxFlowIdx(rxObj,
                                          inArgs->coreId,
                                          inArgs->flowIdx,
                                          true);
    if (status == ENET_SOK)
    {
        Enet_assert(rxObj->internalAllocCount > 0);
        rxObj->internalAllocCount--;
        if (rxObj->internalAllocCount == 0)
        {
            rxObj->internalAllocCoreId = ENET_RM_INVALIDCORE;
        }
    }
    return status;
}

int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_VALIDATE_RX_FLOW(EnetRm_Handle hRm, Enet_IoctlPrms *prms)
{
    EnetRm_ValidateRxFlowInArgs *inArgs = (EnetRm_ValidateRxFlowInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = EnetRm_validateRxFlow(hRm, inArgs->coreKey, inArgs->chIdx, inArgs->flowIdx);
    return status;
}

static int32_t EnetRm_allocMacAddr(EnetRm_Handle hRm,
                                   uint32_t coreKey,
                                   uint8_t macAddr[])
{
    int32_t status;

    status = EnetRm_validateCoreKey(hRm, coreKey);

    if (status == ENET_SOK)
    {
        EnetRm_ResEntry_t *macRes;
        uint32_t coreId = ENET_COREKEY_2_COREID(coreKey);

        macRes = EnetRm_allocResource(&hRm->macObj.macTbl, coreId);
        if (macRes != NULL)
        {
            Enet_assert(macRes->ownerCoreId == coreId);
            Enet_assert(macRes->id < hRm->cfg.macList.numMacAddress);
            Enet_assert(macRes->id < hRm->macObj.resCnt);
            Enet_assert(macRes->id < ENET_ARRAYSIZE(hRm->cfg.macList.macAddress));
            memcpy(macAddr, &hRm->cfg.macList.macAddress[macRes->id][0], ENET_MAC_ADDR_LEN);
            status = ENET_SOK;
        }
        else
        {
            status = ENET_EALLOC;
        }
    }

    return status;
}

static int32_t EnetRm_freeMacAddr(EnetRm_Handle hRm,
                                  uint32_t coreKey,
                                  uint8_t macAddr[])
{
    int32_t status = ENET_SOK;

    status = EnetRm_validateCoreKey(hRm, coreKey);
    if (status == ENET_SOK)
    {
        uint32_t coreId = ENET_COREKEY_2_COREID(coreKey);
        uint32_t macIdx;

        status = EnetRm_mapMacAddr2ResourceIdx(&hRm->cfg.macList, macAddr, &macIdx);
        if (status == ENET_SOK)
        {
            Enet_assert(hRm->macObj.resCnt <= ENET_ARRAYSIZE(hRm->macObj.macRes));
            status = EnetRm_freeResource(&hRm->macObj.macTbl,
                                         hRm->macObj.macRes,
                                         hRm->macObj.resCnt,
                                         coreId,
                                         macIdx);
        }
    }

    return status;
}

static int32_t EnetRm_allocRxFlowIdx(EnetRm_Handle hRm,
                                     uint32_t coreKey,
                                     uint32_t chIdx,
                                     uint32_t *startIdx,
                                     uint32_t *flowIdx)
{
    int32_t status;

    status = EnetRm_validateCoreKey(hRm, coreKey);

    if (status == ENET_SOK)
    {
        uint32_t coreId = ENET_COREKEY_2_COREID(coreKey);

        status = EnetRm_allocInternalRxFlowIdx(&hRm->rxObj[chIdx], coreId, flowIdx, false);
        if (ENET_SOK == status)
        {
            *startIdx = hRm->cfg.rxStartFlowIdx[chIdx];
            Enet_assert(*flowIdx < hRm->cfg.rxFlowIdxCnt[chIdx]);
        }
    }

    return status;
}

static int32_t EnetRm_freeRxFlowIdx(EnetRm_Handle hRm,
                                    uint32_t coreKey,
                                    uint32_t chIdx,
                                    uint32_t flowIdx)
{
    int32_t status;

    status = EnetRm_validateCoreKey(hRm, coreKey);
    if (status == ENET_SOK)
    {
        uint32_t coreId = ENET_COREKEY_2_COREID(coreKey);

        EnetRm_freeInternalRxFlowIdx(&hRm->rxObj[chIdx], coreId, flowIdx, false);
    }

    return status;
}

static int32_t EnetRm_allocTxChNum(EnetRm_Handle hRm,
                                   uint32_t coreKey,
                                   uint32_t *txPSILThreadId)
{
    uint32_t socTxChCnt;
    int32_t status;

    *txPSILThreadId = ENET_RM_TXCHNUM_INVALID;

    status = EnetRm_validateCoreKey(hRm, coreKey);
    if (status == ENET_SOK)
    {
        EnetRm_ResEntry_t *txRes;
        uint32_t coreId = ENET_COREKEY_2_COREID(coreKey);

        txRes = EnetRm_allocResource(&hRm->txObj.txResTbl, coreId);
        if (txRes != NULL)
        {
            socTxChCnt = EnetSoc_getTxChCount(hRm->cfg.enetType, hRm->cfg.instId);
            Enet_assert(txRes->ownerCoreId == coreId);
            Enet_assert(txRes->id < hRm->txObj.resCnt);
            Enet_assert(txRes->id < socTxChCnt);
            *txPSILThreadId = txRes->id + hRm->txObj.txPSILThreadIdOffset;
            status = ENET_SOK;
        }
        else
        {
            status = ENET_EALLOC;
        }
    }

    return status;
}

static int32_t EnetRm_freeTxChNum(EnetRm_Handle hRm,
                                  uint32_t coreKey,
                                  uint32_t txChNum)
{
    EnetRm_TxChObj *txChObj = &hRm->txObj;
    uint32_t txChOffset = txChNum - txChObj->txPSILThreadIdOffset;
    int32_t status = ENET_SOK;

    status = EnetRm_validateCoreKey(hRm, coreKey);

    if (status == ENET_SOK)
    {
        uint32_t coreId = ENET_COREKEY_2_COREID(coreKey);

        Enet_assert(hRm->txObj.resCnt <= ENET_ARRAYSIZE(hRm->txObj.txRes));

        status = EnetRm_freeResource(&hRm->txObj.txResTbl,
                                     hRm->txObj.txRes,
                                     hRm->txObj.resCnt,
                                     coreId,
                                     txChOffset);
    }

    return status;
}

static int32_t EnetRm_validateCoreIoctlPrivilege(EnetRm_Obj *hRm,
                                                 uint32_t cmd,
                                                 uint32_t coreId)
{
    bool ioctlPermitted;
    uint32_t i;
    int32_t status;

    for (i = 0U; i < hRm->cfg.ioctlPermissionInfo.numEntries; i++)
    {
        if (cmd == hRm->cfg.ioctlPermissionInfo.entry[i].cmd)
        {
            break;
        }
    }

    if (i < hRm->cfg.ioctlPermissionInfo.numEntries)
    {
        ioctlPermitted = ENET_IS_BIT_SET(hRm->cfg.ioctlPermissionInfo.entry[i].permittedCoreMask, coreId);
    }
    else
    {
        ioctlPermitted = ENET_IS_BIT_SET(hRm->cfg.ioctlPermissionInfo.defaultPermittedCoreMask, coreId);
    }

    status = ioctlPermitted ? ENET_SOK : ENET_EPERM;

    return status;
}

static int32_t EnetRm_attachCore(EnetRm_Handle hRm,
                                 uint32_t coreId,
                                 uint32_t *coreKey)
{
    bool coreAttached;
    int32_t status = ENET_SOK;

    coreAttached = EnetRm_isCoreAttached(&hRm->coreAttachObj, coreId, (uint32_t *)NULL_PTR);
    if (coreAttached == false)
    {
        EnetRm_addCoreAttached(&hRm->coreAttachObj, coreId);
        *coreKey = ENET_COREID_2_COREKEY(coreId);
    }
    else
    {
        /* If already attached just return coreKey */
        *coreKey = ENET_COREID_2_COREKEY(coreId);
    }

    return status;
}

static int32_t EnetRm_allocInternalRxFlowIdx(EnetRm_RxFlowIdxObj *rxObj,
                                             uint32_t coreId,
                                             uint32_t *flowIdx,
                                             bool internalAllocation)
{
    EnetRm_ResEntry_t *rxRes;
    int32_t status;

    rxRes = EnetRm_allocResource(&rxObj->rxResTbl, coreId);
    if (rxRes != NULL)
    {
        Enet_assert(rxRes->ownerCoreId == coreId);
        if (internalAllocation)
        {
            rxRes->ownerCoreId = ENET_RM_GET_INTERNAL_ALLOC_COREID(rxRes->ownerCoreId);
        }

        Enet_assert(rxRes->id < rxObj->resCnt);
        *flowIdx = rxRes->id;
        status = ENET_SOK;
    }
    else
    {
        status = ENET_EALLOC;
    }

    return status;
}

static int32_t EnetRm_freeInternalRxFlowIdx(EnetRm_RxFlowIdxObj *rxObj,
                                            uint32_t coreId,
                                            uint32_t flowIdx,
                                            bool internalAllocation)
{
    uint32_t matchCoreId;
    int32_t status = ENET_SOK;

    if (internalAllocation)
    {
        matchCoreId = ENET_RM_GET_INTERNAL_ALLOC_COREID(coreId);
    }
    else
    {
        matchCoreId = coreId;
    }

    Enet_assert(rxObj->resCnt <= ENET_ARRAYSIZE(rxObj->rxRes));
    status = EnetRm_freeResource(&rxObj->rxResTbl,
                                 rxObj->rxRes,
                                 rxObj->resCnt,
                                 matchCoreId,
                                 flowIdx);
    return status;
}

static int32_t EnetRm_validateRxFlow(EnetRm_Handle hRm,
                                     uint32_t coreKey,
                                     uint32_t chIdx,
                                     uint32_t flowIdx)
{
    EnetRm_RxFlowIdxObj *rxObj = &hRm->rxObj[chIdx];
    int32_t status = ENET_SOK;

    status = EnetRm_validateCoreKey(hRm, coreKey);
    if (ENET_SOK == status)
    {
        uint32_t coreId = ENET_COREKEY_2_COREID(coreKey);
        EnetRm_ResEntry_t *res;

        res = EnetRm_findResource(rxObj->rxRes,
                                  ENET_ARRAYSIZE(rxObj->rxRes),
                                  coreId,
                                  flowIdx);

        status = (res != NULL) ? ENET_SOK : ENET_EINVALIDPARAMS;
    }

    return status;
}

static EnetQ_Node *EnetRm_allocResourceNode(EnetRm_CoreResTbl_t *resTbl,
                                            uint32_t coreId)
{
    EnetQ_Node *resource;
    uint32_t i;

    Enet_assert(resTbl->numCores <= ENET_ARRAYSIZE(resTbl->coreResTbl));

    for (i = 0U; i < resTbl->numCores; i++)
    {
        if (coreId == resTbl->coreResTbl[i].coreId)
        {
            break;
        }
    }

    if (i < resTbl->numCores)
    {
        resource = EnetQueue_deq(&resTbl->coreResTbl[i].resQ);
    }
    else
    {
        resource = NULL;
    }

    return resource;
}

static EnetRm_ResEntry_t *EnetRm_allocResource(EnetRm_CoreResTbl_t *resTbl,
                                                    uint32_t coreId)
{
    EnetQ_Node *resNode;
    EnetRm_ResEntry_t *resource;

    resNode = EnetRm_allocResourceNode(resTbl, coreId);
    if (resNode != NULL)
    {
        resource = container_of(resNode, EnetRm_ResEntry_t, node);
        memset(&resource->node, 0, sizeof(resource->node));
        Enet_assert(resource->ownerCoreId == ENET_RM_INVALIDCORE);
        resource->ownerCoreId = coreId;
    }
    else
    {
        resource = NULL;
    }

    return resource;
}

static int32_t EnetRm_mapMacAddr2ResourceIdx(EnetRm_MacAddressPool *macPool,
                                             uint8_t macAddr[],
                                             uint32_t *resIndex)
{
    uint32_t i;
    int32_t status = ENET_EFAIL;

    for (i = 0U; i < macPool->numMacAddress; i++)
    {
        if (memcmp(&macPool->macAddress[i][0], macAddr, ENET_MAC_ADDR_LEN) == 0)
        {
            break;
        }
    }

    if (i < macPool->numMacAddress)
    {
        *resIndex = i;
        status = ENET_SOK;
    }

    return status;
}

static void EnetRm_addCoreAttached(EnetRm_CoreAttachInfo *coreAttachInfo,
                                   uint32_t coreId)
{
    bool coreAttached;

    coreAttached = EnetRm_isCoreAttached(coreAttachInfo, coreId, (uint32_t *)NULL_PTR);

    Enet_assert(coreAttached == false);
    Enet_assert(coreAttachInfo->numCores < ENET_ARRAYSIZE(coreAttachInfo->attachedCores));

    coreAttachInfo->attachedCores[coreAttachInfo->numCores] = coreId;
    coreAttachInfo->numCores++;
}
