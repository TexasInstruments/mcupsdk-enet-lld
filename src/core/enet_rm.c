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

typedef enum  EnetRm_ResourceType_tag
{
    ENET_RM_RESOURCE_TXCH,
    ENET_RM_RESOURCE_RXFLOW,
    ENET_RM_RESOURCE_MACADDRESS,
    ENET_RM_RESOURCE_LAST = ENET_RM_RESOURCE_MACADDRESS,
} EnetRm_ResourceType_e;

#define ENET_RM_RESOURCE_COUNT                      (ENET_RM_RESOURCE_LAST + 1)
#define ENET_RM_RESOURCE_ID_ANY                     (~0U)

#define ENET_RM_IOCTL_HANDLER_ENTRY_INIT(x)             \
          {.cmd = x,                                    \
           .fxn = &EnetRm_ioctl_handler_##x}

#define ENET_RM_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(x)     \
          {.cmd = x,                                    \
           .fxn = &EnetRm_ioctl_handler_default}

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef int32_t (EnetRmIoctlHandler)(EnetRm_Handle hRm, Enet_IoctlPrms *prms);

typedef struct EnetRmIoctlHandlerRegistry_s
{
    uint32_t cmd;
    EnetRmIoctlHandler *fxn;
} EnetRmIoctlHandlerRegistry_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static void EnetRm_initResourceNode(EnetRm_ResEntry_t *res,
                                    uint32_t resId);

static void EnetRm_initResourceQ(EnetQ *resQ,
                                 EnetRm_ResEntry_t *resNodes,
                                 uint32_t resNodeLen,
                                 uint32_t startResIdx,
                                 uint32_t numResource);

static void EnetRm_initCoreResourceQ(EnetRm_CoreResInfo_t *coreResTbl,
                                     uint32_t coreId,
                                     EnetRm_ResEntry_t *resNodes,
                                     uint32_t resNodeLen,
                                     uint32_t startResIdx,
                                     uint32_t numResource);

static EnetQ *EnetRm_getFreeQ(EnetRm_CoreResTbl_t *resTbl,
                              uint32_t coreId);

static void EnetRm_freeCoreResource(EnetRm_CoreResTbl_t *resTbl,
                                    EnetRm_ResEntry_t *resList,
                                    uint32_t numTableEntries,
                                    uint32_t coreId);

static void EnetRm_initTxObj(EnetRm_TxChObj *txObj,
                             const EnetRm_Cfg *rmCfg);

static void EnetRm_initRxObj(EnetRm_RxFlowIdxObj *rxObj,
                             const EnetRm_Cfg *rmCfg,
                             uint32_t chIdx);

static void EnetRm_initMacObj(EnetRm_MacAddressObj *macObj,
                              const EnetRm_Cfg *rmCfg);

static void EnetRm_initAttachObj(EnetRm_CoreAttachInfo *coreAttachObj);

static int32_t EnetRm_validateResPartInfo(const EnetRm_Cfg *rmCfg,
                                                    uint32_t maxTxResCnt,
                                                    uint32_t maxRxResCnt,
                                                    uint32_t maxMacResCnt);

int32_t EnetRm_validateCoreKey(EnetRm_Handle hRm,
                               uint32_t coreKey);

static void EnetRm_delCoreAttached(EnetRm_CoreAttachInfo *coreAttachInfo,
                                   uint32_t coreId);

static EnetRm_ResourceInfo *EnetRm_getCoreResourceInfo(EnetRm_Cfg *cfg,
                                                       uint32_t coreId);

static int32_t EnetRm_ioctl_handler_default(EnetRm_Handle hRm, Enet_IoctlPrms *prms);
static int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_REGISTER_HANDLER(EnetRm_Handle hRm, Enet_IoctlPrms *prms);
static int32_t EnetRm_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                         EnetRmIoctlHandler *ioctlHandlerFxn,
                                         EnetRmIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                         uint32_t tableSize);
static EnetRmIoctlHandler * EnetRm_getIoctlHandlerFxn(uint32_t ioctlCmd, EnetRmIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize);
static int32_t EnetRm_getIoctlHandlerIdx(uint32_t ioctlCmd, EnetRmIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#if ENET_CFG_IS_ON(DEV_ERROR)
/* Public RM IOCTL validation data. */
static Enet_IoctlValidate gEnetRm_ioctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(ENET_RM_IOCTL_ALLOC_MAC_ADDR,
                          sizeof(uint32_t),
                          sizeof(EnetRm_AllocMacAddrOutArgs)),

    ENET_IOCTL_VALID_PRMS(ENET_RM_IOCTL_FREE_MAC_ADDR,
                          sizeof(EnetRm_FreeMacAddrInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_RM_IOCTL_ALLOC_RX_FLOW,
                          sizeof(EnetRm_AllocRxFlowInArgs),
                          sizeof(EnetRm_AllocRxFlow)),

    ENET_IOCTL_VALID_PRMS(ENET_RM_IOCTL_FREE_RX_FLOW,
                          sizeof(EnetRm_FreeRxFlowInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_RM_IOCTL_ALLOC_TX_CH_PEERID,
                          sizeof(uint32_t),
                          sizeof(uint32_t *)),

    ENET_IOCTL_VALID_PRMS(ENET_RM_IOCTL_FREE_TX_CH_PEERID,
                          sizeof(EnetRm_FreeTxChInArgs),
                          0U),
};

/* Private RM IOCTL validation data. */
static Enet_IoctlValidate gEnetRm_privIoctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(ENET_RM_IOCTL_VALIDATE_PERMISSION,
                          sizeof(EnetRm_ValidatePermissionInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_RM_IOCTL_ATTACH,
                          sizeof(uint32_t),
                          sizeof(uint32_t)),

    ENET_IOCTL_VALID_PRMS(ENET_RM_IOCTL_DETACH,
                          sizeof(uint32_t),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_RM_IOCTL_INTERNAL_ALLOC_RX_FLOW,
                          sizeof(EnetRm_AllocInternalRxFlowInArgs),
                          sizeof(EnetRm_AllocRxFlow)),

    ENET_IOCTL_VALID_PRMS(ENET_RM_IOCTL_INTERNAL_FREE_RX_FLOW,
                          sizeof(EnetRm_FreeInternalRxFlowInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(ENET_RM_IOCTL_VALIDATE_RX_FLOW,
                          sizeof(EnetRm_ValidateRxFlowInArgs),
                          0U),
    ENET_IOCTL_VALID_PRMS(ENET_RM_IOCTL_REGISTER_HANDLER,
                          sizeof(Enet_IoctlRegisterHandlerInArgs),
                          0U),

};
#endif

static EnetRmIoctlHandlerRegistry_t EnetRmIoctlHandlerRegistry[] =
{
    ENET_RM_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_RM_IOCTL_ALLOC_MAC_ADDR),
    ENET_RM_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_RM_IOCTL_FREE_MAC_ADDR),
    ENET_RM_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_RM_IOCTL_ALLOC_RX_FLOW),
    ENET_RM_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_RM_IOCTL_FREE_RX_FLOW),
    ENET_RM_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_RM_IOCTL_ALLOC_TX_CH_PEERID),
    ENET_RM_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_RM_IOCTL_FREE_TX_CH_PEERID),
    ENET_RM_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_RM_IOCTL_VALIDATE_PERMISSION), /*Private IOCTL*/
    ENET_RM_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_RM_IOCTL_ATTACH), /*Private IOCTL*/
    ENET_RM_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_RM_IOCTL_DETACH), /*Private IOCTL*/
    ENET_RM_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_RM_IOCTL_INTERNAL_ALLOC_RX_FLOW), /*Private IOCTL*/
    ENET_RM_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_RM_IOCTL_INTERNAL_FREE_RX_FLOW), /*Private IOCTL*/
    ENET_RM_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_RM_IOCTL_VALIDATE_RX_FLOW), /*Private IOCTL*/
    ENET_RM_IOCTL_HANDLER_ENTRY_INIT(ENET_RM_IOCTL_REGISTER_HANDLER), /*Private IOCTL*/
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t EnetRm_open(EnetMod_Handle hMod,
                    Enet_Type enetType,
                    uint32_t instId,
                    const void *cfg,
                    uint32_t cfgSize)
{
    EnetRm_Handle hRm = (EnetRm_Handle)hMod;
    const EnetRm_Cfg *rmCfg = (const EnetRm_Cfg *)cfg;
    uint32_t i;
    int32_t status = ENET_SOK;

    Enet_devAssert(cfgSize == sizeof(EnetRm_Cfg),
                   "Invalid RM config params size %u (expected %u)\n",
                   cfgSize, sizeof(EnetRm_Cfg));

    if (rmCfg == NULL)
    {
        ENETTRACE_ERR("Invalid NULL RM config\n");
        status = ENET_EBADARGS;
    }

    if (status == ENET_SOK)
    {
        status = EnetRm_validateResPartInfo(rmCfg,
                                            ENET_ARRAYSIZE(hRm->txObj.txRes),
                                            ENET_ARRAYSIZE(hRm->rxObj[0U].rxRes),
                                            ENET_ARRAYSIZE(hRm->macObj.macRes));
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "Resource partition validation failed: %d\n", status);
    }

    if (ENET_SOK == status)
    {
        hRm->cfg = *rmCfg;
        EnetRm_initTxObj(&hRm->txObj, rmCfg);
        EnetRm_initMacObj(&hRm->macObj, rmCfg);
        EnetRm_initAttachObj(&hRm->coreAttachObj);

        hRm->numRxCh = rmCfg->numRxCh;
        for (i = 0U; i < hRm->numRxCh; i++)
        {
            EnetRm_initRxObj(&hRm->rxObj[i], rmCfg, i);
        }
    }

    return status;
}

int32_t EnetRm_rejoin(EnetMod_Handle hMod,
                      Enet_Type enetType,
                      uint32_t instId)
{
    return ENET_ENOTSUPPORTED;
}

void EnetRm_close(EnetMod_Handle hMod)
{
    EnetRm_Handle hRm = (EnetRm_Handle)hMod;
    EnetRm_ResPrms *resInfo = &hRm->cfg.resPartInfo;
    EnetQ *pQ;
    uint32_t txFreeResCnt = 0U;
    uint32_t rxFreeResCnt[ENET_RM_NUM_RXCHAN_MAX];
    uint32_t macFreeResCnt = 0U;
    uint32_t i;
    uint32_t j;

    for (j = 0U; j < hRm->numRxCh; j++)
    {
        rxFreeResCnt[j] = 0U;
    }

    /* Assert all internally allocated flows are freed */
    for (j = 0U; j < hRm->numRxCh; j++)
    {
        Enet_assert((hRm->rxObj[j].internalAllocCoreId == ENET_RM_INVALIDCORE) &&
                    (hRm->rxObj[j].internalAllocCount == 0U));
    }
    Enet_assert(resInfo->numCores <= ENET_ARRAYSIZE(resInfo->coreDmaResInfo));

    for (i = 0U; i < resInfo->numCores; i++)
    {
        EnetRm_detachCore(hRm, ENET_COREID_2_COREKEY(resInfo->coreDmaResInfo[i].coreId));

        pQ = EnetRm_getFreeQ(&hRm->txObj.txResTbl, resInfo->coreDmaResInfo[i].coreId);
        if (NULL != pQ)
        {
            txFreeResCnt += EnetQueue_getQCount(pQ);
        }

        for (j = 0U; j < hRm->numRxCh; j++)
        {
            pQ = EnetRm_getFreeQ(&hRm->rxObj[j].rxResTbl, resInfo->coreDmaResInfo[i].coreId);
            if (NULL != pQ)
            {
                rxFreeResCnt[j] += EnetQueue_getQCount(pQ);
            }
        }

        pQ = EnetRm_getFreeQ(&hRm->macObj.macTbl, resInfo->coreDmaResInfo[i].coreId);
        if (NULL != pQ)
        {
            macFreeResCnt += EnetQueue_getQCount(pQ);
        }
    }

    Enet_assert(txFreeResCnt == hRm->txObj.resCnt);
    Enet_assert(macFreeResCnt == hRm->macObj.resCnt);

    for (j = 0U; j < hRm->numRxCh; j++)
    {
        Enet_assert(rxFreeResCnt[j] == hRm->rxObj[j].resCnt);
    }
}

int32_t EnetRm_ioctl(EnetMod_Handle hMod,
                     uint32_t cmd,
                     Enet_IoctlPrms *prms)
{
    EnetRm_Handle hRm = (EnetRm_Handle)hMod;
    int32_t status = ENET_SOK;

#if ENET_CFG_IS_ON(DEV_ERROR)
    /* Validate Enet RM IOCTL parameters */
    if (ENET_IOCTL_GET_PER(cmd) == ENET_IOCTL_PER_GENERIC)
    {
        if (ENET_IOCTL_GET_TYPE(cmd) == ENET_IOCTL_TYPE_PUBLIC)
        {
            status = Enet_validateIoctl(cmd, prms,
                                        gEnetRm_ioctlValidate,
                                        ENET_ARRAYSIZE(gEnetRm_ioctlValidate));
        }
        else
        {
            status = Enet_validateIoctl(cmd, prms,
                                        gEnetRm_privIoctlValidate,
                                        ENET_ARRAYSIZE(gEnetRm_privIoctlValidate));
        }

        ENETTRACE_ERR_IF(status != ENET_SOK, "IOCTL 0x%08x params are not valid\n", cmd);
    }
#endif

    if (status == ENET_SOK)
    {
        EnetRmIoctlHandler * ioctlHandlerFxn;

        ioctlHandlerFxn = EnetRm_getIoctlHandlerFxn(cmd, EnetRmIoctlHandlerRegistry, ENET_ARRAYSIZE(EnetRmIoctlHandlerRegistry));
        Enet_devAssert(ioctlHandlerFxn != NULL);
        status = ioctlHandlerFxn(hRm, prms);
    }

    return status;
}


EnetRm_ResEntry_t *EnetRm_findResource(EnetRm_ResEntry_t *resTable,
                                       uint32_t numTableEntries,
                                       uint32_t coreId,
                                       uint32_t resId)
{
    EnetRm_ResEntry_t *resource;
    uint32_t i;

    for (i = 0U; i < numTableEntries; i++)
    {
        if (resTable[i].ownerCoreId == coreId)
        {
            if ((resTable[i].id == resId) ||
                (resId == ENET_RM_RESOURCE_ID_ANY))
            {
                break;
            }
        }
    }

    if (i < numTableEntries)
    {
        resource = &resTable[i];
    }
    else
    {
        resource = NULL;
    }

    return resource;
}

static void EnetRm_initResourceNode(EnetRm_ResEntry_t *res,
                                    uint32_t resId)
{
    memset(res, 0, sizeof(*res));
    res->ownerCoreId = ENET_RM_INVALIDCORE;
    res->id          = resId;
}

static void EnetRm_initResourceQ(EnetQ *resQ,
                                 EnetRm_ResEntry_t *resNodes,
                                 uint32_t resNodeLen,
                                 uint32_t startResIdx,
                                 uint32_t numResource)
{
    uint32_t i;

    EnetQueue_initQ(resQ);

    for (i = 0U; i < numResource; i++)
    {
        Enet_assert(i < resNodeLen);
        EnetRm_initResourceNode(&resNodes[i], startResIdx + i);
        EnetQueue_enq(resQ, &resNodes[i].node);
    }
}

static void EnetRm_initCoreResourceQ(EnetRm_CoreResInfo_t *coreResTbl,
                                     uint32_t coreId,
                                     EnetRm_ResEntry_t *resNodes,
                                     uint32_t resNodeLen,
                                     uint32_t startResIdx,
                                     uint32_t numResource)
{
    coreResTbl->coreId = coreId;
    EnetRm_initResourceQ(&coreResTbl->resQ, resNodes, resNodeLen, startResIdx, numResource);
}

static EnetQ *EnetRm_getFreeQ(EnetRm_CoreResTbl_t *resTbl,
                              uint32_t coreId)
{
    uint32_t i;
    EnetQ *queue;

    Enet_assert(resTbl->numCores <= ENET_ARRAYSIZE(resTbl->coreResTbl));
    for (i = 0U; i < resTbl->numCores; i++)
    {
        if ((coreId == resTbl->coreResTbl[i].coreId) ||
            (coreId == ENET_RM_GET_INTERNAL_ALLOC_COREID(resTbl->coreResTbl[i].coreId)))
        {
            break;
        }
    }

    if (i < resTbl->numCores)
    {
        queue = &resTbl->coreResTbl[i].resQ;
    }
    else
    {
        queue = NULL;
    }

    return queue;
}

int32_t EnetRm_freeResource(EnetRm_CoreResTbl_t *resTbl,
                            EnetRm_ResEntry_t *resList,
                            uint32_t resCount,
                            uint32_t coreId,
                            uint32_t resId)
{
    EnetRm_ResEntry_t *resource;
    EnetQ *freeQ;
    int32_t status;

    freeQ = EnetRm_getFreeQ(resTbl, coreId);
    if (freeQ == NULL)
    {
        status = ENET_EFAIL;
    }
    else
    {
        status = ENET_SOK;
    }

    if (ENET_SOK == status)
    {
        resource = EnetRm_findResource(resList, resCount, coreId, resId);
        if (resource != NULL)
        {
            Enet_assert(resource->ownerCoreId == coreId);

            /* Do not use argument resId to init resource as it may be RSC_ANY
             * which is an invalid id
             */
            EnetRm_initResourceNode(resource, resource->id);
            EnetQueue_enq(freeQ, &resource->node);
            status = ENET_SOK;
        }
        else
        {
            status = ENET_EFAIL;
        }
    }

    return status;
}

static void EnetRm_freeCoreResource(EnetRm_CoreResTbl_t *resTbl,
                                    EnetRm_ResEntry_t *resList,
                                    uint32_t numTableEntries,
                                    uint32_t coreId)
{
    int32_t status;

    do
    {
        status = EnetRm_freeResource(resTbl, resList, numTableEntries, coreId, ENET_RM_RESOURCE_ID_ANY);
    }
    while (status == ENET_SOK);
}

static void EnetRm_initTxObj(EnetRm_TxChObj *txObj,
                             const EnetRm_Cfg *rmCfg)
{
    const uint32_t resTableSize = ENET_ARRAYSIZE(txObj->txRes);
    uint32_t allocResCnt;
    uint32_t txChPeerId;
    uint32_t coreCnt;
    uint32_t i;

    txChPeerId = EnetSoc_getTxChPeerId(rmCfg->enetType, rmCfg->instId, 0U);

    txObj->txPSILThreadIdOffset     = txChPeerId;
    txObj->txResTbl.numCores = rmCfg->resPartInfo.numCores;

    allocResCnt = 0U;
    for (i = 0U; i < rmCfg->resPartInfo.numCores; i++)
    {
        coreCnt = rmCfg->resPartInfo.coreDmaResInfo[i].numTxCh;

#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_UDMA)
        Enet_assert((allocResCnt + coreCnt) <= txChPeerId);
#elif defined (ENET_SOC_HOSTPORT_DMA_TYPE_CPDMA)
       /* No assert check needed */
#else
#error "Unsupported DMA type"
#endif
        Enet_assert((allocResCnt + coreCnt) <= resTableSize);

        EnetRm_initCoreResourceQ(&txObj->txResTbl.coreResTbl[i],
                                 rmCfg->resPartInfo.coreDmaResInfo[i].coreId,
                                 &txObj->txRes[allocResCnt],
                                 (resTableSize - allocResCnt),
                                 allocResCnt,
                                 coreCnt);
        allocResCnt += coreCnt;
    }

    txObj->resCnt = allocResCnt;
}

static void EnetRm_initRxObj(EnetRm_RxFlowIdxObj *rxObj,
                             const EnetRm_Cfg *rmCfg,
                             uint32_t chIdx)
{
    const uint32_t resTableSize = ENET_ARRAYSIZE(rxObj->rxRes);
    uint32_t allocResCnt;
    uint32_t socRxFlowCnt;
    uint32_t coreCnt;
    uint32_t i;

    socRxFlowCnt = EnetSoc_getRxFlowCount(rmCfg->enetType, rmCfg->instId);

    rxObj->rxResTbl.numCores = rmCfg->resPartInfo.numCores;

    allocResCnt = 0U;
    for (i = 0U; i < rmCfg->resPartInfo.numCores; i++)
    {
        coreCnt = rmCfg->resPartInfo.coreDmaResInfo[i].numRxFlows;

        Enet_assert((allocResCnt + coreCnt) <= socRxFlowCnt);
        Enet_assert((allocResCnt + coreCnt) <= resTableSize);

        EnetRm_initCoreResourceQ(&rxObj->rxResTbl.coreResTbl[i],
                                 rmCfg->resPartInfo.coreDmaResInfo[i].coreId,
                                 &rxObj->rxRes[allocResCnt],
                                 resTableSize - allocResCnt,
                                 allocResCnt,
                                 coreCnt);
        allocResCnt += coreCnt;
    }

    rxObj->resCnt       = allocResCnt;
    rxObj->internalAllocCount  = 0U;
    rxObj->internalAllocCoreId = ENET_RM_INVALIDCORE;
}

static void EnetRm_initMacObj(EnetRm_MacAddressObj *macObj,
                              const EnetRm_Cfg *rmCfg)
{
    const uint32_t resTableSize = ENET_ARRAYSIZE(macObj->macRes);
    uint32_t allocResCnt;
    uint32_t coreCnt;
    uint32_t i;

    macObj->macTbl.numCores = rmCfg->resPartInfo.numCores;

    allocResCnt = 0U;
    for (i = 0U; i < rmCfg->resPartInfo.numCores; i++)
    {
        coreCnt = rmCfg->resPartInfo.coreDmaResInfo[i].numMacAddress;

        Enet_assert((allocResCnt + coreCnt) <= resTableSize);
        Enet_assert((allocResCnt + coreCnt) <= rmCfg->macList.numMacAddress);

        EnetRm_initCoreResourceQ(&macObj->macTbl.coreResTbl[i],
                                 rmCfg->resPartInfo.coreDmaResInfo[i].coreId,
                                 &macObj->macRes[allocResCnt],
                                 (resTableSize - allocResCnt),
                                 allocResCnt,
                                 coreCnt);
        allocResCnt += coreCnt;
    }

    macObj->resCnt = allocResCnt;
}

static void EnetRm_initAttachObj(EnetRm_CoreAttachInfo *coreAttachObj)
{
    coreAttachObj->numCores = 0U;
}


static int32_t EnetRm_validateResPartInfo(const EnetRm_Cfg *rmCfg,
                                          uint32_t maxTxResCnt,
                                          uint32_t maxRxResCnt,
                                          uint32_t maxMacResCnt)
{
    const EnetRm_ResPrms *resPartInfo = &rmCfg->resPartInfo;
    uint32_t txChCnt = 0U;
    uint32_t rxFlowCnt = 0U;
    uint32_t rmRxFlowCnt = 0U;
    uint32_t macAddrCnt = 0U;
    uint32_t socTxChCnt;
    uint32_t i;
    int32_t status = ENET_SOK;

    socTxChCnt = EnetSoc_getTxChCount(rmCfg->enetType, rmCfg->instId);

    if (resPartInfo->numCores >
        ENET_ARRAYSIZE(resPartInfo->coreDmaResInfo))
    {
        status = ENET_EINVALIDPARAMS;
    }

    if (ENET_SOK == status)
    {
        for (i = 0U; i < resPartInfo->numCores; i++)
        {
            txChCnt    += resPartInfo->coreDmaResInfo[i].numTxCh;
            rxFlowCnt  += resPartInfo->coreDmaResInfo[i].numRxFlows;
            macAddrCnt += resPartInfo->coreDmaResInfo[i].numMacAddress;
        }

        for (i = 0U; i < ENET_RM_NUM_RXCHAN_MAX; i++)
        {
            rmRxFlowCnt += rmCfg->rxFlowIdxCnt[i];
        }

        if ((txChCnt > socTxChCnt) ||
            (txChCnt > maxTxResCnt))
        {
            status = ENET_EINVALIDPARAMS;
        }

        if ((rxFlowCnt > rmRxFlowCnt) ||
            (rxFlowCnt > maxRxResCnt))
        {
            status = ENET_EINVALIDPARAMS;
        }

        if ((macAddrCnt > rmCfg->macList.numMacAddress) ||
            (macAddrCnt > maxMacResCnt) ||
            (macAddrCnt > ENET_ARRAYSIZE(rmCfg->macList.macAddress)) ||
            (rmCfg->macList.numMacAddress > ENET_ARRAYSIZE(rmCfg->macList.macAddress)))
        {
            status = ENET_EINVALIDPARAMS;
        }
    }

    return status;
}

bool EnetRm_isCoreAttached(EnetRm_CoreAttachInfo *coreAttachInfo,
                           uint32_t coreId,
                           uint32_t *attachIndex)
{
    bool coreAttached;
    uint32_t i;

    /* Initialize to avoid compiler warning.*/
    if (NULL != attachIndex)
    {
        *attachIndex = 0U;
    }

    for (i = 0U; i < coreAttachInfo->numCores; i++)
    {
        if (coreAttachInfo->attachedCores[i] == coreId)
        {
            break;
        }
    }

    if (i < coreAttachInfo->numCores)
    {
        coreAttached = true;
        if (attachIndex != NULL)
        {
            *attachIndex = i;
        }
    }
    else
    {
        coreAttached = false;
    }

    return coreAttached;
}

int32_t EnetRm_validateCoreKey(EnetRm_Handle hRm,
                               uint32_t coreKey)
{
    uint32_t coreId;
    int32_t status = ENET_SOK;

    coreId = ENET_COREKEY_2_COREID(coreKey);

    if (EnetRm_isCoreAttached(&hRm->coreAttachObj, coreId, (uint32_t *)NULL_PTR) == false)
    {
        status = ENET_EINVALIDPARAMS;
    }

    return status;
}

static void EnetRm_delCoreAttached(EnetRm_CoreAttachInfo *coreAttachInfo,
                                   uint32_t coreId)
{
    bool coreAttached;
    uint32_t attachIndex;

    coreAttached = EnetRm_isCoreAttached(coreAttachInfo, coreId, &attachIndex);
    Enet_assert(coreAttached == true);
    Enet_assert(attachIndex < ENET_ARRAYSIZE(coreAttachInfo->attachedCores));
    Enet_assert(coreAttachInfo->numCores > 0);
    /* move the last entry to the detached core slot */
    coreAttachInfo->numCores--;
    coreAttachInfo->attachedCores[attachIndex] =
        coreAttachInfo->attachedCores[coreAttachInfo->numCores];
}

static EnetRm_ResourceInfo *EnetRm_getCoreResourceInfo(EnetRm_Cfg *cfg,
                                                       uint32_t coreId)
{
    EnetRm_ResourceInfo *resInfo;
    uint32_t i;

    Enet_assert(cfg->resPartInfo.numCores <= ENET_ARRAYSIZE(cfg->resPartInfo.coreDmaResInfo));
    for (i = 0U; i < cfg->resPartInfo.numCores; i++)
    {
        if (cfg->resPartInfo.coreDmaResInfo[i].coreId == coreId)
        {
            break;
        }
    }

    if (i < cfg->resPartInfo.numCores)
    {
        resInfo = &cfg->resPartInfo.coreDmaResInfo[i];
    }
    else
    {
        resInfo = NULL;
    }

    return resInfo;
}

int32_t EnetRm_detachCore(EnetRm_Handle hRm,
                          uint32_t coreKey)
{
    uint32_t i;
    int32_t status;

    status = EnetRm_validateCoreKey(hRm, coreKey);
    if (status == ENET_SOK)
    {
        uint32_t coreId = ENET_COREKEY_2_COREID(coreKey);
        bool coreAttached;

        coreAttached = EnetRm_isCoreAttached(&hRm->coreAttachObj, coreId, (uint32_t *)NULL_PTR);
        if (coreAttached == true)
        {
            EnetRm_ResourceInfo *resInfo = EnetRm_getCoreResourceInfo(&hRm->cfg, coreId);
            EnetQ *txFreeQ  = EnetRm_getFreeQ(&hRm->txObj.txResTbl, coreId);
            EnetQ *macFreeQ = EnetRm_getFreeQ(&hRm->macObj.macTbl, coreId);
            uint32_t internalRxAllocCnt;

            if (resInfo == NULL)
            {
                Enet_assert(false);
            }
            else
            {
                Enet_assert(hRm->txObj.resCnt <= ENET_ARRAYSIZE(hRm->txObj.txRes));
                EnetRm_freeCoreResource(&hRm->txObj.txResTbl,
                                        hRm->txObj.txRes,
                                        hRm->txObj.resCnt,
                                        coreId);
                if (NULL != txFreeQ)
                {
                    Enet_assert(EnetQueue_getQCount(txFreeQ) == resInfo->numTxCh);
                }

                for (i = 0U; i < hRm->numRxCh; i++)
                {
                    EnetQ *rxFreeQ  = EnetRm_getFreeQ(&hRm->rxObj[i].rxResTbl, coreId);

                    Enet_assert(hRm->rxObj[i].resCnt <= ENET_ARRAYSIZE(hRm->rxObj[i].rxRes));
                    EnetRm_freeCoreResource(&hRm->rxObj[i].rxResTbl,
                                            hRm->rxObj[i].rxRes,
                                            hRm->rxObj[i].resCnt,
                                            coreId);
                    internalRxAllocCnt = 0;
                    if (coreId == hRm->rxObj[i].internalAllocCoreId)
                    {
                        internalRxAllocCnt = hRm->rxObj[i].internalAllocCount;
                    }

                    if (NULL != rxFreeQ)
                    {
                        Enet_assert((EnetQueue_getQCount(rxFreeQ) + internalRxAllocCnt) == resInfo->numRxFlows);
                    }
                }

                Enet_assert(hRm->macObj.resCnt <= ENET_ARRAYSIZE(hRm->macObj.macRes));
                EnetRm_freeCoreResource(&hRm->macObj.macTbl,
                                        hRm->macObj.macRes,
                                        hRm->macObj.resCnt,
                                        coreId);
                if (NULL != macFreeQ)
                {
                    Enet_assert(EnetQueue_getQCount(macFreeQ) == resInfo->numMacAddress);
                }

                EnetRm_delCoreAttached(&hRm->coreAttachObj, coreId);
            }
        }
        else
        {
            status = ENET_EINVALIDPARAMS;
        }
    }

    return status;
}

static int32_t EnetRm_getIoctlHandlerIdx(uint32_t ioctlCmd, EnetRmIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx)
{
    int32_t status;
    uint32_t i;

    for (i = 0; i < tableSize; i++)
    {
        if (ioctlRegistryTbl[i].cmd == ioctlCmd)
        {
            break;
        }
    }
    if (i < tableSize)
    {
        *tblIdx = i;
        status = ENET_SOK;
    }
    else
    {
        status = ENET_EINVALIDPARAMS;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get ioctl registry index for ioctl cmd: %x\n", ioctlCmd);
    return status;
}

static EnetRmIoctlHandler * EnetRm_getIoctlHandlerFxn(uint32_t ioctlCmd, EnetRmIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;
    EnetRmIoctlHandler *handlerFxn = NULL;

    status = EnetRm_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert(tblIdx < tableSize, "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        handlerFxn = ioctlRegistryTbl[tblIdx].fxn;
    }
    else
    {
        handlerFxn = &EnetRm_ioctl_handler_default;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get ioctl handler for ioctl cmd: %x. Using default ioctl handler \r\n", ioctlCmd);
    return handlerFxn;
}

static int32_t EnetRm_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                         EnetRmIoctlHandler *ioctlHandlerFxn,
                                         EnetRmIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                         uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;

    status = EnetRm_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert((tblIdx < tableSize), "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        Enet_devAssert((ioctlCmd == ioctlRegistryTbl[tblIdx].cmd), "Ioctl table corrupted");
        ioctlRegistryTbl[tblIdx].fxn = ioctlHandlerFxn;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set ioctl handler for ioctl cmd: %x \r\n", ioctlCmd);
    return status;
}

static int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_REGISTER_HANDLER(EnetRm_Handle hRm, Enet_IoctlPrms *prms)
{
    const Enet_IoctlRegisterHandlerInArgs *inArgs = (const Enet_IoctlRegisterHandlerInArgs *)prms->inArgs;
    int32_t status;

    status = EnetRm_setIoctlHandlerFxn(inArgs->cmd,
                                       (EnetRmIoctlHandler *)inArgs->fxn,
                                        EnetRmIoctlHandlerRegistry,
                                        ENET_ARRAYSIZE(EnetRmIoctlHandlerRegistry));
    return status;
}

static int32_t EnetRm_ioctl_handler_default(EnetRm_Handle hRm, Enet_IoctlPrms *prms)
{
    int32_t status;

    status = ENET_ENOTSUPPORTED;
    return status;
}
