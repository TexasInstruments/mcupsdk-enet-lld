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
 * \file  cpsw.c
 *
 * \brief This file contains the implementation of the CPSW_2G/CPSW_3G peripheral
 *        variant which is present in the AM273X and AM263x device.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>
#include <csl_cpswitch.h>
#include <enet_cfg.h>
#include <priv/mod/cpsw_ale_priv.h>
#include <priv/mod/cpsw_ale_ioctl_priv.h>
#include <priv/mod/cpsw_cpts_priv.h>
#include <priv/mod/cpsw_cpts_ioctl_priv.h>
#include <priv/mod/cpsw_hostport_priv.h>
#include <priv/mod/cpsw_macport_priv.h>
#include <priv/mod/cpsw_macport_ioctl_priv.h>
#include <priv/mod/mdio_priv.h>
#include <priv/mod/cpsw_stats_priv.h>
#include <priv/mod/phy_priv.h>
#include <priv/core/enet_rm_priv.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_osal.h>
#include <include/core/enet_soc.h>
#include <include/core/enet_per.h>
#include <include/per/cpsw.h>
#include <include/dma/cpdma/enet_cpdma.h>
#include <priv/per/cpsw_cpdma_priv.h>
#include <priv/per/cpsw_cpdma_ioctl_priv.h>
#include <priv/per/enet_hostport_cpdma.h>
#include <include/core/enet_utils.h>
#include <include/common/enet_phymdio_dflt.h>
#include <include/phy/enetphy.h>
#include <priv/per/cpsw_est_ioctl_priv.h>
#include <kernel/dpl/ClockP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*! \brief Default common TX MTU. */
#define CPSW_COMMON_TX_MTU_DEFAULT            (2024U)

/*!
 * \brief Priority escalation value for switch scheduler.
 *
 * When a port is in escalate priority, this is the number of higher priority
 * packets sent before the next lower priority is allowed to send a packet.
 * Escalate priority allows lower priority packets to be sent at a fixed rate
 * relative to the next higher priority.  The min value of esc_pri_ld_val = 2
 */
#define CPSW_ESC_PRI_LD_VAL                   (2U)

/*! \brief Number of CPDMA RX channels required for CPSW host port */
#define CPSW_CPDMA_NUM_RX_CH                   (1U)

#define CPSW_IOCTL_HANDLER_ENTRY_INIT(x)            \
          {.cmd = x,                                \
           .fxn = &Cpsw_internalIoctl_handler_##x}

#define CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(x)    \
          {.cmd = x,                                \
           .fxn = &Cpsw_internalIoctl_handler_default}

typedef int32_t (CpswIoctlHandler)(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct CpswAleIoctlHandlerRegistry_s
{
    uint32_t cmd;
    CpswIoctlHandler *fxn;
} CpswIoctlHandlerRegistry_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t Cpsw_openInternal(Cpsw_Handle hCpsw,
                                 Enet_Type enetType,
                                 uint32_t instId,
                                 const Cpsw_Cfg *cfg);

static void Cpsw_closeInternal(Cpsw_Handle hCpsw);

static void Cpsw_saveInternalCtxt(Cpsw_Handle hCpsw);

static int32_t Cpsw_restoreInternalCtxt(Cpsw_Handle hCpsw,
                                        Enet_Type enetType,
                                        uint32_t instId,
                                        const Cpsw_Cfg *cfg);

static int32_t Cpsw_ioctlInternal(EnetPer_Handle hPer,
                                  uint32_t cmd,
                                  Enet_IoctlPrms *prms);

static int32_t Cpsw_registerIntrs(Cpsw_Handle hCpsw,
                                  const Cpsw_Cfg *cfg);

static void Cpsw_unregisterIntrs(Cpsw_Handle hCpsw);

static void Cpsw_statsIsr(uintptr_t arg);

static void Cpsw_mdioIsr(uintptr_t arg);

static void Cpsw_cptsIsr(uintptr_t arg);

static void Cpsw_dmaRxThreshIsr(uintptr_t arg);

static void Cpsw_dmaRxIsr(uintptr_t arg);

static void Cpsw_dmaTxIsr(uintptr_t arg);

static void Cpsw_dmaMiscIsr(uintptr_t arg);

static void Cpsw_handleMdioLinkStateChange(EnetMdio_Group group,
                                           Mdio_PhyStatus *phyStatus,
                                           void *cbArgs);

static Cpsw_PortLinkState *Cpsw_getPortLinkState(Cpsw_Handle hCpsw,
                                                 uint32_t phyAddr);
static int32_t Cpsw_handleLinkUp(Cpsw_Handle hCpsw,
                                 Enet_MacPort macPort);

static int32_t Cpsw_registerIoctlHandler(EnetPer_Handle hPer,
                                         Enet_IoctlPrms *prms);

static int32_t Cpsw_registerInternalIoctlHandler(EnetPer_Handle hPer, Enet_IoctlPrms *prms);

static CpswIoctlHandler * Cpsw_getIoctlHandlerFxn(uint32_t ioctlCmd, 
                                                  CpswIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                                  uint32_t tableSize);
int32_t Cpsw_internalIoctl_handler_default(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
static int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_REGISTER_IOCTL_HANDLER(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_ERROR) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
const char *Cpsw_gSpeedNames[] =
{
    [ENET_SPEED_10MBIT]  = "10-Mbps",
    [ENET_SPEED_100MBIT] = "100-Mbps",
    [ENET_SPEED_1GBIT]   = "1-Gbps",
    [ENET_SPEED_AUTO]    = "auto",
};

const char *Cpsw_gDuplexNames[] =
{
    [ENET_DUPLEX_HALF] = "Half-Duplex",
    [ENET_DUPLEX_FULL] = "Full-Duplex",
    [ENET_DUPLEX_AUTO] = "auto",
};
#endif

/*!\ brief Default host and MAC port TX priority MTUs */
static const uint32_t Cpsw_txPriMtuDefault[] =
{
    CPSW_COMMON_TX_MTU_DEFAULT, /* TX Priority 0 MTU */
    CPSW_COMMON_TX_MTU_DEFAULT, /* TX Priority 1 MTU */
    CPSW_COMMON_TX_MTU_DEFAULT, /* TX Priority 2 MTU */
    CPSW_COMMON_TX_MTU_DEFAULT, /* TX Priority 3 MTU */
    CPSW_COMMON_TX_MTU_DEFAULT, /* TX Priority 4 MTU */
    CPSW_COMMON_TX_MTU_DEFAULT, /* TX Priority 5 MTU */
    CPSW_COMMON_TX_MTU_DEFAULT, /* TX Priority 6 MTU */
    CPSW_COMMON_TX_MTU_DEFAULT  /* TX Priority 7 MTU */
};

#if ENET_CFG_IS_ON(DEV_ERROR)
/* Public CPSW peripheral IOCTL validation data. */
static Enet_IoctlValidate gCpsw_ioctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_UNI_EGRESS,
                          sizeof(Cpsw_SetInterVlanRouteUniEgressInArgs),
                          sizeof(Cpsw_SetInterVlanRouteUniEgressOutArgs)),

    ENET_IOCTL_VALID_PRMS(CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_UNI_EGRESS,
                          sizeof(Cpsw_ClearInterVlanRouteUniEgressInArgs),
                          sizeof(CpswMacPort_InterVlanRouteId)),

    ENET_IOCTL_VALID_PRMS(CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_MULTI_EGRESS,
                          sizeof(Cpsw_SetInterVlanRouteMultiEgressInArgs),
                          sizeof(Cpsw_SetInterVlanRouteMultiEgressOutArgs)),

    ENET_IOCTL_VALID_PRMS(CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_MULTI_EGRESS,
                          sizeof(Cpsw_ClearInterVlanRouteMultiEgressInArgs),
                          sizeof(CpswMacPort_InterVlanRouteId)),

    ENET_IOCTL_VALID_PRMS(CPSW_PER_IOCTL_SET_SHORT_IPG_CFG,
                          sizeof(Cpsw_SetTxShortIpgCfgInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_PER_IOCTL_GET_SHORT_IPG_CFG,
                          0U,
                          sizeof(Cpsw_TxShortIpgCfg)),

};
#endif

static CpswIoctlHandlerRegistry_t CpswIoctlHandlerRegistry[] = 
{
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_GET_VERSION),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_PRINT_REGS),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_OPEN_PORT_LINK),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_CLOSE_PORT_LINK),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_IS_PORT_LINK_UP),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_SET_ISOLATE_STATE),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_CLEAR_ISOLATE_STATE),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_GET_PORT_LINK_CFG),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_ATTACH_CORE),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_DETACH_CORE),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_UNI_EGRESS),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_UNI_EGRESS),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_MULTI_EGRESS),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_MULTI_EGRESS),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_PER_IOCTL_SET_SHORT_IPG_CFG),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_PER_IOCTL_GET_SHORT_IPG_CFG),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_IOCTL_REGISTER_RX_DEFAULT_FLOW),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_IOCTL_UNREGISTER_RX_DEFAULT_FLOW),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_IOCTL_REGISTER_DSTMAC_RX_FLOW),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_IOCTL_UNREGISTER_DSTMAC_RX_FLOW),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_HANDLE_EXTPHY_LINKUP_EVENT),
    CPSW_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PER_IOCTL_HANDLE_EXTPHY_LINKDOWN_EVENT),
    CPSW_IOCTL_HANDLER_ENTRY_INIT(ENET_PER_IOCTL_REGISTER_IOCTL_HANDLER),
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Cpsw_initCfg(EnetPer_Handle hPer,
                  Enet_Type enetType,
                  void *cfg,
                  uint32_t cfgSize)
{
    Cpsw_Cfg *cpswCfg = (Cpsw_Cfg *)cfg;
    uint32_t i;

    Enet_devAssert(cfgSize == sizeof(Cpsw_Cfg),
                   "Invalid CPSW peripheral config params size %u (expected %u)\r\n",
                   cfgSize, sizeof(Cpsw_Cfg));

    cpswCfg->escalatePriorityLoadVal = CPSW_ESC_PRI_LD_VAL;

    /* VLAN configuration parameters */
    cpswCfg->vlanCfg.vlanAware  = false;
    cpswCfg->vlanCfg.vlanSwitch = ENET_VLAN_TAG_TYPE_INNER;
    cpswCfg->vlanCfg.outerVlan  = 0x88A8U;
    cpswCfg->vlanCfg.innerVlan  = 0x8100U;

    ENET_UTILS_COMPILETIME_ASSERT(ENET_ARRAYSIZE(cpswCfg->txMtu) == ENET_PRI_NUM);

    for (i = 0U; i < ENET_PRI_NUM; i++)
    {
        cpswCfg->txMtu[i] = Cpsw_txPriMtuDefault[i];
    }

    /* Initialize host port config params */
    CpswHostPort_initCfg(&cpswCfg->hostPortCfg);

    /* Initialize CPTS config params */
    CpswCpts_initCfg(&cpswCfg->cptsCfg);

    /* Initialize MDIO config params */
    Mdio_initCfg(&cpswCfg->mdioCfg);

    /* Initialize ALE params */
    CpswAle_initCfg(&cpswCfg->aleCfg);

    /* Initialize CPDMA params */
    EnetHostPortDma_initCfg(enetType, cpswCfg->dmaCfg);

    cpswCfg->intrPriority              = 1U;
    cpswCfg->mdioLinkStateChangeCb     = NULL;
    cpswCfg->mdioLinkStateChangeCbArg  = NULL;
    cpswCfg->portLinkStatusChangeCb    = NULL;
    cpswCfg->portLinkStatusChangeCbArg = NULL;
    cpswCfg->disablePhyDriver          = false;

}

int32_t Cpsw_open(EnetPer_Handle hPer,
                  Enet_Type enetType,
                  uint32_t instId,
                  const void *cfg,
                  uint32_t cfgSize)
{
    Cpsw_Handle hCpsw = (Cpsw_Handle)hPer;
    Cpsw_Cfg *cpswCfg = (Cpsw_Cfg *)cfg;
    Enet_IoctlPrms prms;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hPer->virtAddr;
    CSL_CPSW_PTYPE pType;
    uintptr_t key;
    uint32_t i;
    int32_t status = ENET_SOK;

    /* Saving CpswCfg context */
    hCpsw->context = *cpswCfg;
    
    Enet_devAssert(cfgSize == sizeof(Cpsw_Cfg),
                   "Invalid CPSW peripheral config params size %u (expected %u)\r\n",
                   cfgSize, sizeof(Cpsw_Cfg));

    /* Save EnetMod handles for easy access */
    for (i = 0U; i < hCpsw->macPortNum; i++)
    {
        hCpsw->hMacPort[i] = ENET_MOD(&hCpsw->macPortObj[i]);
    }

    hCpsw->hHostPort = ENET_MOD(&hCpsw->hostPortObj);
    hCpsw->hStats    = ENET_MOD(&hCpsw->statsObj);
    hCpsw->hAle      = ENET_MOD(&hCpsw->aleObj);
    hCpsw->hCpts     = ENET_MOD(&hCpsw->cptsObj);
    hCpsw->hMdio     = ENET_MOD(&hCpsw->mdioObj);
    hCpsw->hRm       = ENET_MOD(&hCpsw->rmObj);

    /* Open DMA */
    hCpsw->hDma = EnetHostPortDma_open(hPer, cpswCfg->dmaCfg, &(cpswCfg->resCfg));
    
    if (NULL == hCpsw->hDma)
    {
        ENETTRACE_ERR("Failed to open CPSW DMA\r\n");
        status = ENET_EFAIL;
    }

    if (status == ENET_SOK)
    {
        /* Set escalate priority value (number of higher priority packets to be sent
        * before next lower priority is allowed to send a packet */
        CSL_CPSW_getPTypeReg(regs, &pType);
        pType.escPriLoadVal = cpswCfg->escalatePriorityLoadVal;
        CSL_CPSW_setPTypeReg(regs, &pType);

        /* Set VLAN config: aware/non-aware, inner/outer tag */
        if (cpswCfg->vlanCfg.vlanAware)
        {
            CSL_CPSW_enableVlanAware(regs);
        }
        else
        {
            CSL_CPSW_disableVlanAware(regs);
        }

        CSL_CPSW_setVlanType(regs, (uint32_t)cpswCfg->vlanCfg.vlanSwitch);
        CSL_CPSW_setVlanLTypeReg(regs, cpswCfg->vlanCfg.innerVlan, cpswCfg->vlanCfg.outerVlan);

#if ENET_CFG_IS_ON(CPSW_EST)
        /* Enable EST global control */
        if (ENET_FEAT_IS_EN(hPer->features, CPSW_FEATURE_EST))
        {
            Cpsw_enableEst(hPer);
        }
#endif

        /* Set port global config */
        for (i = 0U; i < ENET_PRI_NUM; i++)
        {
            CSL_CPSW_setTxMaxLenPerPriority(regs, i, cpswCfg->txMtu[i]);

            /* Save largest of per priority port egress MTUs to check against
             * host port and MAC port RX MTUs */
            if (cpswCfg->txMtu[i] > hCpsw->maxPerPrioMtu)
            {
                hCpsw->maxPerPrioMtu = cpswCfg->txMtu[i];
            }
        }

        /* Initialize all CPSW modules */
        status = Cpsw_openInternal(hCpsw, enetType, instId, cpswCfg);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to open CPSW modules: %d\r\n", status);
    }

    key = EnetOsal_disableAllIntr();

    /* Register interrupts */
    if (status == ENET_SOK)
    {
        status = Cpsw_registerIntrs(hCpsw, cpswCfg);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to register interrupts: %d\r\n", status);
    }

    /* Enable CPTS interrupt */
    if (status == ENET_SOK)
    {
        ENET_IOCTL_SET_NO_ARGS(&prms);
        CPSW_CPTS_PRIV_IOCTL(hCpsw->hCpts, CPSW_CPTS_IOCTL_ENABLE_INTR, &prms, status);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to enable CPTS Interrupt: %d\r\n", status);
    }

    /* All initialization is complete */
    if (status == ENET_SOK)
    {
        hCpsw->selfCoreId = cpswCfg->resCfg.selfCoreId;
#if ENET_CFG_IS_ON(CPSW_EST)
        hCpsw->cptsRftClkFreq = cpswCfg->cptsCfg.cptsRftClkFreq;
#endif
    }
    else if (status != ENET_EALREADYOPEN)
    {
        /* Rollback if any error other than trying to open while already open */
        Cpsw_unregisterIntrs(hCpsw);
        Cpsw_closeInternal(hCpsw);
    }
    else
    {
        ENETTRACE_ERR("Unexpected status: %d\r\n", status);
    }

    EnetOsal_restoreAllIntr(key);

    return status;
}

int32_t Cpsw_rejoin(EnetPer_Handle hPer,
                    Enet_Type enetType,
                    uint32_t instId)
{
    Cpsw_Handle hCpsw = (Cpsw_Handle)hPer;
    uint32_t i;

    /* Save EnetMod handles for easy access */
    for (i = 0U; i < hCpsw->macPortNum; i++)
    {
        hCpsw->hMacPort[i] = ENET_MOD(&hCpsw->macPortObj[i]);
    }

    hCpsw->hHostPort = ENET_MOD(&hCpsw->hostPortObj);
    hCpsw->hStats    = ENET_MOD(&hCpsw->statsObj);
    hCpsw->hAle      = ENET_MOD(&hCpsw->aleObj);
    hCpsw->hCpts     = ENET_MOD(&hCpsw->cptsObj);
    hCpsw->hMdio     = ENET_MOD(&hCpsw->mdioObj);
    hCpsw->hRm       = ENET_MOD(&hCpsw->rmObj);

    return ENET_SOK;
}

void Cpsw_close(EnetPer_Handle hPer)
{
    Cpsw_Handle hCpsw = (Cpsw_Handle)hPer;
    Enet_IoctlPrms prms;
    uintptr_t key;
    int32_t status;

    key = EnetOsal_disableAllIntr();

    /* Disable CPTS interrupt */
    ENET_IOCTL_SET_NO_ARGS(&prms);
    CPSW_CPTS_PRIV_IOCTL(hCpsw->hCpts, CPSW_CPTS_IOCTL_DISABLE_INTR, &prms, status);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to disable CPTS Interrupt: %d\r\n", status);

    /* Unregister interrupts */
    if (status == ENET_SOK)
    {
        Cpsw_unregisterIntrs(hCpsw);
        Cpsw_closeInternal(hCpsw);
    }

#if ENET_CFG_IS_ON(CPSW_EST)
    /* Disable EST global control */
    if (ENET_FEAT_IS_EN(hPer->features, CPSW_FEATURE_EST))
    {
        Cpsw_disableEst(hPer);
    }
#endif

   EnetOsal_restoreAllIntr(key);
}

int32_t Cpsw_ioctl(EnetPer_Handle hPer,
                   uint32_t cmd,
                   Enet_IoctlPrms *prms)
{
    Cpsw_Handle hCpsw = (Cpsw_Handle)hPer;
    uint32_t major;
    int32_t status = ENET_SOK;

    if (ENET_IOCTL_GET_TYPE(cmd) != ENET_IOCTL_TYPE_PUBLIC)
    {
        ENETTRACE_ERR("%s: IOCTL cmd %u is not public\r\n", hPer->name, cmd);
        status = ENET_EINVALIDPARAMS;
    }

    /* Route IOCTL command to Per or Mod */
    if (status == ENET_SOK)
    {
        major = ENET_IOCTL_GET_MAJ(cmd);
        switch (major)
        {
            case ENET_IOCTL_PER_BASE:
            {
                status = Cpsw_ioctlInternal(hPer, cmd, prms);
            }
            break;

            case ENET_IOCTL_FDB_BASE:
            {
                status = EnetMod_ioctl(hCpsw->hAle, cmd, prms);
            }
            break;

            case ENET_IOCTL_TIMESYNC_BASE:
            {
                status = EnetMod_ioctl(hCpsw->hCpts, cmd, prms);
            }
            break;

            case ENET_IOCTL_HOSTPORT_BASE:
            {
                status = EnetMod_ioctl(hCpsw->hHostPort, cmd, prms);
            }
            break;

            case ENET_IOCTL_TAS_BASE:
            {
#if ENET_CFG_IS_ON(CPSW_EST)
#if ENET_CFG_IS_OFF(CPSW_MACPORT_EST)
#error "CPSW EST feature requires ENET_CFG_CPSW_MACPORT_EST"
#endif
                if (ENET_FEAT_IS_EN(hPer->features, CPSW_FEATURE_EST))
                {
                    status = Cpsw_ioctlEst(hPer, cmd, prms);
                }
                else
                {
                    status = ENET_ENOTSUPPORTED;
                }
#else
                status = ENET_ENOTSUPPORTED;
#endif
            }
            break;

            case ENET_IOCTL_MACPORT_BASE:
            {
                /* Note: Typecast to GenericInArgs is possible because all public
                 * MAC port IOCTL input args have macPort as their first member */
                EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
                uint32_t portNum = ENET_MACPORT_NORM(inArgs->macPort);

                if (portNum < EnetSoc_getMacPortMax(hPer->enetType, hPer->instId))
                {
                    status = EnetMod_ioctl(hCpsw->hMacPort[portNum], cmd, prms);
                }
                else
                {
                    status = ENET_EINVALIDPARAMS;
                }
            }
            break;

            case ENET_IOCTL_MDIO_BASE:
            {
                status = EnetMod_ioctl(hCpsw->hMdio, cmd, prms);
            }
            break;

            case ENET_IOCTL_STATS_BASE:
            {
                status = EnetMod_ioctl(hCpsw->hStats, cmd, prms);
            }
            break;

            case ENET_IOCTL_PHY_BASE:
            {
                /* Note: Typecast to GenericInArgs is possible because all public
                 * MAC port IOCTL input args have macPort as their first member */
                EnetPhy_GenericInArgs *inArgs = (EnetPhy_GenericInArgs *)prms->inArgs;
                uint32_t portNum = ENET_MACPORT_NORM(inArgs->macPort);

                /* Assert if port number is not correct */
                Enet_assert(portNum < EnetSoc_getMacPortMax(hPer->enetType, hPer->instId),
                            "Invalid Port Id: %u\r\n", portNum);

                const EnetPhy_Handle hPhy = hCpsw->hPhy[portNum];

                if (hPhy != NULL)
                {
                    status = EnetPhyMdioDflt_ioctl(hPhy, cmd, prms);
                }
                else
                {
                    status = ENET_EFAIL;
                }
            }
            break;

            case ENET_IOCTL_RM_BASE:
            {
                status = EnetMod_ioctl(hCpsw->hRm, cmd, prms);
            }
            break;

            default:
            {
                status = ENET_ENOTSUPPORTED;
            }
            break;
        }
    }

    return status;
}

void Cpsw_poll(EnetPer_Handle hPer,
               Enet_Event evt,
               const void *arg,
               uint32_t argSize)
{
    /* Nothing to do, not supported */
    ENETTRACE_WARN("Poll feature is not supported by CPSW Per\r\n");
}

void Cpsw_periodicTick(EnetPer_Handle hPer)
{
    Cpsw_Handle hCpsw = (Cpsw_Handle)hPer;
    EnetPhy_Handle hPhy;
    Enet_MacPort macPort;
    Cpsw_PortLinkState *portLinkState;
    uint32_t maxPorts = 0U;
    uint32_t portId;
    uint32_t i;
    int32_t status;
    uint32_t numPortCb = 0;
    struct Cpsw_PortLinkCbInfoList_s
    {
        bool linked;
        Enet_MacPort macPort;
    } portCbInfoList[CPSW_MAC_PORT_NUM];


    /* Get the max number of ports */
    maxPorts = EnetSoc_getMacPortMax(hPer->enetType, hPer->instId);

    /* Run PHY tick and handle link up/down events */
    for (i = 0U; i < maxPorts; i++)
    {
        macPort = ENET_MACPORT_DENORM(i);
        portId = ENET_MACPORT_ID(macPort);

        ENETTRACE_VAR(portId);
        hPhy = hCpsw->hPhy[i];
        portLinkState = &hCpsw->portLinkState[i];

        /* Check if the corresponding PHY is enabled */
        if (portLinkState->isTickEnabled)
        {
            /* TODO: Need to make lock more granular */
            //EnetOsal_lockMutex(hCpsw->lock);

            if (hPhy != NULL)
            {
                /* Run PHY tick */
                const EnetPhy_LinkStatus linkStatus = EnetPhy_tick(hPhy);
                

                /* Handle link up/down events */
                if ((linkStatus == ENETPHY_GOT_LINK) ||
                    (linkStatus == ENETPHY_LOST_LINK))
                {
                    const bool linked = (linkStatus == ENETPHY_GOT_LINK);
                    status = linked ? Cpsw_handleLinkUp(hCpsw, macPort) :
                                      Cpsw_handleLinkDown(hCpsw, macPort);
                    ENETTRACE_ERR_IF(status != ENET_SOK,
                                     "Port %u: Failed to handle link change: %d\r\n", portId, status);

                    /* Call application callback when port link is up - at this point app can
                     * start data flow */
                    if ((status == ENET_SOK) && (hCpsw->portLinkStatusChangeCb != NULL))
                    {
                        /* Add port's to callback info list.
                         * All portLinkStatus Cb functions are invoked at the
                         * end of function after relinquishing locks */
                        Enet_devAssert(numPortCb < ENET_ARRAYSIZE(portCbInfoList),
                                       "Invalid port number %u, expected < %u\r\n",
                                       numPortCb, ENET_ARRAYSIZE(portCbInfoList));
                        portCbInfoList[numPortCb].macPort = macPort;
                        portCbInfoList[numPortCb].linked  = linked;
                        numPortCb++;
                    }

                    /* All checks cleared, link state can be updated now */
                    if (status == ENET_SOK)
                    {
                        portLinkState->isLinkUp = linked;

                        /* Disable periodic tick while link is up. It will be re-enabled
                         * by MDIO_LINKINT upon link down detection */
                        if (portLinkState->isPollEnabled && linked)
                        {
                            portLinkState->isTickEnabled = false;
                        }
                    }
                } /* if ((linkStatus == ENETPHY_GOT_LINK) || (linkStatus == ENETPHY_LOST_LINK)) */
            } /* if (hPhy != NULL) */
            else if (EnetMod_isOpen(hCpsw->hMacPort[i]))
            {
                /* If port is in NOPHY mode, invoke the portLinkUp Cb */
                portLinkState->isLinkUp      = true;
                portLinkState->isTickEnabled = false;
                if (hCpsw->portLinkStatusChangeCb != NULL)
                {
                    /* Add port's to callback info list.
                     * All portLinkStatus Cb functions are invoked at the
                     * end of function after relinquishing locks */
                    Enet_devAssert(numPortCb < ENET_ARRAYSIZE(portCbInfoList),
                                   "Invalid port number %u, expected < %u\r\n",
                                   numPortCb, ENET_ARRAYSIZE(portCbInfoList));
                    portCbInfoList[numPortCb].macPort = macPort;
                    portCbInfoList[numPortCb].linked  = true;
                    numPortCb++;
                }
            }

            //EnetOsal_unlockMutex(hCpsw->lock);
        }
    }

    for (i = 0U; i < numPortCb; i++)
    {
        if (hCpsw->portLinkStatusChangeCb != NULL)
        {
            /* Call application's port link status change callback */
            hCpsw->portLinkStatusChangeCb(portCbInfoList[i].macPort,
                                          portCbInfoList[i].linked,
                                          hCpsw->portLinkStatusChangeCbArg);
        }
    }

    return;
}

EnetDma_Handle Cpsw_getDmaHandle(const Enet_Handle hEnet)
{
    Cpsw_Handle hCpsw = (Cpsw_Handle)Enet_getPerHandle(hEnet);

    return(hCpsw->hDma);
}


static int32_t Cpsw_openInternal(Cpsw_Handle hCpsw,
                                 Enet_Type enetType,
                                 uint32_t instId,
                                 const Cpsw_Cfg *cfg)
{
    Cpsw_MdioLinkIntCtx *linkIntCtx = &hCpsw->mdioLinkIntCtx;
    Cpsw_PortLinkState *portLinkState;
    uint32_t i;
    int32_t status = ENET_SOK;

    /* Clear PHY link state */
    for (i = 0U; i < ENET_ARRAYSIZE(hCpsw->portLinkState); i++)
    {
        portLinkState = &hCpsw->portLinkState[i];

        portLinkState->phyAddr       = ENETPHY_INVALID_PHYADDR;
        portLinkState->isOpen        = false;
        portLinkState->isLinkUp      = false;
        portLinkState->isPollEnabled = false;
        portLinkState->isTickEnabled = true;
    }

    linkIntCtx->aliveMask            = ENET_MDIO_PHY_ADDR_MASK_NONE;
    linkIntCtx->linkedMask           = ENET_MDIO_PHY_ADDR_MASK_NONE;
    linkIntCtx->pollEnableMask       = cfg->mdioCfg.pollEnMask;
    linkIntCtx->linkStateChangeCb    = cfg->mdioLinkStateChangeCb;
    linkIntCtx->linkStateChangeCbArg = cfg->mdioLinkStateChangeCbArg;
    hCpsw->portLinkStatusChangeCb    = cfg->portLinkStatusChangeCb;
    hCpsw->portLinkStatusChangeCbArg = cfg->portLinkStatusChangeCbArg;
    hCpsw->disablePhyDriver          = cfg->disablePhyDriver;
    /* Host port and MAC port MTUs should not be greater than largest of the port
     * egress per priority MTU as packet would get dropped by switch.
     * Though it is valid from hardware configuration, we return an error as it
     * serves no purpose */
    if (cfg->hostPortCfg.rxMtu > hCpsw->maxPerPrioMtu)
    {
        ENETTRACE_ERR("Host Port RX MTU (%d) exceeds max of TX Priority 0-7 MTU (%d)\r\n ",
                      cfg->hostPortCfg.rxMtu, hCpsw->maxPerPrioMtu);
        status = ENET_EINVALIDPARAMS;
    }

    /* Open host port */
    if (status == ENET_SOK)
    {
        
        status = EnetMod_open(hCpsw->hHostPort, enetType, instId, &cfg->hostPortCfg, sizeof(cfg->hostPortCfg));
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to open host port: %d\r\n", status);
    }

    /* Open ALE */
    if (status == ENET_SOK)
    {
        status = EnetMod_open(hCpsw->hAle, enetType, instId, &cfg->aleCfg, sizeof(cfg->aleCfg));
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to open ALE: %d\r\n", status);
    }

    /* Open CPTS */
    if (status == ENET_SOK)
    {
        status = EnetMod_open(hCpsw->hCpts, enetType, instId, &cfg->cptsCfg, sizeof(cfg->cptsCfg));
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to open CPTS: %d\r\n", status);
    }

    /* Open MDIO */
    if (status == ENET_SOK)
    {
        status = EnetMod_open(hCpsw->hMdio, enetType, instId, &cfg->mdioCfg, sizeof(cfg->mdioCfg));
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to open MDIO: %d\r\n", status);
    }

    /* Open statistics */
    if (status == ENET_SOK)
    {
        status = EnetMod_open(hCpsw->hStats, enetType, instId, NULL, 0U);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to open stats: %d\r\n", status);
    }

    /* Open Resource Manager if Rx Channel open succeeded */
    if (status == ENET_SOK)
    {
        EnetRm_Cfg rmCfg;

        rmCfg.enetType              = enetType;
        rmCfg.instId                = instId;
        rmCfg.numRxCh               = CPSW_CPDMA_NUM_RX_CH;
        rmCfg.rxStartFlowIdx[0U]    = 0;
        rmCfg.rxFlowIdxCnt[0U]      = ENET_CPDMA_CPSW_MAX_RX_CH;
        rmCfg.ioctlPermissionInfo   = cfg->resCfg.ioctlPermissionInfo;
        rmCfg.macList               = cfg->resCfg.macList;
        rmCfg.resPartInfo = cfg->resCfg.resPartInfo;

        status = EnetMod_open(hCpsw->hRm, enetType, instId, &rmCfg, sizeof(rmCfg));
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to open RM: %d\r\n", status);
    }

    if (status != ENET_SOK)
    {
        Cpsw_closeInternal(hCpsw);
    }

    return status;
}
static void Cpsw_closeInternal(Cpsw_Handle hCpsw)
{
    EnetMod_close(hCpsw->hHostPort);
    EnetMod_close(hCpsw->hAle);
    EnetMod_close(hCpsw->hCpts);
    EnetMod_close(hCpsw->hMdio);
    EnetMod_close(hCpsw->hStats);

    Enet_assert(hCpsw->hDma != NULL);
    EnetHostPortDma_close(hCpsw->hDma);
    hCpsw->hDma = NULL;
    EnetMod_close(hCpsw->hRm);
}

static void Cpsw_isolatePhy(Cpsw_Handle hCpsw)
{
    uint32_t portNum = 0;
    bool linked;
    Enet_MacPort macPort;

    for(portNum = 0; portNum < hCpsw->macPortNum; portNum++)
    {
        Enet_assert(portNum <= CPSW_MAC_PORT_NUM);
        if(hCpsw->hPhy[portNum] != NULL)
        {
            linked = hCpsw->portLinkState[portNum].isLinkUp;

            if (linked)
            {
                macPort = ENET_MACPORT_DENORM(portNum);
                /* Force link-down handling to undo settings done for this port link
                 * i.e. ALE port state, MAC port disable, etc */
                Cpsw_handleLinkDown(hCpsw, macPort);

                /* Requesting for PHY Isolation */
                hCpsw->hPhy[portNum]->phyCfg.isIsolateStateReq = true;

                /* PHY tick to ensure isolation state take place without waiting for Cpsw_periodicTick*/
                EnetPhy_tick(hCpsw->hPhy[portNum]);
            }
        }
    }
}

void Cpsw_saveCtxt(EnetPer_Handle hPer)
{
    Cpsw_Handle hCpsw = (Cpsw_Handle)hPer;
    Enet_IoctlPrms prms;
    uintptr_t key;
    int32_t status;

    key = EnetOsal_disableAllIntr();

    /* Isolating PHY */
    Cpsw_isolatePhy(hCpsw);

    /* Disable CPTS interrupt */
    ENET_IOCTL_SET_NO_ARGS(&prms);
    CPSW_CPTS_PRIV_IOCTL(hCpsw->hCpts, CPSW_CPTS_IOCTL_DISABLE_INTR, &prms, status);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to disable CPTS Interrupt: %d\r\n", status);

    /* Unregister interrupts */
    if (status == ENET_SOK)
    {
        Cpsw_unregisterIntrs(hCpsw);
        Cpsw_saveInternalCtxt(hCpsw);
    }

#if ENET_CFG_IS_ON(CPSW_EST)
    /* Disable EST global control */
    if (ENET_FEAT_IS_EN(hPer->features, CPSW_FEATURE_EST))
    {
        Cpsw_disableEst(hPer);
    }
#endif

   EnetOsal_restoreAllIntr(key);
}

int32_t Cpsw_restoreCtxt(EnetPer_Handle hPer,
                         Enet_Type enetType,
                         uint32_t instId)
{
    Cpsw_Handle hCpsw = (Cpsw_Handle)hPer;
    Cpsw_Cfg *cpswCfg = &hCpsw->context;
    Enet_IoctlPrms prms;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hPer->virtAddr;
    CSL_CPSW_PTYPE pType;
    uintptr_t key;
    uint32_t i;
    int32_t status = ENET_SOK;

    Enet_devAssert(sizeof(hCpsw->context) == sizeof(Cpsw_Cfg),
                   "Invalid CPSW peripheral config params size %u (expected %u)\r\n",
                   hCpsw->context, sizeof(Cpsw_Cfg));

    /* Save EnetMod handles for easy access */
    for (i = 0U; i < hCpsw->macPortNum; i++)
    {
        hCpsw->hMacPort[i] = ENET_MOD(&hCpsw->macPortObj[i]);
    }

    hCpsw->hHostPort = ENET_MOD(&hCpsw->hostPortObj);
    hCpsw->hStats    = ENET_MOD(&hCpsw->statsObj);
    hCpsw->hAle      = ENET_MOD(&hCpsw->aleObj);
    hCpsw->hCpts     = ENET_MOD(&hCpsw->cptsObj);
    hCpsw->hMdio     = ENET_MOD(&hCpsw->mdioObj);
    hCpsw->hRm       = ENET_MOD(&hCpsw->rmObj);

    /* Open DMA */
    hCpsw->hDma = EnetHostPortDma_restoreCtxt(hPer, &(cpswCfg->resCfg));
    if (NULL == hCpsw->hDma)
    {
        ENETTRACE_ERR("Failed to open CPSW DMA\r\n");
        status = ENET_EFAIL;
    }

    if (status == ENET_SOK)
    {

        /* Set escalate priority value (number of higher priority packets to be sent
        * before next lower priority is allowed to send a packet */
        CSL_CPSW_getPTypeReg(regs, &pType);
        pType.escPriLoadVal = cpswCfg->escalatePriorityLoadVal;
        CSL_CPSW_setPTypeReg(regs, &pType);

        /* Set VLAN config: aware/non-aware, inner/outer tag */
        if (cpswCfg->vlanCfg.vlanAware)
        {
            CSL_CPSW_enableVlanAware(regs);
        }
        else
        {
            CSL_CPSW_disableVlanAware(regs);
        }

        CSL_CPSW_setVlanType(regs, (uint32_t)cpswCfg->vlanCfg.vlanSwitch);
        CSL_CPSW_setVlanLTypeReg(regs, cpswCfg->vlanCfg.innerVlan, cpswCfg->vlanCfg.outerVlan);

#if ENET_CFG_IS_ON(CPSW_EST)
        /* Enable EST global control */
        if (ENET_FEAT_IS_EN(hPer->features, CPSW_FEATURE_EST))
        {
            Cpsw_enableEst(hPer);
        }
#endif

        /* Set port global config */
        for (i = 0U; i < ENET_PRI_NUM; i++)
        {
            CSL_CPSW_setTxMaxLenPerPriority(regs, i, cpswCfg->txMtu[i]);

            /* Save largest of per priority port egress MTUs to check against
             * host port and MAC port RX MTUs */
            if (cpswCfg->txMtu[i] > hCpsw->maxPerPrioMtu)
            {
                hCpsw->maxPerPrioMtu = cpswCfg->txMtu[i];
            }
        }

        /* Initialize all CPSW modules */
		status = Cpsw_restoreInternalCtxt(hCpsw, enetType, instId, cpswCfg);
		ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to open CPSW modules: %d\r\n", status);

    }



    key = EnetOsal_disableAllIntr();

    /* Register interrupts */
    if (status == ENET_SOK)
    {
        status = Cpsw_registerIntrs(hCpsw, cpswCfg);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to register interrupts: %d\r\n", status);
    }

    /* Enable CPTS interrupt */
    if (status == ENET_SOK)
    {
        ENET_IOCTL_SET_NO_ARGS(&prms);
        CPSW_CPTS_PRIV_IOCTL(hCpsw->hCpts, CPSW_CPTS_IOCTL_ENABLE_INTR, &prms, status);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to enable CPTS Interrupt: %d\r\n", status);
    }

    /* All initialization is complete */
    if (status == ENET_SOK)
    {
        hCpsw->selfCoreId = cpswCfg->resCfg.selfCoreId;
#if ENET_CFG_IS_ON(CPSW_EST)
        hCpsw->cptsRftClkFreq = cpswCfg->cptsCfg.cptsRftClkFreq;
#endif
    }
    else if (status != ENET_EALREADYOPEN)
    {
        /* Rollback if any error other than trying to open while already open */
        Cpsw_unregisterIntrs(hCpsw);
        Cpsw_closeInternal(hCpsw);
    }
    else
    {
        ENETTRACE_ERR("Unexpected status: %d\r\n", status);
    }

    /* Moving the PHY from Isolation state to deIsolation state */
    for (i = 0U; i < hCpsw->macPortNum; i++)
    {
        if(hCpsw->hPhy[i] != NULL)
        {
            hCpsw->hPhy[i]->phyCfg.isIsolateStateReq = false;
            hCpsw->portLinkState[i].isTickEnabled = true;
            if (status == ENET_SOK)
            {
                /* Restoring Macport post Reset */
                status = EnetMod_restoreCtxt(hCpsw->hMacPort[i], enetType, instId, NULL, 0U);
                ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to open Macport: %d\r\n", status);
            }
        }
    }
    EnetOsal_restoreAllIntr(key);


    return status;
}

static void Cpsw_saveInternalCtxt(Cpsw_Handle hCpsw)
{
    uint32_t portNum;

    /* Resetting DMA */
    Enet_assert(hCpsw->hDma != NULL);
    EnetHostPortDma_saveCtxt(hCpsw->hDma);
    hCpsw->hDma = NULL;

    /* Save Host Port Context */
    EnetMod_saveCtxt(hCpsw->hHostPort);

    /* Save Macport Context */

    for(portNum = 0; portNum < hCpsw->macPortNum; portNum++)
    {
        EnetMod_saveCtxt(hCpsw->hMacPort[portNum]);
    }

    /* Save ALE Context */
    EnetMod_saveCtxt(hCpsw->hAle);

    /* Save CPTS Context */
    EnetMod_saveCtxt(hCpsw->hCpts);

    /* Save MDIO Context */
    EnetMod_saveCtxt(hCpsw->hMdio);

    /* Save Stats Context */
    EnetMod_saveCtxt(hCpsw->hStats);
}

static int32_t Cpsw_restoreInternalCtxt(Cpsw_Handle hCpsw,
                                        Enet_Type enetType,
                                        uint32_t instId,
                                        const Cpsw_Cfg *cfg)
{
    int32_t status = ENET_SOK;

    /* Restore hostport DMA */
	status = EnetMod_restoreCtxt(hCpsw->hHostPort, enetType, instId, &cfg->hostPortCfg, sizeof(cfg->hostPortCfg));
	ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to open host port: %d\r\n", status);

    /* Open ALE */
    if (status == ENET_SOK)
    {
        status = EnetMod_restoreCtxt(hCpsw->hAle, enetType, instId, &cfg->aleCfg, sizeof(cfg->aleCfg));
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to open ALE: %d\r\n", status);
    }

    /* Open CPTS */
    if (status == ENET_SOK)
    {
        status = EnetMod_restoreCtxt(hCpsw->hCpts, enetType, instId, &cfg->cptsCfg, sizeof(cfg->cptsCfg));
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to open CPTS: %d\r\n", status);
    }

    /* Open MDIO */
    if (status == ENET_SOK)
    {
        status = EnetMod_restoreCtxt(hCpsw->hMdio, enetType, instId, &cfg->mdioCfg, sizeof(cfg->mdioCfg));
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to open MDIO: %d\r\n", status);
    }

    /* Open statistics */
    if (status == ENET_SOK)
    {
        status = EnetMod_restoreCtxt(hCpsw->hStats, enetType, instId, NULL, 0U);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to open stats: %d\r\n", status);
    }

    if (status != ENET_SOK)
    {
        Cpsw_closeInternal(hCpsw);
    }

    return status;
}

static int32_t Cpsw_ioctlInternal(EnetPer_Handle hPer,
                                  uint32_t cmd,
                                  Enet_IoctlPrms *prms)
{
    Cpsw_Handle hCpsw = (Cpsw_Handle)hPer;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hPer->virtAddr;
    int32_t status = ENET_SOK;

#if ENET_CFG_IS_ON(DEV_ERROR)
    /* Validate CPSW peripheral IOCTL parameters */
    if ((ENET_IOCTL_GET_PER(cmd) == ENET_IOCTL_PER_CPSW) &&
        (ENET_IOCTL_GET_TYPE(cmd) == ENET_IOCTL_TYPE_PUBLIC))
    {
        status = Enet_validateIoctl(cmd, prms,
                                    gCpsw_ioctlValidate,
                                    ENET_ARRAYSIZE(gCpsw_ioctlValidate));
        ENETTRACE_ERR_IF(status != ENET_SOK, "IOCTL 0x%08x params are not valid\r\n", cmd);
    }
#endif
    if (status == ENET_SOK)
    {
        CpswIoctlHandler * ioctlHandlerFxn;

        ioctlHandlerFxn = Cpsw_getIoctlHandlerFxn(cmd, CpswIoctlHandlerRegistry, ENET_ARRAYSIZE(CpswIoctlHandlerRegistry));
        Enet_devAssert(ioctlHandlerFxn != NULL);
        status = ioctlHandlerFxn(hCpsw, regs, prms);
    }
    return status;
}

static int32_t Cpsw_registerIntrs(Cpsw_Handle hCpsw,
                                  const Cpsw_Cfg *cfg)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hCpsw;
    Enet_Type enetType = hPer->enetType;
    uint32_t instId = hPer->instId;
    uint32_t intrNum;
    uint32_t trigType;
    int32_t status = ENET_SOK;
    uint32_t statusMask;

    /* Register Rx Threshold interrupt */

    intrNum = EnetSoc_getIntrNum(enetType, instId, CPSW_INTR_RX_THRESH);
    trigType = EnetSoc_getIntrTriggerType(enetType, instId, CPSW_INTR_RX_THRESH);
    hCpsw->hDmaRxThreshIntr = EnetOsal_registerIntr(Cpsw_dmaRxThreshIsr,
                                                    intrNum,
                                                    cfg->intrPriority,
                                                    trigType,
                                                    hCpsw);
    if (hCpsw->hDmaRxThreshIntr == NULL)
    {
        ENETTRACE_ERR("Failed to register rx thresh intr\r\n");
        status = ENET_EFAIL;
    }

    /* Register Rx interrupt */
    if (status == ENET_SOK)
    {
        intrNum = EnetSoc_getIntrNum(enetType, instId, CPSW_INTR_RX_PEND);
        trigType = EnetSoc_getIntrTriggerType(enetType, instId, CPSW_INTR_RX_PEND);
        hCpsw->hDmaRxIntr = EnetOsal_registerIntr(Cpsw_dmaRxIsr,
                                                  intrNum,
                                                  cfg->intrPriority,
                                                  trigType,
                                                  hCpsw);
        if (hCpsw->hDmaRxIntr == NULL)
        {
            ENETTRACE_ERR("Failed to register rx intr\r\n");
            status = ENET_EFAIL;
        }
    }

     /* Register Tx interrupt */
    if (status == ENET_SOK)
    {
        intrNum = EnetSoc_getIntrNum(enetType, instId, CPSW_INTR_TX_PEND);
        trigType = EnetSoc_getIntrTriggerType(enetType, instId, CPSW_INTR_TX_PEND);
        hCpsw->hDmaTxIntr = EnetOsal_registerIntr(Cpsw_dmaTxIsr,
                                                  intrNum,
                                                  cfg->intrPriority,
                                                  trigType,
                                                  hCpsw);
        if (hCpsw->hDmaTxIntr == NULL)
        {
            ENETTRACE_ERR("Failed to register tx intr\r\n");
            status = ENET_EFAIL;
        }
    }

     /* Register Miscellaneous interrupt */
    if (status == ENET_SOK)
    {
        intrNum = EnetSoc_getIntrNum(enetType, instId, CPSW_INTR_MISC_PEND);
        trigType = EnetSoc_getIntrTriggerType(enetType, instId, CPSW_INTR_MISC_PEND);

        if ((hCpsw->mdioLinkIntCtx.linkStateChangeCb != NULL) &&
            (cfg->mdioCfg.mode == MDIO_MODE_MANUAL))
        {
            ENETTRACE_ERR("MDIO INT callbacks should be not be set in MDIO manual mode\r\n");
            status = ENET_EINVALIDPARAMS;
        }
        else
        {
            hCpsw->hDmaMiscIntr = EnetOsal_registerIntr(Cpsw_dmaMiscIsr,
                                                        intrNum,
                                                        cfg->intrPriority,
                                                        trigType,
                                                        hCpsw);
            if (hCpsw->hDmaMiscIntr == NULL)
            {
                ENETTRACE_ERR("Failed to register misc intr\r\n");
                status = ENET_EFAIL;
            }
        }
    }

    /* Unwind in case of any error */
    if (status != ENET_SOK)
    {
        if (hCpsw->hDmaRxThreshIntr != NULL)
        {
            EnetOsal_unregisterIntr(hCpsw->hDmaRxThreshIntr);
        }

        if (hCpsw->hDmaRxIntr != NULL)
        {
            EnetOsal_unregisterIntr(hCpsw->hDmaRxIntr);
        }

        if (hCpsw->hDmaTxIntr != NULL)
        {
            EnetOsal_unregisterIntr(hCpsw->hDmaTxIntr);
        }

        if (hCpsw->hDmaMiscIntr != NULL)
        {
            EnetOsal_unregisterIntr(hCpsw->hDmaMiscIntr);
        }
    }
    else
    {
        EnetDma_Handle hEnetDma = hCpsw->hDma;
        Enet_assert(hEnetDma != NULL);
        /*
         *  If there is any active Misc ISR, process them now so that
         *  peripheral interrupt source, CPDMA EOI are acknowledged before enabling interrupts.
         *  Else no further interrupts will occur as the pending ISR is not acknowledged
         */
        status = EnetCpdma_miscIsrGetStatus(hEnetDma, &statusMask);
        Enet_assert(status == ENET_SOK);
        if (statusMask != 0U)
        {
            Cpsw_dmaMiscIsr((uintptr_t)hCpsw);
        }
    }

    return status;
}

static void Cpsw_unregisterIntrs(Cpsw_Handle hCpsw)
{
    /* Unregister Rx Threshold interrupt */
    if (hCpsw->hDmaRxThreshIntr != NULL)
    {
        EnetOsal_unregisterIntr(hCpsw->hDmaRxThreshIntr);
    }

    /* Unregister Rx interrupt */
    if (hCpsw->hDmaRxIntr != NULL)
    {
        EnetOsal_unregisterIntr(hCpsw->hDmaRxIntr);
    }

    /* Unregister Tx interrupt */
    if (hCpsw->hDmaTxIntr != NULL)
    {
        EnetOsal_unregisterIntr(hCpsw->hDmaTxIntr);
    }

    /* Unregister Miscellaneous interrupt */
    if (hCpsw->hDmaMiscIntr != NULL)
    {
        EnetOsal_unregisterIntr(hCpsw->hDmaMiscIntr);
    }
}

static void Cpsw_statsIsr(uintptr_t arg)
{
    EnetMod_Handle hStats = (EnetMod_Handle)arg;
    Enet_IoctlPrms prms;
    int32_t status;

    ENET_IOCTL_SET_NO_ARGS(&prms);
    status = EnetMod_ioctlFromIsr(hStats, CPSW_STATS_IOCTL_SYNC, &prms);

    /* TODO: Add ISR safe error:
     * ("Failed to sync statistics counters: %d\r\n", status); */
    ENET_UNUSED(status);
}

static void Cpsw_mdioIsr(uintptr_t arg)
{
    Cpsw_Handle hCpsw = (Cpsw_Handle)arg;
    EnetMod_Handle hMdio = hCpsw->hMdio;
    Enet_IoctlPrms prms;
    Mdio_Callbacks callbacks =
    {
        .linkStateCb  = Cpsw_handleMdioLinkStateChange,
        .userAccessCb = NULL,
        .cbArgs       = hCpsw,
    };
    int32_t status;

    ENET_IOCTL_SET_IN_ARGS(&prms, &callbacks);
    status = EnetMod_ioctlFromIsr(hMdio, MDIO_IOCTL_HANDLE_INTR, &prms);

    /* TODO: Add ISR safe error:
     * ("Failed to handle MDIO intr: %d\r\n", status); */
    ENET_UNUSED(status);
}

static void Cpsw_cptsIsr(uintptr_t arg)
{
    Cpsw_Handle hCpsw = (Cpsw_Handle)arg;
    EnetMod_Handle hCpts = hCpsw->hCpts;
    Enet_IoctlPrms prms;
    int32_t status;

    ENET_IOCTL_SET_NO_ARGS(&prms);
    status = EnetMod_ioctlFromIsr(hCpts, CPSW_CPTS_IOCTL_HANDLE_INTR, &prms);

    /* TODO: Add ISR safe error:
     * ("Failed to handle CPTS intr: %d\r\n", status); */
    ENET_UNUSED(status);
}

static void Cpsw_dmaRxThreshIsr(uintptr_t arg)
{
    Cpsw_Handle hCpsw = (Cpsw_Handle)arg;
    EnetDma_Handle hEnetDma = hCpsw->hDma;
    int32_t status;

    Enet_assert(hEnetDma != NULL);

    status = EnetCpdma_rxThreshIsr(hEnetDma);

    /* TODO: Add ISR safe error:
     * failed to handle Rx Thresh intr: %d\r\n", status); */
    ENET_UNUSED(status);
}

static void Cpsw_dmaRxIsr(uintptr_t arg)
{
    Cpsw_Handle hCpsw = (Cpsw_Handle)arg;
    EnetDma_Handle hEnetDma = hCpsw->hDma;
    int32_t status;
    uintptr_t key;

    key = EnetOsal_disableAllIntr();
    Enet_assert(hEnetDma != NULL);

    status = EnetCpdma_rxIsr(hEnetDma);

    /* TODO: Add ISR safe error:
     * failed to handle Rx intr: %d\r\n", status); */
    ENET_UNUSED(status);
    EnetOsal_restoreAllIntr(key);
}

static void Cpsw_dmaTxIsr(uintptr_t arg)
{
    Cpsw_Handle hCpsw = (Cpsw_Handle)arg;
    EnetDma_Handle hEnetDma = hCpsw->hDma;
    int32_t status;
    uintptr_t key;

    Enet_assert(hEnetDma != NULL);
    key = EnetOsal_disableAllIntr();

    status = EnetCpdma_txIsr(hEnetDma);

    /* TODO: Add ISR safe error:
     * failed to handle Tx intr: %d\r\n", status); */
    ENET_UNUSED(status);
    EnetOsal_restoreAllIntr(key);
}

static void Cpsw_dmaMiscIsr(uintptr_t arg)
{
    Cpsw_Handle hCpsw = (Cpsw_Handle)arg;
    EnetDma_Handle hEnetDma = hCpsw->hDma;
    int32_t status;
    uint32_t statusMask;

    Enet_assert(hEnetDma != NULL);

    status = EnetCpdma_miscIsrGetStatus(hEnetDma, &statusMask);
    Enet_assert(status == ENET_SOK);

    if ((statusMask & (CPSW_MISC_INT_MDIO_USERINT_MASK | CPSW_MISC_INT_MDIO_LINKINT_MASK)) != 0U)
    {
        Cpsw_mdioIsr(arg);
    }

    if ((statusMask & CPSW_MISC_INT_CPTS_EVENT_MASK) != 0U)
    {
        Cpsw_cptsIsr(arg);
    }

    if (((statusMask & CPSW_MISC_INT_STAT_OVERFLOW_MASK) != 0U)
        ||
        ((statusMask & CPSW_MISC_INT_HOSTERR_MASK) != 0U))
    {
        Cpsw_statsIsr((uintptr_t)hCpsw->hStats);
    }

    status = EnetCpdma_ackMiscIsr(hEnetDma);
    /* TODO: Add ISR safe error:
     * failed to handle Miscellaneous intr: %d\r\n", status); */
    ENET_UNUSED(status);
}

static void Cpsw_handleMdioLinkStateChange(EnetMdio_Group group,
                                           Mdio_PhyStatus *phyStatus,
                                           void *cbArgs)
{
    Cpsw_Handle hCpsw = (Cpsw_Handle)cbArgs;
    Cpsw_MdioLinkIntCtx *linkIntCtx = &hCpsw->mdioLinkIntCtx;
    const uint32_t aliveMaskChange = phyStatus->aliveMask ^ linkIntCtx->aliveMask;
    const uint32_t linkedMaskChange = phyStatus->linkedMask ^ linkIntCtx->linkedMask;


    for (uint32_t i = 0U; i <= MDIO_MAX_PHY_CNT; i++)
    {
        Cpsw_MdioLinkStateChangeInfo info;
        info.aliveChanged = ENET_IS_BIT_SET(aliveMaskChange, i);
        info.linkChanged  = ENET_IS_BIT_SET(linkedMaskChange, i);

        if (info.aliveChanged || info.linkChanged)
        {
            info.phyAddr  = i;
            info.isAlive  = ENET_IS_BIT_SET(phyStatus->aliveMask, i);
            info.isLinked = ENET_IS_BIT_SET(phyStatus->linkedMask, i);

            /* Re-enable periodic tick when link down is detected */
            if (info.linkChanged && !info.isLinked)
            {
                Cpsw_PortLinkState *pPortLinkState = Cpsw_getPortLinkState(hCpsw, i);
                if (pPortLinkState != NULL)
                {
                    pPortLinkState->isTickEnabled = true;
                }
            }

            if (linkIntCtx->linkStateChangeCb != NULL)
            {
                linkIntCtx->linkStateChangeCb(&info, linkIntCtx->linkStateChangeCbArg);
            }
        }
    }

    linkIntCtx->aliveMask  = phyStatus->aliveMask;
    linkIntCtx->linkedMask = phyStatus->linkedMask;

}

static Cpsw_PortLinkState *Cpsw_getPortLinkState(Cpsw_Handle hCpsw,
                                                 uint32_t phyAddr)
{
    Cpsw_PortLinkState *portLinkState = NULL;
    uint32_t i;

    for (i = 0U; i < CPSW_MAC_PORT_NUM; i++)
    {
        if (hCpsw->portLinkState[i].isOpen &&
            (phyAddr == hCpsw->portLinkState[i].phyAddr))
        {
            portLinkState = &hCpsw->portLinkState[i];
            break;
        }
    }

    return portLinkState;
}



static int32_t Cpsw_handleLinkUp(Cpsw_Handle hCpsw,
                                 Enet_MacPort macPort)
{

    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    EnetMod_Handle hMacPort = hCpsw->hMacPort[portNum];
    EnetPhy_Handle hPhy = hCpsw->hPhy[portNum];
    Enet_IoctlPrms prms;
    CpswAle_SetPortStateInArgs setPortStateInArgs;
    EnetPhy_LinkCfg phyLinkCfg;
    EnetMacPort_LinkCfg macLinkCfg;
    int32_t status;

    /* Assert if port number is not correct */
    Enet_assert(portNum < EnetSoc_getMacPortMax(hCpsw->enetPer.enetType, hCpsw->enetPer.instId),
                "Invalid Port Id: %u\r\n", portNum);

    ENETTRACE_VAR(portId);

    /* Get link parameters (speed/duplexity) from PHY state machine */
    status = EnetPhy_getLinkCfg(hPhy, &phyLinkCfg);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Port %u: Failed to get PHY link config: %d\r\n", portId, status);

    /* Enable MAC port */
    if (status == ENET_SOK)
    {
        macLinkCfg.speed = (Enet_Speed)phyLinkCfg.speed;
        macLinkCfg.duplexity = (Enet_Duplexity)phyLinkCfg.duplexity;
        ENET_IOCTL_SET_IN_ARGS(&prms, &macLinkCfg);

        CPSW_MACPORT_PRIV_IOCTL(hMacPort, CPSW_MACPORT_IOCTL_ENABLE, &prms, status);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Port %u: Failed to enable MAC: %d\r\n", portId, status);
    }

    /* Set ALE port state to 'Forward' */
    if (status == ENET_SOK)
    {
        setPortStateInArgs.portNum   = CPSW_ALE_MACPORT_TO_ALEPORT(portNum);
        setPortStateInArgs.portState = CPSW_ALE_PORTSTATE_FORWARD;
        ENET_IOCTL_SET_IN_ARGS(&prms, &setPortStateInArgs);

        CPSW_ALE_PRIV_IOCTL(hCpsw->hAle, CPSW_ALE_IOCTL_SET_PORT_STATE, &prms, status);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "Port %u: Failed to set ALE port %u to forward state: %d\r\n",
                         portId, setPortStateInArgs.portNum, status);
    }

    ENETTRACE_INFO_IF(status == ENET_SOK,
                      "Port %d: Link up: %s %s\r\n",
                      portId,
                      Cpsw_gSpeedNames[phyLinkCfg.speed],
                      Cpsw_gDuplexNames[phyLinkCfg.duplexity]);

    return status;
}

int32_t Cpsw_handleLinkDown(Cpsw_Handle hCpsw,
                            Enet_MacPort macPort)
{

    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    EnetMod_Handle hMacPort = hCpsw->hMacPort[portNum];
    Enet_IoctlPrms prms;
    CpswAle_SetPortStateInArgs setPortStateInArgs;
#if ENET_CFG_IS_ON(CPSW_EST)
    EnetTas_SetStateInArgs estSetStateInArgs;
#endif
    uint32_t alePortNum;
    uint32_t numEntries = 0U;
    int32_t status;

    /* Assert if port number is not correct */
    Enet_assert(portNum < EnetSoc_getMacPortMax(hCpsw->enetPer.enetType, hCpsw->enetPer.instId),
                "Invalid Port Id: %u\r\n", portNum);

    ENETTRACE_VAR(portId);
    ENETTRACE_INFO("Port %d: Link down\r\n", portId);

    ENET_IOCTL_SET_NO_ARGS(&prms);
    CPSW_MACPORT_PRIV_IOCTL(hMacPort, CPSW_MACPORT_IOCTL_DISABLE, &prms, status);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "Port %u: Failed to disable MAC port: %d\r\n", portId, status);

    if (status == ENET_SOK)
    {
        setPortStateInArgs.portNum   = CPSW_ALE_MACPORT_TO_ALEPORT(portNum);
        setPortStateInArgs.portState = CPSW_ALE_PORTSTATE_DISABLED;
        ENET_IOCTL_SET_IN_ARGS(&prms, &setPortStateInArgs);

        CPSW_ALE_PRIV_IOCTL(hCpsw->hAle, CPSW_ALE_IOCTL_SET_PORT_STATE, &prms, status);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "Port %u: Failed to set ALE port %u to disabled state: %d\r\n",
                         portId, setPortStateInArgs.portNum, status);
    }

    if (status == ENET_SOK)
    {
        alePortNum = CPSW_ALE_MACPORT_TO_ALEPORT(portNum);
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &alePortNum, &numEntries);

        CPSW_ALE_PRIV_IOCTL(hCpsw->hAle, CPSW_ALE_IOCTL_REMOVE_LEARNED_ENTRIES, &prms, status);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "Port %u: Failed to delete learned ALE entries: %d\r\n", portId, status);
    }

#if ENET_CFG_IS_ON(CPSW_EST)
    if (status == ENET_SOK)
    {
        EnetPer_Handle hPer = (EnetPer_Handle) hCpsw;
        estSetStateInArgs.macPort = macPort;
        estSetStateInArgs.state   = ENET_TAS_RESET;
        ENET_IOCTL_SET_IN_ARGS(&prms, &estSetStateInArgs);

        CPSW_EST_PRIV_IOCTL(hPer, ENET_TAS_IOCTL_SET_STATE, &prms, status);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "Port %u: Failed to disable EST: %d\r\n", portId, status);
    }
#endif

    return status;
}

static int32_t Cpsw_getIoctlHandlerIdx(uint32_t ioctlCmd, CpswIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx)
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

static CpswIoctlHandler * Cpsw_getIoctlHandlerFxn(uint32_t ioctlCmd, CpswIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;
    CpswIoctlHandler *handlerFxn = NULL;

    status = Cpsw_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert(tblIdx < tableSize, "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        handlerFxn = ioctlRegistryTbl[tblIdx].fxn;
    }
    else
    {
        handlerFxn = &Cpsw_internalIoctl_handler_default;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get ioctl handler for ioctl cmd: %x. Using default ioctl handler \r\n", ioctlCmd);
    return handlerFxn;
}

static int32_t Cpsw_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                       CpswIoctlHandler *ioctlHandlerFxn,
                                       CpswIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                       uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;

    status = Cpsw_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert((tblIdx < tableSize), "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        Enet_devAssert((ioctlCmd == ioctlRegistryTbl[tblIdx].cmd), "Ioctl table corrupted");
        ioctlRegistryTbl[tblIdx].fxn = ioctlHandlerFxn;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set ioctl handler for ioctl cmd: %x \r\n", ioctlCmd);
    return status;
}

static int32_t Cpsw_registerInternalIoctlHandler(EnetPer_Handle hPer, Enet_IoctlPrms *prms)
{
    const Enet_IoctlRegisterHandlerInArgs *inArgs = (const Enet_IoctlRegisterHandlerInArgs *)prms->inArgs;
    int32_t status;

    status = Cpsw_setIoctlHandlerFxn(inArgs->cmd, 
                                     (CpswIoctlHandler *)inArgs->fxn, 
                                      CpswIoctlHandlerRegistry, 
                                      ENET_ARRAYSIZE(CpswIoctlHandlerRegistry));
    return status;
}

static int32_t Cpsw_registerIoctlHandler(EnetPer_Handle hPer,
                                         Enet_IoctlPrms *prms)
{
    uint32_t major;
    int32_t status = ENET_SOK;
    Enet_IoctlRegisterHandlerInArgs *ioctlHandlerRegister = (Enet_IoctlRegisterHandlerInArgs *)prms->inArgs;
    Cpsw_Handle hCpsw = (Cpsw_Handle)hPer;

    major = ENET_IOCTL_GET_MAJ(ioctlHandlerRegister->cmd);
    switch (major)
    {
        case ENET_IOCTL_PER_BASE:
        {
            status = Cpsw_registerInternalIoctlHandler(hPer, prms);
        }
        break;

        case ENET_IOCTL_FDB_BASE:
        {
            status = EnetMod_ioctl(hCpsw->hAle, CPSW_ALE_IOCTL_REGISTER_HANDLER, prms);
        }
        break;

        case ENET_IOCTL_TIMESYNC_BASE:
        {
            status = EnetMod_ioctl(hCpsw->hCpts, CPSW_CPTS_IOCTL_REGISTER_HANDLER, prms);
        }
        break;

        case ENET_IOCTL_HOSTPORT_BASE:
        {
            status = EnetMod_ioctl(hCpsw->hHostPort, CPSW_HOSTPORT_IOCTL_REGISTER_HANDLER, prms);
        }
        break;

        case ENET_IOCTL_TAS_BASE:
        {
#if ENET_CFG_IS_ON(CPSW_EST)
#if ENET_CFG_IS_OFF(CPSW_MACPORT_EST)
#error "CPSW EST feature requires ENET_CFG_CPSW_MACPORT_EST"
#endif
            if (ENET_FEAT_IS_EN(hCpsw->enetPer.features, CPSW_FEATURE_EST))
            {
                status = Cpsw_ioctlEst(hPer, CPSW_EST_IOCTL_REGISTER_HANDLER, prms);
            }
            else
            {
                status = ENET_ENOTSUPPORTED;
            }
#else
            status = ENET_ENOTSUPPORTED;
#endif
        }
        break;

        case ENET_IOCTL_MACPORT_BASE:
        {
            /* Register Handler is not instance based, pass first macport as mac port
             * instance does not matter. Register Handler updates a table which is
             * per module not per instance
            */
            Enet_MacPort macPort = ENET_MAC_PORT_FIRST;
            uint32_t portNum = ENET_MACPORT_NORM(macPort);

            if (portNum < EnetSoc_getMacPortMax(hCpsw->enetPer.enetType, hCpsw->enetPer.instId))
            {
                status = EnetMod_registerMacportIoctlHandler(hCpsw->hMacPort[portNum],
                                                             ENET_IOCTL_MACPORT_BASE,
                                                             CPSW_MACPORT_IOCTL_REGISTER_HANDLER,
                                                             prms);
            }
            else
            {
                status = ENET_EINVALIDPARAMS;
            }
        }
        break;

        case ENET_IOCTL_MDIO_BASE:
        {
            status = EnetMod_ioctl(hCpsw->hMdio, MDIO_IOCTL_REGISTER_HANDLER, prms);
        }
        break;

        case ENET_IOCTL_STATS_BASE:
        {
            status = EnetMod_ioctl(hCpsw->hStats, CPSW_STATS_IOCTL_REGISTER_HANDLER, prms);
        }
        break;

        case ENET_IOCTL_PHY_BASE:
        {
            /* Register Handler is not instance based, pass first macport as mac port
             * instance does not matter. Register Handler updates a table which is
             * per module not per instance
            */
            Enet_MacPort macPort = ENET_MAC_PORT_FIRST;
            uint32_t portNum = ENET_MACPORT_NORM(macPort);

            /* Assert if port number is not correct */
            Enet_assert(portNum < EnetSoc_getMacPortMax(hCpsw->enetPer.enetType, hCpsw->enetPer.instId),
                        "Invalid Port Id: %u\r\n", portNum);

            EnetPhy_Handle hPhy = hCpsw->hPhy[portNum];

            if (hPhy != NULL)
            {
                status = EnetPhyMdioDflt_ioctl(hCpsw->hPhy[portNum], ENET_PHY_IOCTL_REGISTER_HANDLER, prms);
            }
            else
            {
                status = ENET_EFAIL;
            }
        }
        break;

        case ENET_IOCTL_RM_BASE:
        {
            status = EnetMod_ioctl(hCpsw->hRm, ENET_RM_IOCTL_REGISTER_HANDLER, prms);
        }
        break;

        default:
        {
            status = ENET_EINVALIDPARAMS;
        }
        break;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to register IOCTL handler: %d, %x, %p\r\n", 
                     status, 
                     ioctlHandlerRegister->cmd, 
                     ioctlHandlerRegister->fxn);
    return status;
}

static int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_REGISTER_IOCTL_HANDLER(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status;

    status = Cpsw_registerIoctlHandler(&hCpsw->enetPer, prms);
    return status;
}

int32_t Cpsw_internalIoctl_handler_default(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status;
    
    status = ENET_ENOTSUPPORTED;
    return status;
}



