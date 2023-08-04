/*
 *  Copyright (c) Texas Instruments Incorporated 2022-23
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
 * \file  mdio.c
 *
 * \brief This file contains the implementation of the MDIO module.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>
#include <csl_cpswitch.h>
#include <enet_cfg.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_soc.h>
#include <include/mod/mdio.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/mod/mdio_priv.h>
#include <priv/mod/mdio_ioctl_priv.h>
#include <priv/mod/cpsw_clks.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*! \brief Supported MDIO versions. */
#define MDIO_VER_MODID_J7X                    (0x00000007U)
#define MDIO_VER_REVMAJ_J7X                   (0x00000001U)
#define MDIO_VER_REVMIN_J7X                   (0x00000007U)
#define MDIO_VER_REVRTL_J7X                   (0x00000001U)

/*! \brief Supported ICSSG MDIO versions. */
#define ICSSG_MDIO_VER_MODID_J7X              (0x00000007U)
#define ICSSG_MDIO_VER_REVMAJ_J7X             (0x00000001U)
#define ICSSG_MDIO_VER_REVMIN_J7X             (0x00000007U)
#define ICSSG_MDIO_VER_REVRTL_J7X             (0x00000000U)

/*! \brief AWR2544 MDIO versions. */
#define MDIO_VER_MODID_AWR2544                (0x00000007U)
#define MDIO_VER_REVMAJ_AWR2544               (0x00000001U)
#define MDIO_VER_REVMIN_AWR2544               (0x00000007U)
#define MDIO_VER_REVRTL_AWR2544               (0x00000002U)

/*! \brief Default MDIO bus frequency. */
#define MDIO_MDIOBUS_DFLT_FREQ_HZ             (2200000U)

#define MDIO_IOCTL_HANDLER_ENTRY_INIT(x)            \
          {.cmd = x,                                \
           .fxn = &Mdio_ioctl_handler_##x}

#define MDIO_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(x)    \
          {.cmd = x,                                \
           .fxn = &Mdio_ioctl_handler_default}

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef int32_t (MdioIoctlHandler)(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);

typedef struct MdioIoctlHandlerRegistry_s
{
    uint32_t cmd;
    MdioIoctlHandler *fxn;
} MdioIoctlHandlerRegistry_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

#if ENET_CFG_IS_ON(DEV_ERROR)
static int32_t Mdio_isSupported(CSL_mdioHandle mdioRegs);
#endif

static int32_t Mdio_setupNormalMode(CSL_mdioHandle mdioRegs,
                                    const Mdio_Cfg *cfg);

static void Mdio_setupStatusChangeMode(CSL_mdioHandle mdioRegs,
                                       const Mdio_Cfg *cfg);
static int32_t Mdio_ioctl_handler_default(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
static int32_t Mdio_ioctl_handler_MDIO_IOCTL_REGISTER_HANDLER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
static int32_t Mdio_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                       MdioIoctlHandler *ioctlHandlerFxn,
                                       MdioIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                       uint32_t tableSize);
static MdioIoctlHandler * Mdio_getIoctlHandlerFxn(uint32_t ioctlCmd, MdioIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize);
static int32_t Mdio_getIoctlHandlerIdx(uint32_t ioctlCmd, MdioIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx);
static void Mdio_setupManualMode(CSL_mdioHandle mdioRegs);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#if ENET_CFG_IS_ON(DEV_ERROR)
/*! \brief MDIO versions supported by this driver. */
static CSL_MDIO_VERSION gMdio_supportedVer[] =
{
    {   /* AM65xx and J7x devices */
        .modId  = MDIO_VER_MODID_J7X,
        .revMaj = MDIO_VER_REVMAJ_J7X,
        .revMin = MDIO_VER_REVMIN_J7X,
        .revRtl = MDIO_VER_REVRTL_J7X,
    },
    {   /* ICSSG MDIO on AM65xx and J7x devices */
        .modId  = ICSSG_MDIO_VER_MODID_J7X,
        .revMaj = ICSSG_MDIO_VER_REVMAJ_J7X,
        .revMin = ICSSG_MDIO_VER_REVMIN_J7X,
        .revRtl = ICSSG_MDIO_VER_REVRTL_J7X,
    },
    {   /* MDIO on AWR2544 devices */
        .modId  = MDIO_VER_MODID_AWR2544,
        .revMaj = MDIO_VER_REVMAJ_AWR2544,
        .revMin = MDIO_VER_REVMIN_AWR2544,
        .revRtl = MDIO_VER_REVRTL_AWR2544,
    },
};

/* Private MDIO IOCTL validation data. */
static Enet_IoctlValidate gMdio_privIoctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(MDIO_IOCTL_HANDLE_INTR,
                          sizeof(Mdio_Callbacks),
                          0U),

    ENET_IOCTL_VALID_PRMS(MDIO_IOCTL_REGISTER_HANDLER,
                          sizeof(Enet_IoctlRegisterHandlerInArgs),
                          0U),
};
#endif


static MdioIoctlHandlerRegistry_t MdioIoctlHandlerRegistry[] =
{
    MDIO_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MDIO_IOCTL_GET_VERSION),
    MDIO_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MDIO_IOCTL_PRINT_REGS),
    MDIO_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MDIO_IOCTL_IS_ALIVE),
    MDIO_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MDIO_IOCTL_IS_LINKED),
    MDIO_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MDIO_IOCTL_IS_POLL_ENABLED),
    MDIO_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MDIO_IOCTL_C22_READ),
    MDIO_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MDIO_IOCTL_C22_WRITE),
    MDIO_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MDIO_IOCTL_C45_READ),
    MDIO_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MDIO_IOCTL_C45_WRITE),
    MDIO_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MDIO_IOCTL_C22_ASYNC_READ_TRIGGER),
    MDIO_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MDIO_IOCTL_C22_ASYNC_READ_COMPLETE),
    MDIO_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MDIO_IOCTL_C22_ASYNC_WRITE_TRIGGER),
    MDIO_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MDIO_IOCTL_C22_ASYNC_WRITE_COMPLETE),
    MDIO_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MDIO_IOCTL_C45_ASYNC_READ_TRIGGER),
    MDIO_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MDIO_IOCTL_C45_ASYNC_READ_COMPLETE),
    MDIO_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MDIO_IOCTL_C45_ASYNC_WRITE_TRIGGER),
    MDIO_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MDIO_IOCTL_C45_ASYNC_WRITE_COMPLETE),
    MDIO_IOCTL_HANDLER_ENTRY_INIT(MDIO_IOCTL_HANDLE_INTR), /* Register ISR IOCTLs by default */
    MDIO_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_MDIO_IOCTL_ENABLE_STATE_MACHINE),
    MDIO_IOCTL_HANDLER_ENTRY_INIT(MDIO_IOCTL_REGISTER_HANDLER),
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Mdio_initCfg(Mdio_Cfg *mdioCfg)
{
    mdioCfg->mode               = MDIO_MODE_STATE_CHANGE_MON;
    mdioCfg->mdioBusFreqHz      = MDIO_MDIOBUS_DFLT_FREQ_HZ;
    mdioCfg->phyStatePollFreqHz = mdioCfg->mdioBusFreqHz;
    mdioCfg->pollEnMask         = ENET_MDIO_PHY_ADDR_MASK_ALL;
    mdioCfg->c45EnMask          = ENET_MDIO_PHY_ADDR_MASK_NONE;
    mdioCfg->isMaster           = true;
    mdioCfg->disableStateMachineOnInit = false;
}

int32_t Mdio_open(EnetMod_Handle hMod,
                  Enet_Type enetType,
                  uint32_t instId,
                  const void *cfg,
                  uint32_t cfgSize)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    const Mdio_Cfg *mdioCfg = (const Mdio_Cfg *)cfg;
    CSL_mdioHandle mdioRegs = (CSL_mdioHandle)hMod->virtAddr;
    uint32_t cppiClkFreqHz;
    uint32_t clkdiv;
    uint32_t ipgRatio;
    int32_t status = ENET_SOK;

    Enet_devAssert(cfgSize == sizeof(Mdio_Cfg),
                   "Invalid MDIO config params size %u (expected %u)\n",
                   cfgSize, sizeof(Mdio_Cfg));

    Enet_devAssert(mdioRegs != NULL, "MDIO reg address is not valid\n");

    /* Check supported MDIO module versions */
#if ENET_CFG_IS_ON(DEV_ERROR)
    status = Mdio_isSupported(mdioRegs);
    Enet_devAssert(status == ENET_SOK, "MDIO version is not supported\n");
#endif

    hMdio->isMaster = mdioCfg->isMaster;
    hMdio->mode     = mdioCfg->mode;

    if (mdioCfg->c45EnMask != ENET_MDIO_PHY_ADDR_MASK_NONE)
    {
        status = ENET_ENOTSUPPORTED;
        ENETTRACE_WARN("MDIO Clause 45 is not supported\n");
    }

    /* Perform global MDIO configuration only for module in master role */
    if (hMdio->isMaster)
    {
        /* Compute MDIO clock divider */
        if (status == ENET_SOK)
        {
            if (mdioCfg->mdioBusFreqHz != 0U)
            {
                cppiClkFreqHz = EnetSoc_getClkFreq(enetType, instId, CPSW_CPPI_CLK);

                clkdiv = (cppiClkFreqHz / mdioCfg->mdioBusFreqHz) - 1U;
                if (clkdiv > 0xFFFFU)
                {
                    ENETTRACE_ERR("unsupported clk div %u (CPPI %u Hz, MDIO bus %u Hz)\n",
                                  clkdiv, cppiClkFreqHz, mdioCfg->mdioBusFreqHz);
                    status = ENET_EFAIL;
                }

                ipgRatio = mdioCfg->mdioBusFreqHz / mdioCfg->phyStatePollFreqHz;
                if (ipgRatio > 0xFFU)
                {
                    ENETTRACE_ERR("unsupported IPG ratio %u (MDIO bus %u Hz, PHY poll %u Hz)\n",
                                  ipgRatio, mdioCfg->mdioBusFreqHz, mdioCfg->phyStatePollFreqHz);
                    status = ENET_EFAIL;
                }
            }
            else
            {
                ENETTRACE_ERR("invalid MDIO bus freq %u Hz\n", mdioCfg->mdioBusFreqHz);
                status = ENET_EINVALIDPARAMS;
            }
        }

        /* Setup MDIO mode: normal, monitor or manual mode */
        if (status == ENET_SOK)
        {
            CSL_MDIO_clearUnmaskedLinkStatusChangeInt(mdioRegs, 0U);
            CSL_MDIO_clearUnmaskedLinkStatusChangeInt(mdioRegs, 1U);

            switch (mdioCfg->mode)
            {
                case MDIO_MODE_NORMAL:
                {
                    status = Mdio_setupNormalMode(mdioRegs, mdioCfg);
                }
                break;

                case MDIO_MODE_STATE_CHANGE_MON:
                {
                    Mdio_setupStatusChangeMode(mdioRegs, mdioCfg);
                }
                break;

                case MDIO_MODE_MANUAL:
                {
                    ENETTRACE_INFO("MDIO Manual_Mode enabled\n");
                    hMdio->mdcHalfCycleNs = (ENET_NUM_NANOSECS_PER_SEC >> 1) / mdioCfg->mdioBusFreqHz; /* HalfTime = 1/(2*Freq). Division by 2, as it is halfCycle */;
                    Mdio_setupManualMode(mdioRegs);
                }
                break;
            }
        }

        /* Write MDIO configuration */
        if ((status == ENET_SOK) &&
            (mdioCfg->mode != MDIO_MODE_MANUAL))
        {
            CSL_MDIO_setPollIPG(mdioRegs, (uint8_t)ipgRatio);
            CSL_MDIO_setClkDivVal(mdioRegs, (uint16_t)clkdiv);
            if (mdioCfg->disableStateMachineOnInit != true)
            {
                CSL_MDIO_enableStateMachine(mdioRegs);
            }
#if ENET_CFG_IS_ON(MDIO_CLAUSE45)
            if (ENET_FEAT_IS_EN(hMod->features, MDIO_FEATURE_CLAUSE45))
            {
                CSL_MDIO_setClause45EnableMask(mdioRegs, mdioCfg->c45EnMask);
            }
#endif
        }
    }

    return status;
}

int32_t Mdio_rejoin(EnetMod_Handle hMod,
                    Enet_Type enetType,
                    uint32_t instId)
{
    return ENET_SOK;
}

void Mdio_close(EnetMod_Handle hMod)
{
    Mdio_Handle hMdio = (Mdio_Handle)hMod;
    CSL_mdioHandle mdioRegs = (CSL_mdioHandle)hMod->virtAddr;
    uint32_t i;

    Enet_devAssert(mdioRegs != NULL, "MDIO reg address is not valid\n");

    if (hMdio->isMaster)
    {
        for (i = 0U; i < MDIO_PHY_MONITOR_MAX; i++)
        {
            CSL_MDIO_disableLinkStatusChangeInterrupt(mdioRegs, i, 0U);
        }

        CSL_MDIO_disableStatusChangeModeInterrupt(mdioRegs);
        CSL_MDIO_disableStateMachine(mdioRegs);
        CSL_MDIO_clearUnmaskedLinkStatusChangeInt(mdioRegs, 0U);
        CSL_MDIO_clearUnmaskedLinkStatusChangeInt(mdioRegs, 1U);
    }
}

void Mdio_saveCtxt(EnetMod_Handle hMod)
{
    Mdio_close(hMod);
}

int32_t Mdio_restoreCtxt(EnetMod_Handle hMod,
                         Enet_Type enetType,
                         uint32_t instId,
                         const void *cfg,
                         uint32_t cfgSize)
{
    int32_t status = ENET_SOK;

    status = Mdio_open(hMod, enetType, instId, cfg, cfgSize);

    return status;
}

int32_t Mdio_ioctl(EnetMod_Handle hMod,
                   uint32_t cmd,
                   Enet_IoctlPrms *prms)
{
    CSL_mdioHandle mdioRegs = (CSL_mdioHandle)hMod->virtAddr;
    int32_t status = ENET_SOK;

#if ENET_CFG_IS_ON(DEV_ERROR)
    /* Validate MDIO IOCTL parameters */
    if (ENET_IOCTL_GET_PER(cmd) == ENET_IOCTL_PER_CPSW)
    {
        if (ENET_IOCTL_GET_TYPE(cmd) == ENET_IOCTL_TYPE_PRIVATE)
        {
            status = Enet_validateIoctl(cmd, prms,
                                        gMdio_privIoctlValidate,
                                        ENET_ARRAYSIZE(gMdio_privIoctlValidate));

            ENETTRACE_ERR_IF(status != ENET_SOK, "IOCTL 0x%08x params are not valid\n", cmd);
        }
    }
#endif

    if (status == ENET_SOK)
    {
        MdioIoctlHandler * ioctlHandlerFxn;

        Enet_devAssert(mdioRegs != NULL, "MDIO reg address is not valid\n");

        ioctlHandlerFxn = Mdio_getIoctlHandlerFxn(cmd, MdioIoctlHandlerRegistry, ENET_ARRAYSIZE(MdioIoctlHandlerRegistry));
        Enet_devAssert(ioctlHandlerFxn != NULL);
        status = ioctlHandlerFxn(hMod, mdioRegs, prms);
    }

    return status;
}

#if ENET_CFG_IS_ON(DEV_ERROR)
static int32_t Mdio_isSupported(CSL_mdioHandle mdioRegs)
{
    CSL_MDIO_VERSION version;
    uint32_t i;
    int32_t status = ENET_ENOTSUPPORTED;

    CSL_MDIO_getVersionInfo(mdioRegs, &version);

    for (i = 0U; i < ENET_ARRAYSIZE(gMdio_supportedVer); i++)
    {
        if ((version.revMaj == gMdio_supportedVer[i].revMaj) &&
            (version.revMin == gMdio_supportedVer[i].revMin) &&
            (version.revRtl == gMdio_supportedVer[i].revRtl) &&
            (version.modId  == gMdio_supportedVer[i].modId))
        {
            status = ENET_SOK;
            break;
        }
    }

    return status;
}
#endif

static int32_t Mdio_setupNormalMode(CSL_mdioHandle mdioRegs,
                                    const Mdio_Cfg *cfg)
{
    uint32_t phyMonIndex = 0U;
    uint32_t i;
    int32_t status = ENET_SOK;

    for (i = 0U; i <= MDIO_MAX_PHY_CNT; i++)
    {
        if (ENET_IS_BIT_SET(cfg->pollEnMask, i))
        {
            if (phyMonIndex < MDIO_PHY_MONITOR_MAX)
            {
                CSL_MDIO_enableLinkStatusChangeInterrupt(mdioRegs, phyMonIndex, i);
            }

            phyMonIndex++;
        }
    }

    if (phyMonIndex <= MDIO_PHY_MONITOR_MAX)
    {
        /* Normal Mode implies State Change Mode is disabled */
        CSL_MDIO_disableStateChangeMode(mdioRegs);
    }
    else
    {
        ENETTRACE_ERR("invalid PHY monitor count %d\n", phyMonIndex);
        for (i = 0U; i < MDIO_PHY_MONITOR_MAX; i++)
        {
            CSL_MDIO_disableLinkStatusChangeInterrupt(mdioRegs, i, 0U);
        }

        CSL_MDIO_clearUnmaskedLinkStatusChangeInt(mdioRegs, 0U);
        CSL_MDIO_clearUnmaskedLinkStatusChangeInt(mdioRegs, 1U);
        status = ENET_EINVALIDPARAMS;
    }

    return status;
}

static void Mdio_setupStatusChangeMode(CSL_mdioHandle mdioRegs,
                                       const Mdio_Cfg *cfg)
{
    CSL_MDIO_setPollEnableMask(mdioRegs, cfg->pollEnMask);
    CSL_MDIO_enableStatusChangeModeInterrupt(mdioRegs);
    CSL_MDIO_enableStateChangeMode(mdioRegs);
}

static void Mdio_setupManualMode(CSL_mdioHandle mdioRegs)
{
    CSL_MDIO_clearPollEnableMask(mdioRegs);
    CSL_MDIO_enableManualMode(mdioRegs);
}

static int32_t Mdio_getIoctlHandlerIdx(uint32_t ioctlCmd, MdioIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx)
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

static MdioIoctlHandler * Mdio_getIoctlHandlerFxn(uint32_t ioctlCmd, MdioIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;
    MdioIoctlHandler *handlerFxn = NULL;

    status = Mdio_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert(tblIdx < tableSize, "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        handlerFxn = ioctlRegistryTbl[tblIdx].fxn;
    }
    else
    {
        handlerFxn = &Mdio_ioctl_handler_default;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get ioctl handler for ioctl cmd: %x. Using default ioctl handler \r\n", ioctlCmd);
    return handlerFxn;
}

static int32_t Mdio_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                       MdioIoctlHandler *ioctlHandlerFxn,
                                       MdioIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                       uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;

    status = Mdio_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert((tblIdx < tableSize), "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        Enet_devAssert((ioctlCmd == ioctlRegistryTbl[tblIdx].cmd), "Ioctl table corrupted");
        ioctlRegistryTbl[tblIdx].fxn = ioctlHandlerFxn;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set ioctl handler for ioctl cmd: %x \r\n", ioctlCmd);
    return status;
}

static int32_t Mdio_ioctl_handler_MDIO_IOCTL_REGISTER_HANDLER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    const Enet_IoctlRegisterHandlerInArgs *inArgs = (const Enet_IoctlRegisterHandlerInArgs *)prms->inArgs;
    int32_t status;

    status = Mdio_setIoctlHandlerFxn(inArgs->cmd,
                                     (MdioIoctlHandler *)inArgs->fxn,
                                     MdioIoctlHandlerRegistry,
                                     ENET_ARRAYSIZE(MdioIoctlHandlerRegistry));
    return status;
}


static int32_t Mdio_ioctl_handler_default(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms)
{
    return ENET_ENOTSUPPORTED;
}

