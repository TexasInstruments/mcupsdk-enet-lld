/*
 *  Copyright (c) Texas Instruments Incorporated 2020-23
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
 * \file  cpsw_cpts.c
 *
 * \brief This file contains the implementation of the CPSW CPTS module.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>
#include <csl_cpswitch.h>
#include <enet_cfg.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_osal.h>
#include <include/core/enet_soc.h>
#include <include/mod/cpsw_cpts.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/mod/cpsw_cpts_priv.h>
#include <priv/mod/cpsw_cpts_ioctl_priv.h>
#include <priv/mod/cpsw_clks.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Supported AM273x versions */
#define CPSW_CPTS_VER_REVMAJ_AM273X           (0x00000001U)
#define CPSW_CPTS_VER_REVMIN_AM273X           (0x0000000BU)
#define CPSW_CPTS_VER_REVRTL_AM273X           (0x00000000U)
#define CPSW_HOSTPORT_VER_ID_AM273X           (0x00004E8AU)

/* Supported AM263x versions */
#define CPSW_CPTS_VER_REVMAJ_AM263X           (0x00000001U)
#define CPSW_CPTS_VER_REVMIN_AM263X           (0x0000000BU)
#define CPSW_CPTS_VER_REVRTL_AM263X           (0x00000000U)
#define CPSW_HOSTPORT_VER_ID_AM263X           (0x00004E8AU)

/* Supported AM64x versions */
#define CPSW_CPTS_VER_REVMAJ_AM64X            (0x00000001U)
#define CPSW_CPTS_VER_REVMIN_AM64X            (0x0000000CU)
#define CPSW_CPTS_VER_REVRTL_AM64X            (0x00000000U)
#define CPSW_HOSTPORT_VER_ID_AM64X            (0x00004E8AU)

/* Supported AWR2544 versions */
#define CPSW_CPTS_VER_REVMAJ_AWR2544          (0x00000001U)
#define CPSW_CPTS_VER_REVMIN_AWR2544          (0x0000000DU)
#define CPSW_CPTS_VER_REVRTL_AWR2544          (0x00000000U)
#define CPSW_HOSTPORT_VER_ID_AWR2544          (0x00004E8AU)

/* Supported AM62PX versions */
#define CPSW_CPTS_VER_REVMAJ_AM62PX          (0x00000001U)
#define CPSW_CPTS_VER_REVMIN_AM62PX          (0x0000000DU)
#define CPSW_CPTS_VER_REVRTL_AM62PX          (0x00000000U)
#define CPSW_HOSTPORT_VER_ID_AM62PX          (0x00004E8AU)

/* Supported AM62X versions */
#define CPSW_CPTS_VER_REVMAJ_AM62X          (0x00000001U)
#define CPSW_CPTS_VER_REVMIN_AM62X          (0x0000000DU)
#define CPSW_CPTS_VER_REVRTL_AM62X          (0x00000000U)
#define CPSW_HOSTPORT_VER_ID_AM62X          (0x00004E8AU)

/* Supported AM261X versions */
#define CPSW_CPTS_VER_REVMAJ_AM261X          (0x00000001U)
#define CPSW_CPTS_VER_REVMIN_AM261X          (0x0000000CU)
#define CPSW_CPTS_VER_REVRTL_AM261X          (0x00000001U)
#define CPSW_HOSTPORT_VER_ID_AM261X          (0x00004E8AU)

#define CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT(x)    \
          {.cmd = x,                            \
           .fxn = &CpswCpts_ioctl_handler_##x}

#define CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(x)    \
          {.cmd = x,                            \
           .fxn = &CpswCpts_ioctl_handler_default}

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef int32_t (CpswCptsIoctlHandler)(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);

typedef struct CpswCptsIoctlHandlerRegistry_s
{
    uint32_t cmd;
    CpswCptsIoctlHandler *fxn;
} CpswCptsIoctlHandlerRegistry_t;


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

#if ENET_CFG_IS_ON(DEV_ERROR)
static int32_t CpswCpts_isSupported(CSL_cptsRegs *regs);
#endif


static int32_t CpswCpts_ioctl_handler_default(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
static int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_REGISTER_HANDLER(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms);
static int32_t CpswCpts_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                          CpswCptsIoctlHandler *ioctlHandlerFxn,
                                          CpswCptsIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                          uint32_t tableSize);
static CpswCptsIoctlHandler * CpswCpts_getIoctlHandlerFxn(uint32_t ioctlCmd, CpswCptsIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize);
static int32_t CpswCpts_getIoctlHandlerIdx(uint32_t ioctlCmd, CpswCptsIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#if ENET_CFG_IS_ON(DEV_ERROR)
/*! \brief CPTS versions supported by this driver. */
static CSL_CPSW_VERSION CpswCpts_gSupportedVer[] =
{
    {   /* AM273X CPSW_2G */
        .majorVer = CPSW_CPTS_VER_REVMAJ_AM273X,
        .minorVer = CPSW_CPTS_VER_REVMIN_AM273X,
        .rtlVer   = CPSW_CPTS_VER_REVRTL_AM273X,
        .id       = CPSW_HOSTPORT_VER_ID_AM273X,
    },
	{   /* AM263X CPSW_3G */
        .majorVer = CPSW_CPTS_VER_REVMAJ_AM263X,
        .minorVer = CPSW_CPTS_VER_REVMIN_AM263X,
        .rtlVer   = CPSW_CPTS_VER_REVRTL_AM263X,
        .id       = CPSW_HOSTPORT_VER_ID_AM263X,
    },
    {   /* AM64X CPSW_3G */
        .majorVer = CPSW_CPTS_VER_REVMAJ_AM64X,
        .minorVer = CPSW_CPTS_VER_REVMIN_AM64X,
        .rtlVer   = CPSW_CPTS_VER_REVRTL_AM64X,
        .id       = CPSW_HOSTPORT_VER_ID_AM64X,
    },
    {   /* AWR2544 CPSW_2G */
        .majorVer = CPSW_CPTS_VER_REVMAJ_AWR2544,
        .minorVer = CPSW_CPTS_VER_REVMIN_AWR2544,
        .rtlVer   = CPSW_CPTS_VER_REVRTL_AWR2544,
        .id       = CPSW_HOSTPORT_VER_ID_AWR2544,
    },
    {   /* AM62PX CPSW_2G */
        .majorVer = CPSW_CPTS_VER_REVMAJ_AM62PX,
        .minorVer = CPSW_CPTS_VER_REVMIN_AM62PX,
        .rtlVer   = CPSW_CPTS_VER_REVRTL_AM62PX,
        .id       = CPSW_HOSTPORT_VER_ID_AM62PX,
    },
    {   /* AM62X CPSW_2G */
        .majorVer = CPSW_CPTS_VER_REVMAJ_AM62X,
        .minorVer = CPSW_CPTS_VER_REVMIN_AM62X,
        .rtlVer   = CPSW_CPTS_VER_REVRTL_AM62X,
        .id       = CPSW_HOSTPORT_VER_ID_AM62X,
    },
    {   /* AM261X CPSW_3G */
        .majorVer = CPSW_CPTS_VER_REVMAJ_AM261X,
        .minorVer = CPSW_CPTS_VER_REVMIN_AM261X,
        .rtlVer   = CPSW_CPTS_VER_REVRTL_AM261X,
        .id       = CPSW_HOSTPORT_VER_ID_AM261X,
    },
};

/* Public CPTS IOCTL validation data. */
static Enet_IoctlValidate gCpswCpts_ioctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(CPSW_CPTS_IOCTL_REGISTER_STACK,
                          sizeof(CpswCpts_RegisterStackInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_CPTS_IOCTL_UNREGISTER_STACK,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_CPTS_IOCTL_REGISTER_HWPUSH_CALLBACK,
                          sizeof(CpswCpts_RegisterHwPushCbInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_CPTS_IOCTL_UNREGISTER_HWPUSH_CALLBACK,
                          sizeof(CpswCpts_HwPush),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_CPTS_IOCTL_SET_TS_NUDGE,
                          sizeof(int32_t),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_CPTS_IOCTL_SET_COMP,
                          sizeof(CpswCpts_SetCompValInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_CPTS_IOCTL_SET_COMP_NUDGE,
                          sizeof(int32_t),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_CPTS_IOCTL_SET_GENF,
                          sizeof(CpswCpts_SetFxnGenInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_CPTS_IOCTL_SET_GENF_NUDGE,
                          sizeof(CpswCpts_SetFxnGenNudgeInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_CPTS_IOCTL_SET_ESTF,
                          sizeof(CpswCpts_SetFxnGenInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_CPTS_IOCTL_SET_ESTF_NUDGE,
                          sizeof(CpswCpts_SetFxnGenNudgeInArgs),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_CPTS_IOCTL_SELECT_TS_OUTPUT_BIT,
                          sizeof(CpswCpts_OutputBitSel),
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_CPTS_IOCTL_LOOKUP_EVENT,
                          sizeof(CpswCpts_Event),
                          sizeof(CpswCpts_Event)),

    ENET_IOCTL_VALID_PRMS(CPSW_CPTS_IOCTL_LOOKUP_EST_EVENT,
                          sizeof(CpswCpts_EstEventMatchParams),
                          sizeof(CpswCpts_EstEvent)),
};

/* Private CPTS IOCTL validation data. */
static Enet_IoctlValidate gCpswCpts_privIoctlValidate[] =
{
    ENET_IOCTL_VALID_PRMS(CPSW_CPTS_IOCTL_HANDLE_INTR,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_CPTS_IOCTL_ENABLE_INTR,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_CPTS_IOCTL_DISABLE_INTR,
                          0U,
                          0U),

    ENET_IOCTL_VALID_PRMS(CPSW_CPTS_IOCTL_REGISTER_HANDLER,
                          sizeof(Enet_IoctlRegisterHandlerInArgs),
                          0U),

};
#endif

static CpswCptsIoctlHandlerRegistry_t CpswCptsIoctlHandlerRegistry[] =
{
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TIMESYNC_IOCTL_GET_VERSION),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TIMESYNC_IOCTL_PRINT_REGS),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TIMESYNC_IOCTL_PRINT_STATS),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TIMESYNC_IOCTL_SET_TIMESTAMP),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TIMESYNC_IOCTL_ADJUST_TIMESTAMP),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TIMESYNC_IOCTL_GET_ETH_RX_TIMESTAMP),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TIMESYNC_IOCTL_GET_ETH_TX_TIMESTAMP),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TIMESYNC_IOCTL_RESET),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_CPTS_IOCTL_REGISTER_STACK),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_CPTS_IOCTL_UNREGISTER_STACK),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_CPTS_IOCTL_REGISTER_HWPUSH_CALLBACK),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_CPTS_IOCTL_UNREGISTER_HWPUSH_CALLBACK),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_CPTS_IOCTL_SET_TS_NUDGE),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_CPTS_IOCTL_SET_COMP),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_CPTS_IOCTL_SET_COMP_NUDGE),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_CPTS_IOCTL_SET_GENF),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_CPTS_IOCTL_SET_GENF_NUDGE),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_CPTS_IOCTL_SET_ESTF),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_CPTS_IOCTL_SET_ESTF_NUDGE),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_CPTS_IOCTL_SELECT_TS_OUTPUT_BIT),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_CPTS_IOCTL_LOOKUP_EVENT),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_CPTS_IOCTL_LOOKUP_EST_EVENT),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT(CPSW_CPTS_IOCTL_HANDLE_INTR), /* Register ISR callback IOCTLS by default */
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_CPTS_IOCTL_ENABLE_INTR),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_CPTS_IOCTL_DISABLE_INTR),
    CPSW_CPTS_IOCTL_HANDLER_ENTRY_INIT(CPSW_CPTS_IOCTL_REGISTER_HANDLER),
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void CpswCpts_initCfg(CpswCpts_Cfg *cptsCfg)
{
    cptsCfg->hostRxTsEn     = true;
    cptsCfg->tsCompPolarity = true;
    cptsCfg->tsRxEventsDis  = false;
    cptsCfg->tsGenfClrEn    = true;
    cptsCfg->cptsRftClkFreq = CPSW_CPTS_RFTCLK_FREQ_200MHZ;
}

int32_t CpswCpts_open(EnetMod_Handle hMod,
                      Enet_Type enetType,
                      uint32_t instId,
                      const void *cfg,
                      uint32_t cfgSize)
{
    CpswCpts_Handle hCpts = (CpswCpts_Handle)hMod;
    const CpswCpts_Cfg *cptsCfg = (const CpswCpts_Cfg *)cfg;
    CSL_cptsRegs *regs = (CSL_cptsRegs *)hMod->virtAddr;
    CSL_CPTS_CONTROL control;
    uint32_t i;
    int32_t status = ENET_SOK;

    Enet_devAssert(cfgSize == sizeof(CpswCpts_Cfg),
                   "Invalid CPTS config params size %u (expected %u)\n",
                   cfgSize, sizeof(CpswCpts_Cfg));

    Enet_devAssert(regs != NULL, "CPSW CPTS regs address is not valid\n");

    /* Check supported CPTS module versions */
#if ENET_CFG_IS_ON(DEV_ERROR)
    status = CpswCpts_isSupported(regs);
    Enet_devAssert(status == ENET_SOK, "CPTS version is not supported\n");
#endif

    hCpts->tsPushInFifo = false;
    hCpts->activeGenfIdxMask = 0U;
    hCpts->activeEstfIdxMask = 0U;
    hCpts->tsAddVal = cptsCfg->cptsRftClkFreq;

    memset(&control, 0, sizeof(CSL_CPTS_CONTROL));

    /* Set default values for control register fields */
    control.cptsEn       = TRUE;
    control.intTest      = FALSE;
    control.ts64bMode    = TRUE;
    control.tsOutputBitSel = CPSW_CPTS_TS_OUTPUT_BIT_DISABLED;
    control.seqEn        = FALSE;
    control.tsCompToggle = FALSE;

    for (i = 0U; i < hCpts->hwPushCnt; i++)
    {
        control.tsHwPushEn[i] = TRUE;
    }

    /* Set application-based CPTS control configurations */
    control.tsCompPolarity    = cptsCfg->tsCompPolarity;
    control.tsDisableRxEvents = cptsCfg->tsRxEventsDis;
    control.tsGenfClrEn       = cptsCfg->tsGenfClrEn;
    control.tstampEn          = cptsCfg->hostRxTsEn;

    CSL_CPTS_disableCpts(regs);

    CSL_CPTS_setCntlReg(regs, &control);
    CSL_CPTS_enableCpts(regs);

    /* Configure timestamp add value to enable 1-ns operations */
    CSL_CPTS_setTSAddVal(regs, cptsCfg->cptsRftClkFreq);

    return status;
}

int32_t CpswCpts_rejoin(EnetMod_Handle hMod,
                        Enet_Type enetType,
                        uint32_t instId)
{
    return ENET_SOK;
}

void CpswCpts_saveCtxt(EnetMod_Handle hMod)
{
    CpswCpts_close(hMod);
}

int32_t CpswCpts_restoreCtxt(EnetMod_Handle hMod,
                             Enet_Type enetType,
                             uint32_t instId,
                             const void *cfg,
                             uint32_t cfgSize)
{
    int32_t status = ENET_SOK;

    CpswCpts_open(hMod, enetType, instId, cfg, cfgSize);

    return status;
}

void CpswCpts_close(EnetMod_Handle hMod)
{
    CSL_cptsRegs *regs = (CSL_cptsRegs *)hMod->virtAddr;

    /* Disable CPTS module to disable events*/
    CSL_CPTS_disableCpts(regs);
}

int32_t CpswCpts_ioctl(EnetMod_Handle hMod,
                       uint32_t cmd,
                       Enet_IoctlPrms *prms)
{
    CpswCpts_Handle hCpts = (CpswCpts_Handle)hMod;
    CSL_cptsRegs *regs = (CSL_cptsRegs *)hMod->virtAddr;
    int32_t status = ENET_SOK;

#if ENET_CFG_IS_ON(DEV_ERROR)
    /* Validate CPSW CPTS IOCTL parameters */
    if (ENET_IOCTL_GET_PER(cmd) == ENET_IOCTL_PER_CPSW)
    {
        if (ENET_IOCTL_GET_TYPE(cmd) == ENET_IOCTL_TYPE_PUBLIC)
        {
            status = Enet_validateIoctl(cmd, prms,
                                        gCpswCpts_ioctlValidate,
                                        ENET_ARRAYSIZE(gCpswCpts_ioctlValidate));
        }
        else
        {
            status = Enet_validateIoctl(cmd, prms,
                                        gCpswCpts_privIoctlValidate,
                                        ENET_ARRAYSIZE(gCpswCpts_privIoctlValidate));
        }

        ENETTRACE_ERR_IF(status != ENET_SOK, "IOCTL 0x%08x params are not valid\n", cmd);
    }
#endif

    if (status == ENET_SOK)
    {
        CpswCptsIoctlHandler * ioctlHandlerFxn;

        ioctlHandlerFxn = CpswCpts_getIoctlHandlerFxn(cmd, CpswCptsIoctlHandlerRegistry, ENET_ARRAYSIZE(CpswCptsIoctlHandlerRegistry));
        Enet_devAssert(ioctlHandlerFxn != NULL);
        status = ioctlHandlerFxn(hCpts, regs, prms);
    }

    return status;
}

#if ENET_CFG_IS_ON(DEV_ERROR)
static int32_t CpswCpts_isSupported(CSL_cptsRegs *regs)
{
    CSL_CPTS_VERSION version;
    uint32_t i;
    int32_t status = ENET_ENOTSUPPORTED;

    CSL_CPTS_getCptsVersionInfo(regs, &version);

    for (i = 0U; i < ENET_ARRAYSIZE(CpswCpts_gSupportedVer); i++)
    {
        if ((version.majorVer == CpswCpts_gSupportedVer[i].majorVer) &&
            (version.minorVer == CpswCpts_gSupportedVer[i].minorVer) &&
            (version.rtlVer == CpswCpts_gSupportedVer[i].rtlVer) &&
            (version.id == CpswCpts_gSupportedVer[i].id))
        {
            status = ENET_SOK;
            break;
        }
    }

    return status;
}
#endif


static int32_t CpswCpts_getIoctlHandlerIdx(uint32_t ioctlCmd, CpswCptsIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx)
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

static CpswCptsIoctlHandler * CpswCpts_getIoctlHandlerFxn(uint32_t ioctlCmd, CpswCptsIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;
    CpswCptsIoctlHandler *handlerFxn = NULL;

    status = CpswCpts_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert(tblIdx < tableSize, "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        handlerFxn = ioctlRegistryTbl[tblIdx].fxn;
    }
    else
    {
        handlerFxn = &CpswCpts_ioctl_handler_default;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get ioctl handler for ioctl cmd: %x. Using default ioctl handler \r\n", ioctlCmd);
    return handlerFxn;
}

static int32_t CpswCpts_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                          CpswCptsIoctlHandler *ioctlHandlerFxn,
                                          CpswCptsIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                          uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;

    status = CpswCpts_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert((tblIdx < tableSize), "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        Enet_devAssert((ioctlCmd == ioctlRegistryTbl[tblIdx].cmd), "Ioctl table corrupted");
        ioctlRegistryTbl[tblIdx].fxn = ioctlHandlerFxn;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set ioctl handler for ioctl cmd: %x \r\n", ioctlCmd);
    return status;
}

static int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_REGISTER_HANDLER(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    const Enet_IoctlRegisterHandlerInArgs *inArgs = (const Enet_IoctlRegisterHandlerInArgs *)prms->inArgs;
    int32_t status;

    status = CpswCpts_setIoctlHandlerFxn(inArgs->cmd,
                                        (CpswCptsIoctlHandler *)inArgs->fxn,
                                        CpswCptsIoctlHandlerRegistry,
                                        ENET_ARRAYSIZE(CpswCptsIoctlHandlerRegistry));
    return status;
}


static int32_t CpswCpts_ioctl_handler_default(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    return ENET_ENOTSUPPORTED;
}
