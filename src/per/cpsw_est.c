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
 * \file  cpsw_est.c
 *
 * \brief This file contains the implementation of the global configuration
 *        needed for EST feature.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>
#include <enet.h>
#include <csl_cpswitch.h>
#include <enet_cfg.h>
#if defined(SOC_AM273X) || defined(SOC_AWR294X) || defined (SOC_AWR2544) || defined(SOC_AM263X) || defined(SOC_AM263PX)
#include <priv/per/cpsw_cpdma_priv.h>
#else
#include <priv/per/cpsw_priv.h>
#endif
#include <include/per/cpsw.h>
#include <priv/per/cpsw_est_ioctl_priv.h>

#if ENET_CFG_IS_ON(CPSW_EST)

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define CPSW_EST_IOCTL_HANDLER_ENTRY_INIT(x)    \
          {.cmd = x,                            \
           .fxn = &CpswEst_ioctl_handler_##x}

#define CPSW_EST_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(x)    \
          {.cmd = x,                                    \
           .fxn = &CpswEst_ioctl_handler_default}

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef int32_t (CpswEstIoctlHandler)(Cpsw_Handle hCpsw, uint32_t cmd, Enet_IoctlPrms *prms);

typedef struct CpswEstIoctlHandlerRegistry_s
{
    uint32_t cmd;
    CpswEstIoctlHandler *fxn;
} CpswEstIoctlHandlerRegistry_t;


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static int32_t CpswEst_ioctl_handler_default(Cpsw_Handle hCpsw, uint32_t cmd, Enet_IoctlPrms *prms);
static int32_t CpswEst_ioctl_handler_CPSW_EST_IOCTL_REGISTER_HANDLER(Cpsw_Handle hCpsw, uint32_t cmd, Enet_IoctlPrms *prms);
static int32_t CpswEst_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                          CpswEstIoctlHandler *ioctlHandlerFxn,
                                          CpswEstIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                          uint32_t tableSize);
static CpswEstIoctlHandler * CpswEst_getIoctlHandlerFxn(uint32_t ioctlCmd, CpswEstIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize);
static int32_t CpswEst_getIoctlHandlerIdx(uint32_t ioctlCmd, CpswEstIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx);


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static CpswEstIoctlHandlerRegistry_t CpswEstIoctlHandlerRegistry[] =
{
    CPSW_EST_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_SET_ADMIN_LIST),
    CPSW_EST_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_SET_STATE),
    CPSW_EST_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_GET_ADMIN_LIST),
    CPSW_EST_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_GET_OPER_LIST),
    CPSW_EST_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_GET_OPER_LIST_STATUS),
    CPSW_EST_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_GET_STATE),
    CPSW_EST_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_CONFIG_CHANGE_STATUS_PARAMS),
    CPSW_EST_IOCTL_HANDLER_ENTRY_INIT(CPSW_EST_IOCTL_REGISTER_HANDLER),
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Cpsw_enableEst(EnetPer_Handle hPer)
{
    Cpsw_Handle hCpsw = (Cpsw_Handle)hPer;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hPer->virtAddr;
    CSL_CPSW_CONTROL controlReg;
    uint32_t i;

    for (i = 0U; i < ENET_ARRAYSIZE(hCpsw->estState); i++)
    {
        hCpsw->estState[i].cycleTime = 0U;
        hCpsw->estState[i].state = ENET_TAS_RESET;
    }

    /* Enable EST (global config).  Per-port EST enable control is set via TAS ioctl */
    CSL_CPSW_getCpswControlReg(regs, &controlReg);
    controlReg.estEnable = TRUE;
    CSL_CPSW_setCpswControlReg(regs, &controlReg);
}

void Cpsw_disableEst(EnetPer_Handle hPer)
{
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hPer->virtAddr;
    CSL_CPSW_CONTROL controlReg;

    /* Disable EST (global config) */
    CSL_CPSW_getCpswControlReg(regs, &controlReg);
    controlReg.estEnable = FALSE;
    CSL_CPSW_setCpswControlReg(regs, &controlReg);
}

int32_t Cpsw_ioctlEst(EnetPer_Handle hPer,
                      uint32_t cmd,
                      Enet_IoctlPrms *prms)
{
    Cpsw_Handle hCpsw = (Cpsw_Handle)hPer;
    int32_t status = ENET_SOK;
    CpswEstIoctlHandler * ioctlHandlerFxn;

    ioctlHandlerFxn = CpswEst_getIoctlHandlerFxn(cmd, CpswEstIoctlHandlerRegistry, ENET_ARRAYSIZE(CpswEstIoctlHandlerRegistry));
    Enet_devAssert(ioctlHandlerFxn != NULL);
    status = ioctlHandlerFxn(hCpsw, cmd, prms);

    return status;
}

static int32_t CpswEst_getIoctlHandlerIdx(uint32_t ioctlCmd, CpswEstIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx)
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

static CpswEstIoctlHandler * CpswEst_getIoctlHandlerFxn(uint32_t ioctlCmd, CpswEstIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;
    CpswEstIoctlHandler *handlerFxn = NULL;

    status = CpswEst_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert(tblIdx < tableSize, "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        handlerFxn = ioctlRegistryTbl[tblIdx].fxn;
    }
    else
    {
        handlerFxn = &CpswEst_ioctl_handler_default;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get ioctl handler for ioctl cmd: %x. Using default ioctl handler \r\n", ioctlCmd);
    return handlerFxn;
}

static int32_t CpswEst_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                          CpswEstIoctlHandler *ioctlHandlerFxn,
                                          CpswEstIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                          uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;

    status = CpswEst_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert((tblIdx < tableSize), "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        Enet_devAssert((ioctlCmd == ioctlRegistryTbl[tblIdx].cmd), "Ioctl table corrupted");
        ioctlRegistryTbl[tblIdx].fxn = ioctlHandlerFxn;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set ioctl handler for ioctl cmd: %x \r\n", ioctlCmd);
    return status;
}

static int32_t CpswEst_ioctl_handler_CPSW_EST_IOCTL_REGISTER_HANDLER(Cpsw_Handle hCpsw, uint32_t cmd, Enet_IoctlPrms *prms)
{
    const Enet_IoctlRegisterHandlerInArgs *inArgs = (const Enet_IoctlRegisterHandlerInArgs *)prms->inArgs;
    int32_t status;

    status = CpswEst_setIoctlHandlerFxn(inArgs->cmd,
                                        (CpswEstIoctlHandler *)inArgs->fxn,
                                        CpswEstIoctlHandlerRegistry,
                                        ENET_ARRAYSIZE(CpswEstIoctlHandlerRegistry));
    return status;
}


static int32_t CpswEst_ioctl_handler_default(Cpsw_Handle hCpsw, uint32_t cmd, Enet_IoctlPrms *prms)
{
    return ENET_ENOTSUPPORTED;
}

#endif
