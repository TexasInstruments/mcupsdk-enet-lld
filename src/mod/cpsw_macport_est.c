/*
 *  Copyright (c) Texas Instruments Incorporated 2022
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
 * \file  cpsw_macport_est.c
 *
 * \brief This file contains the implementation of the CPSW EST functionality.
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
#include <include/mod/cpsw_macport.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/mod/cpsw_macport_priv.h>
#include <priv/mod/cpsw_clks.h>
#include "cpsw_macport_est.h"
#include "cpsw_macport_est_ioctl_priv.h"


#if ENET_CFG_IS_ON(CPSW_MACPORT_EST)

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define CPSW_MACPORT_EST_IOCTL_HANDLER_ENTRY_INIT(x)    \
          {.cmd = x,                                    \
           .fxn = &CpswMacPortEst_ioctl_handler_##x}

#define CPSW_MACPORT_EST_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(x)    \
          {.cmd = x,                                            \
           .fxn = &CpswMacPortEst_ioctl_handler_default}

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef int32_t (CpswMacPortEstIoctlHandler)(CpswMacPort_Handle hPort, Enet_IoctlPrms *prms);

typedef struct CpswMacPortEstIoctlHandlerRegistry_s
{
    uint32_t cmd;
    CpswMacPortEstIoctlHandler *fxn;
} CpswMacPortEstIoctlHandlerRegistry_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void CpswEst_clearEstBuf(CpswMacPort_Handle hPort);

static int32_t CpswMacPortEst_ioctl_handler_default(CpswMacPort_Handle hPort, Enet_IoctlPrms *prms);
static int32_t CpswMacPortEst_ioctl_handler_CPSW_MACPORT_IOCTL_REGISTER_HANDLER(CpswMacPort_Handle hPort, Enet_IoctlPrms *prms);
static int32_t CpswMacPortEst_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                          CpswMacPortEstIoctlHandler *ioctlHandlerFxn,
                                          CpswMacPortEstIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                          uint32_t tableSize);
static CpswMacPortEstIoctlHandler * CpswMacPortEst_getIoctlHandlerFxn(uint32_t ioctlCmd, CpswMacPortEstIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize);
static int32_t CpswMacPortEst_getIoctlHandlerIdx(uint32_t ioctlCmd, CpswMacPortEstIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx);


int32_t CpswEst_setState(CpswMacPort_Handle hPort, EnetTas_TasState state);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static CpswMacPortEstIoctlHandlerRegistry_t CpswMacPortEstIoctlHandlerRegistry[] = 
{
    CPSW_MACPORT_EST_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_SET_ADMIN_LIST),
    CPSW_MACPORT_EST_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_GET_OPER_LIST_STATUS),
    CPSW_MACPORT_EST_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_SET_STATE),
    CPSW_MACPORT_EST_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_GET_STATE),
    CPSW_MACPORT_EST_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_GET_ADMIN_LIST),
    CPSW_MACPORT_EST_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_GET_OPER_LIST),
    CPSW_MACPORT_EST_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_TAS_IOCTL_CONFIG_CHANGE_STATUS_PARAMS),
    CPSW_MACPORT_EST_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_EST_ENABLE_TIMESTAMP),
    CPSW_MACPORT_EST_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_EST_DISABLE_TIMESTAMP),
    CPSW_MACPORT_EST_IOCTL_HANDLER_ENTRY_INIT(CPSW_MACPORT_IOCTL_REGISTER_HANDLER),
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t CpswMacPort_openEst(EnetMod_Handle hMod)
{
    CpswMacPort_Handle hPort = (CpswMacPort_Handle)hMod;
    Enet_MacPort macPort = hPort->macPort;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;
    CSL_CPSW_EST_CONFIG estCfg;
    uint32_t portNum = ENET_MACPORT_NORM(macPort) + 1U;
    int32_t status;

    /* Initialize state and control lists. Clears both EST buffer banks
     * and set lower bank for first admin list */
    status = CpswEst_setState(hPort, ENET_TAS_RESET);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                     "MAC %u: failed to reset EST: %d\n",
                     ENET_MACPORT_ID(macPort), status);

    /* Configure EST in two-buffer mode */
    memset(&estCfg, 0, sizeof(estCfg));
    estCfg.estBufSel = 0U;
    estCfg.estOneBuf = 0U;
    CSL_CPSW_setPortEstConfig(regs, portNum, &estCfg);

    return status;
}

int32_t CpswMacPort_ioctlEst(EnetMod_Handle hMod,
                             uint32_t cmd,
                             Enet_IoctlPrms *prms)
{
    CpswMacPort_Handle hPort = (CpswMacPort_Handle)hMod;
    Enet_MacPort macPort = hPort->macPort;
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status = ENET_SOK;
    CpswMacPortEstIoctlHandler * ioctlHandlerFxn;

    ENETTRACE_VAR(portId);


    ioctlHandlerFxn = CpswMacPortEst_getIoctlHandlerFxn(cmd, CpswMacPortEstIoctlHandlerRegistry, ENET_ARRAYSIZE(CpswMacPortEstIoctlHandlerRegistry));
    Enet_devAssert(ioctlHandlerFxn != NULL);
    status = ioctlHandlerFxn(hPort, prms);

    return status;
}




static void CpswEst_clearEstBuf(CpswMacPort_Handle hPort)
{
    EnetMod_Handle hMod = ENET_MOD(hPort);
    Enet_MacPort macPort = hPort->macPort;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;
    uint32_t portNum = ENET_MACPORT_NORM(macPort) + 1U;
    uint32_t i;

    ENETTRACE_DBG("MAC %u: Clearing both EST buffers\n", ENET_MACPORT_ID(macPort));

    for (i = 0U; i < (2 * CPSW_MACPORT_EST_BUF_SIZE); i++)
    {
        CSL_CPSW_writeEstFetchCmd(regs, portNum, i, 0U, 0U);
    }

    memset(&hPort->adminList, 0, sizeof(hPort->adminList));
    memset(&hPort->operList, 0, sizeof(hPort->operList));
}


int32_t CpswEst_setState(CpswMacPort_Handle hPort,
                         EnetTas_TasState state)
{
    EnetMod_Handle hMod = ENET_MOD(hPort);
    Enet_MacPort macPort = hPort->macPort;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;
    CSL_CPSW_PORT_CONTROL portControl;
    CSL_CPSW_EST_CONFIG estCfg;
    uint32_t portNum = ENET_MACPORT_NORM(macPort) + 1U;
    int32_t status = ENET_SOK;

    switch (state)
    {
        case ENET_TAS_ENABLE:
        {
            /* Allow EST enable only if a valid admin list has been set before */
            if ((hPort->state == ENET_TAS_RESET) ||
                (hPort->operList.listLength == 0U))
            {
                ENETTRACE_ERR("MAC %u: EST can be enabled only after a valid admin list is set\n",
                              ENET_MACPORT_ID(macPort));
                status = ENET_EPERM;
            }

            if (status == ENET_SOK)
            {
                ENETTRACE_DBG("MAC %u: Set EST state: ENABLE\n", ENET_MACPORT_ID(macPort));

                /* Enable EST on the port */
                CSL_CPSW_getPortControlReg(regs, portNum, &portControl);
                portControl.estPortEnable = 1U;
                CSL_CPSW_setPortControlReg(regs, portNum, &portControl);
            }
            break;
        }

        case ENET_TAS_DISABLE:
        {
            ENETTRACE_DBG("MAC %u: Set EST state: DISABLE\n", ENET_MACPORT_ID(macPort));

            /* Disable EST on the port */
            CSL_CPSW_getPortControlReg(regs, portNum, &portControl);
            portControl.estPortEnable = 0U;
            CSL_CPSW_setPortControlReg(regs, portNum, &portControl);
            break;
        }

        case ENET_TAS_RESET:
        {
            ENETTRACE_DBG("MAC %u: Set EST state: RESET\n", ENET_MACPORT_ID(macPort));

            memset(&hPort->configStatus, 0, sizeof(hPort->configStatus));

            /* Disable EST on the port */
            CSL_CPSW_getPortControlReg(regs, portNum, &portControl);
            portControl.estPortEnable = 0U;
            CSL_CPSW_setPortControlReg(regs, portNum, &portControl);

            /* Clear both EST buffer banks */
            CpswEst_clearEstBuf(hPort);

            /* Next admin bank is EST lower buffer */
            hPort->estBufUpper = false;
            CSL_CPSW_getPortEstConfig(regs, portNum, &estCfg);
            estCfg.estBufSel = hPort->estBufUpper ? 1U : 0U;
            CSL_CPSW_setPortEstConfig(regs, portNum, &estCfg);
            break;
        }

        default:
        {
            ENETTRACE_WARN("MAC %u: Invalid state %u\n", ENET_MACPORT_ID(macPort), state);
            status = ENET_EINVALIDPARAMS;
            break;
        }
    }

    if (status == ENET_SOK)
    {
        hPort->state = state;
    }

    return status;
}

static int32_t CpswMacPortEst_getIoctlHandlerIdx(uint32_t ioctlCmd, CpswMacPortEstIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx)
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

static CpswMacPortEstIoctlHandler * CpswMacPortEst_getIoctlHandlerFxn(uint32_t ioctlCmd, CpswMacPortEstIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;
    CpswMacPortEstIoctlHandler *handlerFxn = NULL;

    status = CpswMacPortEst_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert(tblIdx < tableSize, "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        handlerFxn = ioctlRegistryTbl[tblIdx].fxn;
    }
    else
    {
        handlerFxn = &CpswMacPortEst_ioctl_handler_default;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get ioctl handler for ioctl cmd: %x. Using default ioctl handler \r\n", ioctlCmd);
    return handlerFxn;
}

static int32_t CpswMacPortEst_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                                 CpswMacPortEstIoctlHandler *ioctlHandlerFxn,
                                                 CpswMacPortEstIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                                 uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;

    status = CpswMacPortEst_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert((tblIdx < tableSize), "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        Enet_devAssert((ioctlCmd == ioctlRegistryTbl[tblIdx].cmd), "Ioctl table corrupted");
        ioctlRegistryTbl[tblIdx].fxn = ioctlHandlerFxn;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set ioctl handler for ioctl cmd: %x \r\n", ioctlCmd);
    return status;
}

static int32_t CpswMacPortEst_ioctl_handler_CPSW_MACPORT_IOCTL_REGISTER_HANDLER(CpswMacPort_Handle hPort, Enet_IoctlPrms *prms)
{
    const Enet_IoctlRegisterHandlerInArgs *inArgs = (const Enet_IoctlRegisterHandlerInArgs *)prms->inArgs;
    int32_t status;

    status = CpswMacPortEst_setIoctlHandlerFxn(inArgs->cmd, 
                                               (CpswMacPortEstIoctlHandler *)inArgs->fxn, 
                                               CpswMacPortEstIoctlHandlerRegistry, 
                                               ENET_ARRAYSIZE(CpswMacPortEstIoctlHandlerRegistry));
    return status;
}


static int32_t CpswMacPortEst_ioctl_handler_default(CpswMacPort_Handle hPort, Enet_IoctlPrms *prms)
{
    return ENET_ENOTSUPPORTED;
}


#endif
