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
 * \file  cpsw_macport_intervlan.c
 *
 * \brief This file contains the implementation of the InterVLAN feature of
 *        the CPSW MAC port module.
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
#include "cpsw_macport_intervlan_ioctl_priv.h"

#if ENET_CFG_IS_ON(CPSW_MACPORT_INTERVLAN)

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define CPSW_MACPORT_INTERVLAN_IOCTL_HANDLER_ENTRY_INIT(x)    \
          {.cmd = x,                            \
           .fxn = &CpswMacPort_interVlan_ioctl_handler_##x}

#define CPSW_MACPORT_INTERVLAN_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(x)    \
          {.cmd = x,                                    \
           .fxn = &CpswMacPort_interVlan_ioctl_handler_default}

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef int32_t (CpswMacPortInterVlanIoctlHandler)(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);

typedef struct CpswMacPortInterVlanIoctlHandlerRegistry_s
{
    uint32_t cmd;
    CpswMacPortInterVlanIoctlHandler *fxn;
} CpswMacPortInterVlanIoctlHandlerRegistry_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void CpswMacPort_clearAllRoutes(CSL_Xge_cpswRegs *regs,
                                       Enet_MacPort macPort);

static int32_t CpswMacPort_interVlan_ioctl_handler_default(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
static int32_t CpswMacPort_interVlan_ioctl_handler_CPSW_MACPORT_IOCTL_REGISTER_HANDLER(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms);
static int32_t CpswMacPort_interVlan_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                          CpswMacPortInterVlanIoctlHandler *ioctlHandlerFxn,
                                          CpswMacPortInterVlanIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                          uint32_t tableSize);
static CpswMacPortInterVlanIoctlHandler * CpswMacPort_interVlan_getIoctlHandlerFxn(uint32_t ioctlCmd, CpswMacPortInterVlanIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize);
static int32_t CpswMacPort_interVlan_getIoctlHandlerIdx(uint32_t ioctlCmd, CpswMacPortInterVlanIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static CpswMacPortInterVlanIoctlHandlerRegistry_t CpswMacPortInterVlanIoctlHandlerRegistry[] = 
{
    CPSW_MACPORT_INTERVLAN_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_SET_INTERVLAN_ROUTE),
    CPSW_MACPORT_INTERVLAN_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_SET_SPECIFIC_INTERVLAN_ROUTE),
    CPSW_MACPORT_INTERVLAN_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_GET_INTERVLAN_ROUTE),
    CPSW_MACPORT_INTERVLAN_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_DELETE_INTERVLAN_ROUTE),
    CPSW_MACPORT_INTERVLAN_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_GET_INTERVLAN_FREEROUTES),
    CPSW_MACPORT_INTERVLAN_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_FIND_INTERVLAN_ROUTE),
    CPSW_MACPORT_INTERVLAN_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(CPSW_MACPORT_IOCTL_IS_INTERVLAN_ROUTE_FREE),
    CPSW_MACPORT_INTERVLAN_IOCTL_HANDLER_ENTRY_INIT(CPSW_MACPORT_IOCTL_REGISTER_HANDLER),
};


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void CpswMacPort_openInterVlan(EnetMod_Handle hMod)
{
    CpswMacPort_Handle hPort = (CpswMacPort_Handle)hMod;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;
    Enet_MacPort macPort = hPort->macPort;

    CpswMacPort_clearAllRoutes(regs, macPort);
}

int32_t CpswMacPort_ioctlInterVlan(EnetMod_Handle hMod,
                                   uint32_t cmd,
                                   Enet_IoctlPrms *prms)
{
    CpswMacPort_Handle hPort = (CpswMacPort_Handle)hMod;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hMod->virtAddr;
    Enet_MacPort macPort = hPort->macPort;
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status = ENET_SOK;
    CpswMacPortInterVlanIoctlHandler * ioctlHandlerFxn;

    ENETTRACE_VAR(portId);
    ioctlHandlerFxn = CpswMacPort_interVlan_getIoctlHandlerFxn(cmd, CpswMacPortInterVlanIoctlHandlerRegistry, ENET_ARRAYSIZE(CpswMacPortInterVlanIoctlHandlerRegistry));
    Enet_devAssert(ioctlHandlerFxn != NULL);
    status = ioctlHandlerFxn(hPort, regs, prms);

    return status;
}

static void CpswMacPort_clearAllRoutes(CSL_Xge_cpswRegs *regs,
                                       Enet_MacPort macPort)
{
    CpswMacPort_InterVlanRouteId i;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);

    for (i = CPSW_MACPORT_INTERVLAN_ROUTEID_FIRST;
         i <= CPSW_MACPORT_INTERVLAN_ROUTEID_LAST;
         i = (CpswMacPort_InterVlanRouteId)(i + 1))
    {
        if (!CpswMacPort_isRouteIdFree(regs, portNum, i))
        {
            CpswMacPort_clearRouteId(regs, portNum, i);
        }
    }
}


void CpswMacPort_clearRouteId(CSL_Xge_cpswRegs *regs,
                              uint32_t portNum,
                              CpswMacPort_InterVlanRouteId routeId)
{
    CSL_CPSW_INTERVLANCFG cslCfg;
    uint32_t interVlanPtr = CPSW_MACPORT_INTERVLAN_ROUTE2PTR(routeId);

    cslCfg.decrementTtl = FALSE;
    cslCfg.destForceUntaggedEgress = FALSE;
    cslCfg.replaceDaSa = FALSE;
    cslCfg.replaceVid = FALSE;
    cslCfg.vid = FALSE;
    EnetUtils_clearMacAddr(&cslCfg.dstMacAddress[0U]);
    EnetUtils_clearMacAddr(&cslCfg.srcMacAddress[0U]);

    CSL_CPGMAC_SL_setInterVLANCfg(regs, portNum, interVlanPtr, &cslCfg);
}

bool CpswMacPort_isRouteIdFree(CSL_Xge_cpswRegs *regs,
                               uint32_t portNum,
                               CpswMacPort_InterVlanRouteId routeId)
{
    CSL_CPSW_INTERVLANCFG cslCfg =  { {0} };
    uint32_t interVlanPtr = CPSW_MACPORT_INTERVLAN_ROUTE2PTR(routeId);
    bool isFree = false;

    CSL_CPGMAC_SL_getInterVLANCfg(regs, portNum, interVlanPtr, &cslCfg);

    /* If a route does not have replace VId enabled it is not used. Use that as an
     * indicator to identify free routes.
     * We cannot use any software state to maintain interVLAN route allocation as
     * software state may get lost during warm boot. */
    if ((cslCfg.replaceVid == 0U) &&
        (cslCfg.destForceUntaggedEgress == 0U))
    {
        isFree = true;
    }

    return isFree;
}


static int32_t CpswMacPort_interVlan_getIoctlHandlerIdx(uint32_t ioctlCmd, CpswMacPortInterVlanIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx)
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

static CpswMacPortInterVlanIoctlHandler * CpswMacPort_interVlan_getIoctlHandlerFxn(uint32_t ioctlCmd, CpswMacPortInterVlanIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;
    CpswMacPortInterVlanIoctlHandler *handlerFxn = NULL;

    status = CpswMacPort_interVlan_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert(tblIdx < tableSize, "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        handlerFxn = ioctlRegistryTbl[tblIdx].fxn;
    }
    else
    {
        handlerFxn = &CpswMacPort_interVlan_ioctl_handler_default;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get ioctl handler for ioctl cmd: %x. Using default ioctl handler \r\n", ioctlCmd);
    return handlerFxn;
}

static int32_t CpswMacPort_interVlan_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                          CpswMacPortInterVlanIoctlHandler *ioctlHandlerFxn,
                                          CpswMacPortInterVlanIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                          uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;

    status = CpswMacPort_interVlan_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert((tblIdx < tableSize), "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        Enet_devAssert((ioctlCmd == ioctlRegistryTbl[tblIdx].cmd), "Ioctl table corrupted");
        ioctlRegistryTbl[tblIdx].fxn = ioctlHandlerFxn;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set ioctl handler for ioctl cmd: %x \r\n", ioctlCmd);
    return status;
}

static int32_t CpswMacPort_interVlan_ioctl_handler_CPSW_MACPORT_IOCTL_REGISTER_HANDLER(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    const Enet_IoctlRegisterHandlerInArgs *inArgs = (const Enet_IoctlRegisterHandlerInArgs *)prms->inArgs;
    int32_t status;

    status = CpswMacPort_interVlan_setIoctlHandlerFxn(inArgs->cmd, 
                                        (CpswMacPortInterVlanIoctlHandler *)inArgs->fxn, 
                                        CpswMacPortInterVlanIoctlHandlerRegistry, 
                                        ENET_ARRAYSIZE(CpswMacPortInterVlanIoctlHandlerRegistry));
    return status;
}


static int32_t CpswMacPort_interVlan_ioctl_handler_default(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    return ENET_ENOTSUPPORTED;
}


#endif
