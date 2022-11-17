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


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static int32_t CpswMacPort_setRoute(CSL_Xge_cpswRegs *regs,
                                    Enet_MacPort macPort,
                                    const CpswMacPort_InterVlanRoutingCfg *interVlanCfg,
                                    CpswMacPort_InterVlanRouteId *routeId);

static int32_t CpswMacPort_setCfg(CSL_Xge_cpswRegs *regs,
                                  Enet_MacPort macPort,
                                  CpswMacPort_InterVlanRouteId routeId,
                                  const CpswMacPort_InterVlanRoutingCfg *interVlanCfg);

static int32_t CpswMacPort_getCfg(CSL_Xge_cpswRegs *regs,
                                  Enet_MacPort macPort,
                                  CpswMacPort_InterVlanRouteId routeId,
                                  CpswMacPort_InterVlanRoutingCfg *interVlanCfg);

static int32_t CpswMacPort_delRoute(CSL_Xge_cpswRegs *regs,
                                    Enet_MacPort macPort,
                                    const CpswMacPort_InterVlanRoutingCfg *interVlanCfg,
                                    CpswMacPort_InterVlanRouteId *routeId);

static int32_t CpswMacPort_getFreeRoutes(CSL_Xge_cpswRegs *regs,
                                         Enet_MacPort macPort,
                                         CpswMacPort_InterVlanFreeRouteInfo *freeRoutes);

static int32_t CpswMacPort_findRouteId(CSL_Xge_cpswRegs *regs,
                                       Enet_MacPort macPort,
                                       const CpswMacPort_InterVlanRoutingCfg *routeMatchCfg,
                                       CpswMacPort_InterVlanRouteId *routeId);

static int32_t CpswMacPort_checkRouteIsFree(CSL_Xge_cpswRegs *regs,
                                            Enet_MacPort macPort,
                                            CpswMacPort_InterVlanRouteId routeId,
                                            bool *isFree);

static bool CpswMacPort_cmpRoutes(const CpswMacPort_InterVlanRoutingCfg *cfg1,
                                  const CpswMacPort_InterVlanRoutingCfg *cfg2);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t  CpswMacPort_interVlan_ioctl_handler_CPSW_MACPORT_IOCTL_SET_INTERVLAN_ROUTE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswMacPort_InterVlanRoutingCfg *interVlanCfg =
        (const CpswMacPort_InterVlanRoutingCfg *)prms->inArgs;
    CpswMacPort_InterVlanRouteId *routeId =
        (CpswMacPort_InterVlanRouteId *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    status = CpswMacPort_setRoute(regs, macPort, interVlanCfg, routeId);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: Failed to set interVLAN route: %d\n", ENET_MACPORT_ID(macPort), status);
    return status;
}

int32_t  CpswMacPort_interVlan_ioctl_handler_CPSW_MACPORT_IOCTL_SET_SPECIFIC_INTERVLAN_ROUTE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswMacPort_SetSpecificInterVlanRouteInArgs *inArgs =
        (const CpswMacPort_SetSpecificInterVlanRouteInArgs *)prms->inArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    status = CpswMacPort_setCfg(regs, macPort, inArgs->routeId, &inArgs->routeCfg);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: Failed to set specific interVLAN cfg: %d\n", ENET_MACPORT_ID(macPort), status);
    return status;
}

int32_t  CpswMacPort_interVlan_ioctl_handler_CPSW_MACPORT_IOCTL_GET_INTERVLAN_ROUTE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    CpswMacPort_InterVlanRouteId routeId =
        *(CpswMacPort_InterVlanRouteId *)prms->inArgs;
    CpswMacPort_InterVlanRoutingCfg *interVlanCfg =
        (CpswMacPort_InterVlanRoutingCfg *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    status = CpswMacPort_getCfg(regs, macPort, routeId, interVlanCfg);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: failed to get interVLAN config: %d\n", ENET_MACPORT_ID(macPort), status);
    return status;
}

int32_t  CpswMacPort_interVlan_ioctl_handler_CPSW_MACPORT_IOCTL_DELETE_INTERVLAN_ROUTE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswMacPort_InterVlanRoutingCfg *interVlanCfg =
        (const CpswMacPort_InterVlanRoutingCfg *)prms->inArgs;
    CpswMacPort_InterVlanRouteId *routeId =
        (CpswMacPort_InterVlanRouteId *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    status = CpswMacPort_delRoute(regs, macPort, interVlanCfg, routeId);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: Failed to delete interVLAN route: %d\n", ENET_MACPORT_ID(macPort), status);
    return status;
}

int32_t  CpswMacPort_interVlan_ioctl_handler_CPSW_MACPORT_IOCTL_GET_INTERVLAN_FREEROUTES(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    CpswMacPort_InterVlanFreeRouteInfo *freeRoutes =
        (CpswMacPort_InterVlanFreeRouteInfo *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    status = CpswMacPort_getFreeRoutes(regs, macPort, freeRoutes);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: failed to get free interVLAN routes: %d\n", ENET_MACPORT_ID(macPort), status);
    return status;
}

int32_t  CpswMacPort_interVlan_ioctl_handler_CPSW_MACPORT_IOCTL_FIND_INTERVLAN_ROUTE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswMacPort_InterVlanRoutingCfg *interVlanCfg =
        (const CpswMacPort_InterVlanRoutingCfg *)prms->inArgs;
    CpswMacPort_InterVlanRouteId *routeId = (CpswMacPort_InterVlanRouteId *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    status = CpswMacPort_findRouteId(regs, macPort, interVlanCfg, routeId);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: failed to find interVLAN route: %d\n", ENET_MACPORT_ID(macPort), status);
    return status;
}

int32_t  CpswMacPort_interVlan_ioctl_handler_CPSW_MACPORT_IOCTL_IS_INTERVLAN_ROUTE_FREE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    CpswMacPort_InterVlanRouteId routeId = *(CpswMacPort_InterVlanRouteId *)prms->inArgs;
    bool *isFree = (bool *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    status = CpswMacPort_checkRouteIsFree(regs, macPort, routeId, isFree);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: failed to check interVLAN route state: %d\n", ENET_MACPORT_ID(macPort), status);
    return status;
}

static int32_t CpswMacPort_setRoute(CSL_Xge_cpswRegs *regs,
                                    Enet_MacPort macPort,
                                    const CpswMacPort_InterVlanRoutingCfg *interVlanCfg,
                                    CpswMacPort_InterVlanRouteId *routeId)
{
    CpswMacPort_InterVlanFreeRouteInfo freeRoutes;
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status;

    ENETTRACE_VAR(portId);
    status = CpswMacPort_findRouteId(regs, macPort, interVlanCfg, routeId);
    if (status == ENET_SOK)
    {
        /* Entry already present. No need to add entry */
    }
    else
    {
        status = CpswMacPort_getFreeRoutes(regs, macPort, &freeRoutes);
        if (status == ENET_SOK)
        {
            if (freeRoutes.numFreeRoutes > 0U)
            {
                status = CpswMacPort_setCfg(regs, macPort, freeRoutes.freeRouteId[0U], interVlanCfg);
                if (status == ENET_SOK)
                {
                    *routeId = freeRoutes.freeRouteId[0];
                }
                else
                {
                    ENETTRACE_ERR("MAC %u: Failed to set interVLAN config: %d\n", portId, status);
                }
            }
            else
            {
                ENETTRACE_ERR("MAC %u: No free routes available\n", portId);
                status = ENET_EFAIL;
            }
        }
        else
        {
            ENETTRACE_ERR("MAC %u: Failed to get free interVLAN routes: %d\n", portId, status);
        }
    }

    return status;
}

static int32_t CpswMacPort_setCfg(CSL_Xge_cpswRegs *regs,
                                  Enet_MacPort macPort,
                                  CpswMacPort_InterVlanRouteId routeId,
                                  const CpswMacPort_InterVlanRoutingCfg *interVlanCfg)
{
    CSL_CPSW_INTERVLANCFG cslCfg;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    uint32_t interVlanPtr;
    int32_t status = ENET_SOK;

    ENETTRACE_VAR(portId);
    if (routeId > CPSW_MACPORT_INTERVLAN_ROUTEID_LAST)
    {
        ENETTRACE_ERR("MAC %u: Invalid route id %u\n", portId, routeId);
        status = ENET_EINVALIDPARAMS;
    }

    if (EnetUtils_isMcastAddr(interVlanCfg->srcAddr))
    {
        ENETTRACE_ERR("MAC %u: source addr is multicast\n", portId);
        status = ENET_EINVALIDPARAMS;
    }

    if (interVlanCfg->vlanId > ENET_VLAN_ID_MAX)
    {
        ENETTRACE_ERR("MAC %u: interVLAN VLAN ID %u exceeds max\n", portId, interVlanCfg->vlanId);
        status = ENET_EINVALIDPARAMS;
    }

    if (status == ENET_SOK)
    {
        interVlanPtr = CPSW_MACPORT_INTERVLAN_ROUTE2PTR(routeId);

        cslCfg.decrementTtl            = interVlanCfg->decrementTTL;
        cslCfg.destForceUntaggedEgress = interVlanCfg->forceUntaggedEgress;
        cslCfg.replaceDaSa             = interVlanCfg->replaceDASA;
        EnetUtils_copyMacAddr(cslCfg.dstMacAddress, interVlanCfg->dstAddr);

        /* CPSW IP required replaceVid irrespective of whether VID needs to be replaced
         * or not. Else DA/SA VLAN strip is not functional. Hence always enable replaceVid. */
        cslCfg.replaceVid = TRUE;
        cslCfg.vid        = interVlanCfg->vlanId;
        EnetUtils_copyMacAddr(cslCfg.srcMacAddress, interVlanCfg->srcAddr);
        CSL_CPGMAC_SL_setInterVLANCfg(regs, portNum, interVlanPtr, &cslCfg);
    }

    return status;
}

static int32_t CpswMacPort_getCfg(CSL_Xge_cpswRegs *regs,
                                  Enet_MacPort macPort,
                                  CpswMacPort_InterVlanRouteId routeId,
                                  CpswMacPort_InterVlanRoutingCfg *interVlanCfg)
{
    CSL_CPSW_INTERVLANCFG cslCfg;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    uint32_t interVlanPtr;
    int32_t status = ENET_SOK;

    ENETTRACE_VAR(portId);
    if (routeId <= CPSW_MACPORT_INTERVLAN_ROUTEID_LAST)
    {
        interVlanPtr = CPSW_MACPORT_INTERVLAN_ROUTE2PTR(routeId);

        CSL_CPGMAC_SL_getInterVLANCfg(regs, portNum, interVlanPtr, &cslCfg);

        interVlanCfg->decrementTTL        = cslCfg.decrementTtl;
        interVlanCfg->forceUntaggedEgress = cslCfg.destForceUntaggedEgress;
        interVlanCfg->replaceDASA         = cslCfg.replaceDaSa;
        interVlanCfg->vlanId              = cslCfg.vid;

        EnetUtils_copyMacAddr(interVlanCfg->dstAddr, cslCfg.dstMacAddress);
        EnetUtils_copyMacAddr(interVlanCfg->srcAddr, cslCfg.srcMacAddress);
    }
    else
    {
        ENETTRACE_ERR("MAC %u: Invalid route id %u\n", portId, routeId);
        status = ENET_EINVALIDPARAMS;
    }

    return status;
}

static int32_t CpswMacPort_delRoute(CSL_Xge_cpswRegs *regs,
                                    Enet_MacPort macPort,
                                    const CpswMacPort_InterVlanRoutingCfg *interVlanCfg,
                                    CpswMacPort_InterVlanRouteId *routeId)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status;

    ENETTRACE_VAR(portId);
    status = CpswMacPort_findRouteId(regs, macPort, interVlanCfg, routeId);
    if (status == ENET_SOK)
    {
        CpswMacPort_clearRouteId(regs, portNum, *routeId);
    }
    else
    {
        ENETTRACE_ERR("MAC %u: Failed to find InterVLAN route to be deleted: %d\n", portId, status);
    }

    return status;
}

static int32_t CpswMacPort_getFreeRoutes(CSL_Xge_cpswRegs *regs,
                                         Enet_MacPort macPort,
                                         CpswMacPort_InterVlanFreeRouteInfo *freeRoutes)
{
    CpswMacPort_InterVlanRouteId i;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status = ENET_SOK;

    ENETTRACE_VAR(portId);
    freeRoutes->numFreeRoutes = 0U;

    for (i = CPSW_MACPORT_INTERVLAN_ROUTEID_FIRST;
         i <= CPSW_MACPORT_INTERVLAN_ROUTEID_LAST;
         i = (CpswMacPort_InterVlanRouteId)(i + 1))
    {
        if (CpswMacPort_isRouteIdFree(regs, portNum, i))
        {
            if (freeRoutes->numFreeRoutes < ENET_ARRAYSIZE(freeRoutes->freeRouteId))
            {
                freeRoutes->freeRouteId[freeRoutes->numFreeRoutes] = i;
                freeRoutes->numFreeRoutes++;
            }
            else
            {
                ENETTRACE_ERR("MAC %u: Not enough space for free routes\n", portId);
                status = ENET_EINVALIDPARAMS;
                break;
            }
        }
    }

    return status;
}

static int32_t CpswMacPort_findRouteId(CSL_Xge_cpswRegs *regs,
                                       Enet_MacPort macPort,
                                       const CpswMacPort_InterVlanRoutingCfg *routeMatchCfg,
                                       CpswMacPort_InterVlanRouteId *routeId)
{
    CpswMacPort_InterVlanRoutingCfg routeCfg;
    CpswMacPort_InterVlanRouteId i;
    bool found = false;
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status;

    ENETTRACE_VAR(portId);
    for (i = CPSW_MACPORT_INTERVLAN_ROUTEID_FIRST;
         i <= CPSW_MACPORT_INTERVLAN_ROUTEID_LAST;
         i = (CpswMacPort_InterVlanRouteId)(i + 1))
    {
        if (CpswMacPort_getCfg(regs, macPort, i, &routeCfg) == ENET_SOK)
        {
            if (CpswMacPort_cmpRoutes(routeMatchCfg, &routeCfg) == true)
            {
                found = true;
                break;
            }
        }
    }

    if (found)
    {
        Enet_devAssert(i <= CPSW_MACPORT_INTERVLAN_ROUTEID_LAST, "MAC %u: Invalid route id: %d\n", portId, i);
        *routeId = i;
        status = ENET_SOK;
    }
    else
    {
        ENETTRACE_DBG("MAC %u: No interVLAN found with matching params\n", portId);
        status = ENET_ENOTFOUND;
    }

    return status;
}

static int32_t CpswMacPort_checkRouteIsFree(CSL_Xge_cpswRegs *regs,
                                            Enet_MacPort macPort,
                                            CpswMacPort_InterVlanRouteId routeId,
                                            bool *isFree)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status = ENET_SOK;

    ENETTRACE_VAR(portId);
    if (routeId <= CPSW_MACPORT_INTERVLAN_ROUTEID_LAST)
    {
        *isFree = CpswMacPort_isRouteIdFree(regs, portNum, routeId);
    }
    else
    {
        ENETTRACE_ERR("MAC %u: Invalid route id %u\n", portId, routeId);
        status = ENET_EINVALIDPARAMS;
    }

    return status;
}


static bool CpswMacPort_cmpRoutes(const CpswMacPort_InterVlanRoutingCfg *cfg1,
                                  const CpswMacPort_InterVlanRoutingCfg *cfg2)
{
    bool isEqual;

    isEqual = (cfg1->decrementTTL == cfg2->decrementTTL) &&
              (cfg1->forceUntaggedEgress == cfg2->forceUntaggedEgress) &&
              (cfg1->replaceDASA == cfg2->replaceDASA);

    if (isEqual && !cfg1->forceUntaggedEgress)
    {
        isEqual = isEqual && (cfg1->vlanId == cfg2->vlanId);
    }

    if (isEqual && cfg1->replaceDASA)
    {
        isEqual = isEqual && EnetUtils_cmpMacAddr(cfg1->dstAddr, cfg2->dstAddr);
        isEqual = isEqual && EnetUtils_cmpMacAddr(cfg1->srcAddr, cfg2->srcAddr);
    }

    return isEqual;
}

#endif
