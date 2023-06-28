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
 * \file  cpsw_intervlan.c
 *
 * \brief This file contains the implementation of the interVLAN feature of
 *        the CPSW peripheral.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>
#include <csl_cpswitch.h>
#include <enet_cfg.h>
#include <include/core/enet_base.h>
#include <include/core/enet_utils.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/mod/cpsw_ale_priv.h>
#include <priv/mod/cpsw_ale_ioctl_priv.h>
#include <priv/mod/cpsw_macport_priv.h>
#include <priv/mod/cpsw_macport_ioctl_priv.h>
#include <include/per/cpsw.h>
#if defined(SOC_AM273X) || defined(SOC_AWR294X) || defined (SOC_AWR2544) || defined(SOC_AM263X) || defined(SOC_AM263PX)
#include <priv/per/cpsw_cpdma_priv.h>
#else
#include <priv/per/cpsw_priv.h>
#endif

#if ENET_CFG_IS_ON(CPSW_INTERVLAN)

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t Cpsw_validateInterVlanEgressCfg(Cpsw_Handle hCpsw,
                                               const Cpsw_InterVlanEgressPortCfg *egressCfg);

static void Cpsw_setInterVlanAlePolicerParams(CpswAle_PolicerMatchParams *aleMatchPrms,
                                              const Cpsw_InterVlanRouteIngressPktMatchCfg *cpswMatchPrms);

static int32_t Cpsw_setInterVlanAleCfg(Cpsw_Handle hCpsw,
                                       uint32_t dstPortMask,
                                       CpswMacPort_InterVlanRouteId routeId,
                                       const Cpsw_InterVlanRouteIngressPktMatchCfg *pktMatchCfg,
                                       CpswAle_PolicerEntryOutArgs *outArgs);

static int32_t Cpsw_setInterVlanRouteUniEgress(Cpsw_Handle hCpsw,
                                               const Cpsw_SetInterVlanRouteUniEgressInArgs *inArgs,
                                               Cpsw_SetInterVlanRouteUniEgressOutArgs *outArgs);

static int32_t Cpsw_clearInterVlanRouteUniEgress(Cpsw_Handle hCpsw,
                                                 const Cpsw_ClearInterVlanRouteUniEgressInArgs *inArgs,
                                                 CpswMacPort_InterVlanRouteId *egressPortRouteId);

static int32_t Cpsw_findCommonFreeSlot(const Cpsw_Handle hCpsw,
                                       uint32_t numEgressPorts,
                                       const Cpsw_InterVlanEgressPortCfg *egressCfgList,
                                       CpswMacPort_InterVlanRouteId *freeRouteId);

static int32_t Cpsw_validateInterVlanMultiEgressConfig(const Cpsw_Handle hCpsw,
                                                       uint32_t numEgressPorts,
                                                       const Cpsw_InterVlanEgressPortCfg *egressCfgList);

static int32_t Cpsw_setInterVlanRouteMultiEgress(const Cpsw_Handle hCpsw,
                                                 const Cpsw_SetInterVlanRouteMultiEgressInArgs *inArgs,
                                                 Cpsw_SetInterVlanRouteMultiEgressOutArgs *outArgs);

static int32_t Cpsw_validateClearInterVlanRouteMultiEgress(const Cpsw_Handle hCpsw,
                                                           const Cpsw_ClearInterVlanRouteMultiEgressInArgs *inArgs);

static int32_t Cpsw_clearInterVlanAleCfg(Cpsw_Handle hCpsw,
                                         const Cpsw_InterVlanRouteIngressPktMatchCfg *pktMatchCfg,
                                         uint32_t delAleEntryMask);

static int32_t Cpsw_clearInterVlanRouteMultiEgress(Cpsw_Handle hCpsw,
                                                   const Cpsw_ClearInterVlanRouteMultiEgressInArgs *inArgs,
                                                   CpswMacPort_InterVlanRouteId *egressPortRouteId);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

#if ENET_CFG_IS_ON(CPSW_INTERVLAN)
int32_t Cpsw_ioctlInterVlan(EnetPer_Handle hPer,
                            uint32_t cmd,
                            Enet_IoctlPrms *prms)
{
    Cpsw_Handle hCpsw = (Cpsw_Handle)hPer;
    int32_t status = ENET_SOK;

    switch (cmd)
    {
        case CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_UNI_EGRESS:
        {
            const Cpsw_SetInterVlanRouteUniEgressInArgs *inArgs =
                (const Cpsw_SetInterVlanRouteUniEgressInArgs *)prms->inArgs;
            Cpsw_SetInterVlanRouteUniEgressOutArgs *outArgs =
                (Cpsw_SetInterVlanRouteUniEgressOutArgs *)prms->outArgs;

            status = Cpsw_setInterVlanRouteUniEgress(hCpsw, inArgs, outArgs);
            ENETTRACE_ERR_IF(status != ENET_SOK,
                             "Failed to set interVLAN route unicast egress: %d\n", status);
        }
        break;

        case CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_UNI_EGRESS:
        {
            const Cpsw_ClearInterVlanRouteUniEgressInArgs *inArgs =
                (const Cpsw_ClearInterVlanRouteUniEgressInArgs *)prms->inArgs;
            CpswMacPort_InterVlanRouteId *routeId =
                (CpswMacPort_InterVlanRouteId *)prms->outArgs;

            status = Cpsw_clearInterVlanRouteUniEgress(hCpsw, inArgs, routeId);
            ENETTRACE_ERR_IF(status != ENET_SOK,
                             "Failed to clear interVLAN route unicast egress: %d\n", status);
        }
        break;

        case CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_MULTI_EGRESS:
        {
            const Cpsw_SetInterVlanRouteMultiEgressInArgs *inArgs =
                (const Cpsw_SetInterVlanRouteMultiEgressInArgs *)prms->inArgs;
            Cpsw_SetInterVlanRouteMultiEgressOutArgs *outArgs =
                (Cpsw_SetInterVlanRouteMultiEgressOutArgs *)prms->outArgs;

            status = Cpsw_setInterVlanRouteMultiEgress(hCpsw, inArgs, outArgs);
            ENETTRACE_ERR_IF(status != ENET_SOK,
                             "Failed to set interVLAN route multicast egress: %d\n", status);
        }
        break;

        case CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_MULTI_EGRESS:
        {
            const Cpsw_ClearInterVlanRouteMultiEgressInArgs *inArgs =
                (const Cpsw_ClearInterVlanRouteMultiEgressInArgs *)prms->inArgs;
            CpswMacPort_InterVlanRouteId *routeId =
                (CpswMacPort_InterVlanRouteId *)prms->outArgs;

            status = Cpsw_clearInterVlanRouteMultiEgress(hCpsw, inArgs, routeId);
            ENETTRACE_ERR_IF(status != ENET_SOK,
                             "Failed to clear interVLAN route multicast egress: %d\n", status);
        }
        break;

        default:
        {
            status = ENET_EINVALIDPARAMS;
        }
        break;
    }

    return status;
}
#endif

static uint32_t Cpsw_normRouteId(CpswMacPort_InterVlanRouteId routeId)
{
    return (routeId - CPSW_MACPORT_INTERVLAN_ROUTEID_FIRST);
}

static int32_t Cpsw_validateInterVlanEgressCfg(Cpsw_Handle hCpsw,
                                               const Cpsw_InterVlanEgressPortCfg *egressCfg)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hCpsw;
    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hPer->virtAddr;
    int32_t status = ENET_SOK;

    if (egressCfg->outPktModCfg.forceUntaggedEgress == false)
    {
        /* If forceUntagged egress is false, it means we need to replace VID
         * Check CPSW is in vlan aware mdoe else VID replace will not work
         * in interVLan routing */
        if (CSL_CPSW_isVlanAwareEnabled(regs) == 0)
        {
            ENETTRACE_ERR("InterVLAN vlan Id replacement on egress requires "
                          "CpswCfg.vlanCfg.vlanAware to be true at open time\n");
            status = ENET_EINVALIDPARAMS;
        }
    }

    return status;
}

static void Cpsw_setInterVlanAlePolicerParams(CpswAle_PolicerMatchParams *aleMatchPrms,
                                              const Cpsw_InterVlanRouteIngressPktMatchCfg *cpswMatchPrms)
{
    aleMatchPrms->policerMatchEnMask = 0U;

    if (cpswMatchPrms->packetMatchEnMask & CPSW_INTERVLAN_INGRESSPKT_MATCH_PORT)
    {
        aleMatchPrms->policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_PORT;
        aleMatchPrms->portNum     = CPSW_ALE_MACPORT_TO_ALEPORT(cpswMatchPrms->ingressPort);
        aleMatchPrms->portIsTrunk = false;
    }

    if (cpswMatchPrms->packetMatchEnMask & CPSW_INTERVLAN_INGRESSPKT_MATCH_MACSRC)
    {
        aleMatchPrms->policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_MACSRC;
        aleMatchPrms->srcMacAddrInfo = cpswMatchPrms->srcMacAddrInfo;
    }

    if (cpswMatchPrms->packetMatchEnMask & CPSW_INTERVLAN_INGRESSPKT_MATCH_MACDST)
    {
        aleMatchPrms->policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_MACDST;
        aleMatchPrms->dstMacAddrInfo = cpswMatchPrms->dstMacAddrInfo;
    }

    if (cpswMatchPrms->packetMatchEnMask & CPSW_INTERVLAN_INGRESSPKT_MATCH_ETHERTYPE)
    {
        aleMatchPrms->policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_ETHERTYPE;
        aleMatchPrms->etherType = cpswMatchPrms->etherType;
    }

    if (cpswMatchPrms->packetMatchEnMask & CPSW_INTERVLAN_INGRESSPKT_MATCH_IPSRC)
    {
        aleMatchPrms->policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_IPSRC;
        aleMatchPrms->srcIpInfo = cpswMatchPrms->srcIpInfo;
    }

    if (cpswMatchPrms->packetMatchEnMask & CPSW_INTERVLAN_INGRESSPKT_MATCH_IPDST)
    {
        aleMatchPrms->policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_IPDST;
        aleMatchPrms->dstIpInfo = cpswMatchPrms->dstIpInfo;
    }

    /* Always set match params for ingress VLAN id */
    aleMatchPrms->policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_IVLAN;
    aleMatchPrms->ivlanId = cpswMatchPrms->vlanId;
}

static int32_t Cpsw_setInterVlanAleCfg(Cpsw_Handle hCpsw,
                                       uint32_t dstPortMask,
                                       CpswMacPort_InterVlanRouteId routeId,
                                       const Cpsw_InterVlanRouteIngressPktMatchCfg *pktMatchCfg,
                                       CpswAle_PolicerEntryOutArgs *outArgs)
{
    Enet_IoctlPrms prms;
    CpswAle_SetInterVlanCfgInArgs aleInterVlanCfgInArgs;
    int32_t status;

    aleInterVlanCfgInArgs.dstPortMask = dstPortMask;
    aleInterVlanCfgInArgs.routeIdx    = CPSW_MACPORT_INTERVLAN_ROUTE2PTR(routeId);
    /* TODO: Support for egress port that is operating as a trunk */
    aleInterVlanCfgInArgs.egressTrunkIdx = 0;
    aleInterVlanCfgInArgs.ttlCheckEn = pktMatchCfg->ttlCheckEn;

    Cpsw_setInterVlanAlePolicerParams(&aleInterVlanCfgInArgs.policerMatch, pktMatchCfg);

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &aleInterVlanCfgInArgs, outArgs);

    CPSW_ALE_PRIV_IOCTL(hCpsw->hAle, CPSW_ALE_IOCTL_SET_INTERVLAN_CFG, &prms, status);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set interVLAN ALE config: %d\n", status);

    return status;
}

static int32_t Cpsw_setInterVlanGlobalConfig(Cpsw_Handle hCpsw)
{
    CpswAle_PolicerGlobalCfg policerGlobalCfg;
    Enet_IoctlPrms prms;
    int32_t status;

    ENET_IOCTL_SET_OUT_ARGS(&prms, &policerGlobalCfg);

    CPSW_ALE_PRIV_IOCTL(hCpsw->hAle, CPSW_ALE_IOCTL_GET_POLICER_GLOBAL_CFG, &prms, status);
    if (status == ENET_SOK)
    {
        if (policerGlobalCfg.policingEn == false)
        {
            ENETTRACE_WARN("Intervlan requires ALE POLICECONTROL.POLICING_EN to be true."
                           "Enabling POLICECONTROL.POLICING_EN\n");
            policerGlobalCfg.policingEn = true;
            ENET_IOCTL_SET_IN_ARGS(&prms, &policerGlobalCfg);

            CPSW_ALE_PRIV_IOCTL(hCpsw->hAle, CPSW_ALE_IOCTL_SET_POLICER_GLOBAL_CFG, &prms, status);
            ENETTRACE_ERR_IF(status != ENET_SOK,
                             "Failed to set policer global config: %d\n", status);
        }
    }
    else
    {
        ENETTRACE_ERR("Failed to get policer global config: %d\n", status);
    }

    return status;
}

static int32_t Cpsw_setInterVlanRouteUniEgress(Cpsw_Handle hCpsw,
                                               const Cpsw_SetInterVlanRouteUniEgressInArgs *inArgs,
                                               Cpsw_SetInterVlanRouteUniEgressOutArgs *outArgs)
{
    CpswMacPort_InterVlanRouteId routeId;
    Enet_IoctlPrms prms;
    uint32_t portNum = ENET_MACPORT_NORM(inArgs->egressCfg.egressPort);
    int32_t status;
    Enet_devAssert(portNum < CPSW_MAC_PORT_NUM);
    EnetMod_Handle hMacPort = hCpsw->hMacPort[portNum];
    status = Cpsw_validateInterVlanEgressCfg(hCpsw, &inArgs->egressCfg);
    if (status == ENET_SOK)
    {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs->egressCfg.outPktModCfg, &routeId);

        CPSW_MACPORT_PRIV_IOCTL(hMacPort, CPSW_MACPORT_IOCTL_SET_INTERVLAN_ROUTE, &prms, status);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set interVLAN route: %d\n", status);
    }

    if (ENET_SOK == status)
    {
        uint32_t aleInterVlanDestPortMask;

        Enet_devAssert(routeId <= CPSW_MACPORT_INTERVLAN_ROUTEID_LAST,
                       "Invalid route id %u (expected < %u)\n",
                       routeId, CPSW_MACPORT_INTERVLAN_ROUTEID_LAST);

        outArgs->egressPortRouteId = routeId;
        aleInterVlanDestPortMask = CPSW_ALE_MACPORT_TO_PORTMASK(inArgs->egressCfg.egressPort);

        status = Cpsw_setInterVlanAleCfg(hCpsw,
                                         aleInterVlanDestPortMask,
                                         routeId,
                                         &inArgs->inPktMatchCfg,
                                         &outArgs->ingressPacketClassifierInfo);
        if (status != ENET_SOK)
        {
            CpswMacPort_InterVlanRouteId delRouteId;

            /* ALE IOCTL failed. Rollback the MAC Port interVLAN route allocation */
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs->egressCfg.outPktModCfg, &delRouteId);

            CPSW_MACPORT_PRIV_IOCTL(hMacPort, CPSW_MACPORT_IOCTL_DELETE_INTERVLAN_ROUTE, &prms, status);
            ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to delete interVLAN route: %d\n", status);

            Enet_devAssert(status == ENET_SOK, "Error unwinding failed: %d\n", status);
            Enet_assert(routeId == delRouteId,
                        "Unexpected route id %u deleted (expected %u)\n",  delRouteId, routeId);
        }
    }

    if (status == ENET_SOK)
    {
        status = Cpsw_setInterVlanGlobalConfig(hCpsw);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set interVLAN global config: %d\n", status);
    }

    return status;
}

static int32_t Cpsw_clearInterVlanRouteUniEgress(Cpsw_Handle hCpsw,
                                                 const Cpsw_ClearInterVlanRouteUniEgressInArgs *inArgs,
                                                 CpswMacPort_InterVlanRouteId *egressPortRouteId)
{
    Enet_IoctlPrms prms;
    uint32_t portNum = ENET_MACPORT_NORM(inArgs->egressCfg.egressPort);
    int32_t status;

    status = Cpsw_clearInterVlanAleCfg(hCpsw, &inArgs->inPktMatchCfg, inArgs->delAleEntryMask);

    if (ENET_SOK == status)
    {
        CpswMacPort_InterVlanRouteId routeId;

        ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs->egressCfg.outPktModCfg, &routeId);
        CPSW_MACPORT_PRIV_IOCTL(hCpsw->hMacPort[portNum], CPSW_MACPORT_IOCTL_DELETE_INTERVLAN_ROUTE, &prms, status);
        if (ENET_SOK == status)
        {
            *egressPortRouteId = routeId;
        }
        else
        {
            ENETTRACE_ERR("Failed to delete interVLAN route: %d\n", status);
        }
    }

    return status;
}

static int32_t Cpsw_findCommonFreeSlot(const Cpsw_Handle hCpsw,
                                       uint32_t numEgressPorts,
                                       const Cpsw_InterVlanEgressPortCfg *egressCfgList,
                                       CpswMacPort_InterVlanRouteId *freeRouteId)
{
    Enet_IoctlPrms prms;
    CpswMacPort_InterVlanRouteId routeIter;
    uint32_t freeRouteMask;
    uint32_t portNum;
    uint32_t i, j;
    int32_t status;

    if (numEgressPorts > 0U)
    {
        freeRouteMask = ENET_MK_ONES(Cpsw_normRouteId(CPSW_MACPORT_INTERVLAN_ROUTEID_LAST), 0U);
    }
    else
    {
        freeRouteMask = 0U;
    }

    for (i = 0U; i < numEgressPorts; i++)
    {
        CpswMacPort_InterVlanFreeRouteInfo freeRoute = {0};
        const Cpsw_InterVlanEgressPortCfg *curEgressCfg = &egressCfgList[i];
        uint32_t portFreeRouteMask;

        ENET_IOCTL_SET_OUT_ARGS(&prms, &freeRoute);

        portNum = ENET_MACPORT_NORM(curEgressCfg->egressPort);
        Enet_assert(portNum < hCpsw->macPortNum, "Invalid port number %u (expected < %u)\n", portNum, hCpsw->macPortNum);

        CPSW_MACPORT_PRIV_IOCTL(hCpsw->hMacPort[portNum],
                               CPSW_MACPORT_IOCTL_GET_INTERVLAN_FREEROUTES,
                               &prms,
                               status);
        ENETTRACE_ERR_IF(status != ENET_SOK, "failed to get interVLAN free routes: %d\n", status);

        portFreeRouteMask = 0U;
        for (j = 0U; j < freeRoute.numFreeRoutes; j++)
        {
            portFreeRouteMask |= ENET_BIT(Cpsw_normRouteId(freeRoute.freeRouteId[j]));
        }

        freeRouteMask &= portFreeRouteMask;
    }

    if (freeRouteMask)
    {
        status = ENET_SOK;
        for (routeIter = CPSW_MACPORT_INTERVLAN_ROUTEID_FIRST;
             routeIter <= CPSW_MACPORT_INTERVLAN_ROUTEID_LAST;
             routeIter = (CpswMacPort_InterVlanRouteId)(routeIter + 1))
        {
            if (freeRouteMask & ENET_BIT(Cpsw_normRouteId(routeIter)))
            {
                *freeRouteId = routeIter;
                break;
            }
        }

        /* loop to programtically validate free found common free slot across ports */
        for (i = 0U; i < numEgressPorts; i++)
        {
            CpswMacPort_InterVlanRouteId routeId = *freeRouteId;
            const Cpsw_InterVlanEgressPortCfg *curEgressCfg = &egressCfgList[i];
            bool isRouteFree;

            ENET_IOCTL_SET_INOUT_ARGS(&prms, &routeId, &isRouteFree);

            portNum = ENET_MACPORT_NORM(curEgressCfg->egressPort);

            Enet_assert(portNum < hCpsw->macPortNum);
            CPSW_MACPORT_PRIV_IOCTL(hCpsw->hMacPort[portNum],
                                   CPSW_MACPORT_IOCTL_IS_INTERVLAN_ROUTE_FREE,
                                   &prms,
                                   status);
            ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to check interVLAN route free state: %d\n", status);
            Enet_assert(status == ENET_SOK, "Failed to free routes: %d\n", status);
            Enet_assert(isRouteFree == true, "Route is not yet free\n");
        }
    }
    else
    {
        ENETTRACE_ERR("No free routes\n");
        status = ENET_EFAIL;
    }

    return status;
}

static int32_t Cpsw_validateInterVlanMultiEgressConfig(const Cpsw_Handle hCpsw,
                                                       uint32_t numEgressPorts,
                                                       const Cpsw_InterVlanEgressPortCfg *egressCfgList)
{
    int32_t status = ENET_SOK;
    uint32_t i;

    for (i = 0U; i < numEgressPorts; i++)
    {
        const Cpsw_InterVlanEgressPortCfg *curEgressCfg;

        curEgressCfg = &egressCfgList[i];
        status = Cpsw_validateInterVlanEgressCfg(hCpsw, curEgressCfg);
        if (status != ENET_SOK)
        {
            ENETTRACE_ERR("Failed to validate interVLAN egress config for port %u: %d\n", i, status);
            break;
        }
    }

    return status;
}

static int32_t Cpsw_setInterVlanRouteMultiEgress(const Cpsw_Handle hCpsw,
                                                 const Cpsw_SetInterVlanRouteMultiEgressInArgs *inArgs,
                                                 Cpsw_SetInterVlanRouteMultiEgressOutArgs *outArgs)
{
    CpswMacPort_InterVlanRouteId commonFreeRouteId = CPSW_MACPORT_INTERVLAN_ROUTEID_FIRST;
    Enet_IoctlPrms prms;
    uint32_t portNum;
    uint32_t i;
    int32_t status;

    status = Cpsw_validateInterVlanMultiEgressConfig(hCpsw, inArgs->numEgressPorts, inArgs->egressCfg);
    if (ENET_SOK == status)
    {
        status = Cpsw_findCommonFreeSlot(hCpsw,
                                         inArgs->numEgressPorts,
                                         inArgs->egressCfg,
                                         &commonFreeRouteId);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to find common free slot: %d\n", status);
    }

    if (ENET_SOK == status)
    {
        for (i = 0U; i < inArgs->numEgressPorts; i++)
        {
            CpswMacPort_SetSpecificInterVlanRouteInArgs portInArgs;

            portInArgs.routeId  = commonFreeRouteId;
            portInArgs.routeCfg = inArgs->egressCfg[i].outPktModCfg;
            ENET_IOCTL_SET_IN_ARGS(&prms, &portInArgs);

            portNum = ENET_MACPORT_NORM(inArgs->egressCfg[i].egressPort);
            Enet_assert(portNum < hCpsw->macPortNum);

            CPSW_MACPORT_PRIV_IOCTL(hCpsw->hMacPort[portNum],
                                    CPSW_MACPORT_IOCTL_SET_SPECIFIC_INTERVLAN_ROUTE,
                                    &prms,
                                    status);
            if (status != ENET_SOK)
            {
                ENETTRACE_ERR("Failed to set specific interVLAN route: %d\n");
                break;
            }
        }
    }

    if (ENET_SOK == status)
    {
        uint32_t aleInterVlanDestPortMask;

        Enet_assert(commonFreeRouteId <= CPSW_MACPORT_INTERVLAN_ROUTEID_LAST,
                    "Unexpected free route id %u (expected < %u)\n",
                    commonFreeRouteId, CPSW_MACPORT_INTERVLAN_ROUTEID_LAST);

        outArgs->egressPortRouteId = commonFreeRouteId;
        aleInterVlanDestPortMask = 0U;

        for (i = 0u; i < inArgs->numEgressPorts; i++)
        {
            aleInterVlanDestPortMask |= CPSW_ALE_MACPORT_TO_PORTMASK(inArgs->egressCfg[i].egressPort);
        }

        status = Cpsw_setInterVlanAleCfg(hCpsw,
                                         aleInterVlanDestPortMask,
                                         commonFreeRouteId,
                                         &inArgs->inPktMatchCfg,
                                         &outArgs->ingressPacketClassifierInfo);
        if (status != ENET_SOK)
        {
            /* ALE IOCTL failed. Rollback the MAC Port interVLAN route allocation */
            for (i = 0u; i < inArgs->numEgressPorts; i++)
            {
                CpswMacPort_InterVlanRouteId delRouteId;

                ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs->egressCfg[i].outPktModCfg, &delRouteId);

                portNum = ENET_MACPORT_NORM(inArgs->egressCfg[i].egressPort);
                Enet_assert(portNum < hCpsw->macPortNum);

                CPSW_MACPORT_PRIV_IOCTL(hCpsw->hMacPort[portNum],
                                       CPSW_MACPORT_IOCTL_DELETE_INTERVLAN_ROUTE,
                                       &prms,
                                       status);
                ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to delete interVLAN route: %d\n");
                Enet_devAssert(status == ENET_SOK, "Error unwinding failed: %d\n", status);
                Enet_assert(commonFreeRouteId == delRouteId,
                            "Unexpected route id %u deleted (expected %u)\n",
                            delRouteId, commonFreeRouteId);
            }
        }
    }

    if (ENET_SOK == status)
    {
        Cpsw_setInterVlanGlobalConfig(hCpsw);
    }

    return status;
}

static int32_t Cpsw_validateClearInterVlanRouteMultiEgress(const Cpsw_Handle hCpsw,
                                                           const Cpsw_ClearInterVlanRouteMultiEgressInArgs *inArgs)
{
    CpswMacPort_InterVlanRouteId routeId;
    CpswMacPort_InterVlanRouteId commonRouteId;
    Enet_IoctlPrms prms;
    bool validateFailed = false;
    uint32_t portNum;
    uint32_t i;
    int32_t status;

    for (i = 0U; ((i < inArgs->numEgressPorts) && (validateFailed == false)); i++)
    {
        portNum = ENET_MACPORT_NORM(inArgs->egressCfg[i].egressPort);
        Enet_assert(portNum < hCpsw->macPortNum);

        ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs->egressCfg[i].outPktModCfg, &routeId);
        CPSW_MACPORT_PRIV_IOCTL(hCpsw->hMacPort[portNum], CPSW_MACPORT_IOCTL_FIND_INTERVLAN_ROUTE, &prms, status);
        if (ENET_SOK == status)
        {
            if (0U == i)
            {
                commonRouteId = routeId;
            }
            else
            {
                if (routeId != commonRouteId)
                {
                    validateFailed = true;
                }
            }
        }
        else
        {
            ENETTRACE_ERR("Failed to find interVLAN route: %d\n", status);
            validateFailed = true;
        }
    }

    status = (validateFailed == true) ? ENET_EFAIL : ENET_SOK;

    return status;
}

static int32_t Cpsw_clearInterVlanAleCfg(Cpsw_Handle hCpsw,
                                         const Cpsw_InterVlanRouteIngressPktMatchCfg *pktMatchCfg,
                                         uint32_t delAleEntryMask)
{
    Enet_IoctlPrms prms;
    CpswAle_DelPolicerEntryInArgs delPolicerInArgs;
    int32_t status;

    Cpsw_setInterVlanAlePolicerParams(&delPolicerInArgs.policerMatch, pktMatchCfg);
    delPolicerInArgs.aleEntryMask = delAleEntryMask;
    ENET_IOCTL_SET_IN_ARGS(&prms, &delPolicerInArgs);

    CPSW_ALE_PRIV_IOCTL(hCpsw->hAle, CPSW_ALE_IOCTL_DEL_POLICER, &prms, status);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to clear interVLAN ALE config: %d\n", status);

    return status;
}

static int32_t Cpsw_clearInterVlanRouteMultiEgress(Cpsw_Handle hCpsw,
                                                   const Cpsw_ClearInterVlanRouteMultiEgressInArgs *inArgs,
                                                   CpswMacPort_InterVlanRouteId *egressPortRouteId)
{
    Enet_IoctlPrms prms;
    CpswMacPort_InterVlanRouteId routeId;
    uint32_t portNum;
    uint32_t i;
    int32_t status;

    status = Cpsw_validateClearInterVlanRouteMultiEgress(hCpsw, inArgs);
    if (ENET_SOK == status)
    {
        status = Cpsw_clearInterVlanAleCfg(hCpsw, &inArgs->inPktMatchCfg, inArgs->delAleEntryMask);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to clear interVLAN ALE config: %d\n", status);
    }

    if (ENET_SOK == status)
    {
        for (i = 0U; i < inArgs->numEgressPorts; i++)
        {
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs->egressCfg[i].outPktModCfg, &routeId);

            portNum = ENET_MACPORT_NORM(inArgs->egressCfg[i].egressPort);
            Enet_assert(portNum < hCpsw->macPortNum);

            CPSW_MACPORT_PRIV_IOCTL(hCpsw->hMacPort[portNum],
                                   CPSW_MACPORT_IOCTL_DELETE_INTERVLAN_ROUTE,
                                   &prms,
                                   status);
            if (status != ENET_SOK)
            {
                ENETTRACE_ERR("Failed to delete interVLAN route: %d\n", status);
                break;
            }
            else
            {
                *egressPortRouteId = routeId;
            }
        }
    }

    return status;
}
#endif
