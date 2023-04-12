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
 * \brief This file contains the implementation of the CPSW peripheral. This
 *        implementation supports CPSW_2G, CPSW_5G and CPSW_9G.
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
#include <priv/mod/cpsw_hostport_priv.h>
#include <priv/mod/cpsw_macport_priv.h>
#include <priv/mod/cpsw_macport_ioctl_priv.h>
#include <priv/mod/mdio_priv.h>
#include <priv/mod/cpsw_stats_priv.h>
#include <priv/core/enet_rm_priv.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_osal.h>
#include <include/core/enet_soc.h>
#include <include/core/enet_per.h>
#include <include/per/cpsw.h>
#include <priv/per/cpsw_priv.h>
#include <priv/per/enet_hostport_udma.h>
#include <priv/per/cpsw_ioctl_priv.h>
#include <include/core/enet_utils.h>
#include <include/common/enet_phymdio_dflt.h>
#include <include/phy/enetphy.h>
#include <networking/enet/core/src/per/cpsw_intervlan.h>
#include <priv/core/enet_rm_ioctl_priv.h>
#include <priv/per/cpsw_est_ioctl_priv.h>
#include <priv/mod/cpsw_hostport_ioctl_priv.h>

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
static int32_t Cpsw_openPortLinkWithPhy(Cpsw_Handle hCpsw,
                                        Enet_MacPort macPort,
                                        const CpswMacPort_Cfg *macCfg,
                                        const EnetPhy_Cfg *phyCfg,
                                        const EnetMacPort_Interface *mii,
                                        const EnetMacPort_LinkCfg *linkCfg);

static int32_t Cpsw_openPortLinkNoPhy(Cpsw_Handle hCpsw,
                                      Enet_MacPort macPort,
                                      const CpswMacPort_Cfg *macCfg,
                                      const EnetMacPort_Interface *mii,
                                      const EnetMacPort_LinkCfg *linkCfg);

static int32_t Cpsw_openPortLink(Cpsw_Handle hCpsw,
                                 Enet_MacPort macPort,
                                 const CpswMacPort_Cfg *macCfg,
                                 const EnetPhy_Cfg *phyCfg,
                                 const EnetMacPort_Interface *mii,
                                 const EnetMacPort_LinkCfg *linkCfg);

static void Cpsw_closePortLink(Cpsw_Handle hCpsw,
                               Enet_MacPort macPort);

static int32_t Cpsw_getPortLinkCfg(Cpsw_Handle hCpsw,
                                   Enet_MacPort macPort,
                                   EnetMacPort_LinkCfg *linkCfg);

static uint32_t Cpsw_getTxMtuPerPriority(Cpsw_Handle hCpsw,
                                         uint32_t priority);

static uint32_t Cpsw_getRxMtuPort0(Cpsw_Handle hCpsw);

static int32_t Cpsw_setTxShortIpgCfg(const Cpsw_Handle hCpsw,
                                     const Cpsw_SetTxShortIpgCfgInArgs *inArgs);

static int32_t Cpsw_validateTxShortIpgCfg(const Cpsw_Handle hCpsw,
                                          const Cpsw_SetTxShortIpgCfgInArgs *inArgs);

static int32_t Cpsw_getTxShortIpgCfg(const Cpsw_Handle hCpsw,
                                     Cpsw_TxShortIpgCfg *shortIpgCfg);

static int32_t Cpsw_validateDfltFlow(Cpsw_Handle hCpsw,
                                     Enet_DfltFlowInfo *dfltFlowInfo,
                                     uint32_t flowId);

static int32_t Cpsw_validateFlowId(Cpsw_Handle hCpsw,
                                   uint32_t coreKey,
                                   uint32_t startIdx,
                                   uint32_t flowIdx);

static int32_t Cpsw_handleExternalPhyLinkUp(Cpsw_Handle hCpsw,
                                             Enet_MacPort macPort,
                                             const EnetPhy_LinkCfg *phyLinkCfg);

#if ENET_CFG_IS_ON(CPSW_SGMII)
static uint32_t Cpsw_mapPort2XgmiiId(Enet_MacPort macPort);

static int32_t Cpsw_setSgmiiMode(Cpsw_Handle hCpsw,
                                 Enet_MacPort macPort,
                                 const EnetMacPort_Interface *mii,
                                 const CpswMacPort_Cfg *macCfg);
#endif


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_GET_VERSION(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    Enet_Version *version = (Enet_Version *)prms->outArgs;
    CSL_CPSW_VERSION ver = {0};
    int32_t status = ENET_SOK;

    CSL_CPSW_getCpswVersionInfo(regs, &ver);
    version->maj = ver.majorVer;
    version->min = ver.minorVer;
    version->rtl = ver.rtlVer;
    version->id  = ver.id;
    version->other1 = ENET_VERSION_NONE;
    version->other2 = ENET_VERSION_NONE;
    return status;
}

int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_PRINT_REGS(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    /* TODO: Print CPSW SS + CPSW NU registers */
    status = ENET_ENOTSUPPORTED;
    return status;
}

int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_OPEN_PORT_LINK(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    const EnetPer_PortLinkCfg *portLinkCfg = (const EnetPer_PortLinkCfg *)prms->inArgs;
    Enet_MacPort macPort = portLinkCfg->macPort;
    const CpswMacPort_Cfg *macCfg = (CpswMacPort_Cfg *)portLinkCfg->macCfg;
    const EnetPhy_Cfg *phyCfg = &portLinkCfg->phyCfg;
    const EnetMacPort_Interface *mii = &portLinkCfg->mii;
    const EnetMacPort_LinkCfg *linkCfg = &portLinkCfg->linkCfg;
    int32_t status = ENET_SOK;

    if (macCfg->rxMtu > hCpsw->maxPerPrioMtu)
    {
        /* Host port and MAC port MTU should not be greater than largest of the port
         * egress per priority MTU as packet would get dropped by Switch.
         * Though it is valid from HW configuration, we return an error as it serves
         * no purpose */
        ENETTRACE_ERR("Port %u: RX MTU (%d) exceeds max of TX Priority 0-7 MTU (%u)\r\n ",
                      ENET_MACPORT_ID(macPort), macCfg->rxMtu, hCpsw->maxPerPrioMtu);
        status = ENET_EINVALIDPARAMS;
    }
    else
    {
        status = Cpsw_openPortLink(hCpsw, macPort, macCfg, phyCfg, mii, linkCfg);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "Port %u: Failed to open port link: %d\r\n",
                         ENET_MACPORT_ID(macPort), status);
    }
    return status;
}

int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_CLOSE_PORT_LINK(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;
    int32_t status = ENET_SOK;

    Cpsw_closePortLink(hCpsw, macPort);
    return status;
}

int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_IS_PORT_LINK_UP(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;
    bool *linkUp = (bool *)prms->outArgs;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    /* Assert if port number is not correct */
    Enet_assert(portNum < EnetSoc_getMacPortMax(hCpsw->enetPer.enetType, hCpsw->enetPer.instId),
                "Invalid Port Id: %u\r\n", portNum);

    *linkUp = hCpsw->portLinkState[portNum].isLinkUp;
    return status;
}

int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_GET_PORT_LINK_CFG(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    Enet_MacPort macPort = *(Enet_MacPort *)prms->inArgs;
    EnetMacPort_LinkCfg *linkCfg = (EnetMacPort_LinkCfg *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = Cpsw_getPortLinkCfg(hCpsw, macPort, linkCfg);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "Port %u: Failed to get link config: %d\r\n",
                     ENET_MACPORT_ID(macPort), status);
    return status;
}

int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_ATTACH_CORE(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    uint32_t coreId = *(uint32_t *)prms->inArgs;
    EnetPer_AttachCoreOutArgs *outArgs = (EnetPer_AttachCoreOutArgs *)prms->outArgs;
    Enet_IoctlPrms rmPrms;
    uint32_t i;
    int32_t status = ENET_SOK;

    ENET_IOCTL_SET_INOUT_ARGS(&rmPrms, &coreId, &outArgs->coreKey);

    ENET_RM_PRIV_IOCTL(hCpsw->hRm, ENET_RM_IOCTL_ATTACH, &rmPrms, status);
    if (status == ENET_SOK)
    {
        /* Get MTU values */
        for (i = 0U; i < ENET_PRI_NUM; i++)
        {
            outArgs->txMtu[i] = Cpsw_getTxMtuPerPriority(hCpsw, i);
        }

        outArgs->rxMtu = Cpsw_getRxMtuPort0(hCpsw);
    }
    else
    {
        ENETTRACE_ERR("Failed to attach core %u: %d\r\n", coreId, status);
    }
    return status;
}

int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_DETACH_CORE(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    uint32_t coreKey = *((uint32_t *)prms->inArgs);
    Enet_IoctlPrms rmPrms;
    int32_t status = ENET_SOK;

    ENET_IOCTL_SET_IN_ARGS(&rmPrms, &coreKey);

    ENET_RM_PRIV_IOCTL(hCpsw->hRm, ENET_RM_IOCTL_DETACH, &rmPrms, status);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to detach core: %d\r\n", status);
    return status;
}

static int32_t Cpsw_internalIoctl_handler_interVlan(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, uint32_t cmd, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

#if ENET_CFG_IS_ON(CPSW_INTERVLAN)
#if ENET_CFG_IS_OFF(CPSW_MACPORT_INTERVLAN)
#error "CPSW interVLAN feature requires ENET_CFG_CPSW_MACPORT_INTERVLAN"
#endif
    if (ENET_FEAT_IS_EN(hCpsw->enetPer.features, CPSW_FEATURE_INTERVLAN))
    {
        EnetPer_Handle hPer = (EnetPer_Handle) hCpsw;

        status = Cpsw_ioctlInterVlan(hPer, cmd, prms);
    }
    else
    {
        status = ENET_ENOTSUPPORTED;
    }
#else
    status = ENET_ENOTSUPPORTED;
#endif
    return status;
}

int32_t Cpsw_internalIoctl_handler_CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_UNI_EGRESS(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status;

    status = Cpsw_internalIoctl_handler_interVlan(hCpsw, regs,  CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_UNI_EGRESS, prms);
    return status;
}

int32_t Cpsw_internalIoctl_handler_CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_UNI_EGRESS(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status;

    status = Cpsw_internalIoctl_handler_interVlan(hCpsw, regs, CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_UNI_EGRESS, prms);
    return status;
}

int32_t Cpsw_internalIoctl_handler_CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_MULTI_EGRESS(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status;

    status = Cpsw_internalIoctl_handler_interVlan(hCpsw, regs, CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_MULTI_EGRESS, prms);
    return status;
}

int32_t Cpsw_internalIoctl_handler_CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_MULTI_EGRESS(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status;

    status = Cpsw_internalIoctl_handler_interVlan(hCpsw, regs, CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_MULTI_EGRESS, prms);
    return status;
}

int32_t Cpsw_internalIoctl_handler_CPSW_PER_IOCTL_SET_SHORT_IPG_CFG(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    const Cpsw_SetTxShortIpgCfgInArgs *inArgs = (const Cpsw_SetTxShortIpgCfgInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = Cpsw_setTxShortIpgCfg(hCpsw, inArgs);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set TX short IPG config: %d\r\n", status);
    return status;
}

int32_t Cpsw_internalIoctl_handler_CPSW_PER_IOCTL_GET_SHORT_IPG_CFG(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    Cpsw_TxShortIpgCfg *shortIpgCfg = (Cpsw_TxShortIpgCfg *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = Cpsw_getTxShortIpgCfg(hCpsw, shortIpgCfg);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get TX short IPG config: %d\r\n", status);
    return status;
}

int32_t Cpsw_internalIoctl_handler_ENET_IOCTL_REGISTER_RX_DEFAULT_FLOW(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    Enet_DfltFlowInfo *dfltFlowInfo = (Enet_DfltFlowInfo *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = Cpsw_validateDfltFlow(hCpsw, dfltFlowInfo, hCpsw->rsvdFlowId);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                       "Invalid Default Flow: %d\r\n", dfltFlowInfo->flowIdx);
    if (status == ENET_SOK)
    {
        status = Cpsw_setDfltThreadCfg(hCpsw, dfltFlowInfo->flowIdx);
    }
    return status;
}

int32_t Cpsw_internalIoctl_handler_ENET_IOCTL_UNREGISTER_RX_DEFAULT_FLOW(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    Enet_DfltFlowInfo *dfltFlowInfo = (Enet_DfltFlowInfo *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = Cpsw_validateDfltFlow(hCpsw, dfltFlowInfo, dfltFlowInfo->flowIdx);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                       "Invalid Default Flow: %d\r\n", dfltFlowInfo->flowIdx);
    if (status == ENET_SOK)
    {
        status = Cpsw_setDfltThreadCfg(hCpsw, hCpsw->rsvdFlowId);
    }
    return status;
}

int32_t Cpsw_internalIoctl_handler_ENET_IOCTL_REGISTER_DSTMAC_RX_FLOW(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    Enet_MacDstFlowInfo *flowInfo = (Enet_MacDstFlowInfo *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = Cpsw_validateFlowId(hCpsw, flowInfo->coreKey, flowInfo->startIdx, flowInfo->flowIdx);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                 "Invalid Flow: %d\r\n", flowInfo->flowIdx);
    if (status == ENET_SOK)
    {
        Enet_IoctlPrms alePrms;
        CpswAle_SetPolicerEntryOutArgs poliOutArgs;
        CpswAle_SetPolicerEntryInArgs poliInArgs;

        memset(&poliInArgs, 0, sizeof(poliInArgs));

        poliInArgs.policerMatch.policerMatchEnMask = CPSW_ALE_POLICER_MATCH_MACDST;
        memcpy(&poliInArgs.policerMatch.dstMacAddrInfo.addr.addr[0U], flowInfo->macAddress,
               sizeof(poliInArgs.policerMatch.dstMacAddrInfo.addr.addr));
        poliInArgs.policerMatch.dstMacAddrInfo.addr.vlanId = 0;
        poliInArgs.policerMatch.dstMacAddrInfo.portNum = CPSW_ALE_HOST_PORT_NUM;
        poliInArgs.threadIdEn             = true;
        poliInArgs.threadId               = flowInfo->flowIdx;
        poliInArgs.peakRateInBitsPerSec   = 0;
        poliInArgs.commitRateInBitsPerSec = 0;

        ENET_IOCTL_SET_INOUT_ARGS(&alePrms, &poliInArgs, &poliOutArgs);

        CPSW_ALE_PRIV_IOCTL(hCpsw->hAle, CPSW_ALE_IOCTL_SET_POLICER,
                            &alePrms, status);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                           "CPSW_ALE_IOCTL_SET_POLICER failed: %d\r\n", status);
        ENETTRACE_INFO_IF((status == ENET_SOK),
                          "CPSW: Registered MAC address.ALE entry:%u, Policer Entry:%u",
                          poliOutArgs.dstMacAleEntryIdx, poliOutArgs.policerEntryIdx);
    }
    return status;
}

int32_t Cpsw_internalIoctl_handler_ENET_IOCTL_UNREGISTER_DSTMAC_RX_FLOW(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    Enet_IoctlPrms alePrms;
    CpswAle_DelPolicerEntryInArgs delPolicerInArgs;
    CpswAle_PolicerMatchParams policerMatchPrms;
    CpswAle_PolicerEntryOutArgs policerEntryOutArgs;
    Enet_MacDstFlowInfo *flowInfo = (Enet_MacDstFlowInfo *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = Cpsw_validateFlowId(hCpsw, flowInfo->coreKey, flowInfo->startIdx, flowInfo->flowIdx);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                       "Invalid Flow: %d\r\n", flowInfo->flowIdx);

    if (status == ENET_SOK)
    {
        memset(&policerMatchPrms, 0, sizeof(policerMatchPrms));
        policerMatchPrms.policerMatchEnMask = CPSW_ALE_POLICER_MATCH_MACDST;
        memcpy(&policerMatchPrms.dstMacAddrInfo.addr.addr[0U], flowInfo->macAddress,
               sizeof(policerMatchPrms.dstMacAddrInfo.addr.addr));
        policerMatchPrms.dstMacAddrInfo.addr.vlanId   = 0;
        policerMatchPrms.dstMacAddrInfo.portNum = CPSW_ALE_HOST_PORT_NUM;

        ENET_IOCTL_SET_INOUT_ARGS(&alePrms, &policerMatchPrms, &policerEntryOutArgs);

        CPSW_ALE_PRIV_IOCTL(hCpsw->hAle, CPSW_ALE_IOCTL_GET_POLICER,
                            &alePrms, status);
        if (status == ENET_SOK)
        {
            if ((policerEntryOutArgs.threadIdEn == true) &&
                (policerEntryOutArgs.threadId == flowInfo->flowIdx))
            {
                status = ENET_SOK;
            }
            else
            {
                status = ENET_EINVALIDPARAMS;
            }
        }
    }

    if (status == ENET_SOK)
    {
        memset(&delPolicerInArgs, 0, sizeof(delPolicerInArgs));

        delPolicerInArgs.policerMatch    = policerMatchPrms;
        delPolicerInArgs.aleEntryMask = CPSW_ALE_POLICER_TABLEENTRY_DELETE_MACDST;

        ENET_IOCTL_SET_IN_ARGS(&alePrms, &delPolicerInArgs);

        CPSW_ALE_PRIV_IOCTL(hCpsw->hAle, CPSW_ALE_IOCTL_DEL_POLICER,
                            &alePrms, status);
        ENETTRACE_ERR_IF((status != ENET_SOK),
                           "Invalid Flow: %d\r\n", flowInfo->flowIdx);
    }
    return status;
}

int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_HANDLE_EXTPHY_LINKUP_EVENT(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    Enet_ExtPhyLinkUpEventInfo *linkInfo = (Enet_ExtPhyLinkUpEventInfo *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = Cpsw_handleExternalPhyLinkUp(hCpsw, linkInfo->macPort, &linkInfo->phyLinkCfg);
    ENETTRACE_ERR_IF((status != ENET_SOK),
                     "%s: Link Up Event Handling failed: %d\r\n",
                     ENET_PER_NAME(hCpsw), status);
    return status;
}

int32_t Cpsw_internalIoctl_handler_ENET_PER_IOCTL_HANDLE_EXTPHY_LINKDOWN_EVENT(Cpsw_Handle hCpsw, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    Enet_MacPort macPort = *((Enet_MacPort *)prms->inArgs);
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    Enet_devAssert(portNum < CPSW_MAC_PORT_NUM);
    status = Cpsw_handleLinkDown(hCpsw, macPort);
    if (ENET_SOK == status)
    {
        hCpsw->portLinkState[portNum].isLinkUp = false;
    }
    ENETTRACE_ERR_IF((status != ENET_SOK),
                     "%s: Link Down Event Handling failed: %d\r\n",
                     ENET_PER_NAME(hCpsw), status);
    return status;
}


static int32_t Cpsw_openPortLinkWithPhy(Cpsw_Handle hCpsw,
                                        Enet_MacPort macPort,
                                        const CpswMacPort_Cfg *macCfg,
                                        const EnetPhy_Cfg *phyCfg,
                                        const EnetMacPort_Interface *mii,
                                        const EnetMacPort_LinkCfg *linkCfg)
{

    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    EnetPhy_MdioHandle hPhyMdio = EnetPhyMdioDflt_getPhyMdio();
    EnetPhy_Mii phyMii;
    EnetPhy_LinkCfg phyLinkCfg;
    CpswMacPort_ModCfg macModCfg;
    uint32_t macPortCaps;
    int32_t status;
    EnetMod_Handle hMacPort;

    /* Assert if port number is not correct */
    Enet_assert(portNum < EnetSoc_getMacPortMax(hCpsw->enetPer.enetType, hCpsw->enetPer.instId),
                "Invalid Port Id: %u\r\n", portNum);
    hMacPort = hCpsw->hMacPort[portNum];

    ENETTRACE_VAR(portId);
    /* Enet module takes a single config structure, which in CPSW MAC port case is
     * EnetMacPort_ModCfg. This structure contains the initial MAC specific configuration
     * and also the link configuration (speed/duplexity) and MII interface type. */
    macModCfg.macCfg = *macCfg;
    macModCfg.mii = *mii;
    macModCfg.linkCfg = *linkCfg;

    /* Open MAC port */
    status = EnetMod_open(hMacPort, hCpsw->enetPer.enetType, hCpsw->enetPer.instId, &macModCfg, sizeof(macModCfg));
    ENETTRACE_ERR_IF(status != ENET_SOK, "Port %u: Failed to open MAC: %d\r\n", portId, status);

    /* Open PHY */
    if (status == ENET_SOK)
    {
        if (hCpsw->disablePhyDriver != true)
        {
	        /* Convert MII and link configuration from Enet to ENETPHY types */
	        phyMii = EnetUtils_macToPhyMii(mii);
	        phyLinkCfg.speed = (EnetPhy_Speed)linkCfg->speed;
	        phyLinkCfg.duplexity = (EnetPhy_Duplexity)linkCfg->duplexity;

	        /* Get MAC port capabilities from SoC standpoint */
            macPortCaps = EnetSoc_getMacPortCaps(hCpsw->enetPer.enetType, hCpsw->enetPer.instId, macPort);

	        /* Open ENETPHY driver */
	        hCpsw->hPhy[portNum] = EnetPhy_open(phyCfg, phyMii, &phyLinkCfg, macPortCaps, hPhyMdio, hCpsw->hMdio);
	        if (hCpsw->hPhy[portNum] == NULL)
	        {
	            ENETTRACE_ERR("Port %u: Failed to open PHY\r\n", portId);
	            CpswMacPort_close(hMacPort);
            }
        }
        else
        {
            hCpsw->hPhy[portNum] = NULL;
        }
    }

    return status;
}

static int32_t Cpsw_openPortLinkNoPhy(Cpsw_Handle hCpsw,
                                      Enet_MacPort macPort,
                                      const CpswMacPort_Cfg *macCfg,
                                      const EnetMacPort_Interface *mii,
                                      const EnetMacPort_LinkCfg *linkCfg)
{

    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    CpswMacPort_ModCfg macModCfg;
    EnetMacPort_LinkCfg *macLinkCfg = &macModCfg.linkCfg;
    Enet_IoctlPrms prms;
    CpswAle_SetPortStateInArgs setPortStateInArgs;
    int32_t status = ENET_SOK;
    EnetMod_Handle hMacPort;

    /* Assert if port number is not correct */
    Enet_assert(portNum < EnetSoc_getMacPortMax(hCpsw->enetPer.enetType, hCpsw->enetPer.instId),
                "Invalid Port Id: %u\r\n", portNum);
    hMacPort = hCpsw->hMacPort[portNum];

    ENETTRACE_VAR(portId);
    hCpsw->hPhy[portNum] = NULL;

    /* Enet module takes a single config structure, which in CPSW MAC port case is
     * EnetMacPort_ModCfg. This structure contains the initial MAC specific configuration
     * and also the link configuration (speed/duplexity) and MII interface type. */
    macModCfg.macCfg = *macCfg;
    macModCfg.mii = *mii;
    macModCfg.linkCfg = *linkCfg;

    /* Explicit speed and duplexity must be provided, can't be discovered */
    if ((macLinkCfg->speed == ENET_SPEED_AUTO) ||
        (macLinkCfg->duplexity == ENET_DUPLEX_AUTO))
    {
        ENETTRACE_ERR("Port %u: Auto-speed/duplexity is not valid for PHY-less links\r\n", portId);
        status = ENET_EINVALIDPARAMS;
    }

    /* Open MAC port in force mode */
    if (status == ENET_SOK)
    {
        if (EnetMacPort_isRgmii(mii))
        {
            if ((macCfg->loopbackEn == true) &&
                (macLinkCfg->speed != ENET_SPEED_1GBIT))
            {
                /* PDK-4633 - RGMII loopback fails in 100Mbps mode  */
                ENETTRACE_WARN("Port %u: RGMII loopback in 1G mode not supported, "
                               "overriding speed to 1Gbps\r\n", portId);
                macLinkCfg->speed = ENET_SPEED_1GBIT;
            }
        }

        status = EnetMod_open(hMacPort, hCpsw->enetPer.enetType, hCpsw->enetPer.instId, &macModCfg, sizeof(macModCfg));
        ENETTRACE_ERR_IF(status != ENET_SOK, "Port %u: Failed to open MAC: %d\r\n", portId, status);
    }

    /* Enable MAC port with requested speed/duplexity */
    if (status == ENET_SOK)
    {
        ENET_IOCTL_SET_IN_ARGS(&prms, macLinkCfg);
        CPSW_MACPORT_PRIV_IOCTL(hMacPort, CPSW_MACPORT_IOCTL_ENABLE, &prms,status);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Port %u: Failed to enable MAC port: %d\r\n", portId, status);
    }

    /* Set ALE port in forward state */
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

    if (status != ENET_SOK)
    {
        Cpsw_closePortLink(hCpsw, macPort);
    }

    return status;
}

static int32_t Cpsw_openPortLink(Cpsw_Handle hCpsw,
                                 Enet_MacPort macPort,
                                 const CpswMacPort_Cfg *macCfg,
                                 const EnetPhy_Cfg *phyCfg,
                                 const EnetMacPort_Interface *mii,
                                 const EnetMacPort_LinkCfg *linkCfg)
{

    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    EnetMod_Handle hMacPort;
    EnetPhy_Handle hPhy;
    Cpsw_PortLinkState *portLinkState;
    uint32_t pollEnMask = hCpsw->mdioLinkIntCtx.pollEnableMask;
    uint32_t phyAddr = phyCfg->phyAddr;
    int32_t status = ENET_SOK;

    /* Assert if port number is not correct */
    Enet_assert(portNum < EnetSoc_getMacPortMax(hCpsw->enetPer.enetType, hCpsw->enetPer.instId),
                "Invalid Port Id: %u\r\n", portNum);
    hMacPort = hCpsw->hMacPort[portNum];
    hPhy = hCpsw->hPhy[portNum];
    portLinkState = &hCpsw->portLinkState[portNum];

    ENETTRACE_VAR(portId);
    if ((portNum >= EnetSoc_getMacPortMax(hCpsw->enetPer.enetType, hCpsw->enetPer.instId)) ||
        EnetMod_isOpen(hMacPort) ||
        (hPhy != NULL))
    {
        ENETTRACE_ERR("Port %u: Failed to open port (MAC port %s, PHY %s)\r\n",
                      portId,
                      EnetMod_isOpen(hMacPort) ? "open" : "closed",
                      (hPhy != NULL) ? "open" : "closed");
        status = ENET_EINVALIDPARAMS;
    }

#if ENET_CFG_IS_ON(CPSW_SGMII)
    if (status == ENET_SOK)
    {
        if (EnetMacPort_isSgmii(mii) ||
            EnetMacPort_isQsgmii(mii))
        {
            /* Configure SGMII mode in CPSW SS */
            status = Cpsw_setSgmiiMode(hCpsw, macPort, mii, macCfg);

            /* Make sure the QSGMII ports are configured correctly */
            if (EnetMacPort_isQsgmii(mii))
            {
                status += EnetSoc_validateQsgmiiCfg(hPer->enetType, hPer->instId);
            }
        }

        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "Invalid Q/SGMII port %u configuration: %d\r\n", portId, status);
    }
#endif

    if (ENET_SOK == status)
    {
        if (phyAddr == ENETPHY_INVALID_PHYADDR)
        {
            status = Cpsw_openPortLinkNoPhy(hCpsw, macPort, macCfg, mii, linkCfg);
        }
        else
        {
            status = Cpsw_openPortLinkWithPhy(hCpsw, macPort, macCfg, phyCfg, mii, linkCfg);
        }
    }

    if (status == ENET_SOK)
    {
        portLinkState->isOpen  = true;
        portLinkState->phyAddr = phyAddr;

        if ((hCpsw->hMdioIntr != NULL) &&
            ENETPHY_IS_ADDR_VALID(phyAddr))
        {
            portLinkState->isPollEnabled = ENET_MDIO_IS_PHY_ADDR_SET(pollEnMask, phyAddr);
        }
    }

    return status;
}

static void Cpsw_closePortLink(Cpsw_Handle hCpsw,
                               Enet_MacPort macPort)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    EnetMod_Handle hMacPort;
    EnetPhy_Handle hPhy;
    bool linked;

    if (portNum < EnetSoc_getMacPortMax(hCpsw->enetPer.enetType, hCpsw->enetPer.instId))
    {
        hMacPort = hCpsw->hMacPort[portNum];
        hPhy = hCpsw->hPhy[portNum];
        if (hPhy == NULL)
        {
            /* Link is always true for no-PHY mode */
            linked = true;
        }
        else
        {
            linked = hCpsw->portLinkState[portNum].isLinkUp;
        }

        /* Force link-down handling to undo settings done for this port link
         * i.e. ALE port state, MAC port disable, etc */
        if (linked)
        {
            Cpsw_handleLinkDown(hCpsw, macPort);
        }

        if (EnetMod_isOpen(hMacPort))
        {
            EnetMod_close(hMacPort);
        }

        if (hPhy != NULL)
        {
            EnetPhy_close(hPhy);
            hCpsw->hPhy[portNum] = NULL;
        }

        hCpsw->portLinkState[portNum].isOpen        = false;
        hCpsw->portLinkState[portNum].isPollEnabled = false;
        hCpsw->portLinkState[portNum].isTickEnabled = true;
    }
    else
    {
        /* Assert if port number is not correct */
        Enet_assert(false);
    }
}

static int32_t Cpsw_getPortLinkCfg(Cpsw_Handle hCpsw,
                                   Enet_MacPort macPort,
                                   EnetMacPort_LinkCfg *linkCfg)
{

    EnetMac_LayerType enetLayer;
    EnetMac_SublayerType enetSublayer;
    EnetMacPort_Interface mii;
    CSL_Xge_cpsw_ss_sRegs *ssRegs = (CSL_Xge_cpsw_ss_sRegs *)hCpsw->enetPer.virtAddr2;
    CSL_CPSW_SS_RGMIISTATUS rgmiiStatus;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    int32_t status;

    ENETTRACE_VAR(portId);
    status = EnetSoc_getMacPortMii(hCpsw->enetPer.enetType, hCpsw->enetPer.instId, macPort, &mii);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Port %u: Failed to get ENET_CTRL: %d\r\n", portId, status);

    if (status == ENET_SOK)
    {
        enetLayer = mii.layerType;
        enetSublayer = mii.sublayerType;

        /* Link configuration is only available for RGMII */
        if ((enetLayer == ENET_MAC_LAYER_GMII) &&
            (enetSublayer == ENET_MAC_SUBLAYER_REDUCED))
        {
            CSL_CPSW_SS_getRGMIIStatus(ssRegs, portNum, &rgmiiStatus);

            /* Get link speed */
            if (rgmiiStatus.speed == 0U)
            {
                linkCfg->speed = ENET_SPEED_10MBIT;
            }
            else if (rgmiiStatus.speed == 1U)
            {
                linkCfg->speed = ENET_SPEED_100MBIT;
            }
            else if (rgmiiStatus.speed == 2U)
            {
                linkCfg->speed = ENET_SPEED_1GBIT;
            }
            else
            {
                ENETTRACE_ERR("Port %u: invalid RGMII speed: %d\r\n", portId, rgmiiStatus.speed);
                Enet_assert(false, "Invalid RGMII speed value: %d\r\n", rgmiiStatus.speed);
            }

            /* Get link duplexity */
            if (rgmiiStatus.fullDuplex == 0U)
            {
                linkCfg->duplexity = ENET_DUPLEX_HALF;
            }
            else
            {
                linkCfg->duplexity = ENET_DUPLEX_FULL;
            }
        }
        else
        {
            ENETTRACE_ERR("Port %u: Link config not available for layer %u sublayer %u\r\n",
                          portId, enetLayer, enetSublayer);
            status = ENET_ENOTSUPPORTED;
        }
    }
    else
    {
        ENETTRACE_ERR("Port %u: Failed to get ENET_CTRL %u: %d\r\n", portId, status);
    }

    return status;
}

static uint32_t Cpsw_getTxMtuPerPriority(Cpsw_Handle hCpsw,
                                         uint32_t priority)
{
    uint32_t txMtu = 0U;

    if (hCpsw != NULL)
    {
        CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hCpsw->enetPer.virtAddr;

        txMtu = CSL_CPSW_getTxMaxLenPerPriority(regs, priority);
    }

    return txMtu;
}

static uint32_t Cpsw_getRxMtuPort0(Cpsw_Handle hCpsw)
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    EnetPort_MaxLen maxLen;

    maxLen.mru = 0U;
    ENET_IOCTL_SET_IN_ARGS(&prms, &maxLen);
    CPSW_HOSTPORT_PRIV_IOCTL(hCpsw->hHostPort,
                             ENET_HOSTPORT_IOCTL_GET_MAXLEN,
                             &prms,
                             status);
    if (status != ENET_SOK)
    {
        ENETTRACE_ERR("failed to get host port's RX MTU: %d\r\n", status);
    }

    return maxLen.mru;
}

static int32_t Cpsw_setTxShortIpgCfg(const Cpsw_Handle hCpsw,
                                     const Cpsw_SetTxShortIpgCfgInArgs *inArgs)
{

    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hCpsw->enetPer.virtAddr;
    const CpswMacPort_PortTxShortIpgCfg *portShortIpgCfg;
    Enet_IoctlPrms prms;
    uint32_t portNum;
    uint32_t portId;
    uint32_t i;
    int32_t status;

    status = Cpsw_validateTxShortIpgCfg(hCpsw, inArgs);
    if (status == ENET_SOK)
    {
        if (inArgs->configureGapThresh)
        {
            CSL_CPSW_setGapThreshold(regs, inArgs->ipgTriggerThreshBlkCnt);
        }

        for (i = 0U; i < inArgs->numMacPorts; i++)
        {
            portShortIpgCfg = &inArgs->portShortIpgCfg[i];
            portNum = ENET_MACPORT_NORM(portShortIpgCfg->macPort);
            portId = ENET_MACPORT_ID(portShortIpgCfg->macPort);
            ENETTRACE_VAR(portId);

            /* Assert if port number is not correct */
            Enet_assert(portNum < EnetSoc_getMacPortMax(hCpsw->enetPer.enetType, hCpsw->enetPer.instId),
                        "Invalid Port Id: %u\r\n", portNum);
            ENET_IOCTL_SET_IN_ARGS(&prms, portShortIpgCfg);

            CPSW_MACPORT_PRIV_IOCTL(hCpsw->hMacPort[portNum], CPSW_MACPORT_IOCTL_SET_SHORT_IPG, &prms, status);
            if (status != ENET_SOK)
            {
                ENETTRACE_ERR("Failed to set MAC port %u's short IPG: %d\r\n", portId, status);
                break;
            }
        }
    }
    else
    {
        ENETTRACE_ERR("Failed to validated short IPG config: %d\r\n", status);
    }

    return status;
}

static int32_t Cpsw_validateTxShortIpgCfg(const Cpsw_Handle hCpsw,
                                          const Cpsw_SetTxShortIpgCfgInArgs *inArgs)
{

    const CpswMacPort_PortTxShortIpgCfg *portShortIpgCfg;
    uint32_t macPortList = 0U;
    uint32_t portNum;
    uint32_t portId;
    uint32_t i;
    int32_t status = ENET_SOK;

    if (inArgs->configureGapThresh &&
        (inArgs->ipgTriggerThreshBlkCnt > CSL_XGE_CPSW_GAP_THRESH_REG_GAP_THRESH_MAX))
    {
        ENETTRACE_ERR("IPG trigger threshold block count %u exceeded max\r\n",
                      inArgs->ipgTriggerThreshBlkCnt);
        status = ENET_EINVALIDPARAMS;
    }

    if (ENET_SOK == status)
    {
        if (inArgs->numMacPorts > ENET_ARRAYSIZE(inArgs->portShortIpgCfg))
        {
            ENETTRACE_ERR("Invalid number of MAC ports %u exceeded %u\r\n",
                          inArgs->numMacPorts, ENET_ARRAYSIZE(inArgs->portShortIpgCfg));
            status = ENET_EINVALIDPARAMS;
        }
    }

    if (ENET_SOK == status)
    {
        for (i = 0U; i < inArgs->numMacPorts; i++)
        {
            portShortIpgCfg = &inArgs->portShortIpgCfg[i];
            portNum = ENET_MACPORT_NORM(portShortIpgCfg->macPort);
            ENETTRACE_VAR(portId);
            portId = ENET_MACPORT_ID(portShortIpgCfg->macPort);

            /* Assert if port number is not correct */
            Enet_assert(portNum < EnetSoc_getMacPortMax(hCpsw->enetPer.enetType, hCpsw->enetPer.instId),
                        "Invalid Port Id: %u\r\n", portNum);

            if ((portNum >= hCpsw->macPortNum) ||
                !EnetMod_isOpen(hCpsw->hMacPort[portNum]))
            {
                ENETTRACE_ERR("Invalid MAC port %u\r\n", portId);
                status = ENET_EINVALIDPARAMS;
                break;
            }

            if (ENET_IS_BIT_SET(macPortList, portNum))
            {
                /* inArgs->portIpgCfg entries should have list of unique MAC ports.
                 * Flag error as previously seen macPort in inArgs->portIpgCfg
                 * is encountered again */
                ENETTRACE_ERR("Multiple occurence of MAC port %u\r\n", portId);
                status = ENET_EINVALIDPARAMS;
                break;
            }

            if ((portShortIpgCfg->shortIpgCfg.txShortGapEn == false) &&
                (portShortIpgCfg->shortIpgCfg.txShortGapLimitEn == true))
            {
                ENETTRACE_ERR("MAC port %u's short gap limit can be enabled only if short gap is enabled\r\n", portId);
                status = ENET_EINVALIDPARAMS;
                break;
            }

            macPortList |= ENET_BIT(portNum);
        }
    }

    return status;
}

static int32_t Cpsw_getTxShortIpgCfg(const Cpsw_Handle hCpsw,
                                     Cpsw_TxShortIpgCfg *shortIpgCfg)
{

    CSL_Xge_cpswRegs *regs = (CSL_Xge_cpswRegs *)hCpsw->enetPer.virtAddr;
    EnetMod_Handle hMacPort;
    Enet_IoctlPrms prms;
    uint32_t i;
    int32_t status = ENET_SOK;

    shortIpgCfg->ipgTriggerThreshBlkCnt = CSL_CPSW_getGapThreshold(regs);
    shortIpgCfg->numMacPorts = 0U;

    for (i = 0U; i < hCpsw->macPortNum; i++)
    {
        hMacPort = hCpsw->hMacPort[i];

        if (EnetMod_isOpen(hMacPort))
        {
            EnetMacPort_GenericInArgs inArgs;
            CpswMacPort_PortTxShortIpgCfg *portIpgCfg;

            inArgs.macPort = ENET_MACPORT_DENORM(i);
            portIpgCfg = &shortIpgCfg->portShortIpgCfg[shortIpgCfg->numMacPorts];
            portIpgCfg->macPort = inArgs.macPort;
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &portIpgCfg->shortIpgCfg);

            CPSW_MACPORT_PRIV_IOCTL(hMacPort, CPSW_MACPORT_IOCTL_GET_SHORT_IPG, &prms, status);
            if (status != ENET_SOK)
            {
                ENETTRACE_ERR("Port %u: Failed to get short IPG: %d\r\n",
                              ENET_MACPORT_ID(inArgs.macPort), status);
                break;
            }

            shortIpgCfg->numMacPorts++;
        }
    }

    return status;
}

static int32_t Cpsw_validateDfltFlow(Cpsw_Handle hCpsw,
                                     Enet_DfltFlowInfo *dfltFlowInfo,
                                     uint32_t flowId)
{
    int32_t status;
    Enet_IoctlPrms prms;

    status = Cpsw_validateFlowId(hCpsw,
                                 dfltFlowInfo->coreKey,
                                 dfltFlowInfo->startIdx,
                                 dfltFlowInfo->flowIdx);
    if (status == ENET_SOK)
    {
        CpswAle_DfltThreadCfg defaultThreadCfg;

        ENET_IOCTL_SET_OUT_ARGS(&prms, &defaultThreadCfg);

        CPSW_ALE_PRIV_IOCTL(hCpsw->hAle,
                            CPSW_ALE_IOCTL_GET_DEFAULT_THREADCFG,
                            &prms,
                            status);
        if (status == ENET_SOK)
        {
            if ((defaultThreadCfg.dfltThreadEn == true)
                &&
                (defaultThreadCfg.threadId == flowId))
            {
                status = ENET_SOK;
            }
            else
            {
                status = ENET_EINVALIDPARAMS;
            }
        }
    }

    return status;
}


static int32_t Cpsw_validateFlowId(Cpsw_Handle hCpsw,
                                   uint32_t coreKey,
                                   uint32_t startIdx,
                                   uint32_t flowIdx)
{
    int32_t status;
    Enet_IoctlPrms prms;
    uint32_t p0FlowIdOffset;

    ENET_IOCTL_SET_OUT_ARGS(&prms, &p0FlowIdOffset);
    CPSW_HOSTPORT_PRIV_IOCTL(hCpsw->hHostPort,
                             CPSW_HOSTPORT_GET_FLOW_ID_OFFSET,
                             &prms,
                             status);

    if (status == ENET_SOK)
    {
        if (startIdx != p0FlowIdOffset)
        {
            status = ENET_EINVALIDPARAMS;
        }
    }

    if (status == ENET_SOK)
    {
        EnetRm_ValidateRxFlowInArgs rmInArgs;

        rmInArgs.chIdx   = CPSW_RM_RX_CH_IDX;
        rmInArgs.coreKey = coreKey;
        rmInArgs.flowIdx = flowIdx;

        ENET_IOCTL_SET_IN_ARGS(&prms, &rmInArgs);

        ENET_RM_PRIV_IOCTL(hCpsw->hRm, ENET_RM_IOCTL_VALIDATE_RX_FLOW, &prms, status);
    }

    return status;
}

static int32_t Cpsw_handleExternalPhyLinkUp(Cpsw_Handle hCpsw,
                                             Enet_MacPort macPort,
                                             const EnetPhy_LinkCfg *phyLinkCfg)
{

    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    EnetMod_Handle hMacPort = NULL;
    Enet_IoctlPrms prms;
    CpswAle_SetPortStateInArgs setPortStateInArgs;
    EnetMacPort_LinkCfg macLinkCfg;
    bool isPortLinked = false;
    int32_t status;

    /* Assert if port number is not correct */
    Enet_assert(portNum < EnetSoc_getMacPortMax(hCpsw->enetPer.enetType, hCpsw->enetPer.instId),
                "Invalid Port Id: %u\r\n", portNum);
    hMacPort = hCpsw->hMacPort[portNum];

    ENETTRACE_VAR(portId);
    /* Check that port status also detected link up */
    do
    {
        /* Need to repeatedly check port status as it does not reflect
         * PHY status immediately */
        status = Cpsw_isPortLinkUp(hCpsw, macPort, &isPortLinked);
    }
    while ((status == ENET_SOK) && !isPortLinked);

    if (status == ENET_SOK)
    {
        if (!isPortLinked)
        {
            ENETTRACE_ERR("Port %d: Link indicator detected link down\r\n", portId);
            Enet_devAssert(false, "Port %d: Link indicator detected link down\r\n", portId);
            status = ENET_EUNEXPECTED;
        }
    }
    else if (status == ENET_ENOTSUPPORTED)
    {
        /* It's not an error if port interface doesn't support link indicator */
        status = ENET_SOK;
    }
    else
    {
        ENETTRACE_ERR("Port %u: Failed to get link indicator: %d\r\n", portId, status);
    }


    /* Enable MAC port */
    if (status == ENET_SOK)
    {
        macLinkCfg.speed = (Enet_Speed)phyLinkCfg->speed;
        macLinkCfg.duplexity = (Enet_Duplexity)phyLinkCfg->duplexity;
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

    if (ENET_SOK == status)
    {
        hCpsw->portLinkState[portNum].isLinkUp = true;
    }
    ENETTRACE_INFO_IF(status == ENET_SOK,
                      "Port %d: Link up: %s %s\r\n",
                      portId,
                      Cpsw_gSpeedNames[phyLinkCfg->speed],
                      Cpsw_gDuplexNames[phyLinkCfg->duplexity]);

    return status;
}

#if ENET_CFG_IS_ON(CPSW_SGMII)
static uint32_t Cpsw_mapPort2XgmiiId(Enet_MacPort macPort)
{
    /* TODO: Needs correct logic to translate portNum to QSGMII Id */
    return 0U;
}

static int32_t Cpsw_setSgmiiMode(Cpsw_Handle hCpsw,
                                 Enet_MacPort macPort,
                                 const EnetMacPort_Interface *mii,
                                 const CpswMacPort_Cfg *macCfg)
{
    EnetPer_Handle hPer = (EnetPer_Handle)hCpsw;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    EnetMod_Handle hMacPort = hCpsw->hMacPort[portNum];
    CSL_Xge_cpsw_ss_sRegs *ssRegs = (CSL_Xge_cpsw_ss_sRegs *)hPer->virtAddr2;
    int32_t status = ENET_EINVALIDPARAMS;

    if (EnetMacPort_isSgmii(mii) &&
        ENET_FEAT_IS_EN(hMacPort->features, CPSW_MACPORT_FEATURE_SGMII))
    {
        status = ENET_SOK;
    }
    else if (EnetMacPort_isQsgmii(mii) &&
             ENET_FEAT_IS_EN(hMacPort->features, CPSW_MACPORT_FEATURE_SGMII)) /* TODO: Add QSGMII feature flag */
    {
        status = ENET_SOK;
    }
    else
    {
        /* Invalid configuration */
        status = ENET_EINVALIDPARAMS;
    }

    if (status == ENET_SOK)
    {
        if (macCfg->sgmiiMode == ENET_MAC_SGMIIMODE_FIBER_WITH_PHY)
        {
            CSL_CPSW_SS_setSGMIIMode(ssRegs, portNum, CSL_SGMII_MODE_FIBER);
        }
        else
        {
            CSL_CPSW_SS_setSGMIIMode(ssRegs, portNum, CSL_SGMII_MODE_SGMII);
        }
    }

    return status;
}
#endif


