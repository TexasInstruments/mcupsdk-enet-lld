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
 * \file  cpsw_macport.c
 *
 * \brief This file contains the implementation of the CPSW MAC port module.
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
#include <include/core/enet_mod_tas.h>
#include <include/mod/cpsw_macport.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/mod/cpsw_macport_priv.h>
#include <priv/mod/cpsw_macport_ioctl_priv.h>
#include <src/mod/cpsw_macport_intervlan.h>
#include <src/mod/cpsw_macport_intervlan_ioctl_priv.h>
#include <priv/mod/cpsw_clks.h>
#include "cpsw_macport_intervlan.h"
#if ENET_CFG_IS_ON(CPSW_MACPORT_EST)
#include "cpsw_macport_est.h"
#include "cpsw_macport_est_ioctl_priv.h"
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/*! \brief MAC port register start offset. */
#define CPSW_MACPORT_START_REG_OFFSET         (offsetof(CSL_Xge_cpswEnetportRegs, PN_CONTROL_REG))

/*! \brief MAC port register end offset. */
#define CPSW_MACPORT_END_REG_OFFSET           (offsetof(CSL_Xge_cpswEnetportRegs, PN_INTERVLAN_OPX_D_REG))


/*! \brief PTP sequence ID offset minimum value. */
#define CPSW_MACPORT_PTP_SEQID_OFFSET_MIN_VAL (0x6U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static void CpswMacPort_printRegs(CSL_Xge_cpswRegs *regs,
                                  Enet_MacPort macPort);

static int32_t CpswMacPort_setDscpPriority(CSL_Xge_cpswRegs *regs,
                                           Enet_MacPort macPort,
                                           EnetPort_DscpPriorityMap *dscpPriority);

static void CpswMacPort_getDscpPriority(CSL_Xge_cpswRegs *regs,
                                        Enet_MacPort macPort,
                                        EnetPort_DscpPriorityMap *dscpPriority);

static int32_t CpswMacPort_setRxPriority(CSL_Xge_cpswRegs *regs,
                                         Enet_MacPort macPort,
                                         EnetPort_PriorityMap *rxPriority);

static void CpswMacPort_getRxPriority(CSL_Xge_cpswRegs *regs,
                                      Enet_MacPort macPort,
                                      EnetPort_PriorityMap *rxPriority);

static int32_t CpswMacPort_setTxPriority(CSL_Xge_cpswRegs *regs,
                                         Enet_MacPort macPort,
                                         EnetPort_PriorityMap *txPriority);

static void CpswMacPort_getTxPriority(CSL_Xge_cpswRegs *regs,
                                      Enet_MacPort macPort,
                                      EnetPort_PriorityMap *txPriority);

static void CpswMacPort_getMaxLen(CSL_Xge_cpswRegs *regs,
                                  Enet_MacPort macPort,
                                  EnetPort_MaxLen *maxLen);

static void CpswMacPort_getLinkCfg(CSL_Xge_cpswRegs *regs,
                                   Enet_MacPort macPort,
                                   EnetMacPort_LinkCfg *linkCfg);

static void CpswMacPort_getFifoStats(CSL_Xge_cpswRegs *regs,
                                     Enet_MacPort macPort,
                                     CpswMacPort_FifoStats *stats);

static int32_t CpswMacPort_enableCptsPortEvents(CSL_Xge_cpswRegs *regs,
                                                Enet_MacPort macPort,
                                                CpswMacPort_TsEventCfg *tsEventCfg);

static void CpswMacPort_disableCptsPortEvents(CSL_Xge_cpswRegs *regs,
                                              Enet_MacPort macPort);

static int32_t CpswMacPort_enablePort(CSL_Xge_cpswRegs *regs,
                                      Enet_MacPort macPort,
                                      const EnetMacPort_Interface *mii,
                                      const EnetMacPort_LinkCfg *linkCfg);

static void CpswMacPort_setShortIpgCfg(CSL_Xge_cpswRegs *regs,
                                       Enet_MacPort macPort,
                                       const CpswMacPort_TxShortIpgCfg *shortIpgCfg);

static void CpswMacPort_getShortIpgCfg(CSL_Xge_cpswRegs *regs,
                                       Enet_MacPort macPort,
                                       CpswMacPort_TxShortIpgCfg *shortIpgCfg);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_VERSION(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    Enet_Version *version = (Enet_Version *)prms->outArgs;
    CSL_CPSW_VERSION ver;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    /* Report CPSW Control version as ours */
    CSL_CPSW_getCpswVersionInfo(regs, &ver);
    version->maj = ver.majorVer;
    version->min = ver.minorVer;
    version->rtl = ver.rtlVer;
    version->id  = ver.id;
    version->other1 = ENET_VERSION_NONE;
    version->other2 = ENET_VERSION_NONE;
    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_PRINT_REGS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    CpswMacPort_printRegs(regs, macPort);
    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_INGRESS_DSCP_PRI_MAP(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_SetIngressDscpPriorityMapInArgs *inArgs =
        (EnetMacPort_SetIngressDscpPriorityMapInArgs *)prms->inArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    status = CpswMacPort_setDscpPriority(regs, macPort, &inArgs->dscpPriorityMap);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: Failed to set DSCP priority: %d\n", ENET_MACPORT_ID(macPort), status);
    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_INGRESS_DSCP_PRI_MAP(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    EnetPort_DscpPriorityMap *dscpPriority = (EnetPort_DscpPriorityMap *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    CpswMacPort_getDscpPriority(regs, macPort, dscpPriority);
    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_SetPriorityRegenMapInArgs *inArgs =
        (EnetMacPort_SetPriorityRegenMapInArgs *)prms->inArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    status = CpswMacPort_setRxPriority(regs, macPort, &inArgs->priorityRegenMap);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: Failed to set RX priority: %d\n", ENET_MACPORT_ID(macPort), status);
    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_PRI_REGEN_MAP(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    EnetPort_PriorityMap *rxPriority = (EnetPort_PriorityMap *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    CpswMacPort_getRxPriority(regs, macPort, rxPriority);
    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_SetEgressPriorityMapInArgs *inArgs =
        (EnetMacPort_SetEgressPriorityMapInArgs *)prms->inArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    status = CpswMacPort_setTxPriority(regs, macPort, &inArgs->priorityMap);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: Failed to set RX priority: %d\n", ENET_MACPORT_ID(macPort), status);
    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_EGRESS_QOS_PRI_MAP(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    EnetPort_PriorityMap *txPriority = (EnetPort_PriorityMap *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    CpswMacPort_getTxPriority(regs, macPort, txPriority);
    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_ENABLE_EGRESS_TRAFFIC_SHAPING(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(CPSW_MACPORT_TRAFFIC_SHAPING)
    const EnetMacPort_EnableEgressTrafficShapingInArgs *inArgs =
        (const EnetMacPort_EnableEgressTrafficShapingInArgs *)prms->inArgs;
    uint32_t cppiClkFreqHz;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", portId, inArgs->macPort);

    cppiClkFreqHz = EnetSoc_getClkFreq(hPort->enetType, hPort->instId, CPSW_CPPI_CLK);

    status = CpswMacPort_setTrafficShaping(regs, macPort, &inArgs->trafficShapingCfg, cppiClkFreqHz);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set traffic shaping: %d\n", status);

                status = ENET_ENOTSUPPORTED;
#endif
    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_DISABLE_EGRESS_TRAFFIC_SHAPING(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(CPSW_MACPORT_TRAFFIC_SHAPING)
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", portId, inArgs->macPort);

    CpswMacPort_disableTrafficShaping(regs, macPort);
#else
    status = ENET_ENOTSUPPORTED;
#endif
    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_EGRESS_TRAFFIC_SHAPING(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(CPSW_MACPORT_TRAFFIC_SHAPING)
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    EnetPort_TrafficShapingCfg *shapingCfg = (EnetPort_TrafficShapingCfg *)prms->outArgs;
    uint32_t cppiClkFreqHz;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", portId, inArgs->macPort);

    cppiClkFreqHz = EnetSoc_getClkFreq(hPort->enetType, hPort->instId, CPSW_CPPI_CLK);

    status = CpswMacPort_getTrafficShaping(regs, macPort, shapingCfg, cppiClkFreqHz);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get traffic shaping: %d\n", status);
#else
    status = ENET_ENOTSUPPORTED;
#endif
    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_MAXLEN(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    EnetPort_MaxLen *maxLen = (EnetPort_MaxLen *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    CpswMacPort_getMaxLen(regs, macPort, maxLen);
    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_LINK_CFG(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    EnetMacPort_LinkCfg *linkCfg = (EnetMacPort_LinkCfg *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    CpswMacPort_getLinkCfg(regs, macPort, linkCfg);
    return status;
}

int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_GET_FIFO_STATS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    CpswMacPort_FifoStats *stats = (CpswMacPort_FifoStats *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    CpswMacPort_getFifoStats(regs, macPort, stats);
    return status;
}

int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_ENABLE_CPTS_EVENT(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    CpswMacPort_EnableTsEventInArgs *inArgs = (CpswMacPort_EnableTsEventInArgs *)prms->inArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    status = CpswMacPort_enableCptsPortEvents(regs, macPort, &inArgs->tsEventCfg);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: Failed to enable CPTS event: %d\n", ENET_MACPORT_ID(macPort), status);
    return status;
}

int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_DISABLE_CPTS_EVENT(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    CpswMacPort_disableCptsPortEvents(regs, macPort);
    return status;
}

int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_ENABLE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    const EnetMacPort_LinkCfg *linkCfg = (const EnetMacPort_LinkCfg *)prms->inArgs;
    EnetMacPort_Interface mii;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    status = EnetSoc_getMacPortMii(hPort->enetType, hPort->instId, macPort, &mii);
    if (status == ENET_SOK)
    {
        if (EnetMacPort_isSgmii(&mii) || EnetMacPort_isQsgmii(&mii))
        {
#if ENET_CFG_IS_ON(CPSW_MACPORT_SGMII)
            if (ENET_FEAT_IS_EN(((EnetMod_Handle)hPort)->features, CPSW_MACPORT_FEATURE_SGMII))
            {
                status = CpswMacPort_enableSgmiiPort(regs, sgmiiRegs, macPort, &mii, linkCfg);
            }
            else
            {
                status = ENET_ENOTSUPPORTED;
            }
#else
            status = ENET_ENOTSUPPORTED;
#endif
        }
        else
        {
            status = CpswMacPort_enablePort(regs, macPort, &mii, linkCfg);
            if (status == ENET_SOK)
            {
                hPort->linkCfg = *linkCfg;
                hPort->enabled = true;
            }
        }
    }

    ENETTRACE_ERR_IF(status != ENET_SOK,
                                 "MAC %u: Failed to enable port: %d\n", ENET_MACPORT_ID(macPort), status);
    return status;
}
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_DISABLE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    CpswMacPort_disablePort(regs, macPort);
    hPort->enabled = false;
    return status;
}

int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_SET_INTERVLAN_ROUTE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status;

    CPSW_MACPORT_INTERVLAN_PRIV_IOCTL(hPort, CPSW_MACPORT_IOCTL_SET_INTERVLAN_ROUTE, prms, status);
    return  status;
}

int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_SET_SPECIFIC_INTERVLAN_ROUTE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status;

    CPSW_MACPORT_INTERVLAN_PRIV_IOCTL(hPort, CPSW_MACPORT_IOCTL_SET_SPECIFIC_INTERVLAN_ROUTE, prms, status);
    return  status;
}
int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_GET_INTERVLAN_ROUTE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status;

    CPSW_MACPORT_INTERVLAN_PRIV_IOCTL(hPort, CPSW_MACPORT_IOCTL_GET_INTERVLAN_ROUTE, prms, status);
    return  status;
}

int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_DELETE_INTERVLAN_ROUTE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status;

    CPSW_MACPORT_INTERVLAN_PRIV_IOCTL(hPort, CPSW_MACPORT_IOCTL_DELETE_INTERVLAN_ROUTE, prms, status);
    return  status;
}

int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_GET_INTERVLAN_FREEROUTES(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status;

    CPSW_MACPORT_INTERVLAN_PRIV_IOCTL(hPort, CPSW_MACPORT_IOCTL_GET_INTERVLAN_FREEROUTES, prms, status);
    return  status;
}

int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_FIND_INTERVLAN_ROUTE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status;

    CPSW_MACPORT_INTERVLAN_PRIV_IOCTL(hPort, CPSW_MACPORT_IOCTL_FIND_INTERVLAN_ROUTE, prms, status);
    return  status;
}

int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_IS_INTERVLAN_ROUTE_FREE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status;

    CPSW_MACPORT_INTERVLAN_PRIV_IOCTL(hPort, CPSW_MACPORT_IOCTL_IS_INTERVLAN_ROUTE_FREE, prms, status);
    return  status;
}

int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_SET_SHORT_IPG(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswMacPort_PortTxShortIpgCfg *inArgs =
        (const CpswMacPort_PortTxShortIpgCfg *)prms->inArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    CpswMacPort_setShortIpgCfg(regs, macPort, &inArgs->shortIpgCfg);
    return status;
}

int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_GET_SHORT_IPG(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    CpswMacPort_TxShortIpgCfg *shortIpgCfg = (CpswMacPort_TxShortIpgCfg *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    CpswMacPort_getShortIpgCfg(regs, macPort, shortIpgCfg);
    return status;
}

int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_GET_SGMII_AUTONEG_LINK_STATUS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

#if ENET_CFG_IS_ON(CPSW_MACPORT_SGMII)
    if (ENET_FEAT_IS_EN(((EnetMod_Handle)hPort)->features, CPSW_MACPORT_FEATURE_SGMII))
    {
        EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
        bool *autoNegDone = (bool *)prms->outArgs;

        Enet_devAssert(macPort == inArgs->macPort,
                       "MAC %u: Port mismatch %u\n", portId, inArgs->macPort);

        status = CpswMacPort_checkSgmiiAutoNegStatus(sgmiiRegs, macPort);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "MAC %u: Failed to get SGMII link status: %d\n", portId, status);

        *autoNegDone = (ENET_SOK == status);
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

int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_GET_SGMII_LINK_STATUS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

#if ENET_CFG_IS_ON(CPSW_MACPORT_SGMII)
    if (ENET_FEAT_IS_EN(((EnetMod_Handle)hPort)->features, CPSW_MACPORT_FEATURE_SGMII))
    {
        EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
        bool *linkUp = (bool *)prms->outArgs;

        Enet_devAssert(macPort == inArgs->macPort,
                       "MAC %u: Port mismatch %u\n", portId, inArgs->macPort);

        *linkUp = CpswMacPort_getSgmiiStatus(sgmiiRegs, macPort);
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

#if ENET_CFG_IS_ON(CPSW_MACPORT_EST)
int32_t CpswMacPort_ioctl_handler_ENET_TAS_IOCTL_SET_ADMIN_LIST(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CPSW_MACPORT_EST_PRIV_IOCTL(hPort, ENET_TAS_IOCTL_SET_ADMIN_LIST,prms,status);
    return status;    
}
int32_t CpswMacPort_ioctl_handler_ENET_TAS_IOCTL_GET_OPER_LIST_STATUS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CPSW_MACPORT_EST_PRIV_IOCTL(hPort, ENET_TAS_IOCTL_GET_OPER_LIST_STATUS,prms,status);
    return status;    
}

int32_t CpswMacPort_ioctl_handler_ENET_TAS_IOCTL_SET_STATE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CPSW_MACPORT_EST_PRIV_IOCTL(hPort, ENET_TAS_IOCTL_SET_STATE,prms,status);
    return status;    
}

int32_t CpswMacPort_ioctl_handler_ENET_TAS_IOCTL_GET_STATE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CPSW_MACPORT_EST_PRIV_IOCTL(hPort, ENET_TAS_IOCTL_GET_STATE,prms,status);
    return status;    
}

int32_t CpswMacPort_ioctl_handler_ENET_TAS_IOCTL_GET_ADMIN_LIST(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CPSW_MACPORT_EST_PRIV_IOCTL(hPort, ENET_TAS_IOCTL_GET_ADMIN_LIST,prms,status);
    return status;    
}

int32_t CpswMacPort_ioctl_handler_ENET_TAS_IOCTL_GET_OPER_LIST(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CPSW_MACPORT_EST_PRIV_IOCTL(hPort, ENET_TAS_IOCTL_GET_OPER_LIST,prms,status);
    return status;    
}

int32_t CpswMacPort_ioctl_handler_ENET_TAS_IOCTL_CONFIG_CHANGE_STATUS_PARAMS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CPSW_MACPORT_EST_PRIV_IOCTL(hPort, ENET_TAS_IOCTL_CONFIG_CHANGE_STATUS_PARAMS,prms,status);
    return status;    
}


int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_EST_ENABLE_TIMESTAMP(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CPSW_MACPORT_EST_PRIV_IOCTL(hPort, CPSW_MACPORT_IOCTL_EST_ENABLE_TIMESTAMP,prms,status);
    return status;    
}

int32_t CpswMacPort_ioctl_handler_CPSW_MACPORT_IOCTL_EST_DISABLE_TIMESTAMP(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CPSW_MACPORT_EST_PRIV_IOCTL(hPort, CPSW_MACPORT_IOCTL_EST_DISABLE_TIMESTAMP,prms,status);
    return status;    
}
#endif

static void CpswMacPort_printRegs(CSL_Xge_cpswRegs *regs,
                                  Enet_MacPort macPort)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    CSL_Xge_cpswEnetportRegs *macPortRegs = &regs->ENETPORT[portNum];
    uint32_t *regAddr = (uint32_t *)((uintptr_t)macPortRegs + CPSW_MACPORT_START_REG_OFFSET);
    uint32_t regIdx = 0U;

    ENETTRACE_VAR(portId);
    while (((uintptr_t)regAddr) <= ((uintptr_t)macPortRegs + CPSW_MACPORT_END_REG_OFFSET))
    {
        if (*regAddr != 0U)
        {
            ENETTRACE_INFO("MACPORT %u: %u: 0x%08x\n", portId, regIdx, *regAddr);
        }

        regAddr++;
        regIdx++;
    }
}

static int32_t CpswMacPort_setDscpPriority(CSL_Xge_cpswRegs *regs,
                                           Enet_MacPort macPort,
                                           EnetPort_DscpPriorityMap *dscpPriority)
{
    CSL_CPSW_PORT_CONTROL control;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    uint32_t i;
    int32_t status = ENET_SOK;

    ENETTRACE_VAR(portId);
    for (i = 0U; i < ENET_TOS_PRI_NUM; i++)
    {
        if (dscpPriority->tosMap[i] > ENET_PRI_MAX)
        {
            ENETTRACE_ERR("MAC %u: Invalid priority map %u -> %u\n", portId, i, dscpPriority->tosMap[i]);
            status = ENET_EINVALIDPARAMS;
            break;
        }
    }

    if (status == ENET_SOK)
    {
        CSL_CPSW_setPortRxDscpMap(regs, portNum + 1U, dscpPriority->tosMap);
        CSL_CPSW_getPortControlReg(regs, portNum + 1U, &control);

        control.dscpIpv4Enable = dscpPriority->dscpIPv4En;
        control.dscpIpv6Enable = dscpPriority->dscpIPv6En;

        CSL_CPSW_setPortControlReg(regs, portNum + 1U, &control);
    }

    return status;
}

static void CpswMacPort_getDscpPriority(CSL_Xge_cpswRegs *regs,
                                        Enet_MacPort macPort,
                                        EnetPort_DscpPriorityMap *dscpPriority)
{
    CSL_CPSW_PORT_CONTROL control;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);

    CSL_CPSW_getPortRxDscpMap(regs, portNum + 1U, dscpPriority->tosMap);
    CSL_CPSW_getPortControlReg(regs, portNum + 1U, &control);

    dscpPriority->dscpIPv4En = control.dscpIpv4Enable;
    dscpPriority->dscpIPv6En = control.dscpIpv6Enable;
}

static int32_t CpswMacPort_setRxPriority(CSL_Xge_cpswRegs *regs,
                                         Enet_MacPort macPort,
                                         EnetPort_PriorityMap *rxPriority)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    uint32_t i;
    int32_t status = ENET_SOK;

    ENETTRACE_VAR(portId);
    for (i = 0U; i < ENET_PRI_NUM; i++)
    {
        if (rxPriority->priorityMap[i] > ENET_PRI_MAX)
        {
            ENETTRACE_ERR("MAC %u: Invalid priority map %u -> %u\n", portId, i, rxPriority->priorityMap[i]);
            status = ENET_EINVALIDPARAMS;
            break;
        }
    }

    if (status == ENET_SOK)
    {
        CSL_CPSW_setPortRxPriMapReg(regs, portNum + 1U, rxPriority->priorityMap);
    }

    return status;
}

static void CpswMacPort_getRxPriority(CSL_Xge_cpswRegs *regs,
                                      Enet_MacPort macPort,
                                      EnetPort_PriorityMap *rxPriority)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);

    CSL_CPSW_getPortRxPriMapReg(regs, portNum + 1U, rxPriority->priorityMap);
}

static int32_t CpswMacPort_setTxPriority(CSL_Xge_cpswRegs *regs,
                                         Enet_MacPort macPort,
                                         EnetPort_PriorityMap *txPriority)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    uint32_t i;
    int32_t status = ENET_SOK;

    ENETTRACE_VAR(portId);
    for (i = 0U; i < ENET_PRI_NUM; i++)
    {
        if (txPriority->priorityMap[i] > ENET_PRI_MAX)
        {
            ENETTRACE_ERR("MAC %u: Invalid priority map %u -> %u\n", portId, i, txPriority->priorityMap[i]);
            status = ENET_EINVALIDPARAMS;
            break;
        }
    }

    if (status == ENET_SOK)
    {
        CSL_CPSW_setPortTxPriMapReg(regs, portNum + 1U, txPriority->priorityMap);
    }

    return status;
}

static void CpswMacPort_getTxPriority(CSL_Xge_cpswRegs *regs,
                                      Enet_MacPort macPort,
                                      EnetPort_PriorityMap *txPriority)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);

    CSL_CPSW_getPortTxPriMapReg(regs, portNum + 1U, txPriority->priorityMap);
}

static void CpswMacPort_getMaxLen(CSL_Xge_cpswRegs *regs,
                                  Enet_MacPort macPort,
                                  EnetPort_MaxLen *maxLen)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t i;

    maxLen->mru = CSL_CPGMAC_SL_getRxMaxLen(regs, portNum);

    for (i = 0U; i < ENET_PRI_NUM; i++)
    {
        maxLen->mtu[i] = CSL_CPSW_getTxMaxLenPerPriority(regs, i);
    }
}

static void CpswMacPort_getLinkCfg(CSL_Xge_cpswRegs *regs,
                                   Enet_MacPort macPort,
                                   EnetMacPort_LinkCfg *linkCfg)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    CSL_CPGMAC_SL_MACSTATUS macStatus;

    CSL_CPGMAC_SL_getMacStatusReg(regs, portNum, &macStatus);

    /* TODO - How to identify 10Mbps */
    if (macStatus.extGigabitEnabled == TRUE)
    {
        linkCfg->speed = ENET_SPEED_1GBIT;
    }
    else
    {
        linkCfg->speed = ENET_SPEED_100MBIT;
    }

    if (macStatus.extFullDuplexEnabled == TRUE)
    {
        linkCfg->duplexity = ENET_DUPLEX_FULL;
    }
    else
    {
        linkCfg->duplexity = ENET_DUPLEX_HALF;
    }
}

static void CpswMacPort_getFifoStats(CSL_Xge_cpswRegs *regs,
                                     Enet_MacPort macPort,
                                     CpswMacPort_FifoStats *stats)
{
    CSL_CPSW_THRURATE thruRate;
    CSL_CPGMAC_SL_FIFOSTATUS fifoStatus;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t i;

    CSL_CPSW_getPortBlockCountReg(regs, portNum + 1U,
                                  &stats->rxExpressBlockCount,
                                  &stats->rxPreemptBlockCount,
                                  &stats->txBlockCount);

    CSL_CPSW_getPortMaxBlksReg(regs, portNum + 1U,
                               &stats->rxMaxBlocks,
                               &stats->txMaxBlocks);

    CSL_CPSW_getThruRateReg(regs, &thruRate);
    stats->rxThroughputRate = thruRate.enetRxThruRate;
    stats->txStartWords = CSL_CPSW_getTxStartWords(regs);

    CSL_CPGMAC_SL_getFifoStatus(regs, portNum, &fifoStatus);
    for (i = 0U; i < ENET_ARRAYSIZE(stats->txActiveFifo); i++)
    {
        stats->txActiveFifo[i] = (fifoStatus.txPriActive >> i) & 0x1U;
    }
}

static int32_t CpswMacPort_enableCptsPortEvents(CSL_Xge_cpswRegs *regs,
                                                Enet_MacPort macPort,
                                                CpswMacPort_TsEventCfg *tsEventCfg)
{
    CSL_CPSW_TSCONFIG timeSyncCfg;
    CpswMacPort_IpTsCfg *commonPortIpCfg;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    if (tsEventCfg->seqIdOffset >= CPSW_MACPORT_PTP_SEQID_OFFSET_MIN_VAL)
    {
        commonPortIpCfg = &tsEventCfg->commonPortIpCfg;

        memset(&timeSyncCfg, 0, sizeof(CSL_CPSW_TSCONFIG));

        timeSyncCfg.tsTxHostEnable   = tsEventCfg->txHostTsEn;
        timeSyncCfg.tsRxAnnexDEnable = tsEventCfg->rxAnnexDEn;
        timeSyncCfg.tsRxAnnexEEnable = tsEventCfg->rxAnnexEEn;
        timeSyncCfg.tsRxAnnexFEnable = tsEventCfg->rxAnnexFEn;
        timeSyncCfg.tsTxAnnexDEnable = tsEventCfg->txAnnexDEn;
        timeSyncCfg.tsTxAnnexEEnable = tsEventCfg->txAnnexEEn;
        timeSyncCfg.tsTxAnnexFEnable = tsEventCfg->txAnnexFEn;
        timeSyncCfg.tsMsgTypeEnable  = tsEventCfg->messageType;
        timeSyncCfg.tsSeqIdOffset    = tsEventCfg->seqIdOffset;
        timeSyncCfg.tsDomainOffset   = tsEventCfg->domainOffset;

        if (tsEventCfg->rxAnnexDEn || tsEventCfg->rxAnnexEEn ||
            tsEventCfg->txAnnexDEn || tsEventCfg->txAnnexEEn)
        {
            if (tsEventCfg->rxAnnexEEn || tsEventCfg->txAnnexEEn)
            {
                timeSyncCfg.tsMcastTypeEnable = tsEventCfg->mcastType;
            }

            timeSyncCfg.tsTTLNonzeroEnable = commonPortIpCfg->ttlNonzeroEn;
            timeSyncCfg.tsUniEnable        = commonPortIpCfg->unicastEn;

            if (timeSyncCfg.tsUniEnable == FALSE)
            {
                timeSyncCfg.ts107Enable = commonPortIpCfg->tsIp107En;
                timeSyncCfg.ts129Enable = commonPortIpCfg->tsIp129En;
                timeSyncCfg.ts130Enable = commonPortIpCfg->tsIp130En;
                timeSyncCfg.ts131Enable = commonPortIpCfg->tsIp131En;
                timeSyncCfg.ts132Enable = commonPortIpCfg->tsIp132En;
                timeSyncCfg.ts319Enable = commonPortIpCfg->tsPort319En;
                timeSyncCfg.ts320Enable = commonPortIpCfg->tsPort320En;
            }
        }

        if (tsEventCfg->rxAnnexFEn || tsEventCfg->txAnnexFEn)
        {
            timeSyncCfg.tsLType2Enable  = tsEventCfg->ltype2En;
            timeSyncCfg.tsLType1        = ENET_ETHERTYPE_PTP;

            if (timeSyncCfg.tsLType2Enable == TRUE)
            {
                timeSyncCfg.tsLType2    = ENET_ETHERTYPE_PTP;
            }
        }

        if (tsEventCfg->rxVlanType != ENET_MACPORT_VLAN_TYPE_NONE)
        {
            timeSyncCfg.tsRxVlanLType1Enable = TRUE;
            if (tsEventCfg->rxVlanType == ENET_MACPORT_VLAN_TYPE_STACKED_TAGS)
            {
                timeSyncCfg.tsRxVlanLType2Enable = TRUE;
            }
        }

        if (tsEventCfg->txVlanType != ENET_MACPORT_VLAN_TYPE_NONE)
        {
            timeSyncCfg.tsTxVlanLType1Enable = TRUE;
            if (tsEventCfg->txVlanType == ENET_MACPORT_VLAN_TYPE_STACKED_TAGS)
            {
                timeSyncCfg.tsTxVlanLType2Enable = TRUE;
            }
        }

        if ((tsEventCfg->rxVlanType != ENET_MACPORT_VLAN_TYPE_NONE) ||
            (tsEventCfg->txVlanType != ENET_MACPORT_VLAN_TYPE_NONE))
        {
            timeSyncCfg.tsVlanLType1 = tsEventCfg->vlanLType1;

            if ((tsEventCfg->rxVlanType != ENET_MACPORT_VLAN_TYPE_STACKED_TAGS)||
                (tsEventCfg->txVlanType != ENET_MACPORT_VLAN_TYPE_STACKED_TAGS))
            {
                timeSyncCfg.tsVlanLType2 = tsEventCfg->vlanLType2;
            }
        }

        CSL_CPSW_setPortTimeSyncConfig(regs, portNum + 1U, &timeSyncCfg);
    }
    else
    {
        status = ENET_EINVALIDPARAMS;
    }

    return status;
}

static void CpswMacPort_disableCptsPortEvents(CSL_Xge_cpswRegs *regs,
                                              Enet_MacPort macPort)
{
    CSL_CPSW_TSCONFIG timeSyncCfg;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);

    CSL_CPSW_getPortTimeSyncConfig(regs, portNum + 1U, &timeSyncCfg);

    timeSyncCfg.tsTxHostEnable   = FALSE;
    timeSyncCfg.tsRxAnnexDEnable = FALSE;
    timeSyncCfg.tsRxAnnexEEnable = FALSE;
    timeSyncCfg.tsRxAnnexFEnable = FALSE;
    timeSyncCfg.tsTxAnnexDEnable = FALSE;
    timeSyncCfg.tsTxAnnexEEnable = FALSE;
    timeSyncCfg.tsTxAnnexFEnable = FALSE;
    timeSyncCfg.tsMsgTypeEnable  = FALSE;

    CSL_CPSW_setPortTimeSyncConfig(regs, portNum + 1U, &timeSyncCfg);
}

static int32_t CpswMacPort_enablePort(CSL_Xge_cpswRegs *regs,
                                      Enet_MacPort macPort,
                                      const EnetMacPort_Interface *mii,
                                      const EnetMacPort_LinkCfg *linkCfg)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    uint32_t extCtrl;
    uint32_t macControl;
    int32_t status;
    bool forced = true;

    ENETTRACE_VAR(portId);

    macControl = CSL_CPGMAC_SL_getMacControlReg(regs, portNum);

    /* Disable all fields, enable as needed based on link config */
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_FULLDUPLEX, 0U);
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_GMII_EN, 0U);
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_GIG, 0U);
#if ENET_CFG_IS_ON(CPSW_XGMII)
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_XGMII_EN, 0U);
#endif
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_IFCTL_A, 0U);
    CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_IFCTL_B, 0U);

    if (EnetMacPort_isRmii(mii))
    {
        /* In RMII mode, ifctl_a bit determines the RMII link speed (0=10mbps, 1=100mbps) */
        if ((linkCfg->speed == ENET_SPEED_100MBIT) ||
            (linkCfg->speed == ENET_SPEED_10MBIT))
        {
            if (linkCfg->speed == ENET_SPEED_100MBIT)
            {
                CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_IFCTL_A, 1U);
            }

            CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_GMII_EN, 1U);
            status = ENET_SOK;
        }
        else
        {
            ENETTRACE_ERR("MAC %u: Invalid speed for RMII mode\n", portId);
            status = ENET_EINVALIDPARAMS;
        }

        forced = true;
    }
    else if (EnetMacPort_isRgmii(mii))
    {
        /* RGMII */
        extCtrl = CSL_FEXT(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_EXT_EN);
        forced = (extCtrl == TRUE) ? false : true;

        if (forced)
        {
            if (linkCfg->speed == ENET_SPEED_1GBIT)
            {
                CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_GIG, 1U);
            }
        }

        CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_GMII_EN, 1U);
        status = ENET_SOK;
    }
#if ENET_CFG_IS_ON(CPSW_XGMII)
    else if (EnetMacPort_isXfi(mii))
    {
        /* XGMII */
        extCtrl = CSL_FEXT(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_EXT_EN_XGIG);
        forced = (extCtrl == TRUE) ? false : true;

        if (forced)
        {
            CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_XGMII_EN, 1U);
            status = ENET_SOK;
        }
        else
        {
            /* XGMII in-band mode is not supported in hardware */
            ENETTRACE_ERR("MAC %u: XGMII in-band mode is not supported\n", portId);
            status = ENET_ENOTSUPPORTED;
        }
    }
#endif
    else
    {
        /* Q/SGMII has a dedicated function, called if features is enabled */
        status = ENET_ENOTSUPPORTED;
    }

    /* Half-duplex mode is supported only at 10/100 Mbps */
    if (status == ENET_SOK)
    {
        if ((linkCfg->duplexity == ENET_DUPLEX_HALF) &&
            (linkCfg->speed != ENET_SPEED_10MBIT) &&
            (linkCfg->speed != ENET_SPEED_100MBIT))
        {
            ENETTRACE_ERR("MAC %u: 1Gbps half-duplex is not supported\n", portId);
            status = ENET_EINVALIDPARAMS;
        }
    }

    /* Set duplexity, speed related fields in MAC control */
    if (status == ENET_SOK)
    {
        if (forced && (linkCfg->duplexity == ENET_DUPLEX_FULL))
        {
            CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_FULLDUPLEX, 1U);
        }

        /* Set interface layer, sublayer, speed */
        CSL_CPGMAC_SL_setMacControlReg(regs, portNum, macControl);
    }

    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: Failed to enable MAC port: %d\n", portId, status);

    return status;
}

static void CpswMacPort_setShortIpgCfg(CSL_Xge_cpswRegs *regs,
                                       Enet_MacPort macPort,
                                       const CpswMacPort_TxShortIpgCfg *shortIpgCfg)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);

    if (shortIpgCfg->txShortGapLimitEn)
    {
        CSL_CPGMAC_SL_enableTxShortGapLimit(regs, portNum);
    }
    else
    {
        CSL_CPGMAC_SL_disableTxShortGapLimit(regs, portNum);
    }

    if (shortIpgCfg->txShortGapEn)
    {
        CSL_CPGMAC_SL_enableTxShortGap(regs, portNum);
    }
    else
    {
        CSL_CPGMAC_SL_disableTxShortGap(regs, portNum);
    }
}

static void CpswMacPort_getShortIpgCfg(CSL_Xge_cpswRegs *regs,
                                       Enet_MacPort macPort,
                                       CpswMacPort_TxShortIpgCfg *shortIpgCfg)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);

    shortIpgCfg->txShortGapEn = (CSL_CPGMAC_SL_isTxShortGapEnabled(regs, portNum) == TRUE);
    shortIpgCfg->txShortGapLimitEn = (CSL_CPGMAC_SL_isTxShortGapLimitEnabled(regs, portNum) == TRUE);
}






