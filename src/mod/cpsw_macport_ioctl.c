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

/*! \brief Rate limiting count to transfer rate divider. */
#define CPSW_MACPORT_RATELIM_DIV              (32768U)

/*! \brief PTP sequence ID offset minimum value. */
#define CPSW_MACPORT_PTP_SEQID_OFFSET_MIN_VAL (0x6U)

/*! \brief Maximum Priority for Cut-thru*/
#define CPSW_MACPORT_MAXIMUM_PRIORITY_CUT_THRU (255U)

#define CPSW_MACPORT_MAXIMUM_CPSW_FREQUENCY (1023U)

/*! \brief Port Speed Auto Enable Mask*/
#define CPSW_MACPORT_SPEED_AUTO_ENABLE_MASK (0x00000001)

#if ENET_CFG_IS_ON(CPSW_MACPORT_TRAFFIC_SHAPING)
static uint64_t CpswMacPort_mapBwToCnt(uint64_t rateInBps,
                                       uint32_t cppiClkFreqHz);

static uint64_t CpswMacPort_mapCntToBw(uint32_t cntVal,
                                       uint32_t cppiClkFreqHz);

static int32_t CpswMacPort_setTrafficShaping(CSL_Xge_cpswRegs *regs,
                                             Enet_MacPort macPort,
                                             const EnetPort_TrafficShapingCfg *cfg,
                                             uint32_t cppiClkFreqHz);

static void CpswMacPort_disableTrafficShaping(CSL_Xge_cpswRegs *regs,
                                              Enet_MacPort macPort);

static int32_t CpswMacPort_getTrafficShaping(CSL_Xge_cpswRegs *regs,
                                             Enet_MacPort macPort,
                                             EnetPort_TrafficShapingCfg *cfg,
                                             uint32_t cppiClkFreqHz);
#endif

#if ENET_CFG_IS_ON(CPSW_IET_INCL)
static void CpswMacPort_enableIET(CSL_Xge_cpswRegs *regs, uint32_t portNum);

static void CpswMacPort_disableIET(CSL_Xge_cpswRegs *regs, uint32_t portNum);

static void CpswMacPort_isPreemptionEnabled(CSL_Xge_cpswRegs *regs, uint32_t portNum, bool *enabled);

static void CpswMacPort_isPreemptionActive(CSL_Xge_cpswRegs *regs, uint32_t portNum, bool *active);

static int32_t CpswMacPort_enableIetVerify(CSL_Xge_cpswRegs *regs,
                                           uint32_t portNum,
                                           Enet_Speed speed);

static void CpswMacPort_disableIetVerify(CSL_Xge_cpswRegs *regs, uint32_t portNum);

static bool CpswMacPort_isPremptVerifyEnabled(CSL_Xge_cpswRegs *regs, uint32_t portNum);

static void CpswMacPort_getPremptVerifyStatus(CSL_Xge_cpswRegs *regs,
                                              uint32_t portNum,
                                              EnetMacPort_PreemptVerifyStatus *verifyStatus);

static int32_t CpswMacPort_setPreemptQueue(CSL_Xge_cpswRegs *regs,
                                           uint32_t portNum,
                                           EnetMacPort_QueuePreemptCfg *queuePreemptCfg);

static void CpswMacPort_getQueuePreemptStatus(CSL_Xge_cpswRegs *regs,
                                              uint32_t portNum,
                                              EnetMacPort_QueuePreemptCfg *queuePreemptCfg);
#endif

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

#if ENET_CFG_IS_ON(CPSW_CUTTHRU)
static int32_t CpswMacPort_setRxCutThruParams(CSL_Xge_cpswRegs *regs,
                                              Enet_MacPort macPort,
                                              EnetPort_CutThruParams *rxCutThruParams);

static int32_t CpswMacPort_getRxCutThruParams(CSL_Xge_cpswRegs *regs,
                                              Enet_MacPort macPort,
                                              EnetPort_CutThruParams *cutThruParams);

static int32_t CpswMacPort_setTxCutThruParams(CSL_Xge_cpswRegs *regs,
                                              Enet_MacPort macPort,
                                              EnetPort_CutThruParams *txCutThruParams);

static int32_t CpswMacPort_getTxCutThruParams(CSL_Xge_cpswRegs *regs,
                                              Enet_MacPort macPort,
                                              EnetPort_CutThruParams *cutThruParams);

static int32_t CpswMacPort_setPortSpeedCutThru(CSL_Xge_cpswRegs *regs,
                                              Enet_MacPort macPort,
                                              EnetPort_CutThruParams *cutThruParams);

static int32_t CpswMacPort_getPortSpeedCutThru(CSL_Xge_cpswRegs *regs,
                                               Enet_MacPort macPort,
                                               EnetPort_CutThruParams *cutThruParams);
#endif

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
    CSL_CPSW_VERSION ver = {0};
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
    Enet_MacPort macPort = hPort->macPort;
    uint32_t cppiClkFreqHz;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    cppiClkFreqHz = EnetSoc_getClkFreq(hPort->enetType, hPort->instId, CPSW_CPPI_CLK);
    status = (cppiClkFreqHz != 0) ? ENET_SOK : ENET_EINVALIDPARAMS;
    if (status == ENET_SOK)
    {
        status = CpswMacPort_setTrafficShaping(regs, macPort, &inArgs->trafficShapingCfg, cppiClkFreqHz);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set traffic shaping: %d\n", status);
    }

#else
     status = ENET_ENOTSUPPORTED;
#endif
    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_DISABLE_EGRESS_TRAFFIC_SHAPING(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(CPSW_MACPORT_TRAFFIC_SHAPING)
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    Enet_MacPort macPort = hPort->macPort;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

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
    Enet_MacPort macPort = hPort->macPort;
    uint32_t cppiClkFreqHz;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

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

#if ENET_CFG_IS_ON(CPSW_CUTTHRU)
int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_CUT_THRU_PARAMS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
    EnetMacPort_CutThruParams *inArgs =
        (EnetMacPort_CutThruParams *)prms->inArgs;
    const Enet_MacPort macPort = hPort->macPort;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    status = CpswMacPort_setRxCutThruParams(regs, macPort, &inArgs->cutThruCfg);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: Failed to set RX Cut-thru Params: %d\n", ENET_MACPORT_ID(macPort), status);

    if (status == ENET_SOK)
    {
        status = CpswMacPort_setTxCutThruParams(regs, macPort, &inArgs->cutThruCfg);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: Failed to set TX Cut-thru Params: %d\n", ENET_MACPORT_ID(macPort), status);
    }

    if (status == ENET_SOK)
    {
        status = CpswMacPort_setPortSpeedCutThru(regs, macPort, &inArgs->cutThruCfg);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "MAC %u: Failed to set Port Speed: %d\n", ENET_MACPORT_ID(macPort), status);
    }

    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_CUT_THRU_PARAMS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    EnetPort_CutThruParams *cutThruParams = (EnetPort_CutThruParams *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    status = CpswMacPort_getRxCutThruParams(regs, macPort, cutThruParams);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: Failed to get RX Cut-thru Params: %d\n", ENET_MACPORT_ID(macPort), status);

    status = CpswMacPort_getTxCutThruParams(regs, macPort, cutThruParams);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: Failed to get TX Cut-thru Params: %d\n", ENET_MACPORT_ID(macPort), status);

    status = CpswMacPort_getPortSpeedCutThru(regs, macPort, cutThruParams);
    ENETTRACE_ERR_IF(status != ENET_SOK,
                     "MAC %u: Failed to get Port Speed: %d\n", ENET_MACPORT_ID(macPort), status);
    return status;
}
#endif

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

#if ENET_CFG_IS_ON(CPSW_IET_INCL)

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_ENABLE_PREEMPTION(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    Enet_MacPort macPort = hPort->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    CpswMacPort_enableIET(regs, portNum);

    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_DISABLE_PREEMPTION(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    Enet_MacPort macPort = hPort->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    CpswMacPort_disableIET(regs, portNum);

    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_PREEMPTION_ENABLE_STATUS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    bool *enabled = (bool *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    CpswMacPort_isPreemptionEnabled(regs, portNum, enabled);

    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_PREEMPTION_ACTIVE_STATUS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    bool *active = (bool *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);
    CpswMacPort_isPreemptionActive(regs, portNum, active);

    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_ENABLE_PREEMPT_VERIFICATION(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    Enet_MacPort macPort = hPort->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;
    Enet_Speed speed = hPort->linkCfg.speed;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);
    CpswMacPort_enableIetVerify(regs, portNum, speed);

    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_DISABLE_PREEMPT_VERIFICATION(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    Enet_MacPort macPort = hPort->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);
    CpswMacPort_disableIetVerify(regs, portNum);

    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_PREEMPT_VERIFY_STATUS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    EnetMacPort_PreemptVerifyStatus *verifyStatus = (EnetMacPort_PreemptVerifyStatus *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);
    if (CpswMacPort_isPremptVerifyEnabled(regs, portNum))
    {
        CpswMacPort_getPremptVerifyStatus(regs, portNum, verifyStatus);
    }
    else
    {
        *verifyStatus = ENET_MAC_VERIFYSTATUS_DISABLED;
    }

    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_PREEMPT_MIN_FRAG_SIZE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_SetPreemptMinFragSizeInArgs *inArgs = (EnetMacPort_SetPreemptMinFragSizeInArgs *)prms->inArgs;
    Enet_MacPort macPort = hPort->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    CSL_FINS(regs->ENETPORT[portNum].PN_IET_CONTROL_REG,
                 XGE_CPSW_PN_IET_CONTROL_REG_MAC_ADDFRAGSIZE, inArgs->preemptMinFragSize);

    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_PREEMPT_MIN_FRAG_SIZE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    uint8_t *preemptMinFragSize = (uint8_t *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    *preemptMinFragSize   = CSL_FEXT(regs->ENETPORT[portNum].PN_IET_CONTROL_REG,
                                        XGE_CPSW_PN_IET_CONTROL_REG_MAC_ADDFRAGSIZE);

    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_PREEMPT_QUEUE(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_SetPreemptQueueInArgs *inArgs = (EnetMacPort_SetPreemptQueueInArgs *)prms->inArgs;
    Enet_MacPort macPort = hPort->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    status = CpswMacPort_setPreemptQueue(regs, portNum, &inArgs->queuePreemptCfg);
    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_QUEUE_PREEMPT_STATUS(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    EnetMacPort_QueuePreemptCfg *queuePreemptCfg = (EnetMacPort_QueuePreemptCfg *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    CpswMacPort_getQueuePreemptStatus(regs, portNum, queuePreemptCfg);
    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_IET_HOLD_PREEMPT_TRAFFIC(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    Enet_MacPort macPort = hPort->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    CSL_FINS(regs->ENETPORT[portNum].PN_IET_CONTROL_REG,
                     XGE_CPSW_PN_IET_CONTROL_REG_MAC_HOLD, 1U);
    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_IET_RELEASE_PREEMPT_TRAFFIC(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    Enet_MacPort macPort = hPort->macPort;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    CSL_FINS(regs->ENETPORT[portNum].PN_IET_CONTROL_REG,
                     XGE_CPSW_PN_IET_CONTROL_REG_MAC_HOLD, 0U);
    return status;
}
#endif

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_SET_CREDIT_BASED_SHAPING(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(CPSW_MACPORT_TRAFFIC_SHAPING)
    const EnetMacPort_CreditBasedShaperInArgs *inArgs =
        (const EnetMacPort_CreditBasedShaperInArgs *)prms->inArgs;
    Enet_MacPort macPort = hPort->macPort;
    uint32_t cppiClkFreqHz,i;
    EnetPort_TrafficShapingCfg trafficShapingCfg;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    cppiClkFreqHz = EnetSoc_getClkFreq(hPort->enetType, hPort->instId, CPSW_CPPI_CLK);
    status = (cppiClkFreqHz != 0) ? ENET_SOK : ENET_EINVALIDPARAMS;
    if (status == ENET_SOK)
    {
        for (i = 0U; i < ENET_PRI_NUM; i++)
        {
            trafficShapingCfg.rates[i].committedRateBitsPerSec = inArgs->cbsCfg.idleSlope[i];
            trafficShapingCfg.rates[i].excessRateBitsPerSec = 0U;
        }

        status = CpswMacPort_setTrafficShaping(regs, macPort, &trafficShapingCfg, cppiClkFreqHz);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set Credit based shaping: %d\n", status);
    }

#else
     status = ENET_ENOTSUPPORTED;
#endif
    return status;
}

int32_t CpswMacPort_ioctl_handler_ENET_MACPORT_IOCTL_GET_CREDIT_BASED_SHAPING(CpswMacPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(CPSW_MACPORT_TRAFFIC_SHAPING)
    EnetMacPort_GenericInArgs *inArgs = (EnetMacPort_GenericInArgs *)prms->inArgs;
    EnetPort_CreditBasedShapingCfg *cbsCfg = (EnetPort_CreditBasedShapingCfg *)prms->outArgs;
    Enet_MacPort macPort = hPort->macPort;
    uint32_t cppiClkFreqHz,i;
    EnetPort_TrafficShapingCfg trafficShapingCfg;

    Enet_devAssert(macPort == inArgs->macPort,
                   "MAC %u: Port mismatch %u\n", ENET_MACPORT_ID(macPort), inArgs->macPort);

    cppiClkFreqHz = EnetSoc_getClkFreq(hPort->enetType, hPort->instId, CPSW_CPPI_CLK);

    status = CpswMacPort_getTrafficShaping(regs, macPort, &trafficShapingCfg, cppiClkFreqHz);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get traffic shaping: %d\n", status);

    for (i = 0U; i < ENET_PRI_NUM; i++)
    {
        cbsCfg->idleSlope[i] = trafficShapingCfg.rates[i].committedRateBitsPerSec;
    }
#else
     status = ENET_ENOTSUPPORTED;
#endif
    return status;
}

#if ENET_CFG_IS_ON(CPSW_MACPORT_TRAFFIC_SHAPING)
static uint64_t CpswMacPort_mapBwToCnt(uint64_t rateInBps,
                                       uint32_t cppiClkFreqHz)
{
    /*
     * Transfer rate to CIR or EIR count value equation:
     *
     *                Transfer rate (Mbps) * 32768
     * Count value = ------------------------------
     *                   CPPI_ICLK freq (MHz)
     *
     * Round division to prevent transfer rates below traffic shaping resolution
     * end up with no shaping at all.
     */
    return ENET_DIV_ROUNDUP(rateInBps * CPSW_MACPORT_RATELIM_DIV, cppiClkFreqHz);
}

static uint64_t CpswMacPort_mapCntToBw(uint32_t cntVal,
                                       uint32_t cppiClkFreqHz)
{
    /*
     * CIR or EIR count value to transfer rate equation:
     *
     *                         count value * CPPI_ICLK freq (MHz)
     * Transfer rate (Mbps) = ------------------------------------
     *                                      32768
     */
    return ((uint64_t)cntVal * cppiClkFreqHz) / CPSW_MACPORT_RATELIM_DIV;
}

static int32_t CpswMacPort_setTrafficShaping(CSL_Xge_cpswRegs *regs,
                                             Enet_MacPort macPort,
                                             const EnetPort_TrafficShapingCfg *cfg,
                                             uint32_t cppiClkFreqHz)
{
    uint64_t cirBps, eirBps;
    uint64_t cir, eir;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    uint32_t i;
    int32_t status= ENET_SOK;
    bool rateLimEnabled = false;

    ENETTRACE_VAR(portId);
    for (i = 0U; i < ENET_PRI_NUM; i++)
    {
        cirBps = cfg->rates[i].committedRateBitsPerSec;
        eirBps = cfg->rates[i].excessRateBitsPerSec;
        cir = CpswMacPort_mapBwToCnt(cirBps, cppiClkFreqHz);
        eir = CpswMacPort_mapBwToCnt(eirBps, cppiClkFreqHz);

        /* CIR must be non-zero if EIR is non-zero */
        if ((cir == 0ULL) && (eir != 0ULL))
        {
            ENETTRACE_ERR("MAC %u: EIR is enabled (%ubps = %llu) but CIR is not (%ubps = %llu) "
                          "for priority %u\n",
                          portId, eirBps, eir, cirBps, cir, i);
            status = ENET_EINVALIDPARAMS;
            break;
        }

        /* Rate limit must be the highest priority channels */
        if ((cir == 0ULL) && rateLimEnabled)
        {
            ENETTRACE_ERR("MAC %u: Highest priority queues should be rate limited, "
                          "rate limiting disabled for priority %u\n", portId, i);
            status = ENET_EINVALIDPARAMS;
            break;
        }

        /* Out of range */
        if ((cir > 0x0FFFFFFFULL) || (eir > 0x0FFFFFFFULL))
        {
            ENETTRACE_ERR("MAC %u: invalid CIR=%llu (%llubps) EIR=%llu (%llubps) for priority %u\n",
                          portId, cir, cirBps, eir, eirBps, i);
            status = ENET_EINVALIDPARAMS;
            break;
        }

        ENETTRACE_DBG("MAC %u: rate limiting %s for priority %u, CIR=%llubps (%llu) EIR=%llubps (%llu)\n",
                      portId, (cir != 0ULL) ? "enabled" : "disabled",
                      i, cirBps, cir, eirBps, eir);

        CSL_CPSW_setPriCirEir(regs, portNum, i, (uint32_t)cir, (uint32_t)eir);

        if (cir != 0ULL)
        {
            rateLimEnabled = true;
        }
    }

    return status;
}

static void CpswMacPort_disableTrafficShaping(CSL_Xge_cpswRegs *regs,
                                              Enet_MacPort macPort)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t i;

    for (i = 0U; i < ENET_PRI_NUM; i++)
    {
        CSL_CPSW_setPriCirEir(regs, portNum, i, 0U, 0U);
    }
}

static int32_t CpswMacPort_getTrafficShaping(CSL_Xge_cpswRegs *regs,
                                             Enet_MacPort macPort,
                                             EnetPort_TrafficShapingCfg *cfg,
                                             uint32_t cppiClkFreqHz)
{
    uint32_t cir, eir;
    uint64_t cirBps, eirBps;
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);
    uint32_t i;
    int32_t status = ENET_SOK;

    ENETTRACE_VAR(portId);
    for (i = 0U; i < ENET_PRI_NUM; i++)
    {
        CSL_CPSW_getPriCirEir(regs, portNum, i, &cir, &eir);

        cirBps = CpswMacPort_mapCntToBw(cir, cppiClkFreqHz);
        eirBps = CpswMacPort_mapCntToBw(eir, cppiClkFreqHz);

        /* CIR must be non-zero if EIR is non-zero */
        if ((cirBps == 0ULL) && (eirBps != 0ULL))
        {
            ENETTRACE_ERR("MAC %u: EIR is enabled (%llubps = %u) but CIR is not (%llubps = %u) "
                          "for priority %u\n",
                          portId, eirBps, eir, cirBps, cir, i);
            status = ENET_EUNEXPECTED;
            break;
        }

        ENETTRACE_DBG("MAC %u: rate limiting %s for priority %u, CIR=%llubps (%u) EIR=%llubps (%u)\n",
                      portId, (cir != 0U) ? "enabled" : "disabled",
                      i, cirBps, cir, eirBps, eir);

        cfg->rates[i].committedRateBitsPerSec = cirBps;
        cfg->rates[i].excessRateBitsPerSec = eirBps;
    }

    return status;
}
#endif

static void CpswMacPort_printRegs(CSL_Xge_cpswRegs *regs,
                                  Enet_MacPort macPort)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    uint32_t portId = ENET_MACPORT_ID(macPort);

    if (portNum < CSL_ARRAYSIZE(regs->ENETPORT))
    {
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

#if ENET_CFG_IS_ON(CPSW_CUTTHRU)
static int32_t CpswMacPort_setRxCutThruParams(CSL_Xge_cpswRegs *regs,
                                              Enet_MacPort macPort,
                                              EnetPort_CutThruParams *cutThruParams)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    if (cutThruParams->rxPriCutThruEn > CPSW_MACPORT_MAXIMUM_PRIORITY_CUT_THRU)
    {
        ENETTRACE_ERR("MAC %u: Invalid priority -> %u\n", ENET_MACPORT_ID(macPort), cutThruParams->rxPriCutThruEn);
        status = ENET_EINVALIDPARAMS;
    }

    if (portNum < ENET_ARRAYSIZE(regs->ENETPORT))
    {
        if (status == ENET_SOK)
        {
            CSL_CPSW_setPortRxCutThruPri(regs, portNum, cutThruParams->rxPriCutThruEn);
        }
    }

    return status;
}

static int32_t CpswMacPort_getRxCutThruParams(CSL_Xge_cpswRegs *regs,
                                              Enet_MacPort macPort,
                                              EnetPort_CutThruParams *cutThruParams)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    if (portNum < ENET_ARRAYSIZE(regs->ENETPORT))
    {
        CSL_CPSW_getPortRxCutThruPri(regs, portNum, &cutThruParams->rxPriCutThruEn);
    }
    else
    {
        status = ENET_EINVALIDPARAMS;
    }

    return status;
}

static int32_t CpswMacPort_setTxCutThruParams(CSL_Xge_cpswRegs *regs,
                                              Enet_MacPort macPort,
                                              EnetPort_CutThruParams *cutThruParams)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    if (cutThruParams->txPriCutThruEn > CPSW_MACPORT_MAXIMUM_PRIORITY_CUT_THRU)
    {
        ENETTRACE_ERR("MAC %u: Invalid priority -> %u\n", ENET_MACPORT_ID(macPort), cutThruParams->txPriCutThruEn);
        status = ENET_EINVALIDPARAMS;
    }

    if (status == ENET_SOK)
    {
        CSL_CPSW_setPortTxCutThruPri(regs, portNum, cutThruParams->txPriCutThruEn);
    }

    return status;
}

static int32_t CpswMacPort_getTxCutThruParams(CSL_Xge_cpswRegs *regs,
                                              Enet_MacPort macPort,
                                              EnetPort_CutThruParams *cutThruParams)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    if (portNum < ENET_ARRAYSIZE(regs->ENETPORT))
    {
        CSL_CPSW_getPortTxCutThruPri(regs, portNum, &cutThruParams->txPriCutThruEn);
    }
    else
    {
        status = ENET_EINVALIDPARAMS;
    }

    return status;
}

static int32_t CpswMacPort_setPortSpeedCutThru(CSL_Xge_cpswRegs *regs,
                                               Enet_MacPort macPort,
                                               EnetPort_CutThruParams *cutThruParams)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    if (cutThruParams->portSpeedAutoEn == 1U)
    {
        CSL_CPSW_setPortSpeedAutoEnable(regs, portNum, cutThruParams->portSpeedAutoEn);
    }
    else
    {
        ENETTRACE_ERR("MAC %u: Auto enable Port Speed not set-> %u\n", ENET_MACPORT_ID(macPort), cutThruParams->portSpeedAutoEn);
        status = ENET_EINVALIDPARAMS;
    }

    return status;
}

static int32_t CpswMacPort_getPortSpeedCutThru(CSL_Xge_cpswRegs *regs,
                                               Enet_MacPort macPort,
                                               EnetPort_CutThruParams *cutThruParams)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    int32_t status = ENET_SOK;

    if (portNum < ENET_ARRAYSIZE(regs->ENETPORT))
    {
        CSL_CPSW_getPortSpeedAutoEnable(regs, portNum, &cutThruParams->portSpeedAutoEn);
    }
    else
    {
        status = ENET_EINVALIDPARAMS;
    }

    return status;
}
#endif

static void CpswMacPort_getLinkCfg(CSL_Xge_cpswRegs *regs,
                                   Enet_MacPort macPort,
                                   EnetMacPort_LinkCfg *linkCfg)
{
    uint32_t portNum = ENET_MACPORT_NORM(macPort);
    CSL_CPGMAC_SL_MACSTATUS macStatus = {0};

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
    CSL_CPSW_THRURATE thruRate = {0};
    CSL_CPGMAC_SL_FIFOSTATUS fifoStatus = {0};
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

    if (EnetMacPort_isMii(mii))
    {
#if ENET_CFG_IS_ON(CPSW_MACPORT_MII)
        /* In RMII mode, ifctl_a bit determines the RMII link speed (0=10mbps, 1=100mbps) */
        if ((linkCfg->speed == ENET_SPEED_100MBIT) ||
            (linkCfg->speed == ENET_SPEED_10MBIT))
        {
            CSL_FINS(macControl, XGE_CPSW_PN_MAC_CONTROL_REG_GMII_EN, 1U);
            status = ENET_SOK;
        }
        else
        {
            ENETTRACE_ERR("MAC %u: Invalid speed for MII mode\n", portId);
            status = ENET_EINVALIDPARAMS;
        }

        forced = true;
#else
        status = ENET_ENOTSUPPORTED;
#endif

    }
    else if (EnetMacPort_isRmii(mii))
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

#if ENET_CFG_IS_ON(CPSW_IET_INCL)
static void CpswMacPort_enableIET(CSL_Xge_cpswRegs *regs, uint32_t portNum)
{
    CSL_CPSW_PORT_CONTROL portControl;

    CSL_CPSW_setPortMaxBlksReg(regs, portNum + 1U, CPSW_MACPORT_RX_MAX_BLKS_IET, CPSW_MACPORT_TX_MAX_BLKS_IET);
    memset(&portControl, 0, sizeof(portControl));
    /* Set IET enable in Port Control reg pn_iet_port_en */
    CSL_CPSW_getPortControlReg(regs, portNum + 1U, &portControl);
    portControl.ietPortEnable = 1U;
    CSL_CPSW_setPortControlReg(regs, portNum + 1U, &portControl);

    /* Enable mac preemption in port IET control reg */
    CSL_FINS(regs->ENETPORT[portNum].PN_IET_CONTROL_REG,
                 XGE_CPSW_PN_IET_CONTROL_REG_MAC_PENABLE, 1U);
}

static void CpswMacPort_disableIET(CSL_Xge_cpswRegs *regs, uint32_t portNum)
{
    CSL_CPSW_PORT_CONTROL portControl;

    /* Set the Port Max Blks to default values */
    CSL_CPSW_setPortMaxBlksReg(regs, portNum + 1U, CPSW_MACPORT_RX_MAX_BLKS_DEFAULT, CPSW_MACPORT_TX_MAX_BLKS_DEFAULT);
    memset(&portControl, 0, sizeof(portControl));
    /* Disable IET enable in Port Control reg pn_iet_port_en */
    CSL_CPSW_getPortControlReg(regs, portNum + 1U, &portControl);
    portControl.ietPortEnable = 0U;
    CSL_CPSW_setPortControlReg(regs, portNum + 1U, &portControl);

    /* Disable mac preemption in port IET control reg */
    CSL_FINS(regs->ENETPORT[portNum].PN_IET_CONTROL_REG,
                 XGE_CPSW_PN_IET_CONTROL_REG_MAC_PENABLE, 0U);
}

static void CpswMacPort_isPreemptionEnabled(CSL_Xge_cpswRegs *regs, uint32_t portNum, bool *enabled)
{
    /* Check if the enable bits are set */
    *enabled = CSL_FEXT(regs->ENETPORT[portNum].PN_IET_CONTROL_REG, XGE_CPSW_PN_IET_CONTROL_REG_MAC_PENABLE) &
                CSL_FEXT(regs->CONTROL_REG, XGE_CPSW_CONTROL_REG_IET_ENABLE);
}

static void CpswMacPort_isPreemptionActive(CSL_Xge_cpswRegs *regs, uint32_t portNum, bool *active)
{
    CSL_CPSW_IET_CONFIG ietControl;
    bool enabled = false;
    *active = false;

    /*
     * Packets will be sent to the prempt MAC only
     * when pn_mac_penable is set, and when mac_verified (from Enet_Pn_IET_Status)
     * or pn_mac_disableverify is set, and when pn_iet_port_en is set.
     */
    CpswMacPort_isPreemptionEnabled(regs, portNum, &enabled);
    if(enabled)
    {
        memset(&ietControl, 0, sizeof(ietControl));
        CSL_CPSW_getPortIetControlReg(regs, portNum + 1U, &ietControl);
        if(ietControl.macDisableVerify == 1U)
        {
            *active = true;
        }
        else
        {
            *active = CSL_FEXT(regs->ENETPORT[portNum].PN_IET_STATUS_REG,
                                    XGE_CPSW_PN_IET_STATUS_REG_MAC_VERIFIED);
        }
    }

}

static int32_t CpswMacPort_enableIetVerify(CSL_Xge_cpswRegs *regs,
                                           uint32_t portNum,
                                           Enet_Speed speed)
{
    uint32_t timeIntvlStep;
    uint32_t verifyTimeoutNs;
    uint32_t timeoutCnt;
    CSL_CPSW_IET_CONFIG ietControl;
    int32_t status = ENET_SOK;

    switch (speed)
    {
        case ENET_SPEED_10MBIT:
            timeIntvlStep = CPSW_MACPORT_EST_TIME_STEP_10M;
            verifyTimeoutNs = 100 * CPSW_MACPORT_GIGABIT_IET_VERIFY_TIMEOUT_NS;
            break;
        case ENET_SPEED_100MBIT:
            timeIntvlStep = CPSW_MACPORT_EST_TIME_STEP_100M;
            verifyTimeoutNs = 10 * CPSW_MACPORT_GIGABIT_IET_VERIFY_TIMEOUT_NS;
            break;
        case ENET_SPEED_1GBIT:
        default:
            timeIntvlStep = CPSW_MACPORT_EST_TIME_STEP_1G;
            verifyTimeoutNs = CPSW_MACPORT_GIGABIT_IET_VERIFY_TIMEOUT_NS;
            break;
    }

    timeoutCnt = ENET_DIV_ROUNDUP(verifyTimeoutNs, timeIntvlStep);
    CSL_CPSW_setPortIetVerifyTimeout(regs, portNum + 1U, timeoutCnt);

    /*
     * Toggle macLinkFail bit to restart verification process.
     * i.e Set macLinkFail to 1 and make it to 0 to reset and start,
     * It needs to be set only at least for one clock cycle.
     */
    memset(&ietControl, 0, sizeof(ietControl));
    CSL_CPSW_getPortIetControlReg(regs, portNum + 1U, &ietControl);
    ietControl.macDisableVerify = 0U;
    ietControl.macPremptEnable = 1U;
    ietControl.macLinkFail = 1U;
    CSL_CPSW_setPortIetControlReg(regs, portNum + 1U, &ietControl);
    memset(&ietControl, 0, sizeof(ietControl));
    CSL_CPSW_getPortIetControlReg(regs, portNum + 1U, &ietControl);
    ietControl.macLinkFail = 0U;
    CSL_CPSW_setPortIetControlReg(regs, portNum + 1U, &ietControl);
    return status;
}

static void CpswMacPort_disableIetVerify(CSL_Xge_cpswRegs *regs, uint32_t portNum)
{
    /* This is forced mode with no IET verification. */
    CSL_FINS(regs->ENETPORT[portNum].PN_IET_CONTROL_REG,
                 XGE_CPSW_PN_IET_CONTROL_REG_MAC_DISABLEVERIFY, 1U);
}

static bool CpswMacPort_isPremptVerifyEnabled(CSL_Xge_cpswRegs *regs, uint32_t portNum)
{
    CSL_CPSW_IET_CONFIG ietControl;
    bool enabled = false;

    /* Check if preemption is enabled */
    CpswMacPort_isPreemptionEnabled(regs, portNum, &enabled);
    if(enabled)
    {
        /* Verify and response frames will be sent/allowed only
         * when macLinkFail is set to zero and macDisableVerify is set to zero.*/
        memset(&ietControl, 0, sizeof(ietControl));
        CSL_CPSW_getPortIetControlReg(regs, portNum + 1U, &ietControl);
        if( (ietControl.macDisableVerify == 1U) || (ietControl.macLinkFail == 1U) )
        {
            enabled = false;
        }
    }
    return enabled;
}

static void CpswMacPort_getPremptVerifyStatus(CSL_Xge_cpswRegs *regs,
                                              uint32_t portNum,
                                              EnetMacPort_PreemptVerifyStatus *verifyStatus)
{
    CSL_CPSW_IET_STATUS ietStatus;

    memset(&ietStatus, 0, sizeof(ietStatus));
    CSL_CPSW_PortIetStatus(regs, portNum + 1U, &ietStatus);

    if (ietStatus.macVerified == 1U)
    {
        *verifyStatus = ENET_MAC_VERIFYSTATUS_SUCCEEDED;
    }
    else if (ietStatus.macRxRespondErr == 1U)
    {
        *verifyStatus = ENET_MAC_VERIFYSTATUS_RXRESPOND_ERROR;
    }
    else if (ietStatus.macRxVerifyErr == 1U)
    {
        *verifyStatus = ENET_MAC_VERIFYSTATUS_RXVERIFY_ERROR;
    }
    else if (ietStatus.macVerifyFail == 1U)
    {
        *verifyStatus = ENET_MAC_VERIFYSTATUS_FAILED;
    }
    else
    {
        *verifyStatus = ENET_MAC_VERIFYSTATUS_UNKNOWN;
    }
}

static int32_t CpswMacPort_setPreemptQueue(CSL_Xge_cpswRegs *regs,
                                           uint32_t portNum,
                                           EnetMacPort_QueuePreemptCfg *queuePreemptCfg)
{
    int32_t status = ENET_SOK;
    uint32_t queueIdx;
    uint32_t queueMask = 0;

    for (queueIdx = 0U; queueIdx < ENET_PRI_NUM; queueIdx++)
    {
        if(queuePreemptCfg->preemptMode[queueIdx] == ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT)
        {
            queueMask |= (1U << queueIdx);
        }
        else if(queuePreemptCfg->preemptMode[queueIdx] == ENET_MAC_QUEUE_PREEMPT_MODE_EXPRESS)
        {
            continue;
        }
        else
        {
            status = ENET_EINVALIDPARAMS;
            break;
        }
    }

    if (status == ENET_SOK)
    {
        CSL_FINS(regs->ENETPORT[portNum].PN_IET_CONTROL_REG,
                        XGE_CPSW_PN_IET_CONTROL_REG_MAC_PREMPT, queueMask);
    }

    return status;

}

static void CpswMacPort_getQueuePreemptStatus(CSL_Xge_cpswRegs *regs,
                                              uint32_t portNum,
                                              EnetMacPort_QueuePreemptCfg *queuePreemptCfg)
{
    uint32_t queueMask = 0;
    uint32_t queueIdx;

    queueMask = CSL_FEXT(regs->ENETPORT[portNum].PN_IET_CONTROL_REG,
                            XGE_CPSW_PN_IET_CONTROL_REG_MAC_PREMPT);

    for (queueIdx = 0U; queueIdx < ENET_PRI_NUM; queueIdx++)
    {
        if( (queueMask & (1U << queueIdx)) != 0U)
        {
            queuePreemptCfg->preemptMode[queueIdx] = ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT;
        }
        else
        {
            queuePreemptCfg->preemptMode[queueIdx] = ENET_MAC_QUEUE_PREEMPT_MODE_EXPRESS;
        }
    }

}
#endif
