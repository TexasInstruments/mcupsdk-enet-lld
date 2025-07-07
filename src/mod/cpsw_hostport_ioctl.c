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
 * \file  cpsw_hostport.c
 *
 * \brief This file contains the implementation of the CPSW host port module.
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
#include <include/mod/cpsw_hostport.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/mod/cpsw_hostport_priv.h>
#include <priv/mod/cpsw_hostport_ioctl_priv.h>
#include <priv/mod/cpsw_clks.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/*! \brief Host port start offset in CSL register overlay. */
#define CPSW_HOSTPORT_START_REG_OFFSET        (CSL_XGE_CPSW_P0_CONTROL_REG)

/*! \brief Host port end offset in CSL register overlay. */
#define CPSW_HOSTPORT_END_REG_OFFSET          (CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG)

/*! \brief Rate limiting count to transfer rate divider. */
#define CPSW_HOSTPORT_RATELIM_DIV             (32768U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static void CpswHostPort_printRegs(CSL_Xge_cpswRegs *regs);
static int32_t CpswHostPort_setDscpPriority(CSL_Xge_cpswRegs *regs,
                                            EnetPort_DscpPriorityMap *dscpPriority);
static void CpswHostPort_getDscpPriority(CSL_Xge_cpswRegs *regs,
                                         EnetPort_DscpPriorityMap *dscpPriority);
static int32_t CpswHostPort_setRxPriority(CSL_Xge_cpswRegs *regs,
                                          EnetPort_PriorityMap *rxPriority);
static void CpswHostPort_getRxPriority(CSL_Xge_cpswRegs *regs,
                                       EnetPort_PriorityMap *rxPriority);
static int32_t CpswHostPort_setTxPriority(CSL_Xge_cpswRegs *regs,
                                          EnetPort_PriorityMap *txPriority);
static void CpswHostPort_getTxPriority(CSL_Xge_cpswRegs *regs,
                                       EnetPort_PriorityMap *txPriority);
static void CpswHostPort_getMaxLen(CSL_Xge_cpswRegs *regs,
                                   EnetPort_MaxLen *maxLen);
static void CpswHostPort_getFifoStats(CSL_Xge_cpswRegs *regs,
                                      CpswHostPort_FifoStats *fifoStats);
#if ENET_CFG_IS_ON(CPSW_HOSTPORT_TRAFFIC_SHAPING)
static uint64_t CpswHostPort_mapBwToCnt(uint64_t rateInBps,
                                        uint32_t cppiClkFreqHz);

static uint64_t CpswHostPort_mapCntToBw(uint32_t cntVal,
                                        uint32_t cppiClkFreqHz);

static int32_t CpswHostPort_setTrafficShaping(CSL_Xge_cpswRegs *regs,
                                              const EnetPort_TrafficShapingCfg *cfg,
                                              uint32_t cppiClkFreqHz);

static void CpswHostPort_disableTrafficShaping(CSL_Xge_cpswRegs *regs);

static int32_t CpswHostPort_getTrafficShaping(CSL_Xge_cpswRegs *regs,
                                              EnetPort_TrafficShapingCfg *cfg,
                                              uint32_t cppiClkFreqHz);
#endif


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_GET_VERSION(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    Enet_Version *version = (Enet_Version *)prms->outArgs;
    CSL_CPSW_VERSION ver = {0};
    int32_t status = ENET_SOK;

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

int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_PRINT_REGS(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CpswHostPort_printRegs(regs);
    return status;
}

int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_ENABLE(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CSL_CPSW_enablePort0(regs);
    return status;
}

int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_DISABLE(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CSL_CPSW_disablePort0(regs);
    return status;
}

int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_SET_INGRESS_DSCP_PRI_MAP(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetPort_DscpPriorityMap *dscpPriority = (EnetPort_DscpPriorityMap *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = CpswHostPort_setDscpPriority(regs, dscpPriority);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set DSCP priority: %d\n", status);
    return status;
}

int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_GET_INGRESS_DSCP_PRI_MAP(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetPort_DscpPriorityMap *dscpPriority = (EnetPort_DscpPriorityMap *)prms->outArgs;
    int32_t status = ENET_SOK;

    CpswHostPort_getDscpPriority(regs, dscpPriority);
    return status;
}

int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_SET_PRI_REGEN_MAP(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetPort_PriorityMap *rxPriority = (EnetPort_PriorityMap *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = CpswHostPort_setRxPriority(regs, rxPriority);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set RX priority: %d\n", status);
    return status;
}

int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_GET_PRI_REGEN_MAP(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetPort_PriorityMap *rxPriority = (EnetPort_PriorityMap *)prms->outArgs;
    int32_t status = ENET_SOK;

    CpswHostPort_getRxPriority(regs, rxPriority);
    return status;
}

int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetPort_PriorityMap *txPriority = (EnetPort_PriorityMap *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = CpswHostPort_setTxPriority(regs, txPriority);
    ENETTRACE_ERR_IF(status != ENET_SOK, "failed to set TX priority: %d\n", status);
    return status;
}

int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_GET_EGRESS_QOS_PRI_MAP(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetPort_PriorityMap *txPriority = (EnetPort_PriorityMap *)prms->outArgs;
    int32_t status = ENET_SOK;

    CpswHostPort_getTxPriority(regs, txPriority);
    return status;
}

int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_ENABLE_INGRESS_TRAFFIC_SHAPING(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(CPSW_HOSTPORT_TRAFFIC_SHAPING)
    EnetPort_TrafficShapingCfg *shapingCfg = (EnetPort_TrafficShapingCfg *)prms->inArgs;
    uint32_t cppiClkFreqHz;

    cppiClkFreqHz = EnetSoc_getClkFreq(hPort->enetType, hPort->instId, CPSW_CPPI_CLK);
    status = (cppiClkFreqHz != 0) ? ENET_SOK : ENET_EINVALIDPARAMS;
    if (status == ENET_SOK)
    {
        status = CpswHostPort_setTrafficShaping(regs, shapingCfg, cppiClkFreqHz);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set traffic shaping: %d\n", status);
    }
#else
    status = ENET_ENOTSUPPORTED;
#endif
    return status;
}

int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_DISABLE_INGRESS_TRAFFIC_SHAPING(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(CPSW_HOSTPORT_TRAFFIC_SHAPING)
    CpswHostPort_disableTrafficShaping(regs);
#else
    status = ENET_ENOTSUPPORTED;
#endif
    return status;
}

int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_GET_INGRESS_TRAFFIC_SHAPING(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(CPSW_HOSTPORT_TRAFFIC_SHAPING)
    EnetPort_TrafficShapingCfg *shapingCfg = (EnetPort_TrafficShapingCfg *)prms->outArgs;
    uint32_t cppiClkFreqHz;

    cppiClkFreqHz = EnetSoc_getClkFreq(hPort->enetType, hPort->instId, CPSW_CPPI_CLK);

    status = CpswHostPort_getTrafficShaping(regs, shapingCfg, cppiClkFreqHz);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get traffic shaping: %d\n", status);
#else
    status = ENET_ENOTSUPPORTED;
#endif
    return status;
}

int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_GET_MAXLEN(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    EnetPort_MaxLen *maxLen = (EnetPort_MaxLen *)prms->inArgs;
    int32_t status = ENET_SOK;

    CpswHostPort_getMaxLen(regs, maxLen);
    return status;
}

int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IS_CSUM_OFFLOAD_ENABLED(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    bool *csumEn = (bool *)prms->outArgs;
    CSL_CPSW_CPPI_P0_CONTROL cppiP0ControlCfg = {0};
    int32_t status = ENET_SOK;

    CSL_CPSW_getCppiP0Control(regs, &cppiP0ControlCfg);
    *csumEn = (cppiP0ControlCfg.p0RxChksumEn != 0U);
    return status;
}

int32_t CpswHostPort_ioctl_handler_CPSW_HOSTPORT_IOCTL_GET_FIFO_STATS(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    CpswHostPort_FifoStats *fifoStats = (CpswHostPort_FifoStats *)prms->outArgs;
    int32_t status = ENET_SOK;

    CpswHostPort_getFifoStats(regs, fifoStats);
    return status;
}

int32_t CpswHostPort_ioctl_handler_CPSW_HOSTPORT_GET_FLOW_ID_OFFSET(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    uint32_t *flowIdOffset = (uint32_t *)prms->outArgs;
    int32_t status = ENET_SOK;

    *flowIdOffset = CSL_CPSW_getPort0FlowIdOffset(regs);
    return status;
}

int32_t CpswHostPort_ioctl_handler_CPSW_HOSTPORT_SET_FLOW_ID_OFFSET(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    uint32_t *flowIdOffset = (uint32_t *)prms->inArgs;
    int32_t status = ENET_SOK;

    CSL_CPSW_setPort0FlowIdOffset(regs, *flowIdOffset);
    return status;
}

int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_SET_CREDIT_BASED_SHAPING(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(CPSW_HOSTPORT_TRAFFIC_SHAPING)
    const EnetPort_CreditBasedShapingCfg *inArgs =
        (const EnetPort_CreditBasedShapingCfg *)prms->inArgs;
    uint32_t cppiClkFreqHz;
    EnetPort_TrafficShapingCfg trafficShapingCfg;

    status = (inArgs->queueNum < ENET_PRI_NUM) ? ENET_SOK : ENET_EINVALIDPARAMS;
    cppiClkFreqHz = EnetSoc_getClkFreq(hPort->enetType, hPort->instId, CPSW_CPPI_CLK);
    status = (cppiClkFreqHz != 0) ? ENET_SOK : ENET_EINVALIDPARAMS;
    if (status == ENET_SOK)
    {
        status = CpswHostPort_getTrafficShaping(regs, &trafficShapingCfg, cppiClkFreqHz);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get traffic shaping: %d\n", status);
    }
    if (status == ENET_SOK)
    {
        trafficShapingCfg.rates[inArgs->queueNum].committedRateBitsPerSec = inArgs->idleSlope;
    }
    status = CpswHostPort_setTrafficShaping(regs, &trafficShapingCfg, cppiClkFreqHz);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set Credit based shaping: %d\n", status);

#else
     status = ENET_ENOTSUPPORTED;
#endif
    return status;
}

int32_t CpswHostPort_ioctl_handler_ENET_HOSTPORT_IOCTL_GET_CREDIT_BASED_SHAPING(CpswHostPort_Handle hPort, CSL_Xge_cpswRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(CPSW_HOSTPORT_TRAFFIC_SHAPING)
    uint32_t *priority = (uint32_t *)prms->inArgs;
    uint64_t *idleSlope = (uint64_t *)prms->outArgs;
    uint32_t cir, eir, cppiClkFreqHz;
    uint64_t cirBps, eirBps;

    cppiClkFreqHz = EnetSoc_getClkFreq(hPort->enetType, hPort->instId, CPSW_CPPI_CLK);
    CSL_CPSW_getCppiPriCirEir(regs, *priority, &cir, &eir);

    cirBps = CpswHostPort_mapCntToBw(cir, cppiClkFreqHz);
    eirBps = CpswHostPort_mapCntToBw(eir, cppiClkFreqHz);

    /* CIR must be non-zero if EIR is non-zero */
    if ((cirBps == 0ULL) && (eirBps != 0ULL))
    {
        ENETTRACE_ERR("HOSTPORT: EIR is enabled (%llubps = %u) but CIR is not (%llubps = %u) "
                      "for priority %u\n", eirBps, eir, cirBps, cir, *priority);
        status = ENET_EUNEXPECTED;
    }

    ENETTRACE_DBG("HOSTPORT: rate limiting %s for priority %u, CIR=%llubps (%u) EIR=%llubps (%u)\n",
                  (cir != 0U) ? "enabled" : "disabled", *priority, cirBps, cir, eirBps, eir);

    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get traffic shaping: %d\n", status);

    *idleSlope = cirBps;

#else
     status = ENET_ENOTSUPPORTED;
#endif
    return status;
}

static void CpswHostPort_printRegs(CSL_Xge_cpswRegs *regs)
{
    uint32_t *regAddr = (uint32_t *)((uintptr_t)regs + CPSW_HOSTPORT_START_REG_OFFSET);
    uint32_t regIdx = 0;

    while ((uintptr_t)regAddr <= ((uintptr_t)regs + CPSW_HOSTPORT_END_REG_OFFSET))
    {
        if (*regAddr != 0U)
        {
            ENETTRACE_INFO("HOSTPORT: %u: 0x%08x\n", regIdx, *regAddr);
        }

        regAddr++;
        regIdx++;
    }
}

static int32_t CpswHostPort_setDscpPriority(CSL_Xge_cpswRegs *regs,
                                            EnetPort_DscpPriorityMap *dscpPriority)
{
    CSL_CPSW_PORT_CONTROL control;
    int32_t status = ENET_SOK;
    uint32_t i;

    for (i = 0U; i < ENET_TOS_PRI_NUM; i++)
    {
        if (dscpPriority->tosMap[i] > ENET_PRI_MAX)
        {
            ENETTRACE_ERR("Invalid priority map %u -> %u\n", i, dscpPriority->tosMap[i]);
            status = ENET_EINVALIDPARAMS;
            break;
        }
    }

    if (status == ENET_SOK)
    {
        CSL_CPSW_setPortRxDscpMap(regs, 0U, dscpPriority->tosMap);

        CSL_CPSW_getPortControlReg(regs, 0U, &control);

        control.dscpIpv4Enable = dscpPriority->dscpIPv4En;
        control.dscpIpv6Enable = dscpPriority->dscpIPv6En;

        CSL_CPSW_setPortControlReg(regs, 0U, &control);
    }

    return status;
}

static void CpswHostPort_getDscpPriority(CSL_Xge_cpswRegs *regs,
                                         EnetPort_DscpPriorityMap *dscpPriority)
{
    CSL_CPSW_PORT_CONTROL control;

    CSL_CPSW_getPortRxDscpMap(regs, 0U, dscpPriority->tosMap);

    CSL_CPSW_getPortControlReg(regs, 0U, &control);

    dscpPriority->dscpIPv4En = control.dscpIpv4Enable;
    dscpPriority->dscpIPv6En = control.dscpIpv6Enable;
}

static int32_t CpswHostPort_setRxPriority(CSL_Xge_cpswRegs *regs,
                                          EnetPort_PriorityMap *rxPriority)
{
    int32_t status = ENET_SOK;
    uint32_t i;

    for (i = 0U; i < ENET_PRI_NUM; i++)
    {
        if (rxPriority->priorityMap[i] > ENET_PRI_MAX)
        {
            ENETTRACE_ERR("Invalid priority map %u -> %u\n", i, rxPriority->priorityMap[i]);
            status = ENET_EINVALIDPARAMS;
            break;
        }
    }

    if (status == ENET_SOK)
    {
        CSL_CPSW_setPortRxPriMapReg(regs, 0U, rxPriority->priorityMap);
    }

    return status;
}

static void CpswHostPort_getRxPriority(CSL_Xge_cpswRegs *regs,
                                       EnetPort_PriorityMap *rxPriority)
{
    CSL_CPSW_getPortRxPriMapReg(regs, 0U, rxPriority->priorityMap);
}

static int32_t CpswHostPort_setTxPriority(CSL_Xge_cpswRegs *regs,
                                          EnetPort_PriorityMap *txPriority)
{
    int32_t status = ENET_SOK;
    uint32_t i;

    for (i = 0U; i < ENET_PRI_NUM; i++)
    {
        if (txPriority->priorityMap[i] > ENET_PRI_MAX)
        {
            ENETTRACE_ERR("Invalid priority map %u -> %u\n", i, txPriority->priorityMap[i]);
            status = ENET_EINVALIDPARAMS;
            break;
        }
    }

    if (status == ENET_SOK)
    {
        CSL_CPSW_setPortTxPriMapReg(regs, 0U, txPriority->priorityMap);
    }

    return status;
}

static void CpswHostPort_getTxPriority(CSL_Xge_cpswRegs *regs,
                                       EnetPort_PriorityMap *txPriority)
{
    CSL_CPSW_getPortTxPriMapReg(regs, 0U, txPriority->priorityMap);
}

static void CpswHostPort_getMaxLen(CSL_Xge_cpswRegs *regs,
                                   EnetPort_MaxLen *maxLen)
{
    uint32_t i;

    maxLen->mru = CSL_CPSW_getPort0RxMaxLen(regs);

    for (i = 0U; i < ENET_PRI_NUM; i++)
    {
        maxLen->mtu[i] = CSL_CPSW_getTxMaxLenPerPriority(regs, i);
    }
}

static void CpswHostPort_getFifoStats(CSL_Xge_cpswRegs *regs,
                                      CpswHostPort_FifoStats *fifoStats)
{
    CSL_CPSW_THRURATE thruRate = {0};
    CSL_CPSW_CPPI_P0_FIFOSTATUS p0FifoStatus = {0};

    CSL_CPSW_getPortBlockCountReg(regs, 0U,
                                  &fifoStats->rxBlockCountExpress,
                                  &fifoStats->rxBlockCountPreempt,
                                  &fifoStats->txBlockCount);

    CSL_CPSW_getThruRateReg(regs, &thruRate);

    fifoStats->rxThroughputRate = thruRate.cppiRxThruRate;

    CSL_CPSW_getP0FifoStatus(regs, &p0FifoStatus);

    fifoStats->txActiveFifo[0U] = p0FifoStatus.p0TxPriActivePri0;
    fifoStats->txActiveFifo[1U] = p0FifoStatus.p0TxPriActivePri1;
    fifoStats->txActiveFifo[2U] = p0FifoStatus.p0TxPriActivePri2;
    fifoStats->txActiveFifo[3U] = p0FifoStatus.p0TxPriActivePri3;
    fifoStats->txActiveFifo[4U] = p0FifoStatus.p0TxPriActivePri4;
    fifoStats->txActiveFifo[5U] = p0FifoStatus.p0TxPriActivePri5;
    fifoStats->txActiveFifo[6U] = p0FifoStatus.p0TxPriActivePri6;
    fifoStats->txActiveFifo[7U] = p0FifoStatus.p0TxPriActivePri7;
}

#if ENET_CFG_IS_ON(CPSW_HOSTPORT_TRAFFIC_SHAPING)
static uint64_t CpswHostPort_mapBwToCnt(uint64_t rateInBps,
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
    return ENET_DIV_ROUNDUP(rateInBps * CPSW_HOSTPORT_RATELIM_DIV, cppiClkFreqHz);
}

static uint64_t CpswHostPort_mapCntToBw(uint32_t cntVal,
                                        uint32_t cppiClkFreqHz)
{
    /*
     * CIR or EIR count value to transfer rate equation:
     *
     *                         count value * CPPI_ICLK freq (MHz)
     * Transfer rate (Mbps) = ------------------------------------
     *                                      32768
     */
    return ((uint64_t)cntVal * cppiClkFreqHz) / CPSW_HOSTPORT_RATELIM_DIV;
}

static int32_t CpswHostPort_setTrafficShaping(CSL_Xge_cpswRegs *regs,
                                              const EnetPort_TrafficShapingCfg *cfg,
                                              uint32_t cppiClkFreqHz)
{
    uint64_t cirBps, eirBps;
    uint64_t cir, eir;
    uint32_t i;
    int32_t status = ENET_SOK;
    bool rateLimEnabled = false;

    for (i = 0U; i < ENET_PRI_NUM; i++)
    {
        cirBps = cfg->rates[i].committedRateBitsPerSec;
        eirBps = cfg->rates[i].excessRateBitsPerSec;
        cir = CpswHostPort_mapBwToCnt(cirBps, cppiClkFreqHz);
        eir = CpswHostPort_mapBwToCnt(eirBps, cppiClkFreqHz);

        /* CIR must be non-zero if EIR is non-zero */
        if ((cir == 0ULL) && (eir != 0ULL))
        {
            ENETTRACE_ERR("EIR is enabled (%llubps = %llu) but CIR is not (%llubps = %llu) for priority %u\n",
                          eirBps, eir, cirBps, cir, i);
            status = ENET_EINVALIDPARAMS;
            break;
        }

        /* Rate limit must be the highest priority channels */
        if ((cir == 0ULL) && rateLimEnabled)
        {
            ENETTRACE_ERR("Rate limiting disabled for priority %u\n", i);
            status = ENET_EINVALIDPARAMS;
            break;
        }

        /* Out of range */
        if ((cir > 0x0FFFFFFFULL) || (eir > 0x0FFFFFFFULL))
        {
            ENETTRACE_ERR("Invalid CIR=%llu (%llubps) EIR=%llu (%llubps) for priority %u\n",
                          cir, cirBps, eir, eirBps, i);
            status = ENET_EINVALIDPARAMS;
            break;
        }

        ENETTRACE_DBG("Rate limiting %s for priority %u, CIR=%ubps (%llu) EIR=%ubps (%llu)\n",
                      (cir != 0ULL) ? "enabled" : "disabled",
                      i, cirBps, cir, eirBps, eir);

        CSL_CPSW_setCppiPriCirEir(regs, i, (uint32_t)cir, (uint32_t)eir);

        if (cir != 0ULL)
        {
            rateLimEnabled = true;
        }
    }

    return status;
}

static void CpswHostPort_disableTrafficShaping(CSL_Xge_cpswRegs *regs)
{
    uint32_t i;

    for (i = 0U; i < ENET_PRI_NUM; i++)
    {
        CSL_CPSW_setCppiPriCirEir(regs, i, 0U, 0U);
    }
}

static int32_t CpswHostPort_getTrafficShaping(CSL_Xge_cpswRegs *regs,
                                              EnetPort_TrafficShapingCfg *cfg,
                                              uint32_t cppiClkFreqHz)
{
    uint32_t cir, eir;
    uint64_t cirBps, eirBps;
    uint32_t i;
    int32_t status = ENET_SOK;

    for (i = 0U; i < ENET_PRI_NUM; i++)
    {
        CSL_CPSW_getCppiPriCirEir(regs, i, &cir, &eir);

        cirBps = CpswHostPort_mapCntToBw(cir, cppiClkFreqHz);
        eirBps = CpswHostPort_mapCntToBw(eir, cppiClkFreqHz);

        /* CIR must be non-zero if EIR is non-zero */
        if ((cirBps == 0ULL) && (eirBps != 0ULL))
        {
            ENETTRACE_ERR("EIR is enabled (%llubps = %u) but CIR is not (%llubps = %u) for priority %u\n",
                          eirBps, eir, cirBps, cir, i);
            status = ENET_EUNEXPECTED;
            break;
        }

        ENETTRACE_DBG("Rate limiting %s for priority %u, CIR=%llubps (%u) EIR=%llubps (%u)\n",
                      (cir != 0U) ? "enabled" : "disabled",
                      i, cirBps, cir, eirBps, eir);

        cfg->rates[i].committedRateBitsPerSec = cirBps;
        cfg->rates[i].excessRateBitsPerSec = eirBps;
    }

    return status;
}
#endif
