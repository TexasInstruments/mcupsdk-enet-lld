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
 * \file  enet_phymdio_dflt.c
 *
 * \brief This file contains the default implementation of the MDIO interface
 *        of the Ethernet PHY (ENETPHY) driver with Enet LLD APIs.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <include/phy/enetphy.h>
#include <include/core/enet_mod_mdio.h>
#include <include/core/enet_mod_phy.h>
#include <include/core/enet_mod_macport.h>
#include <include/common/enet_phymdio_dflt.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/mod/phy_priv.h>
#include <priv/mod/phy_ioctl_priv.h>



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

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */



/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_ID(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_ERROR) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
    const EnetPhy_GenericInArgs *inArgs = (const EnetPhy_GenericInArgs *)prms->inArgs;
#endif
    EnetPhy_Version *version = (EnetPhy_Version *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = EnetPhy_getId(hPhy, version);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "Port %u: Failed to get PHY id: %d\n",
                     ENET_MACPORT_ID(inArgs->macPort), status);
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_SUPPORTED_MODES(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    /* Currently not supported */
    status = ENET_ENOTSUPPORTED;
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_LOOPBACK_STATE(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    /* Currently not supported */
    status = ENET_ENOTSUPPORTED;
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_IS_ALIVE(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    bool *isAlive = (bool *)prms->outArgs;
    int32_t status = ENET_SOK;

    *isAlive = EnetPhy_isAlive(hPhy);
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_IS_LINKED(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    bool *isLinked = (bool *)prms->outArgs;
    int32_t status = ENET_SOK;

    *isLinked = EnetPhy_isLinked(hPhy);
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_LINK_MODE(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
#if ((ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_ERROR) && ENET_CFG_IS_OFF(TRACE_DISABLE_INFOSTRING))
    const EnetPhy_GenericInArgs *inArgs = (const EnetPhy_GenericInArgs *)prms->inArgs;
#endif
    EnetMacPort_LinkCfg *linkCfg = (EnetMacPort_LinkCfg *)prms->outArgs;
    EnetPhy_LinkCfg phyLinkCfg;
    int32_t status = ENET_SOK;

    status = EnetPhy_getLinkCfg(hPhy, &phyLinkCfg);
    if (status == ENETPHY_SOK)
    {
        linkCfg->speed = (Enet_Speed)phyLinkCfg.speed;
        linkCfg->duplexity = (Enet_Duplexity)phyLinkCfg.duplexity;
    }
    else
    {
        ENETTRACE_ERR("Port %u: Failed to get link config: %d\n",
                      ENET_MACPORT_ID(inArgs->macPort), status);
    }
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_LINK_STATUS(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    EnetMacPort_LinkCfg *linkCfg = (EnetMacPort_LinkCfg *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = EnetPhy_getLinkStatus(hPhy, (EnetPhy_Speed*)&linkCfg->speed, (EnetPhy_Duplexity*)&linkCfg->duplexity);
    if (status != ENETPHY_SOK)
    {
        ENETTRACE_ERR("Phy %u: Failed to read phy link speed, duplex: %d\n", hPhy->addr, status);
    }
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_RESET(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    /* Currently not supported */
    status = ENET_ENOTSUPPORTED;
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_READ_REG(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_ReadRegInArgs *inArgs = (const EnetPhy_ReadRegInArgs *)prms->inArgs;
    uint16_t *val = (uint16_t *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = EnetPhy_readReg(hPhy, inArgs->reg, val);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "Port %u: Failed to read reg %u: %d\n",
                     ENET_MACPORT_ID(inArgs->macPort), inArgs->reg, status);
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_WRITE_REG(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_WriteRegInArgs *inArgs = (const EnetPhy_WriteRegInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = EnetPhy_writeReg(hPhy, inArgs->reg, inArgs->val);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "Port %u: Failed to write reg %u: %d\n",
                     ENET_MACPORT_ID(inArgs->macPort), inArgs->reg, status);
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_READ_EXT_REG(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_ReadRegInArgs *inArgs = (const EnetPhy_ReadRegInArgs *)prms->inArgs;
    uint16_t *val = (uint16_t *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = EnetPhy_readExtReg(hPhy, inArgs->reg, val);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "Port %u: Failed to read ext reg %u: %d\n",
                     ENET_MACPORT_ID(inArgs->macPort), inArgs->reg, status);
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_WRITE_EXT_REG(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_WriteRegInArgs *inArgs = (const EnetPhy_WriteRegInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = EnetPhy_writeExtReg(hPhy, inArgs->reg, inArgs->val);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "Port %u: Failed to write ext reg %u: %d\n",
                     ENET_MACPORT_ID(inArgs->macPort), inArgs->reg, status);
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_C45_READ_REG(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_C45ReadRegInArgs *inArgs = (const EnetPhy_C45ReadRegInArgs *)prms->inArgs;
    uint16_t *val = (uint16_t *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = EnetPhy_readC45Reg(hPhy, inArgs->mmd, inArgs->reg, val);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "Port %u: Failed to read C45 mmd %u reg %u: %d\n",
                     ENET_MACPORT_ID(inArgs->macPort), inArgs->mmd, inArgs->reg, status);
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_C45_WRITE_REG(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_C45WriteRegInArgs *inArgs = (const EnetPhy_C45WriteRegInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    status = EnetPhy_writeC45Reg(hPhy, inArgs->mmd, inArgs->reg, inArgs->val);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "Port %u: Failed to write C45 mmd %u reg %u: %d\n",
                     ENET_MACPORT_ID(inArgs->macPort), inArgs->mmd, inArgs->reg, status);
    return status;
}

int32_t  EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_PRINT_REGS(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    EnetPhy_printRegs(hPhy);
    return status;
}


int32_t EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_ADJ_PTP_FREQ(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_AdjPtpFreqInArgs *inArgs =
        (const EnetPhy_AdjPtpFreqInArgs *)prms->inArgs;
    int32_t status = EnetPhy_adjPtpFreq(hPhy, inArgs->ppb);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                        "Port %u: Failed to adj PTP freq: %d\n",
                        ENET_MACPORT_ID(inArgs->macPort), status);
    return status;
}


int32_t EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_ADJ_PTP_PHASE(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_AdjPtpPhaseInArgs *inArgs =
        (const EnetPhy_AdjPtpPhaseInArgs *)prms->inArgs;
    int32_t status = EnetPhy_adjPtpPhase(hPhy, inArgs->offset);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                        "Port %u: Failed to adj PTP phase: %d\n",
                        ENET_MACPORT_ID(inArgs->macPort), status);
    return status;
}

int32_t EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_PTP_TIME(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_GenericInArgs *inArgs =
        (const EnetPhy_GenericInArgs *)prms->inArgs;
    (void)inArgs;
    uint64_t *val = (uint64_t *)prms->outArgs;
    int32_t status = EnetPhy_getPtpTime(hPhy, val);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                        "Port %u: Failed to get PTP time: %d\n",
                        ENET_MACPORT_ID(inArgs->macPort), status);
    return status;
}

int32_t EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_SET_PTP_TIME(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_SetPtpTimeInArgs *inArgs =
        (const EnetPhy_SetPtpTimeInArgs *)prms->inArgs;
    int32_t status = EnetPhy_setPtpTime(hPhy, inArgs->ts64);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                        "Port %u: Failed to set PTP time: %d\n",
                        ENET_MACPORT_ID(inArgs->macPort), status);
    return status;
}

int32_t EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_PTP_TXTS(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_PtpPktTimestampInArgs *inArgs =
        (const EnetPhy_PtpPktTimestampInArgs *)prms->inArgs;
    uint64_t *ts64 = (uint64_t *)prms->outArgs;
    int32_t status = EnetPhy_getPtpTxTime(hPhy, inArgs->domain, inArgs->msgType,
                                    inArgs->seqId, ts64);
    ENETTRACE_ERR_IF((status != ENETPHY_SOK) && (status != ENETPHY_EUNAVAILABLE),
                        "Port %u: Failed to get PTP txTime: %d\n",
                        ENET_MACPORT_ID(inArgs->macPort), status);
    return status;
}

int32_t EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_PTP_RXTS(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_PtpPktTimestampInArgs *inArgs =
        (const EnetPhy_PtpPktTimestampInArgs *)prms->inArgs;
    uint64_t *ts64 = (uint64_t *)prms->outArgs;
    int32_t status = EnetPhy_getPtpRxTime(hPhy, inArgs->domain, inArgs->msgType,
                                    inArgs->seqId, ts64);
    ENETTRACE_ERR_IF((status != ENETPHY_SOK) && (status != ENETPHY_EUNAVAILABLE),
                        "Port %u: Failed to get PTP RxTime: %d\n",
                        ENET_MACPORT_ID(inArgs->macPort), status);
    return status;
}

int32_t EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_WAIT_PTP_TXTS(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_PtpPktTimestampInArgs *inArgs =
        (const EnetPhy_PtpPktTimestampInArgs *)prms->inArgs;
    int32_t status = EnetPhy_waitPtpTxTime(hPhy, inArgs->domain, inArgs->msgType,
                                    inArgs->seqId);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                        "Port %u: Failed to wait PTP TxTime: %d\n",
                        ENET_MACPORT_ID(inArgs->macPort), status);
    return status;
}

int32_t EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_PROC_STATUS_FRAME(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_ProcStatusFrameInArgs *inArgs =
        (const EnetPhy_ProcStatusFrameInArgs *)prms->inArgs;
    EnetPhy_ProcStatusFrameOutArgs *outArgs =
        (EnetPhy_ProcStatusFrameOutArgs *)prms->outArgs;

    int32_t status = EnetPhy_procStatusFrame(hPhy, inArgs->frame, inArgs->size,
                                        &outArgs->types);

    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                        "Port %u: Failed to process PTP status frame: %d\n",
                        ENET_MACPORT_ID(inArgs->macPort), status);
    return status;
}

int32_t EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_STATUS_FRAME_ETHDR(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_GenericInArgs *inArgs =
        (const EnetPhy_GenericInArgs *)prms->inArgs;
    (void)inArgs;
    EnetPhy_GetStatusFrameEthdrOutArgs *outArgs =
        (EnetPhy_GetStatusFrameEthdrOutArgs *)prms->outArgs;

    int32_t status = EnetPhy_getStatusFrameEthHeader(hPhy, outArgs->ethhdr,
                            outArgs->size);

    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                        "Port %u: Failed to get PTP status frame ehthdr: %d\n",
                        ENET_MACPORT_ID(inArgs->macPort), status);
    return status;
}

int32_t EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_ENABLE_PTP(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_EnablePtpInArgs *inArgs =
        (const EnetPhy_EnablePtpInArgs *)prms->inArgs;

    int32_t status = EnetPhy_enablePtp(hPhy, inArgs->on, inArgs->srcMacStatusFrameType);

    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                        "Port %u: Failed to %s PTP: %d\n",
                        ENET_MACPORT_ID(inArgs->macPort),
                        inArgs->on ? "enable" : "disable", status);
    return status;
}

int32_t EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_ENABLE_EVENT_CAPTURE(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_EnableEventCaptureInArgs *inArgs =
        (const EnetPhy_EnableEventCaptureInArgs *)prms->inArgs;

    int32_t status = EnetPhy_enableEventCapture(hPhy, inArgs->eventIdx,
                        inArgs->falling, inArgs->on);

    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                        "Port %u: Failed to %s event capture: %d\n",
                        ENET_MACPORT_ID(inArgs->macPort),
                        inArgs->on ? "enable" : "disable", status);
    return status;
}

int32_t EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_ENABLE_TRIGGER_OUTPUT(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_EnableTriggerOutputInArgs *inArgs =
        (const EnetPhy_EnableTriggerOutputInArgs *)prms->inArgs;

    int32_t status = EnetPhy_enableTriggerOutput(hPhy, inArgs->triggerIdx,
                        inArgs->startNsec, inArgs->periodNsec, inArgs->repeat);

    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                        "Port %u: Failed to %s enable trigger output: %d\n",
                        ENET_MACPORT_ID(inArgs->macPort),
                        inArgs->periodNsec ? "enable" : "disable", status);
    return status;
}

int32_t EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_GET_EVENT_TIMESTAMP(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_GenericInArgs *inArgs =
        (const EnetPhy_GenericInArgs *)prms->inArgs;
    (void)inArgs;
    EnetPhy_GetEventTimestampOutArgs *outArgs =
        (EnetPhy_GetEventTimestampOutArgs *)prms->outArgs;

    int32_t status = EnetPhy_getEventTs(hPhy, &outArgs->eventIdx,
                                &outArgs->seqId, &outArgs->ts64);

    ENETTRACE_ERR_IF((status != ENETPHY_SOK) && (status != ENETPHY_EUNAVAILABLE),
                    "Port %u: Failed to get event TS: %d\n",
                    ENET_MACPORT_ID(inArgs->macPort), status);
    return status;
}

int32_t EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_CONFIG_MEDIA_CLOCK(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_ConfigMediaClockInArgs *inArgs =
        (const EnetPhy_ConfigMediaClockInArgs *)prms->inArgs;

    int32_t status = EnetPhy_configMediaClock(hPhy, inArgs->isMaster, \
                                inArgs->streamIDMatchValue, inArgs->enTrigOut);

    ENETTRACE_ERR_IF((status != ENETPHY_SOK) && (status != ENETPHY_EUNAVAILABLE),
                    "Port %u: Failed to configure Media Clock: %d\n",
                    ENET_MACPORT_ID(inArgs->macPort), status);
    return status;
}

int32_t EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_NUDGE_CODEC_CLOCK(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const EnetPhy_NudgeCodecClockInArgs *inArgs =
        (const EnetPhy_NudgeCodecClockInArgs *)prms->inArgs;

    int32_t status = EnetPhy_nudgeCodecClock(hPhy, inArgs->nudgeValue);

    ENETTRACE_ERR_IF((status != ENETPHY_SOK) && (status != ENETPHY_EUNAVAILABLE),
                    "Port %u: Failed to Nudge Codec Clock: %d\n",
                    ENET_MACPORT_ID(inArgs->macPort), status);
    return status;
}
