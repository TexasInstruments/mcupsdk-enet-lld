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
 * \file  cpsw_cpts.c
 *
 * \brief This file contains the implementation of the CPSW CPTS module.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>
#include <csl_cpswitch.h>
#include <enet_cfg.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_osal.h>
#include <include/core/enet_soc.h>
#include <include/mod/cpsw_cpts.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/mod/cpsw_cpts_priv.h>
#include <priv/mod/cpsw_cpts_ioctl_priv.h>
#include <priv/mod/cpsw_clks.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/*! \brief Timestamp retrieval poll iteration count. */
#define CPSW_CPTS_POLL_INTERVAL               (10000U)

/*! \brief Minimum PPM value. */
#define CPSW_CPTS_PPM_MIN_VAL                 (0x400U)

/*! \brief GENFn minimum length value. */
#define CPSW_CPTS_GENFN_LENGTH_MIN_VAL        (5U)

/*! \brief PPM conversion factor. */
#define CPSW_CPTS_PPM                         (1000000U)

/*! \brief PPM GHz conversion factor. */
#define CPSW_CPTS_PPM_GIGAHERTZ_VAL           (1000000000U)

/*! \brief Secs-to-hours conversion factor. */
#define CPSW_CPTS_SECONDS_PER_HOUR            (3600U)

/*! \brief EST event sequence number mask in sequence_id field. */
#define CPSW_CPTS_EST_EVENT_SEQNUM_MASK       (0x00FFU)

/*! \brief EST event sequence number shift within sequence_id field. */
#define CPSW_CPTS_EST_EVENT_SEQNUM_SHIFT      (0U)

/*! \brief EST event receive port number shift within sequence_id field. */
#define CPSW_CPTS_EST_EVENT_RXPORT_MASK       (0xF000U)

/*! \brief EST event receive port number shift within sequence_id field. */
#define CPSW_CPTS_EST_EVENT_RXPORT_SHIFT      (12U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void CpswCpts_printRegs(CSL_cptsRegs *regs);
static void CpswCpts_printStats(const CpswCpts_EventStats *stats);
static int32_t CpswCpts_lookUpEvent(CpswCpts_Handle hCpts,
                                    const CpswCpts_Event *matchEvent,
                                    CpswCpts_Event *event);

static void CpswCpts_clearEventMemPools(CpswCpts_Handle hCpts);

static int32_t CpswCpts_checkTsCompCompat(CSL_cptsRegs *regs);

static int32_t CpswCpts_checkGenfEstfErrata(EnetMod_Handle hMod,
                                            CSL_cptsRegs *regs,
                                            uint32_t index,
                                            bool isGenf);
static uint64_t CpswCpts_calcPpmVal(uint64_t tsPpmVal,
                                    EnetTimeSync_AdjMode adjMode);

static int32_t CpswCpts_lookUpEstEvent(CpswCpts_Handle hCpts,
                                       Enet_MacPort macPort,
                                       uint32_t domain,
                                       CpswCpts_EstEvent *event);

static int32_t CpswCpts_handleEvents(CpswCpts_Handle hCpts,
                                     CSL_cptsRegs *regs);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


int32_t CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_GET_VERSION(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    Enet_Version *version = (Enet_Version *)prms->outArgs;
    CSL_CPTS_VERSION ver = {0};
    int32_t status = ENET_SOK;

    CSL_CPTS_getCptsVersionInfo(regs, &ver);
    version->maj = ver.majorVer;
    version->min = ver.minorVer;
    version->rtl = ver.rtlVer;
    version->id  = ver.id;
    version->other1 = ENET_VERSION_NONE;
    version->other2 = ENET_VERSION_NONE;
    return status;
}

int32_t CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_PRINT_REGS(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CpswCpts_printRegs(regs);
    return status;
}

int32_t CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_PRINT_STATS(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CpswCpts_printStats(&hCpts->eventStats);
    return status;
}

int32_t CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    uint64_t *tsVal = (uint64_t *)prms->outArgs;
    int32_t loop = CPSW_CPTS_POLL_INTERVAL;
    int32_t status = ENET_SOK;

    if (hCpts->tsPushInFifo == false)
    {
        hCpts->tsPushInFifo = true;
        CSL_CPTS_TSEventPush(regs);

        /* Poll until the Time stamp event occurs */
        while (loop > 0U)
        {
            if (hCpts->tsPushInFifo == false)
            {
                *tsVal = hCpts->tsVal;
                break;
            }

            loop--;
        }

        if (loop == 0U)
        {
            ENETTRACE_ERR("Timed out retrieving current timestamp event\n");
            status = ENET_ENOTFOUND;
            *tsVal = 0U;
        }
    }
    else
    {
        ENETTRACE_WARN("Timestamp push event already in FIFO, try again later...\n");
        status = ENET_EBUSY;
    }
    return status;
}

int32_t CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_SET_TIMESTAMP(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    EnetTimeSync_setTimestamp *tsSet = (EnetTimeSync_setTimestamp *)prms->inArgs;
    const uint64_t tsLoadVal = tsSet->tsLoadVal;
    uint32_t tsLoadValHi = 0U;
    uint32_t tsLoadValLo = 0U;
    int32_t status = ENET_SOK;

    /* Disable CPTS interrupt */
    CSL_CPTS_disableInterrupt(regs);

    tsLoadValHi = (uint32_t)(tsLoadVal >> 32U);
    tsLoadValLo = (uint32_t)(tsLoadVal & 0xFFFFFFFFU);
    CSL_CPTS_setTSVal(regs, tsLoadValLo, tsLoadValHi);

    /* Re-enable CPTS interrupt */
    CSL_CPTS_enableInterrupt(regs);
    return status;
}

int32_t CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_ADJUST_TIMESTAMP(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    const EnetTimeSync_TimestampAdj *tsAdj = (const EnetTimeSync_TimestampAdj *)prms->inArgs;
    CSL_CPTS_TS_PPM_DIR ppmDir;
    CSL_CPTS_ESTF_PPM_DIR estfPpmDir = CSL_CPTS_ESTF_PPM_DIR_DECREASE;
    uint32_t adjOffset;
    uint64_t adjVal;
    uint32_t tsPpmValHi;
    uint32_t tsPpmValLo;
    int32_t status = ENET_SOK;
    uint32_t idx = 0U;
    uint32_t estfIdxMask = 0U;
    uint64_t estfAdjVal = 0U;
    uint32_t estfPpmValHi = 0U;
    uint32_t estfPpmValLo = 0U;

    if (tsAdj->intervalInNsecs == 0U)
    {
        ENETTRACE_ERR("Adjust time interval cannot be 0\n");
        status = ENET_EINVALIDPARAMS;
    }

    estfIdxMask = hCpts->activeEstfIdxMask;
    if (status == ENET_SOK)
    {
        if (tsAdj->adjValInNsecs == 0)
        {
            CSL_CPTS_setTSPpm(regs, 0U, 0U, CSL_CPTS_TS_PPM_DIR_INCREASE);
            for(idx = 0U; idx < 8U; idx++)
            {
                if ((estfIdxMask & ((uint8_t)ENET_BIT(idx))) != 0U)
                {
                    CSL_CPTS_setESTFnPpm(regs, idx, 0U, 0U, 0U);
                }
            }
        }
        else
        {
            if (tsAdj->adjValInNsecs > 0)
            {
                ppmDir = CSL_CPTS_TS_PPM_DIR_INCREASE;
                estfPpmDir = CSL_CPTS_ESTF_PPM_DIR_INCREASE;
                adjOffset = tsAdj->adjValInNsecs;
            }
            else
            {
                ppmDir = CSL_CPTS_TS_PPM_DIR_DECREASE;
                estfPpmDir = CSL_CPTS_ESTF_PPM_DIR_DECREASE;
                adjOffset = (uint64_t)(-1 * tsAdj->adjValInNsecs);
            }

            adjVal = (uint64_t)tsAdj->intervalInNsecs / adjOffset;
            if (adjVal <= (uint64_t)CPSW_CPTS_PPM_MIN_VAL)
            {
                ENETTRACE_WARN("Setting PPM to %u, as %llu is less than minimum value (%u)\n",
                               CPSW_CPTS_PPM_MIN_VAL, adjVal, CPSW_CPTS_PPM_MIN_VAL);
                adjVal = CPSW_CPTS_PPM_MIN_VAL;
            }

            tsPpmValHi = (uint32_t)(adjVal >> 32U);
            tsPpmValLo = (uint32_t)(adjVal & 0xFFFFFFFFU);

            if(estfIdxMask != 0U)
            {
                /* GenF/ESTFn PPM will do correction using cpts refclk tick which is
                 * (cpts->ts_add_val + 1) ns, so GenF/ESTFn length PPM adj period
                 * need to be corrected.
                 */
                estfAdjVal = adjVal * (hCpts->tsAddVal + 1U);
                estfPpmValHi = (uint32_t)(estfAdjVal >> 32U);
                estfPpmValLo = (uint32_t)(estfAdjVal & 0xFFFFFFFFU);
            }
            CSL_CPTS_setTSPpm(regs, tsPpmValLo, tsPpmValHi, ppmDir);
            if (estfIdxMask != 0U)
            {
               for(idx = 0U; idx < ENET_CFG_CPSW_ESTF_NUM; idx++)
               {
                   if ((estfIdxMask & ENET_BIT(idx)) != 0U)
                   {
                       CSL_CPTS_setESTFnPpm(regs, idx, estfPpmValLo, estfPpmValHi, estfPpmDir);
                   }
               }
            }
        }
    }
    return status;
}

int32_t CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_GET_ETH_RX_TIMESTAMP(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    const EnetTimeSync_GetEthTimestampInArgs *inArgs =
        (const EnetTimeSync_GetEthTimestampInArgs *)prms->inArgs;
    uint64_t *tsVal = (uint64_t *)prms->outArgs;
    CpswCpts_Event matchEvent;
    CpswCpts_Event event;
    int32_t status = ENET_SOK;

    matchEvent.eventType = CPSW_CPTS_EVENTTYPE_ETH_RECEIVE;
    matchEvent.msgType   = inArgs->msgType;
    matchEvent.seqId     = inArgs->seqId;
    matchEvent.domain    = inArgs->domain;
    matchEvent.portNum   = inArgs->portNum;
    matchEvent.tsVal     = 0U;

    status = CpswCpts_lookUpEvent(hCpts, &matchEvent, &event);
    if (status == ENET_SOK)
    {
        *tsVal = event.tsVal;
    }
    else if (status == ENET_ENOTFOUND)
    {
        ENETTRACE_ERR("Eth RX event not found (msgType=%u, seqId=%u, dom=%u, portNum=%u)\n",
                      matchEvent.msgType,
                      matchEvent.seqId,
                      matchEvent.domain,
                      matchEvent.portNum);
    }
    else
    {
        ENETTRACE_ERR("Failed to lookup for Ethernet RX event: %d\n", status);
    }
    return status;
}

int32_t CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_GET_ETH_TX_TIMESTAMP(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    const EnetTimeSync_GetEthTimestampInArgs *inArgs =
        (const EnetTimeSync_GetEthTimestampInArgs *)prms->inArgs;
    uint64_t *tsVal = (uint64_t *)prms->outArgs;
    CpswCpts_Event matchEvent;
    CpswCpts_Event event;
    int32_t status = ENET_SOK;

    matchEvent.eventType = CPSW_CPTS_EVENTTYPE_ETH_TRANSMIT;
    matchEvent.msgType   = inArgs->msgType;
    matchEvent.seqId     = inArgs->seqId;
    matchEvent.domain    = inArgs->domain;
    matchEvent.portNum   = inArgs->portNum;
    matchEvent.tsVal     = 0U;

    status = CpswCpts_lookUpEvent(hCpts, &matchEvent, &event);
    if (status == ENET_SOK)
    {
        *tsVal = event.tsVal;
    }
    else if (status == ENET_ENOTFOUND)
    {
        ENETTRACE_ERR("Eth TX event not found (msgType=%u, seqId=%u, dom=%u, portNum=%u)\n",
                      matchEvent.msgType,
                      matchEvent.seqId,
                      matchEvent.domain,
                      matchEvent.portNum);
    }
    else
    {
        ENETTRACE_ERR("Failed to lookup for Ethernet TX event: %d\n", status);
    }
    return status;
}

int32_t CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_RESET(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CpswCpts_clearEventMemPools(hCpts);
    return status;
}

int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_REGISTER_STACK(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    CpswCpts_RegisterStackInArgs *stackInArgs = (CpswCpts_RegisterStackInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    if (hCpts->eventNotifyCb == NULL)
    {
        if (stackInArgs->eventNotifyCb != NULL)
        {
            hCpts->eventNotifyCb = stackInArgs->eventNotifyCb;
            hCpts->eventNotifyCbArg = stackInArgs->eventNotifyCbArg;
        }
        else
        {
            ENETTRACE_ERR("Invalid event notification params\n");
            status = ENET_EINVALIDPARAMS;
        }
    }
    else
    {
        ENETTRACE_ERR("Stack already registered\n");
        status = ENET_EALREADYOPEN;
    }
    return status;
}

int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_UNREGISTER_STACK(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    /* Check if the stack is registered or not */
    if (hCpts->eventNotifyCb != NULL)
    {
        hCpts->eventNotifyCb = NULL;
        hCpts->eventNotifyCbArg = NULL;
    }
    else
    {
        ENETTRACE_ERR("No stack is registered\n");
        status = ENET_EUNEXPECTED;
    }
    return status;
}

int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_REGISTER_HWPUSH_CALLBACK(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswCpts_RegisterHwPushCbInArgs *inArgs = (const CpswCpts_RegisterHwPushCbInArgs *)prms->inArgs;
    uint32_t hwPushIdx = 0U;
    int32_t status = ENET_SOK;

    if ((inArgs->hwPushNum <= hCpts->hwPushCnt) &&
        (inArgs->hwPushNum >= CPSW_CPTS_HWPUSH_FIRST))
    {
        hwPushIdx = CPSW_CPTS_HWPUSH_NORM(inArgs->hwPushNum);
        if ((hwPushIdx < CPSW_CPTS_HWPUSH_COUNT_MAX) &&
            (hCpts->hwPushNotifyCb[hwPushIdx] == NULL))
        {
            if (inArgs->hwPushNotifyCb != NULL)
            {
                hCpts->hwPushNotifyCb[hwPushIdx] = inArgs->hwPushNotifyCb;
                hCpts->hwPushNotifyCbArg[hwPushIdx] = inArgs->hwPushNotifyCbArg;
            }
            else
            {
                ENETTRACE_ERR("Invalid hardware push notification callback params\n");
                status = ENET_EINVALIDPARAMS;
            }
        }
        else
        {
            ENETTRACE_ERR("Hardware push callback for instance %u already registered\n",
                          inArgs->hwPushNum);
            status = ENET_EALREADYOPEN;
        }
    }
    else
    {
        ENETTRACE_ERR("Invalid hardware push instance number %u\n", inArgs->hwPushNum);
        status = ENET_EINVALIDPARAMS;
    }
    return status;
}

int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_UNREGISTER_HWPUSH_CALLBACK(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    CpswCpts_HwPush hwPushNum = *(CpswCpts_HwPush *)prms->inArgs;
    uint32_t hwPushIdx = 0U;
    int32_t status = ENET_SOK;

    if ((hwPushNum <= hCpts->hwPushCnt) &&
        (hwPushNum >= CPSW_CPTS_HWPUSH_FIRST))
    {
        hwPushIdx = CPSW_CPTS_HWPUSH_NORM(hwPushNum);

        /* Check if the callback is registered or not */
        if ((hwPushIdx < CPSW_CPTS_HWPUSH_COUNT_MAX) &&
            (hCpts->hwPushNotifyCb[hwPushIdx] != NULL))
        {
            hCpts->hwPushNotifyCb[hwPushIdx] = NULL;
            hCpts->hwPushNotifyCbArg[hwPushIdx] = NULL;
        }
        else
        {
            ENETTRACE_WARN("No callback for hardware push instance %u is registered\n", hwPushNum);
            status = ENET_SOK;
        }
    }
    else
    {
        ENETTRACE_ERR("Invalid hardware push instance number %u\n", hwPushNum);
        status = ENET_EINVALIDPARAMS;
    }
    return status;
}

int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_SET_TS_NUDGE(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t tsNudge = *(int32_t *)prms->inArgs;
    int32_t status = ENET_SOK;

    if ((tsNudge >= CPSW_CPTS_NUDGE_MIN_VAL) &&
        (tsNudge <= CPSW_CPTS_NUDGE_MAX_VAL))
    {
        CSL_CPTS_setTSNudge(regs, tsNudge);
    }
    else
    {
        ENETTRACE_ERR("Nudge value %u is out of range (%u, %u)\n",
                      tsNudge, CPSW_CPTS_NUDGE_MIN_VAL, CPSW_CPTS_NUDGE_MAX_VAL);
        status = ENET_EINVALIDPARAMS;
    }
    return status;
}

int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_SET_COMP(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    status = CpswCpts_checkTsCompCompat(regs);
    if (status == ENET_SOK)
    {
        const CpswCpts_SetCompValInArgs *inArgs = (const CpswCpts_SetCompValInArgs *)prms->inArgs;
        CSL_CPTS_CONTROL control;
        uint32_t tsCompValHi = 0U;
        uint32_t tsCompValLo = 0U;

        CSL_CPTS_getCntlReg(regs, &control);
        control.tsCompToggle = inArgs->tsCompToggle;
        CSL_CPTS_setCntlReg(regs, &control);

        tsCompValHi = (uint32_t)(inArgs->tsCompVal >> 32U);
        tsCompValLo = (uint32_t)(inArgs->tsCompVal & 0xFFFFFFFFU);
        CSL_CPTS_setTSCompVal(regs, tsCompValLo, tsCompValHi, inArgs->tsCompLen);
    }
    else
    {
        ENETTRACE_ERR("SET_COMP not supported with TS add value or TS PPM, try GENFn instead\n");
        status = ENET_ENOTSUPPORTED;
    }
    return status;
}

int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_SET_COMP_NUDGE(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t tsCompNudge = *(int32_t *)prms->inArgs;
    int32_t status = ENET_SOK;

    if ((tsCompNudge >= CPSW_CPTS_NUDGE_MIN_VAL) &&
        (tsCompNudge <= CPSW_CPTS_NUDGE_MAX_VAL))
    {
        CSL_CPTS_setTSCompNudge(regs, tsCompNudge);
    }
    else
    {
        ENETTRACE_ERR("Nudge value %u is out of range (%u, %u)\n",
                      tsCompNudge, CPSW_CPTS_NUDGE_MIN_VAL, CPSW_CPTS_NUDGE_MAX_VAL);
        status = ENET_EINVALIDPARAMS;
    }
    return status;
}

int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_SET_GENF(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswCpts_SetFxnGenInArgs *inArgs = (const CpswCpts_SetFxnGenInArgs *)prms->inArgs;
    uint64_t adjVal;
    int32_t status = ENET_SOK;
    EnetMod_Handle hMod;

    hMod = (EnetMod_Handle) hCpts;
    status = CpswCpts_checkGenfEstfErrata(hMod, regs, inArgs->index, true);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set GENFn due to reconfig errata\n");

    if (status == ENET_SOK)
    {
        if (inArgs->ppmMode != ENET_TIMESYNC_ADJMODE_DISABLE)
        {
            adjVal = CpswCpts_calcPpmVal(inArgs->ppmVal, inArgs->ppmMode);
            if (inArgs->length < CPSW_CPTS_GENFN_LENGTH_MIN_VAL)
            {
                ENETTRACE_ERR_IF(status != ENET_SOK,
                                 "Invalid GENFn length value of %u, it should be > %u\n",
                                 inArgs->length, CPSW_CPTS_GENFN_LENGTH_MIN_VAL);
                status = ENET_EINVALIDPARAMS;
            }
        }
        else
        {
            adjVal = 0ULL;
        }
    }

    if (status == ENET_SOK)
    {
        status = CSL_CPTS_setupGENFn(regs,
                                     inArgs->index,
                                     inArgs->length,
                                     inArgs->compare,
                                     inArgs->polarityInv,
                                     adjVal,
                                     inArgs->ppmDir);
        if (status == ENET_SOK)
        {
            if (inArgs->length != 0U)
            {
                hCpts->activeGenfIdxMask |= ENET_BIT(inArgs->index);
            }
            else
            {
                /* Zero length means Genf is disabled */
                hCpts->activeGenfIdxMask &= (~(ENET_BIT(inArgs->index)));
            }
        }
        ENETTRACE_ERR_IF(status != ENET_SOK, "Wrong GENFn index value, error: %d\n", status);
    }
    return status;
}

int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_SET_GENF_NUDGE(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswCpts_SetFxnGenNudgeInArgs *inArgs = (const CpswCpts_SetFxnGenNudgeInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    if ((inArgs->tsNudge >= CPSW_CPTS_NUDGE_MIN_VAL) &&
        (inArgs->tsNudge <= CPSW_CPTS_NUDGE_MAX_VAL))
    {
        status = CSL_CPTS_setGENFnNudge(regs, inArgs->index, inArgs->tsNudge);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Wrong GENFn index value, error: %d\n", status);
    }
    else
    {
        ENETTRACE_ERR("Nudge value %u is out of range (%u, %u)\n",
                      inArgs->tsNudge, CPSW_CPTS_NUDGE_MIN_VAL, CPSW_CPTS_NUDGE_MAX_VAL);
        status = ENET_EINVALIDPARAMS;
    }
    return status;
}

int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_SET_ESTF(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    CpswCpts_SetFxnGenInArgs *inArgs = (CpswCpts_SetFxnGenInArgs *)prms->inArgs;
    uint64_t adjVal;
    int32_t status = ENET_SOK;
    EnetMod_Handle hMod;

    hMod = (EnetMod_Handle) hCpts;
    status = CpswCpts_checkGenfEstfErrata(hMod, regs, inArgs->index, false);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set EstFn due to reconfig errata\n");

    if (status == ENET_SOK)
    {
        if (inArgs->ppmMode != ENET_TIMESYNC_ADJMODE_DISABLE)
        {
            adjVal = CpswCpts_calcPpmVal(inArgs->ppmVal, inArgs->ppmMode);
        }
        else
        {
            adjVal = 0ULL;
        }
    }

    if (status == ENET_SOK)
    {
        status = CSL_CPTS_setupESTFn(regs,
                                     inArgs->index,
                                     inArgs->length,
                                     inArgs->compare,
                                     inArgs->polarityInv,
                                     adjVal,
                                     inArgs->ppmDir);
        if (status == ENET_SOK)
        {
            if (inArgs->length != 0U)
            {
                hCpts->activeEstfIdxMask |= ENET_BIT(inArgs->index);
            }
            else
            {
                /* Zero length means Estfn is disabled */
                hCpts->activeEstfIdxMask &= (~(ENET_BIT(inArgs->index)));
            }
        }

        ENETTRACE_ERR_IF(status != ENET_SOK, "Wrong ESTFn index value, error: %d\n", status);
    }
    return status;
}

int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_SET_ESTF_NUDGE(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    const CpswCpts_SetFxnGenNudgeInArgs *inArgs = (const CpswCpts_SetFxnGenNudgeInArgs *)prms->inArgs;
    int32_t status = ENET_SOK;

    if ((inArgs->tsNudge >= CPSW_CPTS_NUDGE_MIN_VAL) &&
        (inArgs->tsNudge <= CPSW_CPTS_NUDGE_MAX_VAL))
    {
        status = CSL_CPTS_setESTFnNudge(regs, inArgs->index, inArgs->tsNudge);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Wrong ESTFn index value, error: %d\n", status);
    }
    else
    {
        ENETTRACE_ERR("Nudge value %u is out of range (%u, %u)\n",
                      inArgs->tsNudge, CPSW_CPTS_NUDGE_MIN_VAL, CPSW_CPTS_NUDGE_MAX_VAL);
        status = ENET_EINVALIDPARAMS;
    }
    return status;
}

int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_SELECT_TS_OUTPUT_BIT(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    CpswCpts_OutputBitSel bitSelect = *(CpswCpts_OutputBitSel *)prms->inArgs;
    CSL_CPTS_CONTROL control = {0};
    int32_t status = ENET_SOK;

    CSL_CPTS_getCntlReg(regs, &control);

    if (control.tsOutputBitSel != CPSW_CPTS_TS_OUTPUT_BIT_DISABLED)
    {
        control.tsOutputBitSel = CPSW_CPTS_TS_OUTPUT_BIT_DISABLED;
        CSL_CPTS_setCntlReg(regs, &control);
    }

    control.tsOutputBitSel = bitSelect;
    CSL_CPTS_setCntlReg(regs, &control);
    return status;
}

int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_LOOKUP_EVENT(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    CpswCpts_Event *matchEvent = (CpswCpts_Event *)prms->inArgs;
    CpswCpts_Event *event = (CpswCpts_Event *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = CpswCpts_lookUpEvent(hCpts, matchEvent, event);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to lookup for event: %d\n", status);
    return status;
}


int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_LOOKUP_EST_EVENT(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    CpswCpts_EstEventMatchParams *matchParams = (CpswCpts_EstEventMatchParams *)prms->inArgs;
    CpswCpts_EstEvent *event = (CpswCpts_EstEvent *)prms->outArgs;
    int32_t status = ENET_SOK;

    status = CpswCpts_lookUpEstEvent(hCpts, matchParams->macPort, matchParams->domain, event);
    ENETTRACE_ERR_IF(((status != ENET_SOK) && (status != ENET_ENOTFOUND)),
                     "Failed to lookup for EST event: %d\n", status);
    return status;
}


int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_HANDLE_INTR(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    /* Handle events and notify the respective callbacks */
    status = CpswCpts_handleEvents(hCpts, regs);
    ENETTRACE_ERR_IF(status != ENET_SOK, "Error in handling CPTS interrupt: %d\n", status);
    return status;
}

int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_ENABLE_INTR(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CSL_CPTS_enableInterrupt(regs);
    return status;
}

int32_t CpswCpts_ioctl_handler_CPSW_CPTS_IOCTL_DISABLE_INTR(CpswCpts_Handle hCpts, CSL_cptsRegs *regs, Enet_IoctlPrms *prms)
{
    int32_t status = ENET_SOK;

    CSL_CPTS_disableInterrupt(regs);
    return status;
}

static void CpswCpts_printRegs(CSL_cptsRegs *cptsRegs)
{
    uint32_t *regAddr = (uint32_t *)cptsRegs;
    uint32_t regIdx = 0U;

    while (((uintptr_t)regAddr) < ((uintptr_t)cptsRegs + sizeof(CSL_cptsRegs)))
    {
        if (*regAddr != 0U)
        {
            ENETTRACE_INFO("CPTS: %u: 0x%08x\n", regIdx, *regAddr);
        }

        regAddr++;
        regIdx++;
    }
}

static void CpswCpts_printStats(const CpswCpts_EventStats *stats)
{
#if ENET_CFG_IS_ON(ENET_CFG_CPSW_CPTS_STATS)
    ENETTRACE_INFO("CPSW CPTS statistics counters\n");
    ENETTRACE_INFO("-------------------------------------------------------------\n");
    ENETTRACE_INFO("Software timestamp event count                : %lld\n", stats->swTsPushEventCnt);
    ENETTRACE_INFO("Hardware timestamp push event count           : %lld\n", stats->hwTsPushEventCnt);
    ENETTRACE_INFO("Compare timestamp event count                 : %lld\n", stats->cmpEventCnt);
    ENETTRACE_INFO("Ethernet Rx timestamp count                   : %lld\n", stats->ethRxEventCnt);
    ENETTRACE_INFO("Ethernet Tx timestamp count                   : %lld\n", stats->ethTxEventCnt);
    ENETTRACE_INFO("Ethernet host Tx timestamp count              : %lld\n", stats->ethHostTxEventCnt);
    ENETTRACE_INFO("Hardware timestamp lookup count               : %lld\n", stats->hwTsPushLookupCnt);
    ENETTRACE_INFO("Compare event lookup count                    : %lld\n", stats->cmpEventLookupCnt);
    ENETTRACE_INFO("Ethernet Rx timestamp lookup event count      : %lld\n", stats->ethRxEventLookupCnt);
    ENETTRACE_INFO("Ethernet Tx timestamp lookup event count      : %lld\n", stats->ethTxEventLookupCnt);
    ENETTRACE_INFO("Ethernet host Tx timestamp lookup event count : %lld\n", stats->ethHostTxEventLookupCnt);
    ENETTRACE_INFO("Hardware timestamp discard count              : %lld\n", stats->hwTsPushDiscardCnt);
    ENETTRACE_INFO("Compare event discard count                   : %lld\n", stats->cmpEventDiscardCnt);
    ENETTRACE_INFO("Ethernet Rx timestamp discard event count     : %lld\n", stats->ethRxEventDiscardCnt);
    ENETTRACE_INFO("Ethernet Tx timestamp discard event count     : %lld\n", stats->ethTxEventDiscardCnt);
    ENETTRACE_INFO("Ethernet host Tx timestamp discard event count: %lld\n", stats->ethHostTxEventDiscardCnt);
#endif
}

static int32_t CpswCpts_lookUpEvent(CpswCpts_Handle hCpts,
                                    const CpswCpts_Event *matchEvent,
                                    CpswCpts_Event *event)
{
    CpswCpts_Event *evtPool = NULL;
    CpswCpts_Event *evt;
#if ENET_CFG_IS_ON(ENET_CFG_CPSW_CPTS_STATS)
    CpswCpts_EventStats *stats = &hCpts->eventStats;
#endif
    bool found = false;
    uint32_t i = 0U;
    uintptr_t key;
    int32_t status = ENET_SOK;

    key = EnetOsal_disableAllIntr();

    switch (matchEvent->eventType)
    {
        case CPSW_CPTS_EVENTTYPE_HW_TS_PUSH:
            evtPool = &hCpts->hwPushEventPool.eventMemPool[0U];
            break;

        case CPSW_CPTS_EVENTTYPE_ETH_RECEIVE:
            evtPool = &hCpts->ethRxEventPool.eventMemPool[0U];
            break;

        case CPSW_CPTS_EVENTTYPE_ETH_TRANSMIT:
            evtPool = &hCpts->ethTxEventPool.eventMemPool[0U];
            break;

        case CPSW_CPTS_EVENTTYPE_TS_COMP:
            evtPool = &hCpts->cmpEventPool.eventMemPool[0U];
            break;

        case CPSW_CPTS_EVENTTYPE_TS_HOST_TX:
            evtPool = &hCpts->hostTxEventPool.eventMemPool[0U];
            break;

        default:
            status = ENET_EINVALIDPARAMS;
            break;
    }

    if (status == ENET_SOK)
    {
        /* Traverse through memory pool for a matching event */
        for (i = 0U; i < ENET_CFG_CPSW_CPTS_EVENTS_POOL_SIZE; i++)
        {
            evt = &evtPool[i];

            switch (evt->eventType)
            {
                case CPSW_CPTS_EVENTTYPE_HW_TS_PUSH:
                {
                    if (matchEvent->hwPushNum == evt->hwPushNum)
                    {
                        found = true;
#if ENET_CFG_IS_ON(ENET_CFG_CPSW_CPTS_STATS)
                        stats->hwTsPushLookupCnt++;
#endif
                    }
                }
                break;

                case CPSW_CPTS_EVENTTYPE_ETH_RECEIVE:
                case CPSW_CPTS_EVENTTYPE_ETH_TRANSMIT:
                case CPSW_CPTS_EVENTTYPE_TS_HOST_TX:
                {
                    if ((matchEvent->msgType == evt->msgType) &&
                        (matchEvent->seqId == evt->seqId) &&
                        (matchEvent->domain == evt->domain) &&
                        (matchEvent->portNum == (evt->portNum - 1U)))
                    {
                        found = true;
#if ENET_CFG_IS_ON(ENET_CFG_CPSW_CPTS_STATS)
                        if (matchEvent->eventType == CPSW_CPTS_EVENTTYPE_ETH_RECEIVE)
                        {
                            stats->ethRxEventLookupCnt++;
                        }
                        else if (matchEvent->eventType == CPSW_CPTS_EVENTTYPE_ETH_TRANSMIT)
                        {
                            stats->ethTxEventLookupCnt++;
                        }
                        else if (matchEvent->eventType == CPSW_CPTS_EVENTTYPE_TS_HOST_TX)
                        {
                            stats->ethHostTxEventLookupCnt++;
                        }
#endif
                    }
                }
                break;

                case CPSW_CPTS_EVENTTYPE_TS_COMP:
                {
                    if (matchEvent->eventType == evt->eventType)
                    {
                        found = true;
#if ENET_CFG_IS_ON(ENET_CFG_CPSW_CPTS_STATS)
                        stats->cmpEventLookupCnt++;
#endif
                    }
                }
                break;

                case CPSW_CPTS_EVENTTYPE_TS_PUSH:
                case CPSW_CPTS_EVENTTYPE_TS_ROLLOVER:
                case CPSW_CPTS_EVENTTYPE_TS_HALFROLLOVER:
                case CPSW_CPTS_EVENTTYPE_INVALID:
                {
                    status = ENET_ENOTSUPPORTED;
                }
                break;
            }

            if (found)
            {
                break;
            }
        }

        if (found)
        {
            /* Copy the event information */
            *event = *evt;

            /* Remove the event from pool after copying */
            memset(evt, 0, sizeof(CpswCpts_Event));
            status = ENET_SOK;
        }
        else
        {
            status = ENET_ENOTFOUND;
        }
    }

    EnetOsal_restoreAllIntr(key);

    return status;
}

static void CpswCpts_clearEventMemPools(CpswCpts_Handle hCpts)
{
    uintptr_t key = EnetOsal_disableAllIntr();

    /* TODO: Do we have to clear the pool buffers or just the indexes? */

    memset(&hCpts->hwPushEventPool.eventMemPool[0U], 0, sizeof(hCpts->hwPushEventPool.eventMemPool));
    hCpts->hwPushEventPool.index = 0U;

    memset(&hCpts->ethTxEventPool.eventMemPool[0U], 0, sizeof(hCpts->ethTxEventPool.eventMemPool));
    hCpts->ethTxEventPool.index = 0U;

    memset(&hCpts->ethRxEventPool.eventMemPool[0U], 0, sizeof(hCpts->ethRxEventPool.eventMemPool));
    hCpts->ethRxEventPool.index = 0U;

    memset(&hCpts->cmpEventPool.eventMemPool[0U], 0U, sizeof(hCpts->cmpEventPool.eventMemPool));
    hCpts->cmpEventPool.index = 0U;

    memset(&hCpts->hostTxEventPool.eventMemPool[0U], 0U, sizeof(hCpts->hostTxEventPool.eventMemPool));
    hCpts->hostTxEventPool.index = 0U;

    EnetOsal_restoreAllIntr(key);
}

static int32_t CpswCpts_checkTsCompCompat(CSL_cptsRegs *regs)
{
    uint32_t tsAddVal = 0U;
    uint32_t tsPpmVal[] = {0U, 0U};
    int32_t status = ENET_SOK;

    tsAddVal = CSL_CPTS_getTSAddVal(regs);
    CSL_CPTS_getTSPpm(regs, tsPpmVal);

    /* Timestamp compare feature is not compatible with
     * timestamp add value or time stamp PPM */
    if ((tsAddVal != 0U) ||
        (tsPpmVal[0U] != 0U) ||
        (tsPpmVal[1U] != 0U))
    {
        status = ENET_ENOTSUPPORTED;
    }

    return status;
}

static int32_t CpswCpts_checkGenfEstfErrata(EnetMod_Handle hMod,
                                            CSL_cptsRegs *regs,
                                            uint32_t index,
                                            bool isGenf)
{
    uint32_t length = 0U;
    int32_t status = ENET_SOK;

    if (ENET_ERRATA_IS_EN(hMod->errata, CPSW_CPTS_ERRATA_GENFN_RECONFIG))
    {
        if (isGenf == true)
        {
            CSL_CPTS_getGENFnLength(regs, index, &length);
        }
        else
        {
            CSL_CPTS_getESTFnLength(regs, index, &length);
        }

        if (length != 0U)
        {
            ENETTRACE_WARN("Re-configuring GENFn/ESTFn is not supported for this CPTS IP version\n");
            status = ENET_ENOTSUPPORTED;
        }
    }

    return status;
}

static uint64_t CpswCpts_calcPpmVal(uint64_t tsPpmVal,
                                    EnetTimeSync_AdjMode adjMode)
{
    uint64_t adjVal = 0U;

    if (adjMode == ENET_TIMESYNC_ADJMODE_PPM)
    {
        adjVal = (CPSW_CPTS_PPM / tsPpmVal);
    }

    if (adjMode == ENET_TIMESYNC_ADJMODE_PPH)
    {
        adjVal = (CPSW_CPTS_PPM_GIGAHERTZ_VAL / tsPpmVal) * CPSW_CPTS_SECONDS_PER_HOUR;
    }

    if (adjVal <= CPSW_CPTS_PPM_MIN_VAL)
    {
        ENETTRACE_WARN("Setting PPM to %u, as %u is less than minimum value (%u)\n",
                       CPSW_CPTS_PPM_MIN_VAL, adjVal, CPSW_CPTS_PPM_MIN_VAL);
        adjVal = CPSW_CPTS_PPM_MIN_VAL;
    }

    return adjVal;
}

static int32_t CpswCpts_lookUpEstEvent(CpswCpts_Handle hCpts,
                                       Enet_MacPort macPort,
                                       uint32_t domain,
                                       CpswCpts_EstEvent *event)
{
    CpswCpts_Event *evtPool = &hCpts->hostTxEventPool.eventMemPool[0U];
    CpswCpts_Event *evt;
#if ENET_CFG_IS_ON(ENET_CFG_CPSW_CPTS_STATS)
    CpswCpts_EventStats *stats = &hCpts->eventStats;
#endif
    bool found = false;
    uint32_t i = 0U;
    uintptr_t key;
    int32_t status = ENET_SOK;

    key = EnetOsal_disableAllIntr();

    /* Traverse through memory pool for a matching event */
    for (i = 0U; i < ENET_CFG_CPSW_CPTS_EVENTS_POOL_SIZE; i++)
    {
        evt = &evtPool[i];

        if ((evt->portNum == ENET_MACPORT_ID(macPort)) &&
            (evt->domain == domain))
        {
            found = true;
#if ENET_CFG_IS_ON(ENET_CFG_CPSW_CPTS_STATS)
            stats->ethHostTxEventLookupCnt++;
#endif
            break;
        }
    }

    if (found)
    {
        event->tsVal    = evt->tsVal;
        event->domain   = evt->domain;
        event->priority = (uint32_t)evt->msgType;
        event->seqNum   = (evt->seqId & CPSW_CPTS_EST_EVENT_SEQNUM_MASK) >> CPSW_CPTS_EST_EVENT_SEQNUM_SHIFT;
        event->ingressPort = (evt->seqId & CPSW_CPTS_EST_EVENT_RXPORT_MASK) >> CPSW_CPTS_EST_EVENT_RXPORT_SHIFT;
        event->egressPort  = evt->portNum;

        /* Remove the event from pool after copying */
        memset(evt, 0, sizeof(CpswCpts_Event));
        status = ENET_SOK;
    }
    else
    {
        status = ENET_ENOTFOUND;
    }

    EnetOsal_restoreAllIntr(key);

    return status;
}

static int32_t CpswCpts_handleEvents(CpswCpts_Handle hCpts,
                                     CSL_cptsRegs *regs)
{
    CpswCpts_Event *evt;
#if ENET_CFG_IS_ON(ENET_CFG_CPSW_CPTS_STATS)
    CpswCpts_EventStats *stats = &hCpts->eventStats;
#endif
    CSL_CPTS_EVENTINFO eventInfo = {0};
    volatile bool intrPendStatus = false;
    uint32_t *idx = NULL;
    uint64_t tsVal = 0U;
    uint32_t hwPushIdx = 0U;
    int32_t status = ENET_SOK;

    /* Disable CPTS interrupt */
    CSL_CPTS_disableInterrupt(regs);

    do
    {
        CSL_CPTS_getEventInfo(regs, &eventInfo);

        /* Pop the previously read value off of the event FIFO */
        CSL_CPTS_popEvent(regs);

        tsVal = ((((uint64_t)(eventInfo.timeStampHi)) << 32U) |
                 ((uint64_t)(eventInfo.timeStamp)));

        if (eventInfo.eventType == CPSW_CPTS_EVENTTYPE_TS_PUSH)
        {
            hCpts->tsVal = tsVal;
            /* Clear the flag that indicates a TS_PUSH event is in the FIFO */
            hCpts->tsPushInFifo = false;
#if ENET_CFG_IS_ON(ENET_CFG_CPSW_CPTS_STATS)
            stats->swTsPushEventCnt++;
#endif
        }
        else
        {
            switch (eventInfo.eventType)
            {
                case CPSW_CPTS_EVENTTYPE_HW_TS_PUSH:
                {
                    idx = &hCpts->hwPushEventPool.index;
                    evt = &hCpts->hwPushEventPool.eventMemPool[*idx];

                    /* Get the HW PUSH input number (1-8) which caused the event */
                    evt->eventType = (CpswCpts_EventType)eventInfo.eventType;
                    evt->hwPushNum = (CpswCpts_HwPush)eventInfo.portNo;

                    /* These fields are meaningless for this event type */
                    evt->seqId   = 0U;
                    evt->domain  = 0U;
                    evt->msgType = ENET_TIMESYNC_MESSAGE_INVALID;

                    /* Check for registered calback and call it */
                    hwPushIdx = CPSW_CPTS_HWPUSH_NORM(evt->hwPushNum);
                    if ((hwPushIdx < CPSW_CPTS_HWPUSH_COUNT_MAX) &&
                        (hCpts->hwPushNotifyCb[hwPushIdx] != NULL))
                    {
                        hCpts->hwPushNotifyCb[hwPushIdx](hCpts->hwPushNotifyCbArg[hwPushIdx],
                                                         evt->hwPushNum);
                    }
#if ENET_CFG_IS_ON(ENET_CFG_CPSW_CPTS_STATS)
                    stats->hwTsPushEventCnt++;
                    if (evt->tsVal != 0U)
                    {
                        stats->hwTsPushDiscardCnt++;
                    }
#endif
                }
                break;

                case CPSW_CPTS_EVENTTYPE_ETH_RECEIVE:
                {
                    idx = &hCpts->ethRxEventPool.index;
                    evt = &hCpts->ethRxEventPool.eventMemPool[*idx];

                    /* For Ethernet events, get the messageType, sequence ID, and port number */
                    evt->eventType = (CpswCpts_EventType)eventInfo.eventType;
                    evt->msgType   = (EnetTimeSync_MsgType)eventInfo.msgType;
                    evt->seqId     = eventInfo.seqId;
                    evt->portNum   = eventInfo.portNo;
                    evt->domain    = eventInfo.domain;

                    /* These fields are meaningless for this event type */
                    evt->hwPushNum = CPSW_CPTS_HWPUSH_INVALID;

#if ENET_CFG_IS_ON(ENET_CFG_CPSW_CPTS_STATS)
                    stats->ethRxEventCnt++;
                    if (evt->tsVal != 0U)
                    {
                        stats->ethRxEventDiscardCnt++;
                    }
#endif
                }
                break;

                case CPSW_CPTS_EVENTTYPE_ETH_TRANSMIT:
                {
                    idx = &hCpts->ethTxEventPool.index;
                    evt = &hCpts->ethTxEventPool.eventMemPool[*idx];

                    /* For Ethernet events, get the messageType, sequence ID, and port number */
                    evt->eventType = (CpswCpts_EventType)eventInfo.eventType;
                    evt->msgType   = (EnetTimeSync_MsgType)eventInfo.msgType;
                    evt->seqId     = eventInfo.seqId;
                    evt->portNum   = eventInfo.portNo;
                    evt->domain    = eventInfo.domain;

                    /* These fields are meaningless for this event type */
                    evt->hwPushNum = CPSW_CPTS_HWPUSH_INVALID;

#if ENET_CFG_IS_ON(ENET_CFG_CPSW_CPTS_STATS)
                    stats->ethTxEventCnt++;
                    if (evt->tsVal != 0U)
                    {
                        stats->ethTxEventDiscardCnt++;
                    }
#endif
                }
                break;

                case CPSW_CPTS_EVENTTYPE_TS_COMP:
                {
                    idx = &hCpts->cmpEventPool.index;
                    evt = &hCpts->cmpEventPool.eventMemPool[*idx];

                    evt->eventType = (CpswCpts_EventType)eventInfo.eventType;

                    /* These fields are meaningless for this event type */
                    evt->portNum   = 0U;
                    evt->seqId     = 0U;
                    evt->domain    = 0U;
                    evt->hwPushNum = CPSW_CPTS_HWPUSH_INVALID;
                    evt->msgType   = ENET_TIMESYNC_MESSAGE_INVALID;

#if ENET_CFG_IS_ON(ENET_CFG_CPSW_CPTS_STATS)
                    stats->cmpEventCnt++;
                    if (evt->tsVal != 0U)
                    {
                        stats->cmpEventDiscardCnt++;
                    }
#endif
                }
                break;

                case CPSW_CPTS_EVENTTYPE_TS_HOST_TX:
                {
                    idx = &hCpts->hostTxEventPool.index;
                    evt = &hCpts->hostTxEventPool.eventMemPool[*idx];

                    evt->eventType = (CpswCpts_EventType)eventInfo.eventType;
                    evt->msgType   = (EnetTimeSync_MsgType)eventInfo.msgType;
                    evt->seqId     = eventInfo.seqId;
                    evt->domain    = eventInfo.domain;
                    evt->portNum   = eventInfo.portNo;

                    /* These fields are meaningless for this event type */
                    evt->hwPushNum = CPSW_CPTS_HWPUSH_INVALID;

#if ENET_CFG_IS_ON(ENET_CFG_CPSW_CPTS_STATS)
                    stats->ethHostTxEventCnt++;
                    if (evt->tsVal != 0U)
                    {
                        stats->ethHostTxEventDiscardCnt++;
                    }
#endif
                }
                break;

                default:
                {
                    status = ENET_EINVALIDPARAMS;
                }
                break;
            }

            if (status == ENET_SOK)
            {
                /* Copy the 64-bit timestamp value to memory pool */
                evt->tsVal = tsVal;

                /* Notify the registered stack that a CPTS event has been detected */
                if (hCpts->eventNotifyCb != NULL)
                {
                    (*hCpts->eventNotifyCb)(hCpts->eventNotifyCbArg, evt);
                }

                /* Increment the index */
                *idx = (((*idx) + 1U) % ENET_CFG_CPSW_CPTS_EVENTS_POOL_SIZE);
            }
        }

        intrPendStatus = CSL_CPTS_isRawInterruptStatusBitSet(regs);
    }
    while (intrPendStatus == true);

    /* Re-enable CPTS interrupt */
    CSL_CPTS_enableInterrupt(regs);

    return status;
}


