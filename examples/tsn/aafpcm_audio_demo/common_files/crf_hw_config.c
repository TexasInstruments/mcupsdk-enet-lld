/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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
#include "tsninit.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include <drivers/i2c.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/TaskP.h>
#include "enetapp_cpsw.h"
#include <drivers/soc.h>
#include <drivers/sciclient.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/pinmux.h>
#include "crf_hw_config.h"

#ifdef SOC_AM275X
#define TIMESYNC_ROUTER_DEVID   (TISCI_DEV_TIMESYNC_EVENT_INTROUTER0)
#else
#error "Not Supported for this SOC"
#endif
/* ========================================================================== */
/*                              Struct & Enums                                */
/* ========================================================================== */
typedef enum
{
    TS_ACQUIRE_STATE_RECORD_LF = 0,
    TS_ACQUIRE_STATE_RECORD_HF,
    TS_ACQUIRE_STATE_END
}tsAcquireState_e;

typedef struct
{
    tsAcquireState_e state; /* State of the statemachine. */
    SemaphoreP_Object StartSem;
    SemaphoreP_Object EndSem;
    int32_t lfCbCount;
    int32_t hfCbCount;
} tsAcquireStateVar;

/* ========================================================================== */
/*                              Global Variables                              */
/* ========================================================================== */

static tsAcquireStateVar gAcquireSm = {0};

/* This is used to access hEnet and Core ID.
   The variable name may vary from application to application.*/
extern EnetApp_Cfg gEnetAppCfg;

int32_t Board_MuxSelPhyRefClk(void);

int32_t Board_CdceConfig(void);

int32_t Board_CdceFineTuneFreq(double relPPM);

static void crfHwConfig_setAtlMux(const uint32_t atlBwsReg, uint32_t muxVal);

static int32_t crfHwConfig_setTimeSyncRouter(const uint32_t src, const uint32_t dest, uint32_t set);

static int32_t crfHwConfig_setClockAtSyncOut(uint32_t atlClkOut);

static int64_t crfHwConfig_estimateEdgeDiff(int32_t atlTsSignal, int32_t atlMcSignal, \
                                                        double mediaClockFreq, double tsSignalFreq);

static int32_t crfHwConfig_attachTssToCpts(int32_t atlTsSignal, void (*tssCb)(void*));

static int32_t crfHwConfig_setTimeSyncRouter(const uint32_t src, const uint32_t dest, uint32_t set)
{
    int32_t                             retVal;

    if (set > 0)
    {
        struct tisci_msg_rm_irq_set_req     rmIrqReq;
        struct tisci_msg_rm_irq_set_resp    rmIrqResp;
        rmIrqReq.valid_params           = 0U;
        rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
        rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
        rmIrqReq.global_event           = 0U;
        rmIrqReq.src_id                 = TIMESYNC_ROUTER_DEVID;
        rmIrqReq.src_index              = src;
        rmIrqReq.dst_id                 = TIMESYNC_ROUTER_DEVID;
        rmIrqReq.dst_host_irq           = dest;
        rmIrqReq.ia_id                  = 0U;
        rmIrqReq.vint                   = 0U;
        rmIrqReq.vint_status_bit_index  = 0U;
        rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;
        retVal = Sciclient_rmIrqSetRaw(&rmIrqReq, &rmIrqResp, SystemP_WAIT_FOREVER);
        if(0 != retVal)
        {
            DebugP_log("[Error] Sciclient event config failed!!!\r\n");
        }
    }
    else
    {
        struct tisci_msg_rm_irq_release_req rmIrqReleaseReq;
        rmIrqReleaseReq.valid_params           = 0;
        rmIrqReleaseReq.valid_params           |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
        rmIrqReleaseReq.valid_params           |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
        rmIrqReleaseReq.src_id                 = TIMESYNC_ROUTER_DEVID;
        rmIrqReleaseReq.src_index              = src;
        rmIrqReleaseReq.dst_id                 = TIMESYNC_ROUTER_DEVID;
        rmIrqReleaseReq.dst_host_irq           = dest;
        rmIrqReleaseReq.ia_id                  = 0;
        rmIrqReleaseReq.vint                   = 0;
        rmIrqReleaseReq.global_event           = 0;
        rmIrqReleaseReq.vint_status_bit_index  = 0;
        rmIrqReleaseReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;
        retVal = Sciclient_rmIrqReleaseRaw(&rmIrqReleaseReq, SystemP_WAIT_FOREVER);
        if(0 != retVal)
        {
            DebugP_log("[Error] Sciclient event config failed!!!\r\n");
        }
    }
    return retVal;
}

static void crfHwConfig_setAtlMux(const uint32_t atlBwsReg, uint32_t muxVal)
{
    volatile uint32_t *ctrl_mmr_addr = \
        (volatile uint32_t*)AddrTranslateP_getLocalAddr(atlBwsReg);

    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, 2);
    *ctrl_mmr_addr = muxVal;
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, 2);
}

static int32_t crfHwConfig_registerCptsPushEvents(CpswCpts_HwPush pushNum, \
                                CpswCpts_HwPushNotifyCb hwPushNotifyCb, void* args)
{
    CpswCpts_RegisterHwPushCbInArgs inArgs;
    inArgs.hwPushNum = pushNum;
    inArgs.hwPushNotifyCb = hwPushNotifyCb;
    inArgs.hwPushNotifyCbArg = args;

    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, CPSW_CPTS_IOCTL_REGISTER_HWPUSH_CALLBACK, &prms, status);
    if(status != ENET_SOK)
    {
        DebugP_log("[Error] Failed to configure HWPUSH CALLBACK !!!\r\n");
        DebugP_assert(false);
    }
    return status;
}

static int32_t crfHwConfig_unRegisterCptsPushEvents(CpswCpts_HwPush pushNum)
{
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;
    ENET_IOCTL_SET_IN_ARGS(&prms, &pushNum);
    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, CPSW_CPTS_IOCTL_UNREGISTER_HWPUSH_CALLBACK, &prms, status);
    if(status != ENET_SOK)
    {
        DebugP_log("[Error] Failed to configure HWPUSH CALLBACK !!!\r\n");
        DebugP_assert(false);
    }

    return status;
}

static inline uint64_t crfHwConfig_absDiff(uint64_t num1, uint64_t num2)
{
    return (num1 > num2) ? (num1 - num2) : (num2 - num1);
}

static int64_t crfHwConfig_getEdgeDiff(uint64_t* topSigArr, uint64_t* bottomSigArr, \
                    uint32_t topSigFreq, uint32_t bottomSigFreq, uint32_t maxJitter_ns)
{
    uint64_t topPeriod_ns = 1e9/(float)topSigFreq;
    uint64_t bottomPeriod_ns = 1e9/(float)bottomSigFreq;

    uint64_t goodTopSample = 0;
    uint64_t goodBottomSample = 0;

    /* Select good samples */
    for (int32_t i = 0; i < 2; i++)
    {
        /* Calc diff */
        if (crfHwConfig_absDiff(topSigArr[i+1]-topSigArr[i], topPeriod_ns) < maxJitter_ns)
        {
            /* Good Samples!!*/
            goodTopSample = topSigArr[i];
        }
        /* Calc diff */
        if (crfHwConfig_absDiff(bottomSigArr[i+1]-bottomSigArr[i], bottomPeriod_ns) < maxJitter_ns)
        {
            /* Good Samples!!*/
            goodBottomSample = bottomSigArr[i];
        }
    }

    DebugP_assert(goodTopSample != 0);
    DebugP_assert(goodBottomSample != 0);
    DebugP_assert(goodTopSample > goodBottomSample);

    int64_t diff = goodTopSample - goodBottomSample;
    double cycleDiff = diff/(double)topPeriod_ns;
    int64_t floor = (int64_t)(cycleDiff);

    diff = (cycleDiff-floor) * topPeriod_ns;

    return diff;
}

static uint64_t crfHwConfig_getCptsHwPushEventTs(uint32_t CptsHwPushNum)
{
    CpswCpts_Event inArgs = {
        .eventType = CPSW_CPTS_EVENTTYPE_HW_TS_PUSH,
        .hwPushNum = CptsHwPushNum,
    };
    CpswCpts_Event outArgs;
    Enet_IoctlPrms prms;
    uint64_t timestamp = 0;
    int32_t status = ENET_SOK;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &outArgs);
    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, CPSW_CPTS_IOCTL_LOOKUP_EVENT, &prms, status);
    if(status != ENET_SOK)
    {
        DebugP_log("No Timstamp found\r\n");
    }
    else
    {
        timestamp = outArgs.tsVal;
    }
    return timestamp;
}

static void crfHwConfig_CptspushNotifyCb(void *hwPushNotifyCbArg, CpswCpts_HwPush hwPushNum)
{
    tsAcquireStateVar* sm = (tsAcquireStateVar*)&gAcquireSm;
    switch (sm->state)
    {
        case TS_ACQUIRE_STATE_RECORD_LF:
            if (hwPushNum == CPSW_CPTS_HWPUSH_1)
            {
                sm->lfCbCount += 1;
                if (sm->lfCbCount >= 3)
                {
                    sm->state = TS_ACQUIRE_STATE_RECORD_HF;
                    sm->hfCbCount = 0;
                    /* Post Start Semaphore. */
                    SemaphoreP_post(&sm->StartSem);
                }
            }
            break;
        case TS_ACQUIRE_STATE_RECORD_HF:
            if (hwPushNum == CPSW_CPTS_HWPUSH_2)
            {
                sm->hfCbCount += 1;
                if (sm->hfCbCount >= 3)
                {
                    sm->state = TS_ACQUIRE_STATE_END;
                    /* Post End Semaphore. */
                    SemaphoreP_post(&sm->EndSem);
                }
            }
            break;
        case TS_ACQUIRE_STATE_END:
        break;
        default:
        break;
    }
}

static int64_t crfHwConfig_estimateEdgeDiff(int32_t atlTsSignal, int32_t atlMcSignal, double mediaClockFreq, double tsSignalFreq)
{
    /* Config Parameters. */
    uint32_t periodJitter   = 500;

    /* Array to store Timestamping signal Edge Timestamps. */
    uint64_t tsSignalTimestamps[3] = {0, 0, 0};

    /* Array to store Media Clock Edge Timestamps. */
    uint64_t mediaClockTimestamps[3] = {0, 0, 0};

    SemaphoreP_constructBinary(&gAcquireSm.StartSem, 0);
    SemaphoreP_constructBinary(&gAcquireSm.EndSem,   0);

    /* Clear up the pends to the Semaphores. */
    SemaphoreP_pend(&gAcquireSm.StartSem, SystemP_NO_WAIT);
    SemaphoreP_pend(&gAcquireSm.EndSem,   SystemP_NO_WAIT);

    crfHwConfig_registerCptsPushEvents(CPSW_CPTS_HWPUSH_1, crfHwConfig_CptspushNotifyCb, (void*)&gAcquireSm);
    crfHwConfig_registerCptsPushEvents(CPSW_CPTS_HWPUSH_2, crfHwConfig_CptspushNotifyCb, (void*)&gAcquireSm);

    /* Start Timestamping Signal Sampling. */
    crfHwConfig_setTimeSyncRouter(atlTsSignal, TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_1, 1);
    SemaphoreP_pend(&gAcquireSm.StartSem, SystemP_WAIT_FOREVER);
    /* Stop Timestamping Signal Sampling. */
    crfHwConfig_setTimeSyncRouter(atlTsSignal, TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_1, 0);

    /* Read Timestamping signal Timestamps*/
    for (int32_t i = 0; i < 3; i++)
    {
        tsSignalTimestamps[i]   = crfHwConfig_getCptsHwPushEventTs(CPSW_CPTS_HWPUSH_1);
    }

    /* Start the Media Clock Sampling. */
    crfHwConfig_setTimeSyncRouter(atlMcSignal, TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_2, 1);
    SemaphoreP_pend(&gAcquireSm.EndSem, SystemP_WAIT_FOREVER);
    /* Stop the Media Clock Sampling. */
    crfHwConfig_setTimeSyncRouter(atlMcSignal, TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_2, 0);

    /* Read Media Clock Timestamps */
    for (int32_t i = 0; i < 3; i++)
    {
        mediaClockTimestamps[i] = crfHwConfig_getCptsHwPushEventTs(CPSW_CPTS_HWPUSH_2);
    }

    DebugP_log("LF: %llu, %llu, %llu\r\n", tsSignalTimestamps[0], tsSignalTimestamps[1], tsSignalTimestamps[2]);
    DebugP_log("HF: %llu, %llu, %llu\r\n", mediaClockTimestamps[0], mediaClockTimestamps[1], mediaClockTimestamps[2]);

    int64_t edgeDifference = crfHwConfig_getEdgeDiff(mediaClockTimestamps, tsSignalTimestamps, \
                                                    mediaClockFreq, tsSignalFreq, periodJitter);

    /* Unregister push event callbacks. */
    crfHwConfig_unRegisterCptsPushEvents(CPSW_CPTS_HWPUSH_1);
    crfHwConfig_unRegisterCptsPushEvents(CPSW_CPTS_HWPUSH_2);

    return edgeDifference;
}

static int32_t crfHwConfig_AtlMuxConfig(uint32_t mediaClkSignal, uint32_t tsClkSignal)
{
    /* Media Clock Signal */
    crfHwConfig_setAtlMux(CTRL_MMR0_CFG0_ATL_BWS1_SEL, mediaClkSignal);

    /* Timestamping Signal. */
    crfHwConfig_setAtlMux(CTRL_MMR0_CFG0_ATL_BWS2_SEL, tsClkSignal);

    return SystemP_SUCCESS;
}

static void CrfHwConfig_pushCb(void *hwPushNotifyCbArg,
                                        CpswCpts_HwPush hwPushNum)
{
    void (*tssCb)(void*) = hwPushNotifyCbArg;
    tssCb(NULL);
}

static int32_t crfHwConfig_setClockAtSyncOut(uint32_t atlClkOut)
{
    crfHwConfig_setAtlMux(CTRL_MMR0_CFG0_ATL_BWS0_SEL, atlClkOut);

    return crfHwConfig_setTimeSyncRouter(TS_ROUTER_IN_ATL_BWS_SEL_0, TS_ROUTER_OUT_SYNC1_OUT, 1);
}

static int32_t crfHwConfig_attachTssToCpts(int32_t atlTsSignal, void (*tssCb)(void*))
{
    crfHwConfig_registerCptsPushEvents(CPSW_CPTS_HWPUSH_1, CrfHwConfig_pushCb, tssCb);
    crfHwConfig_setTimeSyncRouter(atlTsSignal, TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_1, 1);
    return SystemP_SUCCESS;
}

static uint64_t crfHwConfig_getPhyMediaClkEdgeTs(void)
{
    uint64_t retval = 0;
    int32_t status = ENET_EFAIL;
    Enet_IoctlPrms prms;
    EnetPhy_GetEventTimestampOutArgs outArgs;
    EnetPhy_GenericInArgs inArgs;

    inArgs.macPort = ENET_MAC_PORT_1;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &outArgs);
    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId,
            ENET_PHY_IOCTL_GET_EVENT_TIMESTAMP, &prms, status);

    if (status == ENET_SOK)
    {
        retval = outArgs.ts64;
    }

    return retval;
}

static int32_t crfHwConfig_registerPhyTsEvent(void)
{
    int32_t status = ENET_EFAIL;
    Enet_IoctlPrms prms;
    EnetPhy_EnableEventCaptureInArgs inArgs = {
        .macPort  = ENET_MAC_PORT_1,
        .eventIdx = 1, /* TS SYNC OUT is connected to Event Index 1 */
        .falling  = false,
        .on       = true,
    };
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId,
            ENET_PHY_IOCTL_ENABLE_EVENT_CAPTURE, &prms, status);

    return status;
}

uint64_t crfHwConfig_getMediaClockEdge(const crfHwCfg_info* info)
{
    uint64_t mediaClkEdge = 0;

    if (info->clkSrc == CRF_HW_CONFIG_CLKSRC_PHY)
    {
        mediaClkEdge = crfHwConfig_getPhyMediaClkEdgeTs();
    }
    else
    {
        mediaClkEdge = crfHwConfig_getCptsHwPushEventTs(CPSW_CPTS_HWPUSH_1);
    }

    if (mediaClkEdge != 0)
    {
        mediaClkEdge += info->edgeDiff;
    }

    return mediaClkEdge;
}

static int32_t crfHwConfig_NudgePhyClock(int8_t cycles)
{
    int32_t status = ENET_EFAIL;
    EnetPhy_NudgeCodecClockInArgs inArgs = {
        .macPort = ENET_MAC_PORT_1,
        .nudgeValue = cycles,
    };
    Enet_IoctlPrms prms;
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId,
            ENET_PHY_IOCTL_NUDGE_CODEC_CLOCK, &prms, status);
    return status;
}

int32_t crfHwConfig_init(const crfHwCfg_info* info)
{
    int32_t status = ENET_SOK;

    if (info->clkSrc == CRF_HW_CONFIG_CLKSRC_PHY)
    {
        /* Select Phy's Clk as AUDIO_REFCLK2 Input. */
        status |= Board_MuxSelPhyRefClk();
    }
    else
    {
        /* Configure CDCE to Generate CLK. */
        status |= Board_CdceConfig();
    }

    crfHwConfig_AtlMuxConfig(info->mediaClkAtlSignal, info->timestampingAtlSignal);

    if (info->clkSrc == CRF_HW_CONFIG_CLKSRC_PHY)
    {
        crfHwConfig_setClockAtSyncOut(info->timestampingAtlSignal);
    }
    else
    {
        crfHwConfig_setClockAtSyncOut(info->mediaClkAtlSignal);
    }

    return status;
}

int32_t crfHwConfig_setup(crfHwCfg_info* info)
{
    int64_t edgeDiff = crfHwConfig_estimateEdgeDiff(TS_ROUTER_IN_ATL_BWS_SEL_2, TS_ROUTER_IN_ATL_BWS_SEL_1, \
                            info->mediaClkFreq, info->mediaClkFreq/info->timestampingInterval);

    info->edgeDiff = edgeDiff;

    if (info->clkSrc == CRF_HW_CONFIG_CLKSRC_PHY)
    {
        crfHwConfig_registerPhyTsEvent();
#ifdef CONFIG_CRF_TIMER
        TimerP_start(gTimerBaseAddr[CONFIG_CRF_TIMER]);
#endif
    }
    else
    {
        crfHwConfig_attachTssToCpts(TS_ROUTER_IN_ATL_BWS_SEL_2, CrfHwConfig_crfTsCb);
    }

    return SystemP_SUCCESS;
}

int32_t crfHwConfig_fineTuneFreq(const crfHwCfg_info* info, double relPPM)
{
    int32_t status = SystemP_FAILURE;

    if (info->clkSrc == CRF_HW_CONFIG_CLKSRC_CDCE)
    {
        status = Board_CdceFineTuneFreq(relPPM);
    }
    else if (info->clkSrc == CRF_HW_CONFIG_CLKSRC_PHY)
    {
        DebugP_log("%s:Not Supported\r\n", __func__);
    }
    else
    {
        DebugP_log("%s:Not Supported\r\n", __func__);
    }

    return status;
}

int32_t crfHwConfig_adjPhase(const crfHwCfg_info* info, int8_t cycles)
{
    int32_t status = SystemP_FAILURE;

    if (info->clkSrc == CRF_HW_CONFIG_CLKSRC_CDCE)
    {
        DebugP_log("%s:Not Supported\r\n", __func__);
    }
    else if (info->clkSrc == CRF_HW_CONFIG_CLKSRC_PHY)
    {
        crfHwConfig_NudgePhyClock(cycles);
    }
    else
    {
        DebugP_log("%s:Not Supported\r\n", __func__);
    }

    return status;
}
