/*
 *  Copyright (C) Texas Instruments Incorporated 2026
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
 * \file clocksync.c
 *
 * \brief ClockSync state machine and public API. Ports the ratio/offset
 *        adaptive filter from the enet_followerclock prototype
 *        (test/clock_sync_demo/enet_followerclock/follower_clock_sm.c) as
 *        m/c, restructured as a singleton with no RTOS dependency.
 */

#include <stdbool.h>
#include <FreeRTOS.h>
#include <task.h>
#include <kernel/dpl/TimerP.h>
#include <enet_apputils_k3.h>
#include <tsn_unibase/unibase.h>
#ifndef UB_LOGCAT
#define UB_LOGCAT 5
#endif
#include "clocksync.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define CLOCKSYNC_HW_ADDR_TODO      ((uint32_t)0xFFFFFFFFU)
#define CLOCKSYNC_HW_INDEX_TODO     ((uint32_t)0xFFFFFFFFU)

/* Timer PWM trigger-out mode value written to TCLR.TRG by ClockSync_setPwmTriggerMode(). */
#define CLOCKSYNC_PWM_TRIG_MODE              (1U)

#define CLOCKSYNC_TIMER_TCLR                 (uint32_t)0x38U
#define CLOCKSYNC_TIMER_TCLR_TRG_MASK        (uint32_t)0xC00U
#define CLOCKSYNC_TIMER_TCLR_TRG_SHIFT       (uint32_t)0x0AU
#define CLOCKSYNC_TIMER_TWPS                 (uint32_t)0x48U
#define CLOCKSYNC_TIMER_TWPS_TCLR_PEND_MASK  (uint32_t)0x1U

/* Hardcoded timer period; ClockSync_init() takes no period argument. */
#define CLOCKSYNC_TIMER_PERIOD_USEC        (100000U)

/* Hardcoded sync threshold, in nanoseconds, ported from the prototype's target_error_s. */
#define CLOCKSYNC_TARGET_ERROR_NS          (500)

/* alpha is the weight kept on the old m/c estimate: high = smooth and slow, low = follows
 * the new sample quickly. It drops when the error is big and goes back to INIT once small. */
#define CLOCKSYNC_ALPHA_MIN                 (0.05)
#define CLOCKSYNC_ALPHA_MAX                 (0.95)
#define CLOCKSYNC_ALPHA_INIT                (0.95)
#define CLOCKSYNC_ERROR_FACTOR_MAX          (10.0)

/* SET: wait for the first push. BEGIN: seed m/c from two pushes.
 * CONTROLLER: normal tracking with the adaptive filter. */
typedef enum ClockSync_State_e
{
    CLOCKSYNC_STATE_SET,
    CLOCKSYNC_STATE_BEGIN,
    CLOCKSYNC_STATE_CONTROLLER,
    CLOCKSYNC_STATE_STOP,
} ClockSync_State;

typedef struct ClockSync_Obj_s
{
    bool initialized;

    Enet_Handle hEnet;
    uint32_t coreId;
    int32_t timerInstance;
    CpswCpts_HwPush pushNum;

    uint32_t timerBaseAddr;
    uint32_t reloadCount;
    volatile uint32_t overflowCount;

    ClockSync_State state;
    /* gPTP time in ns is modelled as timerCount * m + c (m is ns per timer tick). */
    double m;
    double c;
    double alphaM;
    double alphaC;
    int64_t error;
    bool synced;
    uint64_t prevCptsNs;
    uint64_t prevTimerCount;
} ClockSync_Obj;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static ClockSync_Obj gClockSyncObj;

#if defined(SOC_AM275X)
#define TIMESYNC_ROUTER_DEVID       (TISCI_DEV_TIMESYNC_EVENT_INTROUTER0)

/* 
  These are the Timer based addresses taken from AM275 TRM. 
  TIMER0 address is placed at index 0 and so on.
*/
static const uint32_t gClockSyncTimerBaseAddr[] = { 
    0x2400000u, 0x2410000u, 0x2420000u, 0x2430000u, 
    0x2440000u, 0x2450000u, 0x2460000u, 0x2470000u,
    0x2480000u
 };

/* 
  These are Time Sync Router Source Indices connected 
  to Timer PWM Out. TIMER0 PWM out is mapped to Index 0
   of this array.
*/
static const uint32_t gClockSyncRouterSrcIndex[] = {
    0, 1, 2, 3,
    4, 5, 6, 7,
    8
};

/* 
  These are Time Sync Router Destination Indices connected 
  to CPTS HW Push Event Inputs. Index 0 of this array is 
  mapped to CPSW_CPTS_HWPUSH_1 and so on.
*/
static const uint32_t gClockSyncRouterDstHostIrq[CPSW_CPTS_HWPUSH_COUNT_MAX] =
{
     8,  9, 10, 11,
    12, 13, 14, 15
};

#else
#error "ClockSync: SOC not supported"
#endif

int32_t ClockSync_getTimerBaseAddr(int32_t timerInstance, uint32_t *baseAddr);

int32_t ClockSync_connectTimeSyncRouter(int32_t timerInstance, CpswCpts_HwPush pushNum);

int32_t ClockSync_disconnectTimeSyncRouter(int32_t timerInstance, CpswCpts_HwPush pushNum);

void ClockSync_setPwmTriggerMode(uint32_t timerBaseAddr, uint32_t pwmTrigOutMode);




/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

__attribute__((weak)) void ClockSync_HwPushCallbackHook(void)
{
    /* Weak default - application is expected to override this. */
}

static uint64_t ClockSync_getTimerCount(void)
{
    uint32_t overflow1, overflow2;
    uint64_t count;

    /* Lock-free retry against the overflow counter, which is incremented in
     * ClockSync_internalHwPushCb() (ISR context). */
    do
    {
        overflow1 = gClockSyncObj.overflowCount;
        count = (uint64_t)(TimerP_getCount(gClockSyncObj.timerBaseAddr) - gClockSyncObj.reloadCount);
        overflow2 = gClockSyncObj.overflowCount;
    } while (overflow1 != overflow2);

    /* Add the full timer periods that have already wrapped (period = MAX - reload ticks). */
    count += (uint64_t)overflow1 * (MAX_TIMER_COUNT_VALUE - gClockSyncObj.reloadCount);

    return count;
}

/* Sample used for m/c update in tick() - overflow-count multiple only, no
 * live TimerP_getCount() read. Matches the prototype's
 * FollowerClock_getOverflowedCount(): sync happens at the Timer overflow
 * event (HW push callback), whose overflowCount is already latched by the
 * time tick() runs, so reading the live counter here would just add jitter. */
static uint64_t ClockSync_getOverflowedCount(void)
{
    return (uint64_t)gClockSyncObj.overflowCount * (MAX_TIMER_COUNT_VALUE - gClockSyncObj.reloadCount);
}

static int32_t ClockSync_getCptsPushTimestamp(uint64_t *tsVal)
{
    CpswCpts_Event inArgs =
    {
        .eventType = CPSW_CPTS_EVENTTYPE_HW_TS_PUSH,
        .hwPushNum = gClockSyncObj.pushNum,
    };
    CpswCpts_Event outArgs;
    Enet_IoctlPrms prms;
    int32_t status;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &outArgs);
    ENET_IOCTL(gClockSyncObj.hEnet, gClockSyncObj.coreId, CPSW_CPTS_IOCTL_LOOKUP_EVENT, &prms, status);

    if (status == ENET_SOK)
    {
        *tsVal = outArgs.tsVal;
    }

    return status;
}

static void ClockSync_updateAlpha(int64_t error, double *alpha)
{
    double errorAbs = (error < 0) ? -(double)(error) : (double)error;
    double errorFactor = errorAbs / (double)CLOCKSYNC_TARGET_ERROR_NS;
    double adaptiveAlpha;

    /* Error above target shrinks alpha in proportion (capped at 10x) so we re-converge faster. */
    if (errorFactor > CLOCKSYNC_ERROR_FACTOR_MAX)
    {
        errorFactor = CLOCKSYNC_ERROR_FACTOR_MAX;
    }

    adaptiveAlpha = (errorAbs > (double)CLOCKSYNC_TARGET_ERROR_NS) ? (*alpha / errorFactor) : CLOCKSYNC_ALPHA_INIT;

    if (adaptiveAlpha < CLOCKSYNC_ALPHA_MIN)
    {
        adaptiveAlpha = CLOCKSYNC_ALPHA_MIN;
    }
    if (adaptiveAlpha > CLOCKSYNC_ALPHA_MAX)
    {
        adaptiveAlpha = CLOCKSYNC_ALPHA_MAX;
    }

    *alpha = adaptiveAlpha;
}

static void ClockSync_internalHwPushCb(void *cbArg, CpswCpts_HwPush pushNum, uint64_t tsVal)
{
    (void)cbArg;
    (void)pushNum;

    /* Timer overflow callback is not utilized - the overflow count is
     * incremented here, in the CPTS HW push callback, instead. */
    gClockSyncObj.overflowCount++;

    ClockSync_HwPushCallbackHook();
}

int32_t ClockSync_init(Enet_Handle hEnet, uint32_t coreId, int32_t timerInstance, CpswCpts_HwPush pushNum)
{
    int32_t status;
    uint32_t timerBaseAddr;

    /* Idea: timer overflow -> PWM out -> time sync router -> CPTS HW push. CPTS stamps each
     * overflow in gPTP time, and tick() fits m/c against the timer count from those stamps. */
    status = ClockSync_getTimerBaseAddr(timerInstance, &timerBaseAddr);
    if (status != ENET_SOK)
    {
        UB_LOG(UBL_ERROR, "%s: ClockSync_getTimerBaseAddr failed\r\n", __func__);
    }

    if (status == ENET_SOK)
    {
        status = ClockSync_connectTimeSyncRouter(timerInstance, pushNum);
        if (status != ENET_SOK)
        {
            UB_LOG(UBL_ERROR, "%s: ClockSync_connectTimeSyncRouter failed\r\n", __func__);
        }
    }

    if (status == ENET_SOK)
    {
        CpswCpts_RegisterHwPushCbInArgs inArgs =
        {
            .hwPushNum = pushNum,
            .hwPushNotifyCb = ClockSync_internalHwPushCb,
            .hwPushNotifyCbArg = NULL,
        };
        Enet_IoctlPrms prms;
        int32_t ioctlStatus;

        ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
        ENET_IOCTL(hEnet, coreId, CPSW_CPTS_IOCTL_REGISTER_HWPUSH_CALLBACK, &prms, ioctlStatus);
        status = (ioctlStatus == ENET_SOK) ? ENET_SOK : ENET_EFAIL;
        if (status != ENET_SOK)
        {
            UB_LOG(UBL_ERROR, "%s: ENET_IOCTL REGISTER_HWPUSH_CALLBACK failed\r\n", __func__);
        }
    }

    if (status == ENET_SOK)
    {
        TimerP_Params timerParams;

        gClockSyncObj.hEnet = hEnet;
        gClockSyncObj.coreId = coreId;
        gClockSyncObj.timerInstance = timerInstance;
        gClockSyncObj.pushNum = pushNum;
        gClockSyncObj.timerBaseAddr = timerBaseAddr;
        gClockSyncObj.overflowCount = 0U;

        gClockSyncObj.state = CLOCKSYNC_STATE_SET;
        gClockSyncObj.m = 1.0;
        gClockSyncObj.c = 0.0;
        gClockSyncObj.alphaM = CLOCKSYNC_ALPHA_INIT;
        gClockSyncObj.alphaC = CLOCKSYNC_ALPHA_INIT;
        gClockSyncObj.error = 0;
        gClockSyncObj.synced = false;
        gClockSyncObj.prevCptsNs = 0U;
        gClockSyncObj.prevTimerCount = 0U;

        TimerP_Params_init(&timerParams);
        timerParams.periodInUsec = CLOCKSYNC_TIMER_PERIOD_USEC;

        status = (TimerP_setup(timerBaseAddr, &timerParams) == 0) ? ENET_SOK : ENET_EFAIL;
        if (status != ENET_SOK)
        {
            UB_LOG(UBL_ERROR, "%s: TimerP_setup failed\r\n", __func__);
        }
    }

    if (status == ENET_SOK)
    {
        gClockSyncObj.reloadCount = TimerP_getReloadCount(timerBaseAddr);

        ClockSync_setPwmTriggerMode(timerBaseAddr, CLOCKSYNC_PWM_TRIG_MODE);

        status = (TimerP_start(timerBaseAddr) == SystemP_SUCCESS) ? ENET_SOK : ENET_EFAIL;
        if (status != ENET_SOK)
        {
            UB_LOG(UBL_ERROR, "%s: TimerP_start failed\r\n", __func__);
        }
    }

    gClockSyncObj.initialized = (status == ENET_SOK);
    if (status != ENET_SOK)
    {
        UB_LOG(UBL_ERROR, "%s: init failed, status=%d\r\n", __func__, (int)status);
    }

    return status;
}

int32_t ClockSync_tick(void)
{
    int32_t status = ENET_EFAIL;
    uint64_t cptsNs;
    uint64_t timerCount;

    if (gClockSyncObj.initialized && (gClockSyncObj.state != CLOCKSYNC_STATE_STOP))
    {
        status = ClockSync_getCptsPushTimestamp(&cptsNs);
    }

    if (status == ENET_SOK)
    {
        timerCount = ClockSync_getOverflowedCount();

        taskENTER_CRITICAL();

        switch (gClockSyncObj.state)
        {
            case CLOCKSYNC_STATE_SET:
                /* Nothing to compute yet, this push only primes prevCptsNs/prevTimerCount below. */
                gClockSyncObj.state = CLOCKSYNC_STATE_BEGIN;
                break;

            case CLOCKSYNC_STATE_BEGIN:
                gClockSyncObj.m = (double)(cptsNs - gClockSyncObj.prevCptsNs) /
                                  (double)(timerCount - gClockSyncObj.prevTimerCount);
                gClockSyncObj.c = (double)cptsNs - gClockSyncObj.m * (double)timerCount;
                gClockSyncObj.state = CLOCKSYNC_STATE_CONTROLLER;
                break;

            case CLOCKSYNC_STATE_CONTROLLER:
            {
                double approxNs = (double)timerCount * gClockSyncObj.m + gClockSyncObj.c;
                double currentM = (double)(cptsNs - gClockSyncObj.prevCptsNs) /
                                  (double)(timerCount - gClockSyncObj.prevTimerCount);
                double currentC = (double)cptsNs - currentM * (double)timerCount;
                int64_t errorAbs;

                /* Error = how far the old m/c prediction was from the real CPTS stamp. */
                gClockSyncObj.error = (int64_t)((double)cptsNs - approxNs);

                ClockSync_updateAlpha(gClockSyncObj.error, &gClockSyncObj.alphaM);
                ClockSync_updateAlpha(gClockSyncObj.error, &gClockSyncObj.alphaC);

                gClockSyncObj.m = gClockSyncObj.alphaM * gClockSyncObj.m +
                                  currentM - gClockSyncObj.alphaM * currentM;
                gClockSyncObj.c = gClockSyncObj.alphaC * gClockSyncObj.c +
                                  currentC - gClockSyncObj.alphaC * currentC;

                errorAbs = (gClockSyncObj.error < 0) ? -gClockSyncObj.error : gClockSyncObj.error;
                gClockSyncObj.synced = (errorAbs <= CLOCKSYNC_TARGET_ERROR_NS);
                break;
            }

            case CLOCKSYNC_STATE_STOP:
            default:
                break;
        }

        gClockSyncObj.prevCptsNs = cptsNs;
        gClockSyncObj.prevTimerCount = timerCount;

        taskEXIT_CRITICAL();
    }

    return status;
}

int64_t ClockSync_getCurrentTimeNs(void)
{
    int64_t timeNs = ENET_EFAIL;

    if (gClockSyncObj.initialized)
    {
        /* Cheap path: one timer read plus m/c, no CPTS ioctl, so it is fine to call often. */
        double timerCount = (double)ClockSync_getTimerCount();
        taskENTER_CRITICAL();
        double m = gClockSyncObj.m;
        double c = gClockSyncObj.c;
        taskEXIT_CRITICAL();

        timeNs = (int64_t)(timerCount * m + c);
    }

    return timeNs;
}

int32_t ClockSync_getSyncStatus(void)
{
    int32_t syncStatus = ENET_EFAIL;

    if (gClockSyncObj.initialized)
    {
        taskENTER_CRITICAL();
        bool synced = gClockSyncObj.synced;
        taskEXIT_CRITICAL();

        syncStatus = synced ? 1 : 0;
    }

    return syncStatus;
}

void ClockSync_timeout(void)
{
    if (gClockSyncObj.initialized)
    {
        taskENTER_CRITICAL();
        gClockSyncObj.synced = false;
        taskEXIT_CRITICAL();
    }
}

void ClockSync_deInit(void)
{
    if (gClockSyncObj.initialized)
    {
        Enet_IoctlPrms prms;
        int32_t ioctlStatus;
        CpswCpts_HwPush pushNum = gClockSyncObj.pushNum;

        TimerP_stop(gClockSyncObj.timerBaseAddr);

        ClockSync_disconnectTimeSyncRouter(gClockSyncObj.timerInstance, gClockSyncObj.pushNum);

        ENET_IOCTL_SET_IN_ARGS(&prms, &pushNum);
        ENET_IOCTL(gClockSyncObj.hEnet, gClockSyncObj.coreId, CPSW_CPTS_IOCTL_UNREGISTER_HWPUSH_CALLBACK, &prms, ioctlStatus);

        gClockSyncObj.state = CLOCKSYNC_STATE_STOP;
        gClockSyncObj.initialized = false;
    }
}

int32_t ClockSync_connectTimeSyncRouter(int32_t timerInstance, CpswCpts_HwPush pushNum)
{
    int32_t status = ENET_EFAIL;
    uint32_t srcIndex = CLOCKSYNC_HW_INDEX_TODO;
    uint32_t dstHostIrq = CLOCKSYNC_HW_INDEX_TODO;

    if ((timerInstance >= 0) &&
        (timerInstance < (int32_t)(sizeof(gClockSyncRouterSrcIndex) / sizeof(gClockSyncRouterSrcIndex[0]))))
    {
        srcIndex = gClockSyncRouterSrcIndex[timerInstance];
    }

    if ((pushNum >= CPSW_CPTS_HWPUSH_FIRST) && (pushNum <= CPSW_CPTS_HWPUSH_LAST))
    {
        dstHostIrq = gClockSyncRouterDstHostIrq[CPSW_CPTS_HWPUSH_NORM(pushNum)];
    }

    if ((srcIndex != CLOCKSYNC_HW_INDEX_TODO) && (dstHostIrq != CLOCKSYNC_HW_INDEX_TODO))
    {
        status = EnetAppUtils_setTimeSyncRouter(0, 0, srcIndex, dstHostIrq);
    }

    return status;
}

int32_t ClockSync_disconnectTimeSyncRouter(int32_t timerInstance, CpswCpts_HwPush pushNum)
{
    int32_t status = ENET_EFAIL;
    uint32_t srcIndex = CLOCKSYNC_HW_INDEX_TODO;
    uint32_t dstHostIrq = CLOCKSYNC_HW_INDEX_TODO;

    if ((timerInstance >= 0) &&
        (timerInstance < (int32_t)(sizeof(gClockSyncRouterSrcIndex) / sizeof(gClockSyncRouterSrcIndex[0]))))
    {
        srcIndex = gClockSyncRouterSrcIndex[timerInstance];
    }

    if ((pushNum >= CPSW_CPTS_HWPUSH_FIRST) && (pushNum <= CPSW_CPTS_HWPUSH_LAST))
    {
        dstHostIrq = gClockSyncRouterDstHostIrq[CPSW_CPTS_HWPUSH_NORM(pushNum)];
    }

    if ((srcIndex != CLOCKSYNC_HW_INDEX_TODO) && (dstHostIrq != CLOCKSYNC_HW_INDEX_TODO))
    {
        EnetAppUtils_releaseTimeSyncRouter(0, 0, srcIndex, dstHostIrq);
    }

    return status;
}


int32_t ClockSync_getTimerBaseAddr(int32_t timerInstance, uint32_t *baseAddr)
{
    int32_t status = ENET_EFAIL;

    if ((baseAddr != NULL) && (timerInstance >= 0) &&
        (timerInstance < (int32_t)ENET_ARRAYSIZE(gClockSyncTimerBaseAddr)) &&
        (gClockSyncTimerBaseAddr[timerInstance] != CLOCKSYNC_HW_ADDR_TODO))
    {
        *baseAddr = gClockSyncTimerBaseAddr[timerInstance];
        status = ENET_SOK;
    }

    return status;
}


void ClockSync_setPwmTriggerMode(uint32_t timerBaseAddr, uint32_t pwmTrigOutMode)
{
    volatile uint32_t *twpsAddr = (volatile uint32_t *)(timerBaseAddr + CLOCKSYNC_TIMER_TWPS);
    volatile uint32_t *tclrAddr = (volatile uint32_t *)(timerBaseAddr + CLOCKSYNC_TIMER_TCLR);
    uint32_t regVal;

    /* TCLR writes post through to the timer clock domain, so wait out any pending write first. */
    while ((*twpsAddr & CLOCKSYNC_TIMER_TWPS_TCLR_PEND_MASK) == CLOCKSYNC_TIMER_TWPS_TCLR_PEND_MASK)
    {
    }

    regVal = *tclrAddr;
    regVal &= ~CLOCKSYNC_TIMER_TCLR_TRG_MASK;
    regVal |= (pwmTrigOutMode << CLOCKSYNC_TIMER_TCLR_TRG_SHIFT);
    *tclrAddr = regVal;

    while ((*twpsAddr & CLOCKSYNC_TIMER_TWPS_TCLR_PEND_MASK) == CLOCKSYNC_TIMER_TWPS_TCLR_PEND_MASK)
    {
    }
}
