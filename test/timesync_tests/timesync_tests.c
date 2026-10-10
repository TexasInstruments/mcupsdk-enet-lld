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
 * \file  timesync_tests.c
 *
 * \brief This file contains the time sync test entry point
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include <stdio.h>
#include <stdarg.h>
#include <drivers/i2c.h>
#include <drivers/mcasp.h>
#include <drivers/pinmux.h>

#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/DebugP.h>

#include "ti_dpl_config.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"

#include "timesync_tests.h"
#include "tsninit.h"
#include "enetapp_cpsw.h"
#include "tsn_gptp/gptpmasterclock.h"

extern EnetApp_Cfg gEnetAppCfg;

/* Forward declarations for functions defined later in this file */
int getTsrDestForHwPushEvent(CpswCpts_HwPush pushNum);
int getTsrSrcForGenfOut(int genfIdx);
int32_t stopGenf(int inst);

/* External variables (defined later in this file) */
extern SemaphoreP_Object NotifyPushCallback;
extern volatile void* argsInCallback;
extern volatile CpswCpts_HwPush cptsHwPushNumberInCallback;
extern void cpts_hwpush_callback(void *hwPushNotifyCbArg, CpswCpts_HwPush hwPushNum);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

#define HWPUSH_MAX_IDX 8

typedef struct
{
    int pushIdx;
    int status;
} HwPushResult;

int test_hardwarePush_events(int genfIdx, int freq, int cpts_push_idx, HwPushResult *result);

static void EnetApp_setPinmux(void)
{
    #if defined (SOC_AM275X)
    Pinmux_PerCfg_t gPinMuxMainDomainCfg[] =
    {
        {
            PIN_EXT_REFCLK1,
            ( PIN_MODE(1) )
        },
        {PINMUX_END, 0U}
    };

    Pinmux_config(gPinMuxMainDomainCfg, PINMUX_DOMAIN_ID_MAIN);
    #endif
}

int32_t configureGenf(int inst, int freq, uint64_t startTime)
{
    Enet_IoctlPrms prms;
    CpswCpts_SetFxnGenInArgs setGenFInArgs;
    int32_t status;

    uint64_t lengthTicks = 200*1000*1000/freq;

    memset(&setGenFInArgs, 0, sizeof(setGenFInArgs));
    setGenFInArgs.index       = inst;
    setGenFInArgs.length      = lengthTicks;
    setGenFInArgs.compare     = startTime;
    setGenFInArgs.polarityInv = false;
    setGenFInArgs.ppmVal      = 0U;
    setGenFInArgs.ppmDir      = CPSW_CPTS_GENF_PPM_ADJDIR_INCREASE;
    setGenFInArgs.ppmMode     = ENET_TIMESYNC_ADJMODE_DISABLE;

    ENET_IOCTL_SET_IN_ARGS(&prms, &setGenFInArgs);
    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, CPSW_CPTS_IOCTL_SET_GENF,
               &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("pacing: GENF%u config failed: %d\r\n", inst,
                           status);
    }
    return status;
}

int32_t stopGenf(int inst)
{
    Enet_IoctlPrms prms;
    CpswCpts_SetFxnGenInArgs setGenFInArgs;
    int32_t status;

    memset(&setGenFInArgs, 0, sizeof(setGenFInArgs));
    setGenFInArgs.index       = inst;
    setGenFInArgs.length      = 0;
    setGenFInArgs.compare     = 0;
    setGenFInArgs.polarityInv = false;
    setGenFInArgs.ppmVal      = 0U;
    setGenFInArgs.ppmDir      = CPSW_CPTS_GENF_PPM_ADJDIR_INCREASE;
    setGenFInArgs.ppmMode     = ENET_TIMESYNC_ADJMODE_DISABLE;

    ENET_IOCTL_SET_IN_ARGS(&prms, &setGenFInArgs);
    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, CPSW_CPTS_IOCTL_SET_GENF,
               &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("pacing: GENF%u config failed: %d\r\n", inst,
                           status);
    }
    return status;
}

static int32_t registerCptsPushEvents(CpswCpts_HwPush pushNum, \
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

static int32_t unRegisterCptsPushEvents(CpswCpts_HwPush pushNum)
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

static uint64_t getCptsHwPushEventTs(uint32_t CptsHwPushNum)
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

static void run_genf_and_hw_push_tests(void)
{
    int genfIndices[] = {0, 1};
    int frequencies[] = {1000, 500, 100, 10, 1};
    int numGenf = 2;
    int numFreq = 5;
    int numPush = HWPUSH_MAX_IDX;

    HwPushResult results[2][5][8];

    for (int g = 0; g < numGenf; g++)
    {
        for (int f = 0; f < numFreq; f++)
        {
            int genfIdx = genfIndices[g];
            int freq = frequencies[f];

            DebugP_log("=== Testing genfIdx=%d, freq=%dHz ===\r\n", genfIdx, freq);

            for (int p = 0; p < numPush; p++)
            {
                int pushIdx = CPSW_CPTS_HWPUSH_FIRST + p;
                test_hardwarePush_events(genfIdx, freq, pushIdx, &results[g][f][p]);
            }
        }
    }

    /* Print summary table */
    DebugP_log("\r\n=== TEST SUMMARY ===\r\n");
    DebugP_log("Genf Index | Frequency | Hw Push Idx 1 | Hw Push Idx 2 | Hw Push Idx 3 | Hw Push Idx 4 | Hw Push Idx 5 | Hw Push Idx 6 | Hw Push Idx 7 | Hw Push Idx 8\r\n");
    DebugP_log("-----------|-----------|---------------|---------------|---------------|---------------|---------------|---------------|---------------|--------------\r\n");
    for (int g = 0; g < numGenf; g++)
    {
        for (int f = 0; f < numFreq; f++)
        {
            DebugP_log("     %d     |    %4d    |", genfIndices[g], frequencies[f]);
            for (int p = 0; p < numPush; p++)
            {
                DebugP_log("     %4s     |", (results[g][f][p].status == ENET_SOK) ? "PASS" : "FAIL");
            }
            DebugP_log("\r\n");
        }
    }

    bool allPassed = true;
    for (int g = 0; g < numGenf; g++)
    {
        for (int f = 0; f < numFreq; f++)
        {
            for (int p = 0; p < numPush; p++)
            {
                if (results[g][f][p].status != ENET_SOK)
                {
                    allPassed = false;
                }
            }
        }
    }

    if (allPassed)
    {
        DebugP_log("\r\nAll GENF and CPTS Hw Push test cases PASSED\r\n");
    }
    else
    {
        DebugP_log("\r\nSome GENF and CPTS Hw Push test cases FAILED\r\n");
    }
}

static int monitor_genf1_hwpush1_100hz_60sec(void)
{
    int status = ENET_SOK;
    int genfIdx = 1;
    int freq = 100;
    int cpts_push_idx = CPSW_CPTS_HWPUSH_FIRST;
    int monitorDurationSec = 60;
    // int expectedIntervalNs = 1000*1000*1000/freq;
    // uint64_t tolerance = 100;
    uint64_t lastTimestamp = 0;
    int totalSamples = freq * monitorDurationSec;

    DebugP_log("=== Monitor GENF1 + HwPush1 @ 100Hz for 60s START ===\r\n");

    uint64_t delayTimeNs = 5000000000;
    uint64_t startTime = gptpmasterclock_getts64() + delayTimeNs;
    startTime -= (startTime  % 100000000);

    int dest = getTsrDestForHwPushEvent(cpts_push_idx);
    if (dest == ENET_EFAIL)
    {
        DebugP_log("TSR dest failed\r\n");
        return ENET_EFAIL;
    }

    int src = getTsrSrcForGenfOut(genfIdx);
    if (src == ENET_EFAIL)
    {
        DebugP_log("TSR src failed\r\n");
        return ENET_EFAIL;
    }

    status = EnetAppUtils_setTimeSyncRouter(gEnetAppCfg.enetType, gEnetAppCfg.instId, src, dest);
    if (status == ENET_EFAIL)
    {
        DebugP_log("Set TSR failed\r\n");
        return status;
    }

    void* testArgs = (void*)ENET_MAGIC;

    if (SemaphoreP_constructBinary(&NotifyPushCallback, 0))
    {
        DebugP_log("Failed to initialize semaphore\r\n");
        return ENET_EFAIL;
    }

    status = registerCptsPushEvents(cpts_push_idx, cpts_hwpush_callback, testArgs);
    if (status == ENET_EFAIL)
    {
        DebugP_log("Register callback failed\r\n");
        return status;
    }

    status = configureGenf(genfIdx, freq, startTime);
    if (status == ENET_EFAIL)
    {
        DebugP_log("Configure GENF failed\r\n");
        return status;
    }

    for (int i = 0; i < totalSamples; i++)
    {
        status = SemaphoreP_pend(&NotifyPushCallback, 5000);
        if (status == SystemP_TIMEOUT)
        {
            DebugP_log("Timeout waiting for timestamp %d\r\n", i);
            status = ENET_EFAIL;
            break;
        }

        if (argsInCallback != testArgs)
        {
            DebugP_log("Callback arg mismatch at %d\r\n", i);
            status = ENET_EFAIL;
            break;
        }

        if (cptsHwPushNumberInCallback != cpts_push_idx)
        {
            DebugP_log("Push idx mismatch at %d\r\n", i);
            status = ENET_EFAIL;
            break;
        }

        argsInCallback = NULL;
        cptsHwPushNumberInCallback = CPSW_CPTS_HWPUSH_INVALID;

        uint64_t timestamp = getCptsHwPushEventTs(cpts_push_idx);
        if (timestamp == 0)
        {
            DebugP_log("Timestamp zero at %d\r\n", i);
            status = ENET_EFAIL;
            break;
        }

        DebugP_log("got idx: %d, Ts: %llu, diff: %lld\r\n", i, timestamp, (int64_t)timestamp - (int64_t)lastTimestamp);

        lastTimestamp = timestamp;
    }

    unRegisterCptsPushEvents(cpts_push_idx);
    stopGenf(genfIdx);
    EnetAppUtils_releaseTimeSyncRouter(gEnetAppCfg.enetType, gEnetAppCfg.instId, src, dest);
    SemaphoreP_destruct(&NotifyPushCallback);

    if (status == ENET_SOK)
    {
        DebugP_log("=== Monitor GENF1 + HwPush1 @ 100Hz for 60s PASSED (%d samples) ===\r\n", totalSamples);
    }
    else
    {
        DebugP_log("=== Monitor GENF1 + HwPush1 @ 100Hz for 60s FAILED ===\r\n");
    }

    return status;
}

static void EnetApp_enableTsSync(CpswCpts_OutputBitSel bitSelect)
{
    Enet_IoctlPrms prms;
    int32_t status;

    ENET_IOCTL_SET_IN_ARGS(&prms, &bitSelect);
    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, CPSW_CPTS_IOCTL_SELECT_TS_OUTPUT_BIT, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("%s: Failed to set TS SYNC OUT BIT : %d\r\n", gEnetAppCfg.name, status);
    }
    return;
}

static int run_cpts_sync_out_test(void)
{
    int status = ENET_SOK;
    int genfIdx = 1;
    int cpts_push_idx = CPSW_CPTS_HWPUSH_FIRST;
    CpswCpts_OutputBitSel testSyncBits = CPSW_CPTS_TS_OUTPUT_BIT_17;
    
    uint64_t lastTimestamp = 0;
    const int totalSamples = 20;
    int tolerance = 100;
    uint64_t timestamps[totalSamples];

    EnetApp_enableTsSync(testSyncBits);

    int dest = getTsrDestForHwPushEvent(cpts_push_idx);
    if (dest == ENET_EFAIL)
    {
        DebugP_log("TSR dest failed\r\n");
        return ENET_EFAIL;
    }

    int src = 18;
    if (src == ENET_EFAIL)
    {
        DebugP_log("TSR src failed\r\n");
        return ENET_EFAIL;
    }

    status = EnetAppUtils_setTimeSyncRouter(gEnetAppCfg.enetType, gEnetAppCfg.instId, src, dest);
    if (status == ENET_EFAIL)
    {
        DebugP_log("Set TSR failed\r\n");
        return status;
    }

    void* testArgs = (void*)ENET_MAGIC;

    if (SemaphoreP_constructBinary(&NotifyPushCallback, 0))
    {
        DebugP_log("Failed to initialize semaphore\r\n");
        return ENET_EFAIL;
    }

    status = registerCptsPushEvents(cpts_push_idx, cpts_hwpush_callback, testArgs);
    if (status == ENET_EFAIL)
    {
        DebugP_log("Register callback failed\r\n");
        return status;
    }

    for (int i = 0; i < totalSamples; i++)
    {
        status = SemaphoreP_pend(&NotifyPushCallback, 5000);
        if (status == SystemP_TIMEOUT)
        {
            DebugP_log("Timeout waiting for timestamp %d\r\n", i);
            status = ENET_EFAIL;
            break;
        }

        if (argsInCallback != testArgs)
        {
            DebugP_log("Callback arg mismatch at %d\r\n", i);
            status = ENET_EFAIL;
            break;
        }

        if (cptsHwPushNumberInCallback != cpts_push_idx)
        {
            DebugP_log("Push idx mismatch at %d\r\n", i);
            status = ENET_EFAIL;
            break;
        }

        argsInCallback = NULL;
        cptsHwPushNumberInCallback = CPSW_CPTS_HWPUSH_INVALID;

        timestamps[i] = getCptsHwPushEventTs(cpts_push_idx);
        if (timestamps[i] == 0)
        {
            DebugP_log("Timestamp zero at %d\r\n", i);
            status = ENET_EFAIL;
            break;
        }
    }
    for (int i = 0; i < totalSamples; i++)
    {
        uint64_t mod_17bit = timestamps[i]%(1<<17);
        if (mod_17bit > tolerance)
        {
            DebugP_log("Tolerance check has failed, mod value = %llu\r\n", mod_17bit);
            break;
        }
        DebugP_log("got idx: %d, Ts: %llu, diff: %lld, mod 2^17: %llu\r\n", i, timestamps[i], (int64_t)timestamps[i] - (int64_t)lastTimestamp, mod_17bit);
        lastTimestamp = timestamps[i];
    }

    unRegisterCptsPushEvents(cpts_push_idx);
    stopGenf(genfIdx);
    EnetAppUtils_releaseTimeSyncRouter(gEnetAppCfg.enetType, gEnetAppCfg.instId, src, dest);
    SemaphoreP_destruct(&NotifyPushCallback);

    if (status == ENET_SOK)
    {
        DebugP_log("SYNC OUT Test has Passed\r\n");
    }
    else
    {
        DebugP_log("SYNC OUT Test has Failed\r\n");
    }

    return status;
}


void run_timesync_tests(void)
{
    ClockP_sleep(5);
    /*
        This Application tests a bunch of test cases.
        Hardware tested: GENF, CPTS Hw PUSH Events, SYNC1_OUT

        1. Test Case 1: GENERATE GENF Output (0, 1 indices, Start, Stop)
        2. Test Case 2: GENF PPM Adjustment and Nudge Test
        3. Test Case 3: Hw Push Event Tests (Register, interrupt, verify timestamp, Unregister timestamp.)
        4. Test Case 4: Connect SYNC_OUT to Hw Push Event and check if the edges comes out at correct Timestamp value.
        5. Test Case 5: PPM correction propagation to GENF. (Run gPTP Master on PC, sweep the PPM and confirm that GENF also observes the PPM)

    */
    /* Setup the EXT_REFCLK1 to SYNC1_OUT */
    // EnetApp_setPinmux();

    run_genf_and_hw_push_tests();

    run_cpts_sync_out_test();

    DebugP_log("Start PTP Process\r\n");
    ClockP_sleep(20);

    monitor_genf1_hwpush1_100hz_60sec();

}

int getTsrDestForHwPushEvent(CpswCpts_HwPush pushNum)
{
    int status = ENET_EFAIL;
#if defined (SOC_AM275X)
    uint8_t tsr_hwpush_dest_array[] = {
        [CPSW_CPTS_HWPUSH_NORM(CPSW_CPTS_HWPUSH_1)] = 8,
        [CPSW_CPTS_HWPUSH_NORM(CPSW_CPTS_HWPUSH_2)] = 9,
        [CPSW_CPTS_HWPUSH_NORM(CPSW_CPTS_HWPUSH_3)] = 10,
        [CPSW_CPTS_HWPUSH_NORM(CPSW_CPTS_HWPUSH_4)] = 11,
        [CPSW_CPTS_HWPUSH_NORM(CPSW_CPTS_HWPUSH_5)] = 12,
        [CPSW_CPTS_HWPUSH_NORM(CPSW_CPTS_HWPUSH_6)] = 13,
        [CPSW_CPTS_HWPUSH_NORM(CPSW_CPTS_HWPUSH_7)] = 14,
        [CPSW_CPTS_HWPUSH_NORM(CPSW_CPTS_HWPUSH_8)] = 15,
    };
#else
#error "Unsupported SoC"
#endif
    if (CPSW_CPTS_HWPUSH_NORM(pushNum) < ENET_ARRAYSIZE(tsr_hwpush_dest_array))
    {
        status = tsr_hwpush_dest_array[CPSW_CPTS_HWPUSH_NORM(pushNum)];
    }
    return status;
}

int getTsrSrcForGenfOut(int genfIdx)
{
    int status = ENET_EFAIL;
#if defined (SOC_AM275X)
    uint8_t tsr_genf_src_array[] = {
        [0] = 16,
        [1] = 17,
    };
#else
#error "Unsupported SoC"
#endif
    if (genfIdx < ENET_ARRAYSIZE(tsr_genf_src_array))
    {
        status = tsr_genf_src_array[genfIdx];
    }
    return status;
}

volatile void* argsInCallback = NULL;
volatile CpswCpts_HwPush cptsHwPushNumberInCallback = CPSW_CPTS_HWPUSH_INVALID;
SemaphoreP_Object NotifyPushCallback;

void cpts_hwpush_callback(void *hwPushNotifyCbArg, CpswCpts_HwPush hwPushNum)
{
    argsInCallback             = hwPushNotifyCbArg;
    cptsHwPushNumberInCallback = hwPushNum;
    SemaphoreP_post(&NotifyPushCallback);
}

int test_hardwarePush_events(int genfIdx, int freq, int cpts_push_idx, HwPushResult *result)
{
    int status = ENET_EFAIL;
    int periodMs = 1000/freq;
    bool skipUnregisterCheck = false;
    if (periodMs == 1)
    {
        skipUnregisterCheck = true;
    }

    DebugP_log("=== test_hardwarePush_events START: genfIdx=%d, freq=%dHz, pushIdx=%d ===\r\n", genfIdx, freq, cpts_push_idx);

    uint64_t delayTimeNs = 100000000;
    uint64_t startTime = gptpmasterclock_getts64() + delayTimeNs;
    ClockP_usleep((0.9*delayTimeNs)/1000);

    status = configureGenf(genfIdx, freq, startTime);
    if (status == ENET_EFAIL)
    {
        result->pushIdx = cpts_push_idx;
        result->status = status;
        return status;
    }

    if (SemaphoreP_constructBinary(&NotifyPushCallback, 0))
    {
        result->pushIdx = cpts_push_idx;
        result->status = status;
        return status;
    }

    int dest = getTsrDestForHwPushEvent(cpts_push_idx);
    if (dest == ENET_EFAIL)
    {
        result->pushIdx = cpts_push_idx;
        result->status = status;
        return status;
    }

    int src = getTsrSrcForGenfOut(genfIdx);
    if (src == ENET_EFAIL)
    {
        result->pushIdx = cpts_push_idx;
        result->status = status;
        return status;
    }

    status = EnetAppUtils_setTimeSyncRouter(gEnetAppCfg.enetType, gEnetAppCfg.instId, src, dest);
    if (status == ENET_EFAIL)
    {
        result->pushIdx = cpts_push_idx;
        result->status = status;
        return status;
    }

    void* testArgs = (void*)ENET_MAGIC;

    /* Configure Hw Push event (Register Callback) */
    status = registerCptsPushEvents(cpts_push_idx, cpts_hwpush_callback, testArgs);
    if (status == ENET_EFAIL)
    {
        result->pushIdx = cpts_push_idx;
        result->status = status;
        return status;
    }
    uint64_t ts_collection[10];

    for (int i = 0; i < ENET_ARRAYSIZE(ts_collection); i++)
    {
        status = SemaphoreP_pend(&NotifyPushCallback, 5000);

        if (status == SystemP_TIMEOUT)
        {
            status = ENET_EFAIL;
            break;
        }

        /* Validate the Argument. */
        if (argsInCallback != testArgs)
        {
            status = ENET_EFAIL;
            break;
        }

        /* Validate the Hw push number. */
        if (cptsHwPushNumberInCallback != cpts_push_idx)
        {
            status = ENET_EFAIL;
            break;
        }

        argsInCallback = NULL;
        cptsHwPushNumberInCallback = CPSW_CPTS_HWPUSH_INVALID;

        uint64_t timestamp = getCptsHwPushEventTs(cpts_push_idx);
        if (timestamp == 0)
        {
            status = ENET_EFAIL;
            break;
        }

        ts_collection[i] = timestamp;
    }

    for (int i = 0; i < ENET_ARRAYSIZE(ts_collection); i++)
    {
        DebugP_log("%d:%llu\r\n", i, ts_collection[i]);
    }

    if (status != ENET_SOK)
    {
        result->pushIdx = cpts_push_idx;
        result->status = status;
        return status;
    }

    /* Unregister Hw push event callback */
    status = unRegisterCptsPushEvents(cpts_push_idx);
    if (status == ENET_EFAIL)
    {
        result->pushIdx = cpts_push_idx;
        result->status = status;
        return status;
    }

    if (skipUnregisterCheck == false)
    {
        /* Check if the callback is still firing. */
        int timeOutCounter = 0;
        for (int i = 0; i < 3; i++)
        {
            if (SemaphoreP_pend(&NotifyPushCallback, 2*periodMs) == SystemP_TIMEOUT)
            {
                timeOutCounter += 1;
            }
        }
        if (timeOutCounter == 0)
        {
            status = ENET_EFAIL;
            result->pushIdx = cpts_push_idx;
            result->status = status;
            return status;
        }
    }

    status = stopGenf(genfIdx);
    if (status == ENET_EFAIL)
    {
        result->pushIdx = cpts_push_idx;
        result->status = status;
        return status;
    }

    SemaphoreP_destruct(&NotifyPushCallback);

    /* Break TSR connections */
    status = EnetAppUtils_releaseTimeSyncRouter(gEnetAppCfg.enetType, gEnetAppCfg.instId, src, dest);
    if (status == ENET_EFAIL)
    {
        result->pushIdx = cpts_push_idx;
        result->status = status;
        return status;
    }

    /* Analyse the timestamps */
    uint64_t expectedPeriod = 1000*1000*1000/freq;
    uint64_t tolerance = 100;
    int numSamples = ENET_ARRAYSIZE(ts_collection);

    for (int i = 0; i < numSamples; i++)
    {
        uint64_t expected, actual, diff;
        const char *checkName;

        if (i == 0)
        {
            expected = startTime;
            actual = ts_collection[i];
            checkName = "First timestamp";
        }
        else
        {
            expected = expectedPeriod;
            actual = ts_collection[i] - ts_collection[i-1];
            checkName = "Interval";
        }

        diff = (actual > expected) ? (actual - expected) : (expected - actual);

        if (diff > tolerance)
        {
            DebugP_log("%s:%d: FAIL - %s mismatch. Expected: %llu, Got: %llu, Diff: %lld ns\r\n",
                       __func__, __LINE__, checkName, expected, actual, (long long)diff);
            status = ENET_EFAIL;
            break;
        }

        DebugP_log("%s:%d: PASS - %s match. Expected: %llu, Got: %llu, Diff: %lld ns\r\n",
                   __func__, __LINE__, checkName, expected, actual, (long long)diff);
    }

    DebugP_log("=== test_hardwarePush_events END: genfIdx=%d, freq=%dHz, pushIdx=%d, status=%s ===\r\n",
               genfIdx, freq, cpts_push_idx, (status == ENET_SOK) ? "PASS" : "FAIL");

    result->pushIdx = cpts_push_idx;
    result->status = status;
    return status;
}