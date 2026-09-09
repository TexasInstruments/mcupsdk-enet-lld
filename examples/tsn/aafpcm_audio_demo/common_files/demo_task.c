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

/* ========================================================================== */
/*                             Include Files                                  */
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

#include "crf_task.h"

#include "tsninit.h"
#include "enetapp_cpsw.h"
#include "clocksync.h"
#include "avb_if.h"
#include "mcasp_audio_cfg.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Central node: two SHM rings shared with the C7x, one per direction, 100 KB apart.
 * Auto amp has no C7x peer, so it just uses its own buffers (extern below). */
#if DEF_NODE_CENTRAL
#define R5F_AVB_SHM_START_ADDRESS    (0x72400000U)

#define AVB_C7X_TO_R5_SHM_ADDR       (R5F_AVB_SHM_START_ADDRESS + 0U)
#define AVB_R5_TO_C7X_SHM_ADDR       (R5F_AVB_SHM_START_ADDRESS + 100 * 1024U)
#else
extern uint8_t AutoAmp_c7xToR5fShmBuffer[];
extern uint8_t AutoAmp_r5fToC7xShmBuffer[];
#endif

/* APP_MCASP_SHM_BLOCK_SIZE comes from mcasp_audio_cfg.h.
   PCM frames per SHM ring, in units of APP_MCASP_SAMPLE_FRAME_SIZE. */
#define AAF_AVB_RING_CAPACITY_FRAMES (APP_MCASP_SHM_SAMPLE_FRAMES)

/* Added to the media-clock timestamp derived from the shm index before
   handing it to aaf_avtpc_talker_write() as the AVTP presentation time. */
#define PRESENTATION_TIME_OFFSET_NS  (0.8*UB_MSEC_NS)

/* Wait for Link to be stable (wait time in seconds)*10 */
#define LINK_STABILITY_WAIT_TIME (3 * 10)

/* Timer7's PWM output is routed to CPTS HW push 4 (see clocksync.c). */
#define CLOCKSYNC_APP_TIMER_INSTANCE   (7)
#define CLOCKSYNC_APP_PUSH_NUM         (CPSW_CPTS_HWPUSH_4)

/* Lower number = higher priority (see crf_task.h's CRF_TASK_PRIORITY_DEFAULT
 * comment). Tick task sits between CrfTask (8) and the AVB audio task (16):
 * timing-relevant, but less critical than gPTP tick processing. Query task
 * is purely diagnostic, lowest priority of anything in this app. */
#define CLOCK_SYNC_TICK_TASK_PRIORITY  (10U)
#define CLOCK_SYNC_QUERY_TASK_PRIORITY (20U)

#define CLOCK_SYNC_APP_TASK_STACK_SIZE (4U * 1024U)

/* Talker task stack/priority are app-owned. 16 stays numerically above CrfTask (8). */
#define AVB_TALKER_TASK_PRIORITY       (16)

/* ========================================================================== */
/*                           Global Variables                                 */
/* ========================================================================== */

extern EnetApp_Cfg gEnetAppCfg;

uint32_t EnetApp_ipcClientId = 4u;


static SemaphoreP_Object gClockSyncTickSem;
static volatile uint32_t gClockSyncTickCount = 0U;

static uint8_t clockSyncTickTaskStack[CLOCK_SYNC_APP_TASK_STACK_SIZE] \
    __attribute__((aligned(TSN_TSK_STACK_ALIGN)));

static TaskP_Object clockSyncTickTask;

static uint8_t EnetApp_crfTaskStack[TSN_TSK_STACK_SIZE] \
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

static uint8_t EnetApp_avbTalkerTaskStack[TSN_TSK_STACK_SIZE] \
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

static const AvbIf_TalkerTaskConfig gTalkerTaskCfg = {
    .priority  = AVB_TALKER_TASK_PRIORITY,
    .stack     = EnetApp_avbTalkerTaskStack,
    .stackSize = sizeof(EnetApp_avbTalkerTaskStack),
};

static AvbIf_TalkerHandle   talkerHandle;
static AvbIf_ListenerHandle listenerHandle;

/* CDCE clock source only - clock_rec/CrfTask (see CLOCK_REC_DESIGN.md \S13).
 * PHY (AUTOPHY_MCR) boards still use crf_app.c/crf_hw_config.c above. */
CrfTask_Params EnetApp_crfTaskParams = {
#if DEF_NODE_CENTRAL
    .role                   = CLOCK_REC_ROLE_TALKER,
#else
    .role                   = CLOCK_REC_ROLE_LISTENER,
#endif
    .clkSrc                 = CLOCK_REC_SOURCE_CDCE,
    .correctionFlags        = CLOCK_REC_CORRECT_FREQUENCY | CLOCK_REC_CORRECT_PHASE,
    .parentClockFrequency   = 48000,
    .nominalFrequency       = 48000,
    /* One timestamp event per 160 media-clock edges, i.e. 300 per second at 48 kHz. */
    .tsInterval             = 160,
    /* Audio McASP frame sync is the 48 kHz media clock; the TS McASP (McASP4) frame
     * sync is the slower timestamping signal derived from it. */
    .audioMcaspBaseAddress  = CSL_MCASP1_CFG_BASE,
    .audioMcaspDir          = CLOCK_HW_MCASP_DIR_TX,
    .tsMcaspBaseAddress     = CSL_MCASP4_CFG_BASE,
    .tsMcaspDir             = CLOCK_HW_MCASP_DIR_TX,
    /* Media clock arrives on HW push 1, the timestamping signal on HW push 2. */
    .mcHwPush               = CPSW_CPTS_HWPUSH_1,
    .tsHwPush               = CPSW_CPTS_HWPUSH_2,
    .netdev                 = "tilld0",
    .streamId               = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x00},
    .destMcast              = "91:E0:F0:00:FE:00",
    .joinMcast              = "91:E0:F0:00:FE:00",
    .vlanId                 = 110,
    .vlanPcp                = 2,
    .pull                   = AVBTP_CRF_PULL_MULT_1,
    /* aaf_avb_task.c dispatches CRF-subtype packets into
     * CrfTask_rxCallback() themselves from their shared AVTP-PCM listener -
     * CrfTask must not also claim an independent rx-direct socket. */
    .skipRxRegistration     = true,
    .taskStack              = EnetApp_crfTaskStack,
    .taskStackSize          = sizeof(EnetApp_crfTaskStack)
};

/* The two nodes mirror each other: what one talks (streamID last byte 01 or 02) the other
    * listens to. Each SHM ring needs its own HW spinlock number, hence lockNumber 0..3. */
#if DEF_NODE_CENTRAL
static AvbIf_TalkerConfig gTalkerCfg = {
    .netdev                = "tilld0",
    .streamID              = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x01},
    .destMacAddr           = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x01},
    .vlanID                = 110,
    .priority              = 3,
    /* 125 us at 48 kHz is 6 frames per AVTP packet. */
    .timeInterval_us       = 125,
    .pcminfo               = {.format = AVBTP_AAF_FORMAT_INT_32BIT,
                                .channels = 8, .bit_depth = 32, .srate = 48000},
    .shmAddr               = (void *)AVB_C7X_TO_R5_SHM_ADDR,
    .capacityFrames        = AAF_AVB_RING_CAPACITY_FRAMES,
    .lockNumber            = 2U,
    .presentationOffset_ns = PRESENTATION_TIME_OFFSET_NS,
    .spinlockBaseAddr      = CSL_SPINLOCK0_BASE,
};
static AvbIf_ListenerConfig gListenerCfg = {
    .streamID          = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x02},
    .pcminfo           = {.format = AVBTP_AAF_FORMAT_INT_32BIT,
                           .channels = 8, .bit_depth = 32, .srate = 48000},
    .timeInterval_us   = 125,
    .shmAddr           = (void *)AVB_R5_TO_C7X_SHM_ADDR,
    .capacityFrames    = AAF_AVB_RING_CAPACITY_FRAMES,
    .lockNumber        = 3U,
    .spinlockBaseAddr  = CSL_SPINLOCK0_BASE,
};
#else
static AvbIf_TalkerConfig gTalkerCfg = {
    .netdev                = "tilld0",
    .streamID              = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x02},
    .destMacAddr           = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x01},
    .vlanID                = 110,
    .priority              = 3,
    .timeInterval_us       = 125,
    .pcminfo               = {.format = AVBTP_AAF_FORMAT_INT_32BIT,
                                .channels = 8, .bit_depth = 32, .srate = 48000},
    .shmAddr               = (void *)AutoAmp_c7xToR5fShmBuffer,
    .capacityFrames        = AAF_AVB_RING_CAPACITY_FRAMES,
    .lockNumber            = 1U,
    .presentationOffset_ns = PRESENTATION_TIME_OFFSET_NS,
    .spinlockBaseAddr      = CSL_SPINLOCK0_BASE,
};
static AvbIf_ListenerConfig gListenerCfg = {
    .streamID         = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x01},
    .pcminfo          = {.format = AVBTP_AAF_FORMAT_INT_32BIT,
                             .channels = 8, .bit_depth = 32, .srate = 48000},
    .timeInterval_us  = 125,
    .shmAddr          = (void *)AutoAmp_r5fToC7xShmBuffer,
    .capacityFrames   = AAF_AVB_RING_CAPACITY_FRAMES,
    .spinlockBaseAddr = CSL_SPINLOCK0_BASE,
    .lockNumber       = 0U,
};
#endif
static const AvbIf_ListenerNetConfig gListenerNetCfg = {
    .netdev          = "tilld0",
    .destMacAddr     = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x01},
    .vlanID          = 110,
    .priority        = 3,
    .timeInterval_us = 125,
};

SemaphoreP_Object EnetApp_clockStartedSem;

bool AutoAmp_audioTxStarted = true;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

extern int32_t Board_codecConfig(void);

extern int32_t Board_MuxSelMcASP4(void);

extern void AutoAmp_syncTimer(void);

/* C7x pings the R5F from its McASP Rx callback; that ping stands in for the talker timer tick. */
void EnetApp_ipcNotifyCallback(uint16_t remoteCoreId, \
                                uint16_t localClientId, uint32_t msgValue, void *args)
{
    AutoAmp_syncTimer();
}

extern bool EnetApp_isGptpSyncStable(void);

static void EnetApp_setPinmux(void);

void ClockSyncApp_init(void);

#if !DEF_NODE_CENTRAL
static int32_t EnetApp_TsMcaspConfig(const int32_t instIdx);
extern int32_t AutoAmp_playbackMcaspConfig(void);
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetApp_AudioPlaybackDemoMain(void *args)
{
    int32_t status = SystemP_SUCCESS;

    EnetApp_setPinmux();

    SemaphoreP_constructBinary(&EnetApp_clockStartedSem, 0);

    /* Select McASP4 FS out on J29 Header. */
    status = Board_MuxSelMcASP4();
    DebugP_assert(status == SystemP_SUCCESS);

    /* Configure Codec. */
    status = Board_codecConfig();
    DebugP_assert(status == SystemP_SUCCESS);

#if DEF_NODE_CENTRAL
    status = IpcNotify_registerClient(EnetApp_ipcClientId, EnetApp_ipcNotifyCallback, NULL);
    DebugP_assert(status == SystemP_SUCCESS);
#endif

    /* Wait for gPTP to be Initialized. */
    while(gptpmasterclock_init(NULL))
    {
        DebugP_log("Waiting for tsn_gptpd to be initialzied...\n");
        ClockP_usleep(100000);
    }

    /* Initialize AVB Task: avb_if.c owns the talker task/tick and the
       shared listener socket from here on - register the TX (talker) and
       RX (listener) PCM streams with it (see avbTickTimer_callback()/
       AutoAmp_syncTimer() below for the tick trigger). */
    status = AvbIf_initTalkerTask(&gTalkerTaskCfg);
    DebugP_assert(status == SystemP_SUCCESS);

    status = AvbIf_registerTalker(&gTalkerCfg, &talkerHandle);
    DebugP_assert(status == SystemP_SUCCESS);

    status = AvbIf_initListenerTask(&gListenerNetCfg);
    DebugP_assert(status == SystemP_SUCCESS);

    status = AvbIf_registerListener(&gListenerCfg, &listenerHandle);
    DebugP_assert(status == SystemP_SUCCESS);

#if  defined(AUTOPHY_MCR)
    /* Stable Check Time = 3 seconds.*/
    uint32_t waitTime = LINK_STABILITY_WAIT_TIME;
    do
    {
        bool linkStatus = EnetAppUtils_isPortLinkUp(gEnetAppCfg.hEnet, \
                                                     gEnetAppCfg.coreId, ENET_MAC_PORT_1);
        if (linkStatus)
        {
            waitTime -= 1;
        }
        else
        {
            waitTime = LINK_STABILITY_WAIT_TIME;
        }
        ClockP_usleep(100000);
    } while (waitTime > 0);

    DebugP_log("Link is Stable...\r\n");
#endif

    /* CrfTask waits for gPTP/link readiness itself, registers with the
     * AVTP/CRF stack, runs ClockRec_init()'s hardware bootstrap, then
     * drives the steady-state recovery loop for the life of the
     * application - see CLOCK_REC_DESIGN.md \S13. */
    status = CrfTask_create(&EnetApp_crfTaskParams);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Released by ClockRec_clockStartedHook() once the clock recovery bootstrap is done. */
    SemaphoreP_pend(&EnetApp_clockStartedSem, SystemP_WAIT_FOREVER);
    DebugP_log("Clock Started...\r\n");

    ClockSyncApp_init();

#if DEF_NODE_CENTRAL
    /* Sync with C7x.  
     * The C7x Core waits for this point because the CDCE clock starts only after 
     * this point, so that McASP Configuration can be started. */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

#else
    /* Configure the McASPs for Playback, Record and also start the McASP 
     * that starts the McASP used to generate 300Hz Timestamping signal */
    status = AutoAmp_playbackMcaspConfig();
    DebugP_assert(status == SystemP_SUCCESS);
    status = EnetApp_TsMcaspConfig(CONFIG_TS_MCASP);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Interrupts off so the TS and audio McASPs start back to back and their frame
     * syncs stay as aligned as software can get them. */
    uintptr_t key = HwiP_disable();
    // Todo: Enable this when API is available 
    // status = MCASP_TxSyncStart(mcaspHandle, gMcaspHandle[CONFIG_TS_MCASP],
    //                             true, true, true, false);
    /* 
     * This Starts Timestamping McASP's FSYNC, which is fed to 
     * CPTS for timestamping, with 48KHz media clock this frequency is 
     * set to 300Hz to get 160 division factor.
    */
    MCASP_startTransferTx(gMcaspHandle[CONFIG_TS_MCASP]);
    /* 
     * This Starts Audio McASP's Operations and FSYNC(Media Clock)
    */
    MCASP_startTransferTx(gMcaspHandle[CONFIG_MCASP0]);
    MCASP_startTransferRx(gMcaspHandle[CONFIG_MCASP0]);
    HwiP_restore(key);
#endif

    while (1)
    {
        ClockP_sleep(1);
    }
}

/* --- ClockRec_* audio-facing hooks (CLOCK_REC_DESIGN.md \S13.3) ---
 * The transport-facing hooks are implemented by crf_task.c; these two
 * belong to the audio pipeline, owned by this file. */

int32_t ClockRec_clockStartedHook(void)
{
    /* Hardware bootstrap (edge-diff calibration) complete - gate AVB tx
     * start on this, matching the old crfHwConfig_setup completion point. */
    SemaphoreP_post(&EnetApp_clockStartedSem);

    return 0;
}

void ClockRec_tsAvailableHook(void)
{
    /* Nothing to do per tick; timestamps are pulled on demand through
     * ClockRec_getMediaClockTimestamp() when a presentation time is needed. */
}

/* Timer callback registed in syscfg. Forwards to avb_if.c's own tick
 * callback, stopping the timer once the startup transient (7 ticks) has
 * passed - avb_if.c has no notion of this demo-specific shutoff. */
void avbTickTimer_callback(void *args)
{

    AvbIf_talkerTickCallback(args);
}

void AutoAmp_syncTimer(void)
{
    avbTickTimer_callback(NULL);
}

static void EnetApp_setPinmux(void)
{
    /* Fix I2C Pinmux Config. */
    #if defined (SOC_AM275X)
    Pinmux_PerCfg_t gPinMuxMainDomainCfg[] =
    {
        {
            PIN_GPIO1_72,
            ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DIRECTION  )
        },
        {
            PIN_EXT_REFCLK1,
            ( PIN_MODE(1) )
        },
        {PINMUX_END, 0U}
    };

    Pinmux_config(gPinMuxMainDomainCfg, PINMUX_DOMAIN_ID_MAIN);
    #endif
}

#if !DEF_NODE_CENTRAL
static int32_t EnetApp_TsMcaspConfig(const int32_t instIdx)
{
    int32_t status = SystemP_SUCCESS;

    /* Open MCASP driver after enabling the HCLK */
    gMcaspHandle[instIdx] = MCASP_open(instIdx, &gMcaspOpenParams[instIdx]);
    if(NULL == gMcaspHandle[instIdx])
    {
        DebugP_logError("MCASP open failed for instance %d !!!\r\n", instIdx);
        DebugP_assert(false);
    }


    return status;
}

void EnetApp_TxMcaspTxCb(MCASP_Handle handle,
                          MCASP_Transaction *transaction)
{
    /* Empty Tx Callback. This McASP only runs for its frame sync, there is no data to refill. */
}
#endif

int ClockRec_needsRestartHook(void)
{
    /* Advisory heuristic - this file's policy is to always act; if this
     * proves too eager in practice, debouncing belongs inside
     * clock_rec.c's heuristic, not here. */
    UB_LOG(UBL_INFO, "%s: restart heuristic fired, restarting\r\n", __func__);
    (void)ClockRec_restart();

    return 0;
}

/* Overrides clocksync.c's weak default. Minimal ISR shim - all real work
 * (CPTS lookup, m/c filter update) happens in ClockSyncApp_tickTask(),
 * matching how clock_hw.c's own Tss push callback in this same dependency
 * chain defers all real work out of ISR context. */
void ClockSync_HwPushCallbackHook(void)
{
    SemaphoreP_post(&gClockSyncTickSem);
}

static void ClockSyncApp_tickTask(void *args)
{
    while (1)
    {
        SemaphoreP_pend(&gClockSyncTickSem, SystemP_WAIT_FOREVER);
        gClockSyncTickCount++;
        (void)ClockSync_tick();
    }
}

void ClockSyncApp_init(void)
{
    int32_t status;
    TaskP_Params taskParams;

    status = ClockSync_init(gEnetAppCfg.hEnet, gEnetAppCfg.coreId,
                             CLOCKSYNC_APP_TIMER_INSTANCE, CLOCKSYNC_APP_PUSH_NUM);
    DebugP_assert(status == SystemP_SUCCESS);

    SemaphoreP_constructBinary(&gClockSyncTickSem, 0);

    TaskP_Params_init(&taskParams);
    taskParams.name      = "ClockSync Tick Task";
    taskParams.priority  = CLOCK_SYNC_TICK_TASK_PRIORITY;
    taskParams.stack     = clockSyncTickTaskStack;
    taskParams.stackSize = sizeof(clockSyncTickTaskStack);
    taskParams.taskMain  = ClockSyncApp_tickTask;
    taskParams.args      = NULL;
    TaskP_construct(&clockSyncTickTask, &taskParams);
}

/* avb_if needs gPTP time often, so answer from the CPTS-synced timer (clocksync.c)
 * instead of doing a full gPTP clock read each time. */
int64_t AvbIf_getGptpProxyTime(void)
{
    return ClockSync_getCurrentTimeNs();
}
