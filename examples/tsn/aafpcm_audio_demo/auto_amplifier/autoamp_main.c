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

#include <stdio.h>
#include <drivers/i2c.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/mcasp.h>
#include <drivers/pinmux.h>
#include "../common_files/crf_hw_config.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/SemaphoreP.h>
#include "../common_files/crf_app.h"

uint8_t audioTaskStack[16*1024];
TaskP_Object audioTask;
uint32_t gClientId = 4u;
bool gAutoAmp_audioTxStarted = false;

int32_t Board_CdceConfig(void);
int32_t Board_codecConfig(void);
int32_t Board_MuxSelMcASP4(void);

int32_t TsMcasp_tsMcaspConfig(int instIdx);
int32_t AutoAmp_playbackMcaspConfig(void);

void aaf_audio_task(void *args);
void startAudioTask(void);

static void AutoAmp_setPinmux(void);

int32_t crfApp_init(crfApp_crfConfig* config);
static void demo_crfTsCb(void* args);
void crfApp_tick(uint64_t mediaClockEdge);

SemaphoreP_Object gCrfTickSem;

/* TODO: Rename this to AutoAmp_main() */
void EnetApp_AudioPlaybackDemoMain(void* args)
{
    int32_t status = SystemP_SUCCESS;

    SemaphoreP_constructBinary(&gCrfTickSem, 0);

    AutoAmp_setPinmux();

    status = Board_codecConfig();
    DebugP_assert(status == SystemP_SUCCESS);

    status = Board_CdceConfig();
    DebugP_assert(status == SystemP_SUCCESS);

    /* Setup McASP4 for Timestamping. */
    status = Board_MuxSelMcASP4();
    DebugP_assert(status == SystemP_SUCCESS);

    /* For Playback McASP index = 0, Ts McASP index = 1 */
    status = TsMcasp_tsMcaspConfig(1);
    DebugP_assert(status == SystemP_SUCCESS);

    status = AutoAmp_playbackMcaspConfig();
    DebugP_assert(status == SystemP_SUCCESS);

    crfApp_crfConfig crfConfig = {
        .baseFrequency = 48000,
        .streamID = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x01},
        .timestampingInterval = 160,
        .vlanID = 110,
        .vlanPCP = 2,
        .isListener = true,
    };

    crfApp_init(&crfConfig);

    crfHwConfig_AtlMuxConfig();

    crfHwConfig_setMediaClockAtSyncOut();

    int64_t edgeDiff = crfHwConfig_estimateEdgeDiff(TS_ROUTER_IN_ATL_BWS_SEL_2, TS_ROUTER_IN_ATL_BWS_SEL_1, 48000, 300);
    DebugP_log("Edge Diff = %lld\r\n", edgeDiff);

    crfHwConfig_attachTssToCpts(TS_ROUTER_IN_ATL_BWS_SEL_2, demo_crfTsCb);

    startAudioTask();

    uint64_t mediaClockEdge;
    while (1)
    {
        SemaphoreP_pend(&gCrfTickSem, SystemP_WAIT_FOREVER);
        mediaClockEdge = crfHwConfig_getMediaClockEdge();
        crfApp_tick(mediaClockEdge);

        if (gAutoAmp_audioTxStarted == false)
        {
            /* Check the stable Status. */
            if (crfApp_getFreqStableStatus() == true)
            {
                gAutoAmp_audioTxStarted = true;
                DebugP_log("%.6f: Media Clock Frequency is stable!!!!\r\n", ClockP_getTimeUsec()/(float)1000000);
            }
        }
    }
}

void startAudioTask(void)
{
    TaskP_Params params = {
        .name      =  "Audio Task",
        .priority  =  TaskP_PRIORITY_HIGHEST,
        .stack     = audioTaskStack,
        .stackSize = sizeof(audioTaskStack),
        .taskMain  = aaf_audio_task,
        .args      = NULL,
    };
    TaskP_construct(&audioTask, &params);
}

static void demo_crfTsCb(void* args)
{
    SemaphoreP_post(&gCrfTickSem);
}

static void AutoAmp_setPinmux(void)
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