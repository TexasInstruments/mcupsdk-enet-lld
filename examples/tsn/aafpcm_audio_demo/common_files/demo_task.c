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
#include <drivers/i2c.h>
#include <drivers/mcasp.h>
#include <drivers/pinmux.h>

#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/TaskP.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"

#include "crf_hw_config.h"
#include "crf_app.h"

#include "tsninit.h"
#include "enetapp_cpsw.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define AVB_AUDIO_TASK_PRIORITY (16)

/* Wait for Link to be stable (wait time in seconds)*10 */
#define LINK_STABILITY_WAIT_TIME (3*10)

/* ========================================================================== */
/*                           Global Variables                                 */
/* ========================================================================== */

extern EnetApp_Cfg gEnetAppCfg;

uint8_t audioTaskStack[TSN_TSK_STACK_SIZE] __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

TaskP_Object audioTask;

uint32_t gIPC_ClientId = 4u;

crfHwCfg_info gCrfHwInfo = {
#if AUTOPHY_MCR
    .clkSrc                = CRF_HW_CONFIG_CLKSRC_PHY,
#else
    .clkSrc                = CRF_HW_CONFIG_CLKSRC_CDCE,
#endif
    .mediaClkAtlSignal     = ATL_BWSX_MCASP1_AFSX_IN,
    .timestampingAtlSignal = ATL_BWSX_MCASP4_AFSX_IN,
    .mediaClkFreq          = 48000,
    .timestampingInterval  = 160,
};


crfApp_crfConfig gCrfConfig = {
    .baseFrequency = 48000,
    .streamID = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x01},
    .timestampingInterval = 160,
    .vlanID = 110,
    .vlanPCP = 2,
    .hwInfo = &gCrfHwInfo,
#if DEF_NODE_CENTRAL
    .isListener = false,
#else
    .isListener = true,
#endif
#if AUTOPHY_MCR
    .correctionFlag = CRFAPP_CORRECT_PHASE,
#else
    .correctionFlag = CRFAPP_CORRECT_FREQUENCY,
#endif
};

SemaphoreP_Object gCrfTickSem;

SemaphoreP_Object gStartAVBTxSem;

bool gAutoAmp_audioTxStarted = true;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t Board_codecConfig(void);

int32_t Board_MuxSelMcASP4(void);

void EnetApp_aafAudioTask(void *args);

void EnetApp_ipcNotifyCallback(uint16_t remoteCoreId, \
    uint16_t localClientId, uint32_t msgValue, void *args);

bool EnetApp_isGptpSyncStable(void);

static void EnetApp_startAudioTask(void);

static void EnetApp_setPinmux(void);

static int32_t EnetApp_TsMcaspConfig(const int instIdx);

#if !DEF_NODE_CENTRAL
int32_t AutoAmp_playbackMcaspConfig(void);
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetApp_AudioPlaybackDemoMain(void* args)
{
    int32_t status = SystemP_SUCCESS;

    EnetApp_setPinmux();

    SemaphoreP_constructBinary(&gCrfTickSem, 0);

    SemaphoreP_constructBinary(&gStartAVBTxSem, 0);

    /* Select McASP4 FS out on J29 Header. */
    status = Board_MuxSelMcASP4();
    DebugP_assert(status == SystemP_SUCCESS);

    crfHwConfig_init(&gCrfHwInfo);

    /* Configure Codec. */
    status = Board_codecConfig();
    DebugP_assert(status == SystemP_SUCCESS);

#if DEF_NODE_CENTRAL
    status = IpcNotify_registerClient(gIPC_ClientId, EnetApp_ipcNotifyCallback, NULL);
    DebugP_assert(status==SystemP_SUCCESS);
#endif

    /* Wait for gPTP to be Initialized. */
    while(gptpmasterclock_init(NULL))
    {
        DebugP_log("Waiting for tsn_gptpd to be initialzied...\n");
        ClockP_usleep(100000);
    }

    crfApp_init(&gCrfConfig);

    /* Initialize AVB Task. */
    EnetApp_startAudioTask();

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

    status = EnetApp_TsMcaspConfig(CONFIG_TS_MCASP);
    DebugP_assert(status == SystemP_SUCCESS);

#if DEF_NODE_CENTRAL
    /* Sync with C7x.  */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);
    /* Wait for C7x to Configure McASP */
    ClockP_sleep(1);
#else
    status = AutoAmp_playbackMcaspConfig();
    DebugP_assert(status == SystemP_SUCCESS);
#endif

#if GPTP_MASTER
    ClockP_sleep(3);
#else
    /* Wait for gPTP GM Stable. */
    bool syncStatus = false;
    do
    {
        ClockP_sleep(1);
        DebugP_log("Wait for gPTP Sync........\r\n");
        syncStatus = EnetApp_isGptpSyncStable();
    } while (syncStatus == false);
    DebugP_log("gPTP Synced........\r\n");
#endif

    crfHwConfig_setup(&gCrfHwInfo);

    DebugP_log("Calculated Edge Diff = %d\r\n", gCrfHwInfo.edgeDiff);

    SemaphoreP_post(&gStartAVBTxSem);

    while (1)
    {
        SemaphoreP_pend(&gCrfTickSem, SystemP_WAIT_FOREVER);

        uint64_t mediaClockEdge = crfHwConfig_getMediaClockEdge(&gCrfHwInfo);

        if (mediaClockEdge != 0)
        {
            crfApp_tick(mediaClockEdge);
        }
    }
}

/* Callback function to be called when CRF Timestamp is available. */
void CrfHwConfig_crfTsCb(void* args)
{
    SemaphoreP_post(&gCrfTickSem);
}

static void EnetApp_startAudioTask(void)
{
    TaskP_Params params = {
        .name      = "AVB Audio Task",
        .priority  = AVB_AUDIO_TASK_PRIORITY,
        .stack     = audioTaskStack,
        .stackSize = sizeof(audioTaskStack),
        .taskMain  = EnetApp_aafAudioTask,
        .args      = NULL,
    };
    TaskP_construct(&audioTask, &params);
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

static int32_t EnetApp_TsMcaspConfig(const int instIdx)
{
    int status = SystemP_SUCCESS;

    /* Open MCASP driver after enabling the HCLK */
    gMcaspHandle[instIdx] = MCASP_open(instIdx, &gMcaspOpenParams[instIdx]);
    if(NULL == gMcaspHandle[instIdx])
    {
        DebugP_logError("MCASP open failed for instance %d !!!\r\n", instIdx);
        DebugP_assert(false);
    }

    MCASP_startTransferTx(gMcaspHandle[instIdx]);

    return status;
}

void EnetApp_TxMcaspTxCb(MCASP_Handle handle,
                          MCASP_Transaction *transaction)
{
    /* Empty Tx Callback. */
}