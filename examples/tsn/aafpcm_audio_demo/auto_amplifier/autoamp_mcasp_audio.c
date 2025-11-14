/*
 *  Copyright (c) Texas Instruments Incorporated 2025
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

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/mcasp.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "../common_files/shm_cirbuf.h"
#include "../common_files/gpio_sm.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Audio buffer settings */
#define APP_MCASP_AUDIO_BUFF_COUNT   (2U)
#define APP_MCASP_AUDIO_BUFF_SIZE    (512U)

/* This is shared Memory Block size,
   should be an LCM Of McASP buffer size and AVTPDU Size. */
#define APP_MCASP_SHM_BLOCK_SIZE     (1536U)
#define APP_MCASP_SHM_NUM_BLOCKS     (1U)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static shm_handle AutoAmp_createSharedMemory(void* const address, const int blockSize, const int totalSize);

static bool AutoAmp_startTx(void);

void AutoAmp_SyncTimer(void);

/* ========================================================================== */
/*                           Global Variables                                 */
/* ========================================================================== */

extern MCASP_Config gMcaspConfig[];

uint32_t notifyCounter = 0;

/* Create buffers for transmit and Receive */
uint8_t gMcaspAudioBufferTx[APP_MCASP_AUDIO_BUFF_COUNT][APP_MCASP_AUDIO_BUFF_SIZE] __attribute__((aligned(256)));
uint8_t gMcaspAudioBufferRx[APP_MCASP_AUDIO_BUFF_COUNT][APP_MCASP_AUDIO_BUFF_SIZE] __attribute__((aligned(256)));

/* Create transaction objects for transmit and Receive */
MCASP_Transaction gMcaspAudioTxnTx[APP_MCASP_AUDIO_BUFF_COUNT] = {0};
MCASP_Transaction gMcaspAudioTxnRx[APP_MCASP_AUDIO_BUFF_COUNT] = {0};

/* Allocate Local Buffer to send/receive from AVB Tasks */
uint8_t C7X_TO_R5F_SHM_CIRC_BUFFER[APP_MCASP_SHM_BLOCK_SIZE*APP_MCASP_AUDIO_BUFF_COUNT*2]; __attribute__((aligned(1024)));
uint8_t R5F_TO_C7X_SHM_CIRC_BUFFER[APP_MCASP_SHM_BLOCK_SIZE*APP_MCASP_AUDIO_BUFF_COUNT*2]; __attribute__((aligned(1024)));

shm_handle gShmTxHandle;
shm_handle gShmRxHandle;

gpioStateMachine gMcaspRxSm = {
    .isEnabled = true,
    .isRxMode = true,
    .Periodicity = 3000,
    .pulseWidth = 30,
    .counter    = 0,
};

gpioStateMachine gMcaspTxSm = {
    .isEnabled = true,
    .isRxMode = false,
    .Periodicity = 3000,
    .pulseWidth = 30,
    .counter    = 0,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t AutoAmp_playbackMcaspConfig(void)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t i;
    MCASP_Handle mcaspHandle;

    /* Initialize Audio RX Gpio for Latency Measurement, Uses same pin. Hence enable one at a time. */
    gpio_sm_init(&gMcaspRxSm, CONFIG_GPIO_RX_MCASP_BASE_ADDR, CONFIG_GPIO_RX_MCASP_PIN, CONFIG_GPIO_RX_MCASP_DIR);

    /* Initialize Audio TX Gpio for Latency Measurement, Uses same pin. Hence enable one at a time. */
    gpio_sm_init(&gMcaspTxSm, CONFIG_GPIO_TX_MCASP_BASE_ADDR, CONFIG_GPIO_TX_MCASP_PIN, CONFIG_GPIO_TX_MCASP_DIR);

    /* Playback Instance Index in Driver. */
    int instIdx = CONFIG_MCASP0;

    /* Create Tx Shared Memory. */
    gShmTxHandle = AutoAmp_createSharedMemory((void*)C7X_TO_R5F_SHM_CIRC_BUFFER, APP_MCASP_SHM_BLOCK_SIZE, \
                             APP_MCASP_SHM_BLOCK_SIZE*(APP_MCASP_SHM_NUM_BLOCKS+1));
    /* Create Rx Shared Memory. */
    gShmRxHandle = AutoAmp_createSharedMemory((void*)R5F_TO_C7X_SHM_CIRC_BUFFER, APP_MCASP_SHM_BLOCK_SIZE, \
                             APP_MCASP_SHM_BLOCK_SIZE*(APP_MCASP_SHM_NUM_BLOCKS+1));

    /* Open MCASP driver after enabling the HCLK */
    gMcaspHandle[instIdx] = MCASP_open(instIdx, &gMcaspOpenParams[instIdx]);
    if(NULL == gMcaspHandle[instIdx])
    {
        DebugP_logError("MCASP open failed for instance 0 !!!\r\n");
        DebugP_assert(false);
    }

    mcaspHandle = MCASP_getHandle(instIdx);

    /* Prepare and submit audio transaction transmit objects */
    for (i = 0U; i < APP_MCASP_AUDIO_BUFF_COUNT; i++)
    {
        gMcaspAudioTxnTx[i].buf     = (void*)&gMcaspAudioBufferTx[i][0];
        gMcaspAudioTxnTx[i].count   = APP_MCASP_AUDIO_BUFF_SIZE/4;
        gMcaspAudioTxnTx[i].timeout = 0xFFFFFF;
        gMcaspAudioTxnTx[i].args    = (void*)gShmRxHandle;
        MCASP_submitTx(mcaspHandle, &gMcaspAudioTxnTx[i]);
    }

    /* Prepare and submit audio transaction receive objects */
    for (i = 0U; i < APP_MCASP_AUDIO_BUFF_COUNT; i++)
    {
        gMcaspAudioTxnRx[i].buf     = (void*)&gMcaspAudioBufferRx[i][0];
        gMcaspAudioTxnRx[i].count   = APP_MCASP_AUDIO_BUFF_SIZE/4;
        gMcaspAudioTxnRx[i].timeout = 0xFFFFFF;
        gMcaspAudioTxnRx[i].args    = (void*)gShmTxHandle;
        MCASP_submitRx(mcaspHandle, &gMcaspAudioTxnRx[i]);
    }

    /* Trigger McASP receive operation */
    status = MCASP_startTransferRx(mcaspHandle);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Trigger McASP transmit operation */
    status = MCASP_startTransferTx(mcaspHandle);
    DebugP_assert(status == SystemP_SUCCESS);

    return status;
}

static shm_handle AutoAmp_createSharedMemory(void* const address, const int blockSize, const int totalSize)
{
    shm_handle handle;
    const uint32_t shmOvrHd = shm_metadata_overhead();
    /* floor the totalSize to be a multiple of blocksize, exclude the overhead size. */
    const uint32_t rxShmBufSize = ((totalSize-shmOvrHd)/blockSize)*blockSize + shmOvrHd;
    memset(address, 0, rxShmBufSize);
    handle = shm_create(shm_core_r5f, (uint32_t)address, rxShmBufSize);
    return handle;
}

void AutoAmp_playbackMcaspTxCb(MCASP_Handle handle, MCASP_Transaction *transaction)
{
    shm_handle hShm = (shm_handle)transaction->args;
    DebugP_assert(hShm != NULL);

    uint16_t reqSize = APP_MCASP_AUDIO_BUFF_SIZE;
    shm_read(shm_core_r5f, hShm, transaction->buf, &reqSize);
    CacheP_wb(transaction->buf, APP_MCASP_AUDIO_BUFF_SIZE, CacheP_TYPE_ALL);
    gpio_sm_spin(&gMcaspTxSm, transaction->buf, 16);
    MCASP_submitTx(handle, transaction);
}

void AutoAmp_playbackMcaspRxCb(MCASP_Handle handle, MCASP_Transaction *transaction)
{
    shm_handle hShm = (shm_handle)transaction->args;
    DebugP_assert(hShm != NULL);

    /* Write to Shm only if Start is indicated. */
    if (AutoAmp_startTx() == true)
    {
        CacheP_inv(transaction->buf, APP_MCASP_AUDIO_BUFF_SIZE, CacheP_TYPE_ALL);
        gpio_sm_spin(&gMcaspRxSm, transaction->buf, 16);
        shm_write(shm_core_r5f, hShm, transaction->buf, APP_MCASP_AUDIO_BUFF_SIZE);
    }
    MCASP_submitRx(handle, transaction);

    if (notifyCounter == 0)
    {
        notifyCounter = 2;
        /* Notify R5 Core. to start timer. */
        AutoAmp_SyncTimer();
    }
    else
    {
        notifyCounter -= 1;
    }

}

static bool AutoAmp_startTx(void)
{
    extern bool gAutoAmp_audioTxStarted;
    return gAutoAmp_audioTxStarted;
}