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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <drivers/mcasp.h>
#include <board/ioexp/ioexp_tca6424.h>
#include <drivers/pinmux.h>
#include <drivers/gpio.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "../../common_files/shm_cirbuf.h"
#include "../../common_files/gpio_sm.h"
#include "string.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define C7X_AVB_SHM_START_ADDRESS    (0x80400000U)

#define AVB_C7X_TO_R5_SHM_ADDR       (C7X_AVB_SHM_START_ADDRESS+0U)
#define AVB_R5_TO_C7X_SHM_ADDR       (C7X_AVB_SHM_START_ADDRESS+100*1024U)

#define APP_MCASP_AUDIO_BUFF_COUNT   (2U)
#define APP_MCASP_AUDIO_BUFF_SIZE    (512U)

/* This is shared Memory Block size,
   should be an LCM Of McASP buffer size and AVTPDU Size. */
#define APP_MCASP_SHM_BLOCK_SIZE     (1536U)
#define APP_MCASP_SHM_NUM_BLOCKS     (1U)

#define APP_MCASP_BIT_DEPTH          (4U)

/* ========================================================================== */
/*                           Global Variables                                 */
/* ========================================================================== */

/* Stop Demo Flag. */
volatile bool stopDemo = false;

/* Client ID for IPC Communication. */
uint32_t gClientId = 4u;

/* Create buffers for transmit and Receive */
uint8_t gMcaspAudioBufferTx[APP_MCASP_AUDIO_BUFF_COUNT][APP_MCASP_AUDIO_BUFF_SIZE] __attribute__((aligned(256)));
uint8_t gMcaspAudioBufferRx[APP_MCASP_AUDIO_BUFF_COUNT][APP_MCASP_AUDIO_BUFF_SIZE] __attribute__((aligned(256)));

/* Create transaction objects for transmit and Receive */
MCASP_Transaction   gMcaspAudioTxnTx[APP_MCASP_AUDIO_BUFF_COUNT] = {0};
MCASP_Transaction   gMcaspAudioTxnRx[APP_MCASP_AUDIO_BUFF_COUNT] = {0};

/*
 * Toggle GPIO in this mechanism to observe the McASP to McASP Latency
 * Callback is invoked at 3KHz frequency.
 * Use a counter, increment from 0 to 2999 then reset back to Zero.
 * Start with Counter 0, Make GPIO HIGH.
 * At Counter 30, Make GPIO LOW -> Pulse Width Approx 10ms
 * After Counter becomes 2999, reset it to Zero and Make GPIO High, Repeat this cycle.
*/
gpioStateMachine gMcaspRxSm = {
    .isEnabled = true,
    .isRxMode = true, /* Modifies Channel 3 data peridically with a magic number*/
    .Periodicity = 3000,
    .pulseWidth = 30,
    .counter    = 0,
};

gpioStateMachine gMcaspTxSm = {
    .isEnabled = true,
    .isRxMode = false, /* Monitors Channel 3. */
    .Periodicity = 3000,
    .pulseWidth = 30,
    .counter    = 0,
};

uint32_t notifyCounter = 0;

/* ========================================================================== */
/*                        Extern Function Declaration                         */
/* ========================================================================== */
static void ipc_notify_msg_handler_main_core(uint16_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, void *args);

static shm_handle mcasp_createSharedMemory(void* const address, const int blockSize, const int totalSize);

void mcasp_playback_main(void *args)
{
    int32_t     status = SystemP_SUCCESS;
    uint32_t    i;
    MCASP_Handle    mcaspHandle;

    status = IpcNotify_registerClient(gClientId, ipc_notify_msg_handler_main_core, NULL);
    DebugP_assert(status==SystemP_SUCCESS);

    /* Initialize Audio RX Gpio for Latency Measurement, Uses same pin. Hence enable one at a time. */
    gpio_sm_init(&gMcaspRxSm, CONFIG_GPIO_RX_MCASP_BASE_ADDR, CONFIG_GPIO_RX_MCASP_PIN, CONFIG_GPIO_RX_MCASP_DIR);

    /* Initialize Audio TX Gpio for Latency Measurement, Uses same pin. Hence enable one at a time. */
    gpio_sm_init(&gMcaspTxSm, CONFIG_GPIO_TX_MCASP_BASE_ADDR, CONFIG_GPIO_TX_MCASP_PIN, CONFIG_GPIO_TX_MCASP_DIR);

    /* Create Tx Shared Memory. */
    shm_handle shmTxHandle = mcasp_createSharedMemory((void*)AVB_C7X_TO_R5_SHM_ADDR, APP_MCASP_SHM_BLOCK_SIZE, \
                             APP_MCASP_SHM_BLOCK_SIZE*(APP_MCASP_SHM_NUM_BLOCKS+1));
    /* Create Tx Shared Memory. */
    shm_handle shmRxHandle = mcasp_createSharedMemory((void*)AVB_R5_TO_C7X_SHM_ADDR, APP_MCASP_SHM_BLOCK_SIZE, \
                             APP_MCASP_SHM_BLOCK_SIZE*(APP_MCASP_SHM_NUM_BLOCKS+1));

    /* Sync with R5F Core.  */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    /* Open MCASP driver after enabling the HCLK */
    gMcaspHandle[0] = MCASP_open(0, &gMcaspOpenParams[0]);
    if(NULL == gMcaspHandle[0])
    {
        DebugP_assert(false);
    }

    mcaspHandle = MCASP_getHandle(CONFIG_MCASP0);

    /* Prepare and submit audio transaction transmit objects */
    for (i = 0U; i < APP_MCASP_AUDIO_BUFF_COUNT; i++)
    {
        gMcaspAudioTxnTx[i].buf     = (void*) &gMcaspAudioBufferTx[i][0];
        gMcaspAudioTxnTx[i].count   = APP_MCASP_AUDIO_BUFF_SIZE/4;
        gMcaspAudioTxnTx[i].timeout = 0xFFFFFF;
        gMcaspAudioTxnTx[i].args    = (void*)shmRxHandle;
        MCASP_submitTx(mcaspHandle, &gMcaspAudioTxnTx[i]);
    }

    /* Prepare and submit audio transaction receive objects */
    for (i = 0U; i < APP_MCASP_AUDIO_BUFF_COUNT; i++)
    {
        gMcaspAudioTxnRx[i].buf     = (void*)&gMcaspAudioBufferRx[i][0];
        gMcaspAudioTxnRx[i].count   = APP_MCASP_AUDIO_BUFF_SIZE/4;
        gMcaspAudioTxnRx[i].timeout = 0xFFFFFF;
        gMcaspAudioTxnRx[i].args    = (void*)shmTxHandle;
        MCASP_submitRx(mcaspHandle, &gMcaspAudioTxnRx[i]);
    }

    /* Trigger McASP receive operation */
    status = MCASP_startTransferRx(mcaspHandle);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Trigger McASP transmit operation */
    status = MCASP_startTransferTx(mcaspHandle);
    DebugP_assert(status == SystemP_SUCCESS);

    while (stopDemo != true)
    {
        ClockP_usleep(100000);
    }

    MCASP_stopTransferTx(mcaspHandle);
    MCASP_stopTransferRx(mcaspHandle);
}

void mcasp_txcb(MCASP_Handle handle, MCASP_Transaction *transaction)
{
    shm_handle hShm = (shm_handle)transaction->args;
    uint16_t reqSize = APP_MCASP_AUDIO_BUFF_SIZE;

    DebugP_assert(hShm != NULL);
    shm_read(shm_core_c7x, hShm, transaction->buf, &reqSize);

    if (reqSize == APP_MCASP_AUDIO_BUFF_SIZE)
    CacheP_inv(transaction->buf, APP_MCASP_AUDIO_BUFF_SIZE, CacheP_TYPE_ALL);

    gpio_sm_spin(&gMcaspTxSm, transaction->buf, 16);
    MCASP_submitTx(handle, transaction);
}

void mcasp_rxcb(MCASP_Handle handle, MCASP_Transaction *transaction)
{
    /* Invalidate the Cache. */
    CacheP_inv(transaction->buf, APP_MCASP_AUDIO_BUFF_SIZE, CacheP_TYPE_ALL);

    gpio_sm_spin(&gMcaspRxSm, transaction->buf, 16);

    shm_handle hShm = (shm_handle)transaction->args;
    DebugP_assert(hShm != NULL);

    /* Do not start writing to */
    /* Copy the data to shared Memory. */
    shm_write(shm_core_c7x, hShm, transaction->buf, APP_MCASP_AUDIO_BUFF_SIZE);
    MCASP_submitRx(handle, transaction);

    if (notifyCounter == 0)
    {
        notifyCounter = 2;
        /* Notify R5 Core. to start timer. */
        IpcNotify_sendMsg(CSL_CORE_ID_R5FSS0_0, gClientId, 0, 0);
    }
    else
    {
        notifyCounter -= 1;
    }

}

static shm_handle mcasp_createSharedMemory(void* const address, const int blockSize, const int totalSize)
{
    shm_handle handle;
    const uint32_t shmOvrHd = shm_metadata_overhead();
    /* floor the totalSize to be a multiple of blocksize, exclude the overhead size. */
    const uint32_t rxShmBufSize = ((totalSize-shmOvrHd)/blockSize)*blockSize + shmOvrHd;
    memset(address, 0, rxShmBufSize);
    handle = shm_create(shm_core_c7x, (uint32_t)address, rxShmBufSize);
    return handle;
}

static void ipc_notify_msg_handler_main_core(uint16_t remoteCoreId, \
            uint16_t localClientId, uint32_t msgValue, void *args)
{

}

