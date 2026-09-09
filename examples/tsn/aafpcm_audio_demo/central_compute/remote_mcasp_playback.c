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
#include <kernel/dpl/ClockP.h>
#include <drivers/i2c.h>
#include <drivers/mcasp.h>
#include <drivers/hw_include/cslr_soc.h>
#include <board/ioexp/ioexp_tca6424.h>
#include <drivers/pinmux.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "shm_idxbuf.h"
#include "mcasp_audio_cfg.h"
#include <string.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Shared DDR carve-out for the two rings between C75 (McASP) and R5F (AVB) */
#define C7X_AVB_SHM_START_ADDRESS    (0x80400000U)

#define AVB_C7X_TO_R5_SHM_ADDR       (C7X_AVB_SHM_START_ADDRESS+0U)
#define AVB_R5_TO_C7X_SHM_ADDR       (C7X_AVB_SHM_START_ADDRESS+100*1024U)

/* APP_MCASP_* audio-path sizing macros come from mcasp_audio_cfg.h.
   Mcasp_rxCb indexes the AVB_C7X_TO_R5_SHM_ADDR buffer in units of
   APP_MCASP_SAMPLE_FRAME_SIZE, so the ShmIdxBuf index doubles as the
   media-clock edge count used for presentation-time derivation on the
   R5F Tx task side. */

/* ========================================================================== */
/*                           Global Variables                                 */
/* ========================================================================== */

/* Stop Demo Flag. */
volatile bool Mcasp_stopDemo = false;

/* Client ID for IPC Communication. */
uint32_t Mcasp_clientId = 4u;

/* Create buffers for transmit and Receive */
uint8_t Mcasp_audioBufferTx[APP_MCASP_AUDIO_BUFF_COUNT][APP_MCASP_AUDIO_BUFF_SIZE] __attribute__((aligned(256)));
uint8_t Mcasp_audioBufferRx[APP_MCASP_AUDIO_BUFF_COUNT][APP_MCASP_AUDIO_BUFF_SIZE] __attribute__((aligned(256)));

/* Create transaction objects for transmit and Receive */
MCASP_Transaction   Mcasp_audioTxnTx[APP_MCASP_AUDIO_BUFF_COUNT] = {0};
MCASP_Transaction   Mcasp_audioTxnRx[APP_MCASP_AUDIO_BUFF_COUNT] = {0};

uint32_t Mcasp_notifyCounter = 0;

/* Media-clock edge count of the next sample-frame Mcasp_rxCb will write.
   Advances by APP_MCASP_RXCB_SAMPLE_COUNT every callback; passed as the
   ShmIdxBuf_put() index so the R5F side can recover it from ShmIdxBuf_getFifo()
   for presentation-time computation. */
static uint64_t gMcaspRxEdgeCount = 0U;

/* Media-clock edge count of the next sample-frame Mcasp_txCb will fetch from
   the AVB_R5_TO_C7X_SHM_ADDR buffer - drives ShmIdxBuf_get() so playback
   pulls each PDU at the index its AVTP presentation time landed on
   (see central_aaf_app.c). Advances by APP_MCASP_TXCB_SAMPLE_COUNT every
   callback regardless of hit/miss, since the McASP frame clock free-runs. */
static uint64_t gMcaspTxEdgeCount = 0U;

/* ========================================================================== */
/*                        Extern Function Declaration                         */
/* ========================================================================== */
static void Mcasp_ipcNotifyMsgHandlerMainCore(uint16_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, void *args);

static ShmIdxBuf_Handle Mcasp_createSharedMemory(uint32_t address, uint32_t lockNumber,
                                                  uint32_t frameSize, uint32_t capacityFrames);

void Mcasp_playbackMain(void *args)
{
    int32_t     status = SystemP_SUCCESS;
    uint32_t    i;
    MCASP_Handle    mcaspHandle;

    status = IpcNotify_registerClient(Mcasp_clientId, Mcasp_ipcNotifyMsgHandlerMainCore, NULL);
    DebugP_assert(status==SystemP_SUCCESS);

    /* Create Tx Shared Memory. Frame-sized to one PCM sample-frame so its
       index doubles as a media-clock edge count (see Mcasp_rxCb). */
    ShmIdxBuf_Handle shmTxHandle = Mcasp_createSharedMemory((uint32_t)AVB_C7X_TO_R5_SHM_ADDR, 2U,
                                                             APP_MCASP_SAMPLE_FRAME_SIZE, APP_MCASP_SHM_SAMPLE_FRAMES);
    /* Create Rx Shared Memory. Frame-sized to one PCM sample-frame so its
       index doubles as a media-clock edge count (see Mcasp_txCb). */
    ShmIdxBuf_Handle shmRxHandle = Mcasp_createSharedMemory((uint32_t)AVB_R5_TO_C7X_SHM_ADDR, 3U,
                                                             APP_MCASP_SAMPLE_FRAME_SIZE, APP_MCASP_SHM_SAMPLE_FRAMES);

    /* Sync with R5F Core, so both rings exist before audio starts flowing. */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    /* Open MCASP driver after enabling the HCLK */
    gMcaspHandle[CONFIG_MCASP0] = MCASP_open(CONFIG_MCASP0, &gMcaspOpenParams[CONFIG_MCASP0]);
    if(NULL == gMcaspHandle[CONFIG_MCASP0])
    {
        DebugP_logError("MCASP open failed for instance %d !!!\r\n", CONFIG_MCASP0);
        DebugP_assert(false);
    }

    /* Second McASP only generates the timestamping signal that clock recovery captures */
    gMcaspHandle[CONFIG_TS_MCASP] = MCASP_open(CONFIG_TS_MCASP, &gMcaspOpenParams[CONFIG_TS_MCASP]);
    if (NULL == gMcaspHandle[CONFIG_TS_MCASP])
    {
        DebugP_logError("MCASP open failed for instance %d !!!\r\n", CONFIG_TS_MCASP);
        DebugP_assert(false);
    }

    mcaspHandle = MCASP_getHandle(CONFIG_MCASP0);

    /* Prepare and submit audio transaction transmit objects */
    for (i = 0U; i < APP_MCASP_AUDIO_BUFF_COUNT; i++)
    {
        Mcasp_audioTxnTx[i].buf     = (void *) &Mcasp_audioBufferTx[i][0];
        Mcasp_audioTxnTx[i].count   = APP_MCASP_AUDIO_BUFF_SIZE/4;
        Mcasp_audioTxnTx[i].timeout = 0xFFFFFF;
        Mcasp_audioTxnTx[i].args    = (void *)shmRxHandle;
        MCASP_submitTx(mcaspHandle, &Mcasp_audioTxnTx[i]);
        gMcaspTxEdgeCount += APP_MCASP_TXCB_SAMPLE_COUNT;
    }

    /* Prepare and submit audio transaction receive objects */
    for (i = 0U; i < APP_MCASP_AUDIO_BUFF_COUNT; i++)
    {
        Mcasp_audioTxnRx[i].buf     = (void *)&Mcasp_audioBufferRx[i][0];
        Mcasp_audioTxnRx[i].count   = APP_MCASP_AUDIO_BUFF_SIZE/4;
        Mcasp_audioTxnRx[i].timeout = 0xFFFFFF;
        Mcasp_audioTxnRx[i].args    = (void *)shmTxHandle;
        MCASP_submitRx(mcaspHandle, &Mcasp_audioTxnRx[i]);
    }

    /* IRQs off so the three starts below land as close together as possible */
    uintptr_t key = HwiP_disable();
    // Todo: Enable this when API is available 
    // status = MCASP_TxSyncStart(mcaspHandle, gMcaspHandle[CONFIG_TS_MCASP],
    //                             true, true, true, false);

    /* 
     * This Starts Timestamping McASP's FSYNC, which is fed to 
     * CPTS for timestamping, with 48KHz media clock, this frequency is 
     * set to 300Hz to get 160 division factor
    */
    MCASP_startTransferTx(gMcaspHandle[CONFIG_TS_MCASP]);

    /* 
     * This Starts Audio McASP's Operations and FSYNC(Media Clock)
    */
    MCASP_startTransferTx(mcaspHandle);
    MCASP_startTransferRx(mcaspHandle);

    HwiP_restore(key);

    while (Mcasp_stopDemo != true)
    {
        ClockP_usleep(100000);
    }

    MCASP_stopTransferTx(mcaspHandle);
    MCASP_stopTransferRx(mcaspHandle);
}

void EnetApp_TxMcaspTxCb(MCASP_Handle handle,
                          MCASP_Transaction *transaction)
{
    /* Empty Tx Callback. */
}

void Mcasp_txCb(MCASP_Handle handle, MCASP_Transaction *transaction)
{
    ShmIdxBuf_Handle hShm = (ShmIdxBuf_Handle)transaction->args;
    uint64_t outBaseIndex;
    ShmIdxBuf_Status status;

    DebugP_assert(hShm != NULL);

    /* Fetch the sample-frames due to play out at the current media-clock
       edge count. The McASP frame clock is real-time and free-running, so
       every callback must submit a buffer - on a miss, play silence for
       this slot instead of blocking/retrying. */
    status = ShmIdxBuf_get(hShm, gMcaspTxEdgeCount, APP_MCASP_TXCB_SAMPLE_COUNT, transaction->buf, &outBaseIndex);

    if (status == SHMIDXBUF_OK)
    {
        CacheP_inv(transaction->buf, APP_MCASP_AUDIO_BUFF_SIZE, CacheP_TYPE_ALL);
    }
    else
    {
        /* SHMIDXBUF_ERR_NOT_READY: producer hasn't caught up to this index yet.
           SHMIDXBUF_ERR_TOO_OLD: this window was already evicted or delivered -
           resync to the buffer's current oldest-resident index. */
        memset(transaction->buf, 0, APP_MCASP_AUDIO_BUFF_SIZE);
    }
    gMcaspTxEdgeCount += APP_MCASP_TXCB_SAMPLE_COUNT;

    MCASP_submitTx(handle, transaction);
}

void Mcasp_rxCb(MCASP_Handle handle, MCASP_Transaction *transaction)
{
    /* Invalidate the Cache. */
    CacheP_inv(transaction->buf, APP_MCASP_AUDIO_BUFF_SIZE, CacheP_TYPE_ALL);

    ShmIdxBuf_Handle hShm = (ShmIdxBuf_Handle)transaction->args;
    DebugP_assert(hShm != NULL);

    /* Copy the data to shared Memory, indexed by media-clock edge count so
       the R5F Tx task can derive a presentation timestamp from it. */
    (void)ShmIdxBuf_put(hShm, gMcaspRxEdgeCount, transaction->buf, APP_MCASP_RXCB_SAMPLE_COUNT);
    gMcaspRxEdgeCount += APP_MCASP_RXCB_SAMPLE_COUNT;
    MCASP_submitRx(handle, transaction);

    /* Wakes the R5F, which uses it as the AVB talker tick (EnetApp_ipcNotifyCallback) */
    IpcNotify_sendMsg(CSL_CORE_ID_R5FSS0_0, Mcasp_clientId, 0, 0);

}

static ShmIdxBuf_Handle Mcasp_createSharedMemory(uint32_t address, uint32_t lockNumber,
                                                  uint32_t frameSize, uint32_t capacityFrames)
{
    ShmIdxBuf_Config cfg;

    cfg.startAddr             = address;
    cfg.regionSize            = ShmIdxBuf_getRequiredSize(capacityFrames, frameSize);
    cfg.frameSize             = frameSize;
    cfg.capacityFrames        = capacityFrames;
    cfg.spinlockBaseAddr      = CSL_SPINLOCK0_BASE;
    cfg.lockNumber            = lockNumber;
    cfg.lockTimeoutIterations = 0U;
    cfg.maxForwardJumpFrames  = 0U;
    cfg.forceInit             = false;
    cfg.clearStatsOnRead      = false;

    return ShmIdxBuf_create(&cfg);
}

static void Mcasp_ipcNotifyMsgHandlerMainCore(uint16_t remoteCoreId, \
            uint16_t localClientId, uint32_t msgValue, void *args)
{
    /* Empty IPC Notify Message Handler. */
}
