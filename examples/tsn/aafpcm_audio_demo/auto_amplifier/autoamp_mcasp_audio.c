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
#include <drivers/hw_include/cslr_soc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "shm_idxbuf.h"
#include "mcasp_audio_cfg.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* APP_MCASP_* audio-path sizing macros come from mcasp_audio_cfg.h.
   AutoAmp_playbackMcaspRxCb indexes the AutoAmp_c7xToR5fShmBuffer buffer in
   units of APP_MCASP_SAMPLE_FRAME_SIZE, so the ShmIdxBuf index doubles as
   the media-clock edge count used for presentation-time derivation on the
   AVB Tx task side (aaf_avb_task.c). */

/* ========================================================================== */
/*                           Global Variables                                 */
/* ========================================================================== */

extern MCASP_Config gMcaspConfig[];

uint32_t AutoAmp_notifyCounter = 0;

/* Ping-pong DMA buffers, one pair per direction. Count/size come from
   mcasp_audio_cfg.h, 256 alignment keeps them cache-line safe. */
uint8_t AutoAmp_mcaspAudioBufferTx[APP_MCASP_AUDIO_BUFF_COUNT][APP_MCASP_AUDIO_BUFF_SIZE] __attribute__((aligned(256)));
uint8_t AutoAmp_mcaspAudioBufferRx[APP_MCASP_AUDIO_BUFF_COUNT][APP_MCASP_AUDIO_BUFF_SIZE] __attribute__((aligned(256)));

/* Create transaction objects for transmit and Receive */
MCASP_Transaction AutoAmp_mcaspAudioTxnTx[APP_MCASP_AUDIO_BUFF_COUNT] = {0};
MCASP_Transaction AutoAmp_mcaspAudioTxnRx[APP_MCASP_AUDIO_BUFF_COUNT] = {0};

/* Backing store for the two ShmIdxBuf rings between McASP and the AVB tasks.
   Sized for the max presentation-time offset, see APP_MCASP_SHM_BLOCK_SIZE. */
uint8_t AutoAmp_c7xToR5fShmBuffer[APP_MCASP_SHM_BLOCK_SIZE*APP_MCASP_AUDIO_BUFF_COUNT*2] __attribute__((aligned(1024)));
uint8_t AutoAmp_r5fToC7xShmBuffer[APP_MCASP_SHM_BLOCK_SIZE*APP_MCASP_AUDIO_BUFF_COUNT*2] __attribute__((aligned(1024)));

ShmIdxBuf_Handle AutoAmp_shmTxHandle;
ShmIdxBuf_Handle AutoAmp_shmRxHandle;

/* Media-clock edge count of the next sample-frame AutoAmp_playbackMcaspRxCb
   will write. Advances by APP_MCASP_RXCB_SAMPLE_COUNT every write; passed as
   the ShmIdxBuf_put() index so aaf_avb_task.c can recover it from
   ShmIdxBuf_getFifo() for presentation-time computation. */
static uint64_t gAutoAmpRxEdgeCount = 0U;

/* Media-clock edge count of the next sample-frame AutoAmp_playbackMcaspTxCb
   will fetch from the AutoAmp_r5fToC7xShmBuffer - drives ShmIdxBuf_get() so
   playback pulls each PDU at the index its AVTP presentation time landed on
   (see aaf_avb_task.c). Advances by APP_MCASP_TXCB_SAMPLE_COUNT every
   callback regardless of hit/miss, since the McASP frame clock free-runs. */
static uint64_t gAutoAmpTxEdgeCount = 0U;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static ShmIdxBuf_Handle AutoAmp_createSharedMemory(uint32_t address, uint32_t lockNumber,
                                                    uint32_t frameSize, uint32_t capacityFrames);

static bool AutoAmp_startTx(void);

void AutoAmp_syncTimer(void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t AutoAmp_playbackMcaspConfig(void)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t i;
    MCASP_Handle mcaspHandle;

    /* Playback Instance Index in Driver. */
    int32_t instIdx = CONFIG_MCASP0;

    /* Create Tx Shared Memory. Frame-sized to one PCM sample-frame so its
       index doubles as a media-clock edge count (see AutoAmp_playbackMcaspRxCb). */
    AutoAmp_shmTxHandle = AutoAmp_createSharedMemory((uint32_t)AutoAmp_c7xToR5fShmBuffer, 1U,
                                                      APP_MCASP_SAMPLE_FRAME_SIZE, APP_MCASP_SHM_SAMPLE_FRAMES);
    /* Create Rx Shared Memory. Frame-sized to one PCM sample-frame so its
       index doubles as a media-clock edge count (see AutoAmp_playbackMcaspTxCb). */
    AutoAmp_shmRxHandle = AutoAmp_createSharedMemory((uint32_t)AutoAmp_r5fToC7xShmBuffer, 0U,
                                                      APP_MCASP_SAMPLE_FRAME_SIZE, APP_MCASP_SHM_SAMPLE_FRAMES);

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
        AutoAmp_mcaspAudioTxnTx[i].buf     = (void *)&AutoAmp_mcaspAudioBufferTx[i][0];
        AutoAmp_mcaspAudioTxnTx[i].count   = APP_MCASP_AUDIO_BUFF_SIZE/4;
        AutoAmp_mcaspAudioTxnTx[i].timeout = 0xFFFFFF;
        /* Tx callback reads from the ring the AVB Rx task fills */
        AutoAmp_mcaspAudioTxnTx[i].args    = (void *)AutoAmp_shmRxHandle;
        MCASP_submitTx(mcaspHandle, &AutoAmp_mcaspAudioTxnTx[i]);
        /* Pre-queued buffers will play before the first callback, count them */
        gAutoAmpTxEdgeCount += APP_MCASP_TXCB_SAMPLE_COUNT;
    }

    /* Prepare and submit audio transaction receive objects */
    for (i = 0U; i < APP_MCASP_AUDIO_BUFF_COUNT; i++)
    {
        AutoAmp_mcaspAudioTxnRx[i].buf     = (void *)&AutoAmp_mcaspAudioBufferRx[i][0];
        AutoAmp_mcaspAudioTxnRx[i].count   = APP_MCASP_AUDIO_BUFF_SIZE/4;
        AutoAmp_mcaspAudioTxnRx[i].timeout = 0xFFFFFF;
        /* Rx callback writes into the ring the AVB Tx task drains */
        AutoAmp_mcaspAudioTxnRx[i].args    = (void *)AutoAmp_shmTxHandle;
        MCASP_submitRx(mcaspHandle, &AutoAmp_mcaspAudioTxnRx[i]);
    }

    return status;
}

static ShmIdxBuf_Handle AutoAmp_createSharedMemory(uint32_t address, uint32_t lockNumber,
                                                    uint32_t frameSize, uint32_t capacityFrames)
{
    ShmIdxBuf_Config cfg;

    /* The hardware spinlock guards the ring when both cores/tasks touch it */
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

void AutoAmp_playbackMcaspTxCb(MCASP_Handle handle, MCASP_Transaction *transaction)
{
    ShmIdxBuf_Handle hShm = (ShmIdxBuf_Handle)transaction->args;
    uint64_t outBaseIndex;
    ShmIdxBuf_Status status;
    DebugP_assert(hShm != NULL);

    /* Fetch the sample-frames due to play out at the current media-clock
       edge count. The McASP frame clock is real-time and free-running, so
       every callback must submit a buffer - on a miss, play silence for
       this slot instead of blocking/retrying. */
    status = ShmIdxBuf_get(hShm, gAutoAmpTxEdgeCount, APP_MCASP_TXCB_SAMPLE_COUNT, transaction->buf, &outBaseIndex);
    if (status != SHMIDXBUF_OK)
    {
        /* SHMIDXBUF_ERR_NOT_READY: producer hasn't caught up to this index yet.
           SHMIDXBUF_ERR_TOO_OLD: this window was already evicted or delivered -
           resync to the buffer's current oldest-resident index. */
        memset(transaction->buf, 0, APP_MCASP_AUDIO_BUFF_SIZE);
    }
    gAutoAmpTxEdgeCount += APP_MCASP_TXCB_SAMPLE_COUNT;

    /* Flush so the DMA sees the data we just wrote, not stale cache lines */
    CacheP_wb(transaction->buf, APP_MCASP_AUDIO_BUFF_SIZE, CacheP_TYPE_ALL);
    MCASP_submitTx(handle, transaction);
}

void AutoAmp_playbackMcaspRxCb(MCASP_Handle handle, MCASP_Transaction *transaction)
{
    ShmIdxBuf_Handle hShm = (ShmIdxBuf_Handle)transaction->args;
    DebugP_assert(hShm != NULL);

    /* Write to Shm only if Start is indicated. */
    if (AutoAmp_startTx() == true)
    {
        CacheP_inv(transaction->buf, APP_MCASP_AUDIO_BUFF_SIZE, CacheP_TYPE_ALL);
        /* Indexed by media-clock edge count so the AVB Tx task can derive a
           presentation timestamp from it. */
        (void)ShmIdxBuf_put(hShm, gAutoAmpRxEdgeCount, transaction->buf, APP_MCASP_RXCB_SAMPLE_COUNT);
        gAutoAmpRxEdgeCount += APP_MCASP_RXCB_SAMPLE_COUNT;
    }
    MCASP_submitRx(handle, transaction);

    /* Rx completion is the talker tick source, so AVB Tx stays locked to the audio clock */
    AutoAmp_syncTimer();

}

static bool AutoAmp_startTx(void)
{
    extern bool AutoAmp_audioTxStarted;
    return AutoAmp_audioTxStarted;
}
