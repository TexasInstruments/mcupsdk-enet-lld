/*
 *  Copyright (c) Texas Instruments Incorporated 2022-23
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
 * \file  remote_main.c
 *
 * \brief This file contains the implementation of the client example entry
 *        point
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/ClockP.h>
#include "kernel/dpl/TimerP.h"
#include "ti_dpl_config.h"
#include "../shm_cirbuf.h"
#include "string.h"

#define SHM_AVB_DATA_RX_SIZE            (0x80000)
#define AAF_DOLBY_SYNC_FRAME_SIZE       (768)
#define SMPTE_HEADER_SIZE               (12)
#define SMPTE_FRAME_SIZE                (SMPTE_HEADER_SIZE + AAF_DOLBY_SYNC_FRAME_SIZE)

#if defined(SOC_AM62DX)
#define SHARED_MEMORY_START_ADDRESS     (0xA3000000)
#elif defined(SOC_AM275X)
/*
 * The SRAM address 0x72400000 is aliased to 0x80400000 in C7X Core,
 * Same reflects in the linker.cmd as well.
 */
#define SHARED_MEMORY_START_ADDRESS     (0x80400000)
#endif


SemaphoreP_Object gRemoteAppTimerSem;

shm_handle remoteAppInitShm(void* const address, const int blockSize, const int totalSize)
{
    const uint32_t shmOvrHd     = shm_metadata_overhead();

    /* Floor() the totalSize to be a multiple of blocksize, exclude the overhead size. */
    const uint32_t rxShmBufSize = ((totalSize-shmOvrHd)/blockSize)*blockSize + shmOvrHd;

    memset(address, 0, rxShmBufSize);
    shm_handle handle = shm_create((uint32_t)address, rxShmBufSize);

    DebugP_assert(handle != NULL);
    return handle;
}

void RemoteApp_mainTask(void *args)
{
    shm_handle shmHandle = remoteAppInitShm((void*)SHARED_MEMORY_START_ADDRESS,
                        SMPTE_FRAME_SIZE, SHM_AVB_DATA_RX_SIZE);

    SemaphoreP_constructBinary(&gRemoteAppTimerSem, 0);
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);
    while (1)
    {
        uint8_t test[SMPTE_FRAME_SIZE] = {0};
        SemaphoreP_pend(&gRemoteAppTimerSem, SystemP_WAIT_FOREVER);
        uint16_t dataLen = sizeof(test);
        ShdMemStatus status = shm_read(shmHandle, test, &dataLen);
        if (dataLen > 0)
        {
            /* Reading from shm successful. */
            DebugP_logInfo("Rcvd: %d\n", dataLen);
        }
    }
}

void remoteTimerIsr(void)
{
    SemaphoreP_post(&gRemoteAppTimerSem);
}
