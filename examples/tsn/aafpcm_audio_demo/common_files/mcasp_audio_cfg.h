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

#ifndef MCASP_AUDIO_CFG_H
#define MCASP_AUDIO_CFG_H

/* Single source of truth for McASP audio-path sizing, shared by
   remote_mcasp_playback.c, autoamp_mcasp_audio.c, and demo_task.c. */

/* Audio buffer settings. BUFF_SIZE is in bytes: 192 = 6 frames = 125 us at 48 kHz,
 * the same period as one AVTP packet. */
#define APP_MCASP_AUDIO_BUFF_COUNT   (2U)
#define APP_MCASP_AUDIO_BUFF_SIZE    (192)

/* This is shared Memory Block size,
   Needs to be sized according to the Maximum PT Offset. */
#define APP_MCASP_SHM_BLOCK_SIZE     (3072U)   /* 96 frames, so about 2 ms of audio */

/* Bytes per sample (32-bit slots), not bits. */
#define APP_MCASP_BIT_DEPTH          (4U)
#define APP_MCASP_AUDIO_CHANNELS     (8U)

/* One PCM sample-frame (one sample per channel) in bytes. */
#define APP_MCASP_SAMPLE_FRAME_SIZE  (APP_MCASP_BIT_DEPTH * APP_MCASP_AUDIO_CHANNELS)
#define APP_MCASP_SHM_SAMPLE_FRAMES  (APP_MCASP_SHM_BLOCK_SIZE / APP_MCASP_SAMPLE_FRAME_SIZE)
#define APP_MCASP_RXCB_SAMPLE_COUNT  (APP_MCASP_AUDIO_BUFF_SIZE / APP_MCASP_SAMPLE_FRAME_SIZE)
#define APP_MCASP_TXCB_SAMPLE_COUNT  (APP_MCASP_AUDIO_BUFF_SIZE / APP_MCASP_SAMPLE_FRAME_SIZE)

#endif /* MCASP_AUDIO_CFG_H */
