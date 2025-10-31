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
#ifndef SAMPLE_AUDIO_H_
#define SAMPLE_AUDIO_H_

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

#ifndef DOLBY_SAMPLE_6CHANNELS

#define EC3_SYNC_FRMSIZE                 (384)
#define AUDIO24BIT_AES3FRAMES_NEEDED     (EC3_SYNC_FRMSIZE * 2 / 6)
#define SMPTE_PREAMBLE_AES3FRAMES_NEEDED (2)
#define AES3_SIZE_OF_FRAME               (8)
#define AES3_CARRY_SMPTE_DATA_SIZE       (AUDIO24BIT_AES3FRAMES_NEEDED + SMPTE_PREAMBLE_AES3FRAMES_NEEDED)*AES3_SIZE_OF_FRAME

#else
#define EC3_SYNC_FRMSIZE                 (1792/2)
#define AUDIO24BIT_AES3FRAMES_NEEDED     ((EC3_SYNC_FRMSIZE * 2 / 6) + 1)
#define SMPTE_PREAMBLE_AES3FRAMES_NEEDED (2)
#define AES3_SIZE_OF_FRAME               (8)
#define AES3_CARRY_SMPTE_DATA_SIZE       (AUDIO24BIT_AES3FRAMES_NEEDED + SMPTE_PREAMBLE_AES3FRAMES_NEEDED)*AES3_SIZE_OF_FRAME

#endif

/* ========================================================================== */
/*                              Global Variables                              */
/* ========================================================================== */

extern unsigned char payload_sample[];

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Retrieves the size of the audio payload sample.
 *
 * This function returns the size (in bytes) of a single audio payload sample.
 *
 * @return int Size of the payload sample in bytes.
 */
int get_payload_sample_size(void);

#endif /* SAMPLE_AUDIO_H_ */