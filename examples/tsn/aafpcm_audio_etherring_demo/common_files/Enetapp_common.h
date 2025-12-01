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
#ifndef __ENETAPP_COMMON_H_
#define __ENETAPP_COMMON_H_

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define GPTP_SYNC_WAIT_TIME_SEC  (10)

typedef enum
{
    NODE_INVALID = -1,
    NODE_CENTRAL,
    NODE_LEFT,
    NODE_RIGHT,
    NODE_TAIL,
    NODE_TOTAL_NODES,
}node_index;

typedef struct
{
    const char* nodeName;
}node_info;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t Board_CdceConfig(void);

int32_t Board_codecConfig(void);

int32_t Board_MuxSelMcASP4(void);

void EnetApp_configControlData(void* args);

int32_t TsMcasp_tsMcaspConfig(int instIdx);

void aaf_audio_task(void *args);

void EnetApp_AudioPlaybackDemoMain(void* args);

int32_t EnetApp_addMcastEntry(uint8_t *mcastAddr, uint8_t portMask, uint16_t vlanId);

void aafTask_signalCrfStart(void);

void EnetApp_printStats(uint64_t currentTime);

extern node_index gNodeIndex;

#endif /* __ENETAPP_COMMON_H_ */