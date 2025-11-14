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

#ifndef __CRF_HW_CONFIG_H_
#define __CRF_HW_CONFIG_H_

#include <stdint.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TS_ROUTER_IN_ATL_BWS_SEL_0          (20)
#define TS_ROUTER_IN_ATL_BWS_SEL_1          (21)
#define TS_ROUTER_IN_ATL_BWS_SEL_2          (22)
#define TS_ROUTER_IN_ATL_BWS_SEL_3          (23)

#define TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_1   (8)
#define TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_2   (9)
#define TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_3   (10)
#define TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_4   (11)
#define TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_5   (12)
#define TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_6   (13)
#define TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_7   (14)
#define TS_ROUTER_OUT_CPSW_CPTS_HW_PUSH_8   (15)

#define TS_ROUTER_OUT_SYNC0_OUT             (16)
#define TS_ROUTER_OUT_SYNC1_OUT             (17)
#define TS_ROUTER_OUT_SYNC2_OUT             (18)
#define TS_ROUTER_OUT_SYNC3_OUT             (19)

#define CTRL_MMR0_CFG0_ATL_BWS0_SEL         (0x001082C0)
#define CTRL_MMR0_CFG0_ATL_BWS1_SEL         (0x001082C4)
#define CTRL_MMR0_CFG0_ATL_BWS2_SEL         (0x001082C8)
#define CTRL_MMR0_CFG0_ATL_BWS3_SEL         (0x001082CC)

#define ATL_BWSX_MCASP0_AFSR_IN             (0b0000)
#define ATL_BWSX_MCASP1_AFSR_IN             (0b0001)
#define ATL_BWSX_MCASP2_AFSR_IN             (0b0010)
#define ATL_BWSX_MCASP3_AFSR_IN             (0b0011)
#define ATL_BWSX_MCASP4_AFSR_IN             (0b0100)
#define ATL_BWSX_MCASP0_AFSX_IN             (0b0101)
#define ATL_BWSX_MCASP1_AFSX_IN             (0b0110)
#define ATL_BWSX_MCASP2_AFSX_IN             (0b0111)
#define ATL_BWSX_MCASP3_AFSX_IN             (0b1000)
#define ATL_BWSX_MCASP4_AFSX_IN             (0b1001)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t crfHwConfig_setTimeSyncRouter(const uint32_t src, const uint32_t dest, uint32_t set);

void crfHwConfig_setAtlMux(const uint32_t atlBwsReg, uint32_t muxVal);

void crfHwConfig_setPinmux(void);

int32_t crfHwConfig_AtlMuxConfig(void);

int32_t crfHwConfig_setMediaClockAtSyncOut(void);

int64_t crfHwConfig_estimateEdgeDiff(int atlTsSignal, int atlMcSignal, \
                        double mediaClockFreq, double tsSignalFreq);

int32_t crfHwConfig_attachTssToCpts(int atlTsSignal, void (*tssCb)(void*));

uint64_t crfHwConfig_getMediaClockEdge(void);

#endif /* __CRF_HW_CONFIG_H_ */