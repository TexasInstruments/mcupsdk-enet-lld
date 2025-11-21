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

typedef enum
{
    CRF_HW_CONFIG_CLKSRC_INVALID,
    CRF_HW_CONFIG_CLKSRC_PHY,
    CRF_HW_CONFIG_CLKSRC_CDCE,
} mediaClkSrc;

typedef struct
{
    mediaClkSrc clkSrc;
    uint32_t mediaClkFreq;
    uint32_t timestampingInterval;
    uint32_t mediaClkAtlSignal;
    uint32_t timestampingAtlSignal;
    int32_t edgeDiff;
}crfHwCfg_info;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Initializes the CRF hardware configuration.
 *
 * @param[in] info Pointer to the CRF hardware configuration information structure.
 * @return Status of the initialization (0 for success, negative for error).
 */
int32_t crfHwConfig_init(const crfHwCfg_info* info);

/**
 * @brief Sets up the CRF hardware configuration to be called after Clock is stable
 *
 * @param[in,out] info Pointer to the CRF hardware configuration information structure.
 * @return Status of the setup (0 for success, negative for error).
 */
int32_t crfHwConfig_setup(crfHwCfg_info* info);

/**
 * @brief Retrieves the media clock edge timestamp.
 *
 * @param[in] info Pointer to the CRF hardware configuration information structure.
 * @return Media clock edge timestamp as a 64-bit unsigned integer.
 */
uint64_t crfHwConfig_getMediaClockEdge(const crfHwCfg_info* info);

/**
 * @brief Fine-tunes the frequency of the Media Clock.
 *
 * @param[in] info Pointer to the CRF hardware configuration information structure.
 * @param[in] relPPM Relative frequency adjustment in parts per million (PPM).
 * @return Status of the frequency fine-tuning (0 for success, negative for error).
 */
int32_t crfHwConfig_fineTuneFreq(const crfHwCfg_info* info, double relPPM);

/**
 * @brief Adjusts the phase of the Media Clock.
 *
 * @param[in] info Pointer to the CRF hardware configuration information structure.
 * @param[in] cycles Number of cycles to adjust the phase by (positive or negative).
 * @return Status of the phase adjustment (0 for success, negative for error).
 */
int32_t crfHwConfig_adjPhase(const crfHwCfg_info* info, int8_t cycles);

/**
 * @brief Hook function called when a CRF timestamp is available.
 *
 * @param[in] args Pointer to user-defined arguments or context.
 */
void CrfHwConfig_crfTsCb(void* args);

#endif /* __CRF_HW_CONFIG_H_ */