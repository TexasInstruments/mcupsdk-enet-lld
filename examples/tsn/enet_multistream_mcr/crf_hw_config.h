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

/*!
 * \file   crf_hw_config.h
 *
 * \brief  This file contains CRF Hw Configuration Functions.
 */

#ifndef __CRF_HW_CONFIG_H__
#define __CRF_HW_CONFIG_H__

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "stdint.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/* None */
/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

/**
 * @brief Sets up the CRF hardware configuration.
 *
 * This function initializes and configures the hardware required for
 * Clock Reference Frequency (CRF) operations.
 *
 * @return Returns 0 on success, or a negative error code on failure.
 */
int32_t crfHwConfig_setup(void);

/**
 * @brief Estimates the edge difference between media clock and TS signal.
 *
 * Calculates the difference in edges between the media clock frequency and
 * the timestamp (TS) signal frequency.
 *
 * @param mediaClockFreq Frequency of the media clock in Hz.
 * @param tsSignalFreq Frequency of the TS signal in Hz.
 * @return Estimated edge difference as a 64-bit integer.
 */
int64_t crfHwConfig_estimateEdgeDiff(double mediaClockFreq, double tsSignalFreq);

/**
 * @brief Routes the timestamp signal to the PHY.
 *
 * Configures the hardware to route the timestamp signal to the physical layer (PHY).
 *
 * @return Returns 0 on success, or a negative error code on failure.
 */
int32_t crfHwConfig_routeTsSignalToPhy(void);

#endif /* __CRF_HW_CONFIG_H__ */
