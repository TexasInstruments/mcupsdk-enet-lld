/*
 *  Copyright (c) Texas Instruments Incorporated 2024-2025
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
 * \file   media_clock_ctl.h
 *
 * \brief This file contains all functions related Media Clock Control Functions.
 */

#ifndef __MEDIA_CLOCK_CTRL_H__
#define __MEDIA_CLOCK_CTRL_H__

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <enet.h>
#include "ti_enet_config.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
typedef struct MCC_Obj_s
{
    uint32_t magic;
    Enet_Handle hEnet;
    uint32_t portNum;
    uint32_t MediaClockFrequency;
    uint32_t timestampInterval;
    uint32_t coreId;
}MCC_Obj, *MCC_handle;

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

/**
 * @brief Initializes the Media Clock Control instance.
 *
 * This function initializes the Media Clock Control (MCC) instance with the specified
 * media clock frequency and timestamp interval.
 *
 * @param mcFrequency The media clock frequency in Hz.
 * @param timestampInterval The interval between timestamps in nanoseconds.
 *
 * @return MCC_handle Returns a handle to the initialized MCC instance.
 */
MCC_handle MCC_init(uint32_t mcFrequency, uint32_t timestampInterval);

/**
 * @brief Retrieves the current timestamp.
 *
 * This function retrieves the current timestamp from the media clock control instance.
 *
 * @param handle The handle to the media clock control instance.
 * @param timestamp Pointer to a variable where the retrieved timestamp will be stored.
 *
 * @return int32_t Returns 0 on success, or a negative error code on failure.
 */
int32_t MCC_getTimestamp(MCC_handle handle, uint64_t* timestamp);

#endif /* __MEDIA_CLOCK_CTRL_H__ */