/*
 *  Copyright (C) Texas Instruments Incorporated 2025
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
 * \file follower_clock_sm.c
 *
 * \brief This file implements the API for state machine syncronization for correcting follower clock based on reference clock
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "follower_clock_sm.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


static void FollowerClock_updateAlpha(float error, float targetError, double *alpha)
{
    float error_factor = (float)error / (float)targetError;
    if (error_factor > 10.0f) error_factor = 10.0f;

    float adaptive_alpha = (error > targetError) ? *alpha / error_factor : 0.95;
    if (adaptive_alpha < 0.05f) adaptive_alpha = 0.05f;
    if (adaptive_alpha > 0.95f) adaptive_alpha = 0.95f;
    *alpha = adaptive_alpha;
}

void FollowerClock_initStateMachine(FollowerClock_StateMachine *handle, FollowerClock_params params)
{
    handle->state = FollowerClock_SET;
    handle->ratio = 1;
    handle->offset = 0;

    handle->prev_refCounts = 0;
    handle->prev_followerCounts = 0;
    handle->params = params;

}
void FollowerClock_intStateMachineParams(FollowerClock_params * params)
{
    params->target_error_s = 500;
    params->alpha_ratio = 0.95;
    params->alpha_offset = 0.95;
}

void FollowerClock_syncStateMachine(FollowerClock_StateMachine *handle, uint64_t refclk_count, uint64_t folclk_count)
{

    double currentRatio = 1.0f, currentOffset = 0.0f;
    double error = 0;
    double error_abs = 0, currentDerivedTime = 0;

    switch(handle->state)
    {
        case FollowerClock_SET:
            handle->prev_followerCounts = folclk_count;
            handle->prev_refCounts = refclk_count;
            handle->state = FollowerClock_BEGIN;
            break;
        case FollowerClock_BEGIN:
            handle->ratio = (double)(refclk_count - handle->prev_refCounts) /
                            (double)(folclk_count - handle->prev_followerCounts);
            handle->offset = (double)refclk_count - handle->ratio * (double)folclk_count;
            handle->state = FollowerClock_CONTROLLER;
            break;
        case FollowerClock_CONTROLLER:
            currentDerivedTime = (double)folclk_count * handle->ratio + handle->offset;
            error = (double)refclk_count - currentDerivedTime;
            error_abs = (error > 0) ? error : (-error);
            currentRatio = (double)(refclk_count - handle->prev_refCounts)/ \
                        (double)(folclk_count - handle->prev_followerCounts);
            currentOffset = (double)refclk_count - currentRatio * (double)folclk_count;

            FollowerClock_updateAlpha(error_abs, handle->params.target_error_s, &(handle->params.alpha_offset));
            FollowerClock_updateAlpha(error_abs, handle->params.target_error_s, &(handle->params.alpha_ratio));
            handle->ratio = handle->params.alpha_ratio * handle->ratio + currentRatio - handle->params.alpha_ratio * currentRatio;
            handle->offset = handle->params.alpha_offset * handle->offset + currentOffset - handle->params.alpha_offset * currentOffset;
            break;
        case FollowerClock_STOP:
            break;
        default:
            break;
    }
    handle->prev_refCounts = refclk_count;
    handle->prev_followerCounts = folclk_count;
}

void FollowerClock_deinitStateMachine(FollowerClock_StateMachine *handle)
{
    handle->state = FollowerClock_STOP;
}


