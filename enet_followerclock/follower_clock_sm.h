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
 * \file follower_clock_sm.h
 *
 * \brief This file declares the API for state machine syncronization for correcting follower clock based on reference clock
 */


#ifndef FOLLOWER_CLOCK_SM
#define FOLLOWER_CLOCK_SM
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <math.h>
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */


typedef enum FollowerClock_SM_State_s
{
    FollowerClock_SET,
    FollowerClock_BEGIN,
    FollowerClock_CONTROLLER,
    FollowerClock_STOP,
}FollowerClock_SM_State;

typedef struct FollowerClock_params_s
{
    uint64_t target_error_s;
    double alpha_ratio;
    double alpha_offset;
}FollowerClock_params;

typedef struct FollowerClock_StateMachine_s
{
    double ratio;
    double offset;
    FollowerClock_SM_State state;
    FollowerClock_params params;
    uint64_t prev_refCounts;
    uint64_t prev_followerCounts;
}FollowerClock_StateMachine;


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void FollowerClock_intStateMachineParams(FollowerClock_params * params);

void FollowerClock_initStateMachine(FollowerClock_StateMachine *handle,FollowerClock_params);

void FollowerClock_deinitStateMachine(FollowerClock_StateMachine *handle);

void FollowerClock_syncStateMachine(FollowerClock_StateMachine *handle, uint64_t refclk_count, uint64_t folclk_count);


/* ========================================================================== */                            
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */



#endif