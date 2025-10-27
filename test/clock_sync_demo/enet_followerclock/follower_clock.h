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
 * \file  follower_clock.h
 *
 * \brief This file defines the APIs for accessing timer count and calculation of approx time stamp
 * 
 * This is the interface of this module .
 * user of this module need to provide
 * 1. base address of timer
 * 2. reference timestamp function that will return the timestamp of the 
 *    aforemntioned timer overflow
 * 3. Register the callback for timer overflow
 * 4. Stack for the sync task
 * 
 *  Initialize the handle --> start the sync task --> use FollowerClock_getApproxtime() to get timestamp
 * 
 */

#ifndef FOLLOWER_CLOCK_INTERFACE
#define FOLLOWER_CLOCK_INTERFACE
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <kernel/dpl/TimerP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/TaskP.h>
#include <stdint.h>

#include "follower_clock_sm.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


typedef uint64_t (* getTimestamp_fptr )(void);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */


typedef struct FollowerClock_Handle_s
{
    /*state machine for calculating ratio and offset between reference and follower clock*/
    FollowerClock_StateMachine  sm;

    uint32_t timerBaseaddr;
    uint32_t relVal;
    uint32_t maxCount;
    uint32_t ovrflwCount;

    int8_t performanceLogs;
    uint32_t waitTime;
    SemaphoreP_Object overflowSem;
    TaskP_Object syncTask;

    /*function pointer for getting timestamp of timer overflow event from reference clock*/
    getTimestamp_fptr getReferenceTimestamp;
}FollowerClock_Handle;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void FollowerClock_init(FollowerClock_Handle *handle, 
                         uint32_t baseaddr, 
                         uint32_t periodInUsec,
                         getTimestamp_fptr getReferenceTimestamp,
                         uint32_t waitTime,
                         int8_t logs);

void FollowerClock_deinit(FollowerClock_Handle *handle);

uint64_t FollowerClock_getApproxtime(const FollowerClock_Handle * handle);

uint64_t FollowerClock_getCurrentTimerCount(const FollowerClock_Handle * handle);

int32_t FollowerClock_startSyncTask(FollowerClock_Handle * handle,
                                      uint8_t * stack, 
                                      uint32_t stack_size,
                                      uint32_t priority);

/* Register this call back function for timer overflow at the point of usage*/
void FollowerClock_notifySm(FollowerClock_Handle * handle);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

#endif