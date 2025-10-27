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
 * \file  follower_clock.c
 *
 * \brief This file implements the APIs for accessing timer count and calculation of approx time stamp
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "follower_clock.h"
#include "follower_clock_sm.h"
#include <kernel/dpl/ClockP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TIMER_TCLR                          (uint32_t)0x38U
#define TIMER_TCLR_TRG_MASK                 (uint32_t)0xC00U
#define TIMER_TCLR_TRG_SHIFT                (uint32_t)0x0AU

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

void FollowerClock_notifySm(FollowerClock_Handle * handle)
{
    handle->ovrflwCount++;
    SemaphoreP_post(& (handle->overflowSem) );
}
/*
   This function needs to different between AM243X and AM62DX as accessing timer registers are different
*/
#ifdef SOC_AM243X
static void FollowerClock_setTrigSignalMode(FollowerClock_Handle * handle, uint32_t pwmTrigOutMode)
{
    uint32_t regVal = 0U;
    volatile uint32_t *addr = (volatile uint32_t *)(handle->timerBaseaddr + TIMER_TCLR);
    regVal = *addr;
    regVal &= ~TIMER_TCLR_TRG_MASK;
    regVal |= (pwmTrigOutMode << TIMER_TCLR_TRG_SHIFT);
    *addr = regVal;
}

#elif defined( SOC_AM62DX) || defined(SOC_AM275X)
#define TIMER_TWPS              (0x48u)
#define TIMER_TCLR_PEND_SHIFT   (0U)
#define TIMER_TCLR_PEND_MASK    (1U << TIMER_TCLR_PEND_SHIFT)

static void FollowerClock_setTrigSignalMode(FollowerClock_Handle * handle, uint32_t pwmTrigOutMode)
{
    uint32_t regVal = 0U;
    volatile uint32_t* twps_addr = (volatile uint32_t *)(handle->timerBaseaddr + TIMER_TWPS);
    volatile uint32_t *addr = (volatile uint32_t *)(handle->timerBaseaddr + TIMER_TCLR);
    while((*twps_addr & TIMER_TCLR_PEND_MASK) == TIMER_TCLR_PEND_MASK)
    {}
    regVal = *addr;
    regVal &= ~TIMER_TCLR_TRG_MASK;
    regVal |= (pwmTrigOutMode << TIMER_TCLR_TRG_SHIFT);
    *addr = regVal;
    while((*twps_addr & TIMER_TCLR_PEND_MASK) == TIMER_TCLR_PEND_MASK)
    {}
}
#else
#error "SOC not supported"
#endif

void FollowerClock_init(FollowerClock_Handle *handle, uint32_t baseaddr, 
                 uint32_t periodInUsec,getTimestamp_fptr getReferenceTimestamp,uint32_t waitTime, int8_t logs)
{
    TimerP_Params timer_params;
    FollowerClock_params params;

    FollowerClock_intStateMachineParams(&params);
    FollowerClock_initStateMachine(&handle->sm,params);

    handle->maxCount = 0xffffffffU;
    handle->ovrflwCount = 0;
    handle->timerBaseaddr = baseaddr;

    TimerP_Params_init(&timer_params);

    timer_params.periodInUsec = periodInUsec;

    TimerP_setup(handle->timerBaseaddr,&timer_params);

    handle->relVal = TimerP_getReloadCount(handle->timerBaseaddr);

    handle->waitTime = waitTime;

    handle->performanceLogs = logs;

    handle->getReferenceTimestamp = getReferenceTimestamp;

    FollowerClock_setTrigSignalMode(handle,1U);
}

void FollowerClock_deinit(FollowerClock_Handle *handle)
{
    FollowerClock_deinitStateMachine(&handle->sm);
}

static void FollowerClock_syncTime(FollowerClock_Handle *handle, uint64_t refclk_count, uint64_t folclk_count)
{
    FollowerClock_syncStateMachine(&handle->sm,refclk_count,folclk_count);
}

uint64_t FollowerClock_getApproxtime(const FollowerClock_Handle * handle)
{
    double curr_folclkcnt = (double)FollowerClock_getCurrentTimerCount(handle);
    double approx_ts = (handle->sm.ratio) * (curr_folclkcnt) ;
    approx_ts = approx_ts + handle->sm.offset;
    return (uint64_t)approx_ts;
}

static int64_t FollowerClock_calculateError(FollowerClock_Handle * handle, uint64_t currentCpts, uint64_t currentTimer)
{
    double approx_ts = handle->sm.ratio * (double)currentTimer + handle->sm.offset;
    double error = (double)currentCpts - approx_ts;
    return (int64_t)error;
}

static uint64_t FollowerClock_getOverflowedCount(FollowerClock_Handle * handle)
{
    uint64_t total_overflowed_time = (handle->maxCount - handle->relVal);
    total_overflowed_time *= (handle->ovrflwCount);
    return total_overflowed_time;
}

uint64_t FollowerClock_getCurrentTimerCount(const FollowerClock_Handle * handle)
{
    uint64_t current_timercount;
    uint32_t overflow_count1, overflow_count2;

    /*This is used for resolving  race condition between ISR and this function
      This practically solves it without lock
    */
    do
    {
        overflow_count1 = handle->ovrflwCount;
        current_timercount = (uint64_t)(TimerP_getCount(handle->timerBaseaddr) - handle->relVal);
        overflow_count2 = handle->ovrflwCount;
    }while(overflow_count1 != overflow_count2);

    current_timercount +=  (uint64_t)overflow_count1*(handle->maxCount - handle->relVal);

    return current_timercount;
}

static void FollowerClock_syncTask(void *arg)
{
    uint64_t total_error = 0U;
    int count = 0;

    FollowerClock_Handle* handle = (FollowerClock_Handle*)arg;

    DebugP_log("------- SYNC Task Started ------ \r\n");

    SemaphoreP_constructBinary(&(handle->overflowSem), 0);
    TimerP_start(handle->timerBaseaddr);

    while (handle->sm.state!=FollowerClock_STOP)
    {
        int64_t error = 0U;
        uint64_t cptsCount = 0U, timerCount = 0U, del_cpts;

        SemaphoreP_pend(&(handle->overflowSem), SystemP_WAIT_FOREVER);

        /*as semaphore is posted by timer overfloe ISR, waiting for CPTS PUSH event to stored into the queue*/
        ClockP_usleep(handle->waitTime);

        cptsCount = handle->getReferenceTimestamp();

        /*
        Synchronization between CPTS and folower timer takes place at Timer overflow event.
        The time stamp of Timer overflow event is captured by Hardware thus it is more accurate.
        trying to getting the time from CPTS and approx time from Timer will inherently cause some delay thus
        we can not get synchronization.

        for this reason FollowerClock_getOverflowedCount should be called here instead of FollowerClock_getApproxtime
        */
        timerCount = FollowerClock_getOverflowedCount(handle);

        FollowerClock_syncTime(handle, cptsCount, timerCount);

        if(handle->sm.state==FollowerClock_CONTROLLER){
           count++;
           total_error += (error>0)?error:-error;
        }
        if(handle->performanceLogs==1){

            del_cpts = cptsCount - handle->sm.prev_refCounts;

            error = FollowerClock_calculateError(handle, cptsCount, timerCount);

            DebugP_log("Sync no = %4d  | Ratio = %10.8f | Offset = %10llu | cpts = %8llu |cpts_diff = %10lld | timer_count = %10llu | Error_sync = %lld ns \r\n",
                count, handle->sm.ratio, (uint64_t)handle->sm.offset, cptsCount, del_cpts, timerCount,error);
        }
    }

    double average_error = (double)total_error/count;

    DebugP_log("Sync Task : Avarage sync error : %lf \r\n",average_error);

    SemaphoreP_destruct(&(handle->overflowSem));

    DebugP_log("----- SYNC Task finished ------- \r\n");

    TaskP_exit();
    TaskP_destruct(&handle->syncTask);
}

int32_t FollowerClock_startSyncTask(FollowerClock_Handle * handle, uint8_t * stack, uint32_t stack_size, uint32_t priority)
{
    int32_t status;
    TaskP_Params syncParams;
    TaskP_Params_init(&syncParams);

    syncParams.args = (void*)handle;
    syncParams.name = "CPTS_sync";
    syncParams.stackSize = stack_size;
    syncParams.stack = stack;
    syncParams.priority = priority;
    syncParams.taskMain = FollowerClock_syncTask;

    status = TaskP_construct(&handle->syncTask, &syncParams);

    return status;
}


