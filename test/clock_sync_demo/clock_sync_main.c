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
 * \file  clock_sync_main.c
 *
 * \brief This file contains the implementation of the CLOCK SYNC example entry
 *        point
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <stdint.h>
#include <assert.h>

#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/TimerP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CycleCounterP.h>

#include <enet_apputils.h>

#include "cpsw_cfg.h"
#include "follower_clock.h"

#include "ti_enet_open_close.h"
#include "ti_enet_config.h"
#include "ti_dpl_config.h"


/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */



/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define NUM_ITERATIONS       (20000000)
#define TASK_STACK_SIZE      (3*1024)
#define PERIOD_IN_US         (100000)

#define SYNC_TASK_PRIORITY   (5u)
#define CPU_CLK_FREQ_MHZ     (800u)

#define USE_FOLLOWERCLK
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

FollowerClock_Handle gTimeSync_handle;


static uint8_t gQueryStack[TASK_STACK_SIZE];
#if defined(USE_FOLLOWERCLK)
static uint8_t gSyncStack[TASK_STACK_SIZE];
#endif

TaskP_Object gQueryTask;
SemaphoreP_Object gQuerySem;

extern EnetApp_Cfg gEnetAppCfg;

#if defined SOC_AM243X
#define QUERY_TASK_TIMER     (gTimerBaseAddr[CONFIG_TIMER1])
#else
#define QUERY_TASK_TIMER     (gTimerBaseAddr[CONFIG_TIMER0])
#endif


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void timerISR_syncTask(void *arg0)
{
    FollowerClock_notifySm(&gTimeSync_handle);
}

void timerISR(void)
{
    SemaphoreP_post(&gQuerySem);
}


static void EnetApp_printCpuLoad()
{
    static uint32_t startTime_ms = 0;
    const  uint32_t currTime_ms  = ClockP_getTimeUsec()/1000;
    const  uint32_t printInterval_ms = 5000;

    if (startTime_ms == 0)
    {
        startTime_ms = currTime_ms;
    }
    else if ( (currTime_ms - startTime_ms) > printInterval_ms )
    {
        const uint32_t cpuLoad = TaskP_loadGetTotalCpuLoad();

        DebugP_log(" %6d.%3ds : CPU load = %3d.%02d %%\r\n",
                    currTime_ms/1000, currTime_ms%1000,
                    cpuLoad/100, cpuLoad%100 );

        startTime_ms = currTime_ms;
        TaskP_loadResetAll();
    }
    return;
}

void CPTS_queryTask(void *arg0)
{
    const int32_t stat_print_freq = 10000;

    uint64_t total_ft = 0;
    uint64_t max_ft = 0;
    int count = 0;

    EnetAppUtils_print("Query task started\r\n");

    #if defined(USE_FOLLOWERCLK)
    while (gTimeSync_handle.sm.state!=FollowerClock_CONTROLLER){
        ClockP_sleep(1);
        EnetAppUtils_print("Waiting for clock to Sync\r\n");
    }
    #endif

    TimerP_Params params;
    TimerP_Params_init(&params);
    params.periodInUsec = 50;
    TimerP_setup(QUERY_TASK_TIMER, &params);

    SemaphoreP_constructBinary(&gQuerySem, 0);

    CycleCounterP_reset();

    TimerP_start(QUERY_TASK_TIMER);


    for (int i = 1; i <= NUM_ITERATIONS; i++)
    {
        int32_t start_time = 0, end_time = 0;
        uint64_t fetch_time;

        SemaphoreP_pend(&gQuerySem, SystemP_WAIT_FOREVER);

        start_time = CycleCounterP_getCount32();

        #if defined(USE_FOLLOWERCLK)
        FollowerClock_getApproxtime(&gTimeSync_handle);
        #else
        EnetApp_getCurrentTimestamp();
        #endif

        end_time = CycleCounterP_getCount32();

        if(end_time < start_time){
            continue;
        }

        fetch_time = (end_time - start_time);

        total_ft += fetch_time;

        if(fetch_time>max_ft) max_ft = fetch_time;

        count++;

        if (i % stat_print_freq == 0)
        {
            float avg_ft = (float)total_ft/(count*CPU_CLK_FREQ_MHZ);
            EnetAppUtils_print("Iteration = %d | average fetch time = %f us | maximum fetch time = %llu \r\n", i, avg_ft,max_ft);
            total_ft = 0U;
            count = 0;
            max_ft = 0U;
        }
    }

    FollowerClock_deinit(&gTimeSync_handle);
    TaskP_exit();
    TaskP_destruct(&gQueryTask);
}

static uint8_t Enet_setup(){

    uint8_t status = ENET_SOK;

    Enet_MacPort macPortList[ENET_MAC_PORT_NUM];
    uint8_t numMacPorts;
    memset(&gEnetAppCfg,0,sizeof(gEnetAppCfg));

    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &gEnetAppCfg.enetType,&gEnetAppCfg.instId);

    EnetApp_getEnetInstMacInfo(gEnetAppCfg.enetType,gEnetAppCfg.instId,macPortList,&numMacPorts);

    EnetAppUtils_assert(numMacPorts == 1);

    gEnetAppCfg.macPort = macPortList[0];

    gEnetAppCfg.coreId = EnetSoc_getCoreId();

    EnetApp_driverInit();

    status = EnetApp_driverOpen(gEnetAppCfg.enetType, gEnetAppCfg.instId);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to open ENET: %d\r\n", status);
        return status;
    }

    EnetApp_HandleInfo handleInfo;
    EnetApp_acquireHandleInfo(gEnetAppCfg.enetType, gEnetAppCfg.instId, &handleInfo);
    gEnetAppCfg.hEnet = handleInfo.hEnet;

    EnetPer_AttachCoreOutArgs attachCoreOutArgs;
    EnetApp_coreAttach(gEnetAppCfg.enetType, gEnetAppCfg.instId, gEnetAppCfg.coreId, &attachCoreOutArgs);
    gEnetAppCfg.coreKey = attachCoreOutArgs.coreKey;
    return status;
}

void EnetApp_mainTask(void *args)
{
    TaskP_Params queryParams;

    EnetAppUtils_print("==========================\r\n");
    EnetAppUtils_print("    CLOCK SYNC EXAMPLE    \r\n");
    EnetAppUtils_print("==========================\r\n");

    uint8_t status = Enet_setup();
    if(status!=ENET_SOK)
    {
        EnetAppUtils_print("Failed to set up Enet\r\n");
        goto END;
    }

    #if defined(USE_FOLLOWERCLK)
    uint32_t waitTimeUS = 5000;
    FollowerClock_init(&gTimeSync_handle,TIMER_BASEADDR,PERIOD_IN_US,EnetApp_getHwPushEventTimestamp,waitTimeUS,0);

    EnetApp_setTimeSyncRouter();

    FollowerClock_startSyncTask(&gTimeSync_handle,gSyncStack,TASK_STACK_SIZE,SYNC_TASK_PRIORITY);

    #endif

    TaskP_Params_init(&queryParams);
    queryParams.name = "CPTS_query";
    queryParams.stackSize = sizeof(gQueryStack);
    queryParams.stack = gQueryStack;
    queryParams.priority = 3;
    queryParams.taskMain = CPTS_queryTask;
    TaskP_construct(&gQueryTask, &queryParams);

END:
    //waiting for the heat death of the universe while taking a nap
    while(1)
    {
        ClockP_sleep(1);
        EnetApp_printCpuLoad();
    }
}
