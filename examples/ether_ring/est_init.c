/*
 *  Copyright (c) Texas Instruments Incorporated 2024
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

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include <tsn_combase/combase.h>
#include <tsn_unibase/unibase_binding.h>
#include <tsn_uniconf/yangs/yang_modules.h>
// #include <tsn_l2/tilld/frtos_avtp_include.h>
#include <tsn_uniconf/ucman.h>
#include <tsn_uniconf/uc_dbal.h>
#include "common.h"
#include "debug_log.h"
#include "tsninit.h"
#include "est/est_configure.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define TSND_TASK_PRIORITY          (2)
#define EST_TASK_PRIORITY           (10)

#define TSND_TASK_NAME         "tsnd_task"
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
extern CB_SEM_T g_gptpd_ready_semaphore;
extern EnetApp_Ctx_t gAppCtx;

/* ========================================================================== */
/*                            Local Variables                                */
/* ========================================================================== */
static uint8_t gTsndStackBuf[TSN_TSK_STACK_SIZE] \
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void *EnetApp_tsnTask(void *arg)
{
    int timeout_ms = 3000;
    int res;

    res = uniconf_ready(NULL, UC_CALLMODE_THREAD, timeout_ms);
    if (res)
    {
        DPRINT("The uniconf must be run first !");
    }
    return NULL;
}

#define TSND_TASK_ENTRY \
    [ENETAPP_TSND_TASK_IDX]={ \
        .enable = BFALSE, \
        .stopFlag = BTRUE, \
        .taskPriority = TSND_TASK_PRIORITY, \
        .taskName = TSND_TASK_NAME, \
        .stackBuffer = gTsndStackBuf, \
        .stackSize = sizeof(gTsndStackBuf), \
        .onModuleDBInit = NULL, \
        .onModuleRunner = EnetApp_tsnTask, \
        .appCtx = &gAppCtx \
    }

static int GetArgc(char *argv[])
{
    int argc = 0;

    while (argv[argc] != NULL)
    {
        argc++;
    }
    return argc;
}

extern void est_schedule(EnetApp_ModuleCtx_t *modCtx);
static void *EnetApp_estConfigTask(void *arg)
{
    EnetApp_ModuleCtx_t *modCtx = (EnetApp_ModuleCtx_t *)arg;
    est_schedule(modCtx);

    TaskP_exit();
    return NULL;
}

static int EnetApp_estInit(EnetApp_ModuleCtx_t* modCtx, EnetApp_dbArgs *dbargs)
{
    init_est();
    return 0;
}

static uint8_t gEstCfgStackBuf[TSN_TSK_STACK_SIZE] \
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

#define EST_CFG_ENTRY \
    [ENETAPP_EST_CFG_IDX]={ \
        .enable = BTRUE, \
        .stopFlag = BTRUE, \
        .taskPriority = EST_TASK_PRIORITY, \
        .taskName = "task_EST", \
        .stackBuffer = gEstCfgStackBuf, \
        .stackSize = sizeof(gEstCfgStackBuf), \
        .onModuleDBInit = EnetApp_estInit, \
        .onModuleRunner = EnetApp_estConfigTask, \
        .appCtx = &gAppCtx \
    }

int EnetApp_addTsnModCtx(EnetApp_ModuleCtx_t *modCtxTbl)
{
    int i;

    EnetApp_ModuleCtx_t tsnMods[ENETAPP_MAX_TASK_IDX] =
    {
        TSND_TASK_ENTRY,
        EST_CFG_ENTRY,
    };

    for (i = 0; i < ENETAPP_MAX_TASK_IDX; i++)
    {
        if (tsnMods[i].enable == BTRUE)
        {
            memcpy(&modCtxTbl[i], &tsnMods[i], sizeof(EnetApp_ModuleCtx_t));
        }
    }
    return 0;
}
