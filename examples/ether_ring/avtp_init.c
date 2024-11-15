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
#include <tsn_uniconf/yangs/yang_db_runtime.h>
#include <tsn_uniconf/yangs/yang_modules.h>
#include <tsn_l2/tilld/frtos_avtp_include.h>
#include <tsn_uniconf/ucman.h>
#include <tsn_uniconf/uc_dbal.h>
#include "common.h"
#include "debug_log.h"
#include "tsninit.h"
#include "est/est_configure.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define AVTPD_TASK_PRIORITY         (2)
#define MELCO_APP_CLASSD1_TASK_PRIORITY (10)

#define AVTPD_TASK_NAME         "avtpd_task"
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
#ifdef HAVE_GPTP_READY_NOTICE
extern CB_SEM_T g_gptpd_ready_semaphore;
#endif

extern EnetApp_Ctx_t gAppCtx;

/* ========================================================================== */
/*                            Local Variables                                */
/* ========================================================================== */
static uint8_t gAvtpdStackBuf[TSN_TSK_STACK_SIZE] \
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* AVTPD is always enabled once AVTP is supported */
static int EnetApp_addAvtpModCtx(EnetApp_ModuleCtx_t *modCtxTbl);

int EnetApp_avtpInit(EnetApp_ModuleCtx_t *modCtxTbl)
{
    return EnetApp_addAvtpModCtx(modCtxTbl);
}

static void *EnetApp_avtpdTask(void *arg)
{
//    char *argv[]={"avtpd", "-n", NULL};
    int timeout_ms = 3000;
    int res;

    res = uniconf_ready(NULL, UC_CALLMODE_THREAD, timeout_ms);
    if (res)
    {
        DPRINT("The uniconf must be run first !");
    }
    else
    {
//        AVTPD_MAIN(2, argv);
    }
    return NULL;
}

#define AVTPD_TASK_ENTRY \
    [ENETAPP_AVTPD_TASK_IDX]={ \
        .enable = BFALSE, \
        .stopFlag = BTRUE, \
        .taskPriority = AVTPD_TASK_PRIORITY, \
        .taskName = AVTPD_TASK_NAME, \
        .stackBuffer = gAvtpdStackBuf, \
        .stackSize = sizeof(gAvtpdStackBuf), \
        .onModuleDBInit = NULL, \
        .onModuleRunner = EnetApp_avtpdTask, \
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

#define AVTP_EST_CFG_ENTRY \
    [ENETAPP_MELCO_EST_CFG_IDX]={ \
        .enable = BTRUE, \
        .stopFlag = BTRUE, \
        .taskPriority = MELCO_APP_CLASSD1_TASK_PRIORITY, \
        .taskName = "melcoApp_EST", \
        .stackBuffer = gEstCfgStackBuf, \
        .stackSize = sizeof(gEstCfgStackBuf), \
        .onModuleDBInit = EnetApp_estInit, \
        .onModuleRunner = EnetApp_estConfigTask, \
        .appCtx = &gAppCtx \
    }

static int EnetApp_addAvtpModCtx(EnetApp_ModuleCtx_t *modCtxTbl)
{
    int i;

    EnetApp_ModuleCtx_t avtpMods[ENETAPP_MAX_TASK_IDX] =
    {
        AVTPD_TASK_ENTRY,
        AVTP_EST_CFG_ENTRY,
    };

    for (i = 0; i < ENETAPP_MAX_TASK_IDX; i++)
    {
        if (avtpMods[i].enable == BTRUE)
        {
            memcpy(&modCtxTbl[i], &avtpMods[i], sizeof(EnetApp_ModuleCtx_t));
        }
    }
    return 0;
}
