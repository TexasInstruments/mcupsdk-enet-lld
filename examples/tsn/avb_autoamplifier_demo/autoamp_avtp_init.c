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

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include <tsn_combase/combase.h>
#include <tsn_unibase/unibase_binding.h>
#include <tsn_uniconf/yangs/yang_modules.h>
#include <tsn_l2/tilld/frtos_avtp_include.h>
#include "tsn_gptp/gptpmasterclock.h"
#include <tsn_uniconf/ucman.h>
#include <tsn_uniconf/uc_dbal.h>
#include "debug_log.h"
#include "tsninit.h"
#include "common.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern EnetApp_Ctx_t gAppCtx;

extern CB_SEM_T g_avtpd_ready_sem;

static uint8_t gAutoampTxDemoTask[TSN_TSK_STACK_SIZE] \
                                    __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

static uint8_t gAutoampRxDemoTask[TSN_TSK_STACK_SIZE] \
                                    __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

#if (AVTP_CRF_TALKER_ENABLED || AVTP_CRF_LISTENER_ENABLED)
static uint8_t gCrfTaskStack[TSN_TSK_STACK_SIZE] \
                                    __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));
#endif

/* ========================================================================== */
/*                            Local Variables                                */
/* ========================================================================== */

void start_all_talkers(void);

void start_aaf_pcm_listener(char *netdev);

int crf_task(int argc, char *argv[]);

int uc_dbal_setproc(uc_dbald *dbald, const char *name, int64_t pvalue);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void* EnetApp_AutoampTxTask(void* args)
{
    EnetApp_ModuleCtx_t *mdctx = (EnetApp_ModuleCtx_t *)args;
    int64_t tid = (int64_t)&mdctx->hTaskHandle;
    uc_dbal_setproc(ydbi_access_handle()->dbald, "l2", tid);
    start_all_talkers();
    return NULL;
}

static void* EnetApp_AutoampRxTask(void* args)
{
    EnetApp_ModuleCtx_t *mdctx = (EnetApp_ModuleCtx_t *)args;
    int64_t tid = (int64_t)&mdctx->hTaskHandle;
    uc_dbal_setproc(ydbi_access_handle()->dbald, "l2", tid);
    start_aaf_pcm_listener(NULL);
    return NULL;
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

static void *EnetApp_runCrfTask(void* args)
{
    EnetApp_ModuleCtx_t *mdctx = (EnetApp_ModuleCtx_t *)args;
    EnetApp_Ctx_t *ctx = mdctx->appCtx;
    int64_t tid = (int64_t)&mdctx->hTaskHandle;
    char* streamId = "00:01:02:03:04:05:00:05";
    uc_dbal_setproc(ydbi_access_handle()->dbald, "l2", tid);
#if AVTP_CRF_LISTENER_ENABLED
    char *argv[]={"crf_testclient", "-d", &ctx->netdev[0][0],
        "-m", "l", "-v", "110", "-s", streamId, "-i", "-u", NULL}; /* '-u' must be the last */
    DebugP_log("crf_testclient:listener sid=%s, tid=%" PRId64 "", streamId, tid);
#else
    char *argv[]={"crf_testclient", "-d", &ctx->netdev[0][0],
        "-m", "t", "-v", "110", "-s", streamId, "-i", "-u", NULL}; /* '-u' must be the last */
    DebugP_log("crf_testclient:talker sid=%s, tid=%" PRId64 "", streamId, tid);
#endif
    crf_task(GetArgc(argv), argv);

    return NULL;
}

static int EnetApp_addAvtpModCtx(EnetApp_ModuleCtx_t *modCtxTbl)
{
    EnetApp_ModuleCtx_t avtpMods[ENETAPP_MAX_TASK_IDX] =
    {
        [ENETAPP_AAF_AUTOAMP_APP_TX_CLASSA_TASK_IDX]={
                    .enable = BTRUE,
                    .stopFlag = BTRUE,
                    .taskPriority = 10,
                    .taskName = "Tx Task",
                    .stackBuffer = gAutoampTxDemoTask,
                    .stackSize = sizeof(gAutoampTxDemoTask),
                    .onModuleDBInit = NULL,
                    .onModuleRunner = EnetApp_AutoampTxTask,
                    .appCtx = &gAppCtx,
        },
        [ENETAPP_AUTOAMP_APP_RX_TASK_IDX]={
                    .enable = BTRUE,
                    .stopFlag = BTRUE,
                    .taskPriority = 10,
                    .taskName = "Rx Task",
                    .stackBuffer = gAutoampRxDemoTask,
                    .stackSize = sizeof(gAutoampRxDemoTask),
                    .onModuleDBInit = NULL,
                    .onModuleRunner = EnetApp_AutoampRxTask,
                    .appCtx = &gAppCtx,
        },
#if AVTP_CRF_TALKER_ENABLED
        [ENETAPP_CRF_TALKER_TASK_IDX]={ \
                    .enable = BTRUE, \
                    .stopFlag = BTRUE, \
                    .taskPriority = 10, \
                    .taskName = "CRF Talker Task", \
                    .stackBuffer = gCrfTaskStack, \
                    .stackSize = sizeof(gCrfTaskStack), \
                    .onModuleDBInit = NULL, \
                    .onModuleRunner = EnetApp_runCrfTask, \
                    .appCtx = &gAppCtx \
        }
#if AVTP_CRF_LISTENER_ENABLED
#error "Error: Enabled both CRF Talker and Listener"
#endif

#endif
#if AVTP_CRF_LISTENER_ENABLED
        [ENETAPP_CRF_LISTENER_TASK_IDX]={ \
                    .enable = BTRUE, \
                    .stopFlag = BTRUE, \
                    .taskPriority = 10, \
                    .taskName = "CRF Listener Task", \
                    .stackBuffer = gCrfTaskStack, \
                    .stackSize = sizeof(gCrfTaskStack), \
                    .onModuleDBInit = NULL, \
                    .onModuleRunner = EnetApp_runCrfTask, \
                    .appCtx = &gAppCtx \
        }
#if AVTP_CRF_TALKER_ENABLED
#error "Error: Enabled both CRF Talker and Listener"
#endif

#endif
    };

    for (int i = 0; i < ENETAPP_MAX_TASK_IDX; i++)
    {
        if (avtpMods[i].enable == BTRUE)
        {
            memcpy(&modCtxTbl[i], &avtpMods[i], sizeof(EnetApp_ModuleCtx_t));
        }
    }
    return 0;
}

int EnetApp_avtpInit(EnetApp_ModuleCtx_t *modCtxTbl)
{
    if (CB_SEM_INIT(&g_avtpd_ready_sem, 0, 0) < 0)
    {
        DPRINT("Failed to initialize g_avtpd_ready_sem!");
        return -1;
    }
    return EnetApp_addAvtpModCtx(modCtxTbl);
}

void EnetApp_avtpDeinit(void)
{
    if (g_avtpd_ready_sem != NULL)
    {
        CB_SEM_DESTROY(&g_avtpd_ready_sem);
        g_avtpd_ready_sem = NULL;
    }
}
