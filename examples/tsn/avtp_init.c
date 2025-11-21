/*
 *  Copyright (c) Texas Instruments Incorporated 2023
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
#include <tsn_uniconf/ucman.h>
#include <tsn_uniconf/uc_dbal.h>
#include "debug_log.h"
#include "tsninit.h"
#include "common.h"
#include "aes3_aaf/aes3_aaf_app.h"
#include "aafpcm_app/aaf_pcm_app.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define AVTPD_TASK_PRIORITY         (2)
#define TALKER_TASK_PRIORITY        (2)
#define LISTENER_TASK_PRIORITY      (2)
#define CRF_LISTENER_TASK_PRIORITY  (2)
#define CRF_TALKER_TASK_PRIORITY    (2)
#define ACF_TASK_PRIORITY           (2)
#define AAF_AES3_AAF_TASK_PRIORITY  (2)
#define AAF_PCM_TASK_PRIORITY       (2)
#define AVB_AUDIO_PLAYBACK_TASK_PRIORITY   (2)

#define AVB_AUDIO_PLAYBCK_TASK_NAME   "avb_audio_pb_task"

#define AVTPD_TASK_NAME         "avtpd_task"

#define TALKER_TASK_NAME        "talker_task"
#if AVTP_TALKER_NUM == 2
#define TALKER_TASK_NAME_2      "talker_task2"
#endif

#define LISTENER_TASK_NAME      "listener_task"
#if AVTP_LISTENER_NUM == 2
#define LISTENER_TASK_NAME_2    "listener_task2"
#endif

#define CRF_TALKER_TASK_NAME    "crf_talker_task"
#define CRF_LISTENER_TASK_NAME  "crf_listener_task"
#define ACF_TASK_NAME           "acf_task"

#define AAF_AES3_AAF_TASK_NAME  "aes3_aaf_task"
#define AAF_PCM_TASK_NAME       "aaf_pcm_task"

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
extern CB_SEM_T g_avtpd_ready_sem;
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
extern int AVTPD_MAIN(int argc, char *argv[]);
extern int avtp_testclient(int argc, char *argv[]);
extern int crf_task(int argc, char *argv[]);
extern int acf_testclient(int argc, char *argv[]);
extern int uc_dbal_setproc(uc_dbald *dbald, const char *name, int64_t pvalue);
static void *EnetApp_avtpdTask(void *arg)
{
    EnetApp_ModuleCtx_t *modCtx = (EnetApp_ModuleCtx_t *)arg;
    char *argv[]={"avtpd", "-n", NULL};
    int timeout_ms = 3000;
    int res;

    res = uniconf_ready(NULL, UC_CALLMODE_THREAD, timeout_ms);
    if (res)
    {
        DPRINT("The uniconf must be run first !");
    }
    else
    {
        int64_t tid=(int64_t)&modCtx->hTaskHandle;
        uc_dbal_setproc(ydbi_access_handle()->dbald, "l2", tid);
        AVTPD_MAIN(2, argv);
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

#ifdef HAVE_GPTP_READY_NOTICE
static void waitGptpReady()
{
    int gptpReadyCounter = 0;
    DPRINT("Waiting for GPTP ready!!\n");
    while(BTRUE)
    {
        if (g_gptpd_ready_semaphore != NULL)
        {
            CB_SEM_GETVALUE(&g_gptpd_ready_semaphore, &gptpReadyCounter);
            if (gptpReadyCounter > 0)
            {
                DPRINT("GPTP ready!!\n");
                break;
            }
        }
        CB_USLEEP(100000);
    }
}
#endif

/*-----------For AVTP talker test applications------------*/
#ifdef AVTP_TALKER_ENABLED

static void *EnetApp_runAvtpTalker(EnetApp_ModuleCtx_t *mdctx, char *stream_id)
{
    EnetApp_Ctx_t *ctx = mdctx->appCtx;
    int64_t tid=(int64_t)&mdctx->hTaskHandle;
    uc_dbal_setproc(ydbi_access_handle()->dbald, "l2", tid);
    char *argv[]={"avtp_testclient", "-d", &ctx->netdev[0][0],
        "-m", "t", "-B", "1000", "-v", "110", "-C", "-c",
        "-S", stream_id, "-b", "5", "-i", "-u", NULL}; /* '-u' must be the last opt */

    DPRINT("avtp_testclient:talker sid=%s start, tid=%" PRId64 "", stream_id, tid);
    avtp_testclient(GetArgc(argv), argv);
    return NULL;
}

static void *EnetApp_talkerTask(void *arg)
{
    return EnetApp_runAvtpTalker(arg, "00:01:02:03:04:05:00:00");
}

static uint8_t gTalkerStackBuf[TSN_TSK_STACK_SIZE] \
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

#if AVTP_TALKER_NUM == 2
static void *EnetApp_talkerTask2(void *arg)
{
    return EnetApp_runAvtpTalker(arg, "00:01:02:03:04:05:00:01");
}

static uint8_t gTalkerStackBuf2[TSN_TSK_STACK_SIZE] \
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

#define AVTP_TALKER_ENTRY \
    [ENETAPP_TALKER_TASK_IDX]={ \
        .enable = BTRUE, \
        .stopFlag = BTRUE, \
        .taskPriority = TALKER_TASK_PRIORITY, \
        .taskName = TALKER_TASK_NAME, \
        .stackBuffer = gTalkerStackBuf, \
        .stackSize = sizeof(gTalkerStackBuf), \
        .onModuleDBInit = NULL, \
        .onModuleRunner = EnetApp_talkerTask, \
        .appCtx = &gAppCtx \
    }, \
    [ENETAPP_TALKER_TASK_IDX + 1]={ \
        .enable = BTRUE, \
        .stopFlag = BTRUE, \
        .taskPriority = TALKER_TASK_PRIORITY, \
        .taskName = TALKER_TASK_NAME_2, \
        .stackBuffer = gTalkerStackBuf2, \
        .stackSize = sizeof(gTalkerStackBuf2), \
        .onModuleDBInit = NULL, \
        .onModuleRunner = EnetApp_talkerTask2, \
        .appCtx = &gAppCtx \
    }
#else //AVTP_TALKER_NUM!=2
#define AVTP_TALKER_ENTRY \
    [ENETAPP_TALKER_TASK_IDX]={ \
        .enable = BTRUE, \
        .stopFlag = BTRUE, \
        .taskPriority = TALKER_TASK_PRIORITY, \
        .taskName = TALKER_TASK_NAME, \
        .stackBuffer = gTalkerStackBuf, \
        .stackSize = sizeof(gTalkerStackBuf), \
        .onModuleDBInit = NULL, \
        .onModuleRunner = EnetApp_talkerTask, \
        .appCtx = &gAppCtx \
    }
#endif  //AVTP_TALKER_NUM
#endif //AVTP_TALKER_ENABLED

/*-----------For AVTP listener test applications------------*/
#ifdef AVTP_LISTENER_ENABLED
static void *EnetApp_runAvtpListener(EnetApp_ModuleCtx_t *mdctx, char *stream_id)
{
    /* To have the -B 10000 works, CB_NOIPCSHMEM_DFNUM=200 is needed */
    EnetApp_Ctx_t *ctx = mdctx->appCtx;
    int64_t tid=(int64_t)&mdctx->hTaskHandle;
    uc_dbal_setproc(ydbi_access_handle()->dbald, "l2", tid);
    char *argv[]={"avtp_testclient", "-d", &ctx->netdev[0][0],
        "-m", "l", "-B", "10000", "-v", "110", "-C", "-c", "-F", "-N",
        "-S", stream_id, "-i", "-u", NULL}; /* '-u' must be the last opt */

    DPRINT("avtp_testclient:listener sid=%s, tid=%" PRId64 "", stream_id, tid);
    avtp_testclient(GetArgc(argv), argv);
    return NULL;
}

static void *EnetApp_listenerTask(void *arg)
{
    return EnetApp_runAvtpListener(arg, "00:01:02:03:04:05:00:02");
}

static uint8_t gListenerStackBuf[TSN_TSK_STACK_SIZE] \
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

#if AVTP_LISTENER_NUM == 2
static void *EnetApp_listenerTask2(void *arg)
{
    return EnetApp_runAvtpListener(arg, "00:01:02:03:04:05:00:03");
}
static uint8_t gListenerStackBuf2[TSN_TSK_STACK_SIZE] \
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

#define AVTP_LISTENER_ENTRY \
    [ENETAPP_LISTENER_TASK_IDX]={ \
        .enable = BTRUE, \
        .stopFlag = BTRUE, \
        .taskPriority = LISTENER_TASK_PRIORITY, \
        .taskName = LISTENER_TASK_NAME, \
        .stackBuffer = gListenerStackBuf, \
        .stackSize = sizeof(gListenerStackBuf), \
        .onModuleDBInit = NULL, \
        .onModuleRunner = EnetApp_listenerTask, \
        .appCtx = &gAppCtx \
    }, \
    [ENETAPP_LISTENER_TASK_IDX + 1]={ \
        .enable = BTRUE, \
        .stopFlag = BTRUE, \
        .taskPriority = LISTENER_TASK_PRIORITY, \
        .taskName = LISTENER_TASK_NAME_2, \
        .stackBuffer = gListenerStackBuf2, \
        .stackSize = sizeof(gListenerStackBuf2), \
        .onModuleDBInit = NULL, \
        .onModuleRunner = EnetApp_listenerTask2, \
        .appCtx = &gAppCtx \
    }
#else //AVTP_LISTENER_NUM != 2
#define AVTP_LISTENER_ENTRY \
    [ENETAPP_LISTENER_TASK_IDX]={ \
        .enable = BTRUE, \
        .stopFlag = BTRUE, \
        .taskPriority = LISTENER_TASK_PRIORITY, \
        .taskName = LISTENER_TASK_NAME, \
        .stackBuffer = gListenerStackBuf, \
        .stackSize = sizeof(gListenerStackBuf), \
        .onModuleDBInit = NULL, \
        .onModuleRunner = EnetApp_listenerTask, \
        .appCtx = &gAppCtx \
    }
#endif //AVTP_LISTENER_NUM
#endif //AVTP_LISTENER_ENABLED

/*-----------For AVTP CRF talker test applications------------*/
#ifdef AVTP_CRF_TALKER_ENABLED

static void *EnetApp_runCrfTalker(EnetApp_ModuleCtx_t *mdctx, char *stream_id)
{
    EnetApp_Ctx_t *ctx = mdctx->appCtx;
    int64_t tid=(int64_t)&mdctx->hTaskHandle;
    uc_dbal_setproc(ydbi_access_handle()->dbald, "l2", tid);
    char *argv[]={"crf_testclient", "-d", &ctx->netdev[0][0],
        "-m", "t", "-v", "110", "-s", stream_id, "-i", "-u", NULL}; /* '-u' must be the last */

    DPRINT("crf_testclient:talker sid=%s, tid=%" PRId64 "", stream_id, tid);
    crf_task(GetArgc(argv), argv);
    return NULL;
}

static void *EnetApp_crfTalkerTask(void *arg)
{
    return EnetApp_runCrfTalker(arg, "00:01:02:03:04:05:00:04");
}

static uint8_t gCrfTalkerStackBuf[TSN_TSK_STACK_SIZE] \
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

#define AVTP_CRF_TALKER_ENTRY \
    [ENETAPP_CRF_TALKER_TASK_IDX]={ \
        .enable = BTRUE, \
        .stopFlag = BTRUE, \
        .taskPriority = CRF_TALKER_TASK_PRIORITY, \
        .taskName = CRF_TALKER_TASK_NAME, \
        .stackBuffer = gCrfTalkerStackBuf, \
        .stackSize = sizeof(gCrfTalkerStackBuf), \
        .onModuleDBInit = NULL, \
        .onModuleRunner = EnetApp_crfTalkerTask, \
        .appCtx = &gAppCtx \
    }

#endif //AVTP_CRF_TALKER_ENABLED

/*-----------For AVTP CRF listener test applications------------*/
#ifdef AVTP_CRF_LISTENER_ENABLED

static void *EnetApp_runCrfListener(EnetApp_ModuleCtx_t *mdctx, char *stream_id)
{
    EnetApp_Ctx_t *ctx = mdctx->appCtx;
    int64_t tid=(int64_t)&mdctx->hTaskHandle;
    uc_dbal_setproc(ydbi_access_handle()->dbald, "l2", tid);
    char *argv[]={"crf_testclient", "-d", &ctx->netdev[0][0],
        "-m", "l", "-v", "110", "-s", stream_id, "-i", "-u", NULL}; /* '-u' must be the last */

    DPRINT("crf_testclient:listener sid=%s, tid=%" PRId64 "", stream_id, tid);
    crf_task(GetArgc(argv), argv);
    return NULL;
}

static void *EnetApp_crfListenerTask(void *arg)
{
    return EnetApp_runCrfListener(arg, "00:01:02:03:04:05:00:04");
}

static uint8_t gCrfListenerStackBuf[TSN_TSK_STACK_SIZE] \
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

#define AVTP_CRF_LISTENER_ENTRY \
    [ENETAPP_CRF_LISTENER_TASK_IDX]={ \
        .enable = BTRUE, \
        .stopFlag = BTRUE, \
        .taskPriority = CRF_LISTENER_TASK_PRIORITY, \
        .taskName = CRF_LISTENER_TASK_NAME, \
        .stackBuffer = gCrfListenerStackBuf, \
        .stackSize = sizeof(gCrfListenerStackBuf), \
        .onModuleDBInit = NULL, \
        .onModuleRunner = EnetApp_crfListenerTask, \
        .appCtx = &gAppCtx \
    }

#endif //AVTP_CRF_LISTENER_ENABLED

#ifdef AVTP_ACF_ENABLED

static void *EnetApp_runAcf(EnetApp_ModuleCtx_t *mdctx, char *stream_id)
{
    EnetApp_Ctx_t *ctx = mdctx->appCtx;
    int64_t tid=(int64_t)&mdctx->hTaskHandle;
    uc_dbal_setproc(ydbi_access_handle()->dbald, "l2", tid);
    char *argv[]={"acf_testclient", "-d", &ctx->netdev[0][0],
        "-m", "t", "-v", "110", "-s", stream_id, "-I", "-R", "-n", NULL}; /* '-n' must be the last */

    DPRINT("acf_testclient:sid=%s, tid=%" PRId64 "", stream_id, tid);
    acf_testclient(GetArgc(argv), argv);
    return NULL;
}

static void *EnetApp_acfTask(void *arg)
{
    return EnetApp_runAcf(arg, "00:01:02:03:04:05:00:06");
}

static uint8_t gAcfStackBuf[TSN_TSK_STACK_SIZE] \
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

#define AVTP_ACF_ENTRY \
    [ENETAPP_ACF_TASK_IDX]={ \
        .enable = BTRUE, \
        .stopFlag = BTRUE, \
        .taskPriority = ACF_TASK_PRIORITY, \
        .taskName = ACF_TASK_NAME, \
        .stackBuffer = gAcfStackBuf, \
        .stackSize = sizeof(gAcfStackBuf), \
        .onModuleDBInit = NULL, \
        .onModuleRunner = EnetApp_acfTask, \
        .appCtx = &gAppCtx \
    }
#endif //AVTP_ACF_ENABLED

#ifdef AES3_AAF_ENABLED
static void *EnetApp_aes3_aafTask(void *arg)
{
#ifndef AVTP_DIRECT_MODE
    // loop forever
    WAIT_AVTPD_READY;
#else
#ifdef HAVE_GPTP_READY_NOTICE
    waitGptpReady();
#endif // HAVE_GPTP_READY_NOTICE
#endif // AVTP_DIRECT_MODE
    EnetApp_ModuleCtx_t *modCtx = (EnetApp_ModuleCtx_t *)arg;
    int64_t tid=(int64_t)&modCtx->hTaskHandle;
    uc_dbal_setproc(ydbi_access_handle()->dbald, "l2", tid);
#ifdef AES3_AAF_TALKER_ENABLE
    start_aes3_aaf_talker("tilld0");
#else
    start_aes3_aaf_listener("tilld0");
#endif
    return NULL;
}

static uint8_t gAes3aafStackBuf[TSN_TSK_STACK_SIZE] \
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

#define AVTP_AES3_AAF_ENTRY \
    [ENETAPP_AES3_AAF_TASK_IDX]={ \
        .enable = BTRUE, \
        .stopFlag = BTRUE, \
        .taskPriority = AAF_AES3_AAF_TASK_PRIORITY, \
        .taskName = AAF_AES3_AAF_TASK_NAME, \
        .stackBuffer = gAes3aafStackBuf, \
        .stackSize = sizeof(gAes3aafStackBuf), \
        .onModuleDBInit = NULL, \
        .onModuleRunner = EnetApp_aes3_aafTask, \
        .appCtx = &gAppCtx \
    }
#endif // AES3_AAF_ENABLED

#ifdef AAF_PCM_ENABLED
static void *EnetApp_aafpcmTask(void *arg)
{
#ifndef AVTP_DIRECT_MODE
    // loop forever
    WAIT_AVTPD_READY;
#else
#ifdef HAVE_GPTP_READY_NOTICE
    waitGptpReady();
#endif // HAVE_GPTP_READY_NOTICE
#endif // AVTP_DIRECT_MODE

    EnetApp_ModuleCtx_t *modCtx = (EnetApp_ModuleCtx_t *)arg;
    int64_t tid=(int64_t)&modCtx->hTaskHandle;
    uc_dbal_setproc(ydbi_access_handle()->dbald, "l2", tid);
#ifdef AAF_PCM_TALKER_ENABLE
    start_aaf_pcm_talker("tilld0");
#else
    start_aaf_pcm_listener("tilld0");
#endif
    return NULL;
}

static uint8_t gAafPcmStackBuf[TSN_TSK_STACK_SIZE] \
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

#define AVTP_AAF_PCM_ENTRY \
    [ENETAPP_AAF_PCM_TASK_IDX]={ \
        .enable = BTRUE, \
        .stopFlag = BTRUE, \
        .taskPriority = AAF_PCM_TASK_PRIORITY, \
        .taskName = AAF_PCM_TASK_NAME, \
        .stackBuffer = gAafPcmStackBuf, \
        .stackSize = sizeof(gAafPcmStackBuf), \
        .onModuleDBInit = NULL, \
        .onModuleRunner = EnetApp_aafpcmTask, \
        .appCtx = &gAppCtx \
    }
#endif // AAF_PCM_ENABLED

#ifdef AVB_AUDIO_PLAYBACK_DEMO
static uint8_t gAvbAudioPlaybackTaskStack[TSN_TSK_STACK_SIZE] \
                    __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));
#define AVB_AUDIO_PLAYBACK_DEMO_ENTRY \
    [ENETAPP_AVB_AUDIO_PLAYBACK_DEMO_IDX]={ \
        .enable = BTRUE, \
        .stopFlag = BTRUE, \
        .taskPriority = AVB_AUDIO_PLAYBACK_TASK_PRIORITY, \
        .taskName = AVB_AUDIO_PLAYBCK_TASK_NAME, \
        .stackBuffer = gAvbAudioPlaybackTaskStack, \
        .stackSize = sizeof(gAvbAudioPlaybackTaskStack), \
        .onModuleDBInit = NULL, \
        .onModuleRunner = EnetApp_AvbAudioPlaybackTask, \
        .appCtx = &gAppCtx \
    }

void EnetApp_AudioPlaybackDemoMain(void* args);

static void* EnetApp_AvbAudioPlaybackTask(void* args)
{
    EnetApp_ModuleCtx_t *modCtx = (EnetApp_ModuleCtx_t *)args;
    int64_t tid = (int64_t)&modCtx->hTaskHandle;
    uc_dbal_setproc(ydbi_access_handle()->dbald, "l2", tid);

    EnetApp_AudioPlaybackDemoMain(NULL);

    return NULL;
}

#endif /* AVB_AUDIO_PLAYBACK_DEMO */

static int EnetApp_addAvtpModCtx(EnetApp_ModuleCtx_t *modCtxTbl)
{
    int i;

    EnetApp_ModuleCtx_t avtpMods[ENETAPP_MAX_TASK_IDX] =
    {
        AVTPD_TASK_ENTRY,
#ifdef AVTP_TALKER_ENABLED
        AVTP_TALKER_ENTRY,
#endif
#ifdef AVTP_LISTENER_ENABLED
        AVTP_LISTENER_ENTRY,
#endif
#ifdef AVTP_CRF_TALKER_ENABLED
        AVTP_CRF_TALKER_ENTRY,
#endif
#ifdef AVTP_CRF_LISTENER_ENABLED
        AVTP_CRF_LISTENER_ENTRY,
#endif
#ifdef AVTP_ACF_ENABLED
        AVTP_ACF_ENTRY,
#endif
#ifdef AES3_AAF_ENABLED
        AVTP_AES3_AAF_ENTRY,
#endif
#ifdef AAF_PCM_ENABLED
        AVTP_AAF_PCM_ENTRY,
#endif
#ifdef AVB_AUDIO_PLAYBACK_DEMO
        AVB_AUDIO_PLAYBACK_DEMO_ENTRY,
#endif
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

int EnetApp_avtpInit(EnetApp_ModuleCtx_t *modCtxTbl)
{
    if (g_avtpd_ready_sem != NULL)
    {
        return 0;
    }
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
