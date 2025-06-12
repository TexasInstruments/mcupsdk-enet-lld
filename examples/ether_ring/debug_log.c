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
#include "tsninit.h"
#include "debug_log.h"
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/SemaphoreP.h>

Logger_onConsoleOut sDrvConsoleOut;
#if TSN_USE_LOG_BUFFER == 1

static TaskP_Object gLoggerTask;
static SemaphoreP_Object gLogMutex;
static bool gLogTask_StopFlag = false;
static uint8_t gLogStackBuf[LOG_TASK_STACK_SIZE] \
        __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));
static uint8_t gLogBuf[DEBUG_LOG_BUFFER_SIZE];
static uint8_t gPrintBuf[DEBUG_LOG_BUFFER_SIZE];

static void Logger_task(void *arg)
{
    int len;

    while(1)
    {
        if (gLogTask_StopFlag == true)
        {
            break;
        }

        SemaphoreP_pend(&gLogMutex, SystemP_WAIT_FOREVER);
        len = strlen((const char*)gLogBuf);
        if (len > 0)
        {
            memcpy(gPrintBuf, gLogBuf, len);
            gLogBuf[0] = 0;
            gPrintBuf[len] = 0;
        }
        SemaphoreP_post(&gLogMutex);

        if (len > 0)
        {
            sDrvConsoleOut("%s", (char*)gPrintBuf);
        }

        CB_USLEEP(LOG_FLUSH_PERIOD_MSEC*UB_MSEC_US);
    }

    TaskP_exit();
}

int Logger_logToBuffer(bool flush, const char *str)
{
    int usedLen;
    int remainBufsize;
    int logLen = strlen(str);

    if (str[0] == 0)
    {
        return 0;
    }

    SemaphoreP_pend(&gLogMutex, SystemP_WAIT_FOREVER);
    usedLen = strlen((const char *)gLogBuf);
    remainBufsize = sizeof(gLogBuf)-usedLen;

#ifdef USE_CRLF
    char *lf = strrchr(str, '\n');
    bool replace = BFALSE;
    if (lf)
    {
        *lf = 0;
        replace = BTRUE;
    }
    if (remainBufsize > (logLen+2))
    {
        if(replace){
            snprintf((char *)&gLogBuf[usedLen], remainBufsize, "%s"ENDLINE, str);
        }else{
            snprintf((char *)&gLogBuf[usedLen], remainBufsize, "%s", str);
        }
    }
#else
    if (remainBufsize > logLen)
    {
        snprintf((char *)&gLogBuf[usedLen], remainBufsize, "%s", str);
    }
#endif
    else
    {
        snprintf((char *)&gLogBuf[0], sizeof(gLogBuf), "log ovflow!"ENDLINE);
    }
    SemaphoreP_post(&gLogMutex);

    return 0;
}

int Logger_init(Logger_onConsoleOut consoleOutCb)
{
    int status = SystemP_FAILURE;
    if (consoleOutCb != NULL)
    {
        sDrvConsoleOut = consoleOutCb;
        status = SystemP_SUCCESS;
    }

    if (status == SystemP_SUCCESS)
    {
        status = SemaphoreP_constructMutex(&gLogMutex);

        if (status == SystemP_SUCCESS)
        {
            TaskP_Params loggerTaskParams;
            TaskP_Params_init(&loggerTaskParams);

            loggerTaskParams.name      = "log_task";
            loggerTaskParams.stackSize = sizeof(gLogStackBuf);
            loggerTaskParams.stack     = gLogStackBuf;
            loggerTaskParams.priority  = LOG_TASK_PRIORITY;
            loggerTaskParams.taskMain  = Logger_task;
            loggerTaskParams.args      = NULL;

            status = TaskP_construct(&gLoggerTask, &loggerTaskParams);
        }
    }

    return status;
}

void Logger_deInit(void)
{
    gLogTask_StopFlag = true;
    CB_USLEEP(LOG_FLUSH_PERIOD_MSEC*UB_MSEC_US);
    TaskP_destruct(&gLoggerTask);
    sDrvConsoleOut = NULL;
}

#else /* TSN_USE_LOG_BUFFER */

int Logger_logToBuffer(bool flush, const char *str)
{
    /* Log to buffer is same as direct log here. */
    return Logger_directLog(flush, str);
}

int Logger_init(Logger_onConsoleOut consoleOutCb)
{
    int retval = -1;
    if (consoleOutCb != NULL)
    {
        sDrvConsoleOut = consoleOutCb;
        retval = 0;
    }
    return retval;
}

void Logger_deInit(void)
{
    sDrvConsoleOut = NULL;
    return;
}
#endif

int Logger_directLog(bool flush, const char *str)
{
    if (str[0] == 0)
    {
        return 0;
    }
#ifdef USE_CRLF
    // SITARA uses \r\n(CR,LF) for the new line so
    // we have to replace \n in the general log
    char *lf = strrchr(str, '\n');
    if (lf)
    {
        *lf = 0;
        flush = BTRUE;
    }
#endif
    sDrvConsoleOut((char*)str);
    if (flush)
    {
        sDrvConsoleOut(ENDLINE);
    }
    return 0;
}
