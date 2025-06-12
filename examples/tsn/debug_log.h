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

#ifndef __DEBUG_LOG_H__
#define __DEBUG_LOG_H__

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "tsnapp_porting.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Writing log directly to the console can impact the performance.
 * So by the default the log will be written to the buffer and then a log task
 * will print to the console later. */
#ifndef TSN_USE_LOG_TASK
#define TSN_USE_LOG_TASK 1
#endif

#if TSN_USE_LOG_TASK
#define TSN_USE_LOG_BUFFER     1
#define LOG_UART_BAUD          (115200)
#define MAX_BYTES_OUT_PER_SEC  (LOG_UART_BAUD/8)
#define LOG_FLUSH_PERIOD_MSEC  (10)
#define DEBUG_LOG_BUFFER_SIZE  (2*MAX_BYTES_OUT_PER_SEC* \
                                LOG_FLUSH_PERIOD_MSEC/1000)
#define LOG_TASK_STACK_SIZE    (5*1024)
#define LOG_TASK_PRIORITY      (3)
#define LOG_OUTPUT Logger_logToBuffer
#else
#define LOG_OUTPUT Logger_directLog
#endif

#define DPRINT(str,...) sDrvConsoleOut(str ENDLINE, ##__VA_ARGS__)

typedef void (*Logger_onConsoleOut)(const char *str, ...);

extern Logger_onConsoleOut sDrvConsoleOut;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Initializes the logger module.
 *
 * This function sets up the logger task
 *
 * @param consoleOutCb Callback function to handle console output.
 * @return Returns 0 on success, or a negative error code on failure.
 */
int Logger_init(Logger_onConsoleOut consoleOutCb);
/**
 * @brief Logs a message to an internal buffer.
 *
 * @param flush If true, flushes the buffer after logging.
 * @param str The message string to log.
 * @return Status code indicating success or failure.
 */
int Logger_logToBuffer(bool flush, const char *str);

/**
 * @brief Logs a message directly, bypassing the buffer.
 *
 * @param flush If true, flushes the output after logging.
 * @param str The message string to log.
 * @return Status code indicating success or failure.
 */
int Logger_directLog(bool flush, const char *str);

/**
 * @brief Deinitializes the logger
 */
void Logger_deInit(void);

#endif
