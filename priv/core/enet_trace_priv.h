/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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
 * \file  enet_trace_priv.h
 *
 * \brief This file contains private type definitions and helper macros for the
 *        Enet trace interface.
 */

#ifndef ENET_TRACE_PRIV_H_
#define ENET_TRACE_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_types.h>
#include <include/core/enet_trace.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*!
 * \brief Log a trace message if log level is enabled
 *
 * Log trace messages for log levels that are enabled at runtime.
 *
 * \param globalLevel    Trace module global level
 * \param level          Trace level intended to be logged
 * \param fmt            Print string
 */
void EnetTrace_trace(EnetTrace_TraceLevel globalLevel,
                     EnetTrace_TraceLevel level,
                     const char *fmt,
                     ...);

/*! \brief Trace prefix type */
#if (ENET_CFG_TRACE_LEVEL > ENET_CFG_TRACE_LEVEL_NONE)
#  if (ENET_CFG_TRACE_TRACE_FORMAT == ENET_CFG_TRACE_FORMAT_FUNC)
#    if ENET_CFG_IS_ON(TRACE_DISABLE_INFOSTRING)
/* Trace prefix: "<func>: fmt" */
#define ENETTRACE_trace(globalLevel, level, fmt, ...)        \
    EnetTrace_trace((globalLevel), (level),                  \
                    "%s:%d \r\n" ,                             \
                    __func__,__LINE__)
#    else
#define ENETTRACE_trace(globalLevel, level, fmt, ...)        \
    EnetTrace_trace((globalLevel), (level),                  \
                    "%s: " fmt,                              \
                    __func__, ## __VA_ARGS__)
#    endif
#  endif
#  if (ENET_CFG_TRACE_TRACE_FORMAT == ENET_CFG_TRACE_FORMAT_FILE)
#    if ENET_CFG_IS_ON(TRACE_DISABLE_INFOSTRING)
/* Trace prefix: "<file>: <line>: fmt" */
#define ENETTRACE_trace(globalLevel, level, fmt, ...)        \
    EnetTrace_trace((globalLevel), (level),                  \
                    "%s: %d: \r\n",                          \
                    __FILE__, __LINE__)
#    else
#define ENETTRACE_trace(globalLevel, level, fmt, ...)        \
                    "%s: %d: " fmt,                          \
                    __FILE__, __LINE__, ## __VA_ARGS__)
#    endif
#  endif
#  if (ENET_CFG_TRACE_TRACE_FORMAT == ENET_CFG_TRACE_FORMAT_FULL)
#      if ENET_CFG_IS_ON(TRACE_DISABLE_INFOSTRING)
/* Trace prefix: "<file>: <line>: <func>: fmt" */
#define ENETTRACE_trace(globalLevel, level, fmt, ...)        \
    EnetTrace_trace((globalLevel), (level),                  \
                    "%s: %d: %s: \r\n" ,                     \
                    __FILE__, __LINE__, __func__)
#    else
#define ENETTRACE_trace(globalLevel, level, fmt, ...)       \
     EnetTrace_trace((globalLevel), (level),                 \
                    "%s: %d: %s: " fmt,                      \
                    __FILE__, __LINE__, __func__, ## __VA_ARGS__)
#    endif
#  endif
#else /* ENET_CFG_TRACE_LEVEL_NONE */
#define ENETTRACE_trace(globalLevel, level, fmt, ...)
#endif

/*!
 * \brief Helper macro to add trace message with #ENET_TRACE_ERROR level.
 */
#if (ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_ERROR)
#define ENETTRACE_ERR(fmt, ...) ENETTRACE_trace(gEnetTrace_runtimeLevel, \
                                                ENET_TRACE_ERROR,        \
                                                fmt "\r",                \
                                                ## __VA_ARGS__)
/* TODO: Replace the below #ifdef with a method applicable for both C & C++ */
/*!
 * \brief Helper macro to add trace message with #ENET_TRACE_ERROR level if
 *        a condition is met.
 */
#ifdef __cplusplus
#define ENETTRACE_ERR_IF(cond, ...) ((cond) ? ENETTRACE_ERR(__VA_ARGS__) : void())
#else
#define ENETTRACE_ERR_IF(cond, ...) ((cond) ? ENETTRACE_ERR(__VA_ARGS__) : 0U)
#endif
#else
#define ENETTRACE_ERR(fmt, ...)
#define ENETTRACE_ERR_IF(cond, ...)
#endif

/*!
 * \brief Helper macro to add trace message with #ENET_TRACE_WARN level
 */
#if (ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_WARN)
#define ENETTRACE_WARN(fmt, ...) ENETTRACE_trace(gEnetTrace_runtimeLevel, \
                                                 ENET_TRACE_WARN,         \
                                                 fmt "\r",                \
                                                 ## __VA_ARGS__)
/*!
 * \brief Helper macro to add trace message with #ENET_TRACE_WARN level if
 *        a condition is met.
 */
#ifdef __cplusplus
#define ENETTRACE_WARN_IF(cond, ...) ((cond) ? ENETTRACE_WARN(__VA_ARGS__) : void())
#else
#define ENETTRACE_WARN_IF(cond, ...) ((cond) ? ENETTRACE_WARN(__VA_ARGS__) : 0U)
#endif
#else
#define ENETTRACE_WARN(fmt, ...)
#define ENETTRACE_WARN_IF(cond, ...)
#endif

/*!
 * \brief Helper macro to add trace message with #ENET_TRACE_INFO level
 *
 * Traces with this level should give only important informational messages
 * to the user, which typically they don't occur very often (i.e. "NIMU is
 * ready", "PHY n link is up").
 * This trace level may be enabled by default.
 */
#if (ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_INFO)
#define ENETTRACE_INFO(fmt, ...) ENETTRACE_trace(gEnetTrace_runtimeLevel, \
                                                 ENET_TRACE_INFO,         \
                                                 fmt "\r",                \
                                                 ## __VA_ARGS__)
/*!
 * \brief Helper macro to add trace message with #ENET_TRACE_INFO level if
 *        a condition is met.
 */
#ifdef __cplusplus
#define ENETTRACE_INFO_IF(cond, ...) ((cond) ? ENETTRACE_INFO(__VA_ARGS__) : void())
#else
#define ENETTRACE_INFO_IF(cond, ...) ((cond) ? ENETTRACE_INFO(__VA_ARGS__) : 0U)
#endif
#else
#define ENETTRACE_INFO(fmt, ...)
#define ENETTRACE_INFO_IF(cond, ...)
#endif

/*!
 * \brief Helper macro to add trace message with #ENET_TRACE_DEBUG level
 *
 * Traces with this level will provide the user further information about
 * operations taking place (i.e. "MDIO module is open", "PHY n has started
 * auto-negotiation, etc).
 * This trace level is likely not enabled by default, so most of driver's
 * traces will follow in this category.
 */
#if (ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_DEBUG)
#define ENETTRACE_DBG(fmt, ...) ENETTRACE_trace(gEnetTrace_runtimeLevel, \
                                                ENET_TRACE_DEBUG,        \
                                                fmt "\r",                \
                                                ## __VA_ARGS__)
/*!
 * \brief Helper macro to add trace message with #ENET_TRACE_DEBUG level if
 *        a condition is met.
 */
#ifdef __cplusplus
#define ENETTRACE_DBG_IF(cond, ...) ((cond) ? ENETTRACE_DBG(__VA_ARGS__) : void())
#else
#define ENETTRACE_DBG_IF(cond, ...) ((cond) ? ENETTRACE_DBG(__VA_ARGS__) : 0U)
#endif
#else
#define ENETTRACE_DBG(fmt, ...)
#define ENETTRACE_DBG_IF(cond, ...)
#endif

/*!
 * \brief Helper macro to add trace message with #ENET_TRACE_VERBOSE level
 *
 * Traces with this level will provide even more information and much more
 * often than the DEBUG level (i.e. "PHY n: NWAY_WAIT state", "DMA transfer
 * is complete").
 * Enabling this trace level is likely going to flood with messages, so
 * developers must ensure that their debug messages that occur often enough
 * are set with VERBOSE level.
 */
#if (ENET_CFG_TRACE_LEVEL >= ENET_CFG_TRACE_LEVEL_VERBOSE)
#define ENETTRACE_VERBOSE(fmt, ...) ENETTRACE_trace(gEnetTrace_runtimeLevel, \
                                                    ENET_TRACE_VERBOSE,      \
                                                    fmt "\r",                \
                                                    ## __VA_ARGS__)
/*!
 * \brief Helper macro to add trace message with #ENET_TRACE_VERBOSE level if
 *        a condition is met.
 */
#ifdef __cplusplus
#define ENETTRACE_VERBOSE_IF(cond, ...) ((cond) ? ENETTRACE_VERBOSE(__VA_ARGS__) : void())
#else
#define ENETTRACE_VERBOSE_IF(cond, ...) ((cond) ? ENETTRACE_VERBOSE(__VA_ARGS__) : 0U)
#endif
#else
#define ENETTRACE_VERBOSE(fmt, ...)
#define ENETTRACE_VERBOSE_IF(cond, ...)
#endif

/*!
 * \brief Variable declaration helper macro to avoid unused variable error
 *        (-Werror=unused-variable) when variable is used in TRACE and when
 *        corresponding trace level is not enabled.
 */
#define ENETTRACE_VAR(var)                    ((var) = (var))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/*!
 * \brief Enabled runtime trace level.
 */
extern EnetTrace_TraceLevel gEnetTrace_runtimeLevel;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Set runtime trace level.
 *
 * Set the driver's runtime travel level.
 *
 * \param level    Trace level to be enabled
 * \return Previous trace level
 */
EnetTrace_TraceLevel EnetTrace_setLevel(EnetTrace_TraceLevel level);

/*!
 * \brief Get runtime trace level.
 *
 * Get the driver's runtime travel level.
 *
 * \return Current trace level
 */
EnetTrace_TraceLevel EnetTrace_getLevel(void);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* ENET_TRACE_H_ */
