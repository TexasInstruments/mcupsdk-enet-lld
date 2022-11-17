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
 * \file  enet_trace.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet Trace interface.
 */

/*!
 * \ingroup  DRV_ENET_MODULE
 * \defgroup ENET_TRACE_API Enet Trace API
 *
 * @{
 */

#ifndef ENET_TRACE_H_
#define ENET_TRACE_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief All traces disabled at build-time. */
#define ENET_CFG_TRACE_LEVEL_NONE             (0U)

/*! \brief Build-time error level. */
#define ENET_CFG_TRACE_LEVEL_ERROR            (1U)

/*! \brief Build-time warning level. */
#define ENET_CFG_TRACE_LEVEL_WARN             (2U)

/*! \brief Build-time information level. */
#define ENET_CFG_TRACE_LEVEL_INFO             (3U)

/*! \brief Build-time debug level. */
#define ENET_CFG_TRACE_LEVEL_DEBUG            (4U)

/*! \brief Build-time verbose level. */
#define ENET_CFG_TRACE_LEVEL_VERBOSE          (5U)

/*! \brief Default trace level if none is set. */
#ifndef ENET_CFG_TRACE_LEVEL
#define ENET_CFG_TRACE_LEVEL                  (ENET_CFG_TRACE_LEVEL_INFO)
#endif

/*! \brief Trace prefix: "<func>: string" */
#define ENET_CFG_TRACE_FORMAT_FUNC            (0U)

/*! \brief Trace prefix: "<func>: <line>: string" */
#define ENET_CFG_TRACE_FORMAT_FILE            (1U)

/*! \brief Trace prefix: "<file>: <line>: <func>: <line>: string" */
#define ENET_CFG_TRACE_FORMAT_FULL            (2U)

/*! \brief Default trace format if none is specified. */
#ifndef ENET_CFG_TRACE_TRACE_FORMAT
#define ENET_CFG_TRACE_TRACE_FORMAT           (ENET_CFG_TRACE_FORMAT_FUNC)
#endif

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief Enumerates the types of trace level.
 */
typedef enum
{
    /*! All traces are disabled at runtime */
    ENET_TRACE_NONE    = 0U,

    /*! Error trace level */
    ENET_TRACE_ERROR   = 1U,

    /*! Warning trace level */
    ENET_TRACE_WARN    = 2U,

    /*! Info trace level: enables only important informational messages for the
     * user (i.e. PHY link is up or down, NIMU layer is ready, etc).
     *
     * The amount of info logs is not invasive in nature so this trace level
     * may be enabled by applications at init time. */
    ENET_TRACE_INFO    = 3U,

    /*! Debug trace level: enables further information messages about operations
     * taking place in the driver (i.e. a module is being opened, PHY
     * auto-negotiation is started, etc).
     *
     * The debug level should be enabled by the user on a need basis (i.e.
     * for debugging or tracing execution flow, etc) as the number of messages
     * will increase considerably with respect to #ENET_TRACE_INFO level.
     *
     * This trace level can be enabled at runtime only in 'debug' builds. */
    ENET_TRACE_DEBUG   = 4U,

    /*! Verbose trace level: enables even further messages about operations
     * taking place in the driver (i.e. PHY state transitions, DMA transfer
     * completion, etc) that are periodic in nature or simply happen very often
     * during normal execution.
     *
     * The amount of messages will increase drastically when the verbose level
     * is enabled, so it's recommended to set it only if really needed.
     *
     * This trace level can be enabled at runtime only in 'debug' builds. */
    ENET_TRACE_VERBOSE = 5U,
} EnetTrace_TraceLevel;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* ENET_TRACE_H_ */

/*! @} */
