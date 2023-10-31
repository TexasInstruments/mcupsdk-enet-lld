/*
 *  Copyright (c) Texas Instruments Incorporated 2020-23
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
 * \file  enet_cfg.h
 *
 * \brief This file contains the Enet configuration parameters.
 */

#ifndef ENET_CFG_H_
#define ENET_CFG_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <enet_soc_cfg.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Build-time config option is enabled. */
#define ENET_ON                                     (1U)

/*! \brief Build-time config option is disabled. */
#define ENET_OFF                                    (0U)

/*! \brief Preprocessor check if config option is enabled. */
#define ENET_CFG_IS_ON(name)                        ((ENET_CFG_ ## name) == ENET_ON)

/*! \brief Preprocessor check if config option is disabled. */
#define ENET_CFG_IS_OFF(name)                       ((ENET_CFG_ ## name) == ENET_OFF)

/* --------------------------------------------------------------------------*/
/*                         Enet generic config options                       */
/* --------------------------------------------------------------------------*/

/*! \brief EnetUtils print buffer length. */
#define ENET_CFG_PRINT_BUF_LEN                      (200U)

/*! \brief Whether Enet driver has a default OSAL implementation. */
#define ENET_CFG_HAS_DEFAULT_OSAL                   (ENET_ON)

/*! \brief Whether Enet driver has a default utils implementation. */
#define ENET_CFG_HAS_DEFAULT_UTILS                  (ENET_ON)

/*! \brief Enable top-layer sanity checks and misc debug info. */
#define ENET_CFG_SANITY_CHECKS                      (ENET_ON)

/* --------------------------------------------------------------------------*/
/*        CPSW Peripheral and CPSW Module related config options             */
/* --------------------------------------------------------------------------*/

/*! \brief CPSW CPTS stats */
#define ENET_CFG_CPSW_CPTS_STATS                    (ENET_OFF)

/*! \brief CPSW CPTS Event Pool size */
#define ENET_CFG_CPSW_CPTS_EVENTS_POOL_SIZE         (8U)

/*! \brief Resource Manager support */
#define ENET_CFG_RM_PRESENT                         (ENET_ON)

#if defined(SOC_AM64X) || defined(SOC_AM243X)
/*! \brief Overwrite UDMA config for ICSSG as we use more flows/channels for
 *         multiport testNumber of TX channels. */
#define ENET_CFG_NUM_INSTANCES                      (1U)
#endif

/*! \brief Enabling Optimized IRQ for critical section. */
#if defined(SOC_AM273X) || defined(SOC_AWR294X) || defined(SOC_AWR2544) || defined(SOC_AM263X) || defined(SOC_AM263PX)
#define ENET_CFG_USE_OPTIMIZED_IRQ_CRITICAL_SECTION  (1U)
#else
#define ENET_CFG_USE_OPTIMIZED_IRQ_CRITICAL_SECTION  (0U)
#endif

/*! \brief Eliminate embedded strings from all ASSERT/TRACE invocation.
 *
 * Enabling this option reduces rodata footprint of enet library by removing strings in trace.
 * Removal of strings will result in cryptic trace msgs because informative strings are removed and
 * only function name, file,line num info will be printed based on ENET_CFG_TRACE_TRACE_FORMAT define.
 */
#if (_DEBUG_ == 1)
#define ENET_CFG_TRACE_DISABLE_INFOSTRING           (ENET_OFF)
#else
#define ENET_CFG_TRACE_DISABLE_INFOSTRING           (ENET_ON)
#endif

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

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

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* ENET_CFG_H_ */
