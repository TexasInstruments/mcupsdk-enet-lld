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
 * \file  enet.h
 *
 * \brief This file contains the top-level API of the Enet driver.
 */

/**
 * \defgroup NETWORKING_MODULE APIs for Networking modules
 *
 * This module contains APIs which are used by the networking components.
 */

/*!
 * \defgroup DRV_ENET_MODULE APIs for Enet LLD
 * \ingroup NETWORKING_MODULE
 */

/*!
 * \ingroup  DRV_ENET_MODULE
 * \defgroup DRV_ENET_PERS Enet Peripherals
 */

/*!
 * \ingroup  DRV_ENET_MODULE
 * \defgroup ENET_MAIN_API Enet Main API
 *
 * @{
 */

#ifndef ENET_H_
#define ENET_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include <enet_cfg.h>
#include <include/core/enet_types.h>
#include <include/core/enet_base.h>
#include <include/core/enet_osal.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_ioctl.h>
#include <include/core/enet_trace.h>
#include <include/core/enet_queue.h>
#include <include/core/enet_soc.h>
#include <include/core/enet_mod_fdb.h>
#include <include/core/enet_mod_port.h>
#include <include/core/enet_mod_timesync.h>
#include <include/core/enet_mod_tas.h>
#include <include/core/enet_mod_phy.h>
#include <include/core/enet_dma.h>


#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * Get Enet Type.
 *
 * Gets the enet type of the underlying ethernet peripheral.
 * 
 * \param hEnet   Enet handle fo the underlying etehrnet peripheral
 *
 * \return Enet Type
 */
Enet_Type Enet_getEnetType(uint32_t hEnet);

/*!
 * Get InstId.
 *
 * Gets the instid of the underlying ethernet peripheral.
 * 
 * \param hEnet   Enet handle fo the underlying etehrnet peripheral
 *
 * \return InstId
 */
uint32_t Enet_getInstId(uint32_t hEnet);

/*!
 * Get self core id.
 *
 * Gets the core id of the calling core.  The driver doesn't enforce any
 * specific core id definitions, it's up to the Enet SoC layer to define it.
 *
 * \return Core id
 */
uint32_t Enet_getCoreId(void);

/*!
 * \brief Set global trace level.
 *
 * Sets the trace level of the Enet LLD.  The driver provides the
 * following trace levels: ERROR, WARN, INFO, DEBUG and VERBOSE.
 *
 * This function returns the previous trace level, which comes in
 * handy when restoring trace level, if needed.
 *
 * \param level   Trace level
 *
 * \return Previuos trace level
 */
EnetTrace_TraceLevel Enet_setTraceLevel(EnetTrace_TraceLevel level);

/*!
 * \brief Get current trace level.
 *
 * Get the current global trace level of the Enet LLD.
 *
 * \return Current trace level
 */
EnetTrace_TraceLevel Enet_getTraceLevel(void);

/*!
 * \brief Initialize utils configuration.
 *
 * Initializes the passed utils configuration structure with a default
 * implementation if ENET_CFG_HAS_DEFAULT_OSAL config flag is enabled.
 * Otherwise, the configuration structure will be cleared.
 * The default utils implementation provides UART-based logging and
 * one-to-one address translation.
 *
 * The caller can overwrite any utils functions after calling this API.
 *
 * \param utilsCfg  Utils configuration parameters
 */
void Enet_initUtilsCfg(EnetUtils_Cfg *utilsCfg);

/*!
 * \brief Get number of MAC ports available in the Ethernet peripheral.
 *
 * Gets the number of MAC ports available in the Ethernet peripheral identified
 * by its type and instance id.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return Number of MAC ports
 */
uint32_t Enet_getMacPortMax(Enet_Type enetType,
                            uint32_t instId);

/*!
 * \brief Get number of MAC ports available in the Ethernet peripheral.
 *
 * Gets the number of MAC ports available in the Ethernet peripheral.  This
 * function is similar to Enet_getMacPortMax() except that it takes an Enet
 * handle.
 *
 * \param hEnet        Enet driver handle
 *
 * \return Number of MAC ports
 */
uint32_t Enet_getMacPortCnt(uint32_t hEnet);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* ENET_H_ */

/*! @} */
