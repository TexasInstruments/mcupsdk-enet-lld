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
 * \brief Initialize OSAL configuration.
 *
 * Initializes the passed OSAL configuration structure with a default
 * implementation which is based on the PDK OSAL library if
 * ENET_CFG_HAS_DEFAULT_OSAL config flag is enabled. Otherwise, the
 * configuration structure will be cleared.
 *
 * The caller can overwrite any OSAL functions after calling this API.
 *
 * \param osalCfg   OSAL configuration parameters
 */
void Enet_initOsalCfg(EnetOsal_Cfg *osalCfg);

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
 * \brief Initialize Enet LLD.
 *
 * One-time initialization of the Enet LLD driver.  This function initializes
 * the OSAL and utils infrastructure that the driver requires for handling
 * multiple peripherals as well as logging and tracing.
 *
 * The Enet LLD provides a default OSAL implementation which is based on PDK
 * OSAL library if ENET_CFG_HAS_DEFAULT_OSAL config flag is enabled.  The
 * default OSAL implementation can be used if the caller passes a NULL
 * \p osalCfg.
 *
 * Similarly, the Enet LLD provides a default utils implementation if
 * ENET_CFG_HAS_DEFAULT_UTILS config flag is set.  The default utils
 * implementation can be used if the caller passes a NULL \p utilsCfg.
 *
 * \param osalCfg   OSAL configuration parameters
 * \param utilsCfg  Utils configuration parameters
 */
void Enet_init(const EnetOsal_Cfg *osalCfg,
               const EnetUtils_Cfg *utilsCfg);

/*!
 * \brief De-initialize Enet LLD.
 *
 * One-time de-initialization of the Enet LLD driver.  This function clears
 * the OSAL and utils config parameters passed during Enet_init().
 *
 * It's expected to be called once all Ethernet peripherals have been closed
 * via Enet_close().
 */
void Enet_deinit(void);

/*!
 * \brief Get the Enet handle of a peripheral.
 *
 * Gets the driver handle of a peripheral identified by its type and instance
 * id.  If the driver hasn't been opened via Enet_open(), this function returns
 * NULL.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return Driver handle if opened. Otherwise, NULL.
 */
Enet_Handle Enet_getHandle(Enet_Type enetType,
                           uint32_t instId);

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
 * \brief Initialize the peripheral configuration parameters.
 *
 * Initializes the peripheral configuration parameters with default values.
 *
 * Configuration parameters are peripheral specific, so the Enet LLD doesn't
 * enforce any definition.  Instead, the application should use the
 * configuration structure corresponding to the peripheral(s) available in
 * the platform.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters to be initialized
 * \param cfgSize   Size of the configuration parameters
 */
void Enet_initCfg(Enet_Type enetType,
                  uint32_t instId,
                  void *cfg,
                  uint32_t cfgSize);

/*!
 * \brief Open and initializes the Enet driver for a peripheral.
 *
 * Opens and initializes the Ethernet peripheral with the configuration
 * parameters provided by the caller.  The peripheral is identified by its
 * type and instance id.
 *
 * Configuration parameters are peripheral specific, so the Enet LLD doesn't
 * enforce any definition.  Instead, the application should use the
 * configuration structure corresponding to the peripheral(s) available in
 * the platform.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters
 * \param cfgSize   Size of the configuration parameters
 *
 * \return Enet handle if successfully open, NULL otherwise.
 */
Enet_Handle Enet_open(Enet_Type enetType,
                      uint32_t instId,
                      const void *cfg,
                      uint32_t cfgSize);

/*!
 * \brief Rejoin a running Ethernet peripheral.
 *
 * Reopens the Enet Peripheral, but doesn't perform any hardware initialization.
 * This function is expected to be called to rejoin to a running peripheral.
 * The peripheral is identified by its type and instance id.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return Enet handle if successfully open, NULL otherwise.
 */
Enet_Handle Enet_rejoin(Enet_Type enetType,
                        uint32_t instId);

/*!
 * \brief Get the handle to the DMA used for packet transmit/receive.
 *
 * Gets the handle to the DMA which will later be used as arguments to other
 * DMA related APIs, such as EnetDma_openRxCh() or EnetDma_openTxCh().
 * Enet_getDmaHandle() is required for peripherals where DMA is opened internally,
 * such as J721E.  Enet_getDmaHandle() is not required in peripherals where DMA
 * is opened separately, such as AM273X.
 *
 * \param hEnet        Enet driver handle
 *
 * \return Enet DMA handle if Enet peripherals owns the handle, NULL otherwise.
 */
EnetDma_Handle Enet_getDmaHandle(Enet_Handle hEnet);


/*!
 * \brief Register a callback for an event.
 *
 * Registers a callback for an event.  The callback with be called either via
 * Enet_poll() when interrupts are disabled or internally by the driver when
 * servicing an interrupt in ISR context.
 *
 * \param hEnet        Enet driver handle
 * \param evt          Event being registered for
 * \param evtNum       Event number. Use 0 for single event.
 * \param evtCb        Callback function
 * \param evtCbArgs    Callback function arguments
 */
void Enet_registerEventCb(Enet_Handle hEnet,
                          Enet_Event evt,
                          uint32_t evtNum,
                          Enet_EventCallback evtCb,
                          void *evtCbArgs);

/*!
 * \brief Unregister callback for an event.
 *
 * Unregisters a callback for an event.
 *
 * \param hEnet        Enet driver handle
 * \param evt          Event being registered for
 * \param evtNum       Event number. Use 0 for single event.
 */
void Enet_unregisterEventCb(Enet_Handle hEnet,
                            Enet_Event evt,
                            uint32_t evtNum);

/*!
 * \brief Poll for Ethernet events.
 *
 * Unblocking poll for the events specified in \p evtMask. The event mask can
 * be constructed using the types defined in #Enet_Event.  The callback function
 * must be registered via Enet_registerEventCb() prior to calling Enet_poll().
 *
 * Note that not all peripherals support poll mechanism.
 *
 * \param hEnet        Enet driver handle
 * \param evt          Event type
 * \param arg          Pointer to the poll argument. This is specific to the
 *                     poll event type. Refer to #Enet_Event for required
 *                     argument types
 * \param argSize      Size of \p arg. This is used to validate the argument type
 */
void Enet_poll(Enet_Handle hEnet,
               Enet_Event evt,
               const void *arg,
               uint32_t argSize);

/*!
 * \brief Run periodic tick on the Ethernet peripheral.
 *
 * Run PHY periodic tick on the Ethernet peripheral.
 *
 * \param hEnet        Enet driver handle
 */
void Enet_periodicTick(Enet_Handle hEnet);

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
uint32_t Enet_getMacPortCnt(Enet_Handle hEnet);

/*!
 * \brief Close the Enet peripheral.
 *
 * Closes the Ethernet Peripheral.
 *
 * \param hEnet        Enet driver handle
 */
void Enet_close(Enet_Handle hEnet);

/*!
 * \brief  Save  and closes the context of the Enet peripheral.
 *
 * Save the Ethernet Peripheral.
 *
 * \param hEnet        Enet driver handle
 * 
 * \return status as ENET_SOK if successful
 */
int32_t Enet_saveCtxt(Enet_Handle hEnet);

/*!
 * \brief Restore and open the context of the Enet peripheral.
 *
 * Restore the Ethernet Peripheral.
 *
 * \param enetType        Enet Peripheral type
 * \param instId          Enet Peripheral instance id
 * 
 * \return status as ENET_SOK if successful
 */
int32_t Enet_restoreCtxt(Enet_Type enetType,
                             uint32_t instId);

/*!
 * \brief Hard reset CPSW peripheral
 *
 * Restore the Ethernet Peripheral.
 *
 * \param hEnet                Enet Handle
 * \param enetType             Enet Peripheral type
 * \param instId               Enet Peripheral instance id
 * \param pCpswTriggerResetCb  CPSW Reset SOC specific callback function
 */
int32_t Enet_hardResetCpsw(Enet_Handle hEnet,
                        Enet_Type enetType,
                        uint32_t instId,
                        Enet_notify_t *pCpswTriggerResetCb);
/*!
 * \brief Get enetType and instId info from the enet handle.
 *
 * Returns the enetType and instance id associated with Enet_Handle.
 *
 * \param hEnet        Enet driver handle
 * \param enetType     Pointer to enetType set by this function
 * \param instId       Instance Id
 */
int32_t Enet_getHandleInfo(Enet_Handle hEnet,
                           Enet_Type *enetType,
                           uint32_t *instId);
/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* ENET_H_ */

/*! @} */
