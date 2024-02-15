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
 * \file  enet_soc.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet SoC interface.
 */

#ifndef ENET_SOC_H_
#define ENET_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_types.h>
#include <include/core/enet_base.h>
#include <kernel/dpl/DebugP.h>

#if defined(SOC_AM64X) || defined(SOC_AM243X)
#include <soc/k3/am64x_am243x/enet_soc.h>
#include <soc/k3/k3_soc.h>
#include <drivers/udma/soc/am64x_am243x/udma_soc.h>
#elif defined(SOC_AM62AX)
#include <soc/k3/am62ax/enet_soc.h>
#include <soc/k3/k3_soc.h>
#include <drivers/udma/soc/am62ax/udma_soc.h>
#elif defined (SOC_AM273X)
#include <soc/am273x/enet_soc.h>
#elif defined(SOC_AWR294X)
#include <soc/awr294x/enet_soc.h>
#elif defined(SOC_AM263X)
#include <soc/am263x/enet_soc.h>
#elif defined(SOC_AM263PX)
#include <soc/am263px/enet_soc.h>
#elif defined(SOC_AWR2544)
#include <soc/awr2544/enet_soc.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */


/*!
 * \brief Assertion.
 * Note - We use '!(cond) before calling PDK OSAL as PDK OSAL_Assert is opposite of
 *        normal C assert. It asserts when condition is true.
 */
#if ENET_CFG_IS_ON(ASSERT)
#if defined(__KLOCWORK__) || defined(__cplusplus)
#define EnetSoc_assert(cond, ...)            do { if (!(cond)) abort(); } while (0)
#else /* !defined(__KLOCWORK__) && !defined(__cplusplus) */
#define EnetSoc_assert(cond, ...)                               \
    do {                                                        \
            DebugP_assert((bool)((cond)));                         \
    } while (0)
#endif /* defined(__KLOCWORK__) || defined(__cplusplus) */
#else /* !ENET_CFG_IS_ON(ASSERT) */
#define EnetSoc_assert(cond, ...)           (void)(cond)
#endif /* ENET_CFG_IS_ON(ASSERT) */


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

/*!
 * \brief Initialize SoC layer.
 *
 * Initializes the Enet SoC layer.
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetSoc_init(void);

/*!
 * \brief Deinitialize SoC layer.
 *
 * Deinitializes the Enet SoC layer.
 */
void EnetSoc_deinit(void);

/*!
 * \brief Get handle of the DMA driver for a given Ethernet peripheral.
 *
 * Gets the handle to the DMA driver corresponding to the Ethernet peripheral
 * identified by its type and instance id.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Instance Id
 *
 * \return DMA driver handle. NULL if no driver was found.
 */
EnetDma_Handle EnetSoc_getDmaHandle(Enet_Type enetType,
                                    uint32_t instId);

/*!
 * \brief Get handle of the Enet driver by its index.
 *
 * Gets the handle to the Enet driver by its index.  The index definition
 * itself it SoC specific and not meaningful outside SoC internal
 * implementation.
 *
 * This function is mainly used by the Enet top-layer to iterate over all
 * peripherals supported by the SoC. It's used along with EnetSoc_getEnetNum()
 * to query the number of available peripherals.
 *
 * \param idx       Ethernet peripheral index
 *
 * \return Eth driver handle. NULL if index is greater than the number of
 *         peripherals present in the SoC.
 */
Enet_Handle EnetSoc_getEnetHandleByIdx(uint32_t idx);

/*!
 * \brief Get handle of the Enet driver for a given Ethernet peripheral.
 *
 * Gets the handle to the Enet driver corresponding to the Ethernet peripheral
 * identified by its type and instance id.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Instance Id
 *
 * \return Eth driver handle. NULL if no driver was found.
 */
Enet_Handle EnetSoc_getEnetHandle(Enet_Type enetType,
                                  uint32_t instId);

/*!
 * \brief Get core id of caller
 *
 * Gets id of the core where this function is called from.
 *
 * \return Core Id
 */
uint32_t EnetSoc_getCoreId(void);

/*!
 * \brief Check if core is enable to use an Ethernet peripheral.
 *
 * Checks if a given core is allowed to use the Ethernet peripheral identified
 * by its type and instance id.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param coreId    Core id
 *
 * \return Wheter core is allowed to use the peripheral or not
 */
bool EnetSoc_isCoreAllowed(Enet_Type enetType,
                           uint32_t instId,
                           uint32_t coreId);

/*!
 * \brief Get number of present Ethernet peripherals.
 *
 * Gets the number of Ethernet peripheral present in this SoC.
 *
 * \return Number of peripherals
 */
uint32_t EnetSoc_getEnetNum(void);

/*!
 * \brief Get number of MAC ports in a peripheral.
 *
 * Gets the number of MAC ports in a peripheral identified by its type and
 * instance id.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return Number of MAC ports
 */
uint32_t EnetSoc_getMacPortMax(Enet_Type enetType,
                               uint32_t instId);

/*!
 * \brief Get SoC clock frequency.
 *
 * Gets the frequency (in Hz) of an SoC clock identified by \p clkId.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param clkId     Clock id
 *
 * \return Clock frequency in Hz
 */
uint32_t EnetSoc_getClkFreq(Enet_Type enetType,
                            uint32_t instId,
                            uint32_t clkId);

/*!
 * \brief Configure SoC connection of an interrupt.
 *
 * Perform the "SoC connection" configuration of a peripheral interrupt
 * identified by \p intrId.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param intrId    Interrupt id
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetSoc_setupIntrCfg(Enet_Type enetType,
                             uint32_t instId,
                             uint32_t intrId);

/*!
 * \brief Release SoC connection of an interrupt.
 *
 * Releases the "SoC connection" configuration of a peripheral interrupt
 * identified by \p intrId previuosly done via EnetSoc_setupIntrCfg().
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param intrId    Interrupt id
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetSoc_releaseIntrCfg(Enet_Type enetType,
                               uint32_t instId,
                               uint32_t intrId);

/*!
 * \brief Get the interrupt number of a peripheral interrupt signal.
 *
 * Gets the interrupt number that can be used to register/unregister an
 * ISR which corresponds to a peripheral interrupt signal.
 * The interrupt number could be routed at runtime via configurable
 * interrupt routes, which makes the interrupt number a dynamic parameter
 * from the driver's perspective.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param intrId    Interrupt id
 *
 * \return Interrupt number
 */
uint32_t EnetSoc_getIntrNum(Enet_Type enetType,
                            uint32_t instId,
                            uint32_t intrId);

/*!
 * \brief Get the interrupt trigger type of a peripheral interrupt signal.
 *
 * Gets the interrupt trigger type that can be used to register/unregister an
 * ISR which corresponds to a peripheral interrupt signal.
 * The interrupt trigger type could be routed at runtime via configurable
 * interrupt routes, which makes the interrupt number a dynamic parameter
 * from the driver's perspective.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param intrId    Interrupt id
 *
 * \return Interrupt trigger type
 */
uint32_t EnetSoc_getIntrTriggerType(Enet_Type enetType,
                                    uint32_t instId,
                                    uint32_t intrId);

/*!
 * \brief Get EFused MAC addresses.
 *
 * Get a list of EFused MAC addresses.
 *
 * \param macAddr   MAC address array
 * \param num       Max number of addresses to populate in \p macAddr array.
 *                  It's updated with actual number of addresses filled.
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetSoc_getEFusedMacAddrs(uint8_t macAddr[][ENET_MAC_ADDR_LEN],
                                  uint32_t *num);

/*!
 * \brief Get MAC port capability mask
 *
 * Gets the capability mask of a MAC port.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param macPort   MAC port number
 *
 * \return MAC port capability mask
 */
uint32_t EnetSoc_getMacPortCaps(Enet_Type enetType,
                                uint32_t instId,
                                Enet_MacPort macPort);

/*!
 * \brief Get MAC port MII mode.
 *
 * Gets the SoC-level MII mode of a given MAC port.  The MII mode could be
 * configurable or fixed.  If there is no SoC-level MII mode configuration,
 * then return #ENET_ENOTSUPPORTED.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param macPort   MAC port number
 * \param mii       MII interface
 *
 * \return #ENET_SOK if no errors. #ENET_ENOTSUPPORTED if SoC doesn't provide
 *         SoC-levle MII interface configuration. \ref Enet_ErrorCodes in case
 *         of any other failure.
 */
int32_t EnetSoc_getMacPortMii(Enet_Type enetType,
                              uint32_t instId,
                              Enet_MacPort macPort,
                              EnetMacPort_Interface *mii);
/*!
 * \brief Validate IP support in SOC
 *
 * Validates IP support in given SOC.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return True if enetType is supported otherwise false.
 */
uint32_t EnetSoc_isIpSupported(Enet_Type enetType,
                               uint32_t instId);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* ENET_SOC_H_ */
