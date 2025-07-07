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
 * \file  enet_mod_stats.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet Statistics module interface.
 */

/*!
 * \ingroup  DRV_ENET_MODULE
 * \defgroup ENET_MOD_STATS Enet Statistics
 *
 * @{
 */

#ifndef ENET_MOD_STATS_H_
#define ENET_MOD_STATS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_mod.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Helper macro to create IOCTL commands for STATS module. */
#define ENET_STATS_PUBLIC_IOCTL(x)             (ENET_IOCTL_TYPE_PUBLIC | \
                                                ENET_IOCTL_STATS_BASE |  \
                                                ENET_IOCTL_MIN(x))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief Statistics IOCTL commands.
 */
typedef enum EnetStats_Ioctl_e
{
    /*!
     * \brief Get the hardware version of the Stats module.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: #Enet_Version
     */
    ENET_STATS_IOCTL_GET_VERSION = ENET_STATS_PUBLIC_IOCTL(0U),

    /*!
     * \brief Print Stats registers.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: None
     */
    ENET_STATS_IOCTL_PRINT_REGS = ENET_STATS_PUBLIC_IOCTL(1U),

    /*!
     * \brief Get host port statistics.
     *
     * Gets the network statistics of the host port.  It's caller's
     * responsibility to typecast the returned statistics structure according
     * to the underlying peripheral's definition.
     *
     * Use this IOCTL if the statistics module doesn't support separate host
     * port and MAC port statistics.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: Peripheral/module specific definition.
     */
    ENET_STATS_IOCTL_GET_HOSTPORT_STATS = ENET_STATS_PUBLIC_IOCTL(2U),

    /*!
     * \brief Get MAC port statistics.
     *
     * Gets the network statistics of the given MAC port number. It's caller's
     * responsibility to typecast the returned statistics structure according
     * to the underlying peripheral's definition.
     * For ICSSG MAC mode usecase, port number can't be other than  #ENET_MAC_PORT_1,
     * as each ICSSG MAC supports only one MAC port.
     *
     * IOCTL parameters:
     * -  inArgs: #Enet_MacPort
     * - outArgs: Peripheral/module specific definition.
     */
    ENET_STATS_IOCTL_GET_MACPORT_STATS = ENET_STATS_PUBLIC_IOCTL(3U),

    /*!
     * \brief Reset host port statistics.
     *
     * Resets the network statistics counters of the host port.
     *
     * Use this IOCTL if the statistics module doesn't support separate host
     * port and MAC port statistics.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: None
     */
    ENET_STATS_IOCTL_RESET_HOSTPORT_STATS = ENET_STATS_PUBLIC_IOCTL(4U),

    /*!
     * \brief Reset MAC port statistics.
     *
     * Resets the network statistics counters of the given MAC port number.
     * For ICSSG MAC mode usecase, port number can't be other than #ENET_MAC_PORT_1,
     * as each ICSSG MAC supports only one MAC port.
     *
     * IOCTL parameters:
     * -  inArgs: #Enet_MacPort
     * - outArgs: None
     */
    ENET_STATS_IOCTL_RESET_MACPORT_STATS = ENET_STATS_PUBLIC_IOCTL(5U),
} EnetStats_Ioctl;


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

#endif /* ENET_MOD_STATS_H_ */

/*! @} */
