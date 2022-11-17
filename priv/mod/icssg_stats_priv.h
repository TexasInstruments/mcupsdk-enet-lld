/*
 *  Copyright (c) Texas Instruments Incorporated 2021
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
 * \file  icssg_stats_priv.h
 *
 * \brief This file contains internal type definitions and helper macros for the
 *        ICSSG Hardware and PA statistics.
 */

#ifndef ICSSG_STATS_PRIV_H_
#define ICSSG_STATS_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/mod/icssg_stats.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
/*! \brief Helper macro to create private IOCTL commands for STATS module. */
#define ICSSG_STATS_PRIVATE_IOCTL(x)            (ENET_IOCTL_TYPE_PRIVATE | \
                                                 ENET_IOCTL_STATS_BASE |  \
                                                 ENET_IOCTL_MIN(x))


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/*!
 * \brief ICSSG TAS module Private IOCTL commands.Invoked within Enet LLD and not by application
 */
typedef enum Icssg_StatsPrivIoctl_e
{
    /*!
     * \brief Register IOCTL handler
     *
     * IOCTL params:
     * -  inArgs: #Enet_IoctlRegisterHandlerInArgs
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ICSSG_STATS_IOCTL_REGISTER_HANDLER = ICSSG_STATS_PRIVATE_IOCTL(0U),

} Icssg_StatsPrivIoctl;


/*!
 * \brief ICSSG Statistics object.
 */
typedef struct IcssgStats_Obj_s
{
    /*! EnetMod must be the first member */
    EnetMod_Obj enetMod;

    /*! MAC port 1 base address */
    uintptr_t port1Addr;

    /*! MAC port 2 base address. This is only applicable for ICSSG Switch */
    uintptr_t port2Addr;

    /*! PA statistics base address */
    uintptr_t paStatsAddr;
} IcssgStats_Obj;

/*!
 * \brief ICSSG Statistics handle.
 */
typedef struct IcssgStats_Obj_s *IcssgStats_Handle;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Open and initialize Stats module.
 *
 * Opens and initializes the ICSSG Stats module.  It doesn't take any
 * configuration parameters.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters. Must be NULL
 * \param cfgSize   Size of the configuration parameters. Must be 0
 *
 * \return #ENET_SOK or \ref Enet_ErrorCodes in case of any failure
 */
int32_t IcssgStats_open(EnetMod_Handle hMod,
                        Enet_Type enetType,
                        uint32_t instId,
                        const void *cfg,
                        uint32_t cfgSize);

/*!
 * \brief Rejoin a running ICSSG Stats module.
 *
 * This operation is not currently supported.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \retval #ENET_ENOTSUPPORTED
 */
int32_t IcssgStats_rejoin(EnetMod_Handle hMod,
                          Enet_Type enetType,
                          uint32_t instId);

/*!
 * \brief Run an IOCTL operation on the ICSSG Stats module.
 *
 * Runs a Enet Stats IOCTL operation on the ICSSG Stats module.
 *
 * \param hMod         Enet Module handle
 * \param cmd          IOCTL command Id
 * \param prms         IOCTL parameters
 *
 * \return #ENET_SOK or \ref Enet_ErrorCodes in case of any failure
 */
int32_t IcssgStats_ioctl(EnetMod_Handle hMod,
                         uint32_t cmd,
                         Enet_IoctlPrms *prms);

/*!
 * \brief Close ICSSG Stats module.
 *
 * \param hMod         Enet Module handle
 */
void IcssgStats_close(EnetMod_Handle hMod);

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

#endif /* ICSSG_STATS_PRIV_H_ */
