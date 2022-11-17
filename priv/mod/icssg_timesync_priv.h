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
 * \file  icssg_timesync_priv.h
 *
 * \brief This file contains the private type definitions and helper macros for
 *        the ICSSG TimeSync driver.
 */

#ifndef ICSSG_TIMESYNC_PRIV_H_
#define ICSSG_TIMESYNC_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>

#include <include/mod/icssg_timesync.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
/*! \brief Helper macro to create private IOCTL commands for TIMESYNC module. */
#define ICSSG_TIMESYNC_PRIVATE_IOCTL(x)         (ENET_IOCTL_TYPE_PRIVATE |     \
                                                 ENET_IOCTL_TIMESYNC_BASE |    \
                                                 ENET_IOCTL_MIN(x))


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/*!
 * \brief ICSSG Timesynch module Private IOCTL commands.Invoked within Enet LLD and not by application
 */
typedef enum Icssg_TimesyncPrivIoctl_e
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
    ICSSG_TIMESYNC_IOCTL_REGISTER_HANDLER = ICSSG_TIMESYNC_PRIVATE_IOCTL(0U),

} Icssg_TimesyncPrivIoctl;


/*!
 * \brief ICSSG TimeSync object.
 */
typedef struct IcssgTimeSync_Obj_s
{
    /*! EnetMod must be the first member */
    EnetMod_Obj enetMod;

    /*! ICSSG handle. Required internally cycleTime value */
    void* hIcssg;

    /*! Drift applied on WC timers per sync interval. Workaround for rate computation */
    int32_t drift;

    /*! Sync Interval for drift applied on WC timers. Workaround for rate computation */
    uint32_t syncInterval;

    /*! Flag to indicate setClock is in progress */
    volatile bool setClockOngoing;

    /*! Control variable for set/get operations - WC, ST */
    IcssgTimeSync_ClkType clkType;
} IcssgTimeSync_Obj;

/*!
 * \brief ICSSG TimeSync handle.
 */
typedef struct IcssgTimeSync_Obj_s *IcssgTimeSync_Handle;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize TimeSync configuration parameters.
 *
 * \param timeSyncCfg  Config parameters to be initialized
 */
void IcssgTimeSync_initCfg(IcssgTimeSync_Cfg *timeSyncCfg);

/*!
 * \brief Open and initialize TimeSync module.
 *
 * Opens and initializes the ICSSG TimeSync module.  This functions expects a
 * config structure of type #IcssgTimeSync_Cfg, \p cfgSize must be passed
 * accordingly.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters
 * \param cfgSize   Size of the configuration parameters
 *
 * \return #ENET_SOK or \ref Enet_ErrorCodes in case of any failure
 */
int32_t IcssgTimeSync_open(EnetMod_Handle hMod,
                           Enet_Type enetType,
                           uint32_t instId,
                           const void *cfg,
                           uint32_t cfgSize);

/*!
 * \brief Rejoin a running TimeSync module.
 *
 * This operation is not currently supported.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \retval #ENET_ENOTSUPPORTED
 */
int32_t IcssgTimeSync_rejoin(EnetMod_Handle hMod,
                             Enet_Type enetType,
                             uint32_t instId);

/*!
 * \brief Run an IOCTL operation on the TimeSync module.
 *
 * Runs a Enet TimeSync IOCTL operation on the ICSSG TimeSync module.
 *
 * \param hMod         Enet Module handle
 * \param cmd          IOCTL command Id
 * \param prms         IOCTL parameters
 *
 * \return #ENET_SOK or \ref Enet_ErrorCodes in case of any failure
 */
int32_t IcssgTimeSync_ioctl(EnetMod_Handle hMod,
                          uint32_t cmd,
                          Enet_IoctlPrms *prms);

/*!
 * \brief Close ICSSG TimeSync module.
 *
 * \param hMod         Enet Module handle
 */
void IcssgTimeSync_close(EnetMod_Handle hMod);

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

#endif /* ICSSG_TIMESYNC_PRIV_H_ */
