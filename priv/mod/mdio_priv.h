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
 * \file  mdio_priv.h
 *
 * \brief This file contains private type definitions and helper macros for the
 *        MDIO port module which are meant for internal use in Enet Per drivers.
 */

#ifndef MDIO_PRIV_H_
#define MDIO_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_mod_mdio.h>
#include <include/mod/mdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Helper macro to create private IOCTL commands for MDIO module. */
#define MDIO_PRIVATE_IOCTL(x)                 (ENET_IOCTL_TYPE_PRIVATE | \
                                               ENET_IOCTL_MDIO_BASE |    \
                                               ENET_IOCTL_PER_CPSW |     \
                                               ENET_IOCTL_MIN(x))

/*! \brief MDIO Clause-45 feature mask. */
#define MDIO_FEATURE_CLAUSE45                 (ENET_BIT(0U))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief MDIO private IOCTL commands.
 */
typedef enum Mdio_PrivIoctls_e
{
    /*!
     * \brief Handle MDIO_INTR which combines MDIO_LINKINT and MDIO_USERINT
     *        events.
     *
     * IOCTL parameters:
     *   inArgs: #Mdio_Callbacks
     *  outArgs: None
     */
    MDIO_IOCTL_HANDLE_INTR = MDIO_PRIVATE_IOCTL(0U),

    /*!
     * \brief Register IOCTL handler
     *
     * IOCTL params:
     * -  inArgs: #Enet_IoctlRegisterHandlerInArgs
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    MDIO_IOCTL_REGISTER_HANDLER = MDIO_PRIVATE_IOCTL(1U),
} Mdio_PrivIoctls;

/*!
 * \brief MDIO PHY status.
 *
 * PHY alive and linked status as detected by MDIO.
 */
typedef struct Mdio_PhyStatus_s
{
    /*! Bit mask of PHYs that have been detected as alive */
    uint32_t aliveMask;

    /*! Bit mask of PHYs that have been detected as linked */
    uint32_t linkedMask;
} Mdio_PhyStatus;

/*!
 * \brief MDIO link state change callback function.
 *
 * Callback for the MDIO link state change interrupt (MDIO_LINKINT).  This
 * callback is invoked from interrupt context.
 *
 * \param group        User channel or group. Not applicable for Manual mode
 * \param phyStatus    PHY status: alive and linked masks
 * \param cbArgs       Callback function arguments
 */
typedef void (*Mdio_LinkStateCallback)(EnetMdio_Group group,
                                       Mdio_PhyStatus *phyStatus,
                                       void *cbArgs);

/*!
 * \brief MDIO user access completion callback function.
 *
 * \param group        User channel or group. Not applicable for Manual mode
 * \param phyAddr      Address of the PHY that completed access
 * \param cbArgs       Callback function arguments
 */
typedef void (*Mdio_UserAccessCallback)(EnetMdio_Group group,
                                        uint32_t phyAddr,
                                        void *cbArgs);

/*!
 * \brief MDIO callback functions.
 *
 * Optional callback functions that can be used to get notification about link
 * state change and user access completion events.
 */
typedef struct Mdio_Callbacks_s
{
    /*! Link state change interrupt callback function */
    Mdio_LinkStateCallback linkStateCb;

    /*! User access (read/write) completion callback function */
    Mdio_UserAccessCallback userAccessCb;

    /*! Application specific data to be passed to the callbacks */
    void *cbArgs;
} Mdio_Callbacks;

/*!
 * \brief MDIO port object.
 */
typedef struct Mdio_Obj_s
{
    /*! EnetMod must be the first member */
    EnetMod_Obj enetMod;

    /*! Whether this MDIO object has master behavior or not.  When in master role,
     *  the module will perform all MDIO configuration. In slave role, MDIO
     *  configuration is bypassed. */
    bool isMaster;
    
    /*! MDIO module operating mode */
    Mdio_OpMode mode;

    /*! MDIO bit banging delay in terms of CPU Clock ticks. No that this value shall
     * be set to 'half' of required MDIO CLK period. Applicable only when MDIO mode is set to manual mode */
    uint32_t mdcHalfCycleNs;
} Mdio_Obj;

/*!
 * \brief MDIO module handle.
 */
typedef Mdio_Obj *Mdio_Handle;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Open and initialize MDIO.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters
 * \param cfgSize   Size of the configuration parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t Mdio_open(EnetMod_Handle hMod,
                  Enet_Type enetType,
                  uint32_t instId,
                  const void *cfg,
                  uint32_t cfgSize);

/*!
 * \brief Rejoin a running MDIO.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t Mdio_rejoin(EnetMod_Handle hMod,
                    Enet_Type enetType,
                    uint32_t instId);

/*!
 * \brief Run an IOCTL operation on MDIO.
 *
 * \param hMod         Enet Module handle
 * \param cmd          IOCTL command Id
 * \param prms         IOCTL parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t Mdio_ioctl(EnetMod_Handle hMod,
                   uint32_t cmd,
                   Enet_IoctlPrms *prms);


/*!
 * \brief Close MDIO.
 *
 * \param hMod         Enet Module handle
 */
void Mdio_close(EnetMod_Handle hMod);

/*!
 * \brief Saves and Close MDIO.
 *
 * \param hMod         Enet Module handle
 */
void Mdio_saveCtxt(EnetMod_Handle hMod);

/*!
 * \brief Restores and Open MDIO.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters
 * \param cfgSize   Size of the configuration parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t Mdio_restoreCtxt(EnetMod_Handle hMod,
                         Enet_Type enetType,
                         uint32_t instId,
                         const void *cfg,
                         uint32_t cfgSize);

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

#endif /* MDIO_PRIV_H_ */
