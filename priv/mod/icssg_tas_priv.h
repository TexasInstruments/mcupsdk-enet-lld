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
 * \file  icssg_tas_priv.h
 *
 * \brief This file contains the private type definitions and helper macros for
 *        the ICSSG Tas driver.
 */

#ifndef ICSSG_TAS_PRIV_H_
#define ICSSG_TAS_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>

#include <include/core/enet_mod_tas.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
/*! \brief Helper macro to create private IOCTL commands for TAS module. */
#define ICSSG_TAS_PRIVATE_IOCTL(x)            (ENET_IOCTL_TYPE_PRIVATE | \
                                               ENET_IOCTL_TAS_BASE |    \
                                               ENET_IOCTL_MIN(x))


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/*!
 * \brief ICSSG TAS module Private IOCTL commands.Invoked within Enet LLD and not by application
 */
typedef enum Icssg_TasPrivIoctl_e
{
    /*!
     * \brief Register IOCTL handler
     *
     * IOCTL params:
     * -  inArgs: #EnetTas_RegisterIoctlInArgs
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ICSSG_TAS_IOCTL_REGISTER_HANDLER = ICSSG_TAS_PRIVATE_IOCTL(0U),

} Icssg_TasPrivIoctl;

/*!
 * \brief Generic input args.
 */
typedef struct EnetTas_RegisterIoctlInArgs_s
{
    EnetTas_GenericInArgs commonInArgs;

    Enet_IoctlRegisterHandlerInArgs registerHandler;

} EnetTas_RegisterIoctlInArgs;

/*!
 * \brief TAS List Structure based on firmware memory map
 */
typedef struct IcssgTas_FwList_s
{
    /*! Window gate mask list */
    uint8_t gateMaskList[ENET_TAS_MAX_CMD_LISTS];

    /*! Window end time list */
    uint32_t windowEndTimeList[ENET_TAS_MAX_CMD_LISTS];

    /*! Array of gate close time for each queue in each window */
    uint32_t gateCloseTimeList[ENET_TAS_MAX_CMD_LISTS][ENET_TAS_MAX_NUM_QUEUES];
} IcssgTas_FwList;

/*!
 * \brief ICSSG Tas object.
 */
typedef struct IcssgTas_Obj_s
{
    /*! EnetMod must be the first member */
    EnetMod_Obj enetMod;

    /*! ICSSG handle. Required internally for ICSSG R30 IOCTL access */
    void* hIcssg;

    /*! TAS state for each port */
    EnetTas_TasState state;

    /*! Config change variables */
    volatile EnetTas_ConfigStatus *configStatus;

    /*! Operating control list. Application modifies this and calls API to update actual list */
    EnetTas_ControlList operList;

    /*! Admin control list. Application modifies this and calls API to update actual list */
    EnetTas_ControlList adminList;

    /*! active List pointer used by firmware */
    volatile IcssgTas_FwList *fwActiveList;

    /*! shadow List pointer used by driver */
    volatile IcssgTas_FwList *fwShadowList;
} IcssgTas_Obj;

/*!
 * \brief ICSSG Tas handle.
 */
typedef struct IcssgTas_Obj_s *IcssgTas_Handle;



/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Open and initialize Tas module.
 *
 * Opens and initializes the ICSSG Tas module.  This functions doesn't expect
 * any config structure, \p cfgSize must be set to 0.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters, must be set to NULL
 * \param cfgSize   Size of the configuration parameters, must be set to 0
 *
 * \return #ENET_SOK or \ref Enet_ErrorCodes in case of any failure
 */
int32_t IcssgTas_open(EnetMod_Handle hMod,
                      Enet_Type enetType,
                      uint32_t instId,
                      const void *cfg,
                      uint32_t cfgSize);

/*!
 * \brief Rejoin a running Tas module.
 *
 * This operation is not currently supported.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \retval #ENET_ENOTSUPPORTED
 */
int32_t IcssgTas_rejoin(EnetMod_Handle hMod,
                        Enet_Type enetType,
                        uint32_t instId);

/*!
 * \brief Run an IOCTL operation on the Tas module.
 *
 * Runs a Enet Tas IOCTL operation on the ICSSG Tas module.
 *
 * \param hMod         Enet Module handle
 * \param cmd          IOCTL command Id
 * \param prms         IOCTL parameters
 *
 * \return #ENET_SOK or \ref Enet_ErrorCodes in case of any failure
 */
int32_t IcssgTas_ioctl(EnetMod_Handle hMod,
                       uint32_t cmd,
                       Enet_IoctlPrms *prms);

/*!
 * \brief Close ICSSG Tas module.
 *
 * \param hMod         Enet Module handle
 */
void IcssgTas_close(EnetMod_Handle hMod);

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

#endif /* ICSSG_TAS_PRIV_H_ */
