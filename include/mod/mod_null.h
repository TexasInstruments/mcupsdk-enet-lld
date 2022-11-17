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
 * \file  mod_null.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        "null" Ethernet Module interface.
 */

#ifndef MOD_NULL_H_
#define MOD_NULL_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_types.h>
#include <include/core/enet_mod.h>
#include <include/core/enet_trace.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! Null module dummy feature 1 */
#define ENET_NULLMOD_FEAT1                    (ENET_BIT(0U))

/*! Null module dummy errata 1 */
#define ENET_NULLMOD_ERRATA1                  (ENET_BIT(0U))

/*! Null module dummy errata 2 */
#define ENET_NULLMOD_ERRATA2                  (ENET_BIT(1U))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief Null Module configuration parameters.
 */
typedef struct NullMod_Cfg_s
{
    /*! Dummy config param */
    uint32_t dummy;
} NullMod_Cfg;

/*!
 * \brief Null Module object.
 */
typedef struct NullMod_Obj_s
{
    /*! Enet module header. Must be the first element */
    EnetMod_Obj enetMod;
} NullMod_Obj;

/*!
 * \brief Null Module handle.
 */
typedef struct NullMod_Obj_s *NullMod_Handle;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize Null module's configuration parameters.
 *
 * Initializes the configuration parameters for a Null module.
 *
 * \param nullModCfg  Configuration parameters to be initialized
 */
void NullMod_initCfg(NullMod_Cfg *nullModCfg);

/*!
 * \brief Open and initialize the Null Module.
 *
 * Opens and initializes the Null Module with the configuration parameters
 * provided by the caller.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t NullMod_open(EnetMod_Handle hMod,
                     Enet_Type enetType,
                     uint32_t instId,
                     const void *cfg);

/*!
 * \brief Rejoin the Null Module.
 *
 * Reopens the Null Module, but doesn't perform any hardware initialization.
 * This function is expected to be called to attach to a running module.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t NullMod_rejoin(EnetMod_Handle hMod,
                       Enet_Type enetType,
                       uint32_t instId);

/*!
 * \brief Issue an operation on the Null Module.
 *
 * Issues a control operation on the Null Module.
 *
 * \param hMod         Enet Module handle
 * \param cmd          IOCTL command Id
 * \param prms         IOCTL parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t NullMod_ioctl(EnetMod_Handle hMod,
                      uint32_t cmd,
                      Enet_IoctlPrms *prms);

/*!
 * \brief Close the Null Module.
 *
 * Closes the Null Module.
 *
 * \param hMod         Enet Module handle
 */
void NullMod_close(EnetMod_Handle hMod);


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

#endif /* MOD_NULL_H_ */
