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
 * \file  per_null.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        "null" peripheral interface.
 */

#ifndef PER_NULL_H_
#define PER_NULL_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_types.h>
#include <include/core/enet_mod.h>
#include <include/core/enet_per.h>
#include <include/mod/mod_null.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! Null peripheral dummy feature 1 */
#define ENET_NULLPER_FEAT1                    (ENET_BIT(0U))

/*! Null peripheral dummy feature 2 */
#define ENET_NULLPER_FEAT2                    (ENET_BIT(1U))

/*! Null peripheral dummy errata 1 */
#define ENET_NULLPER_ERRATA1                  (ENET_BIT(0U))

/*! Null peripheral MCLK id */
#define ENET_NULLPER_CLK_MCLK                 (1U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief Null Peripheral configuration parameters.
 */
typedef struct NullPer_Cfg_s
{
    /*! Mod1 configuration */
    NullMod_Cfg mod1Cfg;

    /*! Mod2 configuration */
    NullMod_Cfg mod2Cfg;

    /*! Dummy config param */
    uint32_t dummy;
} NullPer_Cfg;

/*!
 * \brief Null Peripheral object.
 */
typedef struct NullPer_Obj_s
{
    /*! Enet peripheral header. Must be the first element */
    EnetPer_Obj enetPer;

    /*! Null module 1 */
    NullMod_Obj mod1;

    /*! Null module 2 */
    NullMod_Obj mod2;
} NullPer_Obj;

/*!
 * \brief Null Periperhal handle.
 */
typedef struct NullPer_Obj_s *NullPer_Handle;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize Null peripheral's configuration parameters.
 *
 * Initializes the configuration parameters for a Null periperhal.
 *
 * \param hPer      Enet Peripheral handle
 * \param enetType  Enet Peripheral type
 * \param cfg       Configuration parameters to be initialized
 * \param cfgSize   Size of the configuration parameters
 */
void NullPer_initCfg(EnetPer_Handle hPer,
                     Enet_Type enetType,
                     void *cfg,
                     uint32_t cfgSize);

/*!
 * \brief Open and initialize the Null Peripheral.
 *
 * Opens and initializes the Null Peripheral with the configuration parameters
 * provided by the caller.
 *
 * \param hPer      Enet Peripheral handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters
 * \param cfgSize   Size of the configuration parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t NullPer_open(EnetPer_Handle hPer,
                     Enet_Type enetType,
                     uint32_t instId,
                     const void *cfg,
                     uint32_t cfgSize);

/*!
 * \brief Rejoin the Null Peripheral.
 *
 * Reopens the Null Peripheral, but doesn't perform any hardware initialization.
 * This function is expected to be called to attach to a running peripheral.
 *
 * \param hPer      Enet Peripheral handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t NullPer_rejoin(EnetPer_Handle hPer,
                       Enet_Type enetType,
                       uint32_t instId);

/*!
 * \brief Issue an operation on the Null Peripheral.
 *
 * Issues a control operation on the Null Peripheral.
 *
 * \param hPer         Enet Peripheral handle
 * \param cmd          IOCTL command Id
 * \param prms         IOCTL parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t NullPer_ioctl(EnetPer_Handle hPer,
                      uint32_t cmd,
                      Enet_IoctlPrms *prms);

/*!
 * \brief Poll for Ethernet events on the Null Peripheral.
 *
 * Unblocking poll for the events specified in \p evtMask.
 *
 * \param hPer         Enet Peripheral handle
 * \param evtMask      Event bit mask
 */
void NullPer_poll(EnetPer_Handle hPer,
                  uint32_t evtMask);

/*!
 * \brief Run periodic tick on the Null Peripheral.
 *
 * Run PHY periodic tick on the Ethernet peripheral.
 *
 * \param hPer        Enet Peripheral handle
 */
void NullPer_periodicTick(EnetPer_Handle hPer);

/*!
 * \brief Close the Null Peripheral.
 *
 * Closes the Null Peripheral.
 *
 * \param hPer         Enet Peripheral handle
 */
void NullPer_close(EnetPer_Handle hPer);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* PER_NULL_H_ */
