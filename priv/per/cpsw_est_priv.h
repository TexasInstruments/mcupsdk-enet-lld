/*
 *  Copyright (c) Texas Instruments Incorporated 2022
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
 * \file  cpsw_est_priv.h
 *
 * \brief This file contains the private interface for CPSW peripheral
 *        EST feature.
 */

#ifndef CPSW_EST_PRIV_H_
#define CPSW_EST_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_per.h>
#include <include/core/enet_mod_tas.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Helper macro to create private IOCTL commands for TAS module. */
#define CPSW_EST_PRIVATE_IOCTL(x)               (ENET_IOCTL_TYPE_PRIVATE | \
                                                  ENET_IOCTL_TAS_BASE |    \
                                                  ENET_IOCTL_MIN(x))


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief MAC port's EST state (ESTF state, period)
 */
typedef struct Cpsw_EstState_s
{
    /*! Configured ESTF cycle time (0 if not configured). */
    uint64_t cycleTime;

    /*! Last AdminBaseTime passed by the application. */
    uint64_t adminBaseTime;

    /*! Current OperBaseTime, taken from last AdminBaseTime passed by the
     *  application. */
    uint64_t operBaseTime;

    /*! Current EST state */
    EnetTas_TasState state;
} Cpsw_EstState;

/*!
 * \brief CpswMacPortEst module Private IOCTL commands.Invoked within Enet LLD and not by application
 */
typedef enum Cpsw_EstPrivIoctl_e
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
    CPSW_EST_IOCTL_REGISTER_HANDLER = CPSW_EST_PRIVATE_IOCTL(0U),

} Cpsw_EstPrivIoctl;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Enable EST at peripheral level.
 *
 * \param hPer         Enet Peripheral handle
 */
void Cpsw_enableEst(EnetPer_Handle hPer);

/*!
 * \brief Disable EST at peripheral level.
 *
 * \param hPer         Enet Peripheral handle
 */
void Cpsw_disableEst(EnetPer_Handle hPer);

/*!
 * \brief Run an EST/TAS IOCTL operation on CPSW peripheral.
 *
 * \param hPer         Enet Peripheral handle
 * \param cmd          IOCTL command Id
 * \param prms         IOCTL parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t Cpsw_ioctlEst(EnetPer_Handle hPer,
                      uint32_t cmd,
                      Enet_IoctlPrms *prms);

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

#endif /* CPSW_EST_PRIV_H_ */
