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
 * \file  cpsw_stats_priv.h
 *
 * \brief This file contains private type definitions and helper macros for the
 *        CPSW statistics module which are meant for internal use in Enet Per
 *        drivers.
 */

#ifndef CPSW_STATS_PRIV_H_
#define CPSW_STATS_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <include/core/enet_mod_stats.h>
#include <include/mod/cpsw_stats.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Helper macro to create private IOCTL commands for CPSW stats module. */
#define CPSW_STATS_PRIVATE_IOCTL(x)         (ENET_IOCTL_TYPE_PRIVATE | \
                                             ENET_IOCTL_STATS_BASE |   \
                                             ENET_IOCTL_PER_CPSW |     \
                                             ENET_IOCTL_MIN(x))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief Statistics private IOCTL commands.
 */
typedef enum CpswStats_PrivIoctls_e
{
    /*!
     * \brief Synchronize hardware and software statistics.
     *
     * IOCTL parameters:
     *   inArgs: None
     *  outArgs: None
     */
    CPSW_STATS_IOCTL_SYNC = CPSW_STATS_PRIVATE_IOCTL(0U),

    /*!
     * \brief Register IOCTL handler
     *
     * IOCTL params:
     * -  inArgs: #Enet_IoctlRegisterHandlerInArgs
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    CPSW_STATS_IOCTL_REGISTER_HANDLER = CPSW_STATS_PRIVATE_IOCTL(1U),
} CpswStats_PrivIoctls;



/*!
 * \brief CPSW statistics object.
 */
typedef struct CpwStats_Obj_s
{
    /*! EnetMod must be the first member */
    EnetMod_Obj enetMod;

    /*! Ethernet peripheral type, used to differentiate the statistics counters
     *  in CPSW_2G vs CPSW_9G. */
    Enet_Type enetType;

    /*! Host port statistics */
    CpswStats_PortStats *hostPortStats;

    /*! Ethernet ports statistics. Must have enough space for #macPortNum
     *  blocks of statistics data */
    CpswStats_PortStats *macPortStats;

    /*! Host port statistics */
    CpswStats_PortStats hostPortStatsMem;

    /*! Ethernet ports statistics. Must have enough space for #macPortNum
     *  blocks of statistics data */
    CpswStats_PortStats macPortStatsMem[CPSW_STATS_MACPORT_MAX];

    /*! Number of MAC ports. The number of stats blocks in #macPortStats */
    uint32_t macPortNum;
} CpswStats_Obj;

/*!
 * \brief Statistics module handle.
 */
typedef CpswStats_Obj *CpswStats_Handle;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Open and initialize CPSW stats.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters
 * \param cfgSize   Size of the configuration parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t CpswStats_open(EnetMod_Handle hMod,
                       Enet_Type enetType,
                       uint32_t instId,
                       const void *cfg,
                       uint32_t cfgSize);

/*!
 * \brief Rejoin a running CPSW stats module.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t CpswStats_rejoin(EnetMod_Handle hMod,
                         Enet_Type enetType,
                         uint32_t instId);

/*!
 * \brief Run an IOCTL operation on CPSW stats.
 *
 * \param hMod         Enet Module handle
 * \param cmd          IOCTL command Id
 * \param prms         IOCTL parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t CpswStats_ioctl(EnetMod_Handle hMod,
                        uint32_t cmd,
                        Enet_IoctlPrms *prms);

/*!
 * \brief Close CPSW stats.
 *
 * \param hMod         Enet Module handle
 */
void CpswStats_close(EnetMod_Handle hMod);

/*!
 * \brief Saves and Close CPSW stats.
 *
 * \param hMod         Enet Module handle
 */
void CpswStats_saveCtxt(EnetMod_Handle hMod);

/*!
 * \brief Restores and Open CPSW stats.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters
 * \param cfgSize   Size of the configuration parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t CpswStats_restoreCtxt(EnetMod_Handle hMod,
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

#endif /* CPSW_STATS_PRIV_H_ */
