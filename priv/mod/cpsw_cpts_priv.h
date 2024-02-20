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
 * \file  cpsw_cpts_priv.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        CPSW CPTS module.
 */

#ifndef CPSW_CPTS_PRIV_H_
#define CPSW_CPTS_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <enet_cfg.h>
#include <include/core/enet_mod_timesync.h>
#include <include/mod/cpsw_cpts.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Helper macro to create private IOCTL commands for CPSW CPTS module. */
#define CPSW_CPTS_PRIVATE_IOCTL(x)             (ENET_IOCTL_TYPE_PRIVATE |  \
                                                ENET_IOCTL_TIMESYNC_BASE | \
                                                ENET_IOCTL_PER_CPSW |      \
                                                ENET_IOCTL_MIN(x))

/*! \brief CPTS GENFn reconfig errata mask. */
#define CPSW_CPTS_ERRATA_GENFN_RECONFIG        (ENET_BIT(0U))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief CPTS private IOCTL commands.
 */
typedef enum CpswCpts_PrivIoctls_e
{
    /*!
     * \brief Handle the interrupt and notify the event's registered callback.
     *
     * IOCTL parameters:
     *   inArgs: None
     *  outArgs: None
     */
    CPSW_CPTS_IOCTL_HANDLE_INTR = CPSW_CPTS_PRIVATE_IOCTL(0U),

    /*!
     * \brief Enable CPTS interrupt.
     *
     * IOCTL parameters:
     *   inArgs: None
     *  outArgs: None
     */
    CPSW_CPTS_IOCTL_ENABLE_INTR = CPSW_CPTS_PRIVATE_IOCTL(1U),

    /*!
     * \brief Disable CPTS interrupt.
     *
     * IOCTL parameters:
     *   inArgs: None
     *  outArgs: None
     */
    CPSW_CPTS_IOCTL_DISABLE_INTR = CPSW_CPTS_PRIVATE_IOCTL(2U),

    /*!
     * \brief Register IOCTL handler
     *
     * IOCTL params:
     * -  inArgs: #Enet_IoctlRegisterHandlerInArgs
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    CPSW_CPTS_IOCTL_REGISTER_HANDLER = CPSW_CPTS_PRIVATE_IOCTL(3U),

} CpswCpts_PrivIoctls;


/*!
 * \brief CPTS event memory object.
 */
typedef struct CpswCpts_EventMemObj_s
{
    /*! Event memory pool */
    CpswCpts_Event eventMemPool[ENET_CFG_CPSW_CPTS_EVENTS_POOL_SIZE];

    /*! Index where the event is to be written */
    uint32_t index;
} CpswCpts_EventMemObj;

/*!
 * \brief CPTS event statistics.
 */
typedef struct CpswCpts_EventStats_s
{
    /*! Count of software time stamp push events */
    uint64_t swTsPushEventCnt;

    /*! Count of hardware time stamp push events */
    uint64_t hwTsPushEventCnt;

    /*! Count of time stamp compare events */
    uint64_t cmpEventCnt;

    /*! Count of ethernet Rx events */
    uint64_t ethRxEventCnt;

    /*! Count of ethernet Tx events */
    uint64_t ethTxEventCnt;

    /*! Count of ethernet host Tx events */
    uint64_t ethHostTxEventCnt;

    /*! Lookup count of hardware time stamp push events */
    uint64_t hwTsPushLookupCnt;

    /*! Lookup count of time stamp compare events */
    uint64_t cmpEventLookupCnt;

    /*! Lookup count of ethernet Rx events */
    uint64_t ethRxEventLookupCnt;

    /*! Lookup count of ethernet Tx events */
    uint64_t ethTxEventLookupCnt;

    /*! Lookup count of ethernet host Tx events */
    uint64_t ethHostTxEventLookupCnt;

    /*! Discard count of old hardware time stamp push events */
    uint64_t hwTsPushDiscardCnt;

    /*! Discard count of time stamp compare events */
    uint64_t cmpEventDiscardCnt;

    /*! Discard count of ethernet Rx events */
    uint64_t ethRxEventDiscardCnt;

    /*! Discard count of ethernet Tx events */
    uint64_t ethTxEventDiscardCnt;

    /*! Discard count of ethernet host Tx events */
    uint64_t ethHostTxEventDiscardCnt;
} CpswCpts_EventStats;

/*!
 * \brief CPSW CPTS object.
 */
typedef struct CpswCpts_Obj_s
{
    /*! EnetMod must be the first member */
    EnetMod_Obj enetMod;

    /*! Flag to indicate that a SW push event is already active */
    volatile bool tsPushInFifo;

    /*! Current time stamp value maintained by driver, gets updated at every TS_PUSH_EVENT */
    uint64_t tsVal;

    /*! Mask of the active Genf index */
    uint32_t activeGenfIdxMask;

    /*! Mask of the active Estf index */
    uint32_t activeEstfIdxMask;

    /*! Store the tsAddVal set during open. */
    uint8_t tsAddVal;

    /*! Notify callback function for registered stack */
    CpswCpts_EventNotifyCb eventNotifyCb;

    /*! Notify callback function argument for registered stack */
    void *eventNotifyCbArg;

    /*! Maximum hardware push instances supported in hardware */
    uint32_t hwPushCnt;

    /*! Hardware push event notify callback function pointers */
    CpswCpts_HwPushNotifyCb hwPushNotifyCb[CPSW_CPTS_HWPUSH_COUNT_MAX];

    /*! Hardware push event notify callback function arguments */
    void *hwPushNotifyCbArg[CPSW_CPTS_HWPUSH_COUNT_MAX];

    /*! Event memory object for ethernet Tx event */
    CpswCpts_EventMemObj ethTxEventPool;

    /*! Event memory object for ethernet Rx event */
    CpswCpts_EventMemObj ethRxEventPool;

    /*! Event memory object for hardware Ts Push event */
    CpswCpts_EventMemObj hwPushEventPool;

    /*! Event memory object for compare event */
    CpswCpts_EventMemObj cmpEventPool;

    /*! Event memory object for host Tx event */
    CpswCpts_EventMemObj hostTxEventPool;

    /*! Event statistics */
    CpswCpts_EventStats eventStats;
} CpswCpts_Obj;

/*!
 * \brief CPSW CPTS module handle.
 */
typedef CpswCpts_Obj *CpswCpts_Handle;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Open and initialize CPSW CPTS.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters
 * \param cfgSize   Size of the configuration parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t CpswCpts_open(EnetMod_Handle hMod,
                      Enet_Type enetType,
                      uint32_t instId,
                      const void *cfg,
                      uint32_t cfgSize);

/*!
 * \brief Rejoin a running CPSW CPTS module.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t CpswCpts_rejoin(EnetMod_Handle hMod,
                        Enet_Type enetType,
                        uint32_t instId);

/*!
 * \brief Run an IOCTL operation on CPSW CPTS.
 *
 * \param hMod         Enet Module handle
 * \param cmd          IOCTL command Id
 * \param prms         IOCTL parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t CpswCpts_ioctl(EnetMod_Handle hMod,
                       uint32_t cmd,
                       Enet_IoctlPrms *prms);

/*!
 * \brief Close CPSW CPTS.
 *
 * \param hMod         Enet Module handle
 */
void CpswCpts_close(EnetMod_Handle hMod);

/*!
 * \brief Saves and Close CPSW CPTS.
 *
 * \param hMod         Enet Module handle
 */
void CpswCpts_saveCtxt(EnetMod_Handle hMod);

/*!
 * \brief Restores and Open CPSW CPTS.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters
 * \param cfgSize   Size of the configuration parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t CpswCpts_restoreCtxt(EnetMod_Handle hMod,
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

#endif /* CPSW_CPTS_PRIV_H_ */
