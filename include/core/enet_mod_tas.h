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
 * \file  enet_mod_tas.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet Time Aware Shaper (TAS) module interface.
 */

/*!
 * \ingroup  DRV_ENET_MODULE
 * \defgroup ENET_MOD_TAS Enet Time Aware Shaper
 *
 * @{
 */

#ifndef ENET_MOD_TAS_H_
#define ENET_MOD_TAS_H_

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

/*! \brief Helper macro to create IOCTL commands for TAS module. */
#define ENET_TAS_PUBLIC_IOCTL(x)             (ENET_IOCTL_TYPE_PUBLIC | \
                                              ENET_IOCTL_TAS_BASE |    \
                                              ENET_IOCTL_MIN(x))

/*! \brief Helper macro to create a gate mask. */
#define ENET_TAS_GATE_MASK(tc7, tc6, tc5, tc4, tc3, tc2, tc1, tc0)               \
                                            (((((tc7) != 0U) ? 1U : 0U) << 7U) | \
                                             ((((tc6) != 0U) ? 1U : 0U) << 6U) | \
                                             ((((tc5) != 0U) ? 1U : 0U) << 5U) | \
                                             ((((tc4) != 0U) ? 1U : 0U) << 4U) | \
                                             ((((tc3) != 0U) ? 1U : 0U) << 3U) | \
                                             ((((tc2) != 0U) ? 1U : 0U) << 2U) | \
                                             ((((tc1) != 0U) ? 1U : 0U) << 1U) | \
                                             ((((tc0) != 0U) ? 1U : 0U) << 0U))

/*! \brief Maximum number of gate command entries in each list. */
#define ENET_TAS_MAX_CMD_LISTS               (16)

/*! \brief Maximum number of transmit queues supported by implementation. */
#define ENET_TAS_MAX_NUM_QUEUES              (8)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief TAS module IOCTL commands.
 */
typedef enum EnetTas_Ioctl_e
{
    /*!
     * \brief Get the hardware version of the TAS module.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetTas_GenericInArgs
     * - outArgs: #Enet_Version
     *
     * Type: Synchronous.
     */
    ENET_TAS_IOCTL_GET_VERSION = ENET_TAS_PUBLIC_IOCTL(0U),

    /*!
     * \brief Set the admin list parameters of the TAS module.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetTas_SetAdminListInArgs
     * - outArgs: None
     *
     * Type: Asynchronous.
     */
    ENET_TAS_IOCTL_SET_ADMIN_LIST = ENET_TAS_PUBLIC_IOCTL(1U),

    /*!
     * \brief Get the status of the operational list update.
     *
     * This IOCTL should be called after #ENET_TAS_IOCTL_SET_ADMIN_LIST
     * to verify admin list to operational list copy at specified time.
     * This IOCTL should be called until #ENET_TAS_OPER_LIST_UPDATED received
     * as output.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetTas_GenericInArgs
     * - outArgs: #EnetTas_OperStatus
     *
     * Type: Synchronous.
     */
    ENET_TAS_IOCTL_GET_OPER_LIST_STATUS = ENET_TAS_PUBLIC_IOCTL(2U),

    /*!
     * \brief Set the state of the TAS module.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetTas_SetStateInArgs
     * - outArgs: None
     *
     * Type: Asynchronous.
     */
    ENET_TAS_IOCTL_SET_STATE = ENET_TAS_PUBLIC_IOCTL(3U),

    /*!
     * \brief Get the state of the TAS module.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetTas_GenericInArgs
     * - outArgs: #EnetTas_TasState
     *
     * Type: Synchronous.
     */
    ENET_TAS_IOCTL_GET_STATE = ENET_TAS_PUBLIC_IOCTL(4U),

    /*!
     * \brief Get the admin list parameters of the TAS module.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetTas_GenericInArgs
     * - outArgs: #EnetTas_ControlList
     *
     * Type: Synchronous.
     */
    ENET_TAS_IOCTL_GET_ADMIN_LIST = ENET_TAS_PUBLIC_IOCTL(5U),

    /*!
     * \brief Get the operational list parameters of the TAS module.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetTas_GenericInArgs
     * - outArgs: #EnetTas_ControlList
     *
     * Type: Synchronous.
     */
    ENET_TAS_IOCTL_GET_OPER_LIST = ENET_TAS_PUBLIC_IOCTL(6U),

    /*!
     * \brief Get the TAS config change status parameters.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetTas_GenericInArgs
     * - outArgs: #EnetTas_ConfigStatus
     *
     * Type: Synchronous.
     */
    ENET_TAS_IOCTL_CONFIG_CHANGE_STATUS_PARAMS = ENET_TAS_PUBLIC_IOCTL(7U),
} EnetTas_Ioctl;

/*!
 * \brief Generic input args.
 */
typedef struct EnetTas_GenericInArgs_s
{
    /*! Port number. */
    Enet_MacPort macPort;
} EnetTas_GenericInArgs;

/*!
 * \brief TAS state types.
 */
typedef enum EnetTas_OperStatus_s
{
    /*! Operational list is not yet updated. */
    ENET_TAS_OPER_LIST_NOT_YET_UPDATED = 0,

    /*! Operational list is updated. */
    ENET_TAS_OPER_LIST_UPDATED = 1,
} EnetTas_OperStatus;

/*!
 * \brief TAS state types.
 */
typedef enum EnetTas_TasState_s
{
    /*! Idle. */
    ENET_TAS_DISABLE = 0,

    /*! Enable TAS. */
    ENET_TAS_ENABLE = 1,

    /*! Reset the state machine. */
    ENET_TAS_RESET = 2,
} EnetTas_TasState;

/*!
 * \brief Gate control list. See IEEE Std 802.1Q-2018 8.6.8.4.
 */
typedef struct EnetTas_GateCmdEntry_s
{
    /*! 32-bit time interval for the gate command. */
    uint32_t timeInterval;

    /*! 8-bit mask to signify which gate will be open. One bit for each gate. */
    uint8_t gateStateMask;
} EnetTas_GateCmdEntry;

/*!
 * \brief Max SDU table. See IEEE Std 802.1Q-2018 12.29.1.1.
 */
typedef struct EnetTas_MaxSDUTable_s
{
    /*! Maximum service data unit (SDU). One per queue. */
    uint16_t maxSDU[ENET_TAS_MAX_NUM_QUEUES];
} EnetTas_MaxSDUTable;

/*!
 * \brief Gate control list. See IEEE Std 802.1Q-2018 8.6.9.4 D3-1.
 */
typedef struct EnetTas_ControlList_s
{
    /*! Base time for the TAS gate operations. If a list is already active,
     *  then this time will be increased to align with the nearest start of cycle
     *  boundary. This is a 64-bit timestamp in nanoseconds. */
    uint64_t baseTime;

    /*! The actual gate list containing operations and delay associated. */
    EnetTas_GateCmdEntry gateCmdList[ENET_TAS_MAX_CMD_LISTS];

    /*! The list repeats after this time. */
    uint64_t cycleTime;

    /*! Length of the list or number of entries. */
    uint8_t listLength;

    /*! See \ref EnetTas_MaxSDUTable. */
    EnetTas_MaxSDUTable sduTable;
} EnetTas_ControlList;

/*!
 * \brief Config state machine variables. See IEEE Std 802.1Q-2018 8.6.8.4.
 */
typedef struct EnetTas_ConfigStatus_s
{
    /*! New list is copied at this time. */
    uint64_t configChangeTime;

    /*! Config change error counter, incremented if admin->baseTime < current time
     *  and TAS_enabled is true. */
    uint32_t configChangeErrorCounter;

    /*! True if list update is pending. */
    uint8_t configPending;

    /*! Set to true when application trigger updating of admin list to active list,
     *  cleared when configChangeTime is updated. */
    uint8_t configChange;
} EnetTas_ConfigStatus;

/*!
 * \brief Input args for #ENET_TAS_IOCTL_SET_ADMIN_LIST commands.
 */
typedef struct EnetTas_SetAdminListInArgs_s
{
    /*! Port number. */
    Enet_MacPort macPort;

    /*! Admin list. */
    EnetTas_ControlList adminList;
} EnetTas_SetAdminListInArgs;

/*!
 * \brief Input args for #ENET_TAS_IOCTL_SET_STATE commands.
 */
typedef struct EnetTas_SetStateInArgs_s
{
    /*! Port number. */
    Enet_MacPort macPort;

    /*! TAS state. */
    EnetTas_TasState state;
} EnetTas_SetStateInArgs;

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

#endif /* ENET_MOD_TAS_H_ */

/*! @} */
