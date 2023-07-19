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
 * \file  enet_mod_timesync.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet Time Synchronization module interface.
 */

/*!
 * \ingroup  DRV_ENET_MODULE
 * \defgroup ENET_MOD_TIMESYNC Enet Time Synchronization
 *
 * @{
 */

#ifndef ENET_MOD_TIMESYNC_H_
#define ENET_MOD_TIMESYNC_H_

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

/*! \brief Helper macro to create IOCTL commands for TIMESYNC module. */
#define ENET_TIMESYNC_PUBLIC_IOCTL(x)             (ENET_IOCTL_TYPE_PUBLIC |   \
                                                   ENET_IOCTL_TIMESYNC_BASE | \
                                                   ENET_IOCTL_MIN(x))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief TimeSync module IOCTL commands.
 */
typedef enum EnetTimeSync_Ioctl_e
{
    /*!
     * \brief Get the hardware version of the TimeSync module.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: #Enet_Version
     */
    ENET_TIMESYNC_IOCTL_GET_VERSION = ENET_TIMESYNC_PUBLIC_IOCTL(0U),

    /*!
     * \brief Print TimeSync registers.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: None
     */
    ENET_TIMESYNC_IOCTL_PRINT_REGS = ENET_TIMESYNC_PUBLIC_IOCTL(1U),

    /*!
     * \brief Print TimeSync event statistics. They could be either hardware or
     *        software based statistics.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: None
     */
    ENET_TIMESYNC_IOCTL_PRINT_STATS = ENET_TIMESYNC_PUBLIC_IOCTL(2U),

    /*!
     * \brief Get current timestamp.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: uint64_t
     */
    ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP = ENET_TIMESYNC_PUBLIC_IOCTL(3U),

    /*!
     * \brief Set timestamp value.
     *
     * IOCTL parameters:
     * -  inArgs: EnetTimeSync_setTimestamp
     * - outArgs: None
     */
    ENET_TIMESYNC_IOCTL_SET_TIMESTAMP = ENET_TIMESYNC_PUBLIC_IOCTL(4U),

    /*!
     * \brief Adjust the timestamp value.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetTimeSync_TimestampAdj
     * - outArgs: None
     */
    ENET_TIMESYNC_IOCTL_ADJUST_TIMESTAMP = ENET_TIMESYNC_PUBLIC_IOCTL(5U),

    /*!
     * \brief Get Ethernet RX timestamp.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetTimeSync_GetEthTimestampInArgs
     * - outArgs: uint64_t
     */
    ENET_TIMESYNC_IOCTL_GET_ETH_RX_TIMESTAMP = ENET_TIMESYNC_PUBLIC_IOCTL(6U),

    /*!
     * \brief Get Ethernet TX timestamp. For CPSW the driver maintains a pool of the timestamps that
     *  are captured by CPTS and gives back the matched entry. But, In case of ICSSG, Driver doesn't maintain
     *  any software pool, it returns back the top entry in the queue if it matches the seq id passed.
     *
     *
     * IOCTL parameters:
     * -  inArgs: #EnetTimeSync_GetEthTimestampInArgs
     * - outArgs: uint64_t
     */
    ENET_TIMESYNC_IOCTL_GET_ETH_TX_TIMESTAMP = ENET_TIMESYNC_PUBLIC_IOCTL(7U),

    /*!
     * \brief Reset TimeSync module.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: None
     */
    ENET_TIMESYNC_IOCTL_RESET = ENET_TIMESYNC_PUBLIC_IOCTL(8U),

    /*!
     * \brief ICSSG Set timestamp operation completed.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: None
     */
    ENET_TIMESYNC_IOCTL_SET_TIMESTAMP_COMPLETE = ENET_TIMESYNC_PUBLIC_IOCTL(9U),
} EnetTimeSync_Ioctl;

/*!
 * \brief TimeSync Ethernet message type.
 */
typedef enum EnetTimeSync_MsgType_e
{
    /*! TimeSync sync message */
    ENET_TIMESYNC_MESSAGE_SYNC                  = 0x00U,

    /*! TimeSync delay request message */
    ENET_TIMESYNC_MESSAGE_DELAY_REQ             = 0x01U,

    /*! TimeSync Pdelay request message */
    ENET_TIMESYNC_MESSAGE_PDELAY_REQ            = 0x02U,

    /*! TimeSync Pdelay response message */
    ENET_TIMESYNC_MESSAGE_PDELAY_RESP           = 0x03U,

    /*! TimeSync follow up message */
    ENET_TIMESYNC_MESSAGE_FOLLOW_UP             = 0x08U,

    /*! TimeSync delay response message */
    ENET_TIMESYNC_MESSAGE_DELAY_RESP            = 0x09U,

    /*! TimeSync Pdelay response followup message */
    ENET_TIMESYNC_MESSAGE_PDELAY_RESP_FOLLOW_UP = 0x0AU,

    /*! TimeSync announce message */
    ENET_TIMESYNC_MESSAGE_ANNOUNCE              = 0x0BU,

    /*! TimeSync signaling message */
    ENET_TIMESYNC_MESSAGE_SIGNALING             = 0x0CU,

    /*! TimeSync management message */
    ENET_TIMESYNC_MESSAGE_MANAGEMENT            = 0x0DU,

    /*! TimeSync invalid message */
    ENET_TIMESYNC_MESSAGE_INVALID               = -1,
} EnetTimeSync_MsgType;

/*!
 * \brief TimeSync adjustment mode.
 */
typedef enum EnetTimeSync_AdjMode_e
{
    /*! Adjustment value is set to zero */
    ENET_TIMESYNC_ADJMODE_DISABLE = 0U,

    /*! Adjustment value is calculated for parts per million */
    ENET_TIMESYNC_ADJMODE_PPM,

    /*! Adjustment value is calculated for parts per hour */
    ENET_TIMESYNC_ADJMODE_PPH,
} EnetTimeSync_AdjMode;

/*!
 * \brief TimeSync adjustment direction.
 */
typedef enum EnetTimeSync_AdjDir_e
{
    /*! Increase timestamp with adjustment value */
    ENET_TIMESYNC_ADJDIR_INCREASE = 0U,

    /*! Decrease timestamp with adjustment value */
    ENET_TIMESYNC_ADJDIR_DECREASE,
} EnetTimeSync_AdjDir;

/*!
 * \brief Timestamp set.
 */
typedef struct EnetTimeSync_setTimestamp_s
{
    /*! timestamp value in nano seconds unit to set */
    uint64_t tsLoadVal;

    /*! mode of the clock. Applicable only to ICSSG peripheral.
     * Says whether given value  tsLoadVal is absolute or related.
     * 0: absolute, 1: relative */
    uint8_t clkMode;

    /*Sign of the clock. Applicable only to ICSSG peripheral. */
    uint8_t clkSign;
} EnetTimeSync_setTimestamp;

/*!
 * \brief Timestamp adjustment.
 */
typedef struct EnetTimeSync_TimestampAdj_s
{
    /*! Adjustment value for timestamp */
    int32_t adjValInNsecs;

    /*! Interval to apply the adjustment */
    uint32_t intervalInNsecs;
} EnetTimeSync_TimestampAdj;

/*!
 * \brief Input args for #ENET_TIMESYNC_IOCTL_GET_ETH_RX_TIMESTAMP and
 *        #ENET_TIMESYNC_IOCTL_GET_ETH_TX_TIMESTAMP commands.
 */
typedef struct EnetTimeSync_GetEthTimestampInArgs_s
{
    /*! TimeSync event message type */
    EnetTimeSync_MsgType msgType;

    /*! Sequence id of the event */
    uint32_t seqId;

    /*! MAC port number corresponding to the event */
    uint32_t portNum;

    /*! Domain number of the event */
    uint32_t domain;
} EnetTimeSync_GetEthTimestampInArgs;

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

#endif /* ENET_MOD_TIMESYNC_H_ */

/*! @} */
