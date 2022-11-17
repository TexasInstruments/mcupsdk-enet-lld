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
 * \file  enet_mod_hostport.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet Hostport module interface.
 */

/*!
 * \ingroup  DRV_ENET_MODULE
 * \defgroup ENET_MOD_HOSTPORT Enet Host Port
 *
 * @{
 */

#ifndef ENET_MOD_HOSTPORT_H_
#define ENET_MOD_HOSTPORT_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_mod.h>
#include <include/core/enet_mod_port.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Helper macro to create IOCTL commands for host port module */
#define ENET_HOSTPORT_PUBLIC_IOCTL(x)         (ENET_IOCTL_TYPE_PUBLIC |   \
                                               ENET_IOCTL_HOSTPORT_BASE | \
                                               ENET_IOCTL_MIN(x))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief Host port IOCTL commands.
 */
typedef enum EnetHostPort_Ioctl_e
{
    /*!
     * \brief Get the hardware version of the host port module.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: #Enet_Version
     */
    ENET_HOSTPORT_IOCTL_GET_VERSION = ENET_HOSTPORT_PUBLIC_IOCTL(0U),

    /*!
     * \brief Print host port registers.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: None
     */
    ENET_HOSTPORT_IOCTL_PRINT_REGS = ENET_HOSTPORT_PUBLIC_IOCTL(1U),

    /*!
     * \brief Enable host port.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: None
     */
    ENET_HOSTPORT_IOCTL_ENABLE = ENET_HOSTPORT_PUBLIC_IOCTL(2U),

    /*!
     * \brief Disable host port.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: None
     */
    ENET_HOSTPORT_IOCTL_DISABLE = ENET_HOSTPORT_PUBLIC_IOCTL(3U),

    /*!
     * \brief Set ingress DSCP priority (TOS) map.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPort_DscpPriorityMap
     * - outArgs: None
     */
    ENET_HOSTPORT_IOCTL_SET_INGRESS_DSCP_PRI_MAP = ENET_HOSTPORT_PUBLIC_IOCTL(4U),

    /*!
     * \brief Get ingress DSCP priority (TOS) map.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: #EnetPort_DscpPriorityMap
     */
    ENET_HOSTPORT_IOCTL_GET_INGRESS_DSCP_PRI_MAP = ENET_HOSTPORT_PUBLIC_IOCTL(5U),

    /*!
     * \brief Set VLAN priority regeneration map.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPort_PriorityMap
     * - outArgs: None
     */
    ENET_HOSTPORT_IOCTL_SET_PRI_REGEN_MAP = ENET_HOSTPORT_PUBLIC_IOCTL(6U),

    /*!
     * \brief Get VLAN priority regeneration map.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: #EnetPort_PriorityMap
     */
    ENET_HOSTPORT_IOCTL_GET_PRI_REGEN_MAP = ENET_HOSTPORT_PUBLIC_IOCTL(7U),

    /*!
     * \brief Set QoS egress priority map.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPort_PriorityMap
     * - outArgs: None
     */
    ENET_HOSTPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP = ENET_HOSTPORT_PUBLIC_IOCTL(8U),

    /*!
     * \brief Get QoS egress priority map.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: #EnetPort_PriorityMap
     */
    ENET_HOSTPORT_IOCTL_GET_EGRESS_QOS_PRI_MAP = ENET_HOSTPORT_PUBLIC_IOCTL(9U),

    /*!
     * \brief Enable ingress traffic shaping configuration.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPort_TrafficShapingCfg
     * - outArgs: None
     */
    ENET_HOSTPORT_IOCTL_ENABLE_INGRESS_TRAFFIC_SHAPING = ENET_HOSTPORT_PUBLIC_IOCTL(10U),

    /*!
     * \brief Disable ingress traffic shaping configuration.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: None
     */
    ENET_HOSTPORT_IOCTL_DISABLE_INGRESS_TRAFFIC_SHAPING = ENET_HOSTPORT_PUBLIC_IOCTL(11U),

    /*!
     * \brief Get ingress traffic shaping configuration.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: #EnetPort_TrafficShapingCfg
     */
    ENET_HOSTPORT_IOCTL_GET_INGRESS_TRAFFIC_SHAPING = ENET_HOSTPORT_PUBLIC_IOCTL(12U),

    /*!
     * \brief Get MRU and MTU.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: #EnetPort_MaxLen
     */
    ENET_HOSTPORT_IOCTL_GET_MAXLEN = ENET_HOSTPORT_PUBLIC_IOCTL(13U),

    /*!
     * \brief Check if checksum offload is enabled.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: bool
     */
    ENET_HOSTPORT_IS_CSUM_OFFLOAD_ENABLED = ENET_HOSTPORT_PUBLIC_IOCTL(14U),
} EnetHostPort_Ioctl;

/*!
 * \brief Port priority type.
 */
typedef enum EnetHostPort_IngressPriorityType_e
{
    /*! Fixed priority */
    ENET_INGRESS_PRI_TYPE_FIXED = 0U,

    /*! Round-robin priority */
    ENET_INGRESS_PRI_TYPE_RR,
} EnetHostPort_IngressPriorityType;

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

#endif /* ENET_MOD_HOSTPORT_H_ */

/*! @} */
