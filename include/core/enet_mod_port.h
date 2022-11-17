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
 * \file  enet_mod_port.h
 *
 * \brief This file contains the type definitions and helper macros that are
 *        common for host and MAC ports.
 */

#ifndef ENET_MOD_PORT_H_
#define ENET_MOD_PORT_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief VLAN configuration parameters.
 */
typedef struct EnetPort_VlanCfg_s
{
    /*! Port VLAN priority */
    uint32_t portPri;

    /*! Port CFI bit */
    uint32_t portCfi;

    /*! Port VLAN ID */
    uint32_t portVID;
} EnetPort_VlanCfg;

/*!
 * \brief Egress priority type.
 */
typedef enum EnetPort_EgressPriorityType_e
{
    /*! Fixed priority */
    ENET_EGRESS_PRI_TYPE_FIXED = 0U,

    /*! Escalate priority */
    ENET_EGRESS_PRI_TYPE_ESCALATE,
} EnetPort_EgressPriorityType;

/*!
 * \brief Traffic shaping parameters (committed and excess rates).
 */
typedef struct EnetPort_TrafficShapingRates_s
{
    /*! Committed Information Rate (CIR) in bits-per-second */
    uint64_t committedRateBitsPerSec;

    /*! Excess Information Rate (EIR) in bits-per-second */
    uint64_t excessRateBitsPerSec;
} EnetPort_TrafficShapingRates;

/*!
 * \brief Traffic shaping configuration for all priorities.
 */
typedef struct EnetPort_TrafficShapingCfg_s
{
    /*! Traffic shaping rates (committed and excess rates) for all priorities */
    EnetPort_TrafficShapingRates rates[ENET_PRI_NUM];
} EnetPort_TrafficShapingCfg;

/*!
 * \brief Priority map.
 *
 * It can be used for priority regeneration as well as for QoS egress priority
 * remapping.
 */
typedef struct EnetPort_PriorityMap_s
{
    /*! Packet priority map. Map index is the input priority, map value is
     *  the output priority. */
    uint32_t priorityMap[ENET_PRI_NUM];
} EnetPort_PriorityMap;

/*!
 * \brief DSCP priority map.
 */
typedef struct EnetPort_DscpPriorityMap_s
{
    /*! DSCP IPv4/IPv6 to priority map */
    uint32_t tosMap[ENET_TOS_PRI_NUM];

    /*! Enable IPv4 based DSCP */
    bool dscpIPv4En;

    /*! Enable IPv6 based DSCP */
    bool dscpIPv6En;
} EnetPort_DscpPriorityMap;

/*!
 * \brief Maximum packet lengths for TX and RX.
 */
typedef struct EnetPort_MaxLen_s
{
    /*! Per-priority Maximum Transmit Unit (MTU) */
    uint32_t mtu[ENET_PRI_NUM];

    /*! Maximum Receive Unit (MRU) */
    uint32_t mru;
} EnetPort_MaxLen;

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

#endif /* ENET_MOD_PORT_H_ */
