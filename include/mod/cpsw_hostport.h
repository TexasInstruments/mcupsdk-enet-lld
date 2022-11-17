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
 * \file  cpsw_hostport.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        CPSW host port module.
 */

/*!
 * \ingroup  ENET_MOD_HOSTPORT
 * \defgroup CPSW_HOSTPORT_MOD CPSW host port
 *
 * The CPSW statistics module implements only the IOCTLs defined in the generic
 * \ref ENET_MOD_STATS API.  There are no additional CPSW-specific IOCTLs.
 *
 * The CPSW host port module provides additional IOCTL commands than those
 * supported by the generci \ref ENET_MOD_HOSTPORT API set.
 *
 * CPSW host port clocks:
 * - CPSW_CPPI_CLK - CPSW main clock.
 *
 * Compile-time configuration:
 * - #ENET_CFG_CPSW_HOSTPORT_TRAFFIC_SHAPING - Traffic shaping (rate limit) support.
 *
 * @{
 */

#ifndef CPSW_HOSTPORT_H_
#define CPSW_HOSTPORT_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <include/core/enet_mod_hostport.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Helper macro to create IOCTL commands for CPSW host port module. */
#define CPSW_HOSTPORT_PUBLIC_IOCTL(x)         (ENET_IOCTL_TYPE_PUBLIC |   \
                                               ENET_IOCTL_HOSTPORT_BASE | \
                                               ENET_IOCTL_PER_CPSW |      \
                                               ENET_IOCTL_MIN(x))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief CPSW MAC port IOCTL commands.
 */
typedef enum CpswHostPort_Ioctl_s
{
    /*!
     * \brief Get host port's FIFO statistics.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: #CpswHostPort_FifoStats
     */
    CPSW_HOSTPORT_IOCTL_GET_FIFO_STATS = CPSW_HOSTPORT_PUBLIC_IOCTL(0U),

    /*!
     * \brief Get host port flow Id offset.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: uint32_t
     */
    CPSW_HOSTPORT_GET_FLOW_ID_OFFSET = CPSW_HOSTPORT_PUBLIC_IOCTL(1U),
} CpswHostPort_Ioctl;


/*!
 * \brief FIFO related statistics of the host port.
 */
typedef struct CpswHostPort_FifoStats_s
{
    /*! Max throughput of the CPPI FIFO into the CPSW_NU */
    uint32_t rxThroughputRate;

    /*! Number of blocks allocated to the FIFO logical TX queues */
    uint32_t txBlockCount;

    /*! Number of blocks allocated in the RX FIFO Express MAC
     *  If IET is not enabled all traffic is express traffic
     */
    uint32_t rxBlockCountExpress;

    /*! Number of blocks allocated in the RX FIFO Preempt MAC */
    uint32_t rxBlockCountPreempt;

    /*! Whether a FIFO priority has one or more queued packets or not */
    bool txActiveFifo[ENET_PRI_NUM];
} CpswHostPort_FifoStats;

/*!
 * \brief Host port module configuration parameters.
 */
typedef struct CpswHostPort_Cfg_s
{
    /*! Type of CRC on all port 0 egress, regardless of the CRC type in Ethernet
     *  port ingress */
    Enet_CrcType crcType;

    /*! Whether or not CRC is removed on port 0 egress */
    bool removeCrc;

    /*! Whether short packets (ingress) are padded to 64-bytes or dropped */
    bool padShortPacket;

    /*! Whether packets with CRC errors (ingress) are dropped or transferred to
     *  the destination ports */
    bool passCrcErrors;

    /*! Max length of a received frame on ingress, including VLAN */
    uint32_t rxMtu;

    /*! Whether priority tagged packets should be passed unchanged (if set to
     *  true) or replaced with port's VID (if set to false) */
    bool passPriorityTaggedUnchanged;

    /*! Enable checksum offload feature which allows TCP checksum computation
     *  to be offloaded to the CPSW for frames transmitted by any core.
     *  The Protocol specific info needs to be populated in the descriptor
     *  to indicate the location in the packet where the computed checksum
     *  should be inserted */
    bool csumOffloadEn;

    /*! RX VLAN remap controls whether the hardware switch priority for VLAN
     *  tagged or priority tagged packets is determined from CPPI thread number
     *  (remap disabled) or via \ref ENET_HOSTPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP
     *  (remap enabled) */
    bool rxVlanRemapEn;

    /*! RX DSCP IPv4 remap controls whether the hardware switch priority for
     *  IPv4 packets is determined from CPPI thread number (remap disabled) or
     *  via \ref ENET_HOSTPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP (remap enabled) */
    bool rxDscpIPv4RemapEn;

    /*! RX DSCP IPv6 remap controls whether the hardware switch priority for
     *  IPv6 packets is determined from CPPI thread number (remap disabled) or
     *  via \ref ENET_HOSTPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP (remap enabled) */
    bool rxDscpIPv6RemapEn;

    /*! Port VLAN configuration */
    EnetPort_VlanCfg vlanCfg;

    /*! Ingress priority type */
    EnetHostPort_IngressPriorityType rxPriorityType;

    /*! Egress priority type */
    EnetPort_EgressPriorityType txPriorityType;
} CpswHostPort_Cfg;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize CPSW host port configuration parameters.
 *
 * \param hostPortCfg   Configuration parameters to be initialized
 */
void CpswHostPort_initCfg(CpswHostPort_Cfg *hostPortCfg);

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

#endif /* CPSW_HOSTPORT_H_ */

/*! @} */
