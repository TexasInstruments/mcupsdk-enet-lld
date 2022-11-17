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
 * \file  cpsw_stats.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        CPSW Statistics module interface.
 */

/*!
 * \ingroup  ENET_MOD_STATS
 * \defgroup CPSW_STATS_MOD CPSW Statistics
 *
 * The CPSW statistics module implements only the IOCTLs defined in the generic
 * \ref ENET_MOD_STATS API.  There are no additional CPSW-specific IOCTLs.
 *
 * The statistics structures are CPSW specific and may vary depending on the
 * CPSW instance:
 * - CPSW_2G statistics:
 *     - Host port: \ref CpswStats_HostPort_2g
 *     - MAC port: \ref CpswStats_MacPort_2g

 * - CPSW_5G or CPSW9G statistics:
 *     - Host port: \ref CpswStats_HostPort_Ng
 *     - MAC ports: \ref CpswStats_MacPort_Ng
 *
 * Interrupts:
 * - CPSW_STATS_PEND - CPSW half-rollover statistics interrupt. Peripheral driver
 *   must call CPSW_STATS_IOCTL_SYNC command to handle the interrupt which
 *   reads and decrements all statistics values, including the one(s) that had
 *   reached the half-rollover condition.
 *
 * @{
 */

#ifndef CPSW_STATS_H_
#define CPSW_STATS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <include/core/enet_mod_stats.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Number of elements in a statistics block. */
#define CPSW_STATS_BLOCK_ELEM_NUM             (128U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief CPSW 2G host port statistics.
 *
 * Statistics values reported by the CPSW hardware for the host port.
 */
typedef struct CpswStats_HostPort_2g_s
{
    /*! Total number of good frames received */
    uint64_t rxGoodFrames;

    /*! Total number of good multicast frames received */
    uint64_t rxBcastFrames;

    /*! Total number of good multicast frames received */
    uint64_t rxMcastFrames;

    /*! Reserved */
    uint64_t reserved4;

    /*! Total number of CRC error frames received */
    uint64_t rxCrcErrors;

    /*! Reserved */
    uint64_t reserved6;

    /*! Total number of oversized frames received */
    uint64_t rxOversizedFrames;

    /*! Reserved */
    uint64_t reserved8;

    /*! Total number of undersized frames received */
    uint64_t rxUndersizedFrames;

    /*! Total number of fragmented frames received */
    uint64_t rxFragments;

    /*! Total number of frames dropped by the ALE */
    uint64_t aleDrop;

    /*! Total number of overrun frames dropped by the ALE */
    uint64_t aleOverrunDrop;

    /*! Total number of received bytes in good frames */
    uint64_t rxOctets;

    /*! Total number of good frames transmitted */
    uint64_t txGoodFrames;

    /*! Total number of good broadcast frames transmitted */
    uint64_t txBcastFrames;

    /*! Total number of good multicast frames transmitted */
    uint64_t txMcastFrames;

    /*! Reserved */
    uint64_t reserved17to25[9U];

    /*! Total number of bytes in all good frames transmitted */
    uint64_t txOctets;

    /*! Total number of 64-byte frames received and transmitted */
    uint64_t octetsFrames64;

    /*! Total number of frames of size 65 to 127 bytes received and transmitted */
    uint64_t octetsFrames65to127;

    /*! Total number of frames of size 128 to 255 bytes received and transmitted */
    uint64_t octetsFrames128to255;

    /*! Total number of frames of size 256 to 511 bytes received and transmitted */
    uint64_t octetsFrames256to511;

    /*! Total number of frames of size 512 to 1023 bytes received and transmitted */
    uint64_t octetsFrames512to1023;

    /*! Total number of frames of size 1024 or greater transmitted */
    uint64_t octetsFrames1024;

    /*! Total number of bytes received and transmitted */
    uint64_t netOctets;

    /*! Receive bottom of FIFO drop */
    uint64_t rxBottomOfFifoDrop;

    /*! Total number of dropped frames received due to portmask */
    uint64_t portMaskDrop;

    /*! Receive top of FIFO drop */
    uint64_t rxTopOfFifoDrop;

    /*! Total number of frames dropped due to ALE rate limiting */
    uint64_t aleRateLimitDrop;

    /*! Total number of dropped frames due to ALE VID ingress */
    uint64_t aleVidIngressDrop;

    /*! Total number of dropped frames due to DA = SA */
    uint64_t aleDAEqSADrop;

    /*! Total number of dropped frames due to ALE block mode */
    uint64_t aleBlockDrop;

    /*! Total number of dropped frames due to ALE secure mode */
    uint64_t aleSecureDrop;

    /*! Total number of dropped frames due to ALE authentication */
    uint64_t aleAuthDrop;

    /*! ALE receive unknown unicast */
    uint64_t aleUnknownUcast;

    /*! ALE receive unknown unicast bytecount */
    uint64_t aleUnknownUcastBcnt;

    /*! ALE receive unknown multicast */
    uint64_t aleUnknownMcast;

    /*! ALE receive unknown multicast bytecount */
    uint64_t aleUnknownMcastBcnt;

    /*! ALE receive unknown broadcast */
    uint64_t aleUnknownBcast;

    /*! ALE receive unknown broadcast bytecount */
    uint64_t aleUnknownBcastBcnt;

    /*! ALE policer matched */
    uint64_t alePolicyMatch;

    /*! ALE policer matched and condition red */
    uint64_t alePolicyMatchRed;

    /*! ALE policer matched and condition yellow */
    uint64_t alePolicyMatchYellow;

    /*! Reserved */
    uint64_t reserved52to95[44U];

    /*! Transmit memory protect CRC error */
    uint64_t txMemProtectError;

    /*! Reserved */
    uint64_t reserved97to128[32U];
} CpswStats_HostPort_2g;

/*!
 * \brief CPSW 2G MAC port statistics.
 *
 * Statistics values reported by the CPSW hardware for the Ethernet MAC port.
 */
typedef struct CpswStats_MacPort_2g_s
{
    /*! Total number of good frames received */
    uint64_t rxGoodFrames;

    /*! Total number of good multicast frames received */
    uint64_t rxBcastFrames;

    /*! Total number of good multicast frames received */
    uint64_t rxMcastFrames;

    /*! Total number of pause frames received */
    uint64_t rxPauseFrames;

    /*! Total number of CRC error frames received */
    uint64_t rxCrcErrors;

    /*! Total number of alignment/code errors received */
    uint64_t rxAlignCodeErrors;

    /*! Total number of oversized frames received */
    uint64_t rxOversizedFrames;

    /*! Total number of jabber frames received */
    uint64_t rxJabberFrames;

    /*! Total number of undersized frames received */
    uint64_t rxUndersizedFrames;

    /*! Total number of fragmented frames received */
    uint64_t rxFragments;

    /*! Total number of frames dropped by the ALE */
    uint64_t aleDrop;

    /*! Total number of overrun frames dropped by the ALE */
    uint64_t aleOverrunDrop;

    /*! Total number of received bytes in good frames */
    uint64_t rxOctets;

    /*! Total number of good frames transmitted */
    uint64_t txGoodFrames;

    /*! Total number of good broadcast frames transmitted */
    uint64_t txBcastFrames;

    /*! Total number of good multicast frames transmitted */
    uint64_t txMcastFrames;

    /*! Total number of pause frames transmitted */
    uint64_t txPauseFrames;

    /*! Total number of deferred frames transmitted */
    uint64_t txDeferredFrames;

    /*! Total number of transmitted frames experiencing a collsion */
    uint64_t txCollisionFrames;

    /*! Total number of transmitted frames experiencing a single collision */
    uint64_t txSingleCollFrames;

    /*! Total number of transmitted frames experiencing multiple collisions */
    uint64_t txMultipleCollFrames;

    /*! Total number of transmitted frames abandoned due to excessive collisions */
    uint64_t txExcessiveCollFrames;

    /*! Total number of transmitted frames abandoned due to a late collision */
    uint64_t txLateCollFrames;

    /*! Total number of receive inter-packet gap errors (10G only) */
    uint64_t rxIPGError;

    /*! Total number of transmitted frames that experienced a carrier loss */
    uint64_t txCarrierSenseErrors;

    /*! Total number of bytes in all good frames transmitted */
    uint64_t txOctets;

    /*! Total number of 64-byte frames received and transmitted */
    uint64_t octetsFrames64;

    /*! Total number of frames of size 65 to 127 bytes received and transmitted */
    uint64_t octetsFrames65to127;

    /*! Total number of frames of size 128 to 255 bytes received and transmitted */
    uint64_t octetsFrames128to255;

    /*! Total number of frames of size 256 to 511 bytes received and transmitted */
    uint64_t octetsFrames256to511;

    /*! Total number of frames of size 512 to 1023 bytes received and transmitted */
    uint64_t octetsFrames512to1023;

    /*! Total number of frames of size 1024 or greater transmitted */
    uint64_t octetsFrames1024;

    /*! Total number of bytes received and transmitted */
    uint64_t netOctets;

    /*! Receive bottom of FIFO drop */
    uint64_t rxBottomOfFifoDrop;

    /*! Total number of dropped frames received due to portmask */
    uint64_t portMaskDrop;

    /*! Receive top of FIFO drop */
    uint64_t rxTopOfFifoDrop;

    /*! Total number of frames dropped due to ALE rate limiting */
    uint64_t aleRateLimitDrop;

    /*! Total number of dropped frames due to ALE VID ingress */
    uint64_t aleVidIngressDrop;

    /*! Total number of dropped frames due to DA = SA */
    uint64_t aleDAEqSADrop;

    /*! Total number of dropped frames due to ALE block mode */
    uint64_t aleBlockDrop;

    /*! Total number of dropped frames due to ALE secure mode */
    uint64_t aleSecureDrop;

    /*! Total number of dropped frames due to ALE authentication */
    uint64_t aleAuthDrop;

    /*! ALE receive unknown unicast */
    uint64_t aleUnknownUcast;

    /*! ALE receive unknown unicast bytecount */
    uint64_t aleUnknownUcastBcnt;

    /*! ALE receive unknown multicast */
    uint64_t aleUnknownMcast;

    /*! ALE receive unknown multicast bytecount */
    uint64_t aleUnknownMcastBcnt;

    /*! ALE receive unknown broadcast */
    uint64_t aleUnknownBcast;

    /*! ALE receive unknown broadcast bytecount */
    uint64_t aleUnknownBcastBcnt;

    /*! ALE policer matched */
    uint64_t alePolicyMatch;

    /*! ALE policer matched and condition red */
    uint64_t alePolicyMatchRed;

    /*! ALE policer matched and condition yellow */
    uint64_t alePolicyMatchYellow;

    /*! Reserved */
    uint64_t reserved52to95[44U];

    /*! Transmit memory protect CRC error */
    uint64_t txMemProtectError;

    /*! Ethernet port priority packet count */
    uint64_t txPri[8U];

    /*! Ethernet port priority packet byet count */
    uint64_t txPriBcnt[8U];

    /*! Ethernet port priority packet drop count */
    uint64_t txPriDrop[8U];

    /*! Ethernet port priority packet drop byte count */
    uint64_t txPriDropBcnt[8U];
} CpswStats_MacPort_2g;

/*!
 * \brief CPSW 5G or 9G host port statistics.
 *
 * Statistics values reported by the CPSW hardware for the host port.
 */
typedef struct CpswStats_HostPort_Ng_s
{
    /*! Total number of good frames received */
    uint64_t rxGoodFrames;

    /*! Total number of good multicast frames received */
    uint64_t rxBcastFrames;

    /*! Total number of good multicast frames received */
    uint64_t rxMcastFrames;

    /*! Reserved */
    uint64_t reserved4;

    /*! Total number of CRC error frames received */
    uint64_t rxCrcErrors;

    /*! Reserved */
    uint64_t reserved6;

    /*! Total number of oversized frames received */
    uint64_t rxOversizedFrames;

    /*! Reserved */
    uint64_t reserved8;

    /*! Total number of undersized frames received */
    uint64_t rxUndersizedFrames;

    /*! Total number of fragmented frames received */
    uint64_t rxFragments;

    /*! Total number of frames dropped by the ALE */
    uint64_t aleDrop;

    /*! Total number of overrun frames dropped by the ALE */
    uint64_t aleOverrunDrop;

    /*! Total number of received bytes in good frames */
    uint64_t rxOctets;

    /*! Total number of good frames transmitted */
    uint64_t txGoodFrames;

    /*! Total number of good broadcast frames transmitted */
    uint64_t txBcastFrames;

    /*! Total number of good multicast frames transmitted */
    uint64_t txMcastFrames;

    /*! Reserved */
    uint64_t reserved17to25[9U];

    /*! Total number of bytes in all good frames transmitted */
    uint64_t txOctets;

    /*! Total number of 64-byte frames received and transmitted */
    uint64_t octetsFrames64;

    /*! Total number of frames of size 65 to 127 bytes received and transmitted */
    uint64_t octetsFrames65to127;

    /*! Total number of frames of size 128 to 255 bytes received and transmitted */
    uint64_t octetsFrames128to255;

    /*! Total number of frames of size 256 to 511 bytes received and transmitted */
    uint64_t octetsFrames256to511;

    /*! Total number of frames of size 512 to 1023 bytes received and transmitted */
    uint64_t octetsFrames512to1023;

    /*! Total number of frames of size 1024 or greater transmitted */
    uint64_t octetsFrames1024;

    /*! Total number of bytes received and transmitted */
    uint64_t netOctets;

    /*! Receive bottom of FIFO drop */
    uint64_t rxBottomOfFifoDrop;

    /*! Total number of dropped frames received due to portmask */
    uint64_t portMaskDrop;

    /*! Receive top of FIFO drop */
    uint64_t rxTopOfFifoDrop;

    /*! Total number of frames dropped due to ALE rate limiting */
    uint64_t aleRateLimitDrop;

    /*! Total number of dropped frames due to ALE VID ingress */
    uint64_t aleVidIngressDrop;

    /*! Total number of dropped frames due to DA = SA */
    uint64_t aleDAEqSADrop;

    /*! Total number of dropped frames due to ALE block mode */
    uint64_t aleBlockDrop;

    /*! Total number of dropped frames due to ALE secure mode */
    uint64_t aleSecureDrop;

    /*! Total number of dropped frames due to ALE authentication */
    uint64_t aleAuthDrop;

    /*! ALE receive unknown unicast */
    uint64_t aleUnknownUcast;

    /*! ALE receive unknown unicast bytecount */
    uint64_t aleUnknownUcastBcnt;

    /*! ALE receive unknown multicast */
    uint64_t aleUnknownMcast;

    /*! ALE receive unknown multicast bytecount */
    uint64_t aleUnknownMcastBcnt;

    /*! ALE receive unknown broadcast */
    uint64_t aleUnknownBcast;

    /*! ALE receive unknown broadcast bytecount */
    uint64_t aleUnknownBcastBcnt;

    /*! ALE policer matched */
    uint64_t alePolicyMatch;

    /*! ALE policer matched and condition red */
    uint64_t alePolicyMatchRed;

    /*! ALE policer matched and condition yellow */
    uint64_t alePolicyMatchYellow;

    /*! ALE multicast source address drop */
    uint64_t aleMultSADrop;

    /*! ALE dual VLAN drop */
    uint64_t aleDualVlanDrop;

    /*! ALE IEEE 802.3 length error drop */
    uint64_t aleLenErrorDrop;

    /*! ALE IP next header limit drop */
    uint64_t aleIpNextHdrDrop;

    /*! ALE IPv4 fragment drop */
    uint64_t aleIPv4FragDrop;

    /*! Reserved */
    uint64_t reserved57to80[24U];

    /*! IET receive assembly error */
    uint64_t ietRxAssemblyErr;

    /*! IET receive assembly OK */
    uint64_t ietRxAssemblyOk;

    /*! IET receive SMD error */
    uint64_t ietRxSmdError;

    /*! IET recieve merge fragment count */
    uint64_t ietRxFrag;

    /*! IET transmit merge fragment count */
    uint64_t ietTxHold;

    /*! IET transmit merge hold count */
    uint64_t ietTxFrag;

    /*! Reserved */
    uint64_t reserved87to95[9U];

    /*! Transmit memory protect CRC error */
    uint64_t txMemProtectError;

    /*! Host port priority packet count */
    uint64_t txPri[8U];

    /*! Host port priority packet byet count */
    uint64_t txPriBcnt[8U];

    /*! Host port priority packet drop count */
    uint64_t txPriDrop[8U];

    /*! Host port priority packet drop byte count */
    uint64_t txPriDropBcnt[8U];
} CpswStats_HostPort_Ng;

/*!
 * \brief CPSW 5G or 9G MAC port statistics.
 *
 * Statistics values reported by the CPSW hardware for the Ethernet MAC port.
 */
typedef struct CpswStats_MacPort_Ng_s
{
    /*! Total number of good frames received */
    uint64_t rxGoodFrames;

    /*! Total number of good multicast frames received */
    uint64_t rxBcastFrames;

    /*! Total number of good multicast frames received */
    uint64_t rxMcastFrames;

    /*! Total number of pause frames received */
    uint64_t rxPauseFrames;

    /*! Total number of CRC error frames received */
    uint64_t rxCrcErrors;

    /*! Total number of alignment/code errors received */
    uint64_t rxAlignCodeErrors;

    /*! Total number of oversized frames received */
    uint64_t rxOversizedFrames;

    /*! Total number of jabber frames received */
    uint64_t rxJabberFrames;

    /*! Total number of undersized frames received */
    uint64_t rxUndersizedFrames;

    /*! Total number of fragmented frames received */
    uint64_t rxFragments;

    /*! Total number of frames dropped by the ALE */
    uint64_t aleDrop;

    /*! Total number of overrun frames dropped by the ALE */
    uint64_t aleOverrunDrop;

    /*! Total number of received bytes in good frames */
    uint64_t rxOctets;

    /*! Total number of good frames transmitted */
    uint64_t txGoodFrames;

    /*! Total number of good broadcast frames transmitted */
    uint64_t txBcastFrames;

    /*! Total number of good multicast frames transmitted */
    uint64_t txMcastFrames;

    /*! Total number of pause frames transmitted */
    uint64_t txPauseFrames;

    /*! Total number of deferred frames transmitted */
    uint64_t txDeferredFrames;

    /*! Total number of transmitted frames experiencing a collsion */
    uint64_t txCollisionFrames;

    /*! Total number of transmitted frames experiencing a single collision */
    uint64_t txSingleCollFrames;

    /*! Total number of transmitted frames experiencing multiple collisions */
    uint64_t txMultipleCollFrames;

    /*! Total number of transmitted frames abandoned due to excessive collisions */
    uint64_t txExcessiveCollFrames;

    /*! Total number of transmitted frames abandoned due to a late collision */
    uint64_t txLateCollFrames;

    /*! Total number of receive inter-packet gap errors (10G only) */
    uint64_t rxIPGError;

    /*! Total number of transmitted frames that experienced a carrier loss */
    uint64_t txCarrierSenseErrors;

    /*! Total number of bytes in all good frames transmitted */
    uint64_t txOctets;

    /*! Total number of 64-byte frames received and transmitted */
    uint64_t octetsFrames64;

    /*! Total number of frames of size 65 to 127 bytes received and transmitted */
    uint64_t octetsFrames65to127;

    /*! Total number of frames of size 128 to 255 bytes received and transmitted */
    uint64_t octetsFrames128to255;

    /*! Total number of frames of size 256 to 511 bytes received and transmitted */
    uint64_t octetsFrames256to511;

    /*! Total number of frames of size 512 to 1023 bytes received and transmitted */
    uint64_t octetsFrames512to1023;

    /*! Total number of frames of size 1024 or greater transmitted */
    uint64_t octetsFrames1024;

    /*! Total number of bytes received and transmitted */
    uint64_t netOctets;

    /*! Receive bottom of FIFO drop */
    uint64_t rxBottomOfFifoDrop;

    /*! Total number of dropped frames received due to portmask */
    uint64_t portMaskDrop;

    /*! Receive top of FIFO drop */
    uint64_t rxTopOfFifoDrop;

    /*! Total number of frames dropped due to ALE rate limiting */
    uint64_t aleRateLimitDrop;

    /*! Total number of dropped frames due to ALE VID ingress */
    uint64_t aleVidIngressDrop;

    /*! Total number of dropped frames due to DA = SA */
    uint64_t aleDAEqSADrop;

    /*! Total number of dropped frames due to ALE block mode */
    uint64_t aleBlockDrop;

    /*! Total number of dropped frames due to ALE secure mode */
    uint64_t aleSecureDrop;

    /*! Total number of dropped frames due to ALE authentication */
    uint64_t aleAuthDrop;

    /*! ALE receive unknown unicast */
    uint64_t aleUnknownUcast;

    /*! ALE receive unknown unicast bytecount */
    uint64_t aleUnknownUcastBcnt;

    /*! ALE receive unknown multicast */
    uint64_t aleUnknownMcast;

    /*! ALE receive unknown multicast bytecount */
    uint64_t aleUnknownMcastBcnt;

    /*! ALE receive unknown broadcast */
    uint64_t aleUnknownBcast;

    /*! ALE receive unknown broadcast bytecount */
    uint64_t aleUnknownBcastBcnt;

    /*! ALE policer matched */
    uint64_t alePolicyMatch;

    /*! ALE policer matched and condition red */
    uint64_t alePolicyMatchRed;

    /*! ALE policer matched and condition yellow */
    uint64_t alePolicyMatchYellow;

    /*! ALE multicast source address drop */
    uint64_t aleMultSADrop;

    /*! ALE dual VLAN drop */
    uint64_t aleDualVlanDrop;

    /*! ALE IEEE 802.3 length error drop */
    uint64_t aleLenErrorDrop;

    /*! ALE IP next header limit drop */
    uint64_t aleIpNextHdrDrop;

    /*! ALE IPv4 fragment drop */
    uint64_t aleIPv4FragDrop;

    /*! Reserved */
    uint64_t reserved57to80[24U];

    /*! IET receive assembly error */
    uint64_t ietRxAssemblyErr;

    /*! IET receive assembly OK */
    uint64_t ietRxAssemblyOk;

    /*! IET receive SMD error */
    uint64_t ietRxSmdError;

    /*! IET recieve merge fragment count */
    uint64_t ietRxFrag;

    /*! IET transmit merge fragment count */
    uint64_t ietTxHold;

    /*! IET transmit merge hold count */
    uint64_t ietTxFrag;

    /*! Reserved */
    uint64_t reserved87to95[9U];

    /*! Transmit memory protect CRC error */
    uint64_t txMemProtectError;

    /*! Ethernet port priority packet count */
    uint64_t txPri[8U];

    /*! Ethernet port priority packet byet count */
    uint64_t txPriBcnt[8U];

    /*! Ethernet port priority packet drop count */
    uint64_t txPriDrop[8U];

    /*! Ethernet port priority packet drop byte count */
    uint64_t txPriDropBcnt[8U];
} CpswStats_MacPort_Ng;

/*!
 * \brief CPSW nG port statistics.
 *
 * Generic structure for CPSW port statistics.
 */
typedef struct CpswStats_PortStats_s
{
    /*! Statistics blocks are composed of 128 elements */
    uint64_t val[CPSW_STATS_BLOCK_ELEM_NUM];
} CpswStats_PortStats;

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

#endif /* CPSW_STATS_H_ */

/*! @} */
