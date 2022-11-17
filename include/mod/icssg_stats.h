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
 * \file  icssg_stats.h
 *
 * \brief This file contains the type definitions and helper macros for ICSSG
 *        Statistics interface.
 */

/*!
 * \ingroup  ENET_MOD_STATS
 * \defgroup ICCSG_STATS_MOD ICSSG Statistics
 *
 * The ICSSG statistics module implements only the IOCTLs defined in the generic
 * \ref ENET_MOD_STATS API.  There are no additional ICSSG-specific IOCTLs.
 *
 * The statistics structures are ICSSG specific:
 *  - Host port: \ref IcssgStats_Pa
 *  - MAC ports: \ref IcssgStats_MacPort
 *
 * @{
 */

#ifndef ICSSG_STATS_H_
#define ICSSG_STATS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_mod_stats.h>

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
 * \brief ICSSG statistics counters.
 *
 * The statistics counters reported by PRU-ICSSG.  This structure type must be
 * used by application to retrieve ICSSG statistics via
 * #ENET_STATS_IOCTL_GET_MACPORT_STATS.
 */
typedef struct IcssgStats_MacPort_s
{
    /*! RX Good Frame Count Inc on none min err max err crc err odd err, Wrt subtracts */
    uint32_t rxGoodFrames;

    /*! RX BC Frame Count Inc on BC type, Wrt subtracts */
    uint32_t rxBCastFrames;

    /*! RX MC Frame Count Inc on MC type, Wrt subtracts */
    uint32_t rxMCastFrames;

    /*! RX CRC Err Frame Count Inc on crc err, Wrt subtracts */
    uint32_t rxCRCErrors;

    /*! RX MII Err Frame Count Inc on mii sgmii rgmii err, Wrt subtracts */
    uint32_t rxMIIErrors;

    /*! RX Odd Nibble Frame Count Inc on odd nibble mii, Wrt subtracts */
    uint32_t rxOddNibbleFrame;

    /*! RX Max Size Frame Count Limit */
    uint32_t rxMaxSizeFrame;

    /*! RX MAX Size Err Frame Count Inc if > than Limit, Wrt subtracts */
    uint32_t rxMaxSizeErrFrame;

    /*! RX Min Size Frame Limit */
    uint32_t rxMinSizeFrame;

    /*! RX MIN Size Frame Count incremented  if < than limit */
    uint32_t rxMinSizeErrFrame;

    /*! RX L1 FIFO overflow frame count */
    uint32_t rxOverrunFrame;

    /*! RX Class0 Hit Count */
    uint32_t rxClass0;

    /*! RX Class1 Hit Count */
    uint32_t rxClass1;

    /*! RX Class2 Hit Count */
    uint32_t rxClass2;

    /*! RX Class3 Hit Count */
    uint32_t rxClass3;

    /*! RX Class4 Hit Count */
    uint32_t rxClass4;

    /*! RX Class5 Hit Count */
    uint32_t rxClass5;

    /*! RX Class6 Hit Count */
    uint32_t rxClass6;

    /*! RX Class7 Hit Count */
    uint32_t rxClass7;

    /*! RX Class8 Hit Count */
    uint32_t rxClass8;

    /*! RX Class9 Hit Count */
    uint32_t rxClass9;

    /*! RX Class10 Hit Count */
    uint32_t rxClass10;

    /*! RX Class11 Hit Count */
    uint32_t rxClass11;

    /*! RX Class12 Hit Count */
    uint32_t rxClass12;

    /*! RX Class13 Hit Count */
    uint32_t rxClass13;

    /*! RX Class14 Hit Count */
    uint32_t rxClass14;

    /*! RX Class15 Hit Count */
    uint32_t rxClass15;

    /*! SMD FRAG  Frames Received with Errors */
    uint32_t rxSMDFragErr;

    /*! RX Bucket1 Byte Size */
    uint32_t rxBucket1SizeConfig;

    /*! RX Bucket2 Byte Size */
    uint32_t rxBucket2SizeConfig;

    /*! RX Bucket3 Byte Size */
    uint32_t rxBucket3SizeConfig;

    /*! RX Bucket4 Byte Size */
    uint32_t rxBucket4SizeConfig;

    /*! RX 64Byte Frame Count */
    uint32_t rx64BSizedFrame;

    /*! RX Bucket1 Frame Count Inc if <= than Bucket1 Byte Size */
    uint32_t rxBucket1SizedFrame;

    /*! RX Bucket2 Frame Count Inc if <= than Bucket2 Byte Size and if > than Bucket1 Byte Size */
    uint32_t rxBucket2SizedFrame;

    /*! RX Bucket3 Frame Count Inc if <= than Bucket3 Byte Size and if > than Bucket2 Byte Size */
    uint32_t rxBucket3SizedFrame;

    /*! RX Bucket4 Frame Count Inc if <= than Bucket4 Byte Size and if > than Bucket3 Byte Size */
    uint32_t rxBucket4SizedFrame;

    /*! RX Bucket5 Frame Count Inc if > than Bucket4 Byte Size */
    uint32_t rxBucket5SizedFrame;

    /*! RX Total Byte Count */
    uint32_t rxTotalByte;

    /*! RX and TX Total Byte Count */
    uint32_t rxTxTotalByte;

    /*! TX Good Frame Count Inc if no min size err max size err or mii odd nibble */
    uint32_t txGoodFrame;

    /*! TX Broadcast Frame Count Inc if BC */
    uint32_t txBcastFrame;

    /*! TX Multicast Frame Count Inc if MC */
    uint32_t txMcastFrame;

    /*! TX Odd Nibble Frame Count Inc if mii odd nibble */
    uint32_t txOddNibbleFrame;

    /*! TX Max Underflow Error Count */
    uint32_t txUnderFlowErr;

    /*! TX Max Size Frame Count Limit */
    uint32_t txMaxSizeFrame;

    /*! TX Max Size Err Frame Count Inc if > max Limit */
    uint32_t txMaxSizeErrFrame;

    /*! TX Min Size Frame Count Limit */
    uint32_t txMinSizeFrame;

    /*! TX Min Size Err Frame Count Inc if < min Limit */
    uint32_t txMinSizeErrFrame;

    /*! TX Bucket1 Byte Size */
    uint32_t txBucket1SizeConfig;

    /*! TX Bucket2 Byte Size */
    uint32_t txBucket2SizeConfig;

    /*! TX Bucket3 Byte Size */
    uint32_t txBucket3SizeConfig;

    /*! TX Bucket4  Byte Size */
    uint32_t txBucket4SizeConfig;

    /*! TX 64Byte Frame Count Inc if 64B */
    uint32_t tx64BSizedFrame;

    /*! TX Bucket1 Frame Count if <= than Bucket1 */
    uint32_t txBucket1SizedFrame;

    /*! TX Bucket2 Frame Count if <= than Bucket2 Size and > Bucket 1 Size */
    uint32_t txBucket2SizedFrame;

    /*! TX Bucket3 Frame Count if <= than Bucket3 Size and > Bucket 2 Size */
    uint32_t txBucket3SizedFrame;

    /*! TX Bucket4 Frame Count if <= than Bucket4 Size and > Bucket 3 Size */
    uint32_t txBucket4SizedFrame;

    /*! TX Bucket5 Frame Count if > Bucket 4 Size */
    uint32_t txBucket5SizedFrame;

    /*! Total Bytes Sent */
    uint32_t txTotalByte;
} IcssgStats_MacPort;

/*!
 * \brief ICSSG Packet Accelerator (PA) statistics counters.
 *
 * The statistics counters reported by PRU-ICSSG.  This structure type must be
 * used by application to retrieve ICSSG statistics via
 * #ENET_STATS_IOCTL_GET_HOSTPORT_STATS.
 */
typedef struct IcssgStats_Pa_s
{
    /*! Number of valid bytes sent by RX PRU to Host on PSI. Currently disabled */
    uint64_t hostRxByteCnt;

    /*! Number of valid bytes copied by RTU0 to TX queues. Currently disabled */
    uint64_t hostTxByteCnt;

    /*! Number of valid packets sent by RX PRU to Host on PSI */
    uint32_t hostRxPktCnt;

    /*! Number of valid packets copied by RTU0 to TX queues */
    uint32_t hostTxPktCnt;

    /*! PRU diagnostic error counter which increments when RTU0 drops a locally
     *  injected packet due to port disabled or rule violation */
    uint32_t rtu0PktDroppedSlice0;

    /*! PRU diagnostic error counter which increments when RTU1 drops a locally
     *  injected packet due to port disabled or rule violation */
    uint32_t rtu0PktDroppedSlice1;

    /*! Port1 TX Q Overflow Counter */
    uint32_t port1Q0Overflow;

    /*! Port1 TX Q Overflow Counter */
    uint32_t port1Q1Overflow;

    /*! Port1 TX Q Overflow Counter */
    uint32_t port1Q2Overflow;

    /*! Port1 TX Q Overflow Counter */
    uint32_t port1Q3Overflow;

    /*! Port1 TX Q Overflow Counter */
    uint32_t port1Q4Overflow;

    /*! Port1 TX Q Overflow Counter */
    uint32_t port1Q5Overflow;

    /*! Port1 TX Q Overflow Counter */
    uint32_t port1Q6Overflow;

    /*! Port1 TX Q Overflow Counter */
    uint32_t port1Q7Overflow;

    /*! Port2 TX Q Overflow Counter */
    uint32_t port2Q0Overflow;

    /*! Port2 TX Q Overflow Counter */
    uint32_t port2Q1Overflow;

    /*! Port2 TX Q Overflow Counter */
    uint32_t port2Q2Overflow;

    /*! Port2 TX Q Overflow Counter */
    uint32_t port2Q3Overflow;

    /*! Port2 TX Q Overflow Counter */
    uint32_t port2Q4Overflow;

    /*! Port2 TX Q Overflow Counter */
    uint32_t port2Q5Overflow;

    /*! Port2 TX Q Overflow Counter */
    uint32_t port2Q6Overflow;

    /*! Port2 TX Q Overflow Counter */
    uint32_t port2Q7Overflow;

    /*! Host TX Q Overflow Counter */
    uint32_t hostQ0Overflow;

    /*! Host TX Q Overflow Counter */
    uint32_t hostQ1Overflow;

    /*! Host TX Q Overflow Counter */
    uint32_t hostQ2Overflow;

    /*! Host TX Q Overflow Counter */
    uint32_t hostQ3Overflow;

    /*! Host TX Q Overflow Counter */
    uint32_t hostQ4Overflow;

    /*! Host TX Q Overflow Counter */
    uint32_t hostQ5Overflow;

    /*! Host TX Q Overflow Counter */
    uint32_t hostQ6Overflow;

    /*! Host TX Q Overflow Counter */
    uint32_t hostQ7Overflow;

    /*! Host Egress Q (Pre-emptible) Overflow Counter */
    uint32_t hostEgressQPreOverflow;

    /*! Incremented if a packet is dropped at PRU0 because of a rule violation */
    uint32_t droppedPktSlice0;

    /*! Incremented if a packet is dropped at PRU1 because of a rule violation */
    uint32_t droppedPktSlice1;

    /*! Incremented if there was a CRC error or Min/Max frame error at PRU0 */
    uint32_t rxErrorSlice0;

    /*! Incremented if there was a CRC error or Min/Max frame error at PRU1 */
    uint32_t rxErrorSlice1;

    /*! RTU0 diagnostic counter increments when RTU detects Data Status invalid condition */
    uint32_t rxEofRtuDsInvalidSlice0;

    /*! RTU1 diagnostic counter increments when RTU detects Data Status invalid condition */
    uint32_t rxEofRtuDsInvalidSlice1;

    /*! Counter for packets dropped via NRT TX Port1 */
    uint32_t txPort1DroppedPkt;

    /*! Counter for packets dropped via NRT TX Port2 */
    uint32_t txPort2DroppedPkt;

    /*! Counter for packets with TS flag dropped via NRT TX Port1 */
    uint32_t txPort1TsDroppedPkt;

    /*! Counter for packets with TS flag dropped via NRT TX Port2 */
    uint32_t txPort2TsDroppedPkt;

    /*! PRU0 diagnostic error counter which increments when RX frame is dropped
     * due to port is disabled */
    uint32_t infPortDisabledSlice0;

    /*! PRU1 diagnostic error counter which increments when RX frame is dropped
     * due to port is disabled */
    uint32_t infPortDisabledSlice1;

    /*! PRU0 diagnostic error counter which increments when RX frame is dropped
     *  due to SA violation */
    uint32_t infSavSlice0;

    /*! PRU1 diagnostic error counter which increments when RX frame is dropped
     *  due to SA violation */
    uint32_t infSavSlice1;

    /*! PRU0 diagnostic error counter which increments when RX frame is dropped
     *  due to SA black listed */
    uint32_t infSaBlSlice0;

    /*! PRU1 diagnostic error counter which increments when RX frame is dropped
     *  due to SA black listed */
    uint32_t infSaBlSlice1;

    /*! PRU0 diagnostic error counter which increments when RX frame is dropped
     *  due to port blocked and not a special frame */
    uint32_t infPortBlockedSlice0;

    /*! PRU1 diagnostic error counter which increments when RX frame is dropped
     *  due to port blocked and not a special frame */
    uint32_t infPortBlockedSlice1;

    /*! PRU0 diagnostic error counter which increments when RX frame is dropped
     *  due to tagged */
    uint32_t infAftDropTaggedSlice0;

    /*! PRU1 diagnostic error counter which increments when RX frame is dropped
     *  due to tagged */
    uint32_t infAftDropTaggedSlice1;

    /*! PRU0 diagnostic error counter which increments when RX frame is dropped
     *  due to priority tagged */
    uint32_t infAftDropPrioTaggedSlice0;

    /*! PRU1 diagnostic error counter which increments when RX frame is dropped
     * due to priority tagged */
    uint32_t infAftDropPrioTaggedSlice1;

    /*! PRU0 diagnostic error counter which increments when RX frame is dropped
     *  due to untagged */
    uint32_t infAftDropNoTagSlice0;

    /*! PRU1 diagnostic error counter which increments when RX frame is dropped
     *  due to untagged */
    uint32_t infAftDropNoTagSlice1;

    /*! PRU0 diagnostic error counter which increments when RX frame is dropped
     *  due to port not member of VLAN */
    uint32_t infAftDropNotMemberSlice0;

    /*! PRU1 diagnostic error counter which increments when RX frame is dropped
     *  due to port not member of VLAN */
    uint32_t infAftDropNotMemberSlice1;

    /*! PRU diagnostic error counter which increments when an entry couldn't be learned */
    uint32_t fdbNoSpaceToLearn;

    /*! PRU0 Bad fragment Error Counter */
    uint32_t preemptBadFragSlice0;

    /*! PRU1 Bad fragment Error Counter */
    uint32_t preemptBadFragSlice1;

    /*! PRU0 Fragment assembly Error Counter */
    uint32_t preemptAsmErrSlice0;

    /*! PRU1 Fragment assembly Error Counter */
    uint32_t preemptAsmErrSlice1;

    /*! PRU0 Fragment count in TX */
    uint32_t preemptFragCntTxSlice0;

    /*! PRU1 Fragment count in TX */
    uint32_t preemptFragCntTxSlice1;

    /*! PRU0 Assembly Completed */
    uint32_t preemptAsmOkSlice0;

    /*! PRU1 Assembly Completed */
    uint32_t preemptAsmOkSlice1;

    /*! PRU0 Fragments received */
    uint32_t preemptFragCntRxSlice0;

    /*! PRU1 Fragments received */
    uint32_t preemptFragCntRxSlice1;

    /*! PRU0 diagnostic error counter which increments if EOF task is scheduled
     *  without seeing RX_B1 */
    uint32_t rxEofShortFrameErrSlice0;

    /*! PRU1 diagnostic error counter which increments if EOF task is scheduled
     *  without seeing RX_B1 */
    uint32_t rxEofShortFrameErrSlice1;
} IcssgStats_Pa;

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

#endif /* ICSSG_STATS_H_ */

/*! @} */
