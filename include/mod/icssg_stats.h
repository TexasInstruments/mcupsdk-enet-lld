/*
 *  Copyright (c) Texas Instruments Incorporated 2021-2025
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
    /*! Number of valid bytes sent by RX PRU to host via PSI.
     *  Valid in Switch mode only.
     */
    uint64_t hostRxByteCnt;

    /*! Number of valid bytes copied by RTU to TX queues.
     *  Valid in Switch mode only.
     */
    uint64_t hostTxByteCnt;

    /*! Number of valid bytes sent by RX PRU to host via PSI.
     *  Valid in DualMAC mode only. Port 0. */
    uint64_t hostRxByteCntMacSlice0;

    /*! Number of valid bytes sent by RX PRU to host via PSI.
     *  Valid in DualMAC mode only. Port 1. */
    uint64_t hostRxByteCntMacSlice1;

    /*! Number of valid bytes sent by TX PRU to host via PSI.
     *  Valid in DualMAC mode only. Port 0. */
    uint64_t hostTxByteCntMacSlice0;

    /*! Number of valid bytes sent by TX PRU to host via PSI.
     *  Valid in DualMAC mode only. Port 1. */
    uint64_t hostTxByteCntMacSlice1;

    /*! Number of valid packets sent by RX PRU to host via PSI.
     *  Valid in Switch mode only. */
    uint32_t hostRxPktCnt;

    /*! Number of valid packets copied by RTU to TX queues.
     *  Valid in Switch mode only. */
    uint32_t hostTxPktCnt;

    /*! Number of valid packets sent by RX PRU to host via PSI.
     *  Valid in DualMAC mode only. Port 0. */
    uint32_t hostRxPktCntMacSlice0;

    /*! Number of valid packets sent by RX PRU to host via PSI.
     *  Valid in DualMAC mode only. Port 1. */
    uint32_t hostRxPktCntMacSlice1;

    /*! Number of valid packets copied by RTU to TX queues.
     *  Valid in DualMAC mode only. Port 0. */
    uint32_t hostTxPktCntMacSlice0;

    /*! Number of valid packets copied by RTU to TX queues.
     *  Valid in DualMAC mode only. Port 1. */
    uint32_t hostTxPktCntMacSlice1;

    /*! Diagnostic error counter — increments when RTU drops a locally injected packet
     *  due to port being disabled or a rule violation. Port 0. */
    uint32_t rtu0PktDroppedSlice0;

    /*! Diagnostic error counter — increments when RTU drops a locally injected packet
     *  due to port being disabled or a rule violation. Port 1. */
    uint32_t rtu0PktDroppedSlice1;

    /*! TX queue 0 overflow counter for Port 1.
     *  Valid in Switch mode only.
     */
    uint32_t port1Q0Overflow;

    /*! TX queue 1 overflow counter for Port 1.
     *  Valid in Switch mode only.
     */
    uint32_t port1Q1Overflow;

    /*! TX queue 2 overflow counter for Port 1.
     *  Valid in Switch mode only.
     */
    uint32_t port1Q2Overflow;

    /*! TX queue 3 overflow counter for Port 1.
     *  Valid in Switch mode only.
     */
    uint32_t port1Q3Overflow;

    /*! TX queue 4 overflow counter for Port 1.
     *  Valid in Switch mode only.
     */
    uint32_t port1Q4Overflow;

    /*! TX queue 5 overflow counter for Port 1.
     *  Valid in Switch mode only.
     */
    uint32_t port1Q5Overflow;

    /*! TX queue 6 overflow counter for Port 1.
     *  Valid in Switch mode only.
     */
    uint32_t port1Q6Overflow;

    /*! TX queue 7 overflow counter for Port 1.
     *  Valid in Switch mode only.
     */
    uint32_t port1Q7Overflow;

    /*! TX queue 0 overflow counter for Port 2.
     *  Valid in Switch mode only.
     */
    uint32_t port2Q0Overflow;

    /*! TX queue 1 overflow counter for Port 2.
     *  Valid in Switch mode only.
     */
    uint32_t port2Q1Overflow;

    /*! TX queue 2 overflow counter for Port 2.
     *  Valid in Switch mode only.
     */
    uint32_t port2Q2Overflow;

    /*! TX queue 3 overflow counter for Port 2.
     *  Valid in Switch mode only.
     */
    uint32_t port2Q3Overflow;

    /*! TX queue 4 overflow counter for Port 2.
     *  Valid in Switch mode only.
     */
    uint32_t port2Q4Overflow;

    /*! TX queue 5 overflow counter for Port 2.
     *  Valid in Switch mode only.
     */
    uint32_t port2Q5Overflow;

    /*! TX queue 6 overflow counter for Port 2.
     *  Valid in Switch mode only.
     */
    uint32_t port2Q6Overflow;

    /*! TX queue 7 overflow counter for Port 2.
     *  Valid in Switch mode only.
     */
    uint32_t port2Q7Overflow;

    /*! Host egress queue 0 overflow counter.
     *  Reserved, not currently used.
     */
    uint32_t hostQ0Overflow;

    /*! Host egress queue 1 overflow counter.
     *  Reserved, not currently used.
     */
    uint32_t hostQ1Overflow;

    /*! Host egress queue 2 overflow counter.
     *  Reserved, not currently used.
     */
    uint32_t hostQ2Overflow;

    /*! Host egress queue 3 overflow counter.
     *  Reserved, not currently used.
     */
    uint32_t hostQ3Overflow;

    /*! Host egress queue 4 overflow counter.
     *  Reserved, not currently used.
     */
    uint32_t hostQ4Overflow;

    /*! Host egress queue 5 overflow counter.
     *  Reserved, not currently used.
     */
    uint32_t hostQ5Overflow;

    /*! Host egress queue 6 overflow counter.
     *  Reserved, not currently used.
     */
    uint32_t hostQ6Overflow;

    /*! Host egress queue 7 overflow counter.
     *  Reserved, not currently used.
     */
    uint32_t hostQ7Overflow;

    /*! Host RX pre-emptible egress queue overflow counter.
     *  Valid in Switch mode only.
     */
    uint32_t hostEgressQPreOverflow;

    /*! Increments when a packet is dropped at PRU due to a rule violation.
     *  Port 0.
     */
    uint32_t droppedPktSlice0;

    /*! Increments when a packet is dropped at PRU due to a rule violation.
     *  Port 1.
     */
    uint32_t droppedPktSlice1;

    /*! Increments when a CRC error or Min/Max frame size error is detected.
     *  Port 0.
     */
    uint32_t rxErrorSlice0;

    /*! Increments when a CRC error or Min/Max frame size error is detected.
     *  Port 1.
     */
    uint32_t rxErrorSlice1;

    /*! RTU diagnostic counter — increments when RTU detects a Data Status invalid condition.
     *  Port 0.
     */
    uint32_t rxEofRtuDsInvalidSlice0;

    /*! RTU diagnostic counter — increments when RTU detects a Data Status invalid condition.
     *  Port 1.
     */
    uint32_t rxEofRtuDsInvalidSlice1;

    /*! Counter for packets dropped via NRT TX Port1 */
    uint32_t txPort1DroppedPkt;

    /*! Counter for packets dropped via NRT TX Port2 */
    uint32_t txPort2DroppedPkt;

    /*! Counter for packets with TS flag dropped via NRT TX Port1 */
    uint32_t txPort1TsDroppedPkt;

    /*! Counter for packets with TS flag dropped via NRT TX Port2 */
    uint32_t txPort2TsDroppedPkt;

    /*! Port 0 diagnostic error counter which increments when RX frame is dropped
     * due to port is disabled */
    uint32_t infPortDisabledSlice0;

    /*! Port 1 diagnostic error counter which increments when RX frame is dropped
     * due to port is disabled */
    uint32_t infPortDisabledSlice1;

    /*! Port 0 diagnostic error counter which increments when RX frame is dropped
     *  due to SA violation */
    uint32_t infSavSlice0;

    /*! Port 1 diagnostic error counter which increments when RX frame is dropped
     *  due to SA violation */
    uint32_t infSavSlice1;

    /*! Port 0 diagnostic error counter which increments when RX frame is dropped
     *  due to SA black listed */
    uint32_t infSaBlSlice0;

    /*! Port 1 diagnostic error counter which increments when RX frame is dropped
     *  due to SA black listed */
    uint32_t infSaBlSlice1;

    /*! Port 0 diagnostic error counter which increments when RX frame is dropped
     *  due to port blocked and not a special frame */
    uint32_t infPortBlockedSlice0;

    /*! Port 1 diagnostic error counter which increments when RX frame is dropped
     *  due to port blocked and not a special frame */
    uint32_t infPortBlockedSlice1;

    /*! Port 0 diagnostic error counter which increments when RX frame is dropped
     *  due to tagged */
    uint32_t infAftDropTaggedSlice0;

    /*! Port 1 diagnostic error counter which increments when RX frame is dropped
     *  due to tagged */
    uint32_t infAftDropTaggedSlice1;

    /*! Port 0 diagnostic error counter which increments when RX frame is dropped
     *  due to priority tagged */
    uint32_t infAftDropPrioTaggedSlice0;

    /*! Port 1 diagnostic error counter which increments when RX frame is dropped
     * due to priority tagged */
    uint32_t infAftDropPrioTaggedSlice1;

    /*! Port 0 diagnostic error counter which increments when RX frame is dropped
     *  due to untagged */
    uint32_t infAftDropNoTagSlice0;

    /*! Port 1 diagnostic error counter which increments when RX frame is dropped
     *  due to untagged */
    uint32_t infAftDropNoTagSlice1;

    /*! Port 0 diagnostic error counter which increments when RX frame is dropped
     *  due to port not member of VLAN */
    uint32_t infAftDropNotMemberSlice0;

    /*! Port 1 diagnostic error counter which increments when RX frame is dropped
     *  due to port not member of VLAN */
    uint32_t infAftDropNotMemberSlice1;

    /*! PRU diagnostic error counter which increments when an entry couldn't be learned */
    uint32_t fdbNoSpaceToLearn;

    /*! Port 0 bad fragment error counter */
    uint32_t preemptBadFragSlice0;

    /*! Port 1 bad fragment error counter */
    uint32_t preemptBadFragSlice1;

    /*! Port 0 fragment assembly error counter */
    uint32_t preemptAsmErrSlice0;

    /*! Port 1 fragment assembly error counter */
    uint32_t preemptAsmErrSlice1;

    /*! Port 0 fragment count in TX */
    uint32_t preemptFragCntTxSlice0;

    /*! Port 1 fragment count in TX */
    uint32_t preemptFragCntTxSlice1;

    /*! Port 0 assembly completed */
    uint32_t preemptAsmOkSlice0;

    /*! Port 1 assembly completed */
    uint32_t preemptAsmOkSlice1;

    /*! Port 0 fragments received */
    uint32_t preemptFragCntRxSlice0;

    /*! Port 1 fragments received */
    uint32_t preemptFragCntRxSlice1;

    /*! PRU diagnostic error counter — increments if EOF task is scheduled
     *  without receiving RX_B1. Port 0.
     */
    uint32_t rxEofShortFrameErrSlice0;

    /*! PRU diagnostic error counter — increments if EOF task is scheduled
     *  without receiving RX_B1. Port 1.
     */
    uint32_t rxEofShortFrameErrSlice1;

    /*! Diagnostic counter — increments when a frame is dropped
     *  due to early EOF received in the B0 buffer. Port 0.
     */
    uint32_t rxB0DropEarlyEoFSlice0;

    /*! Diagnostic counter — increments when a frame is dropped
     *  due to early EOF received in the B0 buffer. Port 1.
     */
    uint32_t rxB0DropEarlyEoFSlice1;

    /*! TX diagnostic counter — increments when a frame is cut off
     *  to prevent packet size exceeding 2000 bytes. Port 0.
     */
    uint32_t txJumboFrameCutoffSlice0;

    /*! TX diagnostic counter — increments when a frame is cut off
     *  to prevent packet size exceeding 2000 bytes. Port 1.
     */
    uint32_t txJumboFrameCutoffSlice1;

    /*! Diagnostic counter — increments when an express frame is received
     *  in the same queue as a previous fragment. Port 0.
     */
    uint32_t rxExpFragQDropSlice0;

    /*! Diagnostic counter — increments when an express frame is received
     *  in the same queue as a previous fragment. Port 1.
     */
    uint32_t rxExpFragQDropSlice1;

    /*! RX FIFO overrun — increments when the L1 receive FIFO is full because firmware
     *  cannot process frames fast enough; the arriving frame is dropped. Port 0.
     */
    uint32_t rxFifoOverRunSlice0;

    /*! RX FIFO overrun — increments when the L1 receive FIFO is full because firmware
     *  cannot process frames fast enough; the arriving frame is dropped. Port 1.
     */
    uint32_t rxFifoOverRunSlice1;

    /*! Host RX pre-emptible egress queue overflow counter — DualMAC mode only Port 0 */
    uint32_t hostEgrsQPreOvrFloMacSlice0;

    /*! Host RX pre-emptible egress queue overflow counter — DualMAC mode only Port 1 */
    uint32_t hostEgrsQPreOvrFloMacSlice1;

    /*! Host RX express egress queue overflow counter — DualMAC mode only Port 0 */
    uint32_t hostEgrsQExpOvrFloMacSlice0;

    /*! Host RX express egress queue overflow counter — DualMAC mode only Port 1 */
    uint32_t hostEgrsQExpOvrFloMacSlice1;

    /*! Cut-through packet counter for port 0 */
    uint32_t cutThroughPacketSlice0;

    /*! Cut-through packet counter for port 1 */
    uint32_t cutThroughPacketSlice1;
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
