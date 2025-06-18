/*
 *  Copyright (C) Texas Instruments Incorporated 2025
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
 * \file  can_eth_gateway.h
 *
 * \brief This file contains the CAN-ETH gateway declarations
 */
#ifndef ETHERRING_CAN_ETH_GATEWAY_H_
#define ETHERRING_CAN_ETH_GATEWAY_H_
/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include "../can_trafficgen/can_trafficgen.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

/**
 * @brief Sets up the CAN-Ethernet gateway
 *
 * @return int32_t Return status
 *         - ENET_SOK on success
 *         - ENET_EFAIL on failure
 */
int32_t Gateway_setup(void);

/**
 * @brief Converts CAN message to AVTP packet format
 *
 * @param canMsg   Pointer to the CAN message to be converted
 * @param pktInfo  Pointer to the target Ethernet DMA packet
 * @param streamId Stream identifier for the AVTP packet
 * @param macAddr  MacAddr Array
 * @param busId    CAN bus identifier
 *
 * @return uint8_t Status of the conversion
 *         - ENET_SOK if conversion successful
 *         - ENET_EFAIL if conversion failed
 */
int32_t Gateway_convertCanToAvtpPacket(const MCAN_RxBufElement *canMsg,
                                       EnetDma_Pkt *pktInfo,
                                       const uint8_t streamId,
                                       const uint8_t macAddr[ENET_MAC_ADDR_LEN],
                                       uint8_t busId);

/**
 * @brief Converts AVTP packet to CAN message format
 *
 * @param pktInfo  Pointer to the Ethernet DMA packet to be converted
 * @param canMsg   Pointer to the target CAN message
 *
 * @return uint8_t Status of the conversion
 *         - ENET_SOK if conversion successful
 *         - ENET_EFAIL if conversion failed
 */
int32_t Gateway_convertAvtpToCanPacket(const EnetDma_Pkt *pktInfo, MCAN_TxBufElement *canMsg);

#ifdef __cplusplus
}
#endif

#endif /* ETHERRING_CAN_ETH_GATEWAY_H_ */
