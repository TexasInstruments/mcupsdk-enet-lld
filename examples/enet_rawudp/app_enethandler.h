/*
 *  Copyright (c) Texas Instruments Incorporated 2025
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

#ifndef APP_ENETHANDLER_H
#define APP_ENETHANDLER_H


#include <stdint.h>
#include <enet.h>
#include <enet_cfg.h>
#include <include/core/enet_dma.h>

#include <kernel/dpl/SemaphoreP.h>


typedef struct EnetApp_cfg_s
{
    /* Enet driver */
    Enet_Handle hEnet;
    Enet_Type enetType;
    uint32_t instId;
    uint32_t coreId;
    uint32_t coreKey;
    Enet_MacPort macPort;

    EnetDma_RxChHandle hRxCh;
    EnetDma_TxChHandle hTxCh;

    EnetDma_PktQ txFreePktInfoQ;
    uint8_t hostMacAddr[ENET_MAC_ADDR_LEN];

    SemaphoreP_Object txSemObj;

    uint32_t totalTxCnt;
} EnetApp_cfg;


int32_t EnetApp_initApp(void);
int32_t EnetApp_deinitApp(void);
EnetApp_cfg* EnetApp_getAppCfg(void);

/**
 * @brief Transmit an Ethernet frame composed of a header and a payload.
 *
 * This function prepares a DMA packet with two scatter‑gather segments:
 * the first segment contains the Ethernet IPv4 and UDP header and the second contains the payload.
 * It dequeues a free packet from the global transmit free‑packet queue,
 * submits the packet to the TX channel, waits for transmission completion
 * via a semaphore, retrieves the packet back from the driver, and returns it
 * to the free‑packet queue.
 *
 * @param[in] header       Pointer to the header(Enet,IPv4,UDP) buffer.
 * @param[in] header_len   Length of the header in bytes.
 * @param[in] payload      Pointer to the payload buffer.
 * @param[in] payload_len  Length of the payload in bytes.
 *
 * @return ENET_SOK on successful transmission, or a negative error code
 *         (e.g., ENET_EFAIL) if the operation fails.
 */
int32_t EnetApp_sendFrame(uint8_t * header,uint32_t header_len, uint8_t * payload,uint32_t payload_len);

#endif