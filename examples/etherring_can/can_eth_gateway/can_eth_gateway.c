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
 * \file  can_eth_gateway.c
 *
 * \brief This file contains the CAN-ETH gateway implementation
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include "gateway_lookup.h"
#include "can_eth_gateway.h"

/* ========================================================================== */
/*                                  Macros                                    */
/* ========================================================================== */

/* AVTP protocol constants */
#define ENETAPP_AVTP_ETH_TYPE                      (0x22F0)
#define ENETAPP_TSCF_SUBTYPE                       (0x5)
#define ENETAPP_ACF_MSG_TYPE_CAN                   (1U)

/* ========================================================================== */
/*                              Global Variables                              */
/* ========================================================================== */
/* Global instance of the lookup system */
EnetApp_lookupSystem gLookupSystem =
{
    .lookupTable =
    {
        [0] =
        {
            .canID = 0x2,
            .destMac = {0x01, 0x00, 0x5E, 0x7F, 0xFF, 0x2}
        },
        [1] =
        {
            .canID = 0x1,
            .destMac = {0x01, 0x00, 0x5E, 0x7F, 0xFF, 0x1}
        },
        [2] =
        {
            .canID = 0x3,
            .destMac = {0x01, 0x00, 0x5E, 0x7F, 0xFF, 0x3}
        },
        [3] =
        {
            .canID = 0xFF,
            .destMac = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
        },
    },
    .isTableInitialized = false
};

/* ========================================================================== */
/*                                 Structures                                 */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t Gateway_setup(void)
{
    int32_t status = ENET_SOK;
    if (!gLookupSystem.isTableInitialized)
    {
        GatewayLookup_sortLookupTableByCanId(&gLookupSystem);
    }

    return status;
}

static void Gateway_convertDlctoPayloadSize(const uint8_t dlc, uint8_t* payLoadSize)
{
    /* Use lookup table instead of switch statement */
    static const uint8_t dlcToBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

    if (dlc <= 15)
    {
        *payLoadSize = dlcToBytes[dlc];
    }
    else
    {
        *payLoadSize = 0;
    }
}

static int32_t Gateway_convertCanPayloadtoDlc(const uint8_t payload_size, uint32_t* dlc)
{
    int32_t status = ENET_SOK;

    /* Standard sizes mapped to DLC values */
    const uint8_t byteSizes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    const uint32_t byteSizeArrayLength = 16U;

    bool isDlcSet = false;

    for (uint32_t index = 0; index < byteSizeArrayLength; index++)
    {
        if (byteSizes[index] == payload_size)
        {
            *dlc = index;
            isDlcSet = true;
        }
    }

    if (!isDlcSet)
    {
        *dlc = 0xFF;
        status = ENET_EFAIL;
    }

    return status;
}

static inline uint8_t Gateway_buildCanFlagsByte(const MCAN_RxBufElement *canMsg, uint8_t canPadVal)
{
    /* CAN flags bit positions */
    #define ENETAPP_ESI_SHIFT_VAL                      (0U)
    #define ENETAPP_FDF_SHIFT_VAL                      (1U)
    #define ENETAPP_BRS_SHIFT_VAL                      (2U)
    #define ENETAPP_EFF_SHIFT_VAL                      (3U)
    #define ENETAPP_RTR_SHIFT_VAL                      (4U)
    #define ENETAPP_MTV_SHIFT_VAL                      (5U)
    #define ENETAPP_CAN_SHIFT_VAL                      (6U)
    #define ENETAPP_CAN_PAD_MASK                       (0x3)
    /* Header size constants */
    #define ENETAPP_AVTP_COMMON_HEADER_SIZE            (24U)
    #define ENETAPP_ACF_HEADER_SIZE                    (16U)

    uint8_t flags = ((canPadVal & ENETAPP_CAN_PAD_MASK) << ENETAPP_CAN_SHIFT_VAL) |
                    ((0) << ENETAPP_MTV_SHIFT_VAL) | /* Format validation bit cleared by default */
                    ((canMsg->xtd ? 1 : 0) << ENETAPP_EFF_SHIFT_VAL) |
                    ((canMsg->rtr ? 1 : 0) << ENETAPP_RTR_SHIFT_VAL) |
                    ((canMsg->brs ? 1 : 0) << ENETAPP_BRS_SHIFT_VAL) |
                    ((canMsg->fdf ? 1 : 0) << ENETAPP_FDF_SHIFT_VAL) |
                    ((canMsg->esi ? 1 : 0) << ENETAPP_ESI_SHIFT_VAL);

    return flags;
}

int32_t Gateway_convertCanToAvtpPacket(const MCAN_RxBufElement *canMsg,
                                      EnetDma_Pkt *pktInfo,
                                      const uint8_t streamId,
                                      const uint8_t macAddr[ENET_MAC_ADDR_LEN],
                                      uint8_t busId)
{
    int32_t status = ENET_SOK;

    /* Input validation */
    if (canMsg == NULL || pktInfo == NULL)
    {
        status = ENET_EFAIL;
    }

    /* AVTP header field defaults */
    #define ENETAPP_SV_BIT                             (0U)    /* Stream ID valid bit */
    #define ENETAPP_NTSCF_VERSION                      (0x0)
    #define ENETAPP_TV_BIT                             (0U)    /* Timestamp valid */
    #define ENETAPP_MR_BIT                             (0U)    /* Message repeat */
    #define ENETAPP_TU_BIT                             (0U)    /* Time Uncertainty */
    #define ENETAPP_CAN_BUS_ID_MASK                    (0x1F)

    EnetAppUtils_assert(status == ENET_SOK);
    EthFrame *avtpFrame = (EthFrame *)pktInfo->sgList.list[0].bufPtr;
    uint8_t *frame = avtpFrame->payload;
    uint8_t dataLen = 0;
    uint8_t currByteOffset = 0;
    static uint8_t sequenceId = 0;
    uint8_t tempByte = 0;
    uint16_t ntscfDataLength = 0;
    uint8_t canPadVal = 0;
    uint32_t ts = 0x0;
    const uint32_t acfHeaderLength = ENETAPP_ACF_HEADER_SIZE;
    const uint32_t commonHeaderLength = ENETAPP_AVTP_COMMON_HEADER_SIZE;
    uint32_t totalEthPayloadLen = 0U;
    uint32_t totalAcfPayloadLen = 0U;

    /* Fetching the CAN msgId from CAN Element */
    uint32_t canId = (canMsg->id >> 18U) & 0x7FFU;
    uint8_t destMacAdd[6];
    uint32_t isFound = GatewayLookup_findMacAddressByCanId(&gLookupSystem, canId, &destMacAdd[0]);

    if (!isFound)
    {
        status = ENET_EFAIL;
    }

    memcpy(avtpFrame->hdr.dstMac, destMacAdd, ENET_MAC_ADDR_LEN);
    memcpy(avtpFrame->hdr.srcMac, macAddr, ENET_MAC_ADDR_LEN);

    /* Set Ethernet frame type */
    avtpFrame->hdr.etherType = Enet_htons(ENETAPP_AVTP_ETH_TYPE);

    /* Determine actual data length from DLC */
    if (canMsg->fdf)
    {
        /* CAN FD frame - decode DLC to actual bytes */
        Gateway_convertDlctoPayloadSize(canMsg->dlc, &dataLen);
    }
    else
    {
        /* Standard CAN - max 8 bytes */
        dataLen = (canMsg->dlc <= 8) ? canMsg->dlc : 8;
    }

    memset(frame, 0x0, 1400);

    /* Calculate TSCF data length (ACF header + CAN message) */
    ntscfDataLength = Enet_htons(acfHeaderLength + dataLen);

    /* ---------- Prepare TSCF(common) header ---------- */

    /* First byte: NTSCF subtype */
    *(frame + currByteOffset++) = ENETAPP_TSCF_SUBTYPE;

    /* Second byte: SV bit, version, and MSB of data length */
    tempByte = ((uint8_t)ENETAPP_SV_BIT << 7) | ((uint8_t)ENETAPP_NTSCF_VERSION << 4) |
              (uint8_t)(ENETAPP_MR_BIT << 3) | (uint8_t)(ENETAPP_TV_BIT);
    *(frame + currByteOffset++) = tempByte;

    /* Third byte: sequence number */
    *(frame + currByteOffset++) = (uint8_t)sequenceId++;

    /* Fourth byte: TU */
    *(frame + currByteOffset++) = (uint8_t)ENETAPP_TU_BIT;

    /* Stream ID (8 bytes): First 6 bytes from MAC, last 2 from streamId  */
    memcpy(frame + currByteOffset, macAddr, 6);
    memcpy(frame + currByteOffset + 6, &streamId, 2);
    currByteOffset += 8;

    /* Presentation Time (4 bytes) */
    memcpy(frame + currByteOffset, &ts, 4);
    currByteOffset += 4;

    /* Reserved */
    currByteOffset += 4;

    /* Stream data length (2 bytes) */
    memcpy(frame + currByteOffset, &(ntscfDataLength), 2);
    currByteOffset += 2;

    /* Reserved (2B) */
    currByteOffset += 2;

    totalEthPayloadLen = (commonHeaderLength + acfHeaderLength + dataLen);
    totalAcfPayloadLen = (acfHeaderLength + dataLen);

    /* ---------- ACF message header ---------- */

    /* ACF message type and MSB of message length */
    tempByte = (ENETAPP_ACF_MSG_TYPE_CAN << 1) | (uint8_t)(((totalAcfPayloadLen/4) >> 8) & 0x01);
    *(frame + currByteOffset++) = tempByte;

    /* LSB of ACF message length */
    *(frame + currByteOffset++) = (totalAcfPayloadLen/4) & 0xFF;

    /* CAN specific flags */
    tempByte = Gateway_buildCanFlagsByte(canMsg, canPadVal);
    *(frame + currByteOffset++) = tempByte;

    /* CAN bus identifier */
    tempByte = (busId & ENETAPP_CAN_BUS_ID_MASK);
    *(frame + currByteOffset++) = tempByte;

    /* Timestamp - use the MCAN timestamp if available, otherwise zero */
    if (canMsg->rxts != 0)
    {
        uint64_t timestamp = (uint64_t)canMsg->rxts;
        for (int i = 0; i < 8; i++)
        {
            *(frame + currByteOffset + i) = (timestamp >> (8 * (7 - i))) & 0xFF;
        }
    }
    else
    {
       /* Setting the timestamp as 0x0 If rxts is zero */
        memset(frame + currByteOffset, 0x0, 8);
    }
    currByteOffset += 8;

    /* CAN ID - handle standard (11-bit) or extended (29-bit) */
    if (canMsg->xtd)
    {
        /* Extended 29-bit identifier */
        uint32_t id = canMsg->id;
        /* Use direct memory copy to avoid potential byte order issues */
        uint8_t id_bytes[4];
        id_bytes[0] = (0 & 0b11100000) | ((id >> 24) & 0x1F);  /* Reserved byte */
        id_bytes[1] = (id >> 16) & 0xFF;
        id_bytes[2] = (id >> 8) & 0xFF;
        id_bytes[3] = id & 0xFF;
        /* Direct memcpy to ensure bytes are placed correctly */
        memcpy(frame + currByteOffset, id_bytes, 4);
        currByteOffset += 4;
    }
    else
    {
        /* Standard 11-bit identifier */
        uint16_t id = canId & 0x7FF;  /* Ensure only 11 bits are used */
        uint8_t id_bytes[4];
        id_bytes[0] = 0;  /* Reserved byte */
        id_bytes[1] = 0;
        id_bytes[2] = (id >> 8) & 0xFF;
        id_bytes[3] = id & 0xFF;
        memcpy(frame + currByteOffset, id_bytes, 4);
        currByteOffset += 4;
    }

    /* Copy CAN data payload */
    memcpy(frame + currByteOffset, canMsg->data, dataLen);
    currByteOffset += dataLen;

    pktInfo->sgList.list[0].segmentFilledLen = sizeof(EthFrameHeader) + (totalEthPayloadLen);
    pktInfo->sgList.numScatterSegments = 1;
    pktInfo->chkSumInfo = 0U;
    pktInfo->appPriv = NULL;

    EnetDma_checkPktState(&pktInfo->pktState,
                          ENET_PKTSTATE_MODULE_APP,
                          ENET_PKTSTATE_APP_WITH_FREEQ,
                          ENET_PKTSTATE_APP_WITH_DRIVER);

    return status;
}

int32_t Gateway_convertAvtpToCanPacket(const EnetDma_Pkt *pktInfo, MCAN_TxBufElement *canMsg)
{
    int32_t status = ENET_SOK;

    #define ENETAPP_TSCF_SUBTYPE (0x5)

    /* todo: use gateway to convert ETH to CAN or CAN packets can be sent to all CAN interfaces */
    if (pktInfo == NULL || canMsg == NULL)
    {
        EnetAppUtils_assert(pktInfo != NULL || canMsg != NULL);
        status = ENET_EFAIL;
    }

    EnetAppUtils_assert(status == ENET_SOK);
    memset(canMsg, 0, sizeof(MCAN_TxBufElement));

    /* Get pointer to Ethernet frame */
    EthFrame *avtpFrame = (EthFrame *)pktInfo->sgList.list[0].bufPtr;

    /* Check if this is an AVTP packet */
    if (Enet_ntohs(avtpFrame->hdr.etherType) != 0x22F0)
    {
        return 0; /* Not an AVTP packet */
    }

    /* Get pointer to the AVTP payload */
    uint8_t *frame = avtpFrame->payload;

    /* Verify this is a TSCF packet with CAN message */
    if (frame[0] != ENETAPP_TSCF_SUBTYPE)
    {
        status = ENET_EFAIL; /* Not a Time Synchronous Control Format packet */
    }

    /* Calculate offset to ACF message (24 bytes from start of AVTP) */
    uint8_t *acfMsg = (uint8_t *)((uint8_t *)frame + 24);

    /* Check if this is a CAN message */
    uint8_t msgType = (acfMsg[0] >> 1) & 0x7F;
    if (msgType != 1)
    { /* ENETAPP_ACF_MSG_TYPE_CAN */
        status = ENET_EFAIL; /* Not a CAN message */
    }

    /* Extract CAN flags */
    uint8_t canFlags = acfMsg[2];

    /* Extract CAN message fields */
    canMsg->xtd = (canFlags >> 3) & 0x01; /* ENETAPP_EFF_SHIFT_VAL = 3 */
    canMsg->rtr = (canFlags >> 4) & 0x01; /* ENETAPP_RTR_SHIFT_VAL = 4 */
    canMsg->brs = (canFlags >> 2) & 0x01; /* ENETAPP_BRS_SHIFT_VAL = 2 */
    canMsg->fdf = (canFlags >> 1) & 0x01; /* ENETAPP_FDF_SHIFT_VAL = 1 */
    canMsg->esi = canFlags & 0x01;        /* ENETAPP_ESI_SHIFT_VAL = 0 */

    /* Extract CAN ID - depends on whether it's extended or standard */
    if (canMsg->xtd)
    {
        /* Extended 29-bit identifier */
        uint32_t id = 0;
        /* CAN ID is at offset 12 in the ACF message */
        id |= (uint32_t)(acfMsg[12] & 0x1F) << 24; /* Apply mask to reserved bits */
        id |= (uint32_t)(acfMsg[13]) << 16;
        id |= (uint32_t)(acfMsg[14]) << 8;
        id |= (uint32_t)(acfMsg[15]);
        canMsg->id = id;
    }
    else
    {
        /* Standard 11-bit identifier */
        uint32_t id = 0;
        /* Standard ID is in the last 2 bytes of the ID field */
        id |= (uint32_t)(acfMsg[14]) << 8;
        id |= (uint32_t)(acfMsg[15]);
        canMsg->id = id & 0x7FF; /* Ensure only 11 bits */
    }

    /* Calculate data length based on message length */
    uint16_t acfMsgLen = ((acfMsg[0] & 0x01) << 8) | acfMsg[1];
    uint16_t totalBytes = acfMsgLen * 4; /* Convert quadlets to bytes */

    /* Data starts after ACF header (16 bytes) */
    uint8_t dataLen;

    /* Calculate data length - ACF header is 16 bytes */
    if (totalBytes > 16)
    {
        dataLen = totalBytes - 16;
        /* Cap at 64 bytes for CAN FD */
        if (dataLen > 64)
        {
            dataLen = 64;
        }
    }
    else
    {
        dataLen = 0;
    }

    /* Convert byte length to DLC */
    if (canMsg->fdf)
    {
        /* CAN FD frame - encode bytes to DLC */
        Gateway_convertCanPayloadtoDlc(dataLen, &canMsg->dlc);
    }
    else
    {
        /* Standard CAN - max 8 bytes */
        canMsg->dlc = (dataLen <= 8) ? dataLen : 8;
    }

    /* Copy data payload - starts after the ID field (offset 16) */
    if (dataLen > 0)
    {
        memcpy(canMsg->data, acfMsg + 16, dataLen);
    }

    /* Set other fields */
    canMsg->efc = 0; /* Don't store Tx events */
    canMsg->mm = 0;  /* No message marker */

    return status;
}
