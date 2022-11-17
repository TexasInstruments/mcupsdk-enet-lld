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
 * \file  enet_udma_psi.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet UDMA CPPI protocol specific info.
 */

/*!
 * \ingroup  ENET_UDMA_API
 * \defgroup ENET_UDMA_CPPI_PSI UDMA CPPI Protocol Specific Info
 *
 * @{
 */

#ifndef ENET_UDMA_PSI_H_
#define ENET_UDMA_PSI_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/* -------------------- CPPI TX Info Word 0 definitions --------------------- */

/*! \brief CPPI TX Info Word 0 - Flow Id bit shift. */
#define ENETUDMA_CPPI_TXINFO_WORD0_FLOWID_SHIFT              (0U)

/*! \brief CPPI TX Info Word 0 - Flow ID bit mask. */
#define ENETUDMA_CPPI_TXINFO_WORD0_FLOWID_MASK               (((uint32_t) 0xFFU) << ENETUDMA_CPPI_TXINFO_WORD0_FLOWID_SHIFT)

/*! \brief CPPI TX Info Word 0 - CRC Type bit shift. */
#define ENETUDMA_CPPI_TXINFO_WORD0_CRCTYPE_SHIFT             (22U)

/*! \brief CPPI TX Info Word 0 - CRC Type bit mask. */
#define ENETUDMA_CPPI_TXINFO_WORD0_CRCTYPE_MASK              (((uint32_t) 0x1U) << ENETUDMA_CPPI_TXINFO_WORD0_CRCTYPE_SHIFT)

/*! \brief CPPI TX Info Word 0 - Pass CRC bit shift. */
#define ENETUDMA_CPPI_TXINFO_WORD0_PASSCRC_SHIFT             (23U)

/*! \brief CPPI TX Info Word 0 - Pass CRC bit mask. */
#define ENETUDMA_CPPI_TXINFO_WORD0_PASSCRC_MASK              (((uint32_t) 0x1U) << ENETUDMA_CPPI_TXINFO_WORD0_PASSCRC_SHIFT)

/*! \brief CPPI TX Info Word 0 - Packet Type bit shift. */
#define ENETUDMA_CPPI_TXINFO_WORD0_PKTTYPE_SHIFT             (27U)

/*! \brief CPPI TX Info Word 0 - Packet Type bit mask. */
#define ENETUDMA_CPPI_TXINFO_WORD0_PKTTYPE_MASK              (((uint32_t) 0x1FU) << ENETUDMA_CPPI_TXINFO_WORD0_PKTTYPE_SHIFT)


/* -------------------- CPPI TX Info Word 1 definitions --------------------- */

/*! \brief CPPI TX Info Word 1 - Packet Length bit shift. */
#define ENETUDMA_CPPI_TXINFO_WORD1_PKTLEN_SHIFT              (0U)

/*! \brief CPPI TX Info Word 1 - Packet Length bit mask. */
#define ENETUDMA_CPPI_TXINFO_WORD1_PKTLEN_MASK               (((uint32_t) 0x3FFF) << ENETUDMA_CPPI_TXINFO_WORD1_PKTLEN_SHIFT)


/* -------------------- CPPI TX Info Word 3 definitions --------------------- */

/*! \brief CPPI TX Info Word 3 - Source Id bit shift. */
#define ENETUDMA_CPPI_TXINFO_WORD3_SRCID_SHIFT               (16U)

/*! \brief CPPI TX Info Word 3 - Source Id bit mask. */
#define ENETUDMA_CPPI_TXINFO_WORD3_SRCID_MASK                (((uint32_t) 0xFF) << ENETUDMA_CPPI_TXINFO_WORD3_SRCID_SHIFT)


/* -------------------- CPPI TX Status Word 2 definitions --------------------- */

/*! \brief CPPI TX Status Word 2- Checksum Add bit shift. */
#define ENETUDMA_CPPI_TXSTATUS_WORD2_CHKSUM_ADD_SHIFT        (0U)

/*! \brief CPPI TX Status Word 2 - Checksum Add bit mask. */
#define ENETUDMA_CPPI_TXSTATUS_WORD2_CHKSUM_ADD_MASK         (((uint32_t) 0xFFFF) << ENETUDMA_CPPI_TXSTATUS_WORD2_CHKSUM_ADD_SHIFT)

/*! \brief CPPI TX Status Word 2 - Checksum Error bit shift. */
#define ENETUDMA_CPPI_TXSTATUS_WORD2_CHKSUM_ERR_SHIFT        (16U)

/*! \brief CPPI TX Status Word 2 - Checksum Error bit mask. */
#define ENETUDMA_CPPI_TXSTATUS_WORD2_CHKSUM_ERR_MASK         (((uint32_t) 0x1U) << ENETUDMA_CPPI_TXSTATUS_WORD2_CHKSUM_ERR_SHIFT)

/*! \brief CPPI TX Status Word 2 - IP Fragment bit shift. */
#define ENETUDMA_CPPI_TXSTATUS_WORD2_FRAGMENT_SHIFT          (17U)

/*! \brief CPPI TX Status Word 2 - IP Fragment bit mask. */
#define ENETUDMA_CPPI_TXSTATUS_WORD2_FRAGMENT_MASK           (((uint32_t) 0x1U) << ENETUDMA_CPPI_TXSTATUS_WORD2_FRAGMENT_SHIFT)

/*! \brief CPPI TX Status Word 2 - TCP or UDP bit shift. */
#define ENETUDMA_CPPI_TXSTATUS_WORD2_TCP_UDP_N_SHIFT         (18U)

/*! \brief CPPI TX Status Word 2 - TCP or UDP bit mask. */
#define ENETUDMA_CPPI_TXSTATUS_WORD2_TCP_UDP_N_MASK          (((uint32_t) 0x1U) << ENETUDMA_CPPI_TXSTATUS_WORD2_TCP_UDP_N_SHIFT)

/*! \brief CPPI TX Status Word 2 - IPv6 Valid bit shift. */
#define ENETUDMA_CPPI_TXSTATUS_WORD2_IPV6_VALID_SHIFT        (19U)

/*! \brief CPPI TX Status Word 2 - IPv6 Valid bit mask. */
#define ENETUDMA_CPPI_TXSTATUS_WORD2_IPV6_VALID_MASK         (((uint32_t) 0x1U) << ENETUDMA_CPPI_TXSTATUS_WORD2_IPV6_VALID_SHIFT)

/*! \brief CPPI TX Status Word 2 - IPv4 Valid bit shift. */
#define ENETUDMA_CPPI_TXSTATUS_WORD2_IPV4_VALID_SHIFT        (20U)

/*! \brief CPPI TX Status Word 2 - IPv4 Valid bit mask. */
#define ENETUDMA_CPPI_TXSTATUS_WORD2_IPV4_VALID_MASK         (((uint32_t) 0x1U) << ENETUDMA_CPPI_TXSTATUS_WORD2_IPV4_VALID_SHIFT)


/* -------------------- CPPI RX Info Word 0 definitions --------------------- */

/*! \brief CPPI RX Info Word 0 - CRC Type bit shift. */
#define ENETUDMA_CPPI_RXINFO_WORD0_CRCTYPE_SHIFT             (22U)

/*! \brief CPPI RX Info Word 0 - CRC Type bit mask. */
#define ENETUDMA_CPPI_RXINFO_WORD0_CRCTYPE_MASK              (((uint32_t) 0x1U) << ENETUDMA_CPPI_RXINFO_WORD0_CRCTYPE_SHIFT)

/*! \brief CPPI RX Info Word 0 - Pass CRC bit shift. */
#define ENETUDMA_CPPI_RXINFO_WORD0_PASSCRC_SHIFT             (23U)

/*! \brief CPPI RX Info Word 0 - Pass CRC bit mask. */
#define ENETUDMA_CPPI_RXINFO_WORD0_PASSCRC_MASK              (((uint32_t) 0x1U) << ENETUDMA_CPPI_RXINFO_WORD0_PASSCRC_SHIFT)


/* -------------------- CPPI RX Info Word 2 definitions --------------------- */

/*! \brief CPPI RX Info Word 2 - Port To Send bit shift. */
#define ENETUDMA_CPPI_RXINFO_WORD2_TOPORT_SHIFT              (16U)

/*! \brief CPPI RX Info Word 2 - Port To Send bit mask. */
#define ENETUDMA_CPPI_RXINFO_WORD2_TOPORT_MASK               (((uint32_t) 0x1FU) << ENETUDMA_CPPI_RXINFO_WORD2_TOPORT_SHIFT)


/* -------------------- CPPI RX Control Word 1 definitions --------------------- */

/*! \brief CPPI RX Control Word 1 - Timesync Sequence Id bit shift. */
#define ENETUDMA_CPPI_RXCTRL_WORD1_SEQID_SHIFT               (0U)

/*! \brief CPPI RX Control Word 1 - Timesync Sequence Id bit mask. */
#define ENETUDMA_CPPI_RXCTRL_WORD1_SEQID_MASK                (((uint32_t) 0xFFFFU) << ENETUDMA_CPPI_RXCTRL_WORD1_SEQID_SHIFT)

/*! \brief CPPI RX Control Word 1 - Timesync Message Type bit shift. */
#define ENETUDMA_CPPI_RXCTRL_WORD1_MSGTYPE_SHIFT             (16U)

/*! \brief CPPI RX Control Word 1 - Timesync Message Type bit mask. */
#define ENETUDMA_CPPI_RXCTRL_WORD1_MSGTYPE_MASK              (((uint32_t) 0xFU) << ENETUDMA_CPPI_RXCTRL_WORD1_MSGTYPE_SHIFT)

/*! \brief CPPI RX Control Word 1 - Timesync Domain bit shift. */
#define ENETUDMA_CPPI_RXCTRL_WORD1_DOMAIN_SHIFT              (20U)

/*! \brief CPPI RX Control Word 1 - Timesync Domain bit mask. */
#define ENETUDMA_CPPI_RXCTRL_WORD1_DOMAIN_MASK               (((uint32_t) 0xFFU) << ENETUDMA_CPPI_RXCTRL_WORD1_DOMAIN_SHIFT)

/*! \brief CPPI RX Control Word 1 - Timestamp Enabled bit shift. */
#define ENETUDMA_CPPI_RXCTRL_WORD1_TSEN_SHIFT                (31U)

/*! \brief CPPI RX Control Word 1 - Timestamp Enabled bit mask. */
#define ENETUDMA_CPPI_RXCTRL_WORD1_TSEN_MASK                 (((uint32_t) 0x1U) << ENETUDMA_CPPI_RXCTRL_WORD1_TSEN_SHIFT)


/* -------------------- CPPI RX Control Word 2 definitions --------------------- */

/*! \brief CPPI RX Control Word 2 - Checksum Byte Count bit shift. */
#define ENETUDMA_CPPI_RXCTRL_WORD2_CHKSUM_BYTECNT_SHIFT      (0U)

/*! \brief CPPI RX Control Word 2 - Checksum Byte Count bit mask. */
#define ENETUDMA_CPPI_RXCTRL_WORD2_CHKSUM_BYTECNT_MASK       (((uint32_t) 0x3FFFU) << ENETUDMA_CPPI_RXCTRL_WORD1_SEQID_SHIFT)

/*! \brief CPPI RX Control Word 2 - Inverted Checksum bit shift. */
#define ENETUDMA_CPPI_RXCTRL_WORD2_CHKSUM_INV_SHIFT          (15U)

/*! \brief CPPI RX Control Word 2 - Inverted Checksum bit mask. */
#define ENETUDMA_CPPI_RXCTRL_WORD2_CHKSUM_INV_MASK           (((uint32_t) 0x1U) << ENETUDMA_CPPI_RXCTRL_WORD2_CHKSUM_INV_SHIFT)

/*! \brief CPPI RX Control Word 2 - Checksum Start Byte bit shift. */
#define ENETUDMA_CPPI_RXCTRL_WORD2_CHKSUM_STARTBYTE_SHIFT    (16U)

/*! \brief CPPI RX Control Word 2 - Checksum Start Byte bit mask. */
#define ENETUDMA_CPPI_RXCTRL_WORD2_CHKSUM_STARTBYTE_MASK     (((uint32_t) 0xFFU) << ENETUDMA_CPPI_RXCTRL_WORD2_CHKSUM_STARTBYTE_SHIFT)

/*! \brief CPPI RX Control Word 2 - Checksum Result bit shift. */
#define ENETUDMA_CPPI_RXCTRL_WORD2_CHKSUM_RESULT_SHIFT       (24U)

/*! \brief CPPI RX Control Word 2 - Checksum Result bit mask. */
#define ENETUDMA_CPPI_RXCTRL_WORD2_CHKSUM_RESULT_MASK        (((uint32_t) 0xFFU) << ENETUDMA_CPPI_RXCTRL_WORD2_CHKSUM_RESULT_SHIFT)


/* -- Utilities macros to extract fields from the CPPI TX (host RX) PSI words -- */

/*! \brief Get IPv4 flag from CPPI TX Status Word 2. */
#define ENETUDMA_CPPIPSI_GET_IPV4_FLAG(chkSumInfo) \
                ENET_FEXT(chkSumInfo, ENETUDMA_CPPI_TXSTATUS_WORD2_IPV4_VALID)

/*! \brief Get IPv6 flag from CPPI TX Status Word 2. */
#define ENETUDMA_CPPIPSI_GET_IPV6_FLAG(chkSumInfo) \
                ENET_FEXT(chkSumInfo, ENETUDMA_CPPI_TXSTATUS_WORD2_IPV6_VALID)

/*! \brief Get TCP or UDP flag from CPPI TX Status Word 2. */
#define ENETUDMA_CPPIPSI_GET_TCPUDP_N_FLAG(chkSumInfo) \
                ENET_FEXT(chkSumInfo, ENETUDMA_CPPI_TXSTATUS_WORD2_TCP_UDP_N)

/*! \brief Get IP Fragment flag from CPPI TX Status Word 2. */
#define ENETUDMA_CPPIPSI_GET_FRAGMENT_FLAG(chkSumInfo) \
                ENET_FEXT(chkSumInfo, ENETUDMA_CPPI_TXSTATUS_WORD2_FRAGMENT)

/*! \brief Get Checksum Error flag from CPPI TX Status Word 2. */
#define ENETUDMA_CPPIPSI_GET_CHKSUM_ERR_FLAG(chkSumInfo) \
                ENET_FEXT(chkSumInfo, ENETUDMA_CPPI_TXSTATUS_WORD2_CHKSUM_ERR)

/*! \brief Get Checksum Result from CPPI TX Status Word 2. */
#define ENETUDMA_CPPIPSI_GET_CHKSUM_RESULT(chkSumInfo) \
                ENET_FEXT(chkSumInfo, ENETUDMA_CPPI_TXSTATUS_WORD2_CHKSUM_ADD)


/* --- Utilities macros to insert fields into the CPPI RX (host TX) PSI words --- */

/*! \brief Set Checksum Result into CPPI RX Control Word 2. */
#define ENETUDMA_CPPIPSI_SET_CHKSUM_RES(chkSumInfo, val) \
                ENET_FINS(chkSumInfo, ENETUDMA_CPPI_RXCTRL_WORD2_CHKSUM_RESULT, val)

/*! \brief Set Checksum Start Byte into CPPI RX Control Word 2. */
#define ENETUDMA_CPPIPSI_SET_CHKSUM_STARTBYTE(chkSumInfo, val) \
                ENET_FINS(chkSumInfo, ENETUDMA_CPPI_RXCTRL_WORD2_CHKSUM_STARTBYTE, val)

/*! \brief Set Invert Checksum flag into CPPI RX Control Word 2. */
#define ENETUDMA_CPPIPSI_SET_CHKSUM_INV_FLAG(chkSumInfo, val) \
                ENET_FINS(chkSumInfo, ENETUDMA_CPPI_RXCTRL_WORD2_CHKSUM_INV, val)

/*! \brief Set Checksum Byte Count into CPPI RX Control Word 2. */
#define ENETUDMA_CPPIPSI_SET_CHKSUM_BYTECNT(chkSumInfo, val) \
                ENET_FINS(chkSumInfo, ENETUDMA_CPPI_RXCTRL_WORD2_CHKSUM_BYTECNT, val)

/*! \brief Set Timestamp enable bit into CPPI RX Control Word. 1 */
#define ENETUDMA_CPPIPSI_SET_TSEN(tsInfo, val) \
                ENET_FINS(tsInfo, ENETUDMA_CPPI_RXCTRL_WORD1_TSEN, val)

/*! \brief Set domain value into CPPI RX Control Word 1. */
#define ENETUDMA_CPPIPSI_SET_DOMAIN(tsInfo, val) \
                ENET_FINS(tsInfo, ENETUDMA_CPPI_RXCTRL_WORD1_DOMAIN, val)

/*! \brief Set message type value into CPPI RX Control Word 1. */
#define ENETUDMA_CPPIPSI_SET_MSGTYPE(tsInfo, val) \
                ENET_FINS(tsInfo, ENETUDMA_CPPI_RXCTRL_WORD1_MSGTYPE, val)

/*! \brief Set sequence Id value into CPPI RX Control Word 1. */
#define ENETUDMA_CPPIPSI_SET_SEQID(tsInfo, val) \
                ENET_FINS(tsInfo, ENETUDMA_CPPI_RXCTRL_WORD1_SEQID, val)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief Enet UDMA RX Protocol Info structure.
 *
 * It contains IPv4/IPv6 protocol information including checksum.  This is copied
 * from the CPPI TX status word (UDMA flow protocol specific info).
 */
typedef struct EnetUdma_RxProtoInfo_s
{
    /*! Flag to indicate an IPv6 TCP or UDP Packet was detected */
    bool ipv6Valid;

    /*! Flag to indicate an IPv4 TCP or UDP Packet was detected */
    bool ipv4Valid;

    /*! TCP or UDP packet detected. Valid only if #ipv4Valid or #ipv6Valid is true */
    bool tcpudpIndicator;

    /*! IP fragment detected. Valid only if #ipv4Valid or #ipv6Valid is true */
    bool fragment;

    /*! Checksum error flag. Valid only if #ipv4Valid or #ipv6Valid is true */
    bool chkSumErr;

    /*! Computed checksum value */
    uint16_t chkSum;
} EnetUdma_RxProtoInfo;

/*!
 * \brief Enet UDMA TX Protocol Info structure.
 *
 * It contains IPv4/IPv6 protcol information including checksum.  This is copied
 * to the CPPI RX control word (UDMA channel protocol specific info).
 */
typedef struct EnetUdma_TxProtoInfo_s
{
    /*! Checksum result location - packet byte number where the checksum result
     *  will be placed in the outgoing packet. The first packet byte which is
     *  the first byte of the destination address is Byte 1 */
    uint16_t chkSumResOffset;

    /*! Checksum start byte - packet byte number to start the checksum
     *  calculation on. The first packet byte is Byte 1 */
    uint16_t chkSumStartByte;

    /*! Flag to insert invalid checksum. When true a zero checksum value will be
     *  inverted and sent as FFFFh */
    bool chkSumInv;

    /*! Checksum byte count - number of bytes to calculate the checksum on. The
     *  outgoing Ethernet packet will have a checksum inserted when this value
     *  is non-zero */
    uint16_t chkSumByteCnt;
} EnetUdma_TxProtoInfo;

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

#endif /* ENET_UDMA_PSI_H_ */

/*! @} */
