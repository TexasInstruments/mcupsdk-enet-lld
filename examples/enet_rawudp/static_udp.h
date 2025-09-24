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

 /*!
 * \file  static_udp.h
 *
 * \brief This file is the header file for static_udp.c and stores some IP and UDP releated macros.
 */

#ifndef UDP_LITE_H
#define UDP_LITE_H

#include "app_enethandler.h"

#define MAX_PAYLOAD_LEN         (1460U)
#define ENET_IP_ADDR_LEN        (4U)
#define ENET_ETHERTYPE_IPV4     (0x0800)
 
#define ETHERNET_HEADER_SIZE    (14U)
#define IPV4_HEADER_SIZE        (20U)
#define UDP_HEADER_SIZE         (8U)
#define COMBINED_HEADER_SIZE    (ETHERNET_HEADER_SIZE + IPV4_HEADER_SIZE + UDP_HEADER_SIZE)

#define IP_HEADER_OFFSET        (0U)
#define UDP_HEADER_OFFSET       (20U)
#define PAYLOAD_OFFSET          (28U)

#define MAX_PKT_LEN             (1460U)

#define IPV4_HDR_VER_IHL        ((0x4 << 4) | 0x5)
#define IPV4_HDR_TOS            (0x00)
#define IPV4_HDR_TOTAL_PKT_LEN  (1498U)
#define IPV4_HDR_IPID           (0x28)
#define IPV4_HDR_FLAGFRAFOFFSET (0x0000)
#define IPV4_HDR_TTL            (0xFF)
#define IPV4_HDR_UDPLITE        (0x88)
#define IPV4_HDR_UDP            (0x11)

#define UDPLITE_HDR_CSUMCOVERAGE   (0x0008)

/*!
 * \brief IPv4 hrader
 */
typedef struct
{
    /*! Version (4) and Internet Header Length (5) combined */
    uint8_t verIHL;
    /*! Type-of-service field */
    uint8_t tos;
    /*! Total length of the IPv4 packet (header + UDP Lite header + payload) in bytes */
    uint16_t totalPktLen;
    /*! Identification field */
    uint16_t ipId;
    /*! Flags and fragment offset */
    uint16_t flagFragOffset;
    /*! Time-to-live value */
    uint8_t  ttl;
    /*! Protocol identifier (e.g., @c IPV4_HDR_UDPLITE or @c IPV4_HDR_UDP) */
    uint8_t protocol;
    /*! Header checksum (calculated after the header is filled) */
    uint16_t hdrChksum;
    /*! Source IPv4 address (4 bytes) */
    uint8_t  srcIP[ENET_IP_ADDR_LEN];
    /*! Destination IPv4 address (4 bytes) */
    uint8_t  dstIP[ENET_IP_ADDR_LEN];
} __attribute__ ((packed)) EthAppIPv4Header;

/*!
 * \brief UDP Lite hrader
 */
typedef struct
{
    /*! Source port number */
    uint16_t srcPort;
    /*! Destination port number */
    uint16_t dstPort;
    /*! Coverage field (bytes 4-5 of UDP Lite header) */
    uint16_t csumCoverage;
    /*! Checksum (calculated after the header is filled) */
    uint16_t csum;
} __attribute__ ((packed)) EthAppUDPLiteHeader;

/*!
 * \brief Structure for MAC,IP,Port information
 */
typedef struct EnetUDP_AddrInfo_s
{
    uint8_t srcMac[ENET_MAC_ADDR_LEN];
    uint8_t dstMac[ENET_MAC_ADDR_LEN];
    uint8_t srcIP[ENET_IP_ADDR_LEN];
    uint8_t dstIP[ENET_IP_ADDR_LEN];
    uint16_t srcPortUDP;
    uint16_t dstPortUDP;
} EnetUDP_AddrInfo;

/**
 * @brief Creates a complete Ethernet/IPv4/UDP‑Lite header in the provided buffer.
 *
 * The function zeroes the combined header area, fills the Ethernet,
 * IPv4, and UDP‑Lite sub‑headers based on the supplied address information,
 * and calculates the required checksums. The total packet length field
 * in the IPv4 header is set to include the IPv4 header,‑Lite header,
 * and the payload length.
 *
 * @param[in,out] header          Pointer to the buffer where the combined header will be written.
 * @param[in]      payload_len    Length of the payload that will follow the header.
 * @param[in]      enetAppAddrInfo Pointer to an EnetUDP_AddrInfo structure containing source/destination MAC and IP/port information.
 *
 * @note This function does not return a value; it populates the buffer directly.
 */
void EnetApp_createCompleteHeader(uint8_t * header,uint32_t payload_len,const EnetUDP_AddrInfo * enetAppAddrInfo);

/**
 * @brief Sends a UDP frame with the specified header and payload.
 *
 * @param header      Pointer to the header buffer (size COMBINED_HEADER_SIZE).
 * @param payload     Pointer to the payload data.
 * @param payload_len Length of the payload in bytes; must not exceed MAX_PAYLOAD_LEN.
 *
 * @return ENET_SOK on success, ENET_EFAIL if the payload length is invalid or the payload pointer is NULL.
 */
uint32_t EnetApp_sendUDP(uint8_t *header,uint8_t *payload, const uint32_t len );

#endif
