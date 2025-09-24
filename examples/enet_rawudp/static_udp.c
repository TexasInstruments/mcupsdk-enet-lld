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
 * \file  static_udp.c
 *
 * \brief This file contains the code for creating IP and UDP headers and API for sending UDP packets to layer 2.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <enet_apputils.h>      
#include "static_udp.h" 

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void EnetApp_setEthFrameHdr(EthFrameHeader *frameHdr,const EnetUDP_AddrInfo * enetAppAddrInfo);

static void EnetApp_setIPv4Hdr(EthAppIPv4Header *ipHdr,const EnetUDP_AddrInfo * enetAppAddrInfo);

static void EnetApp_setUDPLiteHdr(EthAppUDPLiteHeader *udpHdr,const EnetUDP_AddrInfo * enetAppAddrInfo);

static uint16_t EnetApp_getInetChecksum(uint8_t *data, uint32_t len);

static uint16_t EnetApp_getUdpliteChecksum(const uint8_t *packet);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetApp_createCompleteHeader(uint8_t * header,uint32_t payload_len,const EnetUDP_AddrInfo * enetAppAddrInfo)
{
    EthFrame* frame = (EthFrame *)header;

    memset(header, 0, COMBINED_HEADER_SIZE);

    EnetApp_setEthFrameHdr(&(frame->hdr),enetAppAddrInfo);

    uint8_t * bufPtr = frame->payload;

    EthAppIPv4Header *ipHdr  = (EthAppIPv4Header *)(bufPtr + IP_HEADER_OFFSET);
    EnetApp_setIPv4Hdr(ipHdr,enetAppAddrInfo);
    
    EthAppUDPLiteHeader *udpHdr = (EthAppUDPLiteHeader *)(bufPtr + UDP_HEADER_OFFSET);
    EnetApp_setUDPLiteHdr(udpHdr,enetAppAddrInfo);
    
    ipHdr->hdrChksum   = 0U;
    udpHdr->csum       = 0U;
    ipHdr->totalPktLen = Enet_htons((sizeof(EthAppIPv4Header) + sizeof(EthAppUDPLiteHeader) + payload_len));
    ipHdr->hdrChksum   = Enet_htons(EnetApp_getInetChecksum((uint8_t *)ipHdr, sizeof(EthAppIPv4Header)));
    udpHdr->csum       = Enet_htons(EnetApp_getUdpliteChecksum(bufPtr));
}

uint32_t EnetApp_sendUDP(uint8_t* header,uint8_t *payload, const uint32_t payload_len)
{
    int32_t status = ENET_SOK;

    if(payload_len > MAX_PAYLOAD_LEN)
    {
        EnetAppUtils_print("Invalid payload length\r\n");
        return ENET_EFAIL;
    }
    if(payload == NULL)
    {
        EnetAppUtils_print("Invalid payload pointer\r\n");
        return ENET_EFAIL;
    }
    
    EnetApp_sendFrame(header,COMBINED_HEADER_SIZE,payload,payload_len);

    return status;
}
/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static void EnetApp_setEthFrameHdr(EthFrameHeader *frameHdr,const EnetUDP_AddrInfo * enetAppAddrInfo)
{
    for(int i=0;i<ETH_MAC_ADDR_LEN;i++){
        frameHdr->srcMac[i] = enetAppAddrInfo->srcMac[i];
        frameHdr->dstMac[i] = enetAppAddrInfo->dstMac[i];
    }
    frameHdr->etherType = Enet_htons(ENET_ETHERTYPE_IPV4);
}

static void EnetApp_setIPv4Hdr( EthAppIPv4Header * ipv4Hdr,const EnetUDP_AddrInfo * enetAppAddrInfo)
{
    ipv4Hdr->verIHL = IPV4_HDR_VER_IHL;
    ipv4Hdr->tos    = IPV4_HDR_TOS;
    ipv4Hdr->totalPktLen = Enet_htons(IPV4_HDR_TOTAL_PKT_LEN);
    ipv4Hdr->ipId = Enet_htons(IPV4_HDR_IPID);
    ipv4Hdr->flagFragOffset = Enet_htons(IPV4_HDR_FLAGFRAFOFFSET);
    ipv4Hdr->ttl = IPV4_HDR_TTL;
    ipv4Hdr->protocol = IPV4_HDR_UDPLITE;
    ipv4Hdr->hdrChksum = 0U;
    memcpy(&ipv4Hdr->srcIP,enetAppAddrInfo->srcIP,sizeof(ipv4Hdr->srcIP));
    memcpy(&ipv4Hdr->dstIP,enetAppAddrInfo->dstIP,sizeof(ipv4Hdr->dstIP));
}

static void EnetApp_setUDPLiteHdr(EthAppUDPLiteHeader *udpHdr,const EnetUDP_AddrInfo * enetAppAddrInfo)
{
    udpHdr->srcPort = Enet_htons(enetAppAddrInfo->srcPortUDP);
    udpHdr->dstPort = Enet_htons(enetAppAddrInfo->dstPortUDP);
    udpHdr->csumCoverage = Enet_htons(UDPLITE_HDR_CSUMCOVERAGE);
    udpHdr->csum = 0U;
}
/* Create the Ethernet Ipv4 UDP lite header */
static uint16_t EnetApp_getUdpliteChecksum(const uint8_t *data)
{
    const uint8_t *packet = (uint8_t *)(data + IP_HEADER_OFFSET);
    uint8_t ip_header_len = sizeof(EthAppIPv4Header);
    const uint8_t *udp_header = packet + ip_header_len;

    uint16_t total_length = (packet[2] << 8) | packet[3];
    uint16_t udp_offset = ip_header_len;
    uint16_t available_payload = (total_length > udp_offset) ? (total_length - udp_offset) : 0;

    uint32_t sum = 0;
    uint32_t i;

    /* Source IP */
    for (i = 12; i < 16; i += 2)
        sum += (packet[i] << 8) | packet[i+1];

    /* Destination IP */
    for (i = 16; i < 20; i += 2)
        sum += (packet[i] << 8) | packet[i+1];

    sum += (IPV4_HDR_UDPLITE);  
    sum += available_payload;

    /* Extract coverage field (bytes 6-7 of UDP-Lite header)*/
    uint16_t coverage = (udp_header[4] << 8) | udp_header[5];

    if (coverage > available_payload)
    {
        coverage = available_payload;
    }

    /* UDP-Lite header + covered data */
    for (i = 0; i < coverage; i += 2) {
        uint16_t word = udp_header[i] << 8;
        if (i + 1 < coverage)
            word |= udp_header[i + 1];
        sum += word;
    }

    /* Fold to 16 bits */
    while (sum >> 16)
        sum = (sum & 0xFFFF) + (sum >> 16);

    return (uint16_t)(~sum);
}

static uint16_t EnetApp_getInetChecksum(uint8_t *data, uint32_t len)
{
    uint32_t sum = 0;

    for (uint32_t i = 0; i < len; i += 2) {
        uint16_t word = data[i] << 8;
        if (i + 1 < len) {
            word |= (uint16_t)data[i + 1];  /* Combine two bytes into a 16-bit word (little endian) */
        }
        sum += word;

    }
    /* Fold 32-bit to 16-bit */
    while (sum >> 16)
    {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    return (uint16_t)(~sum); 
}
