//###########################################################################
//
// FILE:   lwip2lwipif.c
//
// TITLE:  lwIP Interface port file.
//
//###########################################################################
// $TI Release: $
// $Release Date: $
// $Copyright: $
//###########################################################################

/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/**
 * Copyright (c) 2018 Texas Instruments Incorporated
 *
 * This file is dervied from the ``ethernetif.c'' skeleton Ethernet network
 * interface driver for lwIP.
 *
 */

/* Standard language headers */
#include <stdio.h>

#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>

/**
 * lwIP specific header files
 */
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/pbuf.h"
#include <lwip/stats.h>
#include <lwip/netifapi.h>
#include <lwip/ip_addr.h>
#include "netif/etharp.h"
#include "lwip/inet_chksum.h"
#include "lwip/prot/ethernet.h"
#include "lwip/prot/ip.h"
#include "lwip/prot/udp.h"
#include "lwip/prot/tcp.h"

/* This module's header */
#include "lwip2enet.h"
#include "lwip2lwipif.h"
#include "lwip2lwipif_priv.h"
#include "lwipopts.h"

/* Define those to better describe your network interface. */
#define IFNAME0 't'
#define IFNAME1 'i'

//TODO this should come from stack
/* Maximum Ethernet Payload Size. */
#ifdef _INCLUDE_JUMBOFRAME_SUPPORT
#define ETH_MAX_PAYLOAD  10236
#else
#define ETH_MAX_PAYLOAD  1514
#endif

#define VLAN_TAG_SIZE         (4U)
#define ETH_FRAME_SIZE        (ETH_MAX_PAYLOAD + VLAN_TAG_SIZE)

/*---------------- */
/* Ethernet Header */
#define ETHHDR_SIZE     14

/*---------------------------------------------------------------------------*\
 |                             Function Definitions                           |
 \*---------------------------------------------------------------------------*/


inline uint8_t* LWIPIF_LWIP_getIpPktStart(uint8_t* pEthpkt)
{
    const u16_t type = ((struct eth_hdr*)pEthpkt)->type;
    const uint32_t ipPacketStartOffset = (type == PP_HTONS(ETHTYPE_VLAN)) ?
                                         (SIZEOF_ETH_HDR + SIZEOF_VLAN_HDR) : (SIZEOF_ETH_HDR);

    return &pEthpkt[ipPacketStartOffset];
}

static inline void LWIPIF_LWIP_getSrcIp(uint8_t *pIpPkt, ip_addr_t* pIpAddr)
{
    struct ip_hdr* pIpHdr = (struct ip_hdr*)pIpPkt;
    ip_addr_copy_from_ip4(*pIpAddr, pIpHdr->src);
}

static inline void LWIPIF_LWIP_getDstIp(uint8_t *pIpPkt, ip_addr_t* pIpAddr)
{
    struct ip_hdr* pIpHdr = (struct ip_hdr*)pIpPkt;
    ip_addr_copy_from_ip4(*pIpAddr, pIpHdr->dest);
}

static uint32_t LWIPIF_LWIP_getUdpLiteChksum(struct pbuf* p, const uint32_t ipHdrLength,
                                             const uint32_t udpPktLength, struct udp_hdr* pUdpLiteHdr,
                                            const ip_addr_t *pSrcIp, const ip_addr_t *pDstIp)
{
    /* For UDP-Lite, checksum length of 0 means checksum
       over the complete packet. (See RFC 3828 chap. 3.1)
       At least the UDP-Lite header must be covered by the
       checksum, therefore, if chksum_len has an illegal
       value, we generate the checksum over the complete
       packet to be safe. */
    const uint32_t chkSumCoverageLength = (lwip_ntohs(pUdpLiteHdr->len) == 0) ? udpPktLength : lwip_ntohs(pUdpLiteHdr->len);
    uint8_t *pEthPkt = (uint8_t*) p->payload;

    const uint32_t updlitePktStartOffset = (uint8_t *)pUdpLiteHdr - (uint8_t *)pEthPkt;

    // 1. move the packet buff start to UDP-Lite packet start from EthPkt start
    p->payload  = &(pEthPkt[updlitePktStartOffset]);
    p->tot_len -= updlitePktStartOffset;
    p->len     -= updlitePktStartOffset;

    // 2. Compute checksum for UDP Lite Packet
    const uint32_t udpLiteChkSum = ip_chksum_pseudo_partial(p, IP_PROTO_UDPLITE, udpPktLength, chkSumCoverageLength, pSrcIp, pDstIp);

    // 3. Revert back to EthPkt start point from UDP-Lite Packet start
    p->payload  = pEthPkt;
    p->tot_len += updlitePktStartOffset;
    p->len     += updlitePktStartOffset;

    return udpLiteChkSum;
}

bool LWIPIF_LWIP_UdpLiteValidateChkSum(struct pbuf *p)
{
    Lwip2Enet_assert(p->len >= (sizeof(struct ip_hdr) + sizeof(struct eth_hdr) + sizeof(struct udp_hdr)));

    bool isChksumPass = true;
    struct ip_hdr *pIpPkt   = (struct ip_hdr *)LWIPIF_LWIP_getIpPktStart((uint8_t*) p->payload);
    uint8_t *pIpPayload     = (uint8_t*)pIpPkt + (IPH_HL(pIpPkt) << 2);
    struct udp_hdr* pUdpHdr = (struct udp_hdr*)pIpPayload;
    ip_addr_t srcIp;
    ip_addr_t dstIp;

    LWIPIF_LWIP_getSrcIp((uint8_t *)pIpPkt, &srcIp);
    LWIPIF_LWIP_getDstIp((uint8_t *)pIpPkt, &dstIp);

    const uint32_t chkSumCovLen = (lwip_ntohs(pUdpHdr->len) == 0) ? (lwip_ntohs(IPH_LEN(pIpPkt)) - (IPH_HL(pIpPkt) << 2)) : lwip_ntohs(pUdpHdr->len);

    if (chkSumCovLen < sizeof(struct udp_hdr))
    {
        isChksumPass = false;
        return false;
    }

    if (0 == LWIPIF_LWIP_getUdpLiteChksum(p, (IPH_HL(pIpPkt) << 2), (lwip_ntohs(IPH_LEN(pIpPkt)) - (IPH_HL(pIpPkt) << 2)), pUdpHdr, &srcIp, &dstIp))
    {
        isChksumPass = true;
    }
    else
    {
        isChksumPass = false;
    }

    /* Return value should indicate true if checksum error found */
    return (!isChksumPass);
}

uint32_t LWIPIF_LWIP_getChkSumInfo(struct pbuf *p)
{
    /* Note: This function assumes pbuf 'p' has atleast ETH_HEADER, IP_HEADER and DATAGRAM (TCP/UDP) HEADER in the first node of pbuf */

    Lwip2Enet_assert(p != NULL);

    uint32_t chkSumInfo     = 0;
    struct eth_hdr *pEthPkt = (struct eth_hdr *) p->payload;
    struct ip_hdr  *pIpPkt  = (struct ip_hdr *) LWIPIF_LWIP_getIpPktStart((uint8_t*)pEthPkt);

    Lwip2Enet_assert(p->len >= (sizeof(struct ip_hdr) + sizeof(struct eth_hdr)));
    const uint32_t ipPktHdrLen     = (IPH_HL(pIpPkt) << 2)  /* multiply by 4 */;
    const uint32_t ipPktPayloadLen = lwip_ntohs(IPH_LEN(pIpPkt)) - ipPktHdrLen;
    const uint32_t protocolType    = IPH_PROTO(pIpPkt);
    ip_addr_t srcIp;
    ip_addr_t dstIp;

    uint8_t *pIpPayload   = (uint8_t*)pIpPkt + ipPktHdrLen;

    LWIPIF_LWIP_getSrcIp((uint8_t *)pIpPkt, &srcIp);
    LWIPIF_LWIP_getDstIp((uint8_t *)pIpPkt, &dstIp);

    switch (protocolType)
    {
        case IP_PROTO_UDPLITE:
        {
            Lwip2Enet_assert(p->len >= (sizeof(struct ip_hdr) + sizeof(struct eth_hdr) + sizeof(struct udp_hdr)));
            struct udp_hdr* pUdpHdr = (struct udp_hdr*)pIpPayload;

            if (pUdpHdr->chksum == 0U)
            {
                /* checksum is valid and not computed by stack */
                /* calculate the checksum in software and fill the corresponding field */
                pUdpHdr->chksum = LWIPIF_LWIP_getUdpLiteChksum(p, ipPktHdrLen, ipPktPayloadLen, pUdpHdr, &srcIp, &dstIp);
            }
            chkSumInfo = 0;
        }
        break;
        case IP_PROTO_UDP:
        case IP_PROTO_TCP:
        {
            uint8_t csumCoverageStartByte = 0;
            uint8_t csumResultByte = 0;
            uint16_t pseudoIpHdrChkSum = 0;

            if (protocolType == IP_PROTO_UDP)
            {
                Lwip2Enet_assert(p->len >= (sizeof(struct ip_hdr) + sizeof(struct eth_hdr) + sizeof(struct udp_hdr)));
                struct udp_hdr* pUdpHdr = (struct udp_hdr*)pIpPayload;

                if (pUdpHdr->chksum == 0U)
                {
                    /* checksum is valid and not computed by stack */

                    csumCoverageStartByte = (uint8_t*)pUdpHdr - (uint8_t*)pEthPkt + 1; /* CPSW cksum info indexing starts from 1 */
                    csumResultByte = (uint8_t*)(&(pUdpHdr->chksum)) - (uint8_t*)pEthPkt + 1;
                    pseudoIpHdrChkSum = ~(ip_chksum_pseudo(NULL, IP_PROTO_UDP, ipPktPayloadLen, &srcIp, &dstIp));
                    pUdpHdr->chksum = pseudoIpHdrChkSum;
                    ENETDMA_TXCSUMINFO_SET_CHKSUM_BYTECNT(chkSumInfo, ipPktPayloadLen);
                    ENETDMA_TXCSUMINFO_SET_CHKSUM_STARTBYTE(chkSumInfo, csumCoverageStartByte);
                    ENETDMA_TXCSUMINFO_SET_CHKSUM_RESBYTE(chkSumInfo, csumResultByte);
                }
            }
            else if (protocolType == IP_PROTO_TCP)
            {
                Lwip2Enet_assert(p->len >= (sizeof(struct ip_hdr) + sizeof(struct eth_hdr) + sizeof(struct tcp_hdr)));
                struct tcp_hdr* pTcpHdr = (struct tcp_hdr*)pIpPayload;

                if (pTcpHdr->chksum == 0U)
                {
                    csumCoverageStartByte = (uint8_t*)pTcpHdr - (uint8_t*)pEthPkt + 1; /* CPSW cksum info indexing starts from 1 */
                    csumResultByte = (uint8_t*)(&(pTcpHdr->chksum)) - (uint8_t*)pEthPkt + 1;
                    pseudoIpHdrChkSum = ~(ip_chksum_pseudo(NULL, IP_PROTO_TCP, ipPktPayloadLen, &srcIp, &dstIp));
                    pTcpHdr->chksum = pseudoIpHdrChkSum;
                    ENETDMA_TXCSUMINFO_SET_CHKSUM_BYTECNT(chkSumInfo, ipPktPayloadLen);
                    ENETDMA_TXCSUMINFO_SET_CHKSUM_STARTBYTE(chkSumInfo, csumCoverageStartByte);
                    ENETDMA_TXCSUMINFO_SET_CHKSUM_RESBYTE(chkSumInfo, csumResultByte);
                }
            }
            else
            {
                EnetUtils_printf("[LWIPIF_LWIP] Un-Supported protocol for HW checksum offload\r\n");
                chkSumInfo = 0;
            }
        }
        break;
        default:
        {
            chkSumInfo = 0;
        }
    }

    return chkSumInfo;
}

/*!
 *  @b LWIPIF_LWIP_input
 *  @n
 *  This is currently a task which consumes the RX packets retrieved from
 *  the driver in RX packet task, and passes them to the LwIP stack via
 *  netif->input().
 *
 *  \param[in]  netif
 *      NETIF_DEVICE structure pointer.
 *
 *  \retval
 *      void
 */
void LWIPIF_LWIP_input(Lwip2Enet_RxObj *rx,
                       struct netif* netif,
                       struct pbuf *hPbufPacket)
{
    Lwip2Enet_assert(netif != NULL);

    /* Pass the packet to the LwIP stack */
    if (netif->input(hPbufPacket, netif) != ERR_OK)
    {
        LWIP_DEBUGF(NETIF_DEBUG, ("lwipif_input: IP input error\n"));
        if (!ENET_UTILS_IS_ALIGNED(hPbufPacket->payload, ENETDMA_CACHELINE_ALIGNMENT))
        {
            Lwip2Enet_assert(FALSE);
        }
        /* Put the new packet on the free queue */
        pbuf_free(hPbufPacket);

        LWIP2ENETSTATS_ADDONE(&rx->stats.freePbufPktEnq);
        LWIP2ENETSTATS_ADDONE(&rx->stats.rxLwipInputFail);
    }
    else
    {
        LWIP2ENETSTATS_ADDONE(&rx->stats.stackNotifyCnt);
    }
}

void LWIPIF_LWIP_periodic_polling(struct netif *netif)
{
        Lwip2Enet_netif_t* pInterface = (Lwip2Enet_netif_t*)netif->state;
        Lwip2Enet_assert(pInterface != NULL);

        /* Periodic Function to update Link status */
        Lwip2Enet_periodicFxn(netif);

        if (!(pInterface->isLinkUp == (netif->flags & 0x04U) >> 2))
        {
            if (pInterface->isLinkUp)
            {
                netif_set_link_up(netif);
            }
            else
            {
                netif_set_link_down(netif);
            }
        }
}

int32_t LWIPIF_LWIP_start(Enet_Type enetType, uint32_t instId, struct netif *netif, uint32_t netifIdx)
{
    int32_t retVal = 0U;
    Lwip2Enet_Handle hLwip2Enet;

    /* Open the translation layer, which itself opens the hardware driver */
    hLwip2Enet = Lwip2Enet_open(enetType, instId, netif, netifIdx);

    if (NULL == hLwip2Enet)
    {
        /* Note - Use System_printf here as we are not sure if hLwip2Enet print
         * is set and not null. */
        EnetUtils_printf("[LWIPIF_LWIP] Failed to start Enet\r\n");
        retVal = -1;
    }
    return retVal;
}

void LWIPIF_LWIP_setNotifyCallbacks(struct netif *netif, Enet_notify_t *pRxNotify, Enet_notify_t *pTxNotify)
{
    Lwip2Enet_netif_t* pInterface = (Lwip2Enet_netif_t*)netif->state;
    for (uint32_t idx = 0; idx < pInterface->count_hTx; idx++)
    {
        Lwip2Enet_setTxNotifyCallback(pInterface->hTx[idx], pTxNotify);
    }

    for (uint32_t idx = 0; idx < pInterface->count_hRx; idx++)
    {
        Lwip2Enet_setRxNotifyCallback(pInterface->hRx[idx], pRxNotify);
    }
}

/*!
 *  @b LWIPIF_LWIP_Stop
 *  @n
 *  The function is used to de-initialize and stop the Enet
 *  controller and device.
 *
 *  \param[in] netif
 *      NETIF structure pointer.
 */
static void LWIPIF_LWIP_stop(struct netif *netif)
{
    Lwip2Enet_Handle hLwip2Enet;

    /* Get the pointer to the private data */
    hLwip2Enet = (Lwip2Enet_Handle)netif->state;

    /* Call low-level close function */
    Lwip2Enet_close(hLwip2Enet, netif);

    /* Enet controller has been stopped. */
}

/*!
 *  @b LWIPIF_LWIP_Init
 *  @n
 *  The function is used to initialize and register the peripheral
 *  with the stack.
 *
 *  \param[in]  *netif
 *      NETIF structure pointer
 *
 *  \retval
 *      Success -   ERR_OK
 */
err_t LWIPIF_LWIP_init(struct netif *netif)
{
    /* Populate the Network Interface Object */
    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;

    /*
     * MTU is i total size of the (IP) packet that can fit into an Ethernet.
     * For Ethernet it is 1500bytes
     */
    netif->mtu = ETH_FRAME_SIZE - ETHHDR_SIZE - VLAN_TAG_SIZE;

    /* Populate the Driver Interface Functions. */
    netif->remove_callback      = LWIPIF_LWIP_stop;
    netif->output               = etharp_output;
    netif->linkoutput           = LWIPIF_LWIP_send;
    netif->flags               |= NETIF_FLAG_ETHARP;

#if !ENET_CFG_IS_ON(CPSW_CSUM_OFFLOAD_SUPPORT)
    EnetUtils_printf("[LWIPIF_LWIP] LwIP Checksum offload support disabled\r\n");
#endif
    EnetUtils_printf("[LWIPIF_LWIP] NETIF INIT SUCCESS\r\n");

    return ERR_OK;
}

void LWIPIF_LWIP_rxPktHandler(struct netif *netif)
{
    Lwip2Enet_netif_t* pInterface = (Lwip2Enet_netif_t*)netif->state;
    for (uint32_t idx = 0; idx < pInterface->count_hRx; idx++)
    {
        Lwip2Enet_rxPktHandler(pInterface->hRx[idx]);
    }
}

void LWIPIF_LWIP_txPktHandler(struct netif *netif)
{
    Lwip2Enet_netif_t* pInterface = (Lwip2Enet_netif_t*)netif->state;
    for (uint32_t idx = 0; idx < pInterface->count_hTx; idx++)
    {
        Lwip2Enet_txPktHandler(pInterface->hTx[idx]);
    }
}

err_t LWIPIF_LWIP_send(struct netif *netif, struct pbuf *p)
{
    /* Get the pointer to the private data */
    Lwip2Enet_netif_t* pInterface  = (Lwip2Enet_netif_t*) netif->state;
    const Enet_MacPort macPort = pInterface->macPort;
    Lwip2Enet_TxHandle hTx = pInterface->hTx[0];

    Lwip2Enet_assert(pInterface != NULL);
    Lwip2Enet_assert(hTx != NULL);
    /*
     * When transmitting a packet, the buffer may be deleted before transmission by the
     * stack. The stack implements a 'ref' feature within the buffers. The following happens
     * internally:
     *  If p->ref > 1, ref--;
     *  If p->ref == 1, free(p);
     * pbuf_ref(p) increments the ref.
     */
    pbuf_ref(p);
    /* Enqueue the packet */
    pbufQ_enQ(&hTx->readyPbufQ, p);
    LWIP2ENETSTATS_ADDONE(&hTx->stats.readyPbufPktEnq);
    /* Pass the packet to the translation layer */
    Lwip2Enet_sendTxPackets(pInterface, macPort);
    /* Packet has been successfully transmitted or enqueued to be sent when link comes up */
    return ERR_OK;
}
