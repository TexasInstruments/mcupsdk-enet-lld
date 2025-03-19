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
#ifndef ETHER_RING_LWIP_TCP_CONFIG_H_
#define ETHER_RING_LWIP_TCP_CONFIG_H_

#include <lwip/netif.h>

/*!
 * \brief This structure store Server Details
 *
 * The parameter structure for ServerInfo
 */
struct EnetApp_hostInfo
{
    /* Server IP Address */
    ip_addr_t ipAddr;

    /* Server port for connection */
    uint16_t port;
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/*!
 * \brief Sets up the TCP/IP Network Stack.
 *
 * \param void
 *
 * \return \ref void
 */
void EnetApp_setupNetworkStack();

/*!
 * \brief Checks the Network Link Status
 *
 * \param pNetif     [IN] Pointer to the Netif
 *
 * \return \ref void
 */
int32_t EnetApp_isNetworkUp(struct netif* pNetif);

/*!
 * \brief Starts TCP Client Application
 *
 * \param void
 *
 * \return \ref void
 */
void EnetApp_startTcpClient();

/*!
 * \brief Starts TCP Server Application
 *
 * \param void
 *
 * \return \ref void
 */
void EnetApp_startTcpServer();

/*!
 * \brief Fills the Socket Configuration for Client App
 *
 * \param void
 *
 * \return \ref void
 */
void EnetApp_fillHostSocketInfo(struct EnetApp_hostInfo* pHostInfo);

#endif /* ETHER_RING_LWIP_TCP_CONFIG_H_ */
