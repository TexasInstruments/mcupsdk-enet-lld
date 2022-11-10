/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

#include <lwip/tcpip.h>
#include <lwip2lwipif.h>
#include <enet_cfg.h>

static struct netif netif;

void init_default_netif(const ip4_addr_t *ipaddr, const ip4_addr_t *netmask, const ip4_addr_t *gw)
{
#if ((ENET_CFG_IS_ON(CPSW_CSUM_OFFLOAD_SUPPORT) == 1) && (ENET_ENABLE_PER_CPSW == 1))
    const uint32_t checksum_offload_flags = (NETIF_CHECKSUM_GEN_UDP | NETIF_CHECKSUM_GEN_TCP | NETIF_CHECKSUM_CHECK_TCP | NETIF_CHECKSUM_CHECK_UDP);
    const uint32_t checksum_flags = (NETIF_CHECKSUM_ENABLE_ALL & ~checksum_offload_flags);
#endif
    netif_add(&netif, ipaddr, netmask, gw, NULL, LWIPIF_LWIP_init, tcpip_input);
    LWIPIF_LWIP_cfg(&netif);
    netif_set_default(&netif);
#if ((ENET_CFG_IS_ON(CPSW_CSUM_OFFLOAD_SUPPORT) == 1) && (ENET_ENABLE_PER_CPSW == 1))
    NETIF_SET_CHECKSUM_CTRL(&netif, checksum_flags);
#endif
}

void default_netif_poll(void)
{
}

void default_netif_shutdown(void)
{
    /* netif_default will internally be reset */
    LOCK_TCPIP_CORE();
    netif_remove(&netif);
    UNLOCK_TCPIP_CORE();
}

