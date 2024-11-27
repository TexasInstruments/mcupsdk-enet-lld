#include <lwip/mem.h>
#include <lwip/tcpip.h>
#include <lwip2lwipif_ic.h>

static struct netif netif_ic;
void init_ic_netif(const ip4_addr_t *ipaddr, const ip4_addr_t *netmask, const ip4_addr_t *gw)
{
    netif_add(&netif_ic, ipaddr, netmask, gw, NULL, LWIPIF_LWIP_IC_init, tcpip_input);
}

void ic_netif_poll(void)
{
}

void ic_netif_shutdown(void)
{
    /* netif_default will internally be reset */
    LOCK_TCPIP_CORE();
    netif_remove(&netif_ic);
    UNLOCK_TCPIP_CORE();
}

