/*
 *  Copyright (C) 2018-2024 Texas Instruments Incorporated
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


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* C runtime includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* NetX includes */
#include <netxduo_enet.h>
#include <tx_port.h>
#include <nx_api.h>
#include <nxd_dhcp_client.h>
#include <nx_web_http_server.h>

/* FileX includes */
#include <fx_api.h>


/* App includes */
#include "app_cpswconfighandler.h"

/* SDK includes */
#include <enet_apputils.h>
#include "ti_board_config.h"
#include "ti_enet_netxduo.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>

#include <nx_secure_tls_api.h>
#include <nx_crypto_sa2ul.h>


#if (NETXDUO_COUNT > 1u)
#error "This example does not support more than one Netx instance."
#endif

#if (NETXDUO_IF_COUNT > 1u)
#error "This example does not support more than one Netx interface."
#endif


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


#define PACKET_SIZE                    (1536)
#define USER_POOL_SIZE                 ((sizeof(NX_PACKET) + PACKET_SIZE) * (ENET_SYSCFG_TOTAL_NUM_TX_PKT / 2u))
#define INTERNAL_POOL_SIZE             ((sizeof(NX_PACKET) + PACKET_SIZE) * (ENET_SYSCFG_TOTAL_NUM_TX_PKT / 2u + ENET_SYSCFG_TOTAL_NUM_RX_PKT))

#define IP_THREAD_STACK_SIZE           (4096u)
#define IP_ARP_THREAD_STACK_SIZE       (4096u)
#define HTTP_SERVER_STACK_SIZE         (4096u)

#define SERVER_PORT                    (443u)


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static const uint8_t BROADCAST_MAC_ADDRESS[ENET_MAC_ADDR_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static NX_SECURE_X509_CERT certificate;

static uint8_t gIpThreadStack[IP_THREAD_STACK_SIZE]__attribute__((aligned(ENET_UTILS_CACHELINE_SIZE)));
static uint8_t gIpArpThreadStack[IP_ARP_THREAD_STACK_SIZE]__attribute__((aligned(ENET_UTILS_CACHELINE_SIZE)));
static uint8_t gHttpServerStack[HTTP_SERVER_STACK_SIZE]__attribute__((aligned(ENET_UTILS_CACHELINE_SIZE)));
static uint8_t gUserPoolMem[USER_POOL_SIZE]__attribute__ ((aligned(ENETDMA_CACHELINE_ALIGNMENT), section(".bss:ENET_DMA_PKT_MEMPOOL")));
static uint8_t gInternalPoolMem[INTERNAL_POOL_SIZE]__attribute__ ((aligned(ENETDMA_CACHELINE_ALIGNMENT), section(".bss:ENET_DMA_PKT_MEMPOOL")));

static NX_PACKET_POOL gInternalPacketPool;
static NX_PACKET_POOL gUserPacketPool;
static NX_IP gIp;
static NX_DHCP gDhcpClient;
static NX_TCP_SOCKET gServerSocket;
static NX_WEB_HTTP_SERVER gHttpServer;

/* NetX built-in ciphers. */
extern const NX_SECURE_TLS_CRYPTO nx_crypto_tls_ciphers;
extern const NX_SECURE_TLS_CRYPTO nx_crypto_tls_ciphers_ecc;
extern const USHORT nx_crypto_ecc_supported_groups[];
extern const NX_CRYPTO_METHOD *nx_crypto_ecc_curves[];
extern const UINT nx_crypto_ecc_supported_groups_size;


/* Define the TLS packet reassembly buffer. */
UCHAR tls_packet_buffer[18000];


/* Define the metadata area for TLS cryptography. The actual size needed can be
 * ascertained by calling nx_secure_tls_metadata_size_calculate.
 */
UCHAR tls_crypto_metadata[30000];




/* Binary data for the TLS Server X.509 certificate, ASN.1 DER-encoded.
 * Generated along with the private key using:
 * openssl req -x509 -nodes -days 365 -new -newkey ec -pkeyopt ec_paramgen_curve:prime256v1 -keyout private_key.key -out self_signed_cert.crt
 * Converted to binary and output in the form of a C array using:
 * openssl x509 -outform der -in self_signed_cert.crt -out self_signed_cert.der
 * xxd -i self_signed_cert.der
 */
static const unsigned char self_signed_cert_der[] = {
  0x30, 0x82, 0x01, 0xe0, 0x30, 0x82, 0x01, 0x85, 0xa0, 0x03, 0x02, 0x01,
  0x02, 0x02, 0x14, 0x3d, 0xaf, 0x9b, 0xc6, 0xb0, 0xdb, 0x76, 0x77, 0xa8,
  0x21, 0x32, 0x90, 0xfb, 0x6d, 0xbb, 0xf3, 0xed, 0x0b, 0x19, 0x90, 0x30,
  0x0a, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x04, 0x03, 0x02, 0x30,
  0x45, 0x31, 0x0b, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02,
  0x41, 0x55, 0x31, 0x13, 0x30, 0x11, 0x06, 0x03, 0x55, 0x04, 0x08, 0x0c,
  0x0a, 0x53, 0x6f, 0x6d, 0x65, 0x2d, 0x53, 0x74, 0x61, 0x74, 0x65, 0x31,
  0x21, 0x30, 0x1f, 0x06, 0x03, 0x55, 0x04, 0x0a, 0x0c, 0x18, 0x49, 0x6e,
  0x74, 0x65, 0x72, 0x6e, 0x65, 0x74, 0x20, 0x57, 0x69, 0x64, 0x67, 0x69,
  0x74, 0x73, 0x20, 0x50, 0x74, 0x79, 0x20, 0x4c, 0x74, 0x64, 0x30, 0x1e,
  0x17, 0x0d, 0x32, 0x35, 0x30, 0x33, 0x32, 0x30, 0x31, 0x39, 0x33, 0x34,
  0x35, 0x39, 0x5a, 0x17, 0x0d, 0x32, 0x36, 0x30, 0x33, 0x32, 0x30, 0x31,
  0x39, 0x33, 0x34, 0x35, 0x39, 0x5a, 0x30, 0x45, 0x31, 0x0b, 0x30, 0x09,
  0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02, 0x41, 0x55, 0x31, 0x13, 0x30,
  0x11, 0x06, 0x03, 0x55, 0x04, 0x08, 0x0c, 0x0a, 0x53, 0x6f, 0x6d, 0x65,
  0x2d, 0x53, 0x74, 0x61, 0x74, 0x65, 0x31, 0x21, 0x30, 0x1f, 0x06, 0x03,
  0x55, 0x04, 0x0a, 0x0c, 0x18, 0x49, 0x6e, 0x74, 0x65, 0x72, 0x6e, 0x65,
  0x74, 0x20, 0x57, 0x69, 0x64, 0x67, 0x69, 0x74, 0x73, 0x20, 0x50, 0x74,
  0x79, 0x20, 0x4c, 0x74, 0x64, 0x30, 0x59, 0x30, 0x13, 0x06, 0x07, 0x2a,
  0x86, 0x48, 0xce, 0x3d, 0x02, 0x01, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce,
  0x3d, 0x03, 0x01, 0x07, 0x03, 0x42, 0x00, 0x04, 0x6d, 0xbd, 0xcd, 0x8d,
  0xee, 0x3a, 0x27, 0x9c, 0xb1, 0xe9, 0xea, 0x53, 0x46, 0x75, 0xe3, 0x4f,
  0xe7, 0xca, 0xba, 0xa4, 0x2e, 0x8a, 0x7a, 0xd4, 0x94, 0xbe, 0xb8, 0xde,
  0x70, 0x86, 0x73, 0xbd, 0x27, 0xa0, 0x3f, 0x5c, 0x8e, 0x40, 0x83, 0xa5,
  0xfc, 0xaf, 0x19, 0xed, 0xef, 0x64, 0x80, 0x98, 0x25, 0x86, 0x04, 0xc5,
  0xde, 0xd9, 0x67, 0x57, 0xdb, 0x6b, 0x94, 0x9a, 0xb0, 0x6e, 0xf6, 0xc8,
  0xa3, 0x53, 0x30, 0x51, 0x30, 0x1d, 0x06, 0x03, 0x55, 0x1d, 0x0e, 0x04,
  0x16, 0x04, 0x14, 0xe1, 0xa8, 0xe0, 0xfb, 0x47, 0xc1, 0xc5, 0xd0, 0xf1,
  0x8a, 0x11, 0x5b, 0xac, 0xab, 0x1a, 0x2d, 0x50, 0x5c, 0x92, 0xf5, 0x30,
  0x1f, 0x06, 0x03, 0x55, 0x1d, 0x23, 0x04, 0x18, 0x30, 0x16, 0x80, 0x14,
  0xe1, 0xa8, 0xe0, 0xfb, 0x47, 0xc1, 0xc5, 0xd0, 0xf1, 0x8a, 0x11, 0x5b,
  0xac, 0xab, 0x1a, 0x2d, 0x50, 0x5c, 0x92, 0xf5, 0x30, 0x0f, 0x06, 0x03,
  0x55, 0x1d, 0x13, 0x01, 0x01, 0xff, 0x04, 0x05, 0x30, 0x03, 0x01, 0x01,
  0xff, 0x30, 0x0a, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x04, 0x03,
  0x02, 0x03, 0x49, 0x00, 0x30, 0x46, 0x02, 0x21, 0x00, 0x82, 0x48, 0x30,
  0x13, 0x9f, 0x3a, 0x3c, 0x64, 0xf6, 0xca, 0xea, 0x7f, 0x81, 0x0a, 0x04,
  0x0e, 0x54, 0x46, 0xd3, 0x7c, 0x02, 0xc2, 0x05, 0xd8, 0xc6, 0x30, 0xba,
  0x05, 0x16, 0x11, 0xf5, 0x89, 0x02, 0x21, 0x00, 0xdc, 0x8a, 0xcd, 0x41,
  0x4b, 0x46, 0xe1, 0x31, 0xca, 0x4f, 0x15, 0x5d, 0x48, 0xb0, 0xd4, 0xb1,
  0x3f, 0x87, 0xc6, 0x94, 0x6b, 0xa6, 0x7d, 0x4e, 0x63, 0x0e, 0x86, 0x37,
  0xb4, 0x58, 0xd8, 0x02
};



/* Private key in DER format.
 * Generated along with the certificate using:
 * openssl req -x509 -nodes -days 365 -new -newkey ec -pkeyopt ec_paramgen_curve:prime256v1 -keyout private_key.key -out self_signed_cert.crt
 * Converted to binary using:
 * openssl pkey -outform der -in private_key.key -out private_key.der
 * xxd -i ecdsa_private_key.der
 */
static const unsigned char private_key_der[] = {
  0x30, 0x77, 0x02, 0x01, 0x01, 0x04, 0x20, 0xcc, 0xd1, 0xee, 0x75, 0xae,
  0x0b, 0xe6, 0xb7, 0xad, 0xef, 0xa3, 0x3a, 0x3f, 0x02, 0xa9, 0x0d, 0x98,
  0x5f, 0xb6, 0xd5, 0xb3, 0x12, 0x55, 0xe9, 0xe1, 0x05, 0x56, 0x48, 0xf3,
  0x1c, 0x37, 0xd5, 0xa0, 0x0a, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d,
  0x03, 0x01, 0x07, 0xa1, 0x44, 0x03, 0x42, 0x00, 0x04, 0x6d, 0xbd, 0xcd,
  0x8d, 0xee, 0x3a, 0x27, 0x9c, 0xb1, 0xe9, 0xea, 0x53, 0x46, 0x75, 0xe3,
  0x4f, 0xe7, 0xca, 0xba, 0xa4, 0x2e, 0x8a, 0x7a, 0xd4, 0x94, 0xbe, 0xb8,
  0xde, 0x70, 0x86, 0x73, 0xbd, 0x27, 0xa0, 0x3f, 0x5c, 0x8e, 0x40, 0x83,
  0xa5, 0xfc, 0xaf, 0x19, 0xed, 0xef, 0x64, 0x80, 0x98, 0x25, 0x86, 0x04,
  0xc5, 0xde, 0xd9, 0x67, 0x57, 0xdb, 0x6b, 0x94, 0x9a, 0xb0, 0x6e, 0xf6,
  0xc8
};



const UCHAR html_data[] = { "<html>\r\n"\
                            "<head>\r\n"\
                            "<title>NetxDuo HTTPS web server</title>\r\n"\
                            "</head>\r\n"\
                            "<body>\r\n"\
                            "<b>Hello NetX Duo Secure User!</b>\r\n"\
                            "This is a simple webpage\r\n"\
                            "served up using NetX Duo Secure!\r\n"\
                            "</body>\r\n"\
                            "</html>\r\n" };



static UINT server_request_callback(NX_WEB_HTTP_SERVER *server_ptr, UINT request_type, CHAR *resource, NX_PACKET *packet_ptr);



/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int netxduo_cpsw_main(ULONG arg)
{
    Enet_Type enetType;
    uint32_t instId;
    Enet_Handle hEnet;
    ULONG ipAddr;
    ULONG netMask;
    Enet_MacPort macPort;
    uint32_t rxChCnt;
    uint32_t txChCnt;
    const uint32_t *rxChIds;
    const uint32_t *txChIds;
    nx_enet_drv_rx_ch_hndl_t rxChs[ENET_SYSCFG_RX_FLOWS_NUM];
    nx_enet_drv_tx_ch_hndl_t txChs[ENET_SYSCFG_TX_CHANNELS_NUM];
    ULONG actual_status;
    EnetApp_GetMacAddrOutArgs outArgs;
    bool isLinked;
    UINT status;
    int32_t res;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("===============================\r\n");
    DebugP_log("   CPSW NETXDUO HTTPS SERVER   \r\n");
    DebugP_log("===============================\r\n");


    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &enetType, &instId);

    EnetAppUtils_enableClocks(enetType, instId);

    EnetApp_driverInit();

    res = EnetApp_driverOpen(enetType, instId);
    DebugP_assert(res == ENET_SOK);


    EnetApp_addMCastEntry(enetType, instId, EnetSoc_getCoreId(), BROADCAST_MAC_ADDRESS, CPSW_ALE_ALL_PORTS_MASK);


    /* Initialize the NetX system.  */
    nx_system_initialize();


    /* Create a packet pool.  */
    status = nx_packet_pool_create(&gUserPacketPool, "User TX packet pool", PACKET_SIZE, &gUserPoolMem[0], USER_POOL_SIZE);
    EnetAppUtils_assert(status == NX_SUCCESS);

    status = nx_packet_pool_create(&gInternalPacketPool, "NetX internal packet pool", PACKET_SIZE, &gInternalPoolMem[0], INTERNAL_POOL_SIZE);
    EnetAppUtils_assert(status == NX_SUCCESS);


    /* Allocate NetX Rx channel and corresponding buffers. */
    NetxEnetApp_getAllRxChIDs(&rxChIds, &rxChCnt);
    for(size_t k = 0u; k < rxChCnt; k++) {

        EnetApp_GetDmaHandleInArgs inArgs = {0};
        EnetApp_GetRxDmaHandleOutArgs outArgs;

        EnetApp_getRxDmaHandle(rxChIds[k], &inArgs, &outArgs);

        EnetAppUtils_assert(outArgs.hRxCh != NULL);
        NetxEnetDriver_allocRxCh(outArgs.hRxCh, outArgs.maxNumRxPkts, &gInternalPacketPool, &rxChs[k]);
    }

    /* Allocate NetX Tx channel and corresponding buffers. */
    NetxEnetApp_getAllTxChIDs(&txChIds, &txChCnt);
    for (size_t k = 0u; k < txChCnt; k++) {

        EnetApp_GetDmaHandleInArgs inArgs = {0};
        EnetApp_GetTxDmaHandleOutArgs outArgs;

        EnetApp_getTxDmaHandle(txChIds[k], &inArgs, &outArgs);

        EnetAppUtils_assert(outArgs.hTxCh != NULL);
        NetxEnetDriver_allocTxCh(outArgs.hTxCh, outArgs.maxNumTxPkts, &txChs[k]);
    }

    /* Allocate NetX interface and bind to DMA channels. */
    macPort = NetxEnetApp_getMacPort(0, 0);
    EnetApp_getMacAddress(rxChIds[0], &outArgs);
    NetxEnetDriver_allocIf("PRI", macPort, &outArgs.macAddr[0][0], &rxChs[0], rxChCnt, txChs, txChCnt);

    /* Wait for the link on default interface to come up. */
    hEnet = Enet_getHandle(enetType, instId);

    isLinked = false;
    while (!isLinked) {

        EnetAppUtils_print("Waiting for link up...\n");
        isLinked = EnetApp_isPortLinked(hEnet);

        tx_thread_sleep(2u * TX_TIMER_TICKS_PER_SECOND);
    }


    /* Open NetX SA2UL crypto acceleration. */
    nx_crypto_sa2ul_asym_open();
    nx_crypto_sa2ul_open();


    /* Create an IP instance.  */
    status = nx_ip_create(&gIp, "NetX IP Instance 0", IP_ADDRESS(0, 0, 0, 0), 0xFFFFFF00UL, &gInternalPacketPool, _nx_enet_driver, (void *)&gIpThreadStack[0], IP_THREAD_STACK_SIZE, 1);
    EnetAppUtils_assert(status == NX_SUCCESS);

    /* Enable ARP */
    status = nx_arp_enable(&gIp, (void *)&gIpArpThreadStack[0], IP_ARP_THREAD_STACK_SIZE);
    EnetAppUtils_assert(status == NX_SUCCESS);

    /* Enable ICMP */
    status = nxd_icmp_enable(&gIp);
    EnetAppUtils_assert(status == NX_SUCCESS);


    /* Enable UDP */
    status = nx_udp_enable(&gIp);
    EnetAppUtils_assert(status == NX_SUCCESS);


    /* Enable TCP */
    status = nx_tcp_enable(&gIp);
    EnetAppUtils_assert(status == NX_SUCCESS);


    /* Create the DHCP instance.  */
    status = nx_dhcp_create(&gDhcpClient, &gIp, "DHCP-CLIENT");
    EnetAppUtils_assert(status == NX_SUCCESS);


    nx_dhcp_interface_enable(&gDhcpClient, 0u);


    /* Start the DHCP Client.  */
    status = nx_dhcp_interface_start(&gDhcpClient, 0u);
    EnetAppUtils_assert(status == NX_SUCCESS);


    /* Wait for DHCP to assign the IP address.  */
    EnetAppUtils_print("Waiting for address from DHCP server on primary interface...\n");
    do {

        /* Check for address resolution.  */
        status = nx_ip_interface_status_check(&gIp, 0u, NX_IP_ADDRESS_RESOLVED, (ULONG *) &actual_status, NX_IP_PERIODIC_RATE);

        tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);

    } while ((actual_status & NX_IP_ADDRESS_RESOLVED) != NX_IP_ADDRESS_RESOLVED);


    status = nx_ip_interface_address_get(&gIp, 0u, &ipAddr, &netMask);
    EnetAppUtils_assert(status == NX_SUCCESS);

    DebugP_log("Local Interface IP is: %lu.%lu.%lu.%lu\n", ((ipAddr >> 24u) & 0xFF), ((ipAddr >> 16u) & 0xFF), ((ipAddr >> 8u) & 0xFF), (ipAddr & 0xFF));


    /* Create a socket.  */
    status =  nx_tcp_socket_create(&gIp, &gServerSocket, "Server Socket", NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, 8000u, NX_NULL, NX_NULL);
    EnetAppUtils_assert(status == NX_SUCCESS);

    DebugP_log("Socket created\r\n");


    nx_secure_tls_initialize();


    /* Give NetX a chance to initialize the system. */
    tx_thread_sleep(NX_IP_PERIODIC_RATE);


    /* Create the HTTPS Server. */
    status = nx_web_http_server_create(&gHttpServer, "My HTTP Server", &gIp, NX_WEB_HTTPS_SERVER_PORT, NULL, &gHttpServerStack, sizeof(gHttpServerStack), &gUserPacketPool, NULL, server_request_callback);
    DebugP_assert(status == NX_SUCCESS);


    /* Initialize an X.509 certificate and private ECC key for our TLS Session. */
    status = nx_secure_x509_certificate_initialize(&certificate, (UCHAR *)&self_signed_cert_der[0], sizeof(self_signed_cert_der), NX_NULL, 0, &private_key_der[0], sizeof(private_key_der), NX_SECURE_X509_KEY_TYPE_EC_DER);
    EnetAppUtils_assert(status == NX_SUCCESS);

    /* Setup TLS session data for the TCP server. */
    status = nx_web_http_server_secure_configure(&gHttpServer, &nx_crypto_tls_ciphers_ecc, tls_crypto_metadata, sizeof(tls_crypto_metadata), tls_packet_buffer, sizeof(tls_packet_buffer), &certificate, NX_NULL, 0, NX_NULL, 0, NX_NULL, 0);
    EnetAppUtils_assert(status == NX_SUCCESS);

    status = nx_web_http_server_secure_ecc_configure(&gHttpServer, nx_crypto_ecc_supported_groups, nx_crypto_ecc_supported_groups_size, nx_crypto_ecc_curves);
    EnetAppUtils_assert(status == NX_SUCCESS);


    /* Start an HTTPS Server with TLS.  */
    status = nx_web_http_server_start(&gHttpServer);
    EnetAppUtils_assert(status == NX_SUCCESS);


    /* HTTP server ready to take requests! */

    /* Let the IP thread execute.    */
    tx_thread_sleep(NX_IP_PERIODIC_RATE);

    /* Main application loop. Requests are handled in the request callback. */
    while (1) {
        tx_thread_sleep(NX_IP_PERIODIC_RATE);
    }

    return (0);
}



/* Server request callback is invoked whenever an HTTP(S) client sends a request to the server port. */
static UINT server_request_callback(NX_WEB_HTTP_SERVER *server_ptr, UINT request_type, CHAR *resource, NX_PACKET *packet_ptr)
{
    NX_PACKET *response_pkt;
    UINT status;

    NX_PARAMETER_NOT_USED(resource);

    /* Process GET request. */
    if (request_type == NX_WEB_HTTP_SERVER_GET_REQUEST) {

        /* Generate HTTP header.  */
        status = nx_web_http_server_callback_generate_response_header(server_ptr, &response_pkt, NX_WEB_HTTP_STATUS_OK, sizeof(html_data) - 1, "html", NX_NULL);
        if (status) return (status);

        status = nx_packet_data_append(response_pkt, (void *)html_data, sizeof(html_data) - 1, server_ptr -> nx_web_http_server_packet_pool_ptr, NX_WAIT_FOREVER);
        if (status) return (status);

        status = nx_web_http_server_callback_packet_send(server_ptr, response_pkt);
        if (status) {
            nx_packet_release(response_pkt);
            return (status);
        }

    } else {

        /* Indicate we have not processed the response to client yet.*/
        return (NX_SUCCESS);
    }

    /* Indicate the response to client is transmitted. */
    return (NX_WEB_HTTP_CALLBACK_COMPLETED);

}
