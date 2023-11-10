/*
 * Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * \file  lwip2lwipif.h
 *
 * \brief Header file for LwIP interface.
 */

#ifndef LWIP_LWIPIF_H_
#define LWIP_LWIPIF_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <lwip/err.h>
#include <enet_types.h>

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/* Forward declaration */
struct netif;

err_t LWIPIF_LWIP_init(struct netif *netif);

/*!
 *  This API is used to initialize and start the Enet and DMA
 *  peripherals for data path and associates with netif
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param netif     LwIP netif that needs to be associated with peripheral
 * \param macPort   MAC port that needs to be associated with the netif
 *
 * \return Return 0 if successful. else negative values.
 */
int32_t LWIPIF_LWIP_start(Enet_Type enetType, uint32_t instId, struct netif *netif, uint32_t netifIdx);

/*!
 *  This API shall set the notification callbacks when the packets are received (along Rx direction)
 *  or when packets has completed transmission (along TX direction)
 *  peripherals for data path and associates with netif
 *
 * \param netif      LwIP netif
 * \param pRxNotify  Rx side notification cabllback details
 * \param pTxNotify  Tx side notification cabllback details
 *
 */
void LWIPIF_LWIP_setNotifyCallbacks(struct netif *netif, Enet_notify_t *pRxNotify, Enet_notify_t *pTxNotify);

/*!
 * Periodic polling API. Suggestion to call the regular interval as per the required packet latency.
 *
 * \param netif      LwIP netif
 *
 */
void LWIPIF_LWIP_periodic_polling(struct netif *netif);

/*!
 * RX packet handler API, that retrieves the received packets/descriptors from driver.
 *
 * \param netif      LwIP netif
 *
 */
void LWIPIF_LWIP_rxPktHandler(struct netif *netif);

/*!
 * TX packet handler API, that retrieves the freed Tx packets/descriptors from driver.
 *
 * \param netif      LwIP netif
 *
 */
void LWIPIF_LWIP_txPktHandler(struct netif *netif);

/*!
 *  @b LWIPIF_LWIP_send
 *  @n
 *  This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 *  \param[in]  netif
 *      The lwip network interface structure for this ethernetif
 *  \param[in]  p
 *      the MAC packet to send (e.g. IP packet including MAC addresses and type)
 *
 *  \retval
 *      ERR_OK if the packet could be sent
 *  \retval
 *      an err_t value if the packet couldn't be sent
 */
 err_t LWIPIF_LWIP_send(struct netif *netif, struct pbuf *p);

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

#endif /* LWIP_LWIPIF_H_ */
