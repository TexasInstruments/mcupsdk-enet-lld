/*
 *  Copyright (c) Texas Instruments Incorporated 2021
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
 * \file  lwip2lwipif_ic.h
 *
 * \brief Header file for LwIP interface.
 */

/*!
 * \ingroup  ENET_LWIP_INTERCORE_NETIF
 * \defgroup ENET_LWIP_INTERCORE_NETIF_HELPER_API Enet intercore netif helper APIs
 *
 * @{
 */

#ifndef LWIP_LWIPIF_IC_H_
#define LWIP_LWIPIF_IC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <lwip/err.h>

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Global Variable Declarations                         */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/*!
 * \brief Inter-core virtual network driver's netif init callback function.
 *
 * This function implements the initialization of the inter-core virtual network
 * interface, so it must be called when adding a netif interface via `netif_add()`.
 *
 * Virtual netifs should be added via `netif_add()` and this function must be
 * passed as the init callback function.
 *
 * \param netif   The netif to initialize.
 *
 * \return ERR_OK if initialization was successful. A corresponding lwIP
 *         error code otherwise.
 */
err_t LWIPIF_LWIP_IC_init(struct netif *netif);

/* ========================================================================== */
/*                     Deprecated Function Declarations                       */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                        Static Function Definitions                         */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* LWIP_LWIPIF_IC_H_ */

/*! @} */
