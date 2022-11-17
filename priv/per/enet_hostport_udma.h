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
 * \file  enet_hostport_udma.h
 *
 * \brief This file contains the CPSW specific DMA type definations & function
 *        prototypes for associated DMA peripheral.
 */

#ifndef ENET_HOSTPORT_UDMA_H_
#define ENET_HOSTPORT_UDMA_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <priv/core/enet_base_priv.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

#define ENET_UDMA_RSVDFLOW_RX_PKTS_NUM              (2U)

/*! \brief Total number of Rx channels for CPSW (always 1 for Jacinto 7 family of devices) */
#define ENET_UDMA_CPSW_RX_CH_NUM                    (1U)

#define ENET_UDMA_ICSSG_RX_CH_NUM                   (6U)

#define ENET_UDMA_RX_CH_NUM                         (ENET_UDMA_CPSW_RX_CH_NUM + \
                                                     ENET_UDMA_ICSSG_RX_CH_NUM)

/*! \brief Enet UDMA invalid RX flow id. */
#define ENET_UDMA_RXFLOWIDX_INVALID                          (0xABCDABCDU)

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

void EnetHostPortDma_initCfg(Enet_Type enetType, const void *dmaCfg);

EnetDma_Handle EnetHostPortDma_open(EnetPer_Handle hPer,
                                    const void *dmaCfg,
                                    const EnetRm_ResCfg *resCfg);

void EnetHostPortDma_close(EnetDma_Handle hDma);

void EnetHostPortDma_getDmaResInfo(EnetDma_Handle hDma,
                                   Enet_dmaResInfo *dmaResInfo,
                                   uint32_t chIdx);

EnetDma_RxChHandle EnetHostPortDma_openRsvdFlow(EnetDma_Handle hDma,
                                                const void *cfg,
                                                uint32_t startIdx,
                                                uint32_t flowIdx,
                                                uint32_t chIdx);

int32_t EnetHostPortDma_closeRsvdFlow(EnetDma_RxChHandle hRxRsvdFlow);

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

#endif /* ENET_HOSTPORT_UDMA_H_ */
