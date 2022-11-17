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
 * \file  lwip2enet_appif.h
 *
 * \brief Header file for App interfaces to the LwIP Enet interface.
 */

#ifndef LWIPIF2ENET_APPIF_H_
#define LWIPIF2ENET_APPIF_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <assert.h>
#include <enet.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

#if (ENET_ENABLE_PER_ICSSG == 1)
#define LWIPIF_MAX_RX_CHANNELS                 (2U)
#else
#define LWIPIF_MAX_RX_CHANNELS                 (1U)
#endif

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

typedef void (*LwipifEnetAppIf_TxFreePktCbFxn)(void *cbArg,
                                           EnetDma_PktQ *fqPktInfoQ,
                                           EnetDma_PktQ *cqPktInfoQ);

typedef void (*LwipifEnetAppIf_RxFreePktCbFxn)(void *cbArg,
                                           EnetDma_PktQ *fqPktInfoQ,
                                           EnetDma_PktQ *cqPktInfoQ,
                                           uint32_t rxChNum);

typedef bool (*LwipifEnetAppIf_IsPhyLinkedCbFxn)(Enet_Handle hEnet);

typedef struct LwipifEnetAppIf_RxConfig_s
{
    void *cbArg;
    EnetDma_PktNotifyCb notifyCb;
    uint32_t numPackets;
} LwipifEnetAppIf_RxConfig;

typedef struct LwipifEnetAppIf_TxConfig_s
{
    void *cbArg;
    EnetDma_PktNotifyCb notifyCb;
    uint32_t numPackets;
} LwipifEnetAppIf_TxConfig;

typedef struct LwipifEnetAppIf_GetHandleInArgs_s
{
    LwipifEnetAppIf_TxConfig txCfg;
    LwipifEnetAppIf_RxConfig rxCfg;
} LwipifEnetAppIf_GetHandleInArgs;

typedef struct LwipifEnetAppIf_RxHandleInfo_s
{
    /** ENET DMA receive channel */
    EnetDma_RxChHandle hRxFlow;
    /** Flow index for flow used  */
    uint32_t rxFlowStartIdx;
    /** Flow index for flow used  */
    uint32_t rxFlowIdx;
} LwipifEnetAppIf_RxHandleInfo;

typedef struct LwipifEnetAppIf_RxInfo_s
{
    uint32_t numRxChannels;
    /** Mac Address allocated for the flow */
    uint8_t macAddr[ENET_CFG_NETIF_MAX][ENET_MAC_ADDR_LEN];

    /*! Whether to use RX event or not. When disabled, it uses pacing timer to
     * retrieve packets periodically from driver */
    bool disableEvent;

    LwipifEnetAppIf_RxHandleInfo rxHandle[LWIPIF_MAX_RX_CHANNELS];
} LwipifEnetAppIf_RxInfo;

typedef struct LwipifEnetAppIf_TxHandleInfo_s
{
    /** DMA transmit channel */
    EnetDma_TxChHandle hTxChannel;
    /** Tx Channel Peer Id */
    uint32_t txChNum;

    /*! Whether to use TX event or not. When disabled, it uses "lazy" recycle mechanism
     *  to defer packet desc retrieval */
    bool disableEvent;

    /*! Directed port number. Set to \ref ENET_MAC_PORT_INV for non-directed packets. */
    Enet_MacPort txPortNum;
} LwipifEnetAppIf_TxHandleInfo;

typedef struct LwipifEnetAppIf_GetHandleOutArgs_s
{
    Enet_Handle hEnet;
#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_UDMA)
    Udma_DrvHandle hUdmaDrv;
#endif
    uint32_t coreId;
    uint32_t coreKey;
    Enet_Print print;
    /** Max Tx packet size per priority */
    uint32_t txMtu[ENET_PRI_NUM];
    /** Max Rx packet size */
    uint32_t hostPortRxMtu;

	/*! Number of netifs allocated by application */
    uint32_t numNetif;
   LwipifEnetAppIf_IsPhyLinkedCbFxn isPortLinkedFxn;
   LwipifEnetAppIf_TxHandleInfo txInfo;
   LwipifEnetAppIf_RxInfo rxInfo;

    /** Timer interval for timer based RX pacing */
    uint32_t timerPeriodUs;
} LwipifEnetAppIf_GetHandleOutArgs;

typedef struct LwipifEnetAppIf_ReleaseHandleInfo_s
{
    Enet_Handle hEnet;
#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_UDMA)
    Udma_DrvHandle hUdmaDrv;
#endif
    uint32_t coreId;
    uint32_t coreKey;
    LwipifEnetAppIf_TxHandleInfo txInfo;
    LwipifEnetAppIf_RxInfo rxInfo;
    LwipifEnetAppIf_TxFreePktCbFxn txFreePktCb;
	void *txFreePktCbArg;
    LwipifEnetAppIf_RxFreePktCbFxn rxFreePktCb;
    void *rxFreePktCbArg;
} LwipifEnetAppIf_ReleaseHandleInfo;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

extern void LwipifEnetAppCb_getHandle(LwipifEnetAppIf_GetHandleInArgs *inArgs,
                                    LwipifEnetAppIf_GetHandleOutArgs *outArgs);

extern void LwipifEnetAppCb_releaseHandle(LwipifEnetAppIf_ReleaseHandleInfo *releaseInfo);

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

#endif /* LWIPIF2ENET_APPIF_H_ */
