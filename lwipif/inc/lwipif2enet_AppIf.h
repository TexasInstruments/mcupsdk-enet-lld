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
#include <lwip2lwipif.h>
#include <pbufQ.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/* Maximum number of MAC ports available on SOC and
 * by supported by LwIP-If layer.
 */
#define LWIPIF_MAX_NUM_MAC_PORTS             (CPSW_STATS_MACPORT_MAX)

/*Max number of netif enabled. */
#define LWIPIF_MAX_NETIFS_SUPPORTED          (LWIPIF_MAX_NUM_MAC_PORTS * LWIPIF_MAX_NUM_PERIPHERALS)

/*Max number of RX channels supported by lwipif*/
#define LWIPIF_MAX_RX_CHANNELS               (LWIPIF_MAX_RX_CHANNELS_PER_PHERIPHERAL * LWIPIF_MAX_NUM_PERIPHERALS)

/*Max number of TX channels supported by lwipif */
#define LWIPIF_MAX_TX_CHANNELS               (LWIPIF_MAX_TX_CHANNELS_PER_PHERIPHERAL * LWIPIF_MAX_NETIFS_SUPPORTED)


/* Maximum number of ENET Peripheral instances supported
 * by LwIP-IF layer.In the current version, we can two ICSSG enet
 * (ICSSG Dual mac/ dual netif) instances, which is considered as maximum.
 * For CPSW case, it is one CPSW instance able to setup dual mac.
 * have ports available on SOC.
 */
#define LWIPIF_MAX_NUM_PERIPHERALS               (CPSW_STATS_MACPORT_MAX)

/* Maximum number of RX DMA channels that can be associated to LwIP per peripheral.
 * Lwip2enet supports only one channel association */
#define LWIPIF_MAX_RX_CHANNELS_PER_PHERIPHERAL   (CPSW_STATS_MACPORT_MAX)

/* Maximum number of TX DMA channel that can be associated to LwIP per peripheral.

 * Lwip2enet supports only one channel association */
#define LWIPIF_MAX_TX_CHANNELS_PER_PHERIPHERAL   (1U)

/* Maximum number of MAC ports supported per peripheral */
#define LWIPIF_MAX_NUM_MAC_PORTS_PER_PHERIPHERAL (CPSW_STATS_MACPORT_MAX)

/* Maximum number of NEtIFs supported per peripheral */
#define LWIPIF_MAX_NETIFS_PER_PHERIPHERAL        (LWIPIF_MAX_NUM_MAC_PORTS_PER_PHERIPHERAL)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

typedef void (*LwipifEnetAppIf_TxFreePktCbFxn)(void *cbArg,
                                           EnetDma_PktQ *fqPktInfoQ,
                                           EnetDma_PktQ *cqPktInfoQ);

typedef void (*LwipifEnetAppIf_RxFreePktCbFxn)(void *cbArg,
                                           EnetDma_PktQ *fqPktInfoQ,
                                           EnetDma_PktQ *cqPktInfoQ);


typedef bool (*LwipifEnetAppIf_IsPhyLinkedCbFxn)(Enet_Handle hEnet);

typedef struct LwipifEnetAppIf_GetTxHandleInArgs_s
{
    void *cbArg;
    Enet_Type enetType;
    uint32_t  instId;
    EnetDma_PktNotifyCb notifyCb;
    uint32_t chId;
    EnetDma_PktQ *pktInfoQ;
} LwipifEnetAppIf_GetTxHandleInArgs;

typedef struct LwipifEnetAppIf_GetRxHandleInArgs_s
{
    void *cbArg;
    Enet_Type enetType;
    uint32_t  instId;
    EnetDma_PktNotifyCb notifyCb;
    uint32_t chId;
    pbufQ *pFreePbufInfoQ;
    EnetDma_PktQ *pReadyRxPktQ;
    EnetDma_PktQ *pFreeRxPktInfoQ;
} LwipifEnetAppIf_GetRxHandleInArgs;


typedef struct LwipifEnetAppIf_TxHandleInfo_s
{
    /** DMA transmit channel */
    EnetDma_TxChHandle hTxChannel;

    /*! Tx Channel Peer Id */
    uint32_t txChNum;

    /*! Whether to use TX event or not. When disabled, it uses "lazy" recycle mechanism
     *  to defer packet desc retrieval */
    bool disableEvent;

    /** Number of packets*/
    uint32_t numPackets;
} LwipifEnetAppIf_TxHandleInfo;


typedef struct LwipifEnetAppIf_RxHandleInfo_s
{
    /** ENET DMA receive channel */
    EnetDma_RxChHandle hRxFlow;
    /** Flow index for flow used  */
    uint32_t rxFlowStartIdx;
    /** Flow index for flow used  */
    uint32_t rxFlowIdx;
    /** Number of packets*/
    uint32_t numPackets;
    /*! Whether to use RX event or not. When disabled, it uses pacing timer to
     * retrieve packets periodically from driver */
    bool disableEvent;
        /** Mac Address allocated for the flow */
    uint8_t macAddr[LWIPIF_MAX_NETIFS_SUPPORTED][ENET_MAC_ADDR_LEN];
} LwipifEnetAppIf_RxHandleInfo;


typedef struct LwipifEnetAppIf_GetEnetLwipIfInstInfo_s
{
    uint32_t txMtu[ENET_PRI_NUM];
    uint32_t hostPortRxMtu;
	LwipifEnetAppIf_IsPhyLinkedCbFxn isPortLinkedFxn;

    /** Timer interval for timer based RX pacing */
    uint32_t timerPeriodUs;
    pbufNode *pPbufInfo;
    uint32_t pPbufInfoSize;
} LwipifEnetAppIf_GetEnetLwipIfInstInfo;

typedef struct LwipifEnetAppIf_ReleaseTxHandleInfo_s
{
    uint32_t txChNum;
    Enet_Type enetType;
    uint32_t  instId;
    LwipifEnetAppIf_TxFreePktCbFxn txFreePktCb;
	void *txFreePktCbArg;
} LwipifEnetAppIf_ReleaseTxHandleInfo;

typedef struct LwipifEnetAppIf_ReleasRxHandleInfo_s
{
    uint32_t rxChNum;
    Enet_Type enetType;
    uint32_t  instId;
    LwipifEnetAppIf_RxFreePktCbFxn rxFreePktCb;
    void *rxFreePktCbArg;
} LwipifEnetAppIf_ReleaseRxHandleInfo;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

extern void LwipifEnetAppCb_getEnetLwipIfInstInfo(Enet_Type enetType, uint32_t instId, LwipifEnetAppIf_GetEnetLwipIfInstInfo *outArgs);

extern void LwipifEnetAppCb_getTxHandleInfo(LwipifEnetAppIf_GetTxHandleInArgs *inArgs,
                                            LwipifEnetAppIf_TxHandleInfo *outArgs);

extern void LwipifEnetAppCb_getRxHandleInfo(LwipifEnetAppIf_GetRxHandleInArgs *inArgs,
                                            LwipifEnetAppIf_RxHandleInfo *outArgs);

extern void LwipifEnetAppCb_releaseTxHandle(LwipifEnetAppIf_ReleaseTxHandleInfo *releaseInfo);

extern void LwipifEnetAppCb_releaseRxHandle(LwipifEnetAppIf_ReleaseRxHandleInfo *releaseInfo);

extern void LwipifEnetAppCb_pbuf_free_custom(struct pbuf *p);
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
