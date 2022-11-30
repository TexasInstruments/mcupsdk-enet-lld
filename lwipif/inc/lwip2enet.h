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
 * \file  lwip2enet.h
 *
 * \brief Header file for the LwIP to Enet helper functions.
 */

#ifndef LWIP2ENET_H_
#define LWIP2ENET_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/* Standard language headers */
#include <stdint.h>
#include <assert.h>

/* OS/Posix headers */
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>

/* Project dependency headers */
#include "lwipif2enet_AppIf.h"
#include <enet.h>
#include <enet_cfg.h>
#include "pbufQ.h"

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
/*
 * Pre-Pad Packet Data Offset
 *
 *   The TCP/IP stack library requires that every packet device
 *   include enough L2 header room for all supported headers. In
 *   order to support PPPoE, this requires a 22 byte L2 header.
 *   Thus, since standard Ethernet is only 14 bytes, we must add
 *   on an additional 8 byte offset, or PPPoE can not function
 *   with our driver.
 */
// #define     PKT_PREPAD                      ((uint32_t)8U)
#define     PKT_PREPAD                      ((uint32_t)0)

/* Indicates whether RAM based multicast lists are suported for this
 * peripheral.
 */
#define     RAM_MCAST                       0U

/* Indicates whether HASH based multicasting is suported for this
 * peripheral.
 */
#define     HASH_MCAST                      0U

/* Multicast Address List Size */
#define     PKT_MAX_MCAST                   ((uint32_t)31U)

#define LWIP_RXFLOW_2_PORTIDX(num) (num - 1U)

/* Netif mapping to Rx port
 * First netif --> ENET_MAC_PORT_FIRST
 * Second netif--> ENET_MAC_PORT_2 and so on.
 */
#define LWIP_NETIF_IDX_2_RX_PORT(num) (num)

/* Netif mapping to Tx port
 * First netif --> ENET_MAC_PORT_FIRST
 * Second netif--> ENET_MAC_PORT_2 and so on.
 */
#define LWIP_NETIF_IDX_2_TX_PORT(num) (num)

#if (ENET_CFG_IS_OFF(CPSW_CSUM_OFFLOAD_SUPPORT))
#if (!(CHECKSUM_CHECK_UDP || CHECKSUM_CHECK_TCP || CHECKSUM_GEN_UDP || CHECKSUM_GEN_TCP))
#error "Hardware csum offload disabled and lwipopts disables sw csum also.Fix lwiptops.h"
#endif
#endif

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

enum LWIP2ENET_IOCTL_
{
    LWIP2ENET_IOCTL_BASE            = 0x00000000,
    /** lwip2enet get rxPacketTaskObj load function IOCTL value. IOCTL param is empty */
    /* IMP: Taking this value as it should not conflict with Enet IOCTL commands */
    LWIP2ENET_IOCTL_GET_RXTASK_LOAD = 0x0000ABAA,
    /** lwip2enet get txPacketTaskObj load function IOCTL value. IOCTL param is empty */
    /* IMP: Taking this value as it should not conflict with Enet IOCTL commands */
    LWIP2ENET_IOCTL_GET_TXTASK_LOAD = 0x0000ABAB
};

#define HISTORY_CNT ((uint32_t)2U)

typedef LwipifEnetAppIf_GetEnetLwipIfInstInfo Lwip2Enet_AppInfo;

typedef struct Lwip2Enet_PktTaskStats_s
{
    uint32_t rawNotifyCnt;
    uint32_t dataNotifyCnt;
    uint32_t zeroNotifyCnt;
    uint32_t totalPktCnt;
    uint32_t totalCycleCnt;

    uint32_t pktsPerNotifyMax;
    uint32_t pktsPerNotify[HISTORY_CNT];
    uint32_t cycleCntPerNotifyMax;
    uint32_t cycleCntPerNotify[HISTORY_CNT];
    uint32_t cycleCntPerPktMax;
    uint32_t cycleCntPerPkt[HISTORY_CNT];
    uint32_t taskLoad[HISTORY_CNT];
} Lwip2Enet_PktTaskStats;

/*!
 * \brief lwIP interface layer's RX statistics.
 */
typedef struct Lwip2Enet_RxStats_s
{
    Lwip2Enet_PktTaskStats pktStats;
    uint32_t freePbufPktEnq;
    uint32_t freePbufPktDeq;
    uint32_t freeAppPktEnq;
    uint32_t freeAppPktDeq;
    uint32_t chkSumErr;
    uint32_t stackNotifyCnt;
	uint32_t pbufAllocFailCnt;
	uint32_t rxLwipInputFail;
} Lwip2Enet_RxStats;

/*!
 * \brief lwIP interface layer's TX statistics.
 */
typedef struct Lwip2Enet_TxStats_s
{
    Lwip2Enet_PktTaskStats pktStats;
    uint32_t readyPbufPktEnq;
    uint32_t readyPbufPktDeq;
    uint32_t freeAppPktEnq;
    uint32_t freeAppPktDeq;
} Lwip2Enet_TxStats;

typedef struct Lwip2Enet_Stats_s
{
    uint32_t cpuLoad[HISTORY_CNT];
    uint32_t hwiLoad[HISTORY_CNT];
} Lwip2Enet_Stats;

struct Lwip2Enet_Obj_s;

/*!
 * \brief RX object which groups variables related to a particular RX channel/flow.
 */
typedef struct Lwip2Enet_RxObj_s
{
    /*! Pointer to parent Lwip2Enet object */
    struct Lwip2Enet_Obj_s *hLwip2Enet;

    /*! Enet DMA receive channel (flow) */
    EnetDma_RxChHandle hFlow;

    /*! Whether this RX object is being used or not */
    bool enabled;

    /*! Reference count for RX flow */
    uint32_t refCount;

    /*! Start index for RX flow */
    uint32_t flowStartIdx;

    /*! Flow index for RX flow */
    uint32_t flowIdx;

    /*! DMA Rx free packet info queue (holds packets returned from the hardware) */
    EnetDma_PktQ freePktInfoQ;

    /*! Number of packets*/
    uint32_t numPackets;

    /*! lwIP interface statistics */
    Lwip2Enet_RxStats stats;

    /*! Whether RX event should be disabled or not. When disabled, it relies on pacing timer
     *  to retrieve packets from RX channel/flow */
    bool disableEvent;
} Lwip2Enet_RxObj, *Lwip2Enet_RxHandle;

/*!
 * \brief TX object which groups variables related to a particular RX channel/flow.
 */
typedef struct Lwip2Enet_TxObj_s
{
    /*! Pointer to parent Lwip2Enet object */
    struct Lwip2Enet_Obj_s *hLwip2Enet;

    /*! Enet DMA transmit channel */
    EnetDma_TxChHandle hCh;

    /*! TX channel peer id */
    uint32_t chNum;

    /*! Whether this TX object is being used or not */
    bool enabled;

    /*! Reference count for TX object */
    uint32_t refCount;

    /*! Number of packets*/
    uint32_t numPackets;

    /*! DMA free queue (holds free hardware packets awaiting) */
    EnetDma_PktQ freePktInfoQ;

    /*! Queue that holds packets ready to be sent to the hardware */
    pbufQ readyPbufQ;

    /*! Queue that holds packets that were not sent to the hardware in previous submit */
    pbufQ unusedPbufQ;

    /*! lwIP interface statistics */
    Lwip2Enet_TxStats stats;

    Enet_notify_t txPktNotify;

    /*! Whether TX event should be disabled or not. When disabled, "lazy" descriptor recycle
     *  is used instead, which defers retrieval till none is available */
    bool disableEvent;
} Lwip2Enet_TxObj, *Lwip2Enet_TxHandle;

/**
 * \brief
 *  Packet device information
 *
 * \details
 *  This structure caches the device info.
 */
typedef struct Lwip2Enet_Obj_s
{
    /*! RX object */
    Lwip2Enet_RxObj rx[LWIPIF_MAX_RX_CHANNELS];

    /*! Number of RX channels allocated by Application */
    uint32_t numRxChannels;

	/*! TX object */
    Lwip2Enet_TxObj tx[LWIPIF_MAX_TX_CHANNELS];

    /*! Number of TX channels allocated by Application */
    uint32_t numTxChannels;

    /*! lwIP network interface */
    struct netif *netif[ENET_CFG_NETIF_MAX];

    uint8_t macAddr[ENET_CFG_NETIF_MAX][ENET_MAC_ADDR_LEN];
	/*! Total number of allocated PktInfo elements */
    uint32_t allocPktInfo;

    Lwip2Enet_AppInfo appInfo;
    /** Initialization flag.*/
    uint32_t initDone;
    /** Index of currently connect physical port.*/
    uint32_t currLinkedIf;

    /** Current RX filter */
    uint32_t rxFilter;
    /** Previous MCast Address Counter */
    uint32_t oldMCastCnt;
    /** Previous Multicast list configured by the Application.*/
    uint8_t bOldMCast[(uint32_t)ENET_MAC_ADDR_LEN * PKT_MAX_MCAST];
    /** Current MCast Address Counter */
    uint32_t MCastCnt;
    /** Multicast list configured by the Application.*/
    uint8_t bMCast[(uint32_t)ENET_MAC_ADDR_LEN * PKT_MAX_MCAST];
    /** Link is up flag. */
    uint32_t linkIsUp;
    /** Device is operating in test digital loopback mode.*/
    uint32_t inDLBMode;
    /** Total number of PBM packets allocated by application - used for debug purpose.*/
    uint32_t numAllocPbufPkts;

    /*
     * Clock handle for triggering the packet Rx notify
     */
    ClockP_Object pacingClkObj;

    /*
     * Handle to Binary Semaphore LWIP_LWIPIF_input when Rx packet queue is ready
     */
    SemaphoreP_Object pollLinkSemObj;

    /**< Print buffer */
    char printBuf[ENET_CFG_PRINT_BUF_LEN];

    /**< Print Function */
    Enet_Print print;

    /*! CPU load stats */
    Lwip2Enet_Stats stats;

    Lwip2Enet_RxHandle mapNetif2Rx[ENET_CFG_NETIF_MAX];

    Lwip2Enet_TxHandle mapNetif2Tx[ENET_CFG_NETIF_MAX];

    struct netif *mapRxPort2Netif[CPSW_STATS_MACPORT_MAX];

    Enet_MacPort mapNetif2TxPortNum[ENET_CFG_NETIF_MAX];


    Enet_notify_t rxPktNotify;

    Enet_notify_t txPktNotify;
}
Lwip2Enet_Obj, *Lwip2Enet_Handle;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/*
 * Functions Provided by our translation layer code
 */
extern Lwip2Enet_Handle Lwip2Enet_open(struct netif *netif);

extern void Lwip2Enet_close(Lwip2Enet_Handle hlwip2enet);

extern void Lwip2Enet_setRx(Lwip2Enet_Handle hlwip2enet);

extern void Lwip2Enet_sendTxPackets(Lwip2Enet_TxObj *tx,
                                    Enet_MacPort macPort);

extern int32_t Lwip2Enet_ioctl(Lwip2Enet_Handle hlwip2enet,
                              uint32_t cmd,
                              void *param,
                              uint32_t size);

extern void Lwip2Enet_poll(Lwip2Enet_Handle hlwip2enet,
                          uint32_t fTimerTick);

extern void Lwip2Enet_periodicFxn(struct netif *netif);

void Lwip2Enet_setRxNotifyCallback(Lwip2Enet_Handle hLwip2Enet, Enet_notify_t *pRxPktNotify);

void Lwip2Enet_setTxNotifyCallback(Lwip2Enet_Handle hLwip2Enet, Enet_notify_t *pTxPktNotify);

void Lwip2Enet_rxPktHandler(Lwip2Enet_Handle hLwip2Enet);

void Lwip2Enet_txPktHandler(Lwip2Enet_Handle hLwip2Enet);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline void Lwip2EnetStats_addOne(uint32_t *statCnt)
{
    *statCnt += 1U;
}

static inline void Lwip2EnetStats_addNum(uint32_t *statCnt,
                                         uint32_t addCnt)
{
    *statCnt += addCnt;
}

#if defined(LWIPIF_INSTRUMENTATION_ENABLED)
#define LWIP2ENETSTATS_ADDONE(statsCntPtr)           Lwip2EnetStats_addOne((statsCntPtr))
#define LWIP2ENETSTATS_ADDNUM(statsCntPtr, addCnt)   Lwip2EnetStats_addNum((statsCntPtr), (addCnt))
#else
#define LWIP2ENETSTATS_ADDONE(statsCntPtr)           do {} while (0)
#define LWIP2ENETSTATS_ADDNUM(statsCntPtr, addCnt)   do {} while (0)
#endif

static inline void Lwip2Enet_assert(bool cond)
{
    volatile static bool gCpswAssertWaitInLoop = TRUE;

    if (!(cond))
    {
        void EnetAppUtils_print(const char *pcString,...);
        EnetAppUtils_print("Assertion @ Line: %d in %s : failed !!!\r\n",
                            (int32_t) __LINE__, (const char *) __FILE__);
        while (gCpswAssertWaitInLoop)
        {
        }
    }
}

#ifdef __cplusplus
}
#endif

#endif /* LWIP2ENET_H_ */
