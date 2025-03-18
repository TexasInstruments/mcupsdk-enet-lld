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
 * \file  lwip2enet_ic.h
 *
 * \brief Header file for the LwIP to Enet helper functions.
 */

#ifndef LWIP2ENET_IC_H_
#define LWIP2ENET_IC_H_

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
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/DebugP.h>


/* Project dependency headers */
#include <enet.h>
#include <enet_cfg.h>
#include <intercore/intercore.h>
#include <bufpool.h>

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! Inter-core virtual driver object instance ID: MCU2_0->MCU2_1
 *  This instance is created on MCU2_0 to interface with MCU2_1 */
#define IC_ETH_IF_MCU2_0_MCU2_1     0
/*! Inter-core virtual driver object instance ID: MCU2_0->MCU2_1
 *  This instance is created on MCU2_1 to interface with MCU2_0 */
#define IC_ETH_IF_MCU2_1_MCU2_0     1
/*! Inter-core virtual driver object instance ID: MCU2_0->A72
 *  This instance is created on MCU2_0 to interface with A72
 *
 *  Note: The A72 side peer peer virtual driver does not use Lwip so we
 *  do not need an A72_to_MCU2_0 driver instance.
 */
#define IC_ETH_IF_MCU2_0_A72        2
/*! Inter-core virtual driver object instance ID: MCU2_0->MCU3_0
 *  This instance is created on MCU2_0 to interface with MCU3_0 */
#define IC_ETH_IF_MCU2_0_MCU3_0     3
/*! Inter-core virtual driver object instance ID: MCU2_0->MCU3_0
 *  This instance is created on MCU3_0 to interface with MCU2_0 */
#define IC_ETH_IF_MCU3_0_MCU2_0     4
/*! Total no. of inter-core virtual interfaces */
#define IC_ETH_MAX_VIRTUAL_IF       5

/*! LWIP2ENETIC Error Code: Success */
#define LWIP2ENETIC_OK              (0)
/*! LWIP2ENETIC Error Code: Failure */
#define LWIP2ENETIC_ERROR           (-1)

/*! Enables polling mode in the packet RX task */
#define IC_ETH_RX_POLLING_MODE      1
/*! Pooling interval used by the packet RX task */
#define IC_ETH_RX_POLLING_INTERVAL  1000 /* micro-seconds */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief lwIP interface layer's RX statistics.
 */
typedef struct Lwip2EnetIc_RxStats_s
{
    uint32_t largePktErr;
    uint32_t pbufNullErr;
    uint32_t pbufTakeErr;
    uint32_t netifInputErr;
    uint32_t rxQEmptyWarn;
    uint32_t pktDeq;
    uint32_t pktSentToStk;
} Lwip2EnetIc_RxStats;

/*!
 * \brief lwIP interface layer's TX statistics.
 */
typedef struct Lwip2EnetIc_TxStats_s
{
    uint32_t pktDropInitErr;
    uint32_t pbufNullErr;
    uint32_t txQFullErr;
    uint32_t pktEnq;
} Lwip2EnetIc_TxStats;

typedef struct Lwip2EnetIc_Stats_s
{
    Lwip2EnetIc_TxStats txStats;
    Lwip2EnetIc_RxStats rxStats;
    uint32_t driverBufGet;
    uint32_t driverBufFree;
    uint32_t driverBufNullErr;
} Lwip2EnetIc_Stats;

/**
 * \brief
 *  Inter-core virtual device object
 *
 * \details
 *  This structure stores the information for an inter-core virtual
 *  device instance.
 */
typedef struct Lwip2EnetIc_OBJECT_
{
    /* Intercore trasport global object */
    Ic_Object icObj;

    /*! Instance ID */
    uint32_t instId;

    /* Mac Address assigned to this interface */
    uint8_t macAddr[ENET_MAC_ADDR_LEN];

    /* Initialization flag.*/
    volatile uint32_t initDone;

    /* Link is up flag. */
    uint32_t linkIsUp;

    /* No of PBM packets allocated by application - used for debug purpose.*/
    uint32_t numAllocPbufPkts;

    /* Handle to IPC receive task */
    TaskP_Object ipcRxTaskObj;

    /*
     * Handle to Rx task: Receives packets from the intercore transport
     * and give them to the LWIP stack.
     */
    TaskP_Object rxPacketTaskObj;

    /*
     * Handle to Rx semaphore: Posted by the IPC RX task on getting a
     * packet notification from the remote core.
     */
#if !(IC_ETH_RX_POLLING_MODE)
    SemaphoreP_Handle hRxPacketSem;
#endif

    /* Handle to input task that sends pbufs to the LWIP stack */
    TaskP_Object LWIPIF2LWIPinputObj;

    /*
     * Handle to counting shutdown semaphore, which all subtasks created in the
     * open function must post before the close operation can complete.
     */
    SemaphoreP_Object shutDownSemObj;

    /* Boolean to indicate shutDownFlag status of translation layer.*/
    volatile bool shutDownFlag;

    /* Print buffer */
    char printBuf[ENET_CFG_PRINT_BUF_LEN];

    /* Print Function */
    Enet_Print print;

    /* Driver buffer pool for this core */
    BufPool_Handle hBufPool;

    /* Stats */
    Lwip2EnetIc_Stats stats;

} Lwip2EnetIc_Object, *Lwip2EnetIc_Handle;

typedef struct Lwip2EnetIc_Params_
{
    /*! Instance ID */
    uint32_t instId;

    /*! Owner core ID: This core initializes the shared mem transport */
    uint32_t ownerId;

    /*! TX-Q ID */
    uint32_t txQId;

    /*! RX-Q ID (=peer core's TX-Q ID) */
    uint32_t rxQId;

    /*! Buffer Pool ID */
    uint32_t bufPoolId;

    /*! Requested IPC end point ID */
    uint32_t reqEndPtId;

    /*! Remote core ID */
    uint32_t remoteCoreId;

    /*! IPC end point name */
    char endPtName[ICETH_ENDPT_NAME_LEN_MAX];

    /*! IPC remote end point name */
    char remoteEndPtName[ICETH_ENDPT_NAME_LEN_MAX];

    /*! MAC address */
    uint8_t macAddr[ENET_MAC_ADDR_LEN];

} Lwip2EnetIc_Params;


/* ========================================================================== */
/*                         External Variable Declarations                     */
/* ========================================================================== */

/* ========================================================================== */
/*                          API/Public Function Declarations                  */
/* ========================================================================== */

/*
 * Functions Provided by our translation layer code
 */
extern Lwip2EnetIc_Handle Lwip2EnetIc_open(uint32_t instId);

extern void Lwip2EnetIc_close(Lwip2EnetIc_Handle hlwip2enetIc);

extern int32_t Lwip2EnetIc_remoteCorePktNotify(Lwip2EnetIc_Handle hLwip2EnetIc);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline void Lwip2EnetIcStats_addOne(uint32_t *statCnt)
{
#if defined(LWIPIFIC_INSTRUMENTATION_ENABLED)
    *statCnt += 1U;
#endif
}

static inline void Lwip2EnetIcStats_addNum(uint32_t *statCnt,
                                        uint32_t addCnt)
{
#if defined(LWIPIFIC_INSTRUMENTATION_ENABLED)
    *statCnt += addCnt;
#endif
}

static inline void Lwip2EnetIc_assert(bool cond)
{
    assert(cond);
}

#ifdef __cplusplus
}
#endif

#endif /* LWIP2ENET_IC_H_ */
