/*
 *  Copyright (c) Texas Instruments Incorporated 2024
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
 * \file  lwip_ic.h
 *
 * \brief Header file for the LwIP Interconnect helper functions.
 */

#ifndef LWIP_IC_H_
#define LWIP_IC_H_

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
#include "../../intercore/intercore.h"

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! Inter-core virtual driver object instance ID: R5_0_0->R5_0_1
 *  This instance is created on R5_0_0 to interface with R5_0_1 */
#define IC_ETH_IF_R5_0_0_R5_0_1     0
/*! Inter-core virtual driver object instance ID: R5_0_0->R5_0_1
 *  This instance is created on R5_0_1 to interface with R5_0_0 */
#define IC_ETH_IF_R5_0_1_R5_0_0     1
/*! Inter-core virtual driver object instance ID: R5_0_0->A53
 *  This instance is created on R5_0_0 to interface with A53
 *
 *  Note: The A53 side peer peer virtual driver does not use Lwip so we
 *  do not need an A53_to_R5_0_0 driver instance.
 */
#define IC_ETH_IF_R5_0_0_A53        2
/*! Total no. of inter-core virtual interfaces */
#define IC_ETH_MAX_VIRTUAL_IF       3

/*! LWIPIC Error Code: Success */
#define LWIPIC_OK              (0)
/*! LWIPIC Error Code: Failure */
#define LWIPIC_ERROR           (-1)

/*! Enables polling mode in the packet RX task */
#define IC_ETH_RX_POLLING_MODE      1
/*! Pooling interval used by the packet RX task */
#define IC_ETH_RX_POLLING_INTERVAL  1 /* milli-seconds */

/*! \brief MAC address length in bytes/octets. */
#define MAC_ADDR_LEN                     (6U)

/** \brief Core definitions */
#define    IPC_MPU1_0           (0U)    /**< ARM A53 - VM0 */
#define    IPC_MCU1_0           (1U)    /**< ARM MCU  R5F - core0 */
#define    IPC_MCU1_1           (2U)    /**< ARM MCU  R5F - core1 */
#define    IPC_R5_0_0           (3U)    /**< ARM Main R5F - core0 */
#define    IPC_R5_0_1           (4U)    /**< ARM Main R5F - core1 */
#define    IPC_R5_1_0           (5U)    /**< ARM Main R5F - core2 */
#define    IPC_R5_1_1           (6U)    /**< ARM Main R5F - core3 */
#define    IPC_C66X_1           (7U)    /**< DSP C66x - core0 */
#define    IPC_C66X_2           (8U)    /**< DSP C66x - core1 */
#define    IPC_C7X_1            (9U)    /**< DSP C7x - core0 */
#define    IPC_MPU1_1          (10U)    /**< ARM A53 - VM1 */
#define    IPC_MAX_PROCS       (11U)    /**< Maximum Processors */

/*!
 * \brief Info/debug print function prototype.
 *
 * This function is used by the driver to print info/debug messages.
 *
 * \param fmt          Formatted string followed by variable arguments
 */
typedef void (*LwipIc_Print)(const char *fmt, ...);

void LwipIc_printf(char *fmt,
                      ...);

#define ICETH_RPMSG_MSG_SIZE        (496U + 32U)
#define ICETH_NUM_RPMSG_BUFS        (256)
#define ICETH_RPMSG_OBJ_SIZE        (256)
#define ICETH_RPMSG_DATA_SIZE       (ICETH_RPMSG_MSG_SIZE * \
                                     ICETH_NUM_RPMSG_BUFS + \
                                     ICETH_RPMSG_OBJ_SIZE)
#define ICETH_IPC_NUM_REMOTE_PROCS  1

#define ICETH_IPC_ENDPT_BASE        10
#define ICETH_IPC_ENDPT_R5_0_0      (ICETH_IPC_ENDPT_BASE + 1)
#define ICETH_IPC_ENDPT_R5_0_0_A53  (ICETH_IPC_ENDPT_BASE + 2)
#define ICETH_IPC_ENDPT_R5_0_1      (ICETH_IPC_ENDPT_BASE + 3)
#define ICETH_IPC_ENDPT_MAX         (ICETH_IPC_ENDPT_BASE + 4)

#define ICETH_IPC_MSG_SIZE          (sizeof(IcEth_msg))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief lwIP interface layer's RX statistics.
 */
typedef struct LwipIc_RxStats_s
{
    uint32_t largePktErr;
    uint32_t pbufNullErr;
    uint32_t pbufTakeErr;
    uint32_t netifInputErr;
    uint32_t rxQEmptyWarn;
    uint32_t pktDeq;
    uint32_t pktSentToStk;
} LwipIc_RxStats;

/*!
 * \brief lwIP interface layer's TX statistics.
 */
typedef struct LwipIc_TxStats_s
{
    uint32_t pktDropInitErr;
    uint32_t pbufNullErr;
    uint32_t txQFullErr;
    uint32_t pktEnq;
} LwipIc_TxStats;

typedef struct LwipIc_Stats_s
{
    LwipIc_TxStats txStats;
    LwipIc_RxStats rxStats;
    uint32_t driverBufGet;
    uint32_t driverBufFree;
    uint32_t driverBufNullErr;
} LwipIc_Stats;

/**
 * \brief
 *  Inter-core virtual device object
 *
 * \details
 *  This structure stores the information for an inter-core virtual
 *  device instance.
 */
typedef struct LwipIc_OBJECT_
{
    /* Intercore trasport global object */
    Ic_Object_Handle hIcObj;

    /*! Instance ID */
    uint32_t instId;

    /* Mac Address assigned to this interface */
    uint8_t macAddr[MAC_ADDR_LEN];

    /* Initialization flag.*/
    volatile uint32_t initDone;

    /* Link is up flag. */
    uint32_t linkIsUp;

    /* No of PBM packets allocated by application - used for debug purpose.*/
    uint32_t numAllocPbufPkts;

    /* Handle to IPC receive task */
    TaskP_Object hIpcRxTask;

    /*
     * Handle to Rx task: Receives packets from the intercore transport
     * and give them to the LWIP stack.
     */
    TaskP_Object hRxPacketTask;

    /*
     * Handle to Rx semaphore: Posted by the IPC RX task on getting a
     * packet notification from the remote core.
     */
    SemaphoreP_Object hRxPacketSem;

    /* Handle to input task that sends pbufs to the LWIP stack */
    TaskP_Object hLWIPIF2LWIPinput;

    /*
     * Handle to counting shutdown semaphore, which all subtasks created in the
     * open function must post before the close operation can complete.
     */
    SemaphoreP_Object hShutDownSem;

    /* Boolean to indicate shutDownFlag status of translation layer.*/
    volatile bool shutDownFlag;

    /* Print Function */
    LwipIc_Print print;

    /* Stats */
    LwipIc_Stats stats;

} LwipIc_Object, *LwipIc_Handle;

typedef struct LwipIc_Params_
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
    char endPtName[IC_ENDPT_NAME_LEN_MAX];

    /*! IPC remote end point name */
    char remoteEndPtName[IC_ENDPT_NAME_LEN_MAX];

    /*! MAC address */
    uint8_t macAddr[MAC_ADDR_LEN];

} LwipIc_Params;

typedef struct LwipIc_QueueTbl_
{
    /*! start address of the shared memory queue from head */
    const void* pShdMemBuffStartAdd;

    /*! Number of elements in a single queue */
    const uint32_t elemCount;

    /*! Data size of each element */
    const uint32_t elemSize;

}LwipIc_QueueTbl;

/*!
 * \brief Intercore eth IPC message ID
 */
typedef enum IcEth_MsgType
{
    /*! General synchronization message*/
    ICETH_SYNC = 1,

    /*! Data available */
    ICETH_PKTS_AVAILABLE,

    /*! Unused message type */
    ICETH_MSG_MAX
} IcEth_MsgId;

/*!
 * \brief Intercore eth IPC message type
 */
typedef struct IcEth_Msg
{
    /*! Message ID */
    IcEth_MsgId msgId;

    /*! Src End point */
    uint32_t    srcCore;

    /*! Dst End point */
    uint32_t    dstCore;

    /*! Total packets queued */
    uint32_t    numPktsQd;

} IcEth_MsgObj;

/* ========================================================================== */
/*                          API/Public Function Declarations                  */
/* ========================================================================== */

/*
 * Functions Provided by our translation layer code
 */
extern LwipIc_Handle LwipIc_open(uint32_t instId, Ic_Object_Handle hIcObj);

extern void LwipIc_close(LwipIc_Handle hLwipIc);

extern int32_t LwipIc_remoteCorePktNotify(LwipIc_Handle hLwipIc);

/* ========================================================================== */
/*                         External Variable Declarations                     */
/* ========================================================================== */

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline void LwipIcStats_addOne(uint32_t *statCnt)
{
#if defined(LWIPIFIC_INSTRUMENTATION_ENABLED)
    *statCnt += 1U;
#endif

#if defined(LWIPIFIC_INSTRUMENTATION_ENABLED)
    *statCnt += addCnt;
#endif
}

#if !(IC_ETH_RX_POLLING_MODE)
int32_t LwipIc_ipcInit(LwipIc_Handle hLwipIc);
#endif

static inline void LwipIc_assert(bool cond)
{
    while(!cond)
        {}
    ;
}

#ifdef __cplusplus
}
#endif

#endif /* LWIP_IC_H_ */
