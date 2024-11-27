/*
 *  Copyright (c) Texas Instruments Incorporated 2025
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
 * \file     ic_ipc.h
 *
 * \brief    IPC related definitions for the inter-core library.
 */

#ifndef IC_IPC_H_
#define IC_IPC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros                                           */
/* ========================================================================== */

#define ICETH_RPMSG_MSG_SIZE        (496U + 32U)
#define ICETH_NUM_RPMSG_BUFS        (256)
#define ICETH_RPMSG_OBJ_SIZE        (256)
#define ICETH_RPMSG_DATA_SIZE       (ICETH_RPMSG_MSG_SIZE * \
                                     ICETH_NUM_RPMSG_BUFS + \
                                     ICETH_RPMSG_OBJ_SIZE)
#define ICETH_IPC_NUM_REMOTE_PROCS  1

#define ICETH_IPC_ENDPT_BASE            10
#define ICETH_IPC_ENDPT_MCU2_0_R5_0_1   (ICETH_IPC_ENDPT_BASE + 1)
#define ICETH_IPC_ENDPT_MCU2_0_A72      (ICETH_IPC_ENDPT_BASE + 2)
#define ICETH_IPC_ENDPT_MCU2_1          (ICETH_IPC_ENDPT_BASE + 3)
#define ICETH_IPC_ENDPT_MCU3_0          (ICETH_IPC_ENDPT_BASE + 4)
#define ICETH_IPC_ENDPT_MCU2_0_R5_1_0   (ICETH_IPC_ENDPT_BASE + 5)
#define ICETH_IPC_ENDPT_MAX             (ICETH_IPC_ENDPT_BASE + 6)

#define ICETH_ENDPT_NAME_LEN_MAX    128
#define ICETH_IPC_MSG_SIZE          (sizeof(IcEth_msg))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

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
/*                          Function Declarations                             */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* IC_IPC_H_ */
