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
 * \file     intercore.h
 *
 * \brief    Intercore transport interface data structures.
 */

#ifndef INTERCORE_H_
#define INTERCORE_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <drivers/ipc_rpmsg.h>
#include <intercore/include/ic_queue.h>
#include <intercore/include/ic_queue_data.h>
#include <intercore/include/ic_ipc.h>
#include <pbufQ_ic.h>

#ifdef __cplusplus
extern "C" {
#endif


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief Intercore transport object
 *
 * \details
 * This structure stores the information for an inter-core transport
 * object instance
 */
typedef struct Ic_Object_s
{
    /*! Self core ID */
    uint32_t selfCoreId;
    
    /*! Remote core ID */
    uint32_t remoteCoreId;

    /*! Number of remote cores */
    uint32_t numRemoteProcs; 

    /*! Remote proc array*/ 
    uint32_t remoteProcArray[ICETH_IPC_NUM_REMOTE_PROCS];

    /*! Our TX-Q ID */
    uint32_t txQId;

    /*! Our TX-Q handle */
    IcQ_Handle txQ_Handle;

    /*! Our RX-Q ID (=peer core's TX-Q ID) */
    uint32_t rxQId;
    
    /*! Our RX-Q handle */
    IcQ_Handle rxQ_Handle;

    /*! Our IPC end point ID */
    uint32_t myReqEndPtId;
    
    /*! Our IPC end point ID */
    uint32_t myEndPtId;

    /*! Our IPC end point name */
    char myEndPtName[ICETH_ENDPT_NAME_LEN_MAX];

    /*! Our IPC end point handle */
    RPMessage_Object* localEndPt_Handle;

    /*! Peer IPC end point name */
    char remoteEndPtName[ICETH_ENDPT_NAME_LEN_MAX];
    
    /*! Peer IPC end point handle */
    RPMessage_Object* remoteEndPt_Handle;

    /*! No. of TX packets sent */
    uint32_t numTxPktsSent;
    
    /*! No. of pkts send before the receiver is notified */
    uint32_t pktNotifyThresh;
    
    /*! No. of packets pending on the RX queue */
    uint32_t numRxPktsPending;

    /*! Flag to signal init complete */
    bool initComplete;

    /*! Queue with empty pbufs, payload is not populated */
    pbufIcQ freePbufQ;

} Ic_Object, *Ic_Object_Handle;

#ifdef __cplusplus
}
#endif

#endif /* INTERCORE_H */
