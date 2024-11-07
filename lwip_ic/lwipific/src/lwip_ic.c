/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/**
 * Copyright (c) 2024 Texas Instruments Incorporated
 *
 * This file is dervied from the ``ethernetif.c'' skeleton Ethernet network
 * interface driver for lwIP.
 *
 */

/* Standard language headers */
#include <stdio.h>
#include <assert.h>
#include <string.h>

/**
 * lwIP specific header files
 */
#include "lwip/opt.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"

/**
 * PDK header files
 */
#include <lwip_ic.h>
#include <ShdMemCircularBufferP_nortos.h>
#include "kernel/dpl/DebugP.h"


/* ========================================================================== */
/*                          Extern variables                                  */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                             Local Macros                                   */
/* ========================================================================== */

#define OS_TASKPRIHIGH              8

#define LWIPIF_IPC_INIT_TASK_PRI       (OS_TASKPRIHIGH)

/* Number of packets sent before a notification is sent to the receiver */
#define IC_PKT_NOTIFY_THRESHOLD     1

#if defined (SAFERTOS)
#define  IPC_TASK_STACK_ALIGN       (IPC_TASK_STACKSIZE)
#else
#define  IPC_TASK_STACK_ALIGN       0x2000U
#endif

/* ========================================================================== */
/*                    Local Function Declarations                             */
/* ========================================================================== */
#if !(IC_ETH_RX_POLLING_MODE)
static void LwipIc_ipcRxTaskFxn(void *arg0,
                                void *arg1);
#endif
/* ========================================================================== */
/*                            Local Variables                                 */
/* ========================================================================== */

LwipIc_Object gLwipIcObj[IC_ETH_MAX_VIRTUAL_IF];

LwipIc_Params gLwipIcMacParams[IC_ETH_MAX_VIRTUAL_IF]=
{
    {
        .macAddr            = {0x00,0x01,0x02,0x03,0x04,0x05},
    },
    {
        .macAddr            = {0x00,0x01,0x02,0x04,0x05,0x06},
    },
    {
        .macAddr            = {0x00,0x01,0x02,0x05,0x06,0x07},
    },
};

#if !(IC_ETH_RX_POLLING_MODE)
static uint8_t gLwipIc_ipcRxTaskStack[IC_ETH_MAX_VIRTUAL_IF][IPC_TASK_STACKSIZE]
__attribute__ ((section(".bss:taskStackSection")))
__attribute__ ((aligned(IPC_TASK_STACK_ALIGN)));
#endif

uint8_t  gLwipIc_rpMsgRecvBuf[IC_ETH_MAX_VIRTUAL_IF][ICETH_RPMSG_DATA_SIZE]
__attribute__ ((section (".ipc_data_buffer"), aligned (8)));

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


/* ========================================================================== */
/*                    API/Public Function Definitions                         */
/* ========================================================================== */

void LwipIc_printf(char *fmt,
                      ...)
{
    DebugP_log(fmt);
}

/**
 * Initializes intercore transport
 */
LwipIc_Handle LwipIc_open(uint32_t instId, Ic_Object_Handle hIcObj)
{
    int32_t status;
    LwipIc_Handle hLwipIc;

    hLwipIc         = &gLwipIcObj[instId];

    /* Initialize the allocated memory block. */
    memset(hLwipIc, 0, sizeof(LwipIc_Object));
    hLwipIc->hIcObj  = hIcObj;
    LwipIc_assert(hIcObj->initComplete == TRUE);

    hLwipIc->instId = instId;
    /* Create semaphore objects, init shutDownFlag status */
    hLwipIc->shutDownFlag = false;

    status = SemaphoreP_constructBinary(&hLwipIc->hShutDownSem, 0);
    LwipIc_assert(status == LWIPIC_OK);

    memcpy((void*)&(hLwipIc->macAddr), (void*)&gLwipIcMacParams[instId].macAddr, sizeof(hLwipIc->macAddr));

    /* TODO: Allow application to provide custom print function */
    hLwipIc->print = (LwipIc_Print) &LwipIc_printf;

    status = SemaphoreP_constructBinary(&hLwipIc->hRxPacketSem, 0);
    LwipIc_assert(status == LWIPIC_OK);

    hLwipIc->initDone = TRUE;
    hLwipIc->linkIsUp = TRUE;

    return hLwipIc;
}

/*!
 *  @b LwipIc_close
 *  @n
 *      Closes Ethernet peripheral and disables interrupts.
 *
 *  \param[in]  hLwipIc
 *      LwipIc_object structure pointer.
 *
 *  \retval
 *      void
 */
void LwipIc_close(LwipIc_Handle hLwipIc)
{

    LwipIc_assert(NULL != hLwipIc);
    /* Set the translation layer shutdown flag */
    hLwipIc->shutDownFlag = true;

    /* Pend on hShutDownSem (twice for two sub-tasks) */
    SemaphoreP_pend(&hLwipIc->hShutDownSem, SystemP_WAIT_FOREVER);
    SemaphoreP_pend(&hLwipIc->hShutDownSem, SystemP_WAIT_FOREVER);
    
    /* Delete the semaphore objects */
    SemaphoreP_destruct(&hLwipIc->hRxPacketSem);
    SemaphoreP_destruct(&hLwipIc->hShutDownSem);

    /* Clear the allocated translation */
    memset(hLwipIc, 0, sizeof(LwipIc_Object));
}

/* ========================================================================== */
/*                    Local Function Definitions                              */
/* ========================================================================== */
#if !(IC_ETH_RX_POLLING_MODE)
int32_t LwipIc_ipcInit(LwipIc_Handle hLwipIc)
{
    TaskP_Params taskParams;

    RPMessage_Params    rpMsgParams;
    int32_t             status;

    Ic_Object_Handle    hIcObj;
    hIcObj              = &(hLwipIc->icObj);

    /* Create RP message endpoint */
    RPMessageParams_init(&rpMsgParams);
    rpMsgParams.requestedEndpt = hIcObj->myReqEndPtId;
    rpMsgParams.buf = gLwipIc_rpMsgRecvBuf[hLwipIc->instId];
    rpMsgParams.bufSize = ICETH_RPMSG_DATA_SIZE;

    hIcObj->localEndPt_Handle = RPMessage_create(&rpMsgParams, &hIcObj->myEndPtId);
    if(!hIcObj->localEndPt_Handle)
    {
        hLwipIc->print("IPC init (core : %s): Failed to create endpoint\r\n", Ipc_mpGetSelfName());
    }
    else
    {
        status = RPMessage_announce(RPMESSAGE_ALL, hIcObj->myEndPtId, hIcObj->myEndPtName);
    }

    if(status != IPC_SOK) 
    {
        hLwipIc->print("IPC init (core : %s): RPMessage_announce() for %s failed\n",
                hIcObj->myEndPtName);
    }
    else
    {
        /* RPmsg RX task for messages coming in to our ic eth endpoint */
        TaskP_Params_init(&taskParams);
        taskParams.name         = (const char *)"LwipIc_ipcRxTask";
        taskParams.priority     = LWIPIF_IPC_INIT_TASK_PRI;
        taskParams.stack        = gLwipIc_ipcRxTaskStack[hLwipIc->instId];
        taskParams.stacksize    = IPC_TASK_STACKSIZE;
        taskParams.arg0         = hLwipIc;

        hLwipIc->hIpcRxTask = TaskP_create(&LwipIc_ipcRxTaskFxn, &taskParams);
        LwipIc_assert(NULL != hLwipIc->hIpcRxTask);
    }

    return status;
}

static void LwipIc_ipcRxTaskFxn(void *arg0,
                                     void *arg1)
{
    uint32_t		remoteEndPt;
    uint32_t		remoteProcId;
    uint16_t		len;
    int32_t		status = 0;
    IcEth_MsgObj        rxMsg;
    
    LwipIc_Handle  hLwipIc; 
    Ic_Object_Handle    hIcObj;

    hLwipIc = (LwipIc_Handle)arg0;
    hIcObj       = &(hLwipIc->icObj);


    while(!hLwipIc->shutDownFlag)
    {
        status = RPMessage_recv(hIcObj->localEndPt_Handle,
                                (void*)&rxMsg, &len,
                                &remoteEndPt,
                                &remoteProcId,
                                IPC_RPMESSAGE_TIMEOUT_FOREVER);
        if(status != IPC_SOK) 
        {
            hLwipIc->print("\nRecvTask: failed with code %d\n", status);
        }
        else
        {
            LwipIc_assert(rxMsg.dstCore == hIcObj->selfCoreId);

            hIcObj->numRxPktsPending += rxMsg.numPktsQd;

            SemaphoreP_post(hLwipIc->hRxPacketSem); 
        }
    }

    /* We are shutting down, notify that we are done */
    SemaphoreP_post(hLwipIc->hShutDownSem);

}


int32_t LwipIc_remoteCorePktNotify(LwipIc_Handle hLwipIc)
{
    uint32_t            remoteEndPt;
    uint32_t            remoteProcId;
    int32_t             status = 0;
    IcEth_MsgObj        txMsg;

    Ic_Object_Handle    hIcObj;
    hIcObj              = &(hLwipIc->icObj);

    /* Prepare the notification */
    memset((void*)&txMsg, 0, sizeof(txMsg));

    txMsg.msgType   = ICETH_PKTS_AVAILABLE;
    txMsg.srcCore   = hIcObj->selfCoreId;
    txMsg.dstCore   = hIcObj->remoteCoreId;
    txMsg.numPktsQd = hIcObj->pktNotifyThresh;

    /* Get remote endpoint Id */
    status = RPMessage_getRemoteEndPt(hIcObj->remoteCoreId,
                                      hIcObj->remoteEndPtName,
                                      &remoteProcId,
                                      &remoteEndPt,
                                      SemaphoreP_WAIT_FOREVER);

    if(hIcObj->remoteCoreId != remoteProcId) 
    {
        hLwipIc->print("\nSendTask-%d: RPMessage_getRemoteEndPt() failed, status %d\n",
                hIcObj->remoteCoreId, status);
    }
    else
    {
        status = RPMessage_send(hIcObj->localEndPt_Handle,
                                hIcObj->remoteCoreId,
                                remoteEndPt,
                                hIcObj->myEndPtId,
                                (Ptr)&txMsg,
                                (uint16_t)ICETH_IPC_MSG_SIZE);
    }

    if (status != IPC_SOK) 
    {
        hLwipIc->print("\nSendTask-%d: Failed from %s to %s...\n", 
                            hIcObj->remoteCoreId,
                            Ipc_mpGetSelfName(),
                            Ipc_mpGetName(hIcObj->remoteCoreId));
    }

    return status;
}
#endif
