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

#ifndef __AVTPCF_H__
#define __AVTPCF_H__

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <stdint.h>
#include <enet.h>
#include <enet_cfg.h>
#include <include/core/enet_dma.h>
#include <include/per/cpsw.h>
#include <enet_ethutils.h>
#include "ether_ring.h"

typedef struct
{
    Enet_Handle hEnet;
    uint32_t coreKey;
    uint32_t coreId;

    EnetDma_TxChHandle hTxCh;
    uint32_t txChNum;

    EnetDma_RxChHandle hRxCh;
    uint32_t rxFlowIdx;

    struct
    {
        uint8_t txChCfgIdx;
        uint8_t rxChCfgIdx;
        uint32_t numTxPkts;
        uint32_t numRxPkts;
        uint32_t packetSize;
    }cfg;

    uint8_t macAddr[ENET_MAC_ADDR_LEN];
    EnetDma_PktQ txFreePktInfoQ;
    EnetDma_PktQ txSubmitQ;
    EtherRing_Handle hEtherRing;
} avtpcf_enetInfo_t;

typedef struct
{
    uint8_t* streamID;
    uint8_t seqID;
    uint8_t subtype;
    uint8_t sv;
    uint8_t version;
    uint8_t r;
    uint16_t ntscf_data_length;
}avtpcf_info;

typedef int (*avtpcf_recv_cb)(avtpcf_info *streamInfo, uint8_t *payload, int size, void *cbArgs);

typedef struct
{
    avtpcf_enetInfo_t enetInfo;
    avtpcf_recv_cb cbArray[255];
    void* cbArgsArray[255];
} avtpcf_data_t;

typedef struct __attribute__ ((packed))
{
    uint8_t subtype;
    uint8_t commonField[2];
    uint8_t sequence_num;
    uint8_t stream_id[8];
} avtpcf_ntscf_header;

typedef struct
{
    uint8_t mcastMac[6];
    uint8_t srcMac[6];
    uint16_t vlanID;
    uint8_t priority;
    uint8_t streamID[8];
} avtpcf_talker_cfg;

typedef struct
{
    uint8_t mcastMac[6];
    uint8_t streamID[8];
    avtpcf_recv_cb callback;
    void* cbArgs;
} avtpcf_listener_cfg;

typedef struct
{
    avtpcf_data_t* avtpcf_data;
    avtpcf_talker_cfg* cfg;
    EthVlanFrameHeader l2Header;
    avtpcf_ntscf_header avtpcf_header;
} avtpcf_talker_Obj;

typedef struct
{
    avtpcf_data_t* avtpcf_data;
    avtpcf_listener_cfg* cfg;
} avtpcf_listener_Obj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Provide enet info, channel number, etc. */
int32_t avtpcf_init(avtpcf_data_t* avtpData);

/* Provide stream ID. */
int32_t avtpcf_talker_init(avtpcf_data_t* avtpcfData, avtpcf_talker_Obj* talkerObj, avtpcf_talker_cfg* talkerCfg);

/* Provide stream ID, Callback function, callback function args. */
int32_t avtpcf_listener_init(avtpcf_data_t* avtpcfData, avtpcf_listener_Obj* listenerObj, avtpcf_listener_cfg* listenerCfg);

int32_t avtpcf_talker_enqueue(avtpcf_talker_Obj* talkerObj, uint8_t *payload, uint32_t size);

int32_t avtpcf_submit_packets(avtpcf_data_t* avtpcf_data);

int32_t avtpcf_recycle_packets(avtpcf_data_t* avtpcf_data);

int32_t avtpcf_listeners_spin(avtpcf_data_t* avtpData);

#endif /* __AVTPCF_H__ */