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

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>
#include <tsn_unibase/unibase.h>
#include <tsn_combase/combase.h>
#include <tsn_combase/cb_tmevent.h>
#include <tsn_gptp/tilld/lld_gptp_private.h>
#include <tsn_unibase/unibase_binding.h>
#include <tsn_uniconf/yangs/yang_modules.h>
#include <tsn_uniconf/ucman.h>
#include <tsn_uniconf/uc_dbal.h>

#ifdef GPTP_ENABLED
#include <tsn_gptp/gptpmasterclock.h>
#endif

#include "debug_log.h"
#include "tsninit.h"
#include "common.h"
#include "qosapp_misc.h"

/*============================================================================*/
/*                          Macros and Constants                              */
/*============================================================================*/
#define DISPLAY_BITRATE_INTERVAL_SEC (20)

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* mode could be "w" for writing  or "r" for reading */
int EnetQoSApp_openDB(EnetApp_dbArgs *dbarg, char *dbName, const char *mode)
{
    int res = 0;
    int timeout_ms = 500;
    do {
        res = uniconf_ready(dbName, UC_CALLMODE_THREAD, timeout_ms);
        if (res != 0)
        {
            DPRINT("The uniconf must be run first!");
            break;
        }
        res = -1;
        dbarg->dbald = uc_dbal_open(dbName, mode, UC_CALLMODE_THREAD);
        if (!dbarg->dbald)
        {
            DPRINT("Failed to open DB for EstApp!");
            break;
        }
        dbarg->ucntd = uc_notice_init(UC_CALLMODE_THREAD, dbName);
        if (!dbarg->ucntd)
        {
            DPRINT("Failed to open uc notice!");
            break;
        }
        res = 0;
    } while (0);
    return res;
}

void EnetQoSApp_closeDB(EnetApp_dbArgs *dbarg)
{
    uc_notice_close(dbarg->ucntd, 0);
    uc_dbal_close(dbarg->dbald, UC_CALLMODE_THREAD);
}

/// Note: How to intput ifk4vk1 param, look at full string below:
/// `k4vk1` it means the function can support setting up to 4 keys (k4) and one value key (vk1) started after `bridge-port`
/// ex: /ietf-interfaces/interfaces/interface|name:eno1|/bridge-port/traffic-class""/tc-data|tc:2|/lqueue 2
///  After `bridge-port`, k1,2,3=IETF_INTERFACES_TRAFFIC_CLASS, IETF_INTERFACES_TC_DATA, IETF_INTERFACES_LQUEUE (set k4=255 as not used)
//   vk1 is for IETF_INTERFACES_TC_DATA, and set next param+size to 2, and sizeof(int8)
//   In case of no vk, set to NULL and vksize is 0
int EnetQoSApp_setCommonParam(QoSAppCommonParam_t *prm,
                              EnetApp_dbArgs *dbarg)
{
    int err = 0, i;

    /* Write the num of traffic classes and value of each TC to DB */
    err=YDBI_SET_ITEM(ifk4vk1, prm->netdev, 
                IETF_INTERFACES_TRAFFIC_CLASS,
                IETF_INTERFACES_TRAFFIC_CLASS_TABLE,
                IETF_INTERFACES_NUMBER_OF_TRAFFIC_CLASSES,
                255,
                NULL, 0, YDBI_CONFIG, (void*)&prm->nTCs, sizeof(prm->nTCs),
                YDBI_NO_NOTICE,
                YANG_DB_ONHW_NOACTION
                );
    DebugP_assert(err == 0);

    /* Use one-to-one mapping of priority to logical queue */
    for (i = 0; i < prm->nTCs; i++)
    {
        err=YDBI_SET_ITEM(ifk4vk1, prm->netdev, 
                IETF_INTERFACES_TRAFFIC_CLASS,
                IETF_INTERFACES_TRAFFIC_CLASS_TABLE,
                IETF_INTERFACES_PRIORITY0+i,
                255,
                NULL, 0, YDBI_CONFIG, (void*)&prm->priority2TcMapping[i],
                sizeof(prm->priority2TcMapping[i]),
                YDBI_NO_NOTICE,
                YANG_DB_ONHW_NOACTION
                );
        DebugP_assert(err == 0);

        /* Map same number of priority to logical queue */
        err=YDBI_SET_ITEM(ifk4vk1, prm->netdev, 
                IETF_INTERFACES_TRAFFIC_CLASS,
                IETF_INTERFACES_TC_DATA,
                IETF_INTERFACES_LQUEUE,
                255,
                &prm->priority2TcMapping[i], sizeof(prm->priority2TcMapping[i]), 
                YDBI_STATUS, 
                (void*)&prm->priority2TcMapping[i],
                sizeof(prm->priority2TcMapping[i]),
                YDBI_NO_NOTICE,
                YANG_DB_ONHW_NOACTION
                );
        DebugP_assert(err == 0);
    }

    err=YDBI_SET_ITEM(ifk4vk1, prm->netdev, 
            IETF_INTERFACES_TRAFFIC_CLASS,
            IETF_INTERFACES_NUMBER_OF_PQUEUES,
            255,
            255,
            NULL, 0, YDBI_STATUS, (void*)&prm->nQueues, sizeof(prm->nQueues),
            YDBI_NO_NOTICE,
            YANG_DB_ONHW_NOACTION
            );
    DebugP_assert(err == 0);

    /* Use one-to-one mapping of logical queue to HW queue */
    for (i = 0; i < prm->nQueues; i++)
    {
        err=YDBI_SET_ITEM(ifk4vk1, prm->netdev, 
            IETF_INTERFACES_TRAFFIC_CLASS,
            IETF_INTERFACES_PQUEUE_MAP,
            IETF_INTERFACES_LQUEUE,
            255,
            &prm->priority2TcMapping[i], 1, YDBI_STATUS,
            (void*)&prm->priority2TcMapping[i], sizeof(prm->priority2TcMapping[i]),
            YDBI_NO_NOTICE,
            YANG_DB_ONHW_NOACTION
            );
        DebugP_assert(err == 0);
    }

    return err;
}

int8_t EnetQoSApp_getPortIdx(EnetQoSApp_AppCtx_t *ctx, char *netdev)
{
    int i;

    for (i = 0; i < ctx->netdevSize; i++)
    {
        if (strncmp(netdev, ctx->netdev[i], strlen(netdev)) == 0)
        {
            return i;
        }
    }

    return -1;
}
