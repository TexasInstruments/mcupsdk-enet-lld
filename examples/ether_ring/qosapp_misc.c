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
#include <tsn_uniconf/yangs/yang_db_runtime.h>
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
        dbarg->ydrd = yang_db_runtime_init(dbarg->dbald, NULL);
        if (!dbarg->ydrd)
        {
            DPRINT("Failed to init DB runtime!");
            break;
        }
        res = 0;
    } while (0);
    return res;
}

void EnetQoSApp_closeDB(EnetApp_dbArgs *dbarg)
{
    uc_notice_close(dbarg->ucntd, 0);
    yang_db_runtime_close(dbarg->ydrd);
    uc_dbal_close(dbarg->dbald, UC_CALLMODE_THREAD);
}

int EnetQoSApp_setCommonParam(QoSAppCommonParam_t *prm,
                              EnetApp_dbArgs *dbarg)
{
    int err = 0, i;
    char buffer[MAX_KEY_SIZE];
    char val[MAX_VAL_SIZE];
    yang_db_runtime_dataq_t *ydrd = dbarg->ydrd;

    /* Write the num of traffic classes and value of each TC to DB */
    snprintf(buffer, sizeof(buffer),
             TRAFFIC_CLASS_TABLE_NODE"/number-of-traffic-classes",
             prm->netdev);
    snprintf(val, sizeof(val), "%d", prm->nTCs);
    YANGDB_RUNTIME_WRITE(buffer, val);

    /* Use one-to-one mapping of priority to logical queue */
    for (i = 0; i < prm->nTCs; i++)
    {
        snprintf(buffer, sizeof(buffer),
                 TRAFFIC_CLASS_TABLE_NODE"/priority%d",
                 prm->netdev, i);
        snprintf(val, sizeof(val), "%d",
                 prm->priority2TcMapping[i]);
        YANGDB_RUNTIME_WRITE(buffer, val);

        /* Map same number of priority to logical queue */
        snprintf(buffer, sizeof(buffer),
                 TRAFFIC_CLASS_DATA_NODE"|tc:%d|/lqueue",
                 prm->netdev, prm->priority2TcMapping[i]);
        snprintf(val, sizeof(val), "%d", prm->priority2TcMapping[i]);
        YANGDB_RUNTIME_WRITE(buffer, val);
    }

    snprintf(buffer, sizeof(buffer),
             TRAFFIC_CLASS_NODE"/number-of-pqueues", prm->netdev);
    snprintf(val, sizeof(val), "%d", prm->nQueues);
    YANGDB_RUNTIME_WRITE(buffer, val);

    /* Use one-to-one mapping of logical queue to HW queue */
    for (i = 0; i < prm->nQueues; i++)
    {
        snprintf(buffer, sizeof(buffer),
                 PHYSICAL_QUEUE_MAP_NODE"|pqueue:%d|/lqueue",
                 prm->netdev, prm->priority2TcMapping[i]);
        snprintf(val, sizeof(val), "%d", prm->priority2TcMapping[i]);
        YANGDB_RUNTIME_WRITE(buffer, val);
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
