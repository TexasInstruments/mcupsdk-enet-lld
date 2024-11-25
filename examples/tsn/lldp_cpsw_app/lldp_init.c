/*
 *  Copyright (c) Texas Instruments Incorporated 2023
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

#include <tsn_combase/combase.h>
#include <tsn_unibase/unibase_binding.h>
#include <tsn_uniconf/yangs/yang_modules.h>
#include <tsn_uniconf/yangs/ieee1588-ptp-tt_access.h>
#include <tsn_uniconf/yangs/ieee802-dot1ab-lldp_access.h>
#include <tsn_uniconf/yangs/ieee802-dot1ab-lldp.h>
#include <tsn_uniconf/ucman.h>
#include <tsn_uniconf/uc_dbal.h>
#include "debug_log.h"
#include "tsninit.h"
#include "common.h"
#include <tsn_lldp/lldpd.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define LLDP_TASK_PRIORITY      (2)
#define LLDP_TASK_NAME          "lldpd_task"

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    const char **confFiles; /*TODO: Should lldp need to read config and write to DB?*/
    int numConf; /*TODO: Should lldp need to read config and write to DB?*/
    int vlanId;
} EnetApp_LldpOpt_t;

typedef struct
{
    char* ipvx;
    char* addr;
    char* key;
    char* val;
} EnetApp_LldpMgmtAddrKv_t;

typedef struct EnetApp_DbKeyVal_IntItem
{
    uint8_t key;
    uint32_t val;
    uint8_t sz;   // value size 1 (bool) or 4 (uint32_t)
    bool rw; // true: rw, false: ro
} EnetApp_DbKeyVal_IntItem_t;

typedef struct EnetApp_DbKeyVal_StrItem
{
    uint8_t key;
    char* val;
    bool rw; // true: rw, false: ro
} EnetApp_DbKeyVal_StrItem_t;

typedef struct
{
    // char* destMac;
    ub_macaddr_t destMac;
    EnetApp_DbKeyVal_IntItem_t cfgKeyValInt[9];
    EnetApp_DbKeyVal_StrItem_t cfgKeyValStr[1];
    // EnetApp_DbNameVal_t cfgKeyVal[11];
} EnetApp_LldpPortCfg_t;
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
extern int uc_dbal_setproc(uc_dbald *dbald, const char *name, int64_t pvalue);
static int EnetApp_initGLocalData(EnetApp_Ctx_t * ctx); // glocal=local+global
static int EnetApp_initPerPortData(char* ndev);
static void *EnetApp_lldpTask(void* arg);
static int EnetApp_lldpDbInit(EnetApp_ModuleCtx_t* modCtx, EnetApp_dbArgs *dbargs);

/* ========================================================================== */
/*                            Local Variables                                */
/* ========================================================================== */
static EnetApp_DbKeyVal_IntItem_t gLldpGlobalDataInt[] =
{
    {IEEE802_DOT1AB_LLDP_MESSAGE_FAST_TX, 1, sizeof(uint32_t), YDBI_CONFIG},
    {IEEE802_DOT1AB_LLDP_MESSAGE_TX_HOLD_MULTIPLIER, 4, sizeof(uint32_t), YDBI_CONFIG},
    {IEEE802_DOT1AB_LLDP_MESSAGE_TX_INTERVAL, 30, sizeof(uint32_t), YDBI_CONFIG},
    {IEEE802_DOT1AB_LLDP_REINIT_DELAY, 2, sizeof(uint32_t), YDBI_CONFIG},
    {IEEE802_DOT1AB_LLDP_TX_CREDIT_MAX, 5, sizeof(uint32_t), YDBI_CONFIG},
    {IEEE802_DOT1AB_LLDP_TX_FAST_INIT, 2, sizeof(uint32_t), YDBI_CONFIG},
};

static EnetApp_DbKeyVal_IntItem_t gLldpLocalSysDataInt[] =
{
    {IEEE802_DOT1AB_LLDP_CHASSIS_ID_SUBTYPE, 7, sizeof(uint32_t), YDBI_STATUS},
    {IEEE802_DOT1AB_LLDP_SYSTEM_CAPABILITIES_SUPPORTED, 0x07FF, 2, YDBI_STATUS},
    {IEEE802_DOT1AB_LLDP_SYSTEM_CAPABILITIES_ENABLED, 0x07BB, 2, YDBI_STATUS},
};

static EnetApp_DbKeyVal_StrItem_t gLldpLocalSysDataStr[] =
{
    // local system data
    {IEEE802_DOT1AB_LLDP_CHASSIS_ID , "00-01-02-03-04-05", YDBI_STATUS}, // Just a simple string in case of subtype is 'local (7)'
    {IEEE802_DOT1AB_LLDP_SYSTEM_NAME , "tilld", YDBI_STATUS},
    {IEEE802_DOT1AB_LLDP_SYSTEM_DESCRIPTION , "tilld", YDBI_STATUS},
};

static EnetApp_LldpPortCfg_t gLldpPortCfgData[] =
{
    {
        .destMac = {0x01, 0x80, 0xc2, 0x00, 0x00, 0x0e},
        .cfgKeyValInt =
        {
            {IEEE802_DOT1AB_LLDP_ADMIN_STATUS, 3, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_TLVS_TX_ENABLE, 0x0F, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_PORT_ID_SUBTYPE, 3, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_MESSAGE_FAST_TX, 2, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_MESSAGE_TX_HOLD_MULTIPLIER, 4, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_MESSAGE_TX_INTERVAL, 30, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_REINIT_DELAY, 2, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_TX_CREDIT_MAX, 5, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_TX_FAST_INIT, 2, sizeof(uint32_t), YDBI_CONFIG},
        },
        .cfgKeyValStr = 
        {
            {IEEE802_DOT1AB_LLDP_PORT_DESC, "tilld", YDBI_CONFIG},
        }
    },
    {
        .destMac = {0x01, 0x80, 0xc2, 0x00, 0x00, 0x03},
        .cfgKeyValInt =
        {
            {IEEE802_DOT1AB_LLDP_ADMIN_STATUS, 3, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_TLVS_TX_ENABLE, 0x0F, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_PORT_ID_SUBTYPE, 3, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_MESSAGE_FAST_TX, 2, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_MESSAGE_TX_HOLD_MULTIPLIER, 4, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_MESSAGE_TX_INTERVAL, 20, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_REINIT_DELAY, 2, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_TX_CREDIT_MAX, 5, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_TX_FAST_INIT, 2, sizeof(uint32_t), YDBI_CONFIG},
        },
        .cfgKeyValStr = 
        {
            {IEEE802_DOT1AB_LLDP_PORT_DESC, "tilld", YDBI_CONFIG},
        }
    },
    {
        .destMac = {0x01, 0x80, 0xc2, 0x00, 0x00, 0x00},
        .cfgKeyValInt =
        {
            {IEEE802_DOT1AB_LLDP_ADMIN_STATUS, 3, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_TLVS_TX_ENABLE, 0x0F, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_PORT_ID_SUBTYPE, 3, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_MESSAGE_FAST_TX, 2, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_MESSAGE_TX_HOLD_MULTIPLIER, 4, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_MESSAGE_TX_INTERVAL, 25, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_REINIT_DELAY, 2, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_TX_CREDIT_MAX, 5, sizeof(uint32_t), YDBI_CONFIG},
            {IEEE802_DOT1AB_LLDP_TX_FAST_INIT, 2, sizeof(uint32_t), YDBI_CONFIG},
        },
        .cfgKeyValStr = 
        {
            {IEEE802_DOT1AB_LLDP_PORT_DESC, "tilld", YDBI_CONFIG},
        }
    },
};

static uint8_t gLldpStackBuf[TSN_TSK_STACK_SIZE] \
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
extern EnetApp_Ctx_t gAppCtx;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int EnetApp_addLldpModCtx(EnetApp_ModuleCtx_t *modCtxTbl)
{
    EnetApp_ModuleCtx_t lldpModCtx = {
        .enable = BTRUE,
        .stopFlag = BTRUE,
        .taskPriority = LLDP_TASK_PRIORITY,
        .taskName = LLDP_TASK_NAME,
        .stackBuffer = gLldpStackBuf,
        .stackSize = sizeof(gLldpStackBuf),
        .onModuleDBInit = EnetApp_lldpDbInit,
        .onModuleRunner = EnetApp_lldpTask,
        .appCtx = &gAppCtx
    };
    memcpy(&modCtxTbl[ENETAPP_LLDP_TASK_IDX], &lldpModCtx,
           sizeof(EnetApp_ModuleCtx_t));
    return 0;
}

static int EnetApp_initGLocalData(EnetApp_Ctx_t *appCtx)
{
    int i;
    int ret;

    for (i = 0; i < sizeof(gLldpGlobalDataInt)/sizeof(gLldpGlobalDataInt[0]); i++)
    {
        ret=YDBI_SET_ITEM(abk1vk0, 
                    (void*)&gLldpGlobalDataInt[i].val, 
                    gLldpGlobalDataInt[i].sz,
                    gLldpGlobalDataInt[i].key, 
                     gLldpGlobalDataInt[i].rw, 
                    YDBI_NO_NOTICE);
        if (ret!=0)
        {
            DPRINT("%s: abk1vk0 failure key=%u/val=%u. ret=%d", __func__, gLldpGlobalDataInt[i].key, gLldpGlobalDataInt[i].val, ret);
            return ret;
        }
    }

    // local-system-data/
    uint8_t kn_if_id[2] = {
		[0] = IEEE802_DOT1AB_LLDP_LOCAL_SYSTEM_DATA,
	};
    attribute_pair_t dummy_attr; // Local system data has no value-key
    for (i = 0; i < sizeof(gLldpLocalSysDataInt)/sizeof(gLldpLocalSysDataInt[0]); i++)
    {
        kn_if_id[1] = gLldpLocalSysDataInt[i].key;
        ret=YDBI_SET_ITEM(abknvkn,
            kn_if_id,
            sizeof(kn_if_id)/sizeof(uint8_t),
            &dummy_attr,
            0,
            gLldpLocalSysDataInt[i].rw, //YDBI_STATUS,
            (void*)&gLldpLocalSysDataInt[i].val,
            gLldpLocalSysDataInt[i].sz,
            YDBI_NO_NOTICE);
        if (ret!=0)
        {
            DPRINT("%s: abknvkn failure key=%u/val=%u. ret=%d", __func__, gLldpGlobalDataInt[i].key, gLldpGlobalDataInt[i].val, ret);
            return ret;
        }
    }

    for (i = 0; i < sizeof(gLldpLocalSysDataStr)/sizeof(gLldpLocalSysDataStr[0]); i++)
    {
        kn_if_id[1] = gLldpLocalSysDataStr[i].key;
        ret=YDBI_SET_ITEM(abknvkn,
            kn_if_id,
            sizeof(kn_if_id)/sizeof(uint8_t),
            &dummy_attr,
            0,
            gLldpLocalSysDataInt[i].rw, //YDBI_STATUS,
            gLldpLocalSysDataStr[i].val,
            strlen(gLldpLocalSysDataStr[i].val)+1,
            YDBI_NO_NOTICE);
        if (ret!=0)
        {
            DPRINT("%s: abk1vk0 failure key=%u/val=%s. ret=%d", __func__, gLldpGlobalDataInt[i].key, gLldpGlobalDataInt[i].val, ret);
            return ret;
        }
    }

    DPRINT("[%s] Initialized LLDP GLocal System data", __func__);

    return ret;
}

static int EnetApp_initPerPortData(char* ndev)
{
    int ret;
    int i, j;
    uint8_t kn_if_id[2] = {
		[0] = IEEE802_DOT1AB_LLDP_PORT,
	};

    for (i = 0; i < sizeof(gLldpPortCfgData)/sizeof(gLldpPortCfgData[0]); i++)
    {
        // /ieee802-dot1ab-lldp/lldp/port|name:%s|dest-mac-address:%s|/
        attribute_pair_t attr[2]=
        {
            [0] = {ndev, strlen(ndev) +1},
            [1] = {gLldpPortCfgData[i].destMac, 6},
        };

        for (j = 0; j < sizeof(gLldpPortCfgData[i].cfgKeyValInt)/ sizeof(EnetApp_DbKeyVal_IntItem_t); j++ )
        {
            kn_if_id[1]=gLldpPortCfgData[i].cfgKeyValInt[j].key;
            ret = YDBI_SET_ITEM(abknvkn,
					kn_if_id,
					2,
					attr,
					2,
					gLldpPortCfgData[i].cfgKeyValInt[j].rw,
					(void*)&gLldpPortCfgData[i].cfgKeyValInt[j].val,
					gLldpPortCfgData[i].cfgKeyValInt[j].sz,
					YDBI_NO_NOTICE);
            if (ret!=0)
            {
                DPRINT("%s: abknvkn failure key=%u/val=%u. ret=%d", __func__, gLldpPortCfgData[i].cfgKeyValInt[j].key, gLldpPortCfgData[i].cfgKeyValInt[j].val, ret);
                return ret;
            }
        }

        for (j = 0; j < sizeof(gLldpPortCfgData[i].cfgKeyValStr)/ sizeof(EnetApp_DbKeyVal_StrItem_t); j++ )
        {
            kn_if_id[1]=gLldpPortCfgData[i].cfgKeyValStr[j].key;
            ret = YDBI_SET_ITEM(abknvkn,
					kn_if_id,
					2,
					attr,
					2,
					gLldpPortCfgData[i].cfgKeyValStr[j].rw,
					gLldpPortCfgData[i].cfgKeyValStr[j].val,
					strlen(gLldpPortCfgData[i].cfgKeyValStr[j].val)+1,
					YDBI_NO_NOTICE);
            if (ret!=0)
            {
                DPRINT("%s: abknvkn failure key=%u/val=%s. ret=%d", __func__, gLldpPortCfgData[i].cfgKeyValStr[j].key, gLldpPortCfgData[i].cfgKeyValStr[j].val, ret);
                return ret;
            }
        }
    }

    DPRINT("[%s] Initialized LLDP Port tilld%d", __func__, ndev);
    return 0;
}

static int EnetApp_lldpDbInit(EnetApp_ModuleCtx_t* modCtx, EnetApp_dbArgs *dbargs)
{
    int ret;
    EnetApp_Ctx_t *appCtx = modCtx->appCtx;

    lldpd_uniconf_access_mode(UC_CALLMODE_THREAD);
    
    ret=EnetApp_initGLocalData(appCtx);
    if ( ret!=0 ){return ret;}

    for (int ndev = 0; ndev < appCtx->netdevSize; ndev++)
    {
        ret=EnetApp_initPerPortData(appCtx->netdev[ndev]);
        if ( ret!=0 ){return ret;}
    }

    return ret;
}

static void *EnetApp_lldpTask(void* arg)
{
    EnetApp_ModuleCtx_t *modCtx = (EnetApp_ModuleCtx_t *)arg;
    EnetApp_Ctx_t *appCtx = modCtx->appCtx;

    if (lldpd_init(appCtx->dbName, NULL, appCtx->netdev, appCtx->netdevSize) == 0)
    {
        int64_t tid=(int64_t)&modCtx->hTaskHandle;
        uc_dbal_setproc(ydbi_access_handle()->dbald, "lldp", tid);
        DPRINT( "%s: register lldp tid=%" PRId64 "\n", __func__, tid);
        uint8_t *terminated = (uint8_t*)&modCtx->stopFlag;
        lldpd_run(terminated); // Blocking task

        lldpd_deinit();
    }
    else
    {
        DPRINT("%s: lldp initialized failure", __func__);
    }

    return NULL;
}
