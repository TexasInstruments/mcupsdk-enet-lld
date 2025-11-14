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

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <tsn_combase/combase.h>
#include <tsn_unibase/unibase_binding.h>
#include <tsn_uniconf/yangs/yang_modules.h>
#include <tsn_gptp/gptpman.h>
#include <tsn_gptp/tilld/lld_gptp_private.h>
#include <tsn_gptp/gptpconf/gptpgcfg.h>
#include <tsn_gptp/gptpconf/xl4-extmod-xl4gptp.h>
#include <tsn_uniconf/yangs/ieee1588-ptp-tt_access.h>
#include <tsn_uniconf/ucman.h>
#include <tsn_uniconf/uc_dbal.h>
#include <stdint.h>
#include "common.h"
#include "tsninit.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define QOSAPP_PRIORITY_MAX     (8)
#define DEFAULT_INTERFACE_INDEX (0)

extern uint8_t IETF_INTERFACES_func(uc_dbald *dbald);
#define IETF_INTERFACES_RW IETF_INTERFACES_func(dbald)

#define MBPS          (1000000ULL)
#define KBPS          (1000U)

#define CBSAPP_TASK_PRIORITY   (1)
#define CBSAPP_TASK_NAME       "cbsapp_task"

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct QoSAppCommonParam
{
    /*! Name of network interface */
    char *netdev;
    /*! index is priority, value of each index is TC, -1: not used. */
    int8_t priority2TcMapping[QOSAPP_PRIORITY_MAX];
    uint8_t nTCs;                 /*! Num of traffic classes */
    uint8_t nQueues;              /*! Num of HW queue */
} QoSAppCommonParam_t;

typedef struct EnetCbsPerQueueParam
{
    uint8_t tc;        //!< Traffic class;
    uint8_t priority;  //!< HW queue priority
    uint64_t idleSlope;    //!< idle slope in bit per second unit.
} EnetCbsPerQueueParam_t;

typedef struct EnetCbsParam
{
    EnetCbsPerQueueParam_t cbsparams[8u]; //!< List of parameters to be configured for each queue.
    int length; //!< How many elements in the list of the parameters.
} EnetCbsParam_t;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern EnetApp_Ctx_t gAppCtx;

static uint8_t gEnetCbsAppStackBuf[TSN_TSK_STACK_SIZE]
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

EnetCbsParam_t gCbsDefaultCfg = {
    .cbsparams =
    {
        /* Initial test program to initialize bandwidth for all queues.
            * The order for setting idleSlope must be stared from highest
            * priority queue 7 and next lower priority queue in decending
            * order.
            */
        {
            .tc = 7, .priority = 7,
            .idleSlope = 10*MBPS
        },
        {
            .tc = 6, .priority = 6,
            .idleSlope = 10*MBPS
        },
        {
            .tc = 5, .priority = 5,
            .idleSlope = 10*MBPS
        },
        {
            .tc = 4, .priority = 4,
            .idleSlope = 10*MBPS
        },
        {
            .tc = 3, .priority = 3,
            .idleSlope = 10*MBPS
        },
        {
            .tc = 2, .priority = 2,
            .idleSlope = 10*MBPS
        },
        {
            .tc = 1, .priority = 0,
            .idleSlope = 10*MBPS
        },
        {
            .tc = 0, .priority = 1,
            .idleSlope = 10*MBPS
        },
    },
    .length = 8U,
};

EnetCbsParam_t gCbsDemoCfg = {
    .cbsparams =
    {
        /* Initial test program to initialize bandwidth for all queues.
            * The order for setting idleSlope must be stared from highest
            * priority queue 7 and next lower priority queue in decending
            * order.
            */
        {
            .tc = 7, .priority = 7,
            .idleSlope = 30*MBPS
        },
        {
            .tc = 5, .priority = 5,
            .idleSlope = 200*MBPS
        },
        {
            .tc = 3, .priority = 3,
            .idleSlope = 200*MBPS
        },
        {
            .tc = 2, .priority = 2,
            .idleSlope = 30*MBPS
        },
    },
    .length = 0U,
};

/* ========================================================================== */
/*                          Function Declerations                             */
/* ========================================================================== */

static int EnetCbsApp_setCbsParam(EnetCbsParam_t *cbsPrm, char *ifname, EnetApp_dbArgs *dbarg, bool syncFlag);

static int EnetQoSApp_setCommonParam(QoSAppCommonParam_t *prm, EnetApp_dbArgs *dbarg);

static int EnetCbsApp_registerCbsEnableToUniconf(uc_dbald *dbald, uc_notice_data_t *ucntd, char *ifname);

static int EnetQoSApp_openDB(EnetApp_dbArgs *dbarg, char *dbName, const char *mode);

static void EnetQoSApp_closeDB(EnetApp_dbArgs *dbarg);

static void EnetCbsApp_configureCbs(char* dbName, char *netdev, EnetCbsParam_t* cbsParam, bool syncFlag);

static int EnetCbsApp_registerAdminIdleSlope(uc_dbald *dbald, uc_notice_data_t *ucntd,
                                             char *ifname, bool syncFlag, EnetCbsParam_t *cbsPrm, int i);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void *EnetApp_cbsAppTask(void *arg)
{
    EnetApp_ModuleCtx_t *modCtx = (EnetApp_ModuleCtx_t *)arg;
    EnetApp_Ctx_t *appCtx  = modCtx->appCtx;
    int netdevIdx = DEFAULT_INTERFACE_INDEX;

    EnetCbsApp_configureCbs(appCtx->dbName, appCtx->netdev[netdevIdx], &gCbsDefaultCfg, BTRUE);

    EnetCbsApp_configureCbs(appCtx->dbName, appCtx->netdev[netdevIdx], &gCbsDemoCfg, BTRUE);


    DebugP_log("Configured CBS \r\n\n\n");

    while (1)
    {
        CB_SLEEP(10);
    }
}

static void EnetCbsApp_configureCbs(char* dbName, char *netdev, EnetCbsParam_t* cbsParam, bool syncFlag)
{
    bool openDBSuccess = BFALSE;
    EnetApp_dbArgs dbarg;
    int err = 0, i;
    do
    {
        err = EnetQoSApp_openDB(&dbarg, dbName, "w");
        if (err)
        {
            DPRINT("Failed to open DB!");
            break;
        }

        openDBSuccess = BTRUE;
        QoSAppCommonParam_t prm = {
            .netdev = netdev,
            /* initialize with invalid TC value */
            {-1, -1, -1, -1, -1, -1, -1, -1},
            .nTCs = cbsParam->length,
            .nQueues = QOSAPP_PRIORITY_MAX,
        };
        for (i = 0; i < prm.nTCs; i++)
        {
            prm.priority2TcMapping[cbsParam->cbsparams[i].priority] =
                cbsParam->cbsparams[i].tc;
        }
        err = EnetQoSApp_setCommonParam(&prm, &dbarg);
        if (err)
        {
            DPRINT("Failed to set common param!");
            break;
        }
        err = EnetCbsApp_setCbsParam(cbsParam,
                                     netdev,
                                     &dbarg, syncFlag);
        if (err)
        {
            DPRINT("Failed to set CBS parameters !");
            break;
        }
    } while (0);

    if (openDBSuccess)
    {
        EnetQoSApp_closeDB(&dbarg);
    }
}

static int EnetCbsApp_setCbsParam(EnetCbsParam_t *cbsPrm, char *ifname,
                                  EnetApp_dbArgs *dbarg, bool syncFlag)
{

    uc_dbald *dbald = dbarg->dbald;
    uc_notice_data_t *ucntd = dbarg->ucntd;
    int i, err = 0;

    err=EnetCbsApp_registerCbsEnableToUniconf(dbald, ucntd, ifname);
    if (err != 0)
    {
        DPRINT("%s, Failed to trigger uniconf to write idleSlope",
               __func__);
    }
    for (i = 0; i < cbsPrm->length; i++)
    {
        // "/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/traffic-class/tc-data/lqueue"
        err=YDBI_SET_ITEM(ifk4vk1, ifname,
                IETF_INTERFACES_TRAFFIC_CLASS,
                IETF_INTERFACES_TC_DATA,
                IETF_INTERFACES_LQUEUE,
                255,
                &cbsPrm->cbsparams[i].tc, sizeof(cbsPrm->cbsparams[i].tc),
                YDBI_STATUS,
                (void*)&cbsPrm->cbsparams[i].tc,
                sizeof(cbsPrm->cbsparams[i].tc),
                YDBI_NO_NOTICE,
                YANG_DB_ONHW_NOACTION
                );
        DebugP_assert(err == 0);

        err=EnetCbsApp_registerAdminIdleSlope(dbald, ucntd,
                                            ifname, syncFlag,
                                            cbsPrm,
                                            i);
        DebugP_assert(err == 0);
    }

    return err;
}

static int EnetCbsApp_registerCbsEnableToUniconf(uc_dbald *dbald, uc_notice_data_t *ucntd,
                                                char *ifname)
{
    int err;
    uint8_t kn_traffic_sched[5] = {
        [0] = IETF_INTERFACES_BRIDGE_PORT,
        [1] = IETF_INTERFACES_TRAFFIC_CLASS,
        [2] = IETF_INTERFACES_CBS_ENABLED,
    };
    uint8_t kn_traffic_sched_size = 3;
    // "/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/traffic-class/cbs-enabled"
    bool cbs_enabled=true;
    err=YDBI_SET_ITEM(ifknvk0, ifname,
                kn_traffic_sched, kn_traffic_sched_size,
                YDBI_CONFIG,
                (void *)&cbs_enabled, sizeof(cbs_enabled),
                YDBI_NO_NOTICE);
    DebugP_assert(err == 0);

    void *kvs[]={(void*)ifname, NULL, NULL};
    uint8_t kss[]={strlen(ifname)+1, 0};
    uint8_t aps[]={IETF_INTERFACES_RW,
        IETF_INTERFACES_INTERFACES,
        IETF_INTERFACES_INTERFACE,
        IETF_INTERFACES_BRIDGE_PORT,
        IETF_INTERFACES_TRAFFIC_CLASS,
        IETF_INTERFACES_CBS_ENABLED,
        255u,
    };
    err=uc_nc_askaction_push(ucntd, dbald, aps, kvs, kss);
    if (err!=0)
    {
        DPRINT("uc_nc_askaction_push failed. err=%d\n", err);
    }
    else
    {
        DPRINT("%s: succeeded \n", __func__);
    }

    return err;
}



static int EnetCbsApp_registerAdminIdleSlope(uc_dbald *dbald, uc_notice_data_t *ucntd,
                                                char *ifname, bool syncFlag,
                                                EnetCbsParam_t *cbsPrm,
                                                int i)
{
    char sem_name[64];
    int err;
    uint32_t ksize;
    char key[UC_MAX_KEYSIZE];
    UC_NOTICE_SIG_T *sem = NULL;
    uint8_t aps[]={IETF_INTERFACES_RW, IETF_INTERFACES_INTERFACES,
                IETF_INTERFACES_INTERFACE, IETF_INTERFACES_BRIDGE_PORT,
                IETF_INTERFACES_TRAFFIC_CLASS, IETF_INTERFACES_TC_DATA,
                IETF_INTERFACES_ADMIN_IDLESLOPE,
                255u};
    if (syncFlag)
    {
        /*
        * We want to register a semaphore for a notification
        * on completing of admin-idleslope seeting at HW side
        * before going to the next TC's idle slope to make sure
        * the idleSlope of highest priority queue must be configured
        * first before going for the lower priority queue.
        * This restriction is only required at the initial time.
        * In the run time configuration, since all priority queues
        * have been configured, setting idle slope of any queue will
        * work fine.
        */
        snprintf(sem_name, sizeof(sem_name), "/cbs_wait_sem_%d",
                cbsPrm->cbsparams[i].tc);
        // scoped variable
        void *kvs[]={(void*)ifname, (void*)&cbsPrm->cbsparams[i].tc, sem_name, NULL};
        uint8_t kss[]={strlen(ifname)+1, sizeof(cbsPrm->cbsparams[i].tc), strlen(sem_name)+1};
        if(uc_nc_notice_register(ucntd, dbald, aps, kvs, kss, UC_NOTICE_DBVAL_ADD, &sem))
        {
            DPRINT("%s: uc_nc_notice_register failure. tc=%d\n", __func__, cbsPrm->cbsparams[i].tc);
            return -1;
        }
    }

    err=YDBI_SET_ITEM(ifk4vk1, ifname,
                IETF_INTERFACES_TRAFFIC_CLASS,
                IETF_INTERFACES_TC_DATA,
                IETF_INTERFACES_ADMIN_IDLESLOPE,
                255,
                &cbsPrm->cbsparams[i].tc, sizeof(cbsPrm->cbsparams[i].tc),
                YDBI_CONFIG,
                (void*)&cbsPrm->cbsparams[i].idleSlope,
                sizeof(cbsPrm->cbsparams[i].idleSlope),
                YDBI_NO_NOTICE,
                YANG_DB_ONHW_NOACTION
                );
    DebugP_assert(err == 0);

    // Ask uniconf to write adminIdleSlop to HW
    void *kvs[]={(void*)ifname, (void*)&cbsPrm->cbsparams[i].tc, NULL};
    uint8_t kss[]={strlen(ifname)+1, sizeof(cbsPrm->cbsparams[i].tc), 0};
    err=uc_nc_askaction_push(ucntd, dbald, aps, kvs, kss);
    if (err!=0)
    {
        DPRINT("uc_nc_askaction_push failed. err=%d\n", err);
    }
    else
    {
        DPRINT("ask uniconf to write adminIdleSlop succeeded \n", __func__);
    }
    // Now wait for uniconf to finish writing adminIdleSlop to HW
    if (sem)
    {
        /* Waiting for setting completed at the HW with the timeout */
        if (uc_notice_sig_check(BTRUE, sem, 200, __func__))
        {
            DPRINT("%s, Failed to get a notice from the uniconf",
                    __func__);
        } else
        {
            err = uc_nc_get_notice_act(ucntd, dbald,
                                        sem_name, key, &ksize);
            if (err)
            {
                DPRINT("There is no notice from the uniconf");
            }
            else
            {
                DPRINT("Registered adminIdleSlope finished. tc=%d", cbsPrm->cbsparams[i].tc);
            }
        }
        /* Release the semaphore */
        err = uc_nc_notice_deregister_all(ucntd, dbald, sem_name);
        if (err != 0)
        {
            DPRINT("Failed to unregister sempahore");
        }
        sem = NULL;
    }
    return err;

}

static int EnetQoSApp_setCommonParam(QoSAppCommonParam_t *prm,
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

static int EnetQoSApp_openDB(EnetApp_dbArgs *dbarg, char *dbName, const char *mode)
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

static void EnetQoSApp_closeDB(EnetApp_dbArgs *dbarg)
{
    uc_notice_close(dbarg->ucntd, 0);
    uc_dbal_close(dbarg->dbald, UC_CALLMODE_THREAD);
}

int EnetApp_addCbsAppModCtx(EnetApp_ModuleCtx_t *modCtxTbl)
{
    EnetApp_ModuleCtx_t cbsAppModCtx = {
        .enable = BTRUE,
        .stopFlag = BTRUE,
        .taskPriority = CBSAPP_TASK_PRIORITY,
        .taskName = CBSAPP_TASK_NAME,
        .stackBuffer = gEnetCbsAppStackBuf,
        .stackSize = sizeof(gEnetCbsAppStackBuf),
        .onModuleDBInit = NULL,
        .onModuleRunner = EnetApp_cbsAppTask,
        .appCtx = &gAppCtx
    };
    memcpy(&modCtxTbl[ENETAPP_CBS_TASK_IDX], &cbsAppModCtx,
           sizeof(EnetApp_ModuleCtx_t));
    return 0;
}