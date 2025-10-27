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
 * \file
 *
 * \brief
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "cpsw_cfg.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EnetApp_Cfg gEnetAppCfg =
{
    .name = ENETAPP_DEFAULT_CFG_NAME,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ============================================================================ */


#ifdef SOC_AM243X
#define TIMESYNC_ROUTER_DEVID   (TISCI_DEV_TIMESYNC_EVENT_INTROUTER0)
#define DEST_IRQ  (30u)
#endif
#ifdef SOC_AM62DX
#define TIMESYNC_ROUTER_DEVID   (TISCI_DEV_TIMESYNC_EVENT_ROUTER0)
#define DEST_IRQ  (10u)
#endif
#ifdef SOC_AM275X
#define TIMESYNC_ROUTER_DEVID   (TISCI_DEV_TIMESYNC_EVENT_INTROUTER0)
#define DEST_IRQ  (8u)
#endif

/*
   Routes the timer overflow to CPTS HW push event 1
*/
int32_t EnetApp_setTimeSyncRouter(void)
{
    int32_t                             retVal;
    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;
    rmIrqReq.valid_params           = 0U;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqReq.global_event           = 0U;
    rmIrqReq.src_id                 = TIMESYNC_ROUTER_DEVID;
    rmIrqReq.src_index              = 0U;
    rmIrqReq.dst_id                 = TIMESYNC_ROUTER_DEVID;
    rmIrqReq.dst_host_irq           = DEST_IRQ;
    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    retVal = Sciclient_rmIrqSetRaw(&rmIrqReq, &rmIrqResp, SystemP_WAIT_FOREVER);
    if(0 != retVal)
    {
        DebugP_log("Timesync router : Sciclient event config failed!!!\r\n");
    }

    return retVal;
}

void EnetApp_registerCallback(void * cb, void* ClockSync_Handle ,Enet_Handle hEnet, uint32_t coreId)
{
    CpswCpts_RegisterHwPushCbInArgs inArgs;
    inArgs.hwPushNum = CPSW_CPTS_HWPUSH_1;
    inArgs.hwPushNotifyCb = (CpswCpts_HwPushNotifyCb)cb;
    inArgs.hwPushNotifyCbArg = ClockSync_Handle;

    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    ENET_IOCTL(hEnet, coreId, CPSW_CPTS_IOCTL_REGISTER_HWPUSH_CALLBACK, &prms, status);
    if(status != ENET_SOK)
    {
        DebugP_log("[Error] Failed to configure HWPUSH CALLBACK !!!\r\n");
        DebugP_assert(false);
    }
}

uint64_t EnetApp_getHwPushEventTimestamp(void)
{
    CpswCpts_Event inArgs = {
        .eventType = CPSW_CPTS_EVENTTYPE_HW_TS_PUSH,
        .hwPushNum = CPSW_CPTS_HWPUSH_1,
    };
    CpswCpts_Event outArgs;
    Enet_IoctlPrms prms;
    uint64_t timestamp = 0;
    int32_t status = ENET_SOK;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &outArgs);
    ENET_IOCTL(gEnetAppCfg.hEnet,
                gEnetAppCfg.coreId,
                CPSW_CPTS_IOCTL_LOOKUP_EVENT,
                &prms, status);
    EnetAppUtils_assert(status == ENET_SOK);
    timestamp = outArgs.tsVal;
    return timestamp;
}


void EnetApp_setCPTStime(int32_t nudge)
{
    int32_t inArgs = nudge;
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    ENET_IOCTL(gEnetAppCfg.hEnet,
                gEnetAppCfg.coreId,
                CPSW_CPTS_IOCTL_SET_TS_NUDGE,
                &prms, status);
    EnetAppUtils_assert(status == ENET_SOK);
}

uint64_t EnetApp_getCurrentTimestamp(void)
{
    Enet_IoctlPrms prms;
    uint64_t timestamp = 0;
    int32_t status = ENET_SOK;
    ENET_IOCTL_SET_OUT_ARGS(&prms, &timestamp);

    ENET_IOCTL(gEnetAppCfg.hEnet,
                gEnetAppCfg.coreId,
                ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP,
                &prms, status);
    EnetAppUtils_assert(status == ENET_SOK);

    return timestamp;
}

void EnetApp_updateCpswInitCfg(Enet_Type enetType,  uint32_t instId, Cpsw_Cfg *cpswCfg)
{
    CpswHostPort_Cfg *hostPortCfg = &cpswCfg->hostPortCfg;
    CpswAle_Cfg *aleCfg = &cpswCfg->aleCfg;
    CpswCpts_Cfg *cptsCfg = &cpswCfg->cptsCfg;

    /* Set Enet global runtime log level */
    Enet_setTraceLevel(ENET_TRACE_DEBUG);

    /* Peripheral config */
    cpswCfg->vlanCfg.vlanAware = false;

    /* Host port config */
    hostPortCfg->removeCrc      = true;
    hostPortCfg->padShortPacket = true;
    hostPortCfg->passCrcErrors  = true;

    /* ALE config */
    aleCfg->modeFlags                          = CPSW_ALE_CFG_MODULE_EN;
    aleCfg->agingCfg.autoAgingEn               = true;
    aleCfg->agingCfg.agingPeriodInMs           = 1000;
    aleCfg->nwSecCfg.vid0ModeEn                = true;
    aleCfg->vlanCfg.aleVlanAwareMode           = false;
    aleCfg->vlanCfg.cpswVlanAwareMode          = false;
    aleCfg->vlanCfg.unknownUnregMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownRegMcastFloodMask   = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownVlanMemberListMask  = CPSW_ALE_ALL_PORTS_MASK;

    /* CPTS config */
    /* Note: Timestamping and MAC loopback are not supported together because of
    * IP limitation, so disabling timestamping for this application */
   cptsCfg->hostRxTsEn = false;

}

