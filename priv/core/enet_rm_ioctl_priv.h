/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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
 * \file  enet_rm_priv.h
 *
 * \brief This file contains the private type definitions and helper macros for
 *        Enet Resource Manager.
 */

/*!
 * \addtogroup ENET_RM_API
 * @{
 */

#ifndef ENET_RM_IOCTL_PRIV_H_
#define ENET_RM_IOCTL_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <enet_cfg.h>
#include <include/core/enet_osal.h>
#include <include/core/enet_queue.h>
#include <include/core/enet_mod.h>
#include <include/core/enet_rm.h>


#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
#define ENET_COREKEY_CONVERT_MAGIC_NUM  (0x38ACB7E6U)
#define ENET_COREID_2_COREKEY(coreId)   (((coreId) * 100U) + ENET_COREKEY_CONVERT_MAGIC_NUM)
#define ENET_COREKEY_2_COREID(coreKey)  (((coreKey) - ENET_COREKEY_CONVERT_MAGIC_NUM) / 100U)
#define ENET_RM_RESOURCE_INTERNAL_FLAG              (1U << 31U)
#define ENET_RM_GET_INTERNAL_ALLOC_COREID(coreId)   ((coreId) | ENET_RM_RESOURCE_INTERNAL_FLAG)


/*!
 * \brief Helper macro used to first register private IOCTL handler and then invoke the
 *        IOCTL
 */
#define ENET_RM_PRIV_IOCTL(hRm, ioctlCmd,prms,status)                                 \
    do {                                                                              \
        Enet_IoctlPrms regIoctlPrms;                                                  \
        Enet_IoctlRegisterHandlerInArgs regIoctlInArgs;                               \
                                                                                      \
        regIoctlInArgs.cmd = ioctlCmd;                                                \
        regIoctlInArgs.fxn = (uintptr_t)&EnetRm_ioctl_handler_##ioctlCmd;             \
                                                                                      \
        ENET_IOCTL_SET_IN_ARGS(&regIoctlPrms, &regIoctlInArgs);                       \
        status = EnetMod_ioctl(hRm, ENET_RM_IOCTL_REGISTER_HANDLER, &regIoctlPrms);   \
        if (ENET_SOK == status)                                                       \
        {                                                                             \
            status = EnetMod_ioctl(hRm, ioctlCmd,prms);                               \
        }                                                                             \
    } while (0)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_ALLOC_MAC_ADDR(EnetRm_Handle hRm, Enet_IoctlPrms *prms);
int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_FREE_MAC_ADDR(EnetRm_Handle hRm, Enet_IoctlPrms *prms);
int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_ALLOC_RX_FLOW(EnetRm_Handle hRm, Enet_IoctlPrms *prms);
int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_FREE_RX_FLOW(EnetRm_Handle hRm, Enet_IoctlPrms *prms);
int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_ALLOC_TX_CH_PEERID(EnetRm_Handle hRm, Enet_IoctlPrms *prms);
int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_FREE_TX_CH_PEERID(EnetRm_Handle hRm, Enet_IoctlPrms *prms);
int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_VALIDATE_PERMISSION(EnetRm_Handle hRm, Enet_IoctlPrms *prms);
int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_ATTACH(EnetRm_Handle hRm, Enet_IoctlPrms *prms);
int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_DETACH(EnetRm_Handle hRm, Enet_IoctlPrms *prms);
int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_INTERNAL_ALLOC_RX_FLOW(EnetRm_Handle hRm, Enet_IoctlPrms *prms);
int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_INTERNAL_FREE_RX_FLOW(EnetRm_Handle hRm, Enet_IoctlPrms *prms);
int32_t EnetRm_ioctl_handler_ENET_RM_IOCTL_VALIDATE_RX_FLOW(EnetRm_Handle hRm, Enet_IoctlPrms *prms);

int32_t EnetRm_detachCore(EnetRm_Handle hRm, uint32_t coreKey);
int32_t EnetRm_validateCoreKey(EnetRm_Handle hRm, uint32_t coreKey);
int32_t EnetRm_freeResource(EnetRm_CoreResTbl_t *resTbl,
                            EnetRm_ResEntry_t *resList,
                            uint32_t resCount,
                            uint32_t coreId,
                            uint32_t resId);
bool EnetRm_isCoreAttached(EnetRm_CoreAttachInfo *coreAttachInfo,
                           uint32_t coreId,
                           uint32_t *attachIndex);
EnetRm_ResEntry_t *EnetRm_findResource(EnetRm_ResEntry_t *resTable,
                                       uint32_t numTableEntries,
                                       uint32_t coreId,
                                       uint32_t resId);



/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* ENET_RM_IOCTL_PRIV_H_ */
