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

#ifndef ENET_RM_PRIV_H_
#define ENET_RM_PRIV_H_

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

/*! \brief Helper macro to create private IOCTL commands for Enet RM module. */
#define ENET_RM_PRIVATE_IOCTL(x)              (ENET_IOCTL_TYPE_PRIVATE | \
                                               ENET_IOCTL_RM_BASE |      \
                                               ENET_IOCTL_PER_GENERIC |  \
                                               ENET_IOCTL_MIN(x))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief Enet RM private IOCTL commands.
 */
typedef enum EnetRm_PrivIoctls_e
{
    /*!
     * \brief Validate IOCTL permission.
     *
     * IOCTL parameters:
     * -  inArgs: EnetRm_ValidatePermissionInArgs
     * - outArgs: None
     */
    ENET_RM_IOCTL_VALIDATE_PERMISSION = ENET_RM_PRIVATE_IOCTL(0U),

    /*!
     * \brief Attach core.
     *
     * IOCTL parameters:
     * -  inArgs: uint32_t coreId
     * - outArgs: uint32_t coreKey
     */
    ENET_RM_IOCTL_ATTACH = ENET_RM_PRIVATE_IOCTL(1U),

    /*!
     * \brief Detach core.
     *
     * IOCTL parameters:
     * -  inArgs: uint32_t coreKey
     * - outArgs: None
     */
    ENET_RM_IOCTL_DETACH = ENET_RM_PRIVATE_IOCTL(2U),

    /*!
     * \brief Internal RX flow allocation.
     *
     * IOCTL parameters:
     * -  inArgs: uint32_t coreId
     * - outArgs: EnetRm_AllocRxFlow rxAllocOutArgs
     */
    ENET_RM_IOCTL_INTERNAL_ALLOC_RX_FLOW = ENET_RM_PRIVATE_IOCTL(3U),

    /*!
     * \brief Internal RX flow free.
     *
     * IOCTL parameters:
     * -  inArgs: EnetRm_FreeInternalRxFlowInArgs
     * - outArgs: None
     */
    ENET_RM_IOCTL_INTERNAL_FREE_RX_FLOW = ENET_RM_PRIVATE_IOCTL(4U),

    /*!
     * \brief Validate RX flow is allocated resource and is from a attached core.
     *
     * IOCTL parameters:
     * -  inArgs: EnetRm_ValidateRxFlowInArgs
     * - outArgs: None
     */
    ENET_RM_IOCTL_VALIDATE_RX_FLOW = ENET_RM_PRIVATE_IOCTL(5U),

    /*!
     * \brief Register IOCTL handler
     *
     * IOCTL params:
     * -  inArgs: #Enet_IoctlRegisterHandlerInArgs
     * - outArgs: None
     *
     * Type: Synchronous.
     */
    ENET_RM_IOCTL_REGISTER_HANDLER = ENET_RM_PRIVATE_IOCTL(6U),
} EnetRm_PrivIoctls;

/*!
 * \brief Input args for ENET_RM_IOCTL_VALIDATE_PERMISSION command.
 */
typedef struct EnetRm_ValidatePermissionInArgs_s
{
    /*! IOCTL command */
    uint32_t cmd;

    /*! Core id from which the cmd is invoked */
    uint32_t coreId;
} EnetRm_ValidatePermissionInArgs;

/*!
 * \brief Input args for #ENET_RM_IOCTL_INTERNAL_ALLOC_RX_FLOW command.
 */
typedef struct EnetRm_AllocInternalRxFlowInArgs_s
{
    /*! Core id */
    uint32_t coreId;

    /*! 0-relative RX channel index */
    uint32_t chIdx;
} EnetRm_AllocInternalRxFlowInArgs;

/*!
 * \brief Input args for ENET_RM_IOCTL_FREE_RX_FLOW command.
 */
typedef struct EnetRm_FreeInternalRxFlowInArgs_s
{
    /*! Core Key */
    uint32_t coreId;

    /*! 0-relative RX channel index */
    uint32_t chIdx;

    /*! Rx Flow Index */
    uint32_t flowIdx;
} EnetRm_FreeInternalRxFlowInArgs;

/*!
 * \brief Input args for ENET_RM_IOCTL_VALIDATE_RX_FLOW command.
 */
typedef EnetRm_FreeRxFlowInArgs EnetRm_ValidateRxFlowInArgs;

/*!
 * \brief Enet RM configuration parameters.
 */
typedef struct EnetRm_Cfg_s
{
    /*! Peripheral type */
    Enet_Type enetType;

    /*! Enet peripheral instance id */
    uint32_t instId;

    /*! Number of RX channels in hardware. This determines the number of
      * rxStartFlowIdx and rxFlowIdxCnt pairs that are needed.
      * For instance, ICSSG Switch requires two channels, hence it needs two
      * RX flow ranges. */
    uint32_t numRxCh;

    /*! RX flow start index */
    uint32_t rxStartFlowIdx[ENET_RM_NUM_RXCHAN_MAX];

    /*! Number of RX flows allocated */
    uint32_t rxFlowIdxCnt[ENET_RM_NUM_RXCHAN_MAX];

    /*! Resource partition information */
    EnetRm_ResPrms resPartInfo;

    /*! IOCTL permission information */
    EnetRm_IoctlPermissionTable ioctlPermissionInfo;

    /*! MAC address list to be managed */
    EnetRm_MacAddressPool macList;
} EnetRm_Cfg;

/*!
 * \brief Resource manager entry.
 */
typedef struct EnetRm_ResEntry_s
{
    /*! Enet queue node entry. Must be the first element */
    EnetQ_Node node;

    /*! Core id of the resource owner */
    uint32_t ownerCoreId;

    /*! Resource id */
    uint32_t id;
}  EnetRm_ResEntry_t;

/*!
 * \brief Resources of a given core.
 */
typedef struct EnetRm_CoreResInfo_s
{
    /*! Core id of the resource owner */
    uint32_t coreId;

    /*! Queue of resources being owned by core */
    EnetQ resQ;
} EnetRm_CoreResInfo_t;

/*!
 * \brief Resource table of all cores.
 */
typedef struct EnetRm_CoreResTbl_s
{
    /*! Number of cores */
    uint32_t numCores;

    /*! Resource table for each core */
    EnetRm_CoreResInfo_t coreResTbl[ENET_CFG_REMOTE_CLIENT_CORES_MAX];
} EnetRm_CoreResTbl_t;

/*!
 * \brief TX resources.
 */
typedef struct EnetRm_TxChObj_s
{
    /*! TX offset */
    uint32_t txPSILThreadIdOffset;

    /*! Resource count */
    uint32_t resCnt;

    /*! TX resource table for all applicable cores */
    EnetRm_CoreResTbl_t txResTbl;

    /*! TX resource entries */
    EnetRm_ResEntry_t txRes[ENET_CFG_RM_TX_CH_MAX];
} EnetRm_TxChObj;

/*!
 * \brief RX resources.
 */
typedef struct EnetRm_RxFlowIdxObj_s
{
    /*! Core id of the owner of the internal RX channel */
    uint32_t internalAllocCoreId;

    /*! Internal allocation counter */
    uint32_t internalAllocCount;

    /*! Resource count */
    uint32_t resCnt;

    /*! RX resource table for all applicable cores */
    EnetRm_CoreResTbl_t rxResTbl;

    /*! RX resource entries */
    EnetRm_ResEntry_t rxRes[ENET_CFG_RM_RX_CH_MAX];
} EnetRm_RxFlowIdxObj;

/*!
 * \brief MAC address resources.
 */
typedef struct EnetRm_MacAddressObj_s
{
    /*! Resource count */
    uint32_t resCnt;

    /*! MAC resource table for all applicable cores */
    EnetRm_CoreResTbl_t macTbl;

    /*! MAC resources entries */
    EnetRm_ResEntry_t macRes[ENET_CFG_RM_MAC_ADDR_MAX];
} EnetRm_MacAddressObj;

/*!
 * \brief Information about attached cores.
 */
typedef struct EnetRm_CoreAttachInfo_s
{
    /*! Number of cores currently attached */
    uint32_t numCores;

    /*! Ids of the attached cores */
    uint32_t attachedCores[ENET_CFG_REMOTE_CLIENT_CORES_MAX];
} EnetRm_CoreAttachInfo;

/*!
 * \brief Enet Resource Manager driver object.
 */
typedef struct EnetRm_Obj_s
{
    /*! EnetMod must be the first member */
    EnetMod_Obj enetMod;

    /*! Configuration parameters passed at open time to the RM module */
    EnetRm_Cfg cfg;

    /*! TX resources being managed */
    EnetRm_TxChObj txObj;

    /*! RX resources being managed */
    EnetRm_RxFlowIdxObj rxObj[ENET_RM_NUM_RXCHAN_MAX];

    /*! Number of RX channels */
    uint32_t numRxCh;

    /*! MAC addresses pool being managed */
    EnetRm_MacAddressObj macObj;

    /*! Information about attached cores */
    EnetRm_CoreAttachInfo coreAttachObj;
} EnetRm_Obj;

/*!
 * \brief Enet RM module handle.
 */
typedef EnetRm_Obj *EnetRm_Handle;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Open and initialize Enet RM module.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters
 * \param cfgSize   Size of the configuration parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetRm_open(EnetMod_Handle hMod,
                    Enet_Type enetType,
                    uint32_t instId,
                    const void *cfg,
                    uint32_t cfgSize);

/*!
 * \brief Rejoin Enet RM module for a running peripheral.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetRm_rejoin(EnetMod_Handle hMod,
                      Enet_Type enetType,
                      uint32_t instId);

/*!
 * \brief Run an IOCTL operation on Enet RM.
 *
 * \param hMod      Enet Module handle
 * \param cmd       IOCTL command Id
 * \param prms      IOCTL parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetRm_ioctl(EnetMod_Handle hMod,
                     uint32_t cmd,
                     Enet_IoctlPrms *prms);

/*!
 * \brief Close Enet RM module.
 *
 * \param hMod      Enet Module handle
 */
void EnetRm_close(EnetMod_Handle hMod);

/*! @} */

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

#endif /* ENET_RM_PRIV_H_ */
