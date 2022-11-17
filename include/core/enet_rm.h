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
 * \file  enet_rm.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet Resource Manager module.
 */

/*!
 * \ingroup  DRV_ENET_MODULE
 * \defgroup ENET_RM_API Enet Resource Management
 * @{
 */

#ifndef ENET_RM_H_
#define ENET_RM_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <include/core/enet_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Helper macro to create IOCTL commands for Enet RM module. */
#define ENET_RM_PUBLIC_IOCTL(x)               (ENET_IOCTL_TYPE_PUBLIC | \
                                               ENET_IOCTL_RM_BASE |     \
                                               ENET_IOCTL_PER_GENERIC | \
                                               ENET_IOCTL_MIN(x))

/*!
 * \name Resource Manager Error Codes
 *
 * Error codes returned by RM functions.
 * @{
 */

/*! \brief Enet RM invalid TX channel number. */
#define ENET_RM_TXCHNUM_INVALID                          (0xABCDABCDU)

/*! \brief Enet RM invalid RX flow id. */
#define ENET_RM_RXFLOWIDX_INVALID                        (0xABCDABCDU)

/*!\brief Enet RM invalid core. */
#define ENET_RM_INVALIDCORE                              (0x5562AEFEU)

/*! @} */

/*! \brief Maximum number of entries for IOCTL permission */
#define ENET_RM_NUM_IOCTL_PERM_ENTRY_MAX                 (16U)

/*! \brief Maximum number of macaddress to be managed by RM */
#define ENET_RM_NUM_MACADDR_MAX                          (10U)

/*!
 * \brief Maximum number of RX channels (i.e. sets of flows).
 *
 * This is needed only for ICSSG Switch where there are two RX channels
 * (with 8 flows each) for the host port.  For all other peripherals,
 * there is a single RX channel and multiple flows.
 */
#define ENET_RM_NUM_RXCHAN_MAX                           (2U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief Enet RM IOCTL commands.
 */
typedef enum EnetRm_Ioctls_e
{
    /*!
     * \brief Alloc MAC address.
     *
     * IOCTL parameters:
     * -  inArgs: uint32_t coreKey
     * - outArgs: #EnetRm_AllocMacAddrOutArgs
     */
    ENET_RM_IOCTL_ALLOC_MAC_ADDR = ENET_RM_PUBLIC_IOCTL(0U),

    /*!
     * \brief Free MAC address.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetRm_FreeMacAddrInArgs
     * - outArgs: None
     */
    ENET_RM_IOCTL_FREE_MAC_ADDR = ENET_RM_PUBLIC_IOCTL(1U),

    /*!
     * \brief Alloc RX flow.
     *
     * IOCTL parameters:
     * -  inArgs: uint32_t coreKey
     * - outArgs: #EnetRm_AllocRxFlow
     */
    ENET_RM_IOCTL_ALLOC_RX_FLOW = ENET_RM_PUBLIC_IOCTL(2U),

    /*!
     * \brief Free RX flow.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetRm_FreeRxFlowInArgs
     * - outArgs: None
     */
    ENET_RM_IOCTL_FREE_RX_FLOW = ENET_RM_PUBLIC_IOCTL(3U),

    /*!
     * \brief Alloc TX channel peer id.
     *
     * IOCTL parameters:
     * -  inArgs: uint32_t coreKey
     * - outArgs: uint32_t txPSILThreadId
     */
    ENET_RM_IOCTL_ALLOC_TX_CH_PEERID = ENET_RM_PUBLIC_IOCTL(4U),

    /*!
     * \brief Free TX channel peer id.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetRm_FreeTxChInArgs
     * - outArgs: None
     */
    ENET_RM_IOCTL_FREE_TX_CH_PEERID = ENET_RM_PUBLIC_IOCTL(5U),
} EnetRm_Ioctls;

/*!
 * \brief Enet RM resource information.
 *
 * This structure holds the number of TX channels, RX flows and MAC addresses
 * partitioned to the core.
 */
typedef struct EnetRm_ResourceInfo_s
{
    /*! Core Id for which resource info is applicable */
    uint32_t coreId;

    /*! Number of TX channels */
    uint32_t numTxCh;

    /*! Number of RX channels */
    uint32_t numRxCh;

    /*! Number of RX Flows. The total number of Rx flows required for the RX
     *  channel is the sum of all RX flows alloted to different cores.
     *
     *  The total number of RX flows should be <= maximum number of RX flows
     *  supported by the SoC. Refer to the Enet SoC layer implementation to find
     *  out the maximum number of flows supported by SoC */
    uint32_t numRxFlows;

    /*! Number of MAC addresses */
    uint32_t numMacAddress;
} EnetRm_ResourceInfo;

/*!
 * \brief Enet RM resource parameters.
 *
 * This structure has the information about reserved flow, default flow and
 * Enet RM DMA resource information of all cores.
 */
typedef struct EnetRm_ResPrms_s
{
    /*! Number of cores */
    uint32_t numCores;

    /*! DMA Resource Information of all cores */
    EnetRm_ResourceInfo coreDmaResInfo[ENET_CFG_REMOTE_CLIENT_CORES_MAX];
} EnetRm_ResPrms;

/*!
 * \brief Enet RM IOCTL permission entry.
 *
 * This structure has the information about IOCTL permissions for each IOCTL
 * command.
 */
typedef struct EnetRm_IoctlPermissionEntry_s
{
    /*! IOCTL command */
    uint32_t cmd;

    /*! Cores from which this IOCTL call is allowed. This is a bitmask of allowed
     *  coreIds */
    uint32_t permittedCoreMask;
} EnetRm_IoctlPermissionEntry;

/*!
 * \brief Enet RM IOCTL permission table.
 *
 * This structure has the information about IOCTL permissions for all IOCTL
 * commands.
 */
typedef struct EnetRm_IoctlPermissionTable_s
{
    /*! Permitted core mask if IOCTL command not found in explicit permission
     *  entry table */
    uint32_t defaultPermittedCoreMask;

    /*! Number of explicit permission entry */
    uint32_t numEntries;

    /*! Permission table entries. Must have numEntries valid entries */
    EnetRm_IoctlPermissionEntry entry[ENET_RM_NUM_IOCTL_PERM_ENTRY_MAX];
} EnetRm_IoctlPermissionTable;

/*!
 * \brief Enet RM MAC address pool.
 *
 * This structure has the pool of MAC addresses that are being managed.
 */
typedef struct EnetRm_MacAddressPool_s
{
    /*! Number of MAC addresses */
    uint32_t numMacAddress;

    /*! Array of MAC addresses, not to exceed #ENET_RM_NUM_MACADDR_MAX */
    uint8_t macAddress[ENET_RM_NUM_MACADDR_MAX][ENET_MAC_ADDR_LEN];
} EnetRm_MacAddressPool;

/*!
 * \brief Input args for #ENET_RM_IOCTL_ALLOC_RX_FLOW command.
 */
typedef struct EnetRm_AllocRxFlowInArgs_s
{
    /*! Core key */
    uint32_t coreKey;

    /*! 0-relative RX channel index */
    uint32_t chIdx;
} EnetRm_AllocRxFlowInArgs;

/*!
 * \brief Output args for #ENET_RM_IOCTL_ALLOC_RX_FLOW command.
 */
typedef struct EnetRm_AllocRxFlow_s
{
    /*! Rx flow base or start index */
    uint32_t startIdx;

    /*! Allocated flow's index (offset from #startIdx) */
    uint32_t flowIdx;
} EnetRm_AllocRxFlow;

/*!
 * \brief Input args for #ENET_RM_IOCTL_ALLOC_MAC_ADDR command.
 */
typedef struct EnetRm_AllocMacAddrOutArgs_s
{
    /*! MAC address */
    uint8_t macAddr[ENET_MAC_ADDR_LEN];
} EnetRm_AllocMacAddrOutArgs;

/*!
 * \brief Input args for #ENET_RM_IOCTL_FREE_TX_CH_PEERID command.
 */
typedef struct EnetRm_FreeTxChInArgs_s
{
    /*! Core key */
    uint32_t coreKey;

    /*! Tx channel PSIL Thread Id  */
    uint32_t txChNum;
} EnetRm_FreeTxChInArgs;

/*!
 * \brief Input args for #ENET_RM_IOCTL_FREE_RX_FLOW command.
 */
typedef struct EnetRm_FreeRxFlowInArgs_s
{
    /*! Core key */
    uint32_t coreKey;

    /*! 0-relative RX channel index */
    uint32_t chIdx;

    /*! Rx flow index */
    uint32_t flowIdx;
} EnetRm_FreeRxFlowInArgs;

/*!
 * \brief Input args for #ENET_RM_IOCTL_FREE_MAC_ADDR command.
 */
typedef struct EnetRm_FreeMacAddrInArgs_s
{
    /*! Core key */
    uint32_t coreKey;

    /*! MAC address */
    uint8_t macAddr[ENET_MAC_ADDR_LEN];
} EnetRm_FreeMacAddrInArgs;

/*!
 * \brief Resource manager configuration parameters.
 *
 * Resources that will be managed by Enet RM module for partitioning across cores.
 */
typedef struct EnetRm_ResCfg_s
{
    /*! Self core Id */
    uint32_t selfCoreId;

    /*! Resource Partition Information */
    EnetRm_ResPrms resPartInfo;

    /*! IOCTL permission info */
    EnetRm_IoctlPermissionTable ioctlPermissionInfo;

    /*! MAC address list to be managed by RM */
    EnetRm_MacAddressPool macList;
} EnetRm_ResCfg;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

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

#endif /* ENET_RM_H_ */

/*!
 *  @}
 */
