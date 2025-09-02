/*
 *  Copyright (C) Texas Instruments Incorporated 2025
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
 * \file  ethernet_config.h
 *
 * \brief This file contains ethernet config API declarations
 */
#ifndef ETHERRING_CAN_CPSW_CONFIG_H_
#define ETHERRING_CAN_CPSW_CONFIG_H_

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <include/core/enet_osal.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <enet.h>
#include <enet_cfg.h>
#include <include/core/enet_dma.h>
#include <include/per/cpsw.h>
#include <enet_apputils.h>
#include <enet_appmemutils.h>
#include <enet_appmemutils_cfg.h>
#include "ti_board_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_enet_config.h"
#include "ti_enet_open_close.h"
#include "../etherring_can_cfg.h"
#include "../etherring_can_app.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

/**
 * @brief Initialize EnetApp configuration
 *
 * @param attachArgs    Pointer to Enet peripheral attach arguments
 * @param handleInfo    Pointer to Enet handle information
 * @param enetAppCfg    Pointer to Enet application configuration
 *
 * @return None
 */
void EnetApp_initAppCfg(EnetPer_AttachCoreOutArgs *attachArgs, EnetApp_HandleInfo *handleInfo, EnetApp_Cfg *enetAppCfg);

/**
 * @brief Open Enet DMA channels
 *
 * @param enetAppCfg    Pointer to Enet application configuration
 *
 * @return int32_t      Status of operation (0 for success, negative for error)
 */
int32_t EnetApp_openDma(EnetApp_Cfg *enetAppCfg);

/**
 * @brief Initialize EtherRing application
 *
 * @param etherRingCfg  Pointer to EtherRing configuration
 * @param enetAppCfg    Pointer to Enet application configuration
 *
 * @return int32_t      Status of operation (0 for success, negative for error)
 */
int32_t EnetApp_etherRingInit(EtherRing_Cfg* etherRingCfg, EnetApp_Cfg *enetAppCfg);

/**
 * @brief Update default VLAN port settings
 *
 * @param hEnet         Enet driver handle
 * @param coreId        Core ID
 *
 * @return int32_t      Status of operation (0 for success, negative for error)
 */
int32_t EnetApp_updateDefaultPortVlan(Enet_Handle hEnet, uint32_t coreId);

/**
 * @brief Add VLAN entries to the ALE table
 *
 * @param hEnet         Enet driver handle
 * @param coreId        Core ID
 * @param vlan          VLAN ID to add
 *
 * @return int32_t      Status of operation (0 for success, negative for error)
 */
int32_t EnetApp_addVlanEntries(Enet_Handle hEnet, uint32_t coreId, uint32_t vlan);

/**
 * @brief Configure multicast address for a specific node
 *
 * @param hEnet         Enet driver handle
 * @param coreId        Core ID
 * @param nodeId        Node ID
 *
 * @return int32_t      Status of operation (0 for success, negative for error)
 */
int32_t EnetApp_configureNodeMcastAddress(Enet_Handle hEnet, uint32_t coreId, uint8_t nodeId);

/**
 * @brief Configure node ID
 *
 * @param nodeId        Pointer to node ID variable
 *
 * @return None
 */
void EnetApp_configureNodeId(uint32_t* nodeId);

/**
 * @brief Update CPSW initialization configuration
 *
 * @param enetType      Enet peripheral type
 * @param instId        Instance ID
 * @param cpswCfg       Pointer to CPSW configuration structure
 *
 * @return None
 */
void EnetApp_updateCpswInitCfg(Enet_Type enetType, uint32_t instId, Cpsw_Cfg *cpswCfg);

/**
 * @brief Add multicast entry to the ALE table
 *
 * @param enetType      Enet peripheral type
 * @param instId        Instance ID
 * @param coreId        Core ID
 * @param testMCastAddr Multicast MAC address to add
 * @param portMask      Port mask for the multicast entry
 *
 * @return None
 */
void EnetApp_addMCastEntry(Enet_Type enetType,
                           uint32_t instId,
                           uint32_t coreId,
                           const uint8_t *testMCastAddr,
                           uint32_t portMask);

/**
 * @brief Wait for Ethernet link to become active
 *
 * @param numMacPorts   Number of MAC ports
 * @param macPorts      Array of MAC ports
 * @param hEnet         Enet driver handle
 * @param coreId        Core ID
 *
 * @return int32_t      Status of operation (0 for success, negative for error)
 */
int32_t EnetApp_waitForLinkUp(uint8_t numMacPorts, Enet_MacPort macPorts[], Enet_Handle hEnet, uint32_t coreId);

/**
 * @brief Add broadcast entry to the ALE table
 *
 * @param enetType      Enet peripheral type
 * @param instId        Instance ID
 * @param coreId        Core ID
 *
 * @return None
 */
void EnetApp_addBCastEntry(Enet_Type enetType,
                          uint32_t instId,
                          uint32_t coreId);

#ifdef __cplusplus
}
#endif

#endif /* ETHERRING_CAN_CPSW_CONFIG_H_ */
