/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * \file  enet_cli.h
 *
 * \brief This file contains the APIs of enet_cli lib.
 */

/*!
 * \ingroup  NETWORKING_MODULE
 * \defgroup ENET_CLI_API Enet CLI API
 *
 * @{
 */

#ifndef _ENET_CLI_H_
#define _ENET_CLI_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "enet_cli_wrapper/enet_cli_wrapper.h"
/* SDK includes */
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"
#include "ti_enet_lwipif.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Maximum write and read buffer length for CLI */
#define MAX_WRITE_BUFFER_LEN 500
#define MAX_READ_BUFFER_LEN 200

/* Status flags for Dma channels */
#define CH_CLOSE 0
#define CH_IDLE 1
#define CH_RUNNING 2
#define CH_STOP 3

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

typedef enum EnetCLi_RemapType_e
{
    ENET_CLI_REMAP_INGRESS = 0,
    ENET_CLI_REMAP_EGRESS = 1
}EnetCli_RemapType;

typedef struct EnetCli_Obj_s
{
    Enet_Type enetType;
    uint32_t instId;
    Enet_Handle hEnet;
    uint8_t numMacPorts;
    uint32_t coreId;
} EnetCli_Obj;

/* Structure with parameters of an Ethernet object */
typedef struct EnetApp_Obj_s
{
    /* Enet driver */
    Enet_Type enetType;
    uint32_t instId;
    uint32_t coreId;
    uint32_t coreKey;
    uint32_t boardId;
    uint8_t numMacPorts;
    Enet_MacPort macPort[ENET_MAC_PORT_NUM];
    emac_mode macMode; /* MAC mode (defined in board library) */
    Enet_Handle hEnet;
    uint8_t hostMacAddr[ENET_MAC_ADDR_LEN];
    uint8_t txDmaCh[ENET_SYSCFG_TX_CHANNELS_NUM];
    uint8_t rxDmaCh[ENET_SYSCFG_RX_FLOWS_NUM];

    /* Packet transmission */
    EnetDma_TxChHandle hTxCh[ENET_SYSCFG_TX_CHANNELS_NUM];
    EnetDma_PktQ txFreePktInfoQ[ENET_SYSCFG_TX_CHANNELS_NUM];
    SemaphoreP_Object txSemObj[ENET_SYSCFG_TX_CHANNELS_NUM];

    /* Packet reception */
    EnetDma_RxChHandle hRxCh[ENET_SYSCFG_RX_FLOWS_NUM];
    TaskP_Object rxTaskObj[ENET_SYSCFG_RX_FLOWS_NUM];
    EnetDma_PktQ rxFreeQ[ENET_SYSCFG_RX_FLOWS_NUM];
    EnetDma_PktQ rxReadyQ[ENET_SYSCFG_RX_FLOWS_NUM];
    SemaphoreP_Object rxSemObj[ENET_SYSCFG_RX_FLOWS_NUM];

    /* Status flags */
    int8_t initFlag;
    int8_t tsnFlag;
    int8_t shellFlag;
} EnetApp_Obj;
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize variables necessary for CLI.
 *
 * Gets necessary data like enet handle, core ID and number of MAC
 * ports that are required by the built-in commands.
 *
 * \param enetType  Enet peripheral type
 * \param instId    Enet peripheral instance ID
 */
void EnetCli_init(Enet_Type enetType, uint32_t instId);

/*!
 * \brief Registers all built-in commands to command interpreter.
 *
 * Use this to enable built-in commands in the application. Should be
 * called only after calling EnetCli_init().
 */
void EnetCli_registerBuiltInCommands();


/* Function to read user input from the terminal */
void UART_readCLI(char *rxBuffer, uint32_t rxBufferLen);

/* Function to display string on the terminal */
void UART_writeCLI(char *txBuffer);

bool EnetCli_configCommandHandler(char *writeBuffer,
        size_t writeBufferLen, const char *commandString);

bool EnetCli_debugCommandHandler(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

bool EnetCli_phyCommandHandler(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

bool EnetCli_utilsCommandHandler(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern EnetCli_Obj EnetCli_inst;


/* Enet object instance declaration */
EnetApp_Obj EnetApp_inst;

/* Transaction for writing to and reading from UART */
extern UART_Transaction UART_trans;

/* Number of packets per tx channel */
int32_t EnetApp_pktPerTxCh[ENET_SYSCFG_TX_CHANNELS_NUM];

/* Number of packets per rx channel */
int32_t EnetApp_pktPerRxCh[ENET_SYSCFG_RX_FLOWS_NUM];
#ifdef __cplusplus
}
#endif

#endif /* _ENET_CLI_H_ */

/*! @} */
