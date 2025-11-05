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
 * \file  enet_cli_port.h
 *
 * \brief Porting layer of enet_cli lib for FreeRTOS+CLI command interpreter.
 */

/*!
 * \ingroup  ENET_CLI_API
 * \defgroup FREERTOS_PLUS_CLI FreeRTOS Plus CLI Porting Layer
 *
 * @{
 */

#ifndef _ENET_CLI_MAIN_H_
#define _ENET_CLI_MAIN_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdbool.h>

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

#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Register custom commands to command interpreter.
 *
 * Use this function to register user defined commands when using FreeRTOS
 * Plus CLI command interpreter.
 *
 * \param commandList       Array of commands which needs to be registered
 * \param numOfCommands     Number of commands that needs to be registered
 */
void EnetCli_registerCustomCommands(CLI_Command_Definition_t *commandList,
        uint32_t numOfCommands);

/*!
 * \brief Processes the command and runs the associated function.
 *
 * Processes the command and executes the function that is associated to the
 * command.
 *
 * \param pCommandInput     The command to be processed
 * \param pWriteBuffer      Buffer to store output after command is executed
 * \param writeBufferLen    Length of the output buffer
 *
 * \return true if more data needs to be returned. Otherwise false.
 */
bool EnetCli_processCommand(const char *pCommandInput, char *pWriteBuffer,
        size_t writeBufferLen);

/*!
 * \brief Extracts a specific parameter from the command.
 *
 * \param pCommandString    The command from which the parameter needs to be extracted
 * \param wantedParam       The parameter index that needs to be extracted
 * \param paramLen          The length of the extracted parameter
 *
 * \return A pointer to the first character of the extracted parameter string.
 */
const char* EnetCli_getParameter(const char *pCommandString,
        uint32_t wantedParam, uint32_t *paramLen);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* _ENET_CLI_MAIN_H_ */

/*! @} */
