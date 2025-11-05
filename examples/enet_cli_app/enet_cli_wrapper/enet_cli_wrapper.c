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
 * \file  enet_cli_port.c
 *
 * \brief This file contains the function definitions of FreeRTOS+CLI
 *        porting layer for enet_cli lib.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "enet_cli_wrapper.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static bool EnetCli_baseTypeToBool(BaseType_t val);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* Function to register custom commands */
void EnetCli_registerCustomCommands(CLI_Command_Definition_t *commandList,
        uint32_t numOfCommands)
{
    BaseType_t status;
    for (uint32_t commandIdx = 0; commandIdx < numOfCommands; commandIdx++)
    {
        status = FreeRTOS_CLIRegisterCommand(commandList + commandIdx);
        EnetAppUtils_assert(status == pdPASS);
    }
}

/* Function to process command input */
bool EnetCli_processCommand(const char *pCommandInput, char *pWriteBuffer,
        size_t writeBufferLen)
{
    BaseType_t moreDataToFollow;
    moreDataToFollow = FreeRTOS_CLIProcessCommand(pCommandInput, pWriteBuffer,
            writeBufferLen);
    return EnetCli_baseTypeToBool(moreDataToFollow);
}

const char* EnetCli_getParameter(const char *pCommandString,
        uint32_t wantedParam, uint32_t *paramLen)
{
    return FreeRTOS_CLIGetParameter(pCommandString, wantedParam,
            (BaseType_t*) paramLen);
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static bool EnetCli_baseTypeToBool(BaseType_t val)
{
    return (val == pdTRUE) ? true : false;
}

