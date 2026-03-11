/*
 * Copyright (C) 2026 Texas Instruments Incorporated
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
 * \file  ale_mcast.c
 *
 * \brief This file contains scripts to add and remove multicast address to ale table
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "cli_ale_mcast.h"
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

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

BaseType_t EnetCLI_addMcast(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    CpswAle_SetMcastEntryInArgs setMcastInArgs;
    uint32_t entryIdx;
    Enet_IoctlPrms prms;
    uint8_t macAddr[ENET_MAC_ADDR_LEN];
    BaseType_t paramLen;
    char *parameter;
    uint32_t paramCnt = 1;
    int32_t status;

    setMcastInArgs.addr.vlanId     = 0;
    setMcastInArgs.info.fwdState   = CPSW_ALE_FWDSTLVL_FWD;
    setMcastInArgs.info.portMask   = 0;
    setMcastInArgs.info.super      = false;
    setMcastInArgs.info.numIgnBits = 0;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
                &paramLen);

    if(parameter == NULL)
    {
        snprintf(writeBuffer, writeBufferLen, "Use 'help' command\r\n");
        return pdFALSE;
    }

    if(strncmp(parameter, "help", paramLen) == 0)
    {
        snprintf(writeBuffer, writeBufferLen, ""
                "\t       <mac addr>      : multicast mac address\r\n"
                "\t[-f]   <fwd_state_lvl> : forward state level\r\n"
                "\t[-mask]<port_mask>     : port mask to route the packet\r\n"
                "\t[-s]                   : super\r\n"
                "\t[-v]   <vlanId>        : vlan ID\r\n\n");
        return pdFALSE;
    }

    status = EnetAppUtils_macAddrAtoI(parameter, macAddr);
    if (status)
    {
        snprintf(writeBuffer, writeBufferLen, "Invalid MAC address\r\n");
        return pdFALSE;
    }
    EnetUtils_copyMacAddr(&setMcastInArgs.addr.addr[0U], macAddr);

    paramCnt++;
    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
            &paramLen);

    while (parameter != NULL)
    {
        if(strncmp(parameter, "-s", paramLen) == 0)
        {
            setMcastInArgs.info.super = true;
        }
        else if(strncmp(parameter, "-mask", paramLen) == 0)
        {
            paramCnt++;
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
                    &paramLen);
            if(parameter == NULL)
            {
                snprintf(writeBuffer, writeBufferLen, "Enter the MASK\r\n");
                return pdFALSE;
            }
            setMcastInArgs.info.portMask = atoi(parameter);
        }
        else if(strncmp(parameter, "-f", paramLen) == 0)
        {
            paramCnt++;
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
                    &paramLen);
            if(parameter == NULL)
            {
                snprintf(writeBuffer, writeBufferLen, "Enter the Forward state level\r\n");
                return pdFALSE;
            }
            setMcastInArgs.info.fwdState = atoi(parameter);
        }
        else if(strncmp(parameter, "-v", paramLen) == 0)
        {
            paramCnt++;
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
                    &paramLen);
            if(parameter == NULL)
            {
                snprintf(writeBuffer, writeBufferLen, "Enter Vlan ID\r\n");
                return pdFALSE;
            }
            setMcastInArgs.addr.vlanId = atoi(parameter);
        }
        else
        {
            snprintf(writeBuffer, writeBufferLen, "Invalid Parameter\r\n");
            return pdFALSE;
        }
        /* TO DO : add num bits ignored */

        paramCnt++;
        parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
                &paramLen);
    }


    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setMcastInArgs, &entryIdx);

    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId,
                CPSW_ALE_IOCTL_ADD_MCAST, &prms, status);

    if(status == ENET_SOK)
    {
        snprintf(writeBuffer, writeBufferLen,
                "Added mac address %02x:%02x:%02x:%02x:%02x:%02x succesfully\r\n",
                macAddr[0],
                macAddr[1],
                macAddr[2],
                macAddr[3],
                macAddr[4],
                macAddr[5]);
    }
    else
    {
        snprintf(writeBuffer, writeBufferLen,
                "Failed to add mac address %02x:%02x:%02x:%02x:%02x:%02x\r\n",
                macAddr[0],
                macAddr[1],
                macAddr[2],
                macAddr[3],
                macAddr[4],
                macAddr[5]);
    }

    return pdFALSE;
}

BaseType_t EnetCLI_removeMcast(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int32_t status;
    CpswAle_MacAddrInfo inArgs;
    char *parameter;
    BaseType_t paramLen;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 1, &paramLen);
    if(parameter == NULL)
    {
        snprintf(writeBuffer, writeBufferLen, "Invalid Parameter\r\n");
        return pdFALSE;
    }

    status = EnetAppUtils_macAddrAtoI(parameter, inArgs.addr);
    if (status)
    {
        snprintf(writeBuffer, writeBufferLen, "Invalid Parameter\r\n");
        return pdFALSE;
    }

    /* Remove multicast entry from ALE */
    Enet_IoctlPrms prms;
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId,
            CPSW_ALE_IOCTL_REMOVE_ADDR, &prms, status);
    if (status)
        snprintf(writeBuffer, writeBufferLen,
                "Failed to remove multicast entry to ALE\r\n");
    else
        snprintf(writeBuffer, writeBufferLen,
                "Removed multicast entry from ALE\r\n");
    return pdFALSE;
}
/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */
