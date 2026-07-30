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
 * \file  ale_unicast.c
 *
 * \brief This file contains scripts to add unicast entry to ALE
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "cli_ale_unicast.h"

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

BaseType_t EnetCLI_addUcast(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    CpswAle_SetUcastEntryInArgs setUcastInArgs;
    int32_t status;
    uint8_t macAddr[ENET_MAC_ADDR_LEN];
    char *parameter;
    BaseType_t paramLen;
    uint32_t paramCnt = 1;
    uint8_t makeDefault = 0;
    uint32_t entryIdx;
    Enet_IoctlPrms prms;

    setUcastInArgs.addr.vlanId = 0U;
    setUcastInArgs.info.portNum = CPSW_ALE_HOST_PORT_NUM;
    setUcastInArgs.info.blocked = false;
    setUcastInArgs.info.secure = false;
    setUcastInArgs.info.super = false;
    setUcastInArgs.info.ageable = false;
    setUcastInArgs.info.trunk = false;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
            &paramLen);

    if(parameter == NULL)
    {
        snprintf(writeBuffer, writeBufferLen, "Use 'help' command to see Usage\r\n");
        return pdFALSE;
    }

    if(strncmp(parameter, "help", paramLen) == 0)
    {
        snprintf(writeBuffer, writeBufferLen, ""
                "\t     <mac address>: unicast mac address\r\n"
                "\t[-d]              : make default mac address\r\n"
                "\t[-p] <portNum>    : mac port number\r\n"
                "\t[-b]              : blocked\r\n"
                "\t[-sec]            : secure\r\n"
                "\t[-s]              : super\r\n"
                "\t[-t]              : trunk\r\n"
                "\t[-a]              : ageable\r\n"
                "\t[-vid]<vlanId>    : vlan ID\r\n\n");
        return pdFALSE;
    }

    status = EnetAppUtils_macAddrAtoI(parameter, macAddr);
    if (status)
    {
        snprintf(writeBuffer, writeBufferLen, "Invalid MAC address\r\n");
        return pdFALSE;
    }
    EnetUtils_copyMacAddr(&setUcastInArgs.addr.addr[0U], macAddr);

    paramCnt++;
    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
            &paramLen);

    while (parameter != NULL)
    {
        if (strncmp(parameter, "-d", paramLen) == 0)
            makeDefault = 1;
        else if(strncmp(parameter, "-p", paramLen) == 0)
        {
            paramCnt++;
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
                    &paramLen);

            if (parameter == NULL)
            {
                snprintf(writeBuffer, writeBufferLen, "Enter Port number\r\n");
                return pdFALSE;
            }

            uint8_t macPortNum = atoi(parameter);
            macPortNum = CPSW_ALE_ALEPORT_TO_MACPORT(macPortNum);
            if ((macPortNum >= ENET_MAC_PORT_FIRST) && (macPortNum <= ENET_MAC_PORT_LAST))
            {
                setUcastInArgs.info.portNum = macPortNum;
            }
            else
            {
                snprintf(writeBuffer, writeBufferLen, "Invalid Parameter for -p (mac port number should be between %d and %d only)\r\n", ENET_MAC_PORT_FIRST, ENET_MAC_PORT_LAST);
                return pdFALSE;
            }
        }
        else if(strncmp(parameter, "-b", paramLen) == 0)
        {
            setUcastInArgs.info.blocked = true;
        }
        else if(strncmp(parameter, "-sec", paramLen) == 0)
        {
            setUcastInArgs.info.secure = true;
        }
        else if(strncmp(parameter, "-t", paramLen) == 0)
        {
            setUcastInArgs.info.trunk = true;
        }
        else if(strncmp(parameter, "-a", paramLen) == 0)
        {
            setUcastInArgs.info.ageable = true;
        }
        else if(strncmp(parameter, "-s", paramLen) == 0)
        {
            setUcastInArgs.info.super = true;
        }
        else if(strncmp(parameter, "-vid", paramLen) == 0)
        {
            paramCnt++;
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
                    &paramLen);

            if (parameter == NULL)
            {
                snprintf(writeBuffer, writeBufferLen, "Enter Vlan ID\r\n");
                return pdFALSE;
            }

            if(atoi(parameter) > 0 && atoi(parameter) < 4096)
            {
                setUcastInArgs.addr.vlanId = atoi(parameter);
            }
            else
            {
                snprintf(writeBuffer, writeBufferLen, "Invalid Vlan ID\r\n");
                return pdFALSE;
            }

        }
        else
        {
            snprintf(writeBuffer, writeBufferLen, "Invalid Parameter\r\n");
            return pdFALSE;
        }

        paramCnt++;
        parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
                &paramLen);
    }

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setUcastInArgs, &entryIdx);

    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId,
            CPSW_ALE_IOCTL_ADD_UCAST, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("[ERR] %s: Failed to add unicast entry: %d\r\n",
                __func__, status);
        return pdFALSE;
    }
    EnetAppUtils_print("[INF] %s: Added Unicast entry with MAC address: ",
            __func__);
    EnetAppUtils_printMacAddr(macAddr);

    snprintf(writeBuffer, writeBufferLen, "Added unicast entry to ALE\r\n");
    if (makeDefault)
    {
        EnetUtils_copyMacAddr(EnetApp_inst.hostMacAddr, macAddr);
        EnetAppUtils_print("[INF] %s: Default MAC address set to ",
            __func__);
        EnetAppUtils_printMacAddr(EnetApp_inst.hostMacAddr);
    }
    return pdFALSE;
}

BaseType_t EnetCLI_removeUcast(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int32_t status;
    CpswAle_MacAddrInfo inArgs;
    char *parameter;
    BaseType_t paramLen;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 1, &paramLen);
    if (parameter == NULL)
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

    /* Remove unicast entry from ALE */
    Enet_IoctlPrms prms;
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId,
            CPSW_ALE_IOCTL_REMOVE_ADDR, &prms, status);
    if (status)
        snprintf(writeBuffer, writeBufferLen,
                "Failed to remove unicast entry to ALE\r\n");
    else
        snprintf(writeBuffer, writeBufferLen,
                "Removed unicast entry from ALE\r\n");
    return pdFALSE;
}
