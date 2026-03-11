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
 * \file  phymode.c
 *
 * \brief This file contains scripts to change phy Mode
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "cli_phy_phymode.h"

#include <enet_board.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct Enetcli_Phy_Cfg_s
{
    /* Duplexity of the phy mode. Can be Full duplex, Half duplex or Auto. */
    Enet_Duplexity duplexity;

    /* Speed of the phy mode. Can be 10mbit, 100mbit, 1gbit or Auto. */
    Enet_Speed speed;

    /* Macport where the phy mode needs to be set. */
    uint32_t macport;

} Enetcli_Phy_Cfg;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Reads the parameters and sets configuration to set the phy
*  mode for a particular macport. */
static BaseType_t Enetcli_phyModeCfg(char *writeBuffer,
                                         size_t writeBufferLen,
                                         const char *commandString);

/* Function which closes a port which is open. */
static int32_t Enetcli_closePortLink();

/* Function which opens the port link after setting the phy
*  mode configuration according to what the user set. */
static int32_t Enetcli_openPortLink();

static int32_t EnetApp_macMode2PhyMii(emac_mode macMode,
                                    EnetPhy_Mii *mii);

static void EnetApp_macMode2MacMii(emac_mode macMode, EnetMacPort_Interface *pMii);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint8_t gMacPortNum;
Enetcli_Phy_Cfg Enetcli_configObj;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

BaseType_t EnetCli_phyMode(char *writeBuffer,
                           size_t writeBufferLen,
                           const char *commandString)
{
    int32_t status = ENET_SOK;
    BaseType_t config_status = true;

    Enetcli_configObj.duplexity = ENET_DUPLEX_FULL;
    Enetcli_configObj.speed = ENET_SPEED_1GBIT;
    Enetcli_configObj.macport = 1;

    /* This will set the configuration */
    config_status = Enetcli_phyModeCfg(writeBuffer,
                                       writeBufferLen,
                                       commandString);

    if (config_status == pdFALSE)
    {
        return config_status;
    }

    if (EnetApp_inst.macPort[Enetcli_configObj.macport] != ENET_MAC_PORT_INV)
    {
        status = Enetcli_closePortLink();
        if (status == ENET_SOK)
        {
            status = Enetcli_openPortLink();
        }
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Was not able to set the phy mode.\r\n");
        }
    }
    else
    {
        status = ENET_EFAIL;
        EnetAppUtils_print("Macport configuration is not correct. Please try again.\r\n");
    }

    if (status == ENET_SOK)
    {
        EnetAppUtils_print("Reset phy mode successfully.\r\n");
    }

    return pdFALSE;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static void EnetApp_macMode2MacMii(emac_mode macMode, EnetMacPort_Interface *pMii)
{
    switch (macMode)
    {
        case MII:
        {
            pMii->layerType    = ENET_MAC_LAYER_MII;
            pMii->sublayerType = ENET_MAC_SUBLAYER_STANDARD;
            pMii->variantType  = ENET_MAC_VARIANT_NONE;
            break;
        }
        case RMII:
        {
            pMii->layerType    = ENET_MAC_LAYER_MII;
            pMii->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
            pMii->variantType  = ENET_MAC_VARIANT_NONE;
            break;
        }
        case RGMII:
        {
            pMii->layerType    = ENET_MAC_LAYER_GMII;
            pMii->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
            pMii->variantType  = ENET_MAC_VARIANT_FORCED;
            break;
        }
        default:
        {
            EnetAppUtils_print("Invalid MAC mode: %u\r\n", macMode);
            EnetAppUtils_assert(false);
        }
    }
}

static int32_t EnetApp_macMode2PhyMii(emac_mode macMode,
                                    EnetPhy_Mii *mii)
{
    int32_t status = ENET_SOK;

    switch (macMode)
    {
        case MII:
            *mii = ENETPHY_MAC_MII_MII;
            break;
        case RMII:
            *mii = ENETPHY_MAC_MII_RMII;
            break;

        case RGMII:
            *mii = ENETPHY_MAC_MII_RGMII;
            break;
        default:
            status = ENET_EFAIL;
            EnetAppUtils_print("Invalid MAC mode: %u\r\n", macMode);
            EnetAppUtils_assert(false);
            break;
    }

    return status;
}

static BaseType_t Enetcli_phyModeCfg(char *writeBuffer,
                                         size_t writeBufferLen,
                                         const char *commandString)
{
    char *parameter;
    BaseType_t paramLen;
    uint32_t paramCnt = 1;
    bool isPortSet = false;
    bool isDuplexitySet = false;
    bool isSpeedSet = false;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString,
                                                 paramCnt,
                                                 &paramLen);

    while (parameter != NULL)
    {
        /* Checking for the -s parameter which is the speed of the phymode */
        if (strncmp("-s", parameter, paramLen) == 0)
        {
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString,
                                                         paramCnt+1,
                                                         &paramLen);
            if(parameter != NULL)
            {
                if(strncmp("10m", parameter, paramLen) == 0)
                {
                    Enetcli_configObj.speed = ENET_SPEED_10MBIT;
                }
                else if(strncmp("100m", parameter, paramLen) == 0)
                {
                    Enetcli_configObj.speed = ENET_SPEED_100MBIT;
                }
                else if(strncmp("1g", parameter, paramLen) == 0)
                {
                    Enetcli_configObj.speed = ENET_SPEED_1GBIT;
                }
                else if(strncmp("auto", parameter, paramLen) == 0)
                {
                    Enetcli_configObj.speed = ENET_SPEED_AUTO;
                }
                else
                {
                    snprintf(writeBuffer, writeBufferLen, "Invalid Parameter for -s\r\n");
                    return pdFALSE;
                }
                isSpeedSet = true;
                paramCnt += 2;
            }
            else
            {
                snprintf(writeBuffer, writeBufferLen, "No Parameter provided for -s\r\n");
                return pdFALSE;
            }
        }

        /* Checking for the -p parameter which is the macport number.*/
        else if (strncmp("-p", parameter, paramLen) == 0)
        {
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString,
                                                         paramCnt+1,
                                                         &paramLen);
            if(parameter != NULL)
            {
                uint8_t macPortNum = atoi(parameter);
                macPortNum = CPSW_ALE_ALEPORT_TO_MACPORT(macPortNum);
                if ((macPortNum >= ENET_MAC_PORT_1) && (macPortNum <= ENET_MAC_PORT_2))
                {
                    Enetcli_configObj.macport = macPortNum;
                    paramCnt += 2;
                }
                else
                {
                    snprintf(writeBuffer, writeBufferLen, "Invalid Parameter for -p (mac port number should be between 1 or 2 only)\r\n");
                    return pdFALSE;
                }
                isPortSet = true;
            }
            else
            {
                snprintf(writeBuffer, writeBufferLen, "No Parameter provided for -p\r\n");
                return pdFALSE;
            }
        }

        /* Checking for the -d parameter which is the duplexity. */
        else if (strncmp("-d", parameter, paramLen) == 0)
        {
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt+1,
            &paramLen);
            if(parameter != NULL)
            {
                if (strncmp("half", parameter, paramLen) == 0)
                {
                    Enetcli_configObj.duplexity = ENET_DUPLEX_HALF;
                }
                else if (strncmp("full", parameter, paramLen) == 0)
                {
                    Enetcli_configObj.duplexity = ENET_DUPLEX_FULL;
                }

                else if (strncmp("auto", parameter, paramLen) == 0)
                {
                    Enetcli_configObj.duplexity = ENET_DUPLEX_AUTO;
                }
                else
                {
                    snprintf(writeBuffer,
                             writeBufferLen,
                             "Invalid Parameter for -d\r\n");
                    return pdFALSE;
                }
                isDuplexitySet = true;
                paramCnt += 2;
            }
            else
            {
                snprintf(writeBuffer, writeBufferLen, "No Parameter provided for -d\r\n");
                return pdFALSE;
            }
        }
        else
        {
            snprintf(writeBuffer, writeBufferLen, "Invalid Parameter\r\n");
            return pdFALSE;
        }
        parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
        &paramLen);
    }

    if (isPortSet == false )
    {
        snprintf(writeBuffer,
                 writeBufferLen,
                 "Provide port number\r\n");
        return pdFALSE;
    }

    if (isDuplexitySet == false)
    {
        snprintf(writeBuffer,
                 writeBufferLen,
                 "Duplexity value not provided\r\n");
        return pdFALSE;
    }

    if (isSpeedSet == false)
    {
        snprintf(writeBuffer,
                 writeBufferLen,
                 "Speed value not provided\r\n");
        return pdFALSE;
    }

    if ((isDuplexitySet == true) && (isSpeedSet == true))
    {
        if((Enetcli_configObj.duplexity == ENET_DUPLEX_AUTO) && (Enetcli_configObj.speed != ENET_SPEED_AUTO))
        {
            snprintf(writeBuffer,
                     writeBufferLen,
                     "if Duplexity is auto, speed should be auto\r\n");
            return pdFALSE;
        }
        if((Enetcli_configObj.duplexity != ENET_DUPLEX_AUTO) && (Enetcli_configObj.speed == ENET_SPEED_AUTO))
        {
            snprintf(writeBuffer,
                     writeBufferLen,
                     "if Speed is auto, duplexity should be auto\r\n");
            return pdFALSE;
        }
    }
    return pdTRUE;
}
static int32_t Enetcli_openPortLink()
{
    /* Setup port link open parameters */
    EnetPer_PortLinkCfg portLinkCfg;
    EnetBoard_EthPort ethPort;
    EnetMacPort_LinkCfg *linkCfg = &portLinkCfg.linkCfg;
    EnetMacPort_Interface *mii = &portLinkCfg.mii;
    EnetPhy_Cfg *phyCfg = &portLinkCfg.phyCfg;
    CpswMacPort_Cfg macCfg;
    EnetPhy_Mii phyMii;
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;

    /* Following is the phy configuration parameters */
    const EnetBoard_PhyCfg *boardPhyCfg = NULL;

    /* Setup the Ethernet Port with correct instance*/
    ethPort.enetType = EnetApp_inst.enetType;
    ethPort.instId   = EnetApp_inst.instId;

    /* Setup board for requested Ethernet port */
    ethPort.macPort = EnetApp_inst.macPort[Enetcli_configObj.macport];
    ethPort.boardId = EnetApp_inst.boardId;
    EnetApp_macMode2MacMii(EnetApp_inst.macMode,
                            &ethPort.mii);

    status = EnetBoard_setupPorts( &ethPort,
                                    1U);
    EnetAppUtils_assert(status == ENET_SOK);

    /* Set port link params */
    portLinkCfg.macPort = EnetApp_inst.macPort[Enetcli_configObj.macport];
    portLinkCfg.macCfg = &macCfg;

    CpswMacPort_initCfg(&macCfg);
    EnetApp_macMode2MacMii(EnetApp_inst.macMode, mii);

    /* Set PHY configuration params */
    EnetPhy_initCfg(phyCfg);
    status = EnetApp_macMode2PhyMii(EnetApp_inst.macMode,
                                    &phyMii);


    if (status == ENET_SOK)
    {
        boardPhyCfg = EnetBoard_getPhyCfg(&ethPort);
        if (boardPhyCfg != NULL)
        {
            phyCfg->phyAddr = boardPhyCfg->phyAddr;
            phyCfg->loopbackEn = false;
            phyCfg->isStrapped = boardPhyCfg->isStrapped;
            phyCfg->skipExtendedCfg = boardPhyCfg->skipExtendedCfg;
            phyCfg->extendedCfgSize = boardPhyCfg->extendedCfgSize;
            memcpy(phyCfg->extendedCfg,
                    boardPhyCfg->extendedCfg,
                    phyCfg->extendedCfgSize);

        }
        else
        {
            EnetAppUtils_print("Port info not found\r\n");
            status = ENET_EFAIL;
        }
    }

    if(status == ENET_SOK)
    {
        linkCfg->speed = Enetcli_configObj.speed;
        linkCfg->duplexity = Enetcli_configObj.duplexity;
    }

    /* Open port link */
    if (status == ENET_SOK)
    {
        ENET_IOCTL_SET_IN_ARGS(&prms, &portLinkCfg);
        ENET_IOCTL(EnetApp_inst.hEnet,EnetApp_inst.coreId,ENET_PER_IOCTL_OPEN_PORT_LINK,&prms,status);

        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open port link: %d\r\n", status);
        }
    }

    return status;
}

static int32_t Enetcli_closePortLink()
{
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;

    /* Close port link */
    ENET_IOCTL_SET_IN_ARGS(&prms, &EnetApp_inst.macPort[Enetcli_configObj.macport]);
    ENET_IOCTL(EnetApp_inst.hEnet,EnetApp_inst.coreId,ENET_PER_IOCTL_CLOSE_PORT_LINK,&prms,status);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to close port link: %d\n", status);
    }
    return status;
}
