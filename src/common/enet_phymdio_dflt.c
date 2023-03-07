/*
 *  Copyright (c) Texas Instruments Incorporated 2020-2023
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
 * \file  enet_phymdio_dflt.c
 *
 * \brief This file contains the default implementation of the MDIO interface
 *        of the Ethernet PHY (ENETPHY) driver with Enet LLD APIs.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <include/core/enet_utils.h>
#include <include/core/enet_per.h>
#include <priv/core/enet_trace_priv.h>
#include <include/phy/enetphy.h>
#include <include/core/enet_mod_mdio.h>
#include <include/core/enet_mod_phy.h>
#include <include/core/enet_mod_macport.h>
#include <include/common/enet_phymdio_dflt.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/mod/phy_priv.h>
#include <priv/mod/phy_ioctl_priv.h>
#include <priv/mod/mdio_ioctl_priv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ENET_PHY_MDIO_DEFAULT_IOCTL_HANDLER_ENTRY_INIT(x)    \
          {.cmd = x,                            \
           .fxn = &EnetPhyMdioDflt_ioctl_handler_##x}

#define ENET_PHY_MDIO_DEFAULT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(x)    \
          {.cmd = x,                            \
           .fxn = &EnetPhyMdioDflt_ioctl_handler_default}

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef int32_t (EnetPhyMdioDfltIoctlHandler)(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);

typedef struct EnetPhyMdioDfltIoctlHandlerRegistry_s
{
    uint32_t cmd;
    EnetPhyMdioDfltIoctlHandler *fxn;
} EnetPhyMdioDfltIoctlHandlerRegistry_t;



/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t EnetPhyMdioDflt_isAlive(uint32_t phyAddr,
                                bool *isAlive,
                                void *args);

int32_t EnetPhyMdioDflt_isLinked(uint32_t phyAddr,
                                 bool *isLinked,
                                 void *args);

int32_t EnetPhyMdioDflt_readC22(uint32_t group,
                                uint32_t phyAddr,
                                uint32_t reg,
                                uint16_t *val,
                                void *args);

int32_t EnetPhyMdioDflt_writeC22(uint32_t group,
                                 uint32_t phyAddr,
                                 uint32_t reg,
                                 uint16_t val,
                                 void *args);

int32_t EnetPhyMdioDflt_readC45(uint32_t group,
                                uint32_t phyAddr,
                                uint8_t mmd,
                                uint16_t reg,
                                uint16_t *val,
                                void *args);

int32_t EnetPhyMdioDflt_writeC45(uint32_t group,
                                 uint32_t phyAddr,
                                 uint8_t mmd,
                                 uint16_t reg,
                                 uint16_t val,
                                 void *args);

static int32_t EnetPhyMdioDflt_ioctl_handler_default(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
static int32_t EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_REGISTER_HANDLER(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms);
static int32_t EnetPhyMdioDflt_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                            EnetPhyMdioDfltIoctlHandler *ioctlHandlerFxn,
                                            EnetPhyMdioDfltIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                            uint32_t tableSize);
static EnetPhyMdioDfltIoctlHandler * EnetPhyMdioDflt_getIoctlHandlerFxn(uint32_t ioctlCmd, EnetPhyMdioDfltIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize);
static int32_t EnetPhyMdioDflt_getIoctlHandlerIdx(uint32_t ioctlCmd, EnetPhyMdioDfltIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static EnetPhy_Mdio gEnet_PhyMdioDflt =
{
    .isAlive  = EnetPhyMdioDflt_isAlive,
    .isLinked = EnetPhyMdioDflt_isLinked,
    .readC22  = EnetPhyMdioDflt_readC22,
    .writeC22 = EnetPhyMdioDflt_writeC22,
    .readC45  = EnetPhyMdioDflt_readC45,
    .writeC45 = EnetPhyMdioDflt_writeC45,
};

static EnetPhyMdioDfltIoctlHandlerRegistry_t EnetPhyMdioDfltIoctlHandlerRegistry[] = 
{
    ENET_PHY_MDIO_DEFAULT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PHY_IOCTL_GET_ID),
    ENET_PHY_MDIO_DEFAULT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PHY_IOCTL_GET_SUPPORTED_MODES),
    ENET_PHY_MDIO_DEFAULT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PHY_IOCTL_GET_LOOPBACK_STATE),
    ENET_PHY_MDIO_DEFAULT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PHY_IOCTL_IS_ALIVE),
    ENET_PHY_MDIO_DEFAULT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PHY_IOCTL_IS_LINKED),
    ENET_PHY_MDIO_DEFAULT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PHY_IOCTL_GET_LINK_MODE),
    ENET_PHY_MDIO_DEFAULT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PHY_IOCTL_RESET),
    ENET_PHY_MDIO_DEFAULT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PHY_IOCTL_READ_REG),
    ENET_PHY_MDIO_DEFAULT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PHY_IOCTL_WRITE_REG),
    ENET_PHY_MDIO_DEFAULT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PHY_IOCTL_READ_EXT_REG),
    ENET_PHY_MDIO_DEFAULT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PHY_IOCTL_WRITE_EXT_REG),
    ENET_PHY_MDIO_DEFAULT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PHY_IOCTL_C45_READ_REG),
    ENET_PHY_MDIO_DEFAULT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PHY_IOCTL_C45_WRITE_REG),
    ENET_PHY_MDIO_DEFAULT_IOCTL_HANDLER_ENTRY_INIT_DEFAULT(ENET_PHY_IOCTL_PRINT_REGS),
    ENET_PHY_MDIO_DEFAULT_IOCTL_HANDLER_ENTRY_INIT(ENET_PHY_IOCTL_REGISTER_HANDLER),
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

EnetPhy_MdioHandle EnetPhyMdioDflt_getPhyMdio(void)
{
    return &gEnet_PhyMdioDflt;
}

int32_t EnetPhyMdioDflt_isAlive(uint32_t phyAddr,
                                bool *isAlive,
                                void *args)
{
    EnetMod_Handle hMdio = ENET_MOD(args);
    Enet_IoctlPrms prms;
    bool isOpen;
    int32_t status = ENETPHY_SOK;

    if (hMdio == NULL)
    {
        ENETTRACE_ERR("PHY %u: Invalid MDIO handle\n", phyAddr);
        status = ENETPHY_EBADARGS;
    }

    if (status == ENET_SOK)
    {
        isOpen = EnetMod_isOpen(hMdio);
        if (!isOpen)
        {
            ENETTRACE_ERR("PHY %u: MDIO module is not open\n", phyAddr);
            status = ENETPHY_EFAIL;
        }
    }

    if (status == ENET_SOK)
    {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &phyAddr, isAlive);

        MDIO_PRIV_IOCTL(hMdio, ENET_MDIO_IOCTL_IS_ALIVE, &prms, status);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "PHY %u: Failed to get MDIO alive status: %d\n", phyAddr, status);
    }

    return status;
}

int32_t EnetPhyMdioDflt_isLinked(uint32_t phyAddr,
                                 bool *isLinked,
                                 void *args)
{
    EnetMod_Handle hMdio = ENET_MOD(args);
    Enet_IoctlPrms prms;
    bool isOpen;
    int32_t status = ENETPHY_SOK;

    if (hMdio == NULL)
    {
        ENETTRACE_ERR("PHY %u: Invalid MDIO handle\n", phyAddr);
        status = ENETPHY_EBADARGS;
    }

    if (status == ENET_SOK)
    {
        isOpen = EnetMod_isOpen(hMdio);
        if (!isOpen)
        {
            ENETTRACE_ERR("PHY %u: MDIO module is not open\n", phyAddr);
            status = ENETPHY_EFAIL;
        }
    }

    if (status == ENET_SOK)
    {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &phyAddr, isLinked);

        MDIO_PRIV_IOCTL(hMdio, ENET_MDIO_IOCTL_IS_LINKED, &prms, status);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "PHY %u: Failed to get link status: %d\n", phyAddr, status);
    }

    return status;
}

int32_t EnetPhyMdioDflt_readC22(uint32_t group,
                                uint32_t phyAddr,
                                uint32_t reg,
                                uint16_t *val,
                                void *args)
{
    EnetMod_Handle hMdio = ENET_MOD(args);
    Enet_IoctlPrms prms;
    EnetMdio_C22ReadInArgs inArgs;
    bool isOpen;
    int32_t status = ENETPHY_SOK;

    if (hMdio == NULL)
    {
        ENETTRACE_ERR("PHY %u: Invalid MDIO handle\n", phyAddr);
        status = ENETPHY_EBADARGS;
    }

    if (status == ENET_SOK)
    {
        isOpen = EnetMod_isOpen(hMdio);
        if (!isOpen)
        {
            ENETTRACE_ERR("PHY %u: MDIO module is not open\n", phyAddr);
            status = ENETPHY_EFAIL;
        }
    }

    if (status == ENET_SOK)
    {
        inArgs.group = (EnetMdio_Group)group;
        inArgs.phyAddr = phyAddr;
        inArgs.reg = reg;
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, val);

        MDIO_PRIV_IOCTL(hMdio, ENET_MDIO_IOCTL_C22_READ, &prms, status);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "PHY %u: Failed to read C22 reg: %d\n", phyAddr, status);
    }

    return status;
}

int32_t EnetPhyMdioDflt_writeC22(uint32_t group,
                                 uint32_t phyAddr,
                                 uint32_t reg,
                                 uint16_t val,
                                 void *args)
{
    EnetMod_Handle hMdio = ENET_MOD(args);
    Enet_IoctlPrms prms;
    EnetMdio_C22WriteInArgs inArgs;
    bool isOpen;
    int32_t status = ENETPHY_SOK;

    if (hMdio == NULL)
    {
        ENETTRACE_ERR("PHY %u: Invalid MDIO handle\n", phyAddr);
        status = ENETPHY_EBADARGS;
    }

    if (status == ENET_SOK)
    {
        isOpen = EnetMod_isOpen(hMdio);
        if (!isOpen)
        {
            ENETTRACE_ERR("PHY %u: MDIO module is not open\n", phyAddr);
            status = ENETPHY_EFAIL;
        }
    }

    if (status == ENET_SOK)
    {
        inArgs.group = (EnetMdio_Group)group;
        inArgs.phyAddr = phyAddr;
        inArgs.reg = reg;
        inArgs.val = val;
        ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);

        MDIO_PRIV_IOCTL(hMdio, ENET_MDIO_IOCTL_C22_WRITE, &prms, status);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "PHY %u: Failed to write C22 reg: %d\n", phyAddr, status);
    }

    return status;
}

int32_t EnetPhyMdioDflt_readC45(uint32_t group,
                                uint32_t phyAddr,
                                uint8_t mmd,
                                uint16_t reg,
                                uint16_t *val,
                                void *args)
{
    EnetMod_Handle hMdio = ENET_MOD(args);
    Enet_IoctlPrms prms;
    EnetMdio_C45ReadInArgs inArgs;
    bool isOpen;
    int32_t status = ENETPHY_SOK;

    if (hMdio == NULL)
    {
        ENETTRACE_ERR("PHY %u: Invalid MDIO handle\n", phyAddr);
        status = ENETPHY_EBADARGS;
    }

    if (status == ENET_SOK)
    {
        isOpen = EnetMod_isOpen(hMdio);
        if (!isOpen)
        {
            ENETTRACE_ERR("PHY %u: MDIO module is not open\n", phyAddr);
            status = ENETPHY_EFAIL;
        }
    }

    if (status == ENET_SOK)
    {
        inArgs.group = (EnetMdio_Group)group;
        inArgs.phyAddr = phyAddr;
        inArgs.mmd = (EnetMdio_C45Mmd)mmd;
        inArgs.reg = reg;
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, val);

        MDIO_PRIV_IOCTL(hMdio, ENET_MDIO_IOCTL_C45_READ, &prms, status);
        ENETTRACE_ERR_IF(status != ENET_SOK,
                         "PHY %u: Failed to read C45 reg: %d\n", phyAddr, status);
    }

    return status;
}

int32_t EnetPhyMdioDflt_writeC45(uint32_t group,
                                 uint32_t phyAddr,
                                 uint8_t mmd,
                                 uint16_t reg,
                                 uint16_t val,
                                 void *args)
{
    EnetMod_Handle hMdio = ENET_MOD(args);
    Enet_IoctlPrms prms;
    EnetMdio_C45WriteInArgs inArgs;
    bool isOpen;
    int32_t status = ENETPHY_SOK;

    if (hMdio == NULL)
    {
        ENETTRACE_ERR("PHY %u: Invalid MDIO handle\n", phyAddr);
        status = ENETPHY_EBADARGS;
    }

    if (status == ENET_SOK)
    {
        isOpen = EnetMod_isOpen(hMdio);
        if (!isOpen)
        {
            ENETTRACE_ERR("PHY %u: MDIO module is not open\n", phyAddr);
            status = ENETPHY_EFAIL;
        }
    }

    if (status == ENET_SOK)
    {
        inArgs.group = (EnetMdio_Group)group;
        inArgs.phyAddr = phyAddr;
        inArgs.mmd = (EnetMdio_C45Mmd)mmd;
        inArgs.reg = reg;
        inArgs.val = val;
        ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);

        MDIO_PRIV_IOCTL(hMdio, ENET_MDIO_IOCTL_C45_WRITE, &prms, status);
        ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to write C45 reg: %d\n", status);
    }

    return status;
}

int32_t EnetPhyMdioDflt_ioctl(EnetPhy_Handle hPhy,
                              uint32_t cmd,
                              Enet_IoctlPrms *prms)
{
    EnetPhyMdioDfltIoctlHandler* ioctlHandlerFxn =
            EnetPhyMdioDflt_getIoctlHandlerFxn(cmd, EnetPhyMdioDfltIoctlHandlerRegistry, ENET_ARRAYSIZE(EnetPhyMdioDfltIoctlHandlerRegistry));
    Enet_devAssert(ioctlHandlerFxn != NULL);
    const int32_t status = ioctlHandlerFxn(hPhy, prms);

    return status;
}

static int32_t EnetPhyMdioDflt_getIoctlHandlerIdx(uint32_t ioctlCmd, EnetPhyMdioDfltIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize, uint32_t *tblIdx)
{
    int32_t status;
    uint32_t i;

    for (i = 0; i < tableSize; i++)
    {
        if (ioctlRegistryTbl[i].cmd == ioctlCmd)
        {
            break;
        }
    }
    if (i < tableSize)
    {
        *tblIdx = i;
        status = ENET_SOK;
    }
    else
    {
        status = ENET_EINVALIDPARAMS;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get ioctl registry index for ioctl cmd: %x\n", ioctlCmd);
    return status;
}

static EnetPhyMdioDfltIoctlHandler * EnetPhyMdioDflt_getIoctlHandlerFxn(uint32_t ioctlCmd, EnetPhyMdioDfltIoctlHandlerRegistry_t *ioctlRegistryTbl, uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;
    EnetPhyMdioDfltIoctlHandler *handlerFxn = NULL;

    status = EnetPhyMdioDflt_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert(tblIdx < tableSize, "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        handlerFxn = ioctlRegistryTbl[tblIdx].fxn;
    }
    else
    {
        handlerFxn = &EnetPhyMdioDflt_ioctl_handler_default;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to get ioctl handler for ioctl cmd: %x. Using default ioctl handler \r\n", ioctlCmd);
    return handlerFxn;
}

static int32_t EnetPhyMdioDflt_setIoctlHandlerFxn(uint32_t ioctlCmd,
                                            EnetPhyMdioDfltIoctlHandler *ioctlHandlerFxn,
                                            EnetPhyMdioDfltIoctlHandlerRegistry_t *ioctlRegistryTbl,
                                            uint32_t tableSize)
{
    int32_t status;
    uint32_t tblIdx;

    status = EnetPhyMdioDflt_getIoctlHandlerIdx(ioctlCmd, ioctlRegistryTbl, tableSize, &tblIdx);
    if (ENET_SOK == status)
    {
        Enet_devAssert((tblIdx < tableSize), "Invalid IOCTL registry table index for IOCTL cmd:%x", ioctlCmd);
        Enet_devAssert((ioctlCmd == ioctlRegistryTbl[tblIdx].cmd), "Ioctl table corrupted");
        ioctlRegistryTbl[tblIdx].fxn = ioctlHandlerFxn;
    }
    ENETTRACE_ERR_IF(status != ENET_SOK, "Failed to set ioctl handler for ioctl cmd: %x \r\n", ioctlCmd);
    return status;
}

static int32_t EnetPhyMdioDflt_ioctl_handler_ENET_PHY_IOCTL_REGISTER_HANDLER(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    const Enet_IoctlRegisterHandlerInArgs *inArgs = (const Enet_IoctlRegisterHandlerInArgs *)prms->inArgs;
    int32_t status;

    status = EnetPhyMdioDflt_setIoctlHandlerFxn(inArgs->cmd, 
                                               (EnetPhyMdioDfltIoctlHandler *)inArgs->fxn, 
                                                EnetPhyMdioDfltIoctlHandlerRegistry, 
                                                ENET_ARRAYSIZE(EnetPhyMdioDfltIoctlHandlerRegistry));
    return status;
}


static int32_t EnetPhyMdioDflt_ioctl_handler_default(EnetPhy_Handle hPhy, Enet_IoctlPrms *prms)
{
    return ENET_ENOTSUPPORTED;
}



