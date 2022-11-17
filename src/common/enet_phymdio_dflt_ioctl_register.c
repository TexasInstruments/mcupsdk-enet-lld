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
 * \file  enet_phymdio_dflt_ioctl_register.c
 *
 * \brief This file contains the default implementation of the MDIO interface
 *        of the Ethernet PHY (ENETPHY) driver with Enet LLD APIs.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <include/core/enet_base.h>
#include <include/phy/enetphy.h>
#include <include/core/enet_mod_mdio.h>
#include <include/core/enet_mod_phy.h>
#include <include/core/enet_mod_macport.h>
#include <include/common/enet_phymdio_dflt.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/core/enet_base_priv.h>
#include <priv/mod/phy_priv.h>
#include <priv/mod/phy_ioctl_priv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define ENET_PHY_MDIO_DEFAULT_GEN_REGISTER_IOCTL_HANDLER_FXN(x)                       \
int32_t Enet_ioctl_register_##x(Enet_Handle hEnet, uint32_t coreId)                   \
                                                                                      \
{                                                                                     \
    int32_t  status;                                                                  \
    Enet_IoctlPrms prms;                                                              \
    Enet_IoctlRegisterHandlerInArgs inArgs;                                           \
                                                                                      \
    inArgs.cmd = x;                                                                   \
    inArgs.fxn = (uintptr_t)&EnetPhyMdioDflt_ioctl_handler_##x;                       \
                                                                                      \
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);                                           \
    status = Enet_ioctl(hEnet, coreId, ENET_PER_IOCTL_REGISTER_IOCTL_HANDLER, &prms); \
    return  status;                                                                   \
                                                                                      \
}


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
ENET_PHY_MDIO_DEFAULT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PHY_IOCTL_GET_ID)
ENET_PHY_MDIO_DEFAULT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PHY_IOCTL_GET_SUPPORTED_MODES)
ENET_PHY_MDIO_DEFAULT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PHY_IOCTL_GET_LOOPBACK_STATE)
ENET_PHY_MDIO_DEFAULT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PHY_IOCTL_IS_ALIVE)
ENET_PHY_MDIO_DEFAULT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PHY_IOCTL_IS_LINKED)
ENET_PHY_MDIO_DEFAULT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PHY_IOCTL_GET_LINK_MODE)
ENET_PHY_MDIO_DEFAULT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PHY_IOCTL_RESET)
ENET_PHY_MDIO_DEFAULT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PHY_IOCTL_READ_REG)
ENET_PHY_MDIO_DEFAULT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PHY_IOCTL_WRITE_REG)
ENET_PHY_MDIO_DEFAULT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PHY_IOCTL_READ_EXT_REG)
ENET_PHY_MDIO_DEFAULT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PHY_IOCTL_WRITE_EXT_REG)
ENET_PHY_MDIO_DEFAULT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PHY_IOCTL_C45_READ_REG)
ENET_PHY_MDIO_DEFAULT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PHY_IOCTL_C45_WRITE_REG)
ENET_PHY_MDIO_DEFAULT_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_PHY_IOCTL_PRINT_REGS)


