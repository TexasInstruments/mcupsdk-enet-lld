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
 * \file  mdio.c
 *
 * \brief This file contains the implementation of the MDIO module.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>
#include <csl_cpswitch.h>
#include <enet_cfg.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_soc.h>
#include <include/mod/mdio.h>
#include <priv/core/enet_trace_priv.h>
#include <priv/core/enet_base_priv.h>
#include <priv/mod/mdio_priv.h>
#include <priv/mod/mdio_ioctl_priv.h>
#include <priv/mod/cpsw_clks.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(x)                                        \
int32_t Enet_ioctl_register_##x(Enet_Handle hEnet, uint32_t coreId)                   \
                                                                                      \
{                                                                                     \
    int32_t  status;                                                                  \
    Enet_IoctlPrms prms;                                                              \
    Enet_IoctlRegisterHandlerInArgs inArgs;                                           \
                                                                                      \
    inArgs.cmd = x;                                                                   \
    inArgs.fxn = (uintptr_t)&Mdio_ioctl_handler_##x;                                  \
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
MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MDIO_IOCTL_GET_VERSION)
MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MDIO_IOCTL_PRINT_REGS)
MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MDIO_IOCTL_IS_ALIVE)
MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MDIO_IOCTL_IS_LINKED)
MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MDIO_IOCTL_IS_POLL_ENABLED)
MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MDIO_IOCTL_C22_READ)
MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MDIO_IOCTL_C22_WRITE)
MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MDIO_IOCTL_C45_READ)
MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MDIO_IOCTL_C45_WRITE)
MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MDIO_IOCTL_C22_ASYNC_READ_TRIGGER)
MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MDIO_IOCTL_C22_ASYNC_READ_COMPLETE)
MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MDIO_IOCTL_C22_ASYNC_WRITE_TRIGGER)
MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MDIO_IOCTL_C22_ASYNC_WRITE_COMPLETE)
MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MDIO_IOCTL_C45_ASYNC_READ_TRIGGER)
MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MDIO_IOCTL_C45_ASYNC_READ_COMPLETE)
MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MDIO_IOCTL_C45_ASYNC_WRITE_TRIGGER)
MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MDIO_IOCTL_C45_ASYNC_WRITE_COMPLETE)
MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(MDIO_IOCTL_HANDLE_INTR)
MDIO_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_MDIO_IOCTL_ENABLE_STATE_MACHINE)



