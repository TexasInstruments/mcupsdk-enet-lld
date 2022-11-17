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
 * \file  mdio_priv.h
 *
 * \brief This file contains private type definitions and helper macros for the
 *        MDIO port module which are meant for internal use in Enet Per drivers.
 */

#ifndef MDIO_IOCTL_PRIV_H_
#define MDIO_IOCTL_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_mod_mdio.h>
#include <priv/mod/mdio_priv.h>
#include <networking/enet/hw_include/mdio/csl_mdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
/*! \brief Max number of PHYs that can be monitored by MDIO in normal mode. */
#define MDIO_PHY_MONITOR_MAX                  (2U)

/*!
 * \brief Helper macro used to first register private IOCTL handler and then invoke the
 *        IOCTL
 */
#define MDIO_PRIV_IOCTL(hMdio, ioctlCmd,prms,status)                                           \
    do {                                                                                       \
        Enet_IoctlPrms regIoctlPrms;                                                           \
        Enet_IoctlRegisterHandlerInArgs regIoctlInArgs;                                        \
                                                                                               \
        regIoctlInArgs.cmd = ioctlCmd;                                                         \
        regIoctlInArgs.fxn = (uintptr_t)&Mdio_ioctl_handler_##ioctlCmd;                        \
                                                                                               \
        ENET_IOCTL_SET_IN_ARGS(&regIoctlPrms, &regIoctlInArgs);                                \
        status = EnetMod_ioctl(hMdio, MDIO_IOCTL_REGISTER_HANDLER, &regIoctlPrms);             \
        if (ENET_SOK == status)                                                                \
        {                                                                                      \
            status = EnetMod_ioctl(hMdio, ioctlCmd,prms);                                      \
        }                                                                                      \
    } while (0)


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */


/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_GET_VERSION(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_PRINT_REGS(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_IS_ALIVE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_IS_LINKED(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_IS_POLL_ENABLED(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C22_READ(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C22_WRITE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C45_READ(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C45_WRITE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_READ_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_READ_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_WRITE_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_WRITE_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_READ_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_READ_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_WRITE_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_WRITE_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_ioctl_handler_MDIO_IOCTL_HANDLE_INTR(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_ioctl_handler_ENET_MDIO_IOCTL_ENABLE_STATE_MACHINE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);


int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_GET_VERSION(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_PRINT_REGS(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_IS_ALIVE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_IS_LINKED(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_IS_POLL_ENABLED(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C22_READ(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C22_WRITE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C45_READ(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C45_WRITE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_READ_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_READ_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_WRITE_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_WRITE_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_READ_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_READ_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_WRITE_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_WRITE_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_normal_ioctl_handler_MDIO_IOCTL_HANDLE_INTR(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_normal_ioctl_handler_ENET_MDIO_IOCTL_ENABLE_STATE_MACHINE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);

int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_GET_VERSION(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_PRINT_REGS(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_IS_ALIVE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_IS_LINKED(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_IS_POLL_ENABLED(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C22_READ(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C22_WRITE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C45_READ(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C45_WRITE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_READ_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_READ_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_WRITE_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C22_ASYNC_WRITE_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_READ_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_READ_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_WRITE_TRIGGER(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_C45_ASYNC_WRITE_COMPLETE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_manual_ioctl_handler_MDIO_IOCTL_HANDLE_INTR(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);
int32_t Mdio_manual_ioctl_handler_ENET_MDIO_IOCTL_ENABLE_STATE_MACHINE(EnetMod_Handle hMod, CSL_mdioHandle mdioRegs, Enet_IoctlPrms *prms);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* MDIO_IOCTL_PRIV_H_ */
