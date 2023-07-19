/*
 *  Copyright (c) Texas Instruments Incorporated 2021
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
 * \file  icssg_timesync_ioctl_register.c
 *
 * \brief This file contains the implementation of the ICSSG TimeSync ioctl Handler register.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdlib.h>
#include <hw_include/cslr_icss.h>
#include <drivers/hw_include/hw_types.h>
#include <include/core/enet_base.h>
#include <include/core/enet_utils.h>
#include <include/core/enet_mod.h>
#include <include/per/icssg.h>
#include <src/per/icssg_utils.h>
#include <priv/core/enet_base_priv.h>
#include <priv/mod/icssg_timesync_priv.h>
#include <priv/mod/icssg_timesync_ioctl_priv.h>
#include <src/per/firmware/icssg/fw_mem_map.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ICSSG_TIMESYNC_GEN_REGISTER_IOCTL_HANDLER_FXN(x)                                    \
int32_t Enet_ioctl_register_##x(Enet_Handle hEnet, uint32_t coreId)                   \
                                                                                      \
{                                                                                     \
    int32_t  status;                                                                  \
    Enet_IoctlPrms prms;                                                              \
    Enet_IoctlRegisterHandlerInArgs inArgs;                                           \
                                                                                      \
    inArgs.cmd = x;                                                                   \
    inArgs.fxn = (uintptr_t)&IcssgTimeSync_ioctl_handler_##x;                               \
                                                                                      \
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);                                           \
    status = Enet_ioctl(hEnet, coreId, ENET_PER_IOCTL_REGISTER_IOCTL_HANDLER, &prms); \
    return  status;                                                                   \
                                                                                      \
}/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
ICSSG_TIMESYNC_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP)
ICSSG_TIMESYNC_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_TIMESYNC_IOCTL_SET_TIMESTAMP)
ICSSG_TIMESYNC_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_TIMESYNC_IOCTL_ADJUST_TIMESTAMP)
ICSSG_TIMESYNC_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_TIMESYNC_IOCTL_SET_TIMESTAMP_COMPLETE)
ICSSG_TIMESYNC_GEN_REGISTER_IOCTL_HANDLER_FXN(ENET_TIMESYNC_IOCTL_GET_ETH_TX_TIMESTAMP)
