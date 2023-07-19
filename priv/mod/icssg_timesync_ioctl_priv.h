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
 * \file  icssg_timesync_ioctl_priv.h
 *
 * \brief This file contains the private type definitions and helper macros for
 *        the ICSSG TimeSync driver.
 */

#ifndef ICSSG_TIMESYNC_IOCTL_PRIV_H_
#define ICSSG_TIMESYNC_IOCTL_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>

#include <include/mod/icssg_timesync.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/* None */

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
int32_t  IcssgTimeSync_ioctl_handler_ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP(EnetMod_Handle hMod,
                                                                                uint32_t cmd,
                                                                                Enet_IoctlPrms *prms);

int32_t  IcssgTimeSync_ioctl_handler_ENET_TIMESYNC_IOCTL_SET_TIMESTAMP(EnetMod_Handle hMod,
                                                                        uint32_t cmd,
                                                                        Enet_IoctlPrms *prms);

int32_t  IcssgTimeSync_ioctl_handler_ENET_TIMESYNC_IOCTL_ADJUST_TIMESTAMP(EnetMod_Handle hMod,
                                                                            uint32_t cmd,
                                                                            Enet_IoctlPrms *prms);

int32_t  IcssgTimeSync_ioctl_handler_ENET_TIMESYNC_IOCTL_SET_TIMESTAMP_COMPLETE(EnetMod_Handle hMod,
                                             uint32_t cmd,
                                             Enet_IoctlPrms *prms);

int32_t IcssgTimeSync_ioctl_handler_ENET_TIMESYNC_IOCTL_GET_ETH_TX_TIMESTAMP(EnetMod_Handle hMod,
                                                                             uint32_t cmd,
                                                                             Enet_IoctlPrms *prms);

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

#endif /* ICSSG_TIMESYNC_IOCTL_PRIV_H_ */
