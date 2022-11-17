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
 * \file  enet_mod_fdb.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet Forwarding Database (FDB) module interface.
 */

/*!
 * \ingroup  DRV_ENET_MODULE
 * \defgroup ENET_MOD_FDB Enet Forwarding Database (FDB)
 *
 * @{
 */

#ifndef ENET_MOD_FDB_H_
#define ENET_MOD_FDB_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_mod.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Helper macro to create IOCTL commands for FDB module. */
#define ENET_FDB_PUBLIC_IOCTL(x)              (ENET_IOCTL_TYPE_PUBLIC | \
                                               ENET_IOCTL_FDB_BASE |    \
                                               ENET_IOCTL_MIN(x))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief FDB IOCTL commands.
 *
 * FDB IOCTLs is a reduced set of commands as forwarding database hardware
 * modules are significantly different that it's more feasible to expose
 * dedicated, module-specific IOCTL commands in module-specific drivers.
 *
 * Refer to CPSW ALE, ICSSG FDB or the FDB module documentation present in the
 * Ethernet peripheral of interest.
 */
typedef enum EnetFdb_Ioctl_e
{
    /*!
     * \brief Get the hardware version of the FDB module.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: #Enet_Version
     */
    ENET_FDB_IOCTL_GET_VERSION = ENET_FDB_PUBLIC_IOCTL(0U),

    /*!
     * \brief Print FDB registers.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: None
     */
    ENET_FDB_IOCTL_PRINT_REGS = ENET_FDB_PUBLIC_IOCTL(1U),
} EnetFdb_Ioctl;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

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

#endif /* ENET_MOD_FDB_H_ */

/*! @} */
