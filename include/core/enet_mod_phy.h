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
 * \file  enet_mod_phy.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Ethernet PHY interface.
 */

#ifndef ENET_MOD_PHY_H_
#define ENET_MOD_PHY_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_mod.h>
#include <include/phy/enetphy.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Helper macro to create PHY IOCTL commands. */
#define ENET_PHY_PUBLIC_IOCTL(x)              (ENET_IOCTL_TYPE_PUBLIC | \
                                               ENET_IOCTL_PHY_BASE |    \
                                               ENET_IOCTL_MIN(x))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief PHY IOCTL commands.
 */
enum EnetPhy_Ioctl_e
{
    /*!
     * \brief Get PHY identification.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_GenericInArgs
     * - outArgs: #EnetPhy_Version
     */
    ENET_PHY_IOCTL_GET_ID = ENET_PHY_PUBLIC_IOCTL(0U),

    /*!
     * \brief Get PHY supported modes by local PHY device.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_GenericInArgs
     * - outArgs: uint32_t
     */
    ENET_PHY_IOCTL_GET_SUPPORTED_MODES = ENET_PHY_PUBLIC_IOCTL(1U),

    /*!
     * \brief Check if PHY is in loopback or not.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_GenericInArgs
     * - outArgs: bool
     */
    ENET_PHY_IOCTL_GET_LOOPBACK_STATE = ENET_PHY_PUBLIC_IOCTL(2U),

    /*!
     * \brief Check PHY alive status.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_GenericInArgs
     * - outArgs: bool
     */
    ENET_PHY_IOCTL_IS_ALIVE = ENET_PHY_PUBLIC_IOCTL(3U),

    /*!
     * \brief Check state-machine link state, that is, whether the state machine
     *        has reached the LINKED state.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_GenericInArgs
     * - outArgs: bool
     */
    ENET_PHY_IOCTL_IS_LINKED = ENET_PHY_PUBLIC_IOCTL(4U),

    /*!
     * \brief Get the link speed and duplexity state after the state machine
     *        has reached the LINKED state.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_GenericInArgs
     * - outArgs: #EnetMacPort_LinkCfg
     */
    ENET_PHY_IOCTL_GET_LINK_MODE = ENET_PHY_PUBLIC_IOCTL(5U),

    /*!
     * \brief Reset PHY.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_GenericInArgs
     * - outArgs: None
     */
    ENET_PHY_IOCTL_RESET = ENET_PHY_PUBLIC_IOCTL(6U),

    /*!
     * \brief Read PHY register.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_ReadRegInArgs
     * - outArgs: uint16_t
     */
    ENET_PHY_IOCTL_READ_REG = ENET_PHY_PUBLIC_IOCTL(7U),

    /*!
     * \brief Write PHY register.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_WriteRegInArgs
     * - outArgs: None
     */
    ENET_PHY_IOCTL_WRITE_REG = ENET_PHY_PUBLIC_IOCTL(8U),

    /*!
     * \brief Read PHY extended register.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_ReadRegInArgs
     * - outArgs: uint16_t
     */
    ENET_PHY_IOCTL_READ_EXT_REG = ENET_PHY_PUBLIC_IOCTL(9U),

    /*!
     * \brief Write PHY extended register.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_WriteRegInArgs
     * - outArgs: None
     */
    ENET_PHY_IOCTL_WRITE_EXT_REG = ENET_PHY_PUBLIC_IOCTL(10U),

    /*!
     * \brief Read PHY register using Clause-45 frame.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_C45ReadRegInArgs
     * - outArgs: uint16_t
     */
    ENET_PHY_IOCTL_C45_READ_REG = ENET_PHY_PUBLIC_IOCTL(11U),

    /*!
     * \brief Write PHY register using Clause-45 frame.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_C45WriteRegInArgs
     * - outArgs: None
     */
    ENET_PHY_IOCTL_C45_WRITE_REG = ENET_PHY_PUBLIC_IOCTL(12U),

    /*!
     * \brief Print PHY registers.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetPhy_GenericInArgs
     * - outArgs: None
     */
    ENET_PHY_IOCTL_PRINT_REGS = ENET_PHY_PUBLIC_IOCTL(13U),
};

/*!
 * \brief Generic input args.
 */
typedef struct EnetPhy_GenericInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;
} EnetPhy_GenericInArgs;

/*!
 * \brief Input args for #ENET_PHY_IOCTL_READ_EXT_REG command.
 */
typedef struct EnetPhy_ReadRegInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! Register address */
    uint16_t reg;
} EnetPhy_ReadRegInArgs;

/*!
 * \brief Input args for #ENET_PHY_IOCTL_WRITE_EXT_REG command.
 */
typedef struct EnetPhy_WriteRegInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! Register address */
    uint16_t reg;

    /*! Value to be written */
    uint16_t val;
} EnetPhy_WriteRegInArgs;

/*!
 * \brief Input args for #ENET_PHY_IOCTL_C45_READ_REG command.
 */
typedef struct EnetPhy_C45ReadRegInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! MMD */
    uint8_t mmd;

    /*! Register address */
    uint16_t reg;
} EnetPhy_C45ReadRegInArgs;

/*!
 * \brief Input args for #ENET_PHY_IOCTL_C45_WRITE_REG command.
 */
typedef struct EnetPhy_C45WriteRegInArgs_s
{
    /*! Port number */
    Enet_MacPort macPort;

    /*! MMD */
    uint8_t mmd;

    /*! Register address */
    uint16_t reg;

    /*! Value to be written */
    uint16_t val;
} EnetPhy_C45WriteRegInArgs;

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

#endif /* ENET_MOD_PHY_H_ */
