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
 * \file  enet_mod_mdio.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet MDIO module.
 */

/*!
 * \ingroup  DRV_ENET_MODULE
 * \defgroup ENET_MOD_MDIO Enet Management Data I/O (MDIO)
 *
 * @{
 */

#ifndef ENET_MOD_MDIO_H_
#define ENET_MOD_MDIO_H_

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

/*! \brief MDIO feature mask for Clause-45 support. */
#define ENET_MDIO_FEAT_CLAUSE45               (ENET_BIT(0U))

/*! \brief MDIO feature mask for PHY state change monitoring. */
#define ENET_MDIO_FEAT_PHY_MONITOR            (ENET_BIT(1U))

/*! \brief Helper macro to create IOCTL commands for MDIO module. */
#define ENET_MDIO_PUBLIC_IOCTL(x)             (ENET_IOCTL_TYPE_PUBLIC | \
                                               ENET_IOCTL_MDIO_BASE |   \
                                               ENET_IOCTL_MIN(x))

/*! \brief Helper macro to create private IOCTL commands for MDIO module. */
#define ENET_MDIO_PRIVATE_IOCTL(x)             (ENET_IOCTL_TYPE_PRIVATE | \
                                               ENET_IOCTL_MDIO_BASE |   \
                                               ENET_IOCTL_MIN(x))

/*! \brief Create a MDIO PHY mask from a PHY address. */
#define ENET_MDIO_PHY_ADDR_MASK(addr)         (ENET_BIT(addr))

/*! \brief Check if the corresponding PHY address mask is set */
#define ENET_MDIO_IS_PHY_ADDR_SET(mask, addr) (((mask) & ENET_BIT(addr)) != 0U)

/*! \brief MDIO PHY address mask for no PHYs present. */
#define ENET_MDIO_PHY_ADDR_MASK_NONE          (0x00000000U)

/*! \brief MDIO PHY address mask for all PHYs present. */
#define ENET_MDIO_PHY_ADDR_MASK_ALL           (0xFFFFFFFFU)

/*! \brief Maximum number of PHYs supported on the MDIO bus. */
#define ENET_MDIO_PHY_CNT_MAX                 (31U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief MDIO IOCTL commands.
 */
typedef enum EnetMdio_Ioctl_e
{
    /*!
     * \brief Get the hardware version of the MDIO module.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: #Enet_Version
     */
    ENET_MDIO_IOCTL_GET_VERSION = ENET_MDIO_PUBLIC_IOCTL(0U),

    /*!
     * \brief Print MDIO registers.
     *
     * IOCTL parameters:
     * -  inArgs: None
     * - outArgs: None
     */
    ENET_MDIO_IOCTL_PRINT_REGS = ENET_MDIO_PUBLIC_IOCTL(1U),

    /*!
     * \brief Get PHY alive status.
     *
     * IOCTL parameters:
     * -  inArgs: uint8_t
     * - outArgs: bool
     */
    ENET_MDIO_IOCTL_IS_ALIVE = ENET_MDIO_PUBLIC_IOCTL(2U),

    /*!
     * \brief Get PHY link status.
     *
     * IOCTL parameters:
     * -  inArgs: uint8_t
     * - outArgs: bool
     */
    ENET_MDIO_IOCTL_IS_LINKED = ENET_MDIO_PUBLIC_IOCTL(3U),

    /*!
     * \brief Get link state change poll enable status.
     *
     * Checks if PHY state change is being monitored for the given PHY
     * address regardless of the underlying monitoring mechanism or mode.
     *
     * IOCTL parameters:
     * -  inArgs: uint8_t
     * - outArgs: bool
     */
    ENET_MDIO_IOCTL_IS_POLL_ENABLED = ENET_MDIO_PUBLIC_IOCTL(4U),

    /*!
     * \brief Read a PHY register using clause-22 frame.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMdio_C22ReadInArgs
     * - outArgs: uint16_t
     */
    ENET_MDIO_IOCTL_C22_READ = ENET_MDIO_PUBLIC_IOCTL(5U),

    /*!
     * \brief Write a PHY register using clause-22 frame.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMdio_C22WriteInArgs
     * - outArgs: None
     */
    ENET_MDIO_IOCTL_C22_WRITE = ENET_MDIO_PUBLIC_IOCTL(6U),

    /*!
     * \brief Read a PHY register using clause-45 frame.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMdio_C45ReadInArgs
     * - outArgs: uint16_t
     */
    ENET_MDIO_IOCTL_C45_READ = ENET_MDIO_PUBLIC_IOCTL(7U),

    /*!
     * \brief Write a PHY register using clause-45 frame.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMdio_C45WriteInArgs
     * - outArgs: None
     */
    ENET_MDIO_IOCTL_C45_WRITE = ENET_MDIO_PUBLIC_IOCTL(8U),

    /*!
     * \brief Trigger Asynchronous read to PHY register
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMdio_C45ReadInArgs
     * - outArgs: None
     */
    ENET_MDIO_IOCTL_C22_ASYNC_READ_TRIGGER = ENET_MDIO_PUBLIC_IOCTL(9U),

        /*!
     * \brief Checks for async read completion to PHY register
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMdio_C45ReadInArgs
     * - outArgs: None
     */
    ENET_MDIO_IOCTL_C22_ASYNC_READ_COMPLETE = ENET_MDIO_PUBLIC_IOCTL(10U),

        /*!
     * \brief Trigger Asynchronous Write to PHY register
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMdio_C45WriteInArgs
     * - outArgs: None
     */
    ENET_MDIO_IOCTL_C22_ASYNC_WRITE_TRIGGER = ENET_MDIO_PUBLIC_IOCTL(11U),

        /*!
     * \brief Checks for async Write completion to PHY register
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMdio_C45WriteInArgs
     * - outArgs: None
     */
    ENET_MDIO_IOCTL_C22_ASYNC_WRITE_COMPLETE = ENET_MDIO_PUBLIC_IOCTL(12U),

        /*!
     * \brief  Trigger Asynchronous Read to PHY register using clause-45 frame.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMdio_C45ReadInArgs
     * - outArgs: uint16_t
     */
    ENET_MDIO_IOCTL_C45_ASYNC_READ_TRIGGER = ENET_MDIO_PUBLIC_IOCTL(13U),

            /*!
     * \brief Checks for async Read completion to PHY register using clause-45 frame.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMdio_C45ReadInArgs
     * - outArgs: uint16_t
     */
    ENET_MDIO_IOCTL_C45_ASYNC_READ_COMPLETE = ENET_MDIO_PUBLIC_IOCTL(14U),

    /*!
     * \brief  Trigger Asynchronous Write to PHY register using clause-45 frame.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMdio_C45WriteInArgs
     * - outArgs: None
     */
    ENET_MDIO_IOCTL_C45_ASYNC_WRITE_TRIGGER = ENET_MDIO_PUBLIC_IOCTL(15U),

    /*!
     * \brief Checks for async Write completion to PHY register using clause-45 frame.
     *
     * IOCTL parameters:
     * -  inArgs: #EnetMdio_C45WriteInArgs
     * - outArgs: None
     */
    ENET_MDIO_IOCTL_C45_ASYNC_WRITE_COMPLETE = ENET_MDIO_PUBLIC_IOCTL(16U),

    /*!
     * \brief Enable MDIO state machine. Use
     *
     * If Mdio_Cfg.disableStateMachineOnInit is set, this IOCTL allows
     * enabling Mdio state machine via IOCTL. This is used for
     * sequencing external PHY management operation with MDIO state
     * machine so that any link interrupts are not missed and MDIO
     * state machine is enabled only after PHY initialization is complete
     * IOCTL parameters:
     * -  inArgs: None
     * -  outArgs: None
     */

    ENET_MDIO_IOCTL_ENABLE_STATE_MACHINE = ENET_MDIO_PUBLIC_IOCTL(17U),

} EnetMdio_Ioctl;

/*!
 * \brief MDIO user group.
 */
typedef enum EnetMdio_Group_e
{
    /*! Group 0 */
    ENET_MDIO_GROUP_0 = 0U,

    /*! Group 1 */
    ENET_MDIO_GROUP_1,

    /*! Number of groups */
    ENET_MDIO_GROUP_NUM,
} EnetMdio_Group;

/*!
 * \brief Frame format.
 */
typedef enum EnetMdio_FrameFmt_s
{
    /*! Clause 22 frame format */
    ENET_MDIO_FRAME_FMT_C22 = 0U,

    /*! Clause 45 frame format */
    ENET_MDIO_FRAME_FMT_C45,
} EnetMdio_FrameFmt;

/*!
 * \brief Clause-45 MDIO Manageable Device (MMD) addresses.
 */
typedef enum EnetMdio_C45Mmd_e
{
    /*! PMA/PMD */
    ENET_MDIO_MMD_PMA_PMD  = 1U,

    /*! WIS */
    ENET_MDIO_MMD_WIS      = 2U,

    /*! PCS */
    ENET_MDIO_MMD_PCS      = 3U,

    /*! PHY XS */
    ENET_MDIO_MMD_PHY_XS   = 4U,

    /*! DTE XS */
    ENET_MDIO_MMD_DTE_XS   = 5U,

    /*! Vendor specific 1 */
    ENET_MDIO_MMD_VENDOR_1 = 30U,

    /*! Vendor specific 2 */
    ENET_MDIO_MMD_VENDOR_2 = 31U,
} EnetMdio_C45Mmd;

/*!
 * \brief Input args for #ENET_MDIO_IOCTL_C22_READ command.
 *
 * Clause-22 register read arguments.
 */
typedef struct EnetMdio_C22ReadInArgs_s
{
    /*! User channel to be used to perform the register access.
     * Simultaneous access from same group is not allowed.
     * Not applicable in manual mode.
     */
    EnetMdio_Group group;

    /*! PHY address */
    uint32_t phyAddr;

    /*! Register address */
    uint16_t reg;
} EnetMdio_C22ReadInArgs;

/*!
 * \brief Input args for #ENET_MDIO_IOCTL_C45_READ command.
 *
 * Clause-45 register read arguments.
 */
typedef struct EnetMdio_C45ReadInArgs_s
{
    /*! User channel to be used to perform the register access.
     * Simultaneous access from same group is not allowed.
     * Not applicable in manual mode.
     */
    EnetMdio_Group group;

    /*! PHY address */
    uint32_t phyAddr;

    /*! MMD */
    EnetMdio_C45Mmd mmd;

    /*! Register address */
    uint16_t reg;
} EnetMdio_C45ReadInArgs;

/*!
 * \brief Input args for #ENET_MDIO_IOCTL_C22_WRITE command.
 *
 * Clause-22 register write arguments.
 */
typedef struct EnetMdio_C22WriteInArgs_s
{
    /*! User channel to be used to perform the register access.
     * Simultaneous access from same group is not allowed.
     * Not applicable in manual mode.
     */
    EnetMdio_Group group;

    /*! PHY address */
    uint32_t phyAddr;

    /*! Register address */
    uint16_t reg;

    /*! Value to be written */
    uint16_t val;
} EnetMdio_C22WriteInArgs;

/*!
 * \brief Input args for #ENET_MDIO_IOCTL_C45_WRITE command.
 *
 * Clause-45 register write arguments.
 */
typedef struct EnetMdio_C45WriteInArgs_s
{
    /*! User channel to be used to perform the register access.
     * Simultaneous access from same group is not allowed.
     * Not applicable in manual mode.
     */
    EnetMdio_Group group;

    /*! PHY address */
    uint32_t phyAddr;

    /*! MMD */
    EnetMdio_C45Mmd mmd;

    /*! Register address */
    uint16_t reg;

    /*! Value to be written */
    uint16_t val;
} EnetMdio_C45WriteInArgs;

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

#endif /* ENET_MOD_MDIO_H_ */

/*! @} */
