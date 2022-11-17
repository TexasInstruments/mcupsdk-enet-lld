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
 * \file  vsc8514_priv.h
 *
 * \brief This file contains private type definitions and helper macros for the
 *        VSC8514 QSGMII PHY.
 */

#ifndef VSC8514_PRIV_H_
#define VSC8514_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! Vitesse Extended Page Access Register */
#define VSC8514_PAGE_ACCESS                   (31U)

/*! \brief Extended PHY Control Set 1 */
#define VSC8514_EXTENDED_PHYCTRL1             (23U)

/*! \brief Micro command register */
#define VSC8514_GPIO_COMMAND                  (18U)

/*! \brief MAC Mode and Fast Link Configuration */
#define VSC8514_GPIO_MACMODE                  (19U)

/*! \brief Extended page 3 - MAC SerDes PCS Control */
#define VSC8514_EXT3_MACSERDES_PCS_CTRL       (16U)

/* MAC mode register definitions */
#define VSC8514_GPIO_COMMAND_GO_MASK          (0x8000U)
#define VSC8514_GPIO_COMMAND_GO_SHIFT         (15U)
#define VSC8514_GPIO_COMMAND_GO_SET           (0x8000U)
#define VSC8514_GPIO_COMMAND_ERRBIT_MASK      (0x4000U)
#define VSC8514_GPIO_COMMAND_ERRBIT_SHIFT     (14U)
#define VSC8514_GPIO_COMMAND_QSGMII           (0xE0U)

/* MAC mode register definitions */
#define VSC8514_GPIO_MACMODE_MACMODE_MASK     (0xC000U)
#define VSC8514_GPIO_MACMODE_MACMODE_SHIFT    (14U)
#define VSC8514_GPIO_MACMODE_MACMODE_QSGMII   (1U)

/* MAC mode register definitions */
#define VSC8514_EXT3_MACSERDES_PCS_CTRL_AUTONEG_EN_MASK         (0x80U)
#define VSC8514_EXT3_MACSERDES_PCS_CTRL_AUTONEG_EN_SHIFT        (7U)
#define VSC8514_EXT3_MACSERDES_PCS_CTRL_AUTONEG_EN              (1U)
#define VSC8514_EXT3_MACSERDES_PCS_CTRL_AUTONEG_RESTART_MASK    (0x1000U)
#define VSC8514_EXT3_MACSERDES_PCS_CTRL_AUTONEG_RESTART_SHIFT   (12U)
#define VSC8514_EXT3_MACSERDES_PCS_CTRL_AUTONEG_RESTART         (1U)
#define VSC8514_EXT3_MACSERDES_PCS_CTRL_MACIF_RESTART_MASK      (0x4000U)
#define VSC8514_EXT3_MACSERDES_PCS_CTRL_MACIF_RESTART_SHIFT     (14U)
#define VSC8514_EXT3_MACSERDES_PCS_CTRL_MACIF_RESTART           (1U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief PHY extended register page.
 *
 * Used to select the pages of PHY extended registers.
 */
typedef enum Vsc8514_ExtRegSetType_e
{
    /*! Main page register space */
    VSC8514_EXTREGSET_MAIN  = 0x0000U,

    /*! Extended page 1 register space */
    VSC8514_EXTREGSET_PAGE1 = 0x0001U,

    /*! Extended page 2 register space */
    VSC8514_EXTREGSET_PAGE2 = 0x0002U,

    /*! Extended page 3 register space */
    VSC8514_EXTREGSET_PAGE3 = 0x0003U,

    /*! General purpose register space */
    VSC8514_EXTREGSET_GPIO  = 0x0010U,
} Vsc8514_ExtRegSetType;

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

#endif /* VSC8514_PRIV_H_ */
