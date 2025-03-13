/*
 *  Copyright (c) Texas Instruments Incorporated 2024
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
 * \file  dp83tg720_priv.h
 *
 * \brief This file contains private type definitions and helper macros for the
 *        DP83TG721 AVB Ethernet PHY.
 */

#ifndef DP83TG721_PRIV_H_
#define DP83TG721_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include "../src/phy_common_priv.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

#define ENCODE_BIGFIELD(field, val) (((val) << field##_SHIFT) & field##_MASK)
#define DECODE_BIGFIELD(field, val) (((val) & field##_MASK) >> field##_SHIFT)
#define MODIFY_BIGFIELD(field, org, val) \
	((org) = (((org) & ~field##_MASK) | (((val) << field##_SHIFT) & field##_MASK)))
#define SET_BIT(mask, val) ((val) = ((val) | (mask)))
#define CLEAR_BIT(mask, val) ((val) = ((val) & ~(mask)))

#define ENETPHY_MASK(h, l) ((0xFFFF - (1 << (l)) + 1) & (0xFFFF >> (15 - (h))))

/* General registers */
#define MII_REG_12              (0x12U)
#define MII_REG_13              (0x13U)
#define MII_REG_18              (0x18U)
#define MII_REG_1F              (0x1FU)
#define SOR_VECTOR_1            (0x45DU)
#define RGMII_CTRL              (0x600U)
#define RGMII_DELAY_CTRL        (0x602U)
#define SGMII_CTRL_1            (0x608U)
#define PMA_PMD_CONTROL         (0x1834U)
#define A2D_REG_48              (0x430U)
#define LPS_CFG3                (0x018CU)

/*! \brief MII_REG_1F bits */
#define HW_RESET                ENETPHY_BIT(15)
#define SW_RESET                ENETPHY_BIT(14)

/*! \brief MII_REG_12 bits */
#define TRAINING_DONE_INT_EN    ENETPHY_BIT(2)
#define ESD_EVENT_INT_EN        ENETPHY_BIT(3)
#define LINK_STAT_INT_EN        ENETPHY_BIT(5)
#define ENERGY_DET_INT_EN       ENETPHY_BIT(6)

/*! \brief MII_REG_13 bits */
#define OVERTEMP_INT_EN         ENETPHY_BIT(3)
#define OVERVOLTAGE_INT_EN      ENETPHY_BIT(6)
#define UNDERVOLTAGE_INT_EN     ENETPHY_BIT(7)

/*! \brief MII_REG_18 bits */
#define LPS_INT_EN              ENETPHY_BIT(0)
#define WUR_INT_EN              ENETPHY_BIT(1)
#define POR_DONE_INT_EN         ENETPHY_BIT(3)

/*! \brief SGMII_CTRL_1 bits */
#define SGMII_AUTO_NEG_EN       ENETPHY_BIT(0)
#define SGMII_EN                ENETPHY_BIT(9)

/*! \brief SOR_VECTOR_1 bits */
#define MASTER_MODE             ENETPHY_BIT(5)
#define RGMII_IS_EN             ENETPHY_BIT(12)
#define SGMII_IS_EN             ENETPHY_BIT(13)
#define RX_SHIFT_EN             ENETPHY_BIT(14)
#define TX_SHIFT_EN             ENETPHY_BIT(15)

/*! \brief RGMII_DELAY_CTRL */
#define RGMII_RX_SHIFT          ENETPHY_BIT(1)
#define RGMII_TX_SHIFT          ENETPHY_BIT(0)

/* 1588 PTP registers */
#define PTP_CTL      0xD00 /* PTP Control Register */
#define PTP_TDR      0xD01 /* PTP Time Data Register */
#define PTP_STS      0xD02 /* PTP Status Register */
#define PTP_TSTS     0xD03 /* PTP Trigger Status Register */
#define PTP_RATEL    0xD04 /* PTP Rate Low Register */
#define PTP_RATEH    0xD05 /* PTP Rate High Register */
#define PTP_TXTS     0xD08 /* PTP Transmit Timestamp Register */
#define PTP_RXTS     0xD09 /* PTP Receive Timestamp Register */
#define PTP_ESTS     0xD0A /* PTP Event Status Register */
#define PTP_EDATA    0xD0B /* PTP Event Data Register */
#define PTP_TRIG     0xD10 /* PTP Trigger Configuration Register */
#define PTP_EVNT     0xD11 /* PTP Event Configuration Register */
#define PTP_TXCFG0   0xD12 /* PTP Transmit Configuration Register 0 */
#define PTP_TXCFG1   0xD13 /* PTP Transmit Configuration Register 1 */
#define PSF_CFG0     0xD14 /* PHY Status Frame Configuration Register 0 */
#define PTP_RXCFG0   0xD15 /* PTP Receive Configuration Register 0 */
#define PTP_RXCFG1   0xD16 /* PTP Receive Configuration Register 1 */
#define PTP_RXCFG2   0xD17 /* PTP Receive Configuration Register 2 */
#define PTP_RXCFG3   0xD18 /* PTP Receive Configuration Register 3 */
#define PTP_RXCFG4   0xD19 /* PTP Receive Configuration Register 4 */
#define PTP_TRDL     0xD1A /* PTP Temporary Rate Duration Low Register */
#define PTP_TRDH     0xD1B /* PTP Temporary Rate Duration High Register */
#define PTP_COC      0xD20 /* PTP Clock Output Control Register */
#define PSF_CFG1     0xD21 /* Phy Status Frame Configuration Register 1 */
#define PSF_CFG2     0xD22 /* Phy Status Frame Configuration Register 2 */
#define PSF_CFG3     0xD23 /* Phy Status Frame Configuration Register 3 */
#define PSF_CFG4     0xD24 /* Phy Status Frame Configuration Register 4 */
#define PTP_SFDCFG   0xD25 /* PTP SFD Configuration Register */
#define PTP_INTCTL   0xD26 /* PTP Interrupt Control Register */
#define PTP_CLKSRC   0xD27 /* PTP Clock Source Register */
#define PTP_ETR      0xD28 /* PTP Ethernet Type Register */
#define PTP_OFF      0xD29 /* PTP Offset Register */
#define PTP_GPIOMON  0xD2A /* PTP GPIO Monitor Register */
#define PTP_RXHASH   0xD2B /* PTP Receive Hash Register */
#define PTP_EVENT_GPIO_SEL   0xD30 /* PTP Event GPIO selection */
#define RX_SMD_GPIO_CTL      0xD31 /* RX path SMD detection and GPIO control */
#define TX_SMD_GPIO_CTL      0xD32 /* TX path SMD detection and GPIO control */
#define SCH_CTL_1            0xD33 /* Scheduler control 1 */
#define SCH_CTL_2            0xD34 /* Scheduler control 2 */
#define FREQ_CTL_1           0xD35 /* Base frequency control 1 */
#define FREQ_CTL_2           0xD36 /* Base frequency control 2 */
#define PTP_RATEL_ACC_ONLY   0xD37 /* PTP Rate ACC only LSB Register */
#define PTP_RATEH_ACC_ONLY   0xD38 /* PTP Rate ACC only MSB Register and enable */
#define PTP_PLL_CTL          0xD39 /* PTP_PLL control register */
#define PTP_PTP_PLL_RD_1     0xD3A /* PTP timestamp read register 1 */
#define PTP_PTP_PLL_RD_2     0xD3B /* PTP timestamp read register 2 */
#define PTP_PTP_PLL_RD_3     0xD3C /* PTP timestamp read register 3 */
#define PTP_PTP_PLL_RD_4     0xD3D /* PTP timestamp read register 4 */
#define PTP_PTP_PLL_RD_5     0xD3E /* PTP timestamp read register 5 */
#define PTP_PTP_PLL_RD_6     0xD3F /* PTP timestamp read register 6 */
#define PTP_ONESTEP_OFF      0xD40 /* PTP ONESTEP OFFSET register */
#define PTP_PLL_FORCE_CTRL_1 0xD41 /* PTP_PLL Force control 1 */
#define PTP_PLL_FORCE_CTRL_2 0xD42 /* PTP_PLL Force control 2 */
#define PTP_PLL_FORCE_CTRL_3 0xD43 /* PTP_PLL Force control 3 */

#define CRF_MAS_TS_CAPT      0xD92 /* CRF master TS capture  */

#define CRF_MAS_MCLK_LOC_SEC_31_16    0xD93  /* Media Clock Edge Location Sec MSB */
#define CRF_MAS_MCLK_LOC_SEC_15_0     0xD94  /* Media Clock Edge Location Sec LSB */
#define CRF_MAS_MCLK_LOC_NSEC_31_16   0xD95  /* Media Clock Edge Location nSec MSB */
#define CRF_MAS_MCLK_LOC_NSEC_15_0    0xD96  /* Media Clock Edge Location nSec LSB */

#define PTP_PLL_EN_CTL       0xD97 /* PTP_PLL Enable control */
#define MCLK_PH_ADJ_LIM_1    0xD9D /* Media Clock Phase Adj Lim 1 */
#define MCLK_PH_ADJ_LIM_2    0xD9E /* Media Clock Phase Adj Lim 2 */
#define MCLK_DIV_CTL_1       0xD9F /* Media clock Division factor 1 */
#define MCLK_DIV_CTL_2       0xDA0 /* Media clock Division factor 2 */
#define MCLK_PH_ADJ_CTL_1    0xDA3 /* Media clock phase adjustment control 1 */
#define MCLK_PH_ADJ_CTL_2    0xDA4 /* Media clock phase adjustment control 2 */
#define CLKOUT_MUX_CTL       0xDA8 /* CLKOUT muxing control */

#define CRF_PARSE_CTL             0xD80 /* CRF parsing control register */
#define CRF_ETYPE                 0xD81 /* CRF ethertype control register */
#define CRF_IP_CTL                0xD88 /* CRF IP parse control register */
#define CRF_SAMP_CTL              0xD8A /* CRF sample control register */
#define CRF_INT                   0xD8B /* CRF Interrupt register */
#define CRF_BUF_RD_63_48          0xD8C /* CRF Buffer Read 63:48 */
#define CRF_BUF_RD_47_32          0xD8D /* CRF Buffer Read 47:32 */
#define CRF_BUF_RD_31_16          0xD8E /* CRF Buffer Read 31:16 */
#define CRF_BUF_RD_15_0           0xD8F /* CRF Buffer Read 15:0 */
#define CRF_TS_INTRVL_RD          0xDA6 /* CRF Timestamp Interval Read register */
#define CRF_INT_2                 0xDB1 /* CRF interrupt register 2 */
#define AUDIO_CLK_CTRL            0xDC1 /* Audio Clock Gating Control */
#define CRF_INT_REG_3             0xDC2 /* CRF interrupt register 3 */
#define MEDIA_CLK_GATE_CTRL_1     0xDC3 /* Media Clock Gating Control 1 */
#define MEDIA_CLK_GATE_CTRL_2     0xDC4 /* Media Clock Gating Control 2 */
#define MEDIA_CLK_GATE_CTRL_3     0xDC5 /* Media Clock Gating Control 3 */
#define MEDIA_CLK_GATE_CTRL_4     0xDC6 /* Media Clock Gating Control 4 */
#define MEDIA_CLK_GATE_CTRL_5     0xDC7 /* Media Clock Gating Control 5 */

/* Bit definitions for the PTP_CTL register */
#define TRIG_SEL_SHIFT  (10)   /* PTP Trigger Select */
#define TRIG_SEL_MASK   ENETPHY_MASK(12, 10)
#define TRIG_DIS        ENETPHY_BIT(9) /* Disable PTP Trigger */
#define TRIG_EN         ENETPHY_BIT(8) /* Enable PTP Trigger */
#define TRIG_READ       ENETPHY_BIT(7) /* Read PTP Trigger */
#define TRIG_LOAD       ENETPHY_BIT(6) /* Load PTP Trigger */
#define PTP_RD_CLK      ENETPHY_BIT(5) /* Read PTP Clock */
#define PTP_LOAD_CLK    ENETPHY_BIT(4) /* Load PTP Clock */
#define PTP_STEP_CLK    ENETPHY_BIT(3) /* Step PTP Clock */
#define PTP_ENABLE      ENETPHY_BIT(2) /* Enable PTP Clock */
#define PTP_DISABLE     ENETPHY_BIT(1) /* Disable PTP Clock */
#define PTP_RESET       ENETPHY_BIT(0) /* Reset PTP Clock */

/* Bit definitions for the PTP_STS register */
#define CRF_RXTS_RDY    ENETPHY_BIT(12)  /* CRF receive Timestamp ready */
#define TXTS_RDY        ENETPHY_BIT(11)  /* Transmit Timestamp Ready */
#define RXTS_RDY        ENETPHY_BIT(10)  /* Receive Timestamp Ready */
#define TRIG_DONE       ENETPHY_BIT(9)   /* PTP Trigger Done */
#define EVENT_RDY       ENETPHY_BIT(8)   /* PTP Event Timestamp Ready */
#define CRF_RXTS_IE     ENETPHY_BIT(4)   /* Enable CRF receive Timestamp interrupt */
#define TXTS_IE         ENETPHY_BIT(3)   /* Enable Transmit Timestamp Interrupt */
#define RXTS_IE         ENETPHY_BIT(2)   /* Enable Receive Timestamp Interrupt */
#define TRIG_IE         ENETPHY_BIT(1)   /* Enable Trigger Interrupt */
#define EVENT_IE        ENETPHY_BIT(0)   /* Enable Event Interrupt: Enable Interrupt on Event Timestamp Read */

/* Bit definitions for the PTP_TSTS register */
#define TRIG7_ERROR     ENETPHY_BIT(15) /* Trigger 7 Error */
#define TRIG7_ACTIVE    ENETPHY_BIT(14) /* Trigger 7 Active */
#define TRIG6_ERROR     ENETPHY_BIT(13) /* Trigger 6 Error */
#define TRIG6_ACTIVE    ENETPHY_BIT(12) /* Trigger 6 Active */
#define TRIG5_ERROR     ENETPHY_BIT(11) /* Trigger 5 Error */
#define TRIG5_ACTIVE    ENETPHY_BIT(10) /* Trigger 5 Active */
#define TRIG4_ERROR     ENETPHY_BIT(9)  /* Trigger 4 Error */
#define TRIG4_ACTIVE    ENETPHY_BIT(8)  /* Trigger 4 Active */
#define TRIG3_ERROR     ENETPHY_BIT(7)  /* Trigger 3 Error */
#define TRIG3_ACTIVE    ENETPHY_BIT(6)  /* Trigger 3 Active */
#define TRIG2_ERROR     ENETPHY_BIT(5)  /* Trigger 2 Error */
#define TRIG2_ACTIVE    ENETPHY_BIT(4)  /* Trigger 2 Active */
#define TRIG1_ERROR     ENETPHY_BIT(3)  /* Trigger 1 Error */
#define TRIG1_ACTIVE    ENETPHY_BIT(2)  /* Trigger 1 Active */
#define TRIG0_ERROR     ENETPHY_BIT(1)  /* Trigger 0 Error */
#define TRIG0_ACTIVE    ENETPHY_BIT(0)  /* Trigger 0 Active */

/* Bit definitions for the PTP_RATEH register */
#define PTP_RATE_DIR      ENETPHY_BIT(15) /* PTP Rate Direction */
#define PTP_TMP_RATE      ENETPHY_BIT(14) /* PTP Temporary Rate */
#define PTP_RATE_HI_SHIFT (0)     /* PTP Rate High 10-bits */
#define PTP_RATE_HI_MASK  ENETPHY_MASK(9, 0)

/* Bit definitions for the PTP_ESTS register */
#define EVNTS_MISSED_SHIFT (8)     /* Number of Events Missed */
#define EVNTS_MISSED_MASK  ENETPHY_MASK(10, 8)
#define EVNT_TS_LEN_SHIFT  (6)     /* Event Timestamp Length */
#define EVNT_TS_LEN_MASK   ENETPHY_MASK(7, 6)
#define EVNT_RF            ENETPHY_BIT(5)  /* Event edge configuration */
#define EVNT_NUM_SHIFT     (2)     /* Event Number */
#define EVNT_NUM_MASK      ENETPHY_MASK(4, 2)
#define MULT_EVNT          ENETPHY_BIT(1)  /* Multiple Event Detect */
#define EVENT_DET          ENETPHY_BIT(0)  /* PTP Event Detected */

/* Bit definitions for the PTP_EDATA register */
#define E7_RISE   ENETPHY_BIT(15) /* Rise/Fall edge direction for Event 7 */
#define E7_DET    ENETPHY_BIT(14) /* Event 7 detected */
#define E6_RISE   ENETPHY_BIT(13) /* Rise/Fall edge direction for Event 6 */
#define E6_DET    ENETPHY_BIT(12) /* Event 6 detected */
#define E5_RISE   ENETPHY_BIT(11) /* Rise/Fall edge direction for Event 5 */
#define E5_DET    ENETPHY_BIT(10) /* Event 5 detected */
#define E4_RISE   ENETPHY_BIT(9)  /* Rise/Fall edge direction for Event 4 */
#define E4_DET    ENETPHY_BIT(8)  /* Event 4 detected */
#define E3_RISE   ENETPHY_BIT(7)  /* Rise/Fall edge direction for Event 3 */
#define E3_DET    ENETPHY_BIT(6)  /* Event 3 detected */
#define E2_RISE   ENETPHY_BIT(5)  /* Rise/Fall edge direction for Event 2 */
#define E2_DET    ENETPHY_BIT(4)  /* Event 2 detected */
#define E1_RISE   ENETPHY_BIT(3)  /* Rise/Fall edge direction for Event 1 */
#define E1_DET    ENETPHY_BIT(2)  /* Event 1 detected */
#define E0_RISE   ENETPHY_BIT(1)  /* Rise/Fall edge direction for Event 0 */
#define E0_DET    ENETPHY_BIT(0)  /* Event 0 detected */

/* Bit definitions for the PTP_TRIG register */
#define TRIG_PULSE       ENETPHY_BIT(15) /* Trigger Pulse */
#define TRIG_PER         ENETPHY_BIT(14) /* Trigger Periodic */
#define TRIG_IF_LATE     ENETPHY_BIT(13) /* Trigger-if-late Control */
#define TRIG_NOTIFY      ENETPHY_BIT(12) /* Trigger Notification Enable */
#define TRIG_GPIO_SHIFT  (8)     /* GPIO trigger configuration */
#define TRIG_GPIO_MASK   ENETPHY_MASK(11, 8)
#define TRIG_TOGGLE      ENETPHY_BIT(7)  /* Trigger Toggle mode enable */
#define TRIG_CSEL_SHIFT  (1)     /* Trigger Configuration Select */
#define TRIG_CSEL_MASK   ENETPHY_MASK(3, 1)
#define TRIG_WR          ENETPHY_BIT(0)  /* Trigger Configuration Write */

/* Bit definitions for the PTP_EVNT register */
#define EVENT_SYNC_BYPASS ENETPHY_BIT(15) /* PTP Event Sync */
#define EVNT_RISE         ENETPHY_BIT(14) /* Event Rise Detect Enable */
#define EVNT_FALL         ENETPHY_BIT(13) /* Event Fall Detect Enable */
#define EVNT_SINGLE       ENETPHY_BIT(12) /* Single Event Capture */
#define EVNT_GPIO_SHIFT   (8)     /* Event GPIO connection */
#define EVNT_GPIO_MASK    ENETPHY_MASK(11, 8)
#define EVNT_SEL_SHIFT    (1)     /* Event Select */
#define EVNT_SEL_MASK     ENETPHY_MASK(3, 1)
#define EVNT_WR           ENETPHY_BIT(0)  /* Event Configuration Write */

/* Bit definitions for the PTP_TXCFG0 register */
#define SYNC_1STEP   ENETPHY_BIT(15) /* Sync Message One-Step Enable */
#define DR_INSERT    ENETPHY_BIT(13) /* Insert Delay_Req timestamp in Delay_Resp */
#define NTP_TS_EN    ENETPHY_BIT(12) /* Enable Timestamping of NTP Packets */
#define IGNORE_2STEP ENETPHY_BIT(11) /* Ignore Two_Step flag for One-Step operation */
#define CRC_1STEP    ENETPHY_BIT(10) /* Disable checking of CRC for One-Step operation */
#define CHK_1STEP    ENETPHY_BIT(9)  /* Enable UDP Checksum correction for One-Step Operation */
#define IP1588_EN    ENETPHY_BIT(8)  /* Enable IEEE 1588 defined IP address filter */
#define TX_L2_EN     ENETPHY_BIT(7)  /* Layer2 Timestamp Enable */
#define TX_IPV6_EN   ENETPHY_BIT(6)  /* IPv6 Timestamp Enable */
#define TX_IPV4_EN   ENETPHY_BIT(5)  /* IPv4 Timestamp Enable */
#define TX_PTP_VER_SHIFT (1)     /* PTP Version */
#define TX_PTP_VER_MASK  ENETPHY_MASK(4, 1)
#define TX_TS_EN         ENETPHY_BIT(0)  /* Transmit Timestamp Enable */

/* Bit definitions for the PTP_TXCFG1 register */
#define BYTE0_MASK_SHIFT (8)     /* Byte0 Data */
#define BYTE0_MASK_MASK  ENETPHY_MASK(15, 8)
#define BYTE0_DATA_SHIFT (0)     /* Byte0 Mask */
#define BYTE0_DATA_MASK  ENETPHY_MASK(7, 0)

/* Bit definitions for the PSF_CFG0 register */
#define MAC_SRC_ADD_SHIFT (11)    /* Phy Status Frame Mac Source Address */
#define MAC_SRC_ADD_MASK  ENETPHY_MASK(12, 11)
#define MIN_PRE_SHIFT     (8)     /* Phy Status Frame minimum preamble */
#define MIN_PRE_MASK      ENETPHY_MASK(10, 8)
#define PSF_ENDIAN        ENETPHY_BIT(7)  /* Phy Status Frame Endian control */
#define PSF_IPV4          ENETPHY_BIT(6)  /* This bit controls the type of packet used for Phy Status Frames */
#define PSF_PCF_RD   ENETPHY_BIT(5)  /* Phy Control Frame Read Phy Status Frame enable */
#define PSF_ERR_EN   ENETPHY_BIT(4)  /* PSF Error Phy Status Frame enable */
#define PSF_TXTS_EN  ENETPHY_BIT(3)  /* Transmit Timestamp Phy Status Frame enable */
#define PSF_RXTS_EN  ENETPHY_BIT(2)  /* Receive Timestamp Phy Status Frame enable */
#define PSF_TRIG_EN  ENETPHY_BIT(1)  /* Trigger Phy Status Frame enable */
#define PSF_EVNT_EN  ENETPHY_BIT(0)  /* Event Phy Status Frame enable */

/* Bit definitions for the PTP_RXCFG0 register */
#define DOMAIN_EN        ENETPHY_BIT(15) /* Domain Match Enable */
#define ALT_MAST_DIS     ENETPHY_BIT(14) /* Alternate Master Timestamp Disable */
#define USER_IP_SEL      ENETPHY_BIT(13) /* IP Address data select */
#define USER_IP_EN       ENETPHY_BIT(12) /* Enable User-programmed IP address filter */
#define RX_SLAVE         ENETPHY_BIT(11) /* Receive Slave Only */
#define IP1588_EN_SHIFT  (8)     /* Enable IEEE 1588 defined IP address filters */
#define IP1588_EN_MASK   ENETPHY_MASK(10, 8)
#define RX_L2_EN         ENETPHY_BIT(7)  /* Layer2 Timestamp Enable */
#define RX_IPV6_EN       ENETPHY_BIT(6)  /* IPv6 Timestamp Enable */
#define RX_IPV4_EN       ENETPHY_BIT(5)  /* IPv4 Timestamp Enable */
#define RX_PTP_VER_SHIFT (1)     /* PTP Version */
#define RX_PTP_VER_MASK  ENETPHY_MASK(4, 1)
#define RX_TS_EN         ENETPHY_BIT(0)  /* Receive Timestamp Enable */

/* Bit definitions for the PTP_RXCFG1 register */
#define BYTE0_MASK_SHIFT (8)     /* Byte0 Mask */
#define BYTE0_MASK_MASK  ENETPHY_MASK(15, 8)
#define BYTE0_DATA_SHIFT (0)     /* Byte0 Data */
#define BYTE0_DATA_MASK  ENETPHY_MASK(7, 0)

/* Bit definitions for the PTP_RXCFG3 register */
#define TS_MIN_IFG_SHIFT (12)    /* Minimum Inter-frame Gap */
#define TS_MIN_IFG_MASK  ENETPHY_MASK(15, 12)
#define ACC_UDP          ENETPHY_BIT(11) /* Record Timestamp if UDP Checksum Error */
#define ACC_CRC          ENETPHY_BIT(10) /* Record Timestamp if CRC Error */
#define TS_APPEND        ENETPHY_BIT(9 ) /* Enable Timestamp append for L2 */
#define TS_INSERT        ENETPHY_BIT(8 ) /* Enable Timestamp Insertion */
#define PTP_DOMAIN_SHIFT (0)     /* PTP Domain */
#define PTP_DOMAIN_MASK  ENETPHY_MASK(7, 0)

/* Bit definitions for the PTP_RXCFG4 register */
#define IPV4_UDP_MOD       ENETPHY_BIT(15) /* Enable IPV4 UDP modification */
#define TS_SEC_EN          ENETPHY_BIT(14) /* Enable Timestamp Seconds */
#define TS_SEC_LEN_SHIFT   (12)    /* Inserted Timestamp Seconds Length */
#define TS_SEC_LEN_MASK    ENETPHY_MASK(13, 12)
#define RXTS_NS_OFF_SHIFT  (6)     /* Receive Timestamp Nanoseconds offset */
#define RXTS_NS_OFF_MASK   ENETPHY_MASK(11, 6)
#define RXTS_SEC_OFF_SHIFT (0)     /* Receive Timestamp Seconds offset */
#define RXTS_SEC_OFF_MASK  ENETPHY_MASK(5, 0)

/* Bit definitions for the PTP_TRDH register */
#define PTP_TR_DURH_SHIFT  (0)     /* PTP Temporary Rate Duration High 10-bits */
#define PTP_TR_DURH_MASK   ENETPHY_MASK(9, 0)

/* Bit definitions for the PTP_COC register */
#define PTP_CLKDIV_SHIFT   (0)     /* PTP Clock Divide-by Value */
#define PTP_CLKDIV_MASK    ENETPHY_MASK(7, 0)

/* Bit definitions for the PSF_CFG1 register */
#define VERSIONPTP_SHIFT         (8)     /* PTP v2 versionPTP field */
#define VERSIONPTP_MASK          ENETPHY_MASK(11, 8)
#define TRANSPORTSPECIFIC_SHIFT  (4)     /* PTP v2 Header transportSpecific field */
#define TRANSPORTSPECIFIC_MASK   ENETPHY_MASK(7, 4)
#define MESSAGETYPE_SHIFT        (0)     /* PTP v2 messageType field */
#define MESSAGETYPE_MASK         ENETPHY_MASK(3, 0)

/* Bit definitions for the PSF_CFG2 register */
#define IP_SA_BYTE1_SHIFT  (8)     /* Second byte of IP source address */
#define IP_SA_BYTE1_MASK   ENETPHY_MASK(15, 8)
#define IP_SA_BYTE0_SHIFT  (0)     /* First byte of IP source address */
#define IP_SA_BYTE0_MASK   ENETPHY_MASK(7, 0)

/* Bit definitions for the PSF_CFG3 register */
#define IP_SA_BYTE3_SHIFT  (8)     /* Fourth byte of IP source address */
#define IP_SA_BYTE3_MASK   ENETPHY_MASK(15, 8)
#define IP_SA_BYTE2_SHIFT  (0)     /* Third byte of IP source address */
#define IP_SA_BYTE2_MASK   ENETPHY_MASK(7, 0)

/* Bit definitions for the PTP_SFDCFG register */
#define TX_SFD_GPIO_SHIFT  (4)     /* TX SFD GPIO Select */
#define TX_SFD_GPIO_MASK   ENETPHY_MASK(7, 4)
#define RX_SFD_GPIO_SHIFT  (0)     /* RX SFD GPIO Select */
#define RX_SFD_GPIO_MASK   ENETPHY_MASK(3, 0)

/* Bit definitions for the PTP_INTCTL register */
#define PTP_INT_GPIO_SHIFT (0)     /* PTP Interrupt GPIO Select */
#define PTP_INT_GPIO_MASK  ENETPHY_MASK(3, 0)

/* Bit definitions for the PTP_CLKSRC register */
#define CLK_SRC_1_SHIFT    (14)    /* PTP Clock Source Select */
#define CLK_SRC_1_MASK     ENETPHY_MASK(15, 14)
#define CLK_DIV_EN         ENETPHY_BIT(13) /* Clock division enable */
#define CLK_SRC_2_SHIFT    (11)    /* PTP Clock Source Select */
#define CLK_SRC_2_MASK     ENETPHY_MASK(12, 11)
#define CLK_DIV_VAL_SHIFT  (7)     /* Clock division divider value */
#define CLK_DIV_VAL_MASK   ENETPHY_MASK(10, 7)
#define CLK_SRC_PER_SHIFT  (0)     /* PTP Clock Source Period */
#define CLK_SRC_PER_MASK   ENETPHY_MASK(6, 0)

/* Bit definitions for the PTP_OFF register */
#define PTP_OFFSET_SHIFT   (0)     /* PTP Offset */
#define PTP_OFFSET_MASK    ENETPHY_MASK(7, 0)

/* Bit definitions for the PTP_GPIOMON register */
#define PTP_GPIO_IN_SHIFT  (0)     /* This field reflects the current values seen on the GPIO inputs. */
#define PTP_GPIO_IN_MASK   ENETPHY_MASK(4, 0)

/* Bit definitions for the PTP_RXHASH register */
#define RX_HASH_EN         ENETPHY_BIT(12) /* Receive Hash Enable */
#define PTP_RX_HASH_SHIFT  (0)     /* Receive Hash */
#define PTP_RX_HASH_MASK   ENETPHY_MASK(11, 0)

/* Bit definitions for the PTP_EVENT_GPIO_SEL register */
#define PTP_GPIO_EVENT_EN_SHIFT (0)     /* GPIO PTP Event timestamp enable */
#define PTP_GPIO_EVENT_EN_MASK  ENETPHY_MASK(4, 0)

/* Bit definitions for the RX_SMD_GPIO_CTL register */
#define RX_SMD_GPIO_SEL_SHIFT  (5)
#define RX_SMD_GPIO_SEL_MASK   ENETPHY_MASK(8, 5) /* RX_SMD detect signal output configuration */
#define RX_SMD_S_E_SEL	       ENETPHY_BIT(4) /* SMD detection type for RX */
#define RX_SMD_DET_SEL_SHIFT   (0)
#define RX_SMD_DET_SEL_MASK	   ENETPHY_MASK(3, 0) /* RX_SMD detect Enable */

/* Bit definitions for the TX_SMD_GPIO_CTL register */
#define MR_DUAL_VLAN_TAG_EN    ENETPHY_BIT(15) /* Dual VLAN tagging packet parsing enable */
#define TX_SMD_GPIO_SEL_SHIFT  (5)
#define TX_SMD_GPIO_SEL_MASK   ENETPHY_MASK(8, 5) /* TX_SMD detect signal output configuration */
#define TX_SMD_S_E_SEL         ENETPHY_BIT(4) /* TX_SMD detect Enable */
#define TX_SMD_DET_SEL_SHIFT   (0)
#define TX_SMD_DET_SEL_MASK	   ENETPHY_MASK(3, 0) /* TX_SMD detect Enable */

/* Bit definitions for the FREQ_CTL_1 register */
#define MR_BASE_FREQ_15_0_SHIFT (0)
#define MR_BASE_FREQ_15_0_MASK  ENETPHY_MASK(15, 0) /* Base frequency programmable for PTP_PLL LSB word */

/* Bit definitions for the FREQ_CTL_2 register */
#define MR_BASE_FREQ_31_16_SHIFT (0)
#define MR_BASE_FREQ_31_16_MASK  ENETPHY_MASK(15, 0) /* Base frequency programmable for PTP_PLL MSB word */

/* Bit definitions for the PTP_RATEH_ACC_ONLY register */
#define PTP_RATE_DIR_ACC_ONLY           ENETPHY_BIT(15) /* PTP Rate ACC only Direction */
#define PTP_ACC_MODE_CFG                ENETPHY_BIT(14) /* PTP Accumulator mode */
#define PTP_RATE_ACC_ONLY_25_16_SHIFT   (0) /* PTP Rate ACC only high 10-bits */
#define PTP_RATE_ACC_ONLY_25_16_MASK    ENETPHY_MASK(9, 0)

/* Bit definitions for the PTP_RATEL_ACC_ONLY register */
#define PTP_RATE_ACC_ONLY_15_0_SHIFT    (0) /* Rate Control value for PTP accumulator */
#define PTP_RATE_ACC_ONLY_15_0_MASK     ENETPHY_MASK(15, 0)

/* Bit definitions for the PTP_ONESTEP_OFF register */
#define PTP_RATE_ACC_ONLY_31_26_SHIFT   (10) /* PTP accumulator Rate Control value */
#define PTP_RATE_ACC_ONLY_31_26_MASK    ENETPHY_MASK(15, 10) /* PTP accumulator Rate Control value */

/* Bit definitions for the PTP_PLL_EN_CTL register */
#define MR_PTP_PLL_PTP_PLL_EN           ENETPHY_BIT(0) /* Enable PTP_PLL */
#define MR_PTP_PLL_CLK_DIV_EN           ENETPHY_BIT(1) /* Enable PTP_PLL output clock divide */
#define MR_PTP_PLL_INITIAL_SETTLE_TIMER_SHIFT (4)
#define MR_PTP_PLL_INITIAL_SETTLE_TIMER_MASK  ENETPHY_MASK(15, 4)

/* Bit definitions for the CLKOUT_MUX_CTL register */
#define CLK_MUX_SEL_SHIFT               (0) /* GPIO and Clock output configuration */
#define CLK_MUX_SEL_MASK                ENETPHY_MASK(7, 0)
/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

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

#endif /* DP83TG721_PRIV_H_ */
