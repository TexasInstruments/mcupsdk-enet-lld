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

/*******************************************************************************
 *  INCLUDE FILES
 *******************************************************************************
 */

/* Standard language headers */
#include <stddef.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdint.h>

/* OS/Posix headers */

#include <xdc/std.h>

/*Project headers*/
#include "cpsw_phyaccess_raw.h"
#include <examples/utils/include/enet_apputils.h>

#include <ti/sysbios/knl/Task.h>

/*******************************************************************************
 *  Defines
 *******************************************************************************
 */

#define CPSW_2G_MDIO_BASE_ADDR      0x46000F00
#define CPSW_9G_MDIO_BASE_ADDR      0x0C000F00

#define CPSW_MDIO_VER               0x00
#define CPSW_MDIO_CONTROL           0x04
#define CPSW_MDIO_ALIVE             0x08
#define CPSW_MDIO_LINK              0x0c
#define CPSW_MDIO_LINKINTRAW        0x10
#define CPSW_MDIO_LINKINTMASKED     0x14
#define CPSW_MDIO_USERINTRAW        0x20
#define CPSW_MDIO_USERINTMASKED     0x24
#define CPSW_MDIO_USERINTMASKSET    0x28
#define CPSW_MDIO_USERINTMASKCLR    0x2c
#define CPSW_MDIO_USERACCESS0       0x80
#define CPSW_MDIO_USERPHYSEL0       0x84
#define CPSW_MDIO_USERACCESS1       0x88
#define CPSW_MDIO_USERPHYSEL1       0x8c

/* wait until hardware is ready for another user access */
#define USERACCESS_GO               0x80000000
#define USERACCESS_READ             0x00000000
#define USERACCESS_WRITE            0x40000000
#define USERACCESS_ACK              1
#define USERACCESS_DATA             0x0000ffff


//register address of PHY
#define Control_0                   0
#define Status_1                    1
#define PHYID_2                     2
#define PHYID_3                     3
#define ANAR_4                      4
#define ANARLPA_5                   5  //status
//6,7,8 not needed to configure
#define ANEXP_6                     6
#define ANNP_7                      7
#define BT_Control_9                9
#define BT_Status_10                10
//11-12 reserved
#define Extn_Status_15              15
//16-17 Reserved.
#define Intr_Mask_18                18
#define Intr_Status_19              19
#define LED_control_24              24
#define LBK_control_19              19

#define PHY_CONF_TXCLKEN            0x0020
#define PHY_DPLX                    0x0100 //8bit
#define PHY_AUTON                   0x1000 //12bit
#define PHY_100_MBPS                0x2000 //6=0,13 =1
#define PHY_10_MBPS                 0x0000 //6=0,13 =0
#define PHY_1000_MBPS               0x0040 //6=1,13 =0

#define PHY_BMCR_RST_NEG            0x0200 //9bit
#define PHY_1000BTCR_1000FD         0x0200
#define PHY_1000BTCR_1000HD         0x0100
#define PHY_BMSR_AUTN_COMP          0x0020//5bit
#define PHY_DIG                     0x4000

#define WR_MEM_32(addr, data)    *(volatile unsigned int *)(addr) = (unsigned int)(data)
#define RD_MEM_32(addr)          *(volatile unsigned int *)(addr)

/*! Macro to get bit at given bit position  */
#define ENET_GET_BIT(val, n)            (((val) & (1 << (n))) >> (n))

#define MDIO_TIMEOUT 0xFFFF

#define MDIO_CTRL_REG_CFG_VAL 0x40000059

/*******************************************************************************
 *  Function's
 *******************************************************************************
 */

/*******************************************************************************
 *  Global's
 *******************************************************************************
 */

int globalVar = 0;
int gResetPhyFlag = 1;


void delay_mdio(uint32_t cnt)
{
    uint32_t i;

    for (i = 0; i < cnt; i++)
    {
        ;
    }
}

// wait for go bit to 0 and ack bit to become 1
Bool wait_for_user_access(int baseAddr)
{
    Bool retVal              = TRUE;
    volatile int32_t timeout = MDIO_TIMEOUT;
    uint32_t reg;

    reg = RD_MEM_32(baseAddr + CPSW_MDIO_USERACCESS0) & 0x80000000;
    while (reg != 0x0)
    {
        delay_mdio(10000);
        delay_mdio(10000);
        reg = RD_MEM_32(baseAddr + CPSW_MDIO_USERACCESS0) & 0x80000000;
        if (timeout > 0)
        {
            timeout--;
        }
        else
        {
            retVal = FALSE;
            break;
        }
    }

    timeout = MDIO_TIMEOUT;
    reg     = RD_MEM_32(baseAddr + CPSW_MDIO_USERACCESS0) & 0x20000000;
    while (reg != 0x20000000 && retVal)
    {
        delay_mdio(10000);
        delay_mdio(10000);
        reg = RD_MEM_32(baseAddr + CPSW_MDIO_USERACCESS0) & 0x20000000;

        if (timeout > 0)
        {
            timeout--;
        }
        else
        {
            retVal = FALSE;
            break;
        }
    }

    return retVal;
}

uint32_t cpsw_mdio_read(unsigned char phy_reg,
                        uint32_t PHY_ADDR,
                        int baseAddr)
{
    Bool success = TRUE;
    uint32_t reg = 0;

    reg =
        (USERACCESS_GO | USERACCESS_READ | (phy_reg << 21) | (PHY_ADDR << 16));
    WR_MEM_32((baseAddr + CPSW_MDIO_USERACCESS0), reg);
    success = wait_for_user_access(baseAddr);
    if (!success)
    {
        printf("\tMDIO Read Time Out to PHY 0x%02x:[0x%04x]\r\n", PHY_ADDR, phy_reg);
    }

    reg = RD_MEM_32(baseAddr + CPSW_MDIO_USERACCESS0);
    reg = reg & 0x0000ffff;
#if defined(DEBUG_FLAG)
    printf("\tPHYREG READ VALUE =  %x \r\n", reg);
#endif
    return reg;
}

void cpsw_mdio_write(unsigned char phy_reg,
                     unsigned short data,
                     uint32_t PHY_ADDR,
                     int baseAddr)
{
    Bool success = TRUE;
    uint32_t reg = 0;

    reg =
        (USERACCESS_GO | USERACCESS_WRITE | (phy_reg << 21) | (PHY_ADDR << 16) |
         (data & USERACCESS_DATA));
    WR_MEM_32((baseAddr + CPSW_MDIO_USERACCESS0), reg);
    success = wait_for_user_access(baseAddr);
    if (!success)
    {
        printf("\tMDIO Write Time Out to PHY 0x%02x:[0x%04x]\r\n", PHY_ADDR, phy_reg);
    }

    reg = RD_MEM_32(baseAddr + CPSW_MDIO_USERACCESS0);
    reg = reg & 0x0000ffff;
#if defined(DEBUG_FLAG)
    printf("\tPHYREG WRITE VALUE  is  = %x \r\n", reg);
#endif

    //reg = cpsw_mdio_read(phy_reg,PHY_ADDR,baseAddr);
    //EnetAppUtils_print("\nPHY REG %04x READ AFTER WRITE VAL: %04x\r\r\n",phy_reg,reg);
}

uint32_t phy_mmd_rd(uint32_t enetType,
                  uint32_t phy_addr ,
                  uint32_t mmd_number)
{
    /*Writing 0x8000 to register 0 of
      MMD3
      1.Write 0x3 to register 0xD: 0xD = 0x0003;
      (function = address; set the device address)
      2. Write 0x0 to register 0xE: 0xE = 0x0; (set the
      register offset address)
      3. Write 0x4003 to register 0xD:0xD=0x4003;
      (function = data; keep the device address)
      4. Read register 0xE:0xE == (data from register
      0x0 of MMD3)
      5. Write 0x8000 to register 0xE: 0xE = 0x8000
      (write 0x8000 to register 0x0 of MMD3 */

    /*0xd is MMD Access Control Register having field device address and function (data or address)
     * 0xE is MMD Access Address Data Register*/

    uint32_t baseAddr;
    uint32_t reg;

    if (enetType == 0)
    {
        baseAddr = CPSW_2G_MDIO_BASE_ADDR;
    }
    else
    {
        baseAddr = CPSW_9G_MDIO_BASE_ADDR;
    }

    cpsw_mdio_write(0x0D, 0x001F,phy_addr, baseAddr);
    cpsw_mdio_write(0x0E, mmd_number,phy_addr, baseAddr);
    cpsw_mdio_write(0x0D, 0x401F,phy_addr, baseAddr);
    reg=cpsw_mdio_read(0x0E, phy_addr, baseAddr);

    return reg;
}

void phy_mmd_wr(uint32_t enetType,
                uint32_t phy_addr ,
                uint32_t mmd_number,
                uint32_t val)
{
    /*Writing 0x8000 to register 0 of
      MMD3
      1.Write 0x3 to register 0xD: 0xD = 0x0003;
      (function = address; set the device address)
      2. Write 0x0 to register 0xE: 0xE = 0x0; (set the
      register offset address)
      3. Write 0x4003 to register 0xD:0xD=0x4003;
      (function = data; keep the device address)
      4. Read register 0xE:0xE == (data from register
      0x0 of MMD3)
      5. Write 0x8000 to register 0xE: 0xE = 0x8000
      (write 0x8000 to register 0x0 of MMD3 */

    /*0xd is MMD Access Control Register having field device address and function (data or address)
     * 0xE is MMD Access Address Data Register*/

    int baseAddr;

    if (enetType == 0)
    {
        baseAddr = CPSW_2G_MDIO_BASE_ADDR;
    }
    else
    {
        baseAddr = CPSW_9G_MDIO_BASE_ADDR;
    }

    cpsw_mdio_write(0x0D, 0x001F,phy_addr, baseAddr);
    cpsw_mdio_write(0x0E, mmd_number,phy_addr, baseAddr);
    cpsw_mdio_write(0x0D, 0x401F,phy_addr, baseAddr);
    cpsw_mdio_write(0x0E, val,phy_addr, baseAddr);
}

/* This shoould be called after CPSW2G is initialzied */
void cpsw_enable_mdio_2g(void)
{
    int writeValue;
    RD_MEM_32(CPSW_2G_MDIO_BASE_ADDR + CPSW_MDIO_CONTROL);
    writeValue=MDIO_CTRL_REG_CFG_VAL;
    WR_MEM_32(CPSW_2G_MDIO_BASE_ADDR + CPSW_MDIO_CONTROL,writeValue);
    //GEL_TextOut("**** MDIO ENABLED ****** \r\n");
}

/* This shoould be called after CPSW9G is initialzied */
void cpsw_enable_mdio_9g(void)
{
    int writeValue;
    RD_MEM_32(CPSW_9G_MDIO_BASE_ADDR + CPSW_MDIO_CONTROL);
    writeValue=MDIO_CTRL_REG_CFG_VAL;
    WR_MEM_32(CPSW_9G_MDIO_BASE_ADDR + CPSW_MDIO_CONTROL,writeValue);
    //GEL_TextOut("**** MDIO ENABLED ****** \r\n");
}

void mdio_print_detected_phys(int Cpsw_type)
{
    int i, baseAddr, phyAlive;

    if (Cpsw_type == 0)
    {
        baseAddr = CPSW_2G_MDIO_BASE_ADDR;
    }
    else
    {
        baseAddr = CPSW_9G_MDIO_BASE_ADDR;
    }

    phyAlive = RD_MEM_32(baseAddr + CPSW_MDIO_ALIVE);
    phyLink = RD_MEM_32(baseAddr + CPSW_MDIO_LINK);

    EnetAppUtils_print("\r\n");
    for (i = 0; i < 32U; i++)
    {
        if (1U == ENET_GET_BIT(phyAlive, i) )
        {
            EnetAppUtils_print("PHY %d (%x)\r\n",i,i);
        }
    }

    EnetAppUtils_print("PHY LINK STATUS \r\n");
    for (i = 0; i < 32U; i++)
    {
        if (1 == ENET_GET_BIT(phyLink, i) )
        {
            EnetAppUtils_print("PHY %d (%x)\r\n",i,i);
        }
    }
}
