/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <kernel/dpl/ClockP.h>
#include "ti_drivers_open_close.h"
#include <board/cdce6214/cdce6214_drv.h>
#include "board/ioexp/ioexp_tca6416.h"
#include "board/ioexp/ioexp_tca6424.h"

#define CDCE6214_I2C_ADDR    (0x68)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct {
    uint8_t reg;
    uint8_t val;
} Adc_RegCfg;

typedef struct {
    uint8_t reg;
    uint8_t val;
} Dac_RegCfg;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

CDCE6214_Handle gHCdce;

Adc_RegCfg gPcm6240RegInst0[] =
{
    /* Page select 0 */
    {0x00, 0x00},
    /* Reset */
    {0x01, 0x01},
    /* Page select 0 */
    {0x00, 0x00},
    /* Disable sleep */
    {0x02, 0x09},
    /* High impedance for unused cycles */
    {0x07, 0x31},
    /* 1 BCLK delay between FSYNC and data */
    {0x08, 0x01},
    /* CH1 at slot 0 */
    {0x0B, 0x00},
    /* CH2 at slot 4 */
    {0x0C, 0x04},
    /* CH3 at slot 1 */
    {0x0D, 0x01},
    /* CH4 at slot 5 */
    {0x0E, 0x05},
    /* Set micbias to 5v */
    {0x3b, 0x70},
    /* CH1 Microphone input */
    {0x3c, 0x10},
    /* CH2 Microphone input */
    {0x41, 0x10},
    /* CH3 Microphone input */
    {0x46, 0x10},
    /* CH4 Microphone input */
    {0x4b, 0x10},
    /* Input channel enable */
    {0x74, 0xF0},
    /* Power up micbias and power up all ADC channels */
    {0x75, 0xE0},
};

Adc_RegCfg gPcm6240RegInst1[] =
{
    /* Page select 0 */
    {0x00, 0x00},
    /* Reset */
    {0x01, 0x01},
    /* Page select 0 */
    {0x00, 0x00},
    /* Disable sleep */
    {0x02, 0x09},
    /* High impedance for unused cycles */
    {0x07, 0x31},
    /* 1 BCLK delay between FSYNC and data */
    {0x08, 0x01},
    /* CH1 at slot 2 */
    {0x0B, 0x02},
    /* CH2 at slot 6 */
    {0x0C, 0x06},
    /* CH3 at slot 3 */
    {0x0D, 0x03},
    /* CH4 at slot 7 */
    {0x0E, 0x07},
    /* Set micbias to 5v */
    {0x3b, 0x70},
    /* CH1 Microphone input */
    {0x3c, 0x10},
    /* CH2 Microphone input */
    {0x41, 0x10},
    /* CH3 Microphone input */
    {0x46, 0x10},
    /* CH4 Microphone input */
    {0x4b, 0x10},
    /* Input channel enable */
    {0x74, 0xF0},
    /* Power up micbias and power up all ADC channels */
    {0x75, 0xE0},
};

Dac_RegCfg gTad5212RegInst0[] =
{
    /* Page select 0 */
    {0x00, 0x00},
    /* Disable sleep */
    {0x02, 0x01},
    /* TDM, 32b mode */
    {0x1A, 0x30},
    /* 1b delay between FSYNC and data */
    {0x26, 0x01},
    /* PASI CH1 is TDM slot 0 */
    {0x28, 0x20},
    /* PASI CH2 is TDM slot 4 */
    {0x29, 0x24},
    /* CH1 - Input from DAC, Mono single ended output at OUT1P */
    {0x64, 0x28},
    /* CH1 - Headphone with 4ohm impedence, 0dB gain */
    {0x65, 0x60},
    /* CH1 - 0dB gain */
    {0x67, 0xC9},
    /* CH2 - Input from DAC, Mono single ended output at OUT2P */
    {0x6B, 0x28},
    /* CH2 - Headphone with 4ohm impedence, 0dB gain */
    {0x6C, 0x60},
    /* CH2 - 0dB gain */
    {0x6E, 0xC9},
    /* Enable output CH1 CH2 */
    {0x76, 0x0C},
    /* Page Select 0 */
    {0x00, 0x00},
    {0x78, 0x40},

};

Dac_RegCfg gTad5212RegInst1[] =
{
    /* Page select 0 */
    {0x00, 0x00},
    /* Disable sleep */
    {0x02, 0x01},
    /* TDM, 32b mode */
    {0x1A, 0x30},
    /* 1b delay between FSYNC and data */
    {0x26, 0x01},
    /* PASI CH1 is TDM slot 1 */
    {0x28, 0x21},
    /* PASI CH2 is TDM slot 5 */
    {0x29, 0x25},
    /* CH1 - Input from DAC, Mono single ended output at OUT1P */
    {0x64, 0x28},
    /* CH1 - Headphone with 4ohm impedence, 0dB gain */
    {0x65, 0x60},
    /* CH1 - 0dB gain */
    {0x67, 0xC9},
    /* CH2 - Input from DAC, Mono single ended output at OUT2P */
    {0x6B, 0x28},
    /* CH2 - Headphone with 4ohm impedence, 0dB gain */
    {0x6C, 0x60},
    /* CH2 - 0dB gain */
    {0x6E, 0xC9},
    /* Enable output CH1 CH2 */
    {0x76, 0x0C},
    /* Page Select 0 */
    {0x00, 0x00},
    {0x78, 0x40},

};

Dac_RegCfg gTad5212RegInst2[] =
{
    /* Page select 0 */
    {0x00, 0x00},
    /* Disable sleep */
    {0x02, 0x01},
    /* TDM, 32b mode */
    {0x1A, 0x30},
    /* 1b delay between FSYNC and data */
    {0x26, 0x01},
    /* PASI CH1 is TDM slot 2 */
    {0x28, 0x22},
    /* PASI CH1 is TDM slot 6 */
    {0x29, 0x26},
    /* CH1 - Input from DAC, Mono single ended output at OUT1P */
    {0x64, 0x28},
    /* CH1 - Headphone with 4ohm impedence, 0dB gain */
    {0x65, 0x60},
    /* CH1 - 0dB gain */
    {0x67, 0xC9},
    /* CH2 - Input from DAC, Mono single ended output at OUT2P */
    {0x6B, 0x28},
    /* CH2 - Headphone with 4ohm impedence, 0dB gain */
    {0x6C, 0x60},
    /* CH2 - 0dB gain */
    {0x6E, 0xC9},
    /* Enable output CH1 CH2 */
    {0x76, 0x0C},
    /* Page Select 0 */
    {0x00, 0x00},
    {0x78, 0x40},

};

Dac_RegCfg gTad5212RegInst3[] =
{
    /* Page select 0 */
    {0x00, 0x00},
    /* Disable sleep */
    {0x02, 0x01},
    /* TDM, 32b mode */
    {0x1A, 0x30},
    /* 1b delay between FSYNC and data */
    {0x26, 0x01},
    /* PASI CH1 is TDM slot 3 */
    {0x28, 0x23},
    /* PASI CH1 is TDM slot 7 */
    {0x29, 0x27},
    /* CH1 - Input from DAC, Mono single ended output at OUT1P */
    {0x64, 0x28},
    /* CH1 - Headphone with 4ohm impedence, 0dB gain */
    {0x65, 0x60},
    /* CH1 - 0dB gain */
    {0x67, 0xC9},
    /* CH2 - Input from DAC, Mono single ended output at OUT2P */
    {0x6B, 0x28},
    /* CH2 - Headphone with 4ohm impedence, 0dB gain */
    {0x6C, 0x60},
    /* CH2 - 0dB gain */
    {0x6E, 0xC9},
    /* Enable output CH1 CH2 */
    {0x76, 0x0C},
    /* Page Select 0 */
    {0x00, 0x00},
    {0x78, 0x40},
};

/*
  Base Configuration of CDCE
  1. Select SECREF (24.576MHZ Crystal)
  2. Select 99 as Integer, 500000 as Num and Den for the Fraction
     This Effectively sets the PLL Frequency to 2457.6MHz.
  3. Enable DCO Mode
  4. Enable Register Mode for DCO.
  5. Set the DCO Step Size to 10.
  6. Select the Clock Divider to enable 24.576MHZ on OUT1.
  7. Disable OUT2, OUT3, OUT4.
*/
static const CDCE6214_RegCfg gCdceBaseConfig[CDCE6214_NUM_REGISTER] = {
    {85,  0x0000},
    {84,  0x0000},
    {83,  0x0000},
    {82,  0x0000},
    {81,  0x0004},
    {80,  0x0000},
    {79,  0x0208},
    {78,  0x0000},
    {77,  0x0000},
    {76,  0x0008},
    {75,  0x0008},
    {74,  0xA181},
    {73,  0x2000},
    {72,  0x0006},
    {71,  0x0000},
    {70,  0x0008},
    {69,  0xA181},
    {68,  0x2000},
    {67,  0x0006},
    {66,  0x0006},
    {65,  0x0808},
    {64,  0xA181},
    {63,  0x1000},
    {62,  0x0014},
    {61,  0x0000},
    {60,  0x6008},
    {59,  0x8008},
    {58,  0x502C},
    {57,  0x1000},
    {56,  0x0019},
    {55,  0x001E},
    {54,  0x3400},
    {53,  0x0069},
    {52,  0x5000},
    {51,  0x40C0},
    {50,  0x07C0},
    {49,  0x0013},
    {48,  0x23C7},
    {47,  0x0380},
    {46,  0x0000},
    {45,  0x4F80},
    {44,  0x0318},
    {43,  0x000A},
    {42,  0x0002},
    {41,  0x0000},
    {40,  0x0000},
    {39,  0x0000},
    {38,  0x0000},
    {37,  0x0000},
    {36,  0x0000},
    {35,  0x0000},
    {34,  0x0007},
    {33,  0xA120},
    {32,  0x0007},
    {31,  0xA120},
    {30,  0x0063},
    {29,  0x0000},
    {28,  0x0000},
    {27,  0x0005},
    {26,  0x0000},
    {25,  0x0401},
    {24,  0x0024},
    {23,  0x0000},
    {22,  0x0000},
    {21,  0x0000},
    {20,  0x0000},
    {19,  0x0000},
    {18,  0x0000},
    {17,  0x26C4},
    {16,  0x921F},
    {15,  0xA037},
    {14,  0x0000},
    {13,  0x0000},
    {12,  0x0000},
    {11,  0x0000},
    {10,  0x0000},
    {9,   0x0000},
    {8,   0x0000},
    {7,   0x0C0D},
    {6,   0x0000},
    {5,   0x0008},
    {4,   0x00E0},
    {3,   0x0018},
    {2,   0x0002},
    {1,   0x2310},
    {0,   0x1010},
};


/* ========================================================================== */
/*                          Function Declerations                             */
/* ========================================================================== */

/* Configure PCM6240 */
int32_t Board_adcConfig(I2C_Handle handle, uint8_t devAddr, uint32_t instNum);
/* Configure TAD5212 */
int32_t Board_dacConfig(I2C_Handle handle, uint8_t devAddr);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Board_codecConfig(void)
{
    int32_t status = SystemP_SUCCESS;

    /* Configure TAD5212 instances */
    {
        I2C_Handle      i2cHandle;
        i2cHandle = gI2cHandle[CONFIG_I2C0];

        status = Board_dacConfig(i2cHandle, 0x50);
        DebugP_assert(status == SystemP_SUCCESS);

        status = Board_dacConfig(i2cHandle, 0x51);
        DebugP_assert(status == SystemP_SUCCESS);

        status = Board_dacConfig(i2cHandle, 0x52);
        DebugP_assert(status == SystemP_SUCCESS);

        status = Board_dacConfig(i2cHandle, 0x53);
        DebugP_assert(status == SystemP_SUCCESS);
    }

    /* Configure PCM6240 instances */
    {
        I2C_Handle      i2cHandle;
        i2cHandle = gI2cHandle[CONFIG_I2C0];

        status = Board_adcConfig(i2cHandle, 0x48, 0);
        DebugP_assert(status == SystemP_SUCCESS);

        status = Board_adcConfig(i2cHandle, 0x49, 1);
        DebugP_assert(status == SystemP_SUCCESS);
    }

    return status;
}

int32_t Board_dacConfig(I2C_Handle handle, uint8_t devAddr)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t count = 0;
    uint32_t i = 0;
    I2C_Transaction i2cTransaction;
    uint8_t txBuffer[2];

    status = I2C_probe(handle, devAddr);
    DebugP_assert(status == SystemP_SUCCESS);

    ClockP_usleep(100);

    if(devAddr == 0x50)
    {
        count = sizeof(gTad5212RegInst0)/sizeof(Dac_RegCfg);
    }
    if(devAddr == 0x51)
    {
        count = sizeof(gTad5212RegInst1)/sizeof(Dac_RegCfg);
    }
    if(devAddr == 0x52)
    {
        count = sizeof(gTad5212RegInst2)/sizeof(Dac_RegCfg);
    }
    if(devAddr == 0x53)
    {
        count = sizeof(gTad5212RegInst3)/sizeof(Dac_RegCfg);
    }

    for(i = 0; i < count; i++)
    {
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf   = txBuffer;
        i2cTransaction.writeCount = 2;
        i2cTransaction.targetAddress = devAddr;
        if(devAddr == 0x50)
        {
            txBuffer[0] = gTad5212RegInst0[i].reg;
            txBuffer[1] = gTad5212RegInst0[i].val;
        }
        if(devAddr == 0x51)
        {
            txBuffer[0] = gTad5212RegInst1[i].reg;
            txBuffer[1] = gTad5212RegInst1[i].val;
        }
        if(devAddr == 0x52)
        {
            txBuffer[0] = gTad5212RegInst2[i].reg;
            txBuffer[1] = gTad5212RegInst2[i].val;
        }
        if(devAddr == 0x53)
        {
            txBuffer[0] = gTad5212RegInst3[i].reg;
            txBuffer[1] = gTad5212RegInst3[i].val;
        }

        status = I2C_transfer(handle, &i2cTransaction);

        if(status != SystemP_SUCCESS)
        {
            break;
        }

        ClockP_usleep(100);
    }

    return status;
}

int32_t Board_adcConfig(I2C_Handle handle, uint8_t devAddr, uint32_t inst)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t count = 0;
    uint32_t i = 0;
    I2C_Transaction i2cTransaction;
    uint8_t txBuffer[2];

    status = I2C_probe(handle, devAddr);
    DebugP_assert(status == SystemP_SUCCESS);

    ClockP_usleep(200);

    if(inst == 0)
    {
        count = sizeof(gPcm6240RegInst0)/sizeof(gPcm6240RegInst0[0]);
    }
    else if(inst == 1)
    {
        count = sizeof(gPcm6240RegInst1)/sizeof(gPcm6240RegInst1[0]);
    }

    for(i = 0; i < count; i++)
    {
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf   = txBuffer;
        i2cTransaction.writeCount = 2;
        i2cTransaction.targetAddress = devAddr;
        if(inst == 0)
        {
            txBuffer[0] = gPcm6240RegInst0[i].reg;
            txBuffer[1] = gPcm6240RegInst0[i].val;
        }
        else if(inst == 1)
        {
            txBuffer[0] = gPcm6240RegInst1[i].reg;
            txBuffer[1] = gPcm6240RegInst1[i].val;
        }

        status = I2C_transfer(handle, &i2cTransaction);

        if(status != SystemP_SUCCESS)
        {
            break;
        }

        ClockP_usleep(200);
    }

    return status;
}

int32_t Board_CdceConfig(void)
{
    int32_t retval = SystemP_SUCCESS;
    I2C_Handle i2cHandle = gI2cHandle[CONFIG_I2C0];
    gHCdce = CDCE6214_open(i2cHandle, CDCE6214_I2C_ADDR, \
                    CDCE6214_REF_FREQ_24_576MHZ, gCdceBaseConfig);
    if (gHCdce != NULL)
    {
        retval = CDCE6214_commitWsrToCdce(gHCdce);
    }
    else
    {
        retval = SystemP_FAILURE;
    }

    return retval;
}

int32_t Board_CdceFineTuneFreq(double relPPM)
{
    return CDCE6214_fineTuneFrequency(gHCdce, relPPM);
}

int32_t Board_MuxSelMcASP4(void)
{
    int status;
    uint32_t pinNum = 3;

    TCA6416_Config  TCA6416_Config;
    TCA6416_Params  tca6416Params;
    TCA6416_Params_init(&tca6416Params);
    tca6416Params.i2cInstance = CONFIG_I2C0;
    tca6416Params.i2cAddress = 0x20;
    TCA6416_open(&TCA6416_Config, &tca6416Params);


    status = TCA6416_setOutput(
                    &TCA6416_Config,
                    pinNum,
                    TCA6416_OUT_STATE_LOW);

    /* Configure as output  */
    status += TCA6416_config(
                    &TCA6416_Config,
                    pinNum,
                    TCA6416_MODE_OUTPUT);

    status = TCA6416_setOutput(
                    &TCA6416_Config,
                    pinNum,
                    TCA6416_OUT_STATE_HIGH);

    /* Todo: Implement Close function. */
    return status;
}

int32_t Board_MuxSelPhyRefClk(void)
{
    /*
        Configure the clock mux using IO Expander
        Ref: PROC190E1 (AM275-EVM Schematics)

        To connect RGMII1_BCLK to AUDIO_EXT_REFCLK2
        IO Expander Addr: 0x22
        AUDIO_EXT_REFCLK2_SO -> P24 -> 0(LOW)
        AUDIO_EXT_REFCLK2_S1 -> P25 -> 1(HIGH)
    */
   const uint32_t I2C_ADDRESS = 0x22;
   const uint32_t REFCLK2_S0  = (8 * 2) + 4; // P24
   const uint32_t REFCLK2_S1  = (8 * 2) + 5; // P25

    int32_t status;
    /* IO Expander Config Object */
    static TCA6424_Config IOExp_Config;

    TCA6424_Params  IOExp_Params = {
        .i2cInstance = CONFIG_I2C0,
        .i2cAddress  = I2C_ADDRESS,
    };
    /* open the IO Expander object */
    status = TCA6424_open(&IOExp_Config, &IOExp_Params);

    if (status == SystemP_SUCCESS)
    {
        status = TCA6424_config(&IOExp_Config,
            REFCLK2_S0, TCA6424_MODE_OUTPUT);
        status += TCA6424_config(&IOExp_Config,
            REFCLK2_S1, TCA6424_MODE_OUTPUT);
    }

    if (status == SystemP_SUCCESS)
    {
        status = TCA6424_setOutput(&IOExp_Config,
            REFCLK2_S0, TCA6424_OUT_STATE_LOW);
        status += TCA6424_setOutput(&IOExp_Config,
            REFCLK2_S1, TCA6424_OUT_STATE_HIGH);
    }

    TCA6424_close(&IOExp_Config);

    return status;
}


