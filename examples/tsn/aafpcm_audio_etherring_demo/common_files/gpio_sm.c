/*
 *  Copyright (c) Texas Instruments Incorporated 2025
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
/*                              Include Files                                 */
/* ========================================================================== */

#include "gpio_sm.h"
#include "drivers/gpio.h"
#include <kernel/dpl/AddrTranslateP.h>

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void gpio_sm_init(gpioStateMachine* gsm, uint32_t gpioPinAddr, uint32_t gpioPinNum, uint32_t pinDir)
{
    gsm->gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(gpioPinAddr);
    gsm->pinNum   = gpioPinNum;
    GPIO_setDirMode(gpioPinAddr, gpioPinNum, pinDir);
}

void gpio_sm_setState(gpioStateMachine* gsm, bool state)
{
    if (state)
    {
        GPIO_pinWriteHigh(gsm->gpioBaseAddr, gsm->pinNum);
    }
    else
    {
        GPIO_pinWriteLow(gsm->gpioBaseAddr, gsm->pinNum);
    }
}

void gpio_sm_spin(gpioStateMachine* gsm, uint32_t* buffer, uint32_t sampleSize)
{
    if (gsm->isEnabled == true)
    {
        if (gsm->isRxMode == true)
        {
            if (gsm->counter == 0)
            {
                int i = 0;
                gpio_sm_setState(gsm, true);
                for (i = 0; i < sampleSize; i++)
                {
                    buffer[8*i+3] = 0x1234ABCD;
                }
            }
            else if (gsm->counter == gsm->pulseWidth)
            {
                gpio_sm_setState(gsm, false);
            }
            else {}
            gsm->counter += 1;
            gsm->counter %= gsm->Periodicity;
        }
        else
        {
            bool isMagicNumber = false;
            int i;
            /* Browse through Channel 3 to find the magic number. */
            for (i = 0; i < sampleSize; i++)
            {
                if (buffer[8*i+3] == 0x1234ABCD)
                {
                    /* Found Magic Number. */
                    isMagicNumber = true;
                }
            }
            if (isMagicNumber == true)
            {
                gsm->counter = gsm->pulseWidth;
                gpio_sm_setState(gsm, true);
            }

            if (gsm->counter == 1)
            {
                gsm->counter = 0;
                gpio_sm_setState(gsm, false);
            }

            if (gsm->counter > 0)
            {
                gsm->counter -= 1;
            }
        }
    }
}
