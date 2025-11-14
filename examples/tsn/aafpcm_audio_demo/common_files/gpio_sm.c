#include "gpio_sm.h"
#include "drivers/gpio.h"
#include <kernel/dpl/AddrTranslateP.h>

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
