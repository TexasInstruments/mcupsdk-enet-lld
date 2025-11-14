#ifndef _GPIO_SM_H_
#define _GPIO_SM_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint32_t gpioBaseAddr;
    uint32_t pinNum;
    uint32_t counter;
    uint32_t pulseWidth;
    uint32_t Periodicity;
    bool isRxMode;
    bool isEnabled;
}gpioStateMachine;

void gpio_sm_init(gpioStateMachine* gsm, uint32_t gpioPinAddr, uint32_t gpioPinNum, uint32_t pinDir);

void gpio_sm_setState(gpioStateMachine* gsm, bool state);

void gpio_sm_spin(gpioStateMachine* gsm, uint32_t* buffer, uint32_t sampleSize);

#endif /* _GPIO_SM_H_ */
