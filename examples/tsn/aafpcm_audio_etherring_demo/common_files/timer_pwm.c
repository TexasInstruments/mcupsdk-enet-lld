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
#include "stdint.h"
#include "ti_drivers_config.h"
#include "kernel/dpl/TimerP.h"
#include "ti_dpl_config.h"
#include <drivers/pinmux.h>
#include <drivers/soc.h>
#include <drivers/sciclient.h>
#include <kernel/dpl/ClockP.h>

/* ========================================================================== */
/*                          Macros and Definitions                            */
/* ========================================================================== */

#define TIMER_PWM_CTRL_MMR0_CFG0_TIMERIO_CTRL      (0x0010429C)
#define TIMER_PWM_TIMERIO_DRIVE_BY_TIMER           (0x01)
#define TIMER_PWM_PWM_TIMER_BASE_ADDRESS           (0x02410000)

#define TIMER_IRQ_EOI           (0x20u)
#define TIMER_IRQ_STATUS_RAW    (0x24u)
#define TIMER_IRQ_STATUS        (0x28u)
#define TIMER_IRQ_INT_ENABLE    (0x2Cu)
#define TIMER_IRQ_INT_DISABLE   (0x30u)
#define TIMER_TCLR              (0x38u)
#define TIMER_TCRR              (0x3cu)
#define TIMER_TLDR              (0x40u)
#define TIMER_TWPS              (0x48u)
#define TIMER_TMAR              (0x4Cu)
#define TIMER_TCLR_PEND_SHIFT   (0U)
#define TIMER_TCRR_PEND_SHIFT   (1U)
#define TIMER_TLDR_PEND_SHIFT   (2U)
#define TIMER_TMAR_PEND_SHIFT   (4U)
#define TIMER_TCLR_PEND_MASK    (1U << TIMER_TCLR_PEND_SHIFT)
#define TIMER_TCRR_PEND_MASK    (1U << TIMER_TCRR_PEND_SHIFT)
#define TIMER_TLDR_PEND_MASK    (1U << TIMER_TLDR_PEND_SHIFT)
#define TIMER_TMAR_PEND_MASK    (1U << TIMER_TMAR_PEND_SHIFT)
#define TIMER_CLOCK_FREQ        (25000000U)

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static int32_t timerPwm_configPinMux(void)
{
    static Pinmux_PerCfg_t gPinMuxMainDomainCfg[] = {

        /*  pin config */
        /* PIN_GPIO1_49 -> TIMER_IO7 (V2) */
        {
            PIN_GPIO1_49,
            ( PIN_MODE(2) | PIN_PULL_DISABLE)
        },

        {PINMUX_END, 0U}
    };

    Pinmux_config(gPinMuxMainDomainCfg, PINMUX_DOMAIN_ID_MAIN);
    return SystemP_SUCCESS;
}

static void timerPwm_setupTimer(uint32_t baseAddr, uint32_t freq)
{
    volatile uint32_t* addr;
    TimerP_stop(baseAddr);
    TimerP_clearOverflowInt(baseAddr);

    volatile uint32_t *twps_addr = (volatile uint32_t *)(baseAddr + TIMER_TWPS);

    uint32_t tclr = 0;
    /* Select Auto Reload. */
    tclr |= (1<<1U);
    /* Select Trigger Output Mode. Overflow and Match Trigger. */
    tclr |= (2<<10U);
    /* Select PWM Mode - Toggle Mode. */
    tclr |= (1 << 12U);
    /* Configure PWM output Pin Default Value. */
    tclr |= (0 << 7U);

    /* Commit the TCLR */
    while((*twps_addr & TIMER_TCLR_PEND_MASK) == TIMER_TCLR_PEND_MASK)
    {}
    addr = (volatile uint32_t *)(baseAddr + TIMER_TCLR);
    *addr = tclr;
    while((*twps_addr & TIMER_TCLR_PEND_MASK) == TIMER_TCLR_PEND_MASK)
    {}

    /*
        Load the Timer Load Value
        Set the output Frequency as 10Hz.
    */
    uint32_t tldr = 0xFFFFFFFF - (TIMER_CLOCK_FREQ/freq);
    while((*twps_addr & TIMER_TLDR_PEND_MASK) == TIMER_TLDR_PEND_MASK)
    {}
    addr = (volatile uint32_t *)(baseAddr + TIMER_TLDR);
    *addr = tldr;
    while((*twps_addr & TIMER_TLDR_PEND_MASK) == TIMER_TLDR_PEND_MASK)
    {}

    /* Load the Timer Compare Value. */
    uint32_t tmar = tldr + (0.5)*(0xFFFFFFFF - tldr);
    while((*twps_addr & TIMER_TMAR_PEND_MASK) == TIMER_TMAR_PEND_MASK)
    {}
    addr = (volatile uint32_t *)(baseAddr + TIMER_TMAR);
    *addr = tmar;
    while((*twps_addr & TIMER_TMAR_PEND_MASK) == TIMER_TMAR_PEND_MASK)
    {}

    /* set timer current count value equal to reload value. */
    while((*twps_addr & TIMER_TCRR_PEND_MASK) == TIMER_TCRR_PEND_MASK)
    {}
    addr = (volatile uint32_t *)(baseAddr + TIMER_TCRR);
    *addr = tldr;
    while((*twps_addr & TIMER_TCRR_PEND_MASK) == TIMER_TCRR_PEND_MASK)
    {}

    /* Enable the Compare. */
    while((*twps_addr & TIMER_TCLR_PEND_MASK) == TIMER_TCLR_PEND_MASK)
    {}
    addr = (volatile uint32_t *)(baseAddr + TIMER_TCLR);
    *addr |= (1U<<6);
    while((*twps_addr & TIMER_TCLR_PEND_MASK) == TIMER_TCLR_PEND_MASK)
    {}

    /* Start the Timer. */
    TimerP_start(baseAddr);
}

void timerPwm_configTestSignal(uint32_t freq)
{
    /* Configure Pinmux */
    timerPwm_configPinMux();

    /* Configure TIMERIOx to be driver by TIMERy */
    volatile uint32_t *ctrl_mmr_addr = \
        (volatile uint32_t*)AddrTranslateP_getLocalAddr(TIMER_PWM_CTRL_MMR0_CFG0_TIMERIO_CTRL);
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, 1);
    CSL_REG32_WR(ctrl_mmr_addr, TIMER_PWM_TIMERIO_DRIVE_BY_TIMER);
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, 1);

    timerPwm_setupTimer(TIMER_PWM_PWM_TIMER_BASE_ADDRESS, freq);
}