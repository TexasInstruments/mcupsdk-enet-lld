/*
 *  Copyright (C) Texas Instruments Incorporated 2026
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
 * \file  tsndemo_pacing.c
 *
 * \brief TSN demo talker - express generator pacing, phase-locked to the
 *        EST gate schedule when EST is on (the same mechanism
 *        examples/ether_ring uses on AM263x-lp):
 *
 *        - GENF_CLK (CPTS GENF0): free-running square wave, routed through
 *          the AM261x CTRLMMR clock-source mux directly into CONFIG_TIMER0's
 *          (RTI1) clock input, so the timer is FREQUENCY-locked to the
 *          gPTP-disciplined CPTS clock and cannot drift against the EST
 *          gates.
 *        - GENF_START (CPTS GENF1): one-shot-style compare at
 *          (baseTime + windowOffset - leadTime) - matching
 *          examples/ether_ring's own GENF1 compare exactly (no cycle-time
 *          term). A -cycleTime-leadTime variant was tried (theory: RTI's
 *          first compare/interrupt after TimerP_start() only fires one
 *          full cycleTime later, never immediately - TimerP_setupPriv()/
 *          TimerP_startPriv(), source/kernel/nortos/dpl/common/
 *          TimerP_rti_priv.c), but made no measurable difference on
 *          hardware to the window0-miss rate - see TsnDemoPacing_
 *          startGenf()'s own comment for the full history. GENF1's edge
 *          reaches the
 *          CPTS hardware-push logic via the CPSW's timesync crossbar
 *          (soc_timesync_xbar1 in example.syscfg, CPTS_GENF1 ->
 *          CPSW_MODULE_0) - that xbar routing is what CPSW_CPTS_HWPUSH_FIRST
 *          actually listens on; the callback below only fires once it's
 *          configured. When it does fire, it (re)configures and starts
 *          CONFIG_TIMER0, PHASE-locking it to the schedule.
 *          NOTE (open issue, see TSNDEMO_TIMER_CLK_PERIOD_NS below): RTI1's
 *          own period can't exactly reproduce the real (live-readback) EST
 *          cycle length, so it drifts out of phase with the true gate
 *          schedule as a run progresses. Two attempted fixes for that -
 *          raising RTI1's input clock to match the CPTS tick exactly, and
 *          periodically re-syncing via repeated GENF1 hw-push edges - were
 *          each tried and reverted after making things worse on real
 *          hardware (data abort, and a regressed RTT average,
 *          respectively). Deliberately back to this simplest one-shot
 *          form until a working fix is found - do not re-attempt either
 *          of those two approaches without new hardware evidence.
 *        - CONFIG_TIMER0's own Hwi/ISR wiring (ti_dpl_config.c, generated
 *          from example.syscfg) is reused unchanged in both paths below -
 *          only its clock source and reload period are reprogrammed here.
 *          There is no second, independent timer/interrupt object.
 *
 *        Fallback (baseTimeNs == 0, i.e. EST off): CONFIG_TIMER0 is started
 *        as-is, on whatever clock source example.syscfg configured at boot
 *        (an ordinary SoC peripheral clock) - this is today's plain,
 *        non-gate-locked behavior, unchanged, and is also what a failed
 *        GENF configuration falls back to.
 *
 *        AM261x-specific notes (never copy tick counts or register
 *        addresses between SoCs without re-checking):
 *        - The CTRLMMR clock-source-mux address for CONFIG_TIMER0/RTI1 is
 *          0x53208144 (CSL_MSS_RCM_RTI1_CLK_SRC_SEL, am261x/cslr_mss_rcm.h),
 *          cross-checked against SysConfig's own generated
 *          CONFIG_TIMER0_CLOCK_SRC_MUX_ADDR. A previous AM261x port of this
 *          file copied AM263x's address (0x53208118) unverified, which lands
 *          on an undefined register gap on AM261x and measured ~100x off -
 *          and never added the soc_timesync_xbar1 routing at all, so
 *          GENF_START could never have reached the hardware-push logic
 *          either. Both are fixed in example.syscfg + this file.
 *        - The CPTS reference clock differs by board variant, not just by
 *          SoC: am261x-lp runs CPTS at 250 MHz (4 ns/tick),
 *          am261x-som at 200 MHz (5 ns/tick) - see each board's
 *          enet_cpsw1.cptsRftClkFreq in example.syscfg. TSNDEMO_CPTS_TICK_NS
 *          below is selected by a per-board macro (Makefile
 *          DEFINES_common), not a single shared AM261x constant.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <string.h>
#include "tsndemo_talker.h"

#include <kernel/dpl/TimerP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/soc.h>
#include "ti_dpl_config.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#ifndef TSNDEMO_USE_GENF_PACING
#define TSNDEMO_USE_GENF_PACING     (1)
#endif

/*! CPTS tick period in ns = cptsRftClkFreq enum + 1 (see
 *  CpswCpts_ioctl_handler_.../cpsw_est_ioctl.c for the same rule). Must
 *  match each board's own enet_cpsw1.cptsRftClkFreq in example.syscfg -
 *  am261x-lp (AM2612, 250 MHz) and am261x-som (AM2611, 200 MHz) differ.
 *  AM261x-only: TSNDEMO_NS_TO_GENF_LEN() is only ever called from
 *  TsnDemoPacing_startGenf(), itself compiled only under
 *  TSNDEMO_USE_GENF_PACING==1 below, which is forced to 0 for every SoC
 *  but AM261x - so non-AM261x boards (am263x-lp/cc, am263px-lp-dp83869)
 *  need neither a board macro nor this constant at all. */
#if defined(SOC_AM261X)
#if defined(TSNDEMO_BOARD_AM261X_LP)
#define TSNDEMO_CPTS_TICK_NS        (4U)    /* CPSW_CPTS_RFTCLK_FREQ_250MHZ */
#elif defined(TSNDEMO_BOARD_AM261X_SOM)
#define TSNDEMO_CPTS_TICK_NS        (5U)    /* CPSW_CPTS_RFTCLK_FREQ_200MHZ */
#else
#error "tsndemo_pacing.c: define TSNDEMO_BOARD_AM261X_LP or _SOM (Makefile) " \
       "so TSNDEMO_CPTS_TICK_NS matches this board's cptsRftClkFreq - " \
       "never assume a shared default, see the file header."
#endif

#define TSNDEMO_NS_TO_GENF_LEN(ns)  ((uint32_t)((ns) / TSNDEMO_CPTS_TICK_NS))
#endif /* SOC_AM261X */

#define TSNDEMO_GENF_CLK_IDX        (0U)
#define TSNDEMO_GENF_START_IDX      (1U)
#define TSNDEMO_PACE_HWPUSH         (CPSW_CPTS_HWPUSH_FIRST)

/*!
 * GENF_CLK output period. 20 ns -> 50 MHz timer input clock.
 *
 * KNOWN OPEN ISSUE: TimerP_setupPriv() (source/kernel/nortos/dpl/common/
 * TimerP_rti_priv.c) truncates cycleTimeNs to an RTI tick count via integer
 * division, so whenever cycleTimeNs isn't an exact multiple of this period
 * (confirmed on hardware: the live EST cycle read back from GET_OPER_LIST
 * often isn't - e.g. 125008 ns, 125008%20=8), RTI1 runs measurably short of
 * the real gate period every cycle, drifting out of phase with the true EST
 * schedule as a run progresses. Three fixes were tried, all made things
 * worse on real hardware:
 *   - Raising this to match the CPTS tick exactly (4 ns/250 MHz on
 *     am261x-lp, eliminating the truncation) produced a data abort at
 *     CONFIG_TIMER0_BASE_ADDR - RTI1's clock path isn't rated for that
 *     input frequency.
 *   - Dithering the reload register every tick (Bresenham/DDS trick)
 *     worsened the RTT average, likely because TimerP_setupPriv() stops the
 *     timer before touching its registers, and this rewrote the live
 *     compare register while running.
 *   - Periodically re-syncing via repeated GENF1 hw-push edges (instead of
 *     one-shot) also worsened the RTT average, similarly to dithering -
 *     likely variable ISR-response latency per re-sync edge adding phase
 *     noise worse than the slow, deterministic drift it aimed to bound.
 * Deliberately left AS-IS (20 ns/50 MHz, one-shot GENF1 arm, truncation
 * uncorrected) until a working fix is found - do not re-attempt any of the
 * above without new hardware evidence.
 */
#define TSNDEMO_TIMER_CLK_PERIOD_NS (20U)
#define TSNDEMO_TIMER_INPUT_CLK_HZ  (1000000000U / TSNDEMO_TIMER_CLK_PERIOD_NS)

#if defined(SOC_AM261X) && (TSNDEMO_USE_GENF_PACING == 1)
/*! CSL_MSS_RCM_RTI1_CLK_SRC_SEL (source/drivers/hw_include/am261x/cslr_mss_rcm.h) -
 *  same register SysConfig's generated ti_dpl_config.c pokes at boot
 *  (CONFIG_TIMER0_CLOCK_SRC_MUX_ADDR) to select the default clock source. */
#define TSNDEMO_TIMER_CLKSRC_MUX_ADDR    (0x53208144U)
#define TSNDEMO_TIMER_CLKSRC_GENF0_VAL   (0x777U)
/*! CONFIG_TIMER0_CLOCK_SRC_WUCPUCLK in ti_dpl_config.c - not exposed via a
 *  header, mirrored here so pacing can revert to it on stop(). */
#define TSNDEMO_TIMER_CLKSRC_DEFAULT_VAL (0x0U)
#else
#undef TSNDEMO_USE_GENF_PACING
#define TSNDEMO_USE_GENF_PACING     (0)
#endif

/* ========================================================================== */
/*                            Local Variables                                 */
/* ========================================================================== */

static struct
{
    TsnDemo_PacingTickCb tickCb;
    volatile bool        running;
    volatile bool        timerStarted;
    bool                 genfLocked;
    /*! Set only inside TsnDemoPacing_hwPushCb() - the one place that
     *  actually switches CONFIG_TIMER0/RTI1's clock-source mux to GENF0.
     *  genfLocked can't be reused for this: it only becomes true once the
     *  *entire* startGenf() sequence succeeds, not when the mux switch
     *  itself happens. TsnDemoPacing_stop() gates the mux-revert +
     *  TimerP_setup() call on THIS flag, not genfLocked - see stop(). */
    volatile bool        clkMuxOnGenf;
    uint32_t             cycleTimeNs;
} gPacing;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static int32_t TsnDemoPacing_setGenf(uint32_t index, uint32_t lengthTicks,
                                     uint64_t compareNs, bool polarityInv)
{
    Enet_IoctlPrms prms;
    CpswCpts_SetFxnGenInArgs setGenFInArgs;
    int32_t status;

    memset(&setGenFInArgs, 0, sizeof(setGenFInArgs));
    setGenFInArgs.index       = index;
    setGenFInArgs.length      = lengthTicks;
    setGenFInArgs.compare     = compareNs;
    setGenFInArgs.polarityInv = polarityInv;
    setGenFInArgs.ppmVal      = 0U;
    setGenFInArgs.ppmDir      = CPSW_CPTS_GENF_PPM_ADJDIR_INCREASE;
    setGenFInArgs.ppmMode     = ENET_TIMESYNC_ADJMODE_DISABLE;

    ENET_IOCTL_SET_IN_ARGS(&prms, &setGenFInArgs);
    ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId, CPSW_CPTS_IOCTL_SET_GENF,
               &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("pacing: GENF%u config failed: %d\r\n", index,
                           status);
    }
    return status;
}

/*! \brief Reprogram CONFIG_TIMER0's clock source and reload period in
 *         place. Reuses the base address and Hwi SysConfig already set up
 *         (ti_dpl_config.c) - TimerP_setup() internally stops the timer
 *         before reconfiguring, so this is safe to call whether or not it
 *         is currently running. */
static void TsnDemoPacing_timerReconfig(uint32_t inputClkHz, uint32_t muxVal)
{
    TimerP_Params timerParams;

#if (TSNDEMO_USE_GENF_PACING == 1)
    {
        volatile uint32_t *muxReg = (volatile uint32_t *)
            AddrTranslateP_getLocalAddr(TSNDEMO_TIMER_CLKSRC_MUX_ADDR);
        volatile uint32_t settleCount;

        SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, MSS_RCM_PARTITION0);
        *muxReg = muxVal;
        SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, MSS_RCM_PARTITION0);

        /*
         * Confirmed on hardware (data abort, DFAR = CONFIG_TIMER0_BASE_ADDR,
         * DFSR external-abort/slave-error - i.e. a genuine bus rejection,
         * not a software fault): switching CONFIG_TIMER0/RTI1's clock
         * source and then immediately touching its registers via
         * TimerP_setup() below leaves no settling time - the mux switch
         * isn't instantaneous, and RTI1's register interface can stay
         * unresponsive for a brief window after its input clock changes.
         * This is a marginal timing issue (worked the first time, failed
         * intermittently afterward) - the readback forces the mux write to
         * complete on the bus before proceeding, and the delay loop lets
         * the clock domain settle. Plain busy-wait, not ClockP_usleep():
         * this function also runs from TsnDemoPacing_hwPushCb(), which is
         * CPTS-interrupt context (Cpsw_cptsIsr(), src/per/V2/cpsw.c), not
         * task context.
         */
        (void)*muxReg;
        for (settleCount = 0U; settleCount < 2000U; settleCount++)
        {
        }
    }
#else
    (void)muxVal;
#endif

    TimerP_Params_init(&timerParams);
    timerParams.inputPreScaler    = CONFIG_TIMER0_INPUT_PRE_SCALER;
    timerParams.inputClkHz        = inputClkHz;
    timerParams.periodInNsec      = gPacing.cycleTimeNs;
    timerParams.oneshotMode       = 0U;
    timerParams.enableOverflowInt = 1U;
    timerParams.enableDmaTrigger  = 0U;
    TimerP_setup(gTimerBaseAddr[CONFIG_TIMER0], &timerParams);
}

#if (TSNDEMO_USE_GENF_PACING == 1)

/*! CPTS hardware-push callback: the first GENF_START edge starts the timer,
 *  phase-locking it to the EST base time. One-shot deliberately - see the
 *  file header and TSNDEMO_TIMER_CLK_PERIOD_NS's comment: periodic
 *  re-syncing (acting on every edge, not just the first) was tried and
 *  measurably made the RTT average worse on hardware, reverted. */
static void TsnDemoPacing_hwPushCb(void *hwPushNotifyCbArg,
                                   CpswCpts_HwPush hwPushNum)
{
    (void)hwPushNotifyCbArg;
    (void)hwPushNum;
    if (!gPacing.timerStarted)
    {
        gPacing.timerStarted = true;
        TsnDemoPacing_timerReconfig(TSNDEMO_TIMER_INPUT_CLK_HZ,
                                    TSNDEMO_TIMER_CLKSRC_GENF0_VAL);
        gPacing.clkMuxOnGenf = true;
        TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);
    }
}

static int32_t TsnDemoPacing_startGenf(uint64_t baseTimeNs,
                                       uint32_t cycleTimeNs,
                                       uint32_t windowOffsetNs,
                                       uint32_t leadTimeNs)
{
    Enet_IoctlPrms prms;
    int32_t status;

    /* 1. GENF_CLK: free-running clock for the timer */
    status = TsnDemoPacing_setGenf(TSNDEMO_GENF_CLK_IDX,
                                   TSNDEMO_NS_TO_GENF_LEN(
                                       TSNDEMO_TIMER_CLK_PERIOD_NS),
                                   0ULL, true);
    if (status == ENET_SOK)
    {
        /* 2. HW-push callback, registered before GENF_START is armed.
         *    Delivered via the soc_timesync_xbar1 routing configured in
         *    example.syscfg (CPTS_GENF1 -> CPSW_MODULE_0) - without that
         *    routing this registration succeeds but the callback never
         *    fires. */
        CpswCpts_RegisterHwPushCbInArgs cptsInArgs;

        cptsInArgs.hwPushNum         = TSNDEMO_PACE_HWPUSH;
        cptsInArgs.hwPushNotifyCb    = TsnDemoPacing_hwPushCb;
        cptsInArgs.hwPushNotifyCbArg = NULL;
        ENET_IOCTL_SET_IN_ARGS(&prms, &cptsInArgs);
        ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                   CPSW_CPTS_IOCTL_REGISTER_HWPUSH_CALLBACK, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("pacing: HW-push register failed: %d\r\n",
                               status);
        }
        else
        {
            /*
             * 3. GENF_START: hw-push edge target, phase = (baseTime + window -
             *    lead) mod cycleTime - the PLAIN leadTimeNs-only formula (matching
             *    examples/ether_ring's own GENF1 compare exactly: enetapp_cpsw.c
             *    "setGenFInArgs.compare = gbaseTime - 20000", no cycle-time term),
             *    but not applied directly against the possibly-stale baseTimeNs
             *    from the caller.
             *
             *    Confirmed bug: baseTimeNs comes from a LIVE oper-list readback
             *    current only as of whenever EST was last (re-)enabled ('e');
             *    startGenf() runs again on every subsequent '3' press, so
             *    baseTimeNs+window-lead can easily already be BEHIND the CPTS
             *    free-running clock by execution time. Arming GENF1 with an
             *    already-past compare value is unverified/undefined behavior on
             *    this hardware - a plausible contributor to the run-to-run
             *    "sometimes locks, sometimes doesn't" pattern seen throughout
             *    this investigation.
             *
             *    Fix: re-anchor to a fresh point >= (now + 2s), snapped to the
             *    SAME phase (mod cycleTimeNs) the original formula would have
             *    used - a fresh now-relative target, not baseTimeNs +
             *    k*cycleTimeNs, matching the "now + margin, snapped to phase"
             *    approach TsnDemoFeature_deriveBaseTime() already uses for
             *    baseTimeNs itself. This guarantees ~2s of real margin on every
             *    '3' press, first or tenth. Does NOT touch or re-arm ESTF/the
             *    admin list, which stays live since 'e' was pressed; only GENF1
             *    is freshly re-armed here.
             */
            uint64_t nowNs   = TsnDemoFlow_getTimeNs();
            uint64_t phaseNs = (baseTimeNs + (uint64_t)windowOffsetNs -
                                (uint64_t)leadTimeNs) % (uint64_t)cycleTimeNs;
            uint64_t anchorNs = nowNs + 2000000000ULL; /* now + 2s */
            uint64_t remNs    = anchorNs % (uint64_t)cycleTimeNs;
            uint64_t targetNs = (remNs <= phaseNs) ?
                                (anchorNs - remNs + phaseNs) :
                                (anchorNs - remNs + phaseNs + (uint64_t)cycleTimeNs);

            EnetAppUtils_print(
                "pacing: now=%llu.%09llu ESTF base=%llu.%09llu\r\n",
                nowNs / 1000000000ULL, nowNs % 1000000000ULL,
                baseTimeNs / 1000000000ULL, baseTimeNs % 1000000000ULL);

            status = TsnDemoPacing_setGenf(TSNDEMO_GENF_START_IDX,
                                           TSNDEMO_NS_TO_GENF_LEN(
                                               2ULL * cycleTimeNs),
                                           targetNs, false);
            if (status == ENET_SOK)
            {
                gPacing.genfLocked = true;
                EnetAppUtils_print(
                    "pacing: GENF armed, hw-push edge at %llu.%09llu "
                    "(+%llu ms from now)\r\n",
                    targetNs / 1000000000ULL, targetNs % 1000000000ULL,
                    (targetNs - nowNs) / 1000000ULL);
            }
        }
    }
    return status;
}
#endif /* TSNDEMO_USE_GENF_PACING */

/*! \brief RTI timer ISR (registered as CONFIG_TIMER0's timerCallback in
 *         example.syscfg, called from generated TimerP_isr0() which also
 *         handles TimerP_clearOverflowInt() - see ti_dpl_config.c). Called
 *         once every CONFIG_TIMER0 period, on whichever clock source is
 *         currently muxed in (plain SoC clock, or GENF0 once gate-locked). */
void TsnDemoPacing_timerCb(void *args)
{
    (void)args;
    if ((gPacing.running) && (gPacing.tickCb != NULL))
    {
        gPacing.tickCb();
    }
}

int32_t TsnDemoPacing_start(uint64_t baseTimeNs, uint32_t cycleTimeNs,
                            uint32_t windowOffsetNs, uint32_t leadTimeNs,
                            TsnDemo_PacingTickCb tickCb)
{
    int32_t status = ENET_SOK;
    bool genfOk = false;

    gPacing.tickCb       = tickCb;
    gPacing.cycleTimeNs  = cycleTimeNs;
    gPacing.running      = true;
    gPacing.timerStarted = false;
    gPacing.genfLocked   = false;

#if (TSNDEMO_USE_GENF_PACING == 1)
    if (baseTimeNs != 0ULL)
    {
        status = TsnDemoPacing_startGenf(baseTimeNs, cycleTimeNs,
                                         windowOffsetNs, leadTimeNs);
        if (status == ENET_SOK)
        {
            genfOk = true;
        }
        else
        {
            EnetAppUtils_print(
                "pacing: GENF path failed, falling back to plain timer\r\n");
        }
    }
#else
    (void)baseTimeNs;
    (void)windowOffsetNs;
    (void)leadTimeNs;
#endif

    if (!genfOk)
    {
        /* Plain-timer path: today's default (EST-off) behavior, and also what
         * a failed GENF attempt above falls back to - CONFIG_TIMER0 starts
         * as-is, on whatever clock source it was last configured for
         * (example.syscfg's boot default the first time through;
         * TsnDemoPacing_stop() restores that default after a GENF-locked
         * run, see below). `status` is deliberately left as the GENF
         * failure code here (if any), not reset to ENET_SOK, so the caller
         * still learns the GENF path failed even though the fallback timer
         * is now running. */
        gPacing.timerStarted = true;
        TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);
    }
    return status;
}

void TsnDemoPacing_stop(void)
{
    gPacing.running = false;
    if (gPacing.timerStarted)
    {
        TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);
        gPacing.timerStarted = false;
    }
#if (TSNDEMO_USE_GENF_PACING == 1)
    /*
     * Unconditional, not gated on gPacing.genfLocked - genfLocked only becomes
     * true if the *entire* startGenf() sequence succeeded, but
     * CPSW_CPTS_IOCTL_REGISTER_HWPUSH_CALLBACK (and GENF0 itself) can already
     * have been applied even when a later step failed (e.g. GENF1 config
     * failing after hw-push registration already succeeded). Leaving the
     * hw-push callback registered from a failed or prior attempt makes every
     * subsequent REGISTER_HWPUSH_CALLBACK call fail with ENET_EALREADYOPEN
     * (cpsw_cpts_ioctl.c) - confirmed bug: this silently capped GENF locking to
     * at most once per boot. Both the unregister and disable-GENF calls below
     * are safe/idempotent even when nothing was actually registered/active
     * (driver returns SOK with a benign trace).
     */
    {
        CpswCpts_HwPush hwPushNum = TSNDEMO_PACE_HWPUSH;
        Enet_IoctlPrms prms;
        int32_t unregStatus;

        ENET_IOCTL_SET_IN_ARGS(&prms, &hwPushNum);
        ENET_IOCTL(gTsnDemo.hEnet, gTsnDemo.coreId,
                   CPSW_CPTS_IOCTL_UNREGISTER_HWPUSH_CALLBACK, &prms,
                   unregStatus);
        (void)unregStatus;
    }

    /*
     * Unlike the unregister call above and the disable-GENF calls below, this
     * revert step is NOT unconditional - gated on clkMuxOnGenf (see its
     * declaration), true only if hwPushCb() actually switched CONFIG_TIMER0/
     * RTI1's clock source to GENF0. An earlier, unconditional version of this
     * call ran even on runs that never touched GENF (e.g. the first BASELINE/
     * CONTENTION-TSN-OFF run of a fresh boot, since TsnDemoPacing_stop() runs
     * at the top of every TsnDemoTalker_runMode() call) - a TimerP_setup()
     * call on RTI1 that never happened in the pre-GENF code, and crashed on
     * hardware (confirmed: the abort moved from "only after a prior GENF
     * lock" to "on the very first run" once unconditional).
     *
     * Ordered BEFORE disabling GENF0/GENF1 below - confirmed bug (crash
     * reproduced on the *second* '3', the first time this function has a
     * live GENF0 to disable): the old order disabled GENF0 (length 0,
     * silencing its output) while CONFIG_TIMER0/RTI1 was still muxed to it,
     * then only switched RTI1's clock source away afterward - leaving RTI1
     * clocked by an already-silenced source for that window. Switching the
     * mux away first, while GENF0 is still toggling, avoids ever clocking
     * RTI1 from a dead source.
     */
    if (gPacing.clkMuxOnGenf)
    {
        /* GENF_CLK is about to go silent below, so leaving CONFIG_TIMER0
         * muxed to it would hang the next (possibly non-EST) run's
         * plain-timer start - revert to example.syscfg's default clock
         * source now, before GENF0 is disabled. */
        gPacing.cycleTimeNs = CONFIG_TIMER0_NSEC_PER_TICK;
        TsnDemoPacing_timerReconfig(CONFIG_TIMER0_INPUT_CLK_HZ,
                                    TSNDEMO_TIMER_CLKSRC_DEFAULT_VAL);
        gPacing.clkMuxOnGenf = false;
    }

    /* Disable both GENFs (length 0) - safe now that RTI1 (if it was ever
     * muxed to GENF0) has already been switched away above. */
    (void)TsnDemoPacing_setGenf(TSNDEMO_GENF_CLK_IDX, 0U, 0ULL, false);
    (void)TsnDemoPacing_setGenf(TSNDEMO_GENF_START_IDX, 0U, 0ULL, false);
    gPacing.genfLocked = false;
#endif
}

bool TsnDemoPacing_isLocked(void)
{
    return gPacing.genfLocked && gPacing.timerStarted;
}
