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
 * \file  tsndemo_talker.h
 *
 * \brief TSN demo talker - internal types: latency statistics (fixed-bin
 *        histograms) and the EST-gate-locked generator pacing API.
 */

#ifndef TSNDEMO_TALKER_H_
#define TSNDEMO_TALKER_H_

#include "tsndemo.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                       Statistics (tsndemo_stats.c)                         */
/* ========================================================================== */

/*! 512 bins x 100 ns = 0..51.2 us range; overflow counted, not clipped */
#define TSNDEMO_HIST_BINS               (512U)
#define TSNDEMO_HIST_BIN_NS             (100U)

typedef struct TsnDemo_LatencyHist_s
{
    uint64_t count;
    uint64_t sumNs;
    uint64_t sumSqNs;
    uint32_t minNs;
    uint32_t maxNs;
    uint32_t overflowCount;      /*!< samples >= BINS*BIN_NS                */
    uint32_t negativeCount;      /*!< t_late < t_early: clock not stable    */
    uint32_t bins[TSNDEMO_HIST_BINS];
} TsnDemo_LatencyHist;

typedef struct TsnDemo_RunStats_s
{
    TsnDemo_LatencyHist fwd;     /*!< T2 - T1 (talker -> listener)          */
    TsnDemo_LatencyHist rev;     /*!< T4 - T3 (listener -> talker)          */
    TsnDemo_LatencyHist rtt;     /*!< T4 - T1                               */
    TsnDemo_LatencyHist turn;    /*!< T3 - T2 (listener turnaround)         */

    uint32_t txCount;
    uint32_t rxCount;
    uint32_t lostCount;
    uint32_t outOfOrderCount;
    uint32_t staleCount;         /*!< wrong runId                           */
    uint32_t tsFailCount;        /*!< probes sent with t1 == 0              */

    uint64_t windowStartNs;
    uint64_t windowEndNs;
    uint64_t bulkTxBytes;        /*!< bulk payload+L2 bytes in the window   */

    TsnDemo_RunMode    mode;
    TsnDemo_FeatureSet features;
    bool               valid;
} TsnDemo_RunStats;

void     TsnDemoStats_reset(TsnDemo_RunStats *stats, TsnDemo_RunMode mode,
                            const TsnDemo_FeatureSet *features);
void     TsnDemoStats_addSample(TsnDemo_LatencyHist *hist, uint64_t tEarlyNs,
                                uint64_t tLateNs);
/*! \brief pctX100: 5000 = p50, 9900 = p99. Returns ns. */
uint32_t TsnDemoStats_percentile(const TsnDemo_LatencyHist *hist,
                                 uint32_t pctX100);
void     TsnDemoStats_print(const TsnDemo_RunStats *stats);
/*! \brief Side-by-side table: tsnOff = TSN off, tsnOn = TSN on. */
void     TsnDemoStats_printComparison(const TsnDemo_RunStats *tsnOff,
                                      const TsnDemo_RunStats *tsnOn);

/* ========================================================================== */
/*                        Pacing (tsndemo_pacing.c)                           */
/* ========================================================================== */

/*!
 * \brief Called from the pacing tick (CONFIG_TIMER0 ISR) once per period.
 */
typedef void (*TsnDemo_PacingTickCb)(void);

/*!
 * \brief Start the express pacing source (CONFIG_TIMER0 / RTI1) and
 *        register tickCb to be called from its ISR once per period.
 *
 *        If baseTimeNs is non-zero (EST on), phase- and frequency-locks the
 *        timer to the EST gate schedule via CPTS GENF0/GENF1 - see the
 *        design note at the top of tsndemo_pacing.c. Falls back to a plain,
 *        non-gate-locked timer if baseTimeNs is 0 (EST off) or if GENF
 *        configuration fails. Use TsnDemoPacing_isLocked() to tell which
 *        path actually took effect.
 *
 * \param baseTimeNs      EST schedule base time (gTsnDemo.estBaseTimeNs),
 *                         or 0 to skip gate-locking entirely
 * \param cycleTimeNs     EST cycle time (TSNDEMO_EST_CYCLE_NS)
 * \param windowOffsetNs  offset of the express gate window from the start
 *                         of the cycle (0 - express opens the cycle)
 * \param leadTimeNs      guard time before the window edge to arm the
 *                         timer start, covering clock-mux-switch +
 *                         TimerP_setup latency
 * \param tickCb          called once per timer period
 */
int32_t TsnDemoPacing_start(uint64_t baseTimeNs, uint32_t cycleTimeNs,
                            uint32_t windowOffsetNs, uint32_t leadTimeNs,
                            TsnDemo_PacingTickCb tickCb);

void    TsnDemoPacing_stop(void);

/*! \brief True once pacing has actually started ticking phase-locked to
 *         the EST gate schedule (GENF path succeeded and fired). False if
 *         EST is off, or if pacing fell back to the plain timer. */
bool    TsnDemoPacing_isLocked(void);

#ifdef __cplusplus
}
#endif

#endif /* TSNDEMO_TALKER_H_ */
